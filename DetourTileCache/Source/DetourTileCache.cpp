#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <string.h>
#include <new>

dtTileCache* dtAllocTileCache()
{
	void* mem = dtAlloc(sizeof(dtTileCache), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtTileCache;
}

void dtFreeTileCache(dtTileCache* tc)
{
	if (!tc) return;
	tc->~dtTileCache();
	dtFree(tc);
}

static bool contains(const dtCompressedTileRef* a, const int n, const dtCompressedTileRef v)
{
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return true;
	return false;
}

// ������Ƭ������ϣ���Ͱid
inline int computeTileHash(int x, int y, const int mask)
{
	// h1 �� h2 ������ħ������ֹ��ϣ��ͻ�õ�
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}


struct NavMeshTileBuildContext
{
	inline NavMeshTileBuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
	inline ~NavMeshTileBuildContext() { purge(); }
	void purge()
	{
		dtFreeTileCacheLayer(alloc, layer);
		layer = 0;
		dtFreeTileCacheContourSet(alloc, lcset);
		lcset = 0;
		dtFreeTileCachePolyMesh(alloc, lmesh);
		lmesh = 0;
	}
	struct dtTileCacheLayer* layer;
	struct dtTileCacheContourSet* lcset;
	struct dtTileCachePolyMesh* lmesh;
	struct dtTileCacheAlloc* alloc;
};


dtTileCache::dtTileCache() :
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFreeTile(0),	
	m_tiles(0),	
	m_saltBits(0),
	m_tileBits(0),
	m_talloc(0),
	m_tcomp(0),
	m_tmproc(0),
	m_obstacles(0),
	m_nextFreeObstacle(0),
	m_nreqs(0),
	m_nupdate(0)
{
	memset(&m_params, 0, sizeof(m_params));
	memset(m_reqs, 0, sizeof(ObstacleRequest) * MAX_REQUESTS);
}
	
dtTileCache::~dtTileCache()
{
	for (int i = 0; i < m_params.maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_COMPRESSEDTILE_FREE_DATA)
		{
			dtFree(m_tiles[i].data);
			m_tiles[i].data = 0;
		}
	}
	dtFree(m_obstacles);
	m_obstacles = 0;
	dtFree(m_posLookup);
	m_posLookup = 0;
	dtFree(m_tiles);
	m_tiles = 0;
	m_nreqs = 0;
	m_nupdate = 0;
}

const dtCompressedTile* dtTileCache::getTileByRef(dtCompressedTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodeTileIdTile(ref);
	unsigned int tileSalt = decodeTileIdSalt(ref);
	if ((int)tileIndex >= m_params.maxTiles)
		return 0;
	const dtCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}


dtStatus dtTileCache::init(const dtTileCacheParams* params,
						   dtTileCacheAlloc* talloc,
						   dtTileCacheCompressor* tcomp,
						   dtTileCacheMeshProcess* tmproc)
{
	m_talloc = talloc;
	m_tcomp = tcomp;
	m_tmproc = tmproc;
	m_nreqs = 0;
	memcpy(&m_params, params, sizeof(m_params));
	
	// Alloc space for obstacles.
	// �����ϰ�����ڴ�ռ䣬���ڴ洢�ϰ��������
	m_obstacles = (dtTileCacheObstacle*)dtAlloc(sizeof(dtTileCacheObstacle)*m_params.maxObstacles, DT_ALLOC_PERM);
	if (!m_obstacles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_obstacles, 0, sizeof(dtTileCacheObstacle)*m_params.maxObstacles);
	m_nextFreeObstacle = 0;
	// �����ʼ���ϰ��������ÿһ���ϰ������һ�����ö��������
	for (int i = m_params.maxObstacles-1; i >= 0; --i)
	{
		m_obstacles[i].salt = 1;
		m_obstacles[i].next = m_nextFreeObstacle;
		m_nextFreeObstacle = &m_obstacles[i];
	}
	
	// Init tiles
	// ��ʼ����Ƭ��ϣ��ѯ��Ĵ�С��������2���ݴη�
	m_tileLutSize = dtNextPow2(m_params.maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	// ��Ƭ��ʶ����ϣ��ѯ���С
	m_tileLutMask = m_tileLutSize-1;
	// ��ʼ�����ڴ洢��Ƭ���������
	m_tiles = (dtCompressedTile*)dtAlloc(sizeof(dtCompressedTile)*m_params.maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	// ��Ƭ��ϣ��ѯ���ʼ��
	m_posLookup = (dtCompressedTile**)dtAlloc(sizeof(dtCompressedTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(dtCompressedTile)*m_params.maxTiles);
	memset(m_posLookup, 0, sizeof(dtCompressedTile*)*m_tileLutSize);
	m_nextFreeTile = 0;
	// �����ʼ����Ƭ��������ÿһ����Ƭ����һ�����ö��������
	for (int i = m_params.maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFreeTile;
		m_nextFreeTile = &m_tiles[i];
	}
	
	// Init ID generator values.
	// ͨ����Ƭ�������������̬������Ƭռ�õı���λ��
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	// ֻ����31�� salt ����λ����Ϊ salt ������ʹ��32λ��unit����ģ��������ֽ��������
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);
	// ȷ�� salt ����ʹ��10������λ
	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const 
{
	// ʵ�ʷ��ص���Ƭ������������ֵС�ڵ��� maxTiles
	int n = 0;
	
	// Find tile based on hash.
	// ���ڹ�ϣ�������Ƭ����
	int h = computeTileHash(tx,ty,m_tileLutMask);
	// ȡ����ѹ������Ƭ����
	dtCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		// �ҵ���������ͬһ�С��е���Ƭ����
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty)
		{
			// ���û�дﵽ�����Ƭ����������������ޣ��Ͱ��ҵ�����Ƭ�洢����
			if (n < maxTiles)
				tiles[n++] = getTileRef(tile);
		}
		tile = tile->next;
	}
	
	return n;
}

// ��������Ĺ����� getTilesAt ���ƣ�Ψһ����������������һ��������ָ���㼶����
dtCompressedTile* dtTileCache::getTileAt(const int tx, const int ty, const int tlayer)
{
	// Find tile based on hash.
	int h = computeTileHash(tx,ty,m_tileLutMask);
	dtCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty &&
			tile->header->tlayer == tlayer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

// ͨ��salt��dtCompressedTile�������е��±꣬���һ������id
dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile* tile) const
{
	if (!tile) return 0;
	// ͨ����ǰָ��λ�ã���ȥ����ĵ�һ��Ԫ�ص�ָ��λ�ã������ tile �������е��±꣨��������
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtCompressedTileRef)encodeTileId(tile->salt, it);
}

// ͨ��salt��dtTileCacheObstacle�������е��±꣬���һ������id
dtObstacleRef dtTileCache::getObstacleRef(const dtTileCacheObstacle* ob) const
{
	if (!ob) return 0;
	const unsigned int idx = (unsigned int)(ob - m_obstacles);
	return encodeObstacleId(ob->salt, idx);
}

const dtTileCacheObstacle* dtTileCache::getObstacleByRef(dtObstacleRef ref)
{
	if (!ref)
		return 0;
	// ͨ������id������� dtTileCacheObstacle �����������е��±꣨��������λ��
	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_params.maxObstacles)
		return 0;
	// ��������ȡ�� dtTileCacheObstacle ���������
	const dtTileCacheObstacle* ob = &m_obstacles[idx];
	// ͨ������id������� dtTileCacheObstacle ����� salt ֵ
	unsigned int salt = decodeObstacleIdSalt(ref);
	// ���μ�飬ȷ�������saltֵ�븴��id��saltֵƥ��
	if (ob->salt != salt)
		return 0;
	return ob;
}

dtTileCacheMeshProcess::~dtTileCacheMeshProcess()
{
	// Defined out of line to fix the weak v-tables warning
}

dtStatus dtTileCache::addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result)
{
	// Make sure the data is in right format.
	// ȷ�����ݸ�ʽ����ȷ��
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
	if (header->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	// Make sure the location is free.
	// ȷ����tx��ty��layer��ʾ��λ�ô��ڿ���״̬
	if (getTileAt(header->tx, header->ty, header->tlayer))
		return DT_FAILURE;
	
	// Allocate a tile.
	// ����һ����Ƭ����
	dtCompressedTile* tile = 0;
	if (m_nextFreeTile)
	{
		tile = m_nextFreeTile;
		m_nextFreeTile = tile->next;
		tile->next = 0;
	}
	
	// Make sure we could allocate a tile.
	// ���пռ��ѱ�ռ�ã������б�Ϊ��
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	// ������Ƭ�����ڹ�ϣ���λ��
	int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
	// ����ϣ����ԭ�еĶ�������Ϊ��ǰ��Ƭ����һ�����ö���
	tile->next = m_posLookup[h];
	// ���ǵ�ԭ������ָ�룬ָ���´�������Ƭ����
	m_posLookup[h] = tile;
	
	// Init tile.
	// ��ʼ����Ƭ�ĳ�Ա����
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	tile->header = (dtTileCacheLayerHeader*)data;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->compressed = tile->data + headerSize;
	tile->compressedSize = tile->dataSize - headerSize;
	tile->flags = flags;
	
	// ������Ƭ�ĸ���id
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	// �������Ƭ�������е��±꣨��������
	unsigned int tileIndex = decodeTileIdTile(ref);
	// �������Ƭ��Ӧ�� salt ֵ
	unsigned int tileSalt = decodeTileIdSalt(ref);
	// ����±��Ƿ񳬳�����������ɵ���Ƭ��������
	if ((int)tileIndex >= m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	// ���μ�飬ȷ���������Ƭ����id�д洢�� salt ֵ���������洢�� salt ֵһ��
	dtCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	// ������Ƭ�Ĺ�ϣֵ
	const int h = computeTileHash(tile->header->tx,tile->header->ty,m_tileLutMask);
	dtCompressedTile* prev = 0;
	dtCompressedTile* cur = m_posLookup[h];
	// while ѭ�������ű�ɾ������Ƭ�ĵ�����һֱ������ȥ��ֱ��û����һ�����õ�Ϊֹ
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				// ����һ������ڴ棬�Żع�ϣ��
				m_posLookup[h] = cur->next;
			break;
		}
		// ����ǰһ����ƬΪ��ǰ��Ƭ
		prev = cur;
		// ���õ�ǰ��ƬΪ��һ��������Ƭ
		cur = cur->next;
	}
	
	// Reset tile.
	// �����Ƭ�Ĵ�����ʶ����������� dtTileCache ������ڴ棬�� dtTileCache �����ͷ�
	if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	// ���򣬰��ڴ���뵽�ڴ�أ��Ա���һ��ʹ��
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}
	
	// ���ñ�ɾ������Ƭ����ĳ�Ա����
	tile->header = 0;
	tile->data = 0;
	tile->dataSize = 0;
	tile->compressed = 0;
	tile->compressedSize = 0;
	tile->flags = 0;
	
	// Update salt, salt should never be zero.
	// ������Ƭ�����ݱ��������salt ������һ�����ڵ���1������
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
	if (tile->salt == 0)
		tile->salt++;
	
	// Add to free list.
	tile->next = m_nextFreeTile;
	m_nextFreeTile = tile;
	
	return DT_SUCCESS;
}

// ���һ��Բ�������͵��ϰ��ʹ��Բ�����Χ�м��
dtStatus dtTileCache::addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result)
{
	// ��鵱ǰ���������ϰ���������ͷ��Ѵﵽ����
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	// ���������ӵ��ϰ����Ƿ��Ѵﵽ����
	dtTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	unsigned short salt = ob->salt;
	// �����ϰ���ĳ�ʼ����
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_CYLINDER;
	dtVcopy(ob->cylinder.pos, pos);
	ob->cylinder.radius = radius;
	ob->cylinder.height = height;
	
	// ���洴���ϰ��������
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	// ����������ϰ���ĸ���id
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

// �� addObstacle ���ƣ���������ʹ�õ��� AABB��������Χ�У�
dtStatus dtTileCache::addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	dtTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_BOX;
	dtVcopy(ob->box.bmin, bmin);
	dtVcopy(ob->box.bmax, bmax);

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

// �� addObstacle ���ƣ���������ʹ�õ��� OBB�������Χ�У�
dtStatus dtTileCache::addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_ORIENTED_BOX;
	dtVcopy(ob->orientedBox.center, center);
	dtVcopy(ob->orientedBox.halfExtents, halfExtents);

	float coshalf= cosf(0.5f*yRadians);
	float sinhalf = sinf(-0.5f*yRadians);
	ob->orientedBox.rotAux[0] = coshalf*sinhalf;
	ob->orientedBox.rotAux[1] = coshalf*coshalf - 0.5f;

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);

	if (result)
		*result = req->ref;

	return DT_SUCCESS;
}

// ͨ������id��ɾ���ϰ���
dtStatus dtTileCache::removeObstacle(const dtObstacleRef ref)
{
	if (!ref)
		return DT_SUCCESS;
	// ����Ƿ����㹻�Ŀռ仺�����Ƭ����
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	// ������Ƭɾ���Ĳ���ָ��
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_REMOVE;
	req->ref = ref;
	
	return DT_SUCCESS;
}

// bmin �� bmax ָ��һ����ѯ��Χ
// results ��¼��ѯ������Ƭ�ĸ���id����һ����������
// resultCount ��ѯ���ļ�¼����
// maxResults results ���Դ洢����������
dtStatus dtTileCache::queryTiles(const float* bmin, const float* bmax,
								 dtCompressedTileRef* results, int* resultCount, const int maxResults) const 
{
	const int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES];
	
	int n = 0;
	
	// ���㵼�������� xz ���ϵ��ܳߴ�
	const float tw = m_params.width * m_params.cs;
	const float th = m_params.height * m_params.cs;
	// �������ʼ����Ƭλ�ã��������е���ʽ
	const int tx0 = (int)dtMathFloorf((bmin[0]-m_params.orig[0]) / tw);
	const int tx1 = (int)dtMathFloorf((bmax[0]-m_params.orig[0]) / tw);
	const int ty0 = (int)dtMathFloorf((bmin[2]-m_params.orig[2]) / th);
	const int ty1 = (int)dtMathFloorf((bmax[2]-m_params.orig[2]) / th);
	
	for (int ty = ty0; ty <= ty1; ++ty)
	{
		for (int tx = tx0; tx <= tx1; ++tx)
		{
			const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);
			
			for (int i = 0; i < ntiles; ++i)
			{
				const dtCompressedTile* tile = &m_tiles[decodeTileIdTile(tiles[i])];
				float tbmin[3], tbmax[3];
				// ������Ƭ�� AABB �߽�
				calcTightTileBounds(tile->header, tbmin, tbmax);
				
				// �����Ƭ�ı߽��Ƿ����ж������ص�
				if (dtOverlapBounds(bmin,bmax, tbmin,tbmax))
				{
					// ��¼�ص�����Ƭ����
					if (n < maxResults)
						results[n++] = tiles[i];
				}
			}
		}
	}
	
	*resultCount = n;
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::update(const float /*dt*/, dtNavMesh* navmesh,
							 bool* upToDate)
{
	if (m_nupdate == 0)
	{
		// Process requests.
		// �����������ÿ�δ��������������˳�ǰ������ m_nreqs = 0
		for (int i = 0; i < m_nreqs; ++i)
		{
			ObstacleRequest* req = &m_reqs[i];
			// �����ϰ����������е��±�
			unsigned int idx = decodeObstacleIdObstacle(req->ref);
			if ((int)idx >= m_params.maxObstacles)
				continue;
			// ȡ���ϰ��������֤�ϰ����Ƿ���Ч
			dtTileCacheObstacle* ob = &m_obstacles[idx];
			unsigned int salt = decodeObstacleIdSalt(req->ref);
			if (ob->salt != salt)
				continue;
			
			// ��������ϰ��������
			if (req->action == REQUEST_ADD)
			{
				// Find touched tiles.
				float bmin[3], bmax[3];
				getObstacleBounds(ob, bmin, bmax);

				int ntouched = 0;
				queryTiles(bmin, bmax, ob->touched, &ntouched, DT_MAX_TOUCHED_TILES);
				ob->ntouched = (unsigned char)ntouched;
				// Add tiles to update list.
				ob->npending = 0;
				for (int j = 0; j < ob->ntouched; ++j)
				{
					if (m_nupdate < MAX_UPDATE)
					{
						// �����Ƭ�Ƿ��Ѿ��ڸ����б��У�����������뵽�б�
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						// �ϰ�����������Ƭ���뵽������� pending �б���
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
			// ����ɾ���ϰ��������
			else if (req->action == REQUEST_REMOVE)
			{
				// Prepare to remove obstacle.
				ob->state = DT_OBSTACLE_REMOVING;
				// Add tiles to update list.
				ob->npending = 0;
				for (int j = 0; j < ob->ntouched; ++j)
				{
					if (m_nupdate < MAX_UPDATE)
					{
						// �����Ƭ�Ƿ��Ѿ��ڸ����б��У�����������뵽�б�
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						// �ϰ�����������Ƭ���뵽������� pending �б���
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
		}
		
		m_nreqs = 0;
	}
	
	dtStatus status = DT_SUCCESS;
	// Process updates
	// ������һ����Ƭ���¼��뵽 m_update �б��
	if (m_nupdate)
	{
		// Build mesh
		const dtCompressedTileRef ref = m_update[0];
		status = buildNavMeshTile(ref, navmesh);
		m_nupdate--;
		if (m_nupdate > 0)
			memmove(m_update, m_update+1, m_nupdate*sizeof(dtCompressedTileRef));

		// Update obstacle states.
		for (int i = 0; i < m_params.maxObstacles; ++i)
		{
			dtTileCacheObstacle* ob = &m_obstacles[i];
			if (ob->state == DT_OBSTACLE_PROCESSING || ob->state == DT_OBSTACLE_REMOVING)
			{
				// Remove handled tile from pending list.
				for (int j = 0; j < (int)ob->npending; j++)
				{
					if (ob->pending[j] == ref)
					{
						ob->pending[j] = ob->pending[(int)ob->npending-1];
						ob->npending--;
						break;
					}
				}
				
				// If all pending tiles processed, change state.
				if (ob->npending == 0)
				{
					if (ob->state == DT_OBSTACLE_PROCESSING)
					{
						ob->state = DT_OBSTACLE_PROCESSED;
					}
					else if (ob->state == DT_OBSTACLE_REMOVING)
					{
						ob->state = DT_OBSTACLE_EMPTY;
						// Update salt, salt should never be zero.
						ob->salt = (ob->salt+1) & ((1<<16)-1);
						if (ob->salt == 0)
							ob->salt++;
						// Return obstacle to free list.
						ob->next = m_nextFreeObstacle;
						m_nextFreeObstacle = ob;
					}
				}
			}
		}
	}
	
	if (upToDate)
		*upToDate = m_nupdate == 0 && m_nreqs == 0;

	return status;
}


dtStatus dtTileCache::buildNavMeshTilesAt(const int tx, const int ty, dtNavMesh* navmesh)
{
	const int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES];
	const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);
	
	for (int i = 0; i < ntiles; ++i)
	{
		dtStatus status = buildNavMeshTile(tiles[i], navmesh);
		if (dtStatusFailed(status))
			return status;
	}
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::buildNavMeshTile(const dtCompressedTileRef ref, dtNavMesh* navmesh)
{	
	dtAssert(m_talloc);
	dtAssert(m_tcomp);
	
	unsigned int idx = decodeTileIdTile(ref);
	if (idx > (unsigned int)m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	const dtCompressedTile* tile = &m_tiles[idx];
	unsigned int salt = decodeTileIdSalt(ref);
	if (tile->salt != salt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_talloc->reset();
	
	NavMeshTileBuildContext bc(m_talloc);
	// ������ƶ������ڴ�ֱ����������������Ԫ������
	const int walkableClimbVx = (int)(m_params.walkableClimb / m_params.ch);
	dtStatus status;
	
	// Decompress tile layer data. 
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;
	
	// Rasterize obstacles.
	// �ϰ���դ�񻯣�����ά�ռ�����ת��Ϊ��������Ķ�ά��ʾ������ߵ�����·���滮�����ܺ�Ч�ʡ�
	for (int i = 0; i < m_params.maxObstacles; ++i)
	{
		const dtTileCacheObstacle* ob = &m_obstacles[i];
		// ��ʶ�ϰ��ﲻ���ڣ������ڵȴ�ɾ���Ľ׶�
		if (ob->state == DT_OBSTACLE_EMPTY || ob->state == DT_OBSTACLE_REMOVING)
			continue;
		// �ж���Ƭ�Ƿ��Ѿ����������
		if (contains(ob->touched, ob->ntouched, ref))
		{
			// 
			if (ob->type == DT_OBSTACLE_CYLINDER)
			{
				dtMarkCylinderArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
							    ob->cylinder.pos, ob->cylinder.radius, ob->cylinder.height, 0);
			}
			else if (ob->type == DT_OBSTACLE_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob->box.bmin, ob->box.bmax, 0);
			}
			else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob->orientedBox.center, ob->orientedBox.halfExtents, ob->orientedBox.rotAux, 0);
			}
		}
	}
	
	// Build navmesh
	status = dtBuildTileCacheRegions(m_talloc, *bc.layer, walkableClimbVx);
	if (dtStatusFailed(status))
		return status;
	
	bc.lcset = dtAllocTileCacheContourSet(m_talloc);
	if (!bc.lcset)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCacheContours(m_talloc, *bc.layer, walkableClimbVx,
									  m_params.maxSimplificationError, *bc.lcset);
	if (dtStatusFailed(status))
		return status;
	
	bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
	if (!bc.lmesh)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
	if (dtStatusFailed(status))
		return status;
	
	// Early out if the mesh tile is empty.
	if (!bc.lmesh->npolys)
	{
		// Remove existing tile.
		navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);
		return DT_SUCCESS;
	}
	
	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = bc.lmesh->verts;
	params.vertCount = bc.lmesh->nverts;
	params.polys = bc.lmesh->polys;
	params.polyAreas = bc.lmesh->areas;
	params.polyFlags = bc.lmesh->flags;
	params.polyCount = bc.lmesh->npolys;
	params.nvp = DT_VERTS_PER_POLYGON;
	params.walkableHeight = m_params.walkableHeight;
	params.walkableRadius = m_params.walkableRadius;
	params.walkableClimb = m_params.walkableClimb;
	params.tileX = tile->header->tx;
	params.tileY = tile->header->ty;
	params.tileLayer = tile->header->tlayer;
	params.cs = m_params.cs;
	params.ch = m_params.ch;
	params.buildBvTree = false;
	dtVcopy(params.bmin, tile->header->bmin);
	dtVcopy(params.bmax, tile->header->bmax);
	
	if (m_tmproc)
	{
		m_tmproc->process(&params, bc.lmesh->areas, bc.lmesh->flags);
	}
	
	unsigned char* navData = 0;
	int navDataSize = 0;
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		return DT_FAILURE;

	// Remove existing tile.
	navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);

	// Add new tile, or leave the location empty.
	if (navData)
	{
		// Let the navmesh own the data.
		status = navmesh->addTile(navData,navDataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			return status;
		}
	}
	
	return DT_SUCCESS;
}

void dtTileCache::calcTightTileBounds(const dtTileCacheLayerHeader* header, float* bmin, float* bmax) const
{
	const float cs = m_params.cs;
	bmin[0] = header->bmin[0] + header->minx*cs;
	bmin[1] = header->bmin[1];
	bmin[2] = header->bmin[2] + header->miny*cs;
	bmax[0] = header->bmin[0] + (header->maxx+1)*cs;
	bmax[1] = header->bmax[1];
	bmax[2] = header->bmin[2] + (header->maxy+1)*cs;
}

void dtTileCache::getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const
{
	// ɸѡ�߽��������
	// Բ����
	if (ob->type == DT_OBSTACLE_CYLINDER)
	{
		const dtObstacleCylinder &cl = ob->cylinder;

		bmin[0] = cl.pos[0] - cl.radius;
		bmin[1] = cl.pos[1];
		bmin[2] = cl.pos[2] - cl.radius;
		bmax[0] = cl.pos[0] + cl.radius;
		bmax[1] = cl.pos[1] + cl.height;
		bmax[2] = cl.pos[2] + cl.radius;
	}
	// AABB��������Χ�У�
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		dtVcopy(bmin, ob->box.bmin);
		dtVcopy(bmax, ob->box.bmax);
	}
	// OBB�������Χ�У�
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		const dtObstacleOrientedBox &orientedBox = ob->orientedBox;
		// ʹ����������뾶�����Եõ�������ˮƽ���ϵ����ߴ�Ĺ�ֵ��
		// ���ַ�ʽ������Ч�ļ���ײ��⡢�����������Ϳռ�ָ�Ȳ�����
		// ���ǣ����ַ������ܻᵼ��һЩ�󱨣����ڴ��������£����ּ��㷽ʽ�Ѿ��㹻���ˣ��Ҽ���ɱ��ϵ͡�
		float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
		bmin[0] = orientedBox.center[0] - maxr;
		bmax[0] = orientedBox.center[0] + maxr;
		bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
		bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
		bmin[2] = orientedBox.center[2] - maxr;
		bmax[2] = orientedBox.center[2] + maxr;
	}
}
