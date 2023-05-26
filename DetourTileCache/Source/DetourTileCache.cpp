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

// 计算瓦片所属哈希表的桶id
inline int computeTileHash(int x, int y, const int mask)
{
	// h1 和 h2 是两个魔数，防止哈希冲突用的
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
	// 分配障碍物的内存空间，用于存储障碍物的数组
	m_obstacles = (dtTileCacheObstacle*)dtAlloc(sizeof(dtTileCacheObstacle)*m_params.maxObstacles, DT_ALLOC_PERM);
	if (!m_obstacles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_obstacles, 0, sizeof(dtTileCacheObstacle)*m_params.maxObstacles);
	m_nextFreeObstacle = 0;
	// 倒序初始化障碍物，并设置每一个障碍物的下一个可用对象的引用
	for (int i = m_params.maxObstacles-1; i >= 0; --i)
	{
		m_obstacles[i].salt = 1;
		m_obstacles[i].next = m_nextFreeObstacle;
		m_nextFreeObstacle = &m_obstacles[i];
	}
	
	// Init tiles
	// 初始化瓦片哈希查询表的大小，必须是2的幂次方
	m_tileLutSize = dtNextPow2(m_params.maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	// 瓦片标识符哈希查询表大小
	m_tileLutMask = m_tileLutSize-1;
	// 初始化用于存储瓦片对象的数组
	m_tiles = (dtCompressedTile*)dtAlloc(sizeof(dtCompressedTile)*m_params.maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	// 瓦片哈希查询表初始化
	m_posLookup = (dtCompressedTile**)dtAlloc(sizeof(dtCompressedTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(dtCompressedTile)*m_params.maxTiles);
	memset(m_posLookup, 0, sizeof(dtCompressedTile*)*m_tileLutSize);
	m_nextFreeTile = 0;
	// 倒序初始化瓦片，并设置每一个瓦片的下一个可用对象的引用
	for (int i = m_params.maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFreeTile;
		m_nextFreeTile = &m_tiles[i];
	}
	
	// Init ID generator values.
	// 通过瓦片的最大数量，动态计算瓦片占用的比特位数
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	// 只允许31个 salt 比特位，因为 salt 掩码是使用32位的unit计算的，超出部分将会溢出。
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);
	// 确保 salt 至少使用10个比特位
	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const 
{
	// 实际返回的瓦片对象数量，此值小于等于 maxTiles
	int n = 0;
	
	// Find tile based on hash.
	// 基于哈希表查找瓦片对象
	int h = computeTileHash(tx,ty,m_tileLutMask);
	// 取出已压缩的瓦片对象
	dtCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		// 找到所有属于同一行、列的瓦片对象
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty)
		{
			// 如果没有达到外带瓦片对象数组的容量上限，就把找到的瓦片存储起来
			if (n < maxTiles)
				tiles[n++] = getTileRef(tile);
		}
		tile = tile->next;
	}
	
	return n;
}

// 这个函数的功能与 getTilesAt 类似，唯一的区别是它仅返回一个对象，在指定层级查找
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

// 通过salt和dtCompressedTile在数组中的下标，组成一个复合id
dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile* tile) const
{
	if (!tile) return 0;
	// 通过当前指针位置，减去数组的第一个元素的指针位置，计算出 tile 在数组中的下标（即索引）
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtCompressedTileRef)encodeTileId(tile->salt, it);
}

// 通过salt和dtTileCacheObstacle在数组中的下标，组成一个复合id
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
	// 通过复合id，计算出 dtTileCacheObstacle 对象在数组中的下标（即索引）位置
	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_params.maxObstacles)
		return 0;
	// 从数组中取出 dtTileCacheObstacle 对象的引用
	const dtTileCacheObstacle* ob = &m_obstacles[idx];
	// 通过复合id，计算出 dtTileCacheObstacle 对象的 salt 值
	unsigned int salt = decodeObstacleIdSalt(ref);
	// 二次检查，确保对象的salt值与复合id的salt值匹配
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
	// 确保数据格式是正确的
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
	if (header->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	// Make sure the location is free.
	// 确保由tx、ty及layer表示的位置处于空闲状态
	if (getTileAt(header->tx, header->ty, header->tlayer))
		return DT_FAILURE;
	
	// Allocate a tile.
	// 分配一个瓦片对象
	dtCompressedTile* tile = 0;
	if (m_nextFreeTile)
	{
		tile = m_nextFreeTile;
		m_nextFreeTile = tile->next;
		tile->next = 0;
	}
	
	// Make sure we could allocate a tile.
	// 所有空间已被占用，空闲列表为空
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	// 计算瓦片对象在哈希表的位置
	int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
	// 将哈希表中原有的对象设置为当前瓦片的下一个可用对象
	tile->next = m_posLookup[h];
	// 覆盖掉原来对象指针，指向新创建的瓦片对象
	m_posLookup[h] = tile;
	
	// Init tile.
	// 初始化瓦片的成员变量
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	tile->header = (dtTileCacheLayerHeader*)data;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->compressed = tile->data + headerSize;
	tile->compressedSize = tile->dataSize - headerSize;
	tile->flags = flags;
	
	// 设置瓦片的复合id
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	// 解码出瓦片在数组中的下标（即索引）
	unsigned int tileIndex = decodeTileIdTile(ref);
	// 解码出瓦片对应的 salt 值
	unsigned int tileSalt = decodeTileIdSalt(ref);
	// 检查下标是否超出了数组可容纳的瓦片数量上限
	if ((int)tileIndex >= m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	// 二次检查，确保传入的瓦片复合id中存储的 salt 值与对象自身存储的 salt 值一致
	dtCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	// 计算瓦片的哈希值
	const int h = computeTileHash(tile->header->tx,tile->header->ty,m_tileLutMask);
	dtCompressedTile* prev = 0;
	dtCompressedTile* cur = m_posLookup[h];
	// while 循环会沿着被删除的瓦片的调用链一直遍历下去，直到没有下一个可用的为止
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				// 把下一块可用内存，放回哈希表
				m_posLookup[h] = cur->next;
			break;
		}
		// 设置前一个瓦片为当前瓦片
		prev = cur;
		// 设置当前瓦片为下一个可用瓦片
		cur = cur->next;
	}
	
	// Reset tile.
	// 检查瓦片的创建标识符，如果是由 dtTileCache 管理的内存，由 dtTileCache 负责释放
	if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	// 否则，把内存加入到内存池，以便再一次使用
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}
	
	// 重置被删除的瓦片对象的成员变量
	tile->header = 0;
	tile->data = 0;
	tile->dataSize = 0;
	tile->compressed = 0;
	tile->compressedSize = 0;
	tile->flags = 0;
	
	// Update salt, salt should never be zero.
	// 更新瓦片的数据变更计数，salt 必须是一个大于等于1的数字
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
	if (tile->salt == 0)
		tile->salt++;
	
	// Add to free list.
	tile->next = m_nextFreeTile;
	m_nextFreeTile = tile;
	
	return DT_SUCCESS;
}

// 添加一个圆柱体类型的障碍物，使用圆柱体包围盒检查
dtStatus dtTileCache::addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result)
{
	// 检查当前缓存的添加障碍物的请求释放已达到上限
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	// 检查允许添加的障碍物是否已达到上限
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
	// 重置障碍物的初始数据
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_CYLINDER;
	dtVcopy(ob->cylinder.pos, pos);
	ob->cylinder.radius = radius;
	ob->cylinder.height = height;
	
	// 缓存创建障碍物的请求
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	// 外带创建的障碍物的复合id
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

// 与 addObstacle 类似，区别在于使用的是 AABB（轴对齐包围盒）
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

// 与 addObstacle 类似，区别在于使用的是 OBB（有向包围盒）
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

// 通过复合id，删除障碍物
dtStatus dtTileCache::removeObstacle(const dtObstacleRef ref)
{
	if (!ref)
		return DT_SUCCESS;
	// 检查是否有足够的空间缓存对瓦片操作
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	// 缓存瓦片删除的操作指令
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_REMOVE;
	req->ref = ref;
	
	return DT_SUCCESS;
}

// bmin 和 bmax 指定一个查询范围
// results 记录查询到的瓦片的复合id，是一个数组类型
// resultCount 查询到的记录数量
// maxResults results 可以存储的数量上限
dtStatus dtTileCache::queryTiles(const float* bmin, const float* bmax,
								 dtCompressedTileRef* results, int* resultCount, const int maxResults) const 
{
	const int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES];
	
	int n = 0;
	
	// 计算导航网格在 xz 轴上的总尺寸
	const float tw = m_params.width * m_params.cs;
	const float th = m_params.height * m_params.cs;
	// 计算出起始的瓦片位置，即以行列的形式
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
				// 计算瓦片的 AABB 边界
				calcTightTileBounds(tile->header, tbmin, tbmax);
				
				// 检测瓦片的边界是否与判断区间重叠
				if (dtOverlapBounds(bmin,bmax, tbmin,tbmax))
				{
					// 记录重叠的瓦片对象
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
		// 处理缓存的请求，每次处理所有请求，在退出前，重置 m_nreqs = 0
		for (int i = 0; i < m_nreqs; ++i)
		{
			ObstacleRequest* req = &m_reqs[i];
			// 解码障碍物在数组中的下标
			unsigned int idx = decodeObstacleIdObstacle(req->ref);
			if ((int)idx >= m_params.maxObstacles)
				continue;
			// 取出障碍物对象，验证障碍物是否有效
			dtTileCacheObstacle* ob = &m_obstacles[idx];
			unsigned int salt = decodeObstacleIdSalt(req->ref);
			if (ob->salt != salt)
				continue;
			
			// 处理添加障碍物的请求
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
						// 检查瓦片是否已经在更新列表中，不存在则加入到列表
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						// 障碍物所属的瓦片加入到待处理的 pending 列表中
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
			// 处理删除障碍物的请求
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
						// 检查瓦片是否已经在更新列表中，不存在则加入到列表
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						// 障碍物所属的瓦片加入到待处理的 pending 列表中
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
		}
		
		m_nreqs = 0;
	}
	
	dtStatus status = DT_SUCCESS;
	// Process updates
	// 至少有一个瓦片是新加入到 m_update 列表的
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
	// 计算可移动物体在垂直方向可以攀爬的最大单元格数量
	const int walkableClimbVx = (int)(m_params.walkableClimb / m_params.ch);
	dtStatus status;
	
	// Decompress tile layer data. 
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;
	
	// Rasterize obstacles.
	// 障碍物栅格化，将三维空间问题转化为基于网格的二维表示，以提高导航和路径规划的性能和效率。
	for (int i = 0; i < m_params.maxObstacles; ++i)
	{
		const dtTileCacheObstacle* ob = &m_obstacles[i];
		// 标识障碍物不存在，或正在等待删除的阶段
		if (ob->state == DT_OBSTACLE_EMPTY || ob->state == DT_OBSTACLE_REMOVING)
			continue;
		// 判断瓦片是否已经被允许放置
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
	// 筛选边界检测的类型
	// 圆柱体
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
	// AABB（轴对齐包围盒）
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		dtVcopy(bmin, ob->box.bmin);
		dtVcopy(bmax, ob->box.bmax);
	}
	// OBB（有向包围盒）
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		const dtObstacleOrientedBox &orientedBox = ob->orientedBox;
		// 使用物体的最大半径，可以得到物体在水平面上的最大尺寸的估值。
		// 这种方式可以有效的简化碰撞检测、物体间距离计算和空间分割等操作。
		// 但是，这种方法可能会导致一些误报，但在大多数情况下，这种计算方式已经足够好了，且计算成本较低。
		float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
		bmin[0] = orientedBox.center[0] - maxr;
		bmax[0] = orientedBox.center[0] + maxr;
		bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
		bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
		bmin[2] = orientedBox.center[2] - maxr;
		bmax[2] = orientedBox.center[2] + maxr;
	}
}
