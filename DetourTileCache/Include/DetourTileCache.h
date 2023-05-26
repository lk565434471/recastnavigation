#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"

typedef unsigned int dtObstacleRef;
typedef unsigned int dtCompressedTileRef;

/// Flags for addTile
// �����Ƭ�ı�ʶ��
enum dtCompressedTileFlags
{
	// �������������Ƭ���ڴ棬�������ͷ���
	DT_COMPRESSEDTILE_FREE_DATA = 0x01	///< Navmesh owns the tile memory and should free it.
};

// ��Ƭ��ѹ������
struct dtCompressedTile
{
	// ����������Ƭ���޸ĵĴ����ļ��������ֵ������ڵ���1
	unsigned int salt;						///< Counter describing modifications to the tile.
	// ��Ƭ�����ͷ��Ϣ
	struct dtTileCacheLayerHeader* header;
	// ѹ�����ݵ���ʼָ��λ��
	unsigned char* compressed;
	// ѹ�����ݵĳ���
	int compressedSize;
	// δѹ������
	unsigned char* data;
	// δѹ�����ݴ�С
	int dataSize;
	// ��ʶ�� ��� enum dtCompressedTileFlags
	unsigned int flags;
	// ��һ��������Ƭ���ݽṹ
	dtCompressedTile* next;
};

// �ϰ���״̬
enum ObstacleState
{
	// û���ϰ���
	DT_OBSTACLE_EMPTY,
	// ������
	DT_OBSTACLE_PROCESSING,
	// ����ɴ���
	DT_OBSTACLE_PROCESSED,
	// �����Ƴ�
	DT_OBSTACLE_REMOVING
};

// �ϰ�������
enum ObstacleType
{
	// Բ����
	DT_OBSTACLE_CYLINDER,
	// AABB��Axis-Aligned Bounding Box�� ������Χ��
	DT_OBSTACLE_BOX, // AABB
	// OBB��Oriented Bounding Box�� �����Χ��
	DT_OBSTACLE_ORIENTED_BOX // OBB
};

// Բ�����ϰ�����Ϣ
struct dtObstacleCylinder
{
	// λ����Ϣ
	float pos[ 3 ];
	// �뾶
	float radius;
	// �߶�
	float height;
};

// AABB ������Χ���ϰ�����Ϣ
struct dtObstacleBox
{
	// �������εķ�Χ
	float bmin[ 3 ];
	float bmax[ 3 ];
};

// OBB �����Χ���ϰ�����Ϣ
struct dtObstacleOrientedBox
{
	// ���ĵ�
	float center[ 3 ];
	//
	float halfExtents[ 3 ];
	float rotAux[ 2 ]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

static const int DT_MAX_TOUCHED_TILES = 8;


struct dtTileCacheObstacle
{
	// ���ڲ�ͬ�ϰ��ﴦ���������
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
	};

	// touched �洢�����ϰ�����Է������ĸ���Ƭ�ϣ�����Ƭ�ĸ���id����
	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	// ���������Ƭid�б�
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	unsigned short salt;
	unsigned char type;
	unsigned char state;
	unsigned char ntouched;
	unsigned char npending;
	// ָ����һ����Ƭ�����ϰ���
	dtTileCacheObstacle* next;
};

// ��Ƭ�������
struct dtTileCacheParams
{
	float orig[3];
	// cs = cell size ��Ԫ��ߴ�
	// ch = cell height ��Ԫ��߶�
	float cs, ch;
	// width �� height �ֱ��ʾ�к��У���ˮƽ��ĵ�Ԫ������
	int width, height;
	// �������ĸ߶�
	float walkableHeight;
	// 
	float walkableRadius;
	// ���ƶ�����������߶�
	float walkableClimb;
	//
	float maxSimplificationError;
	// ������Ƭ����
	int maxTiles;
	// �����ϰ�������
	int maxObstacles;
};

// ��Ƭ����������ĳ���ӿ�
struct dtTileCacheMeshProcess
{
	virtual ~dtTileCacheMeshProcess();
	virtual void process(struct dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags) = 0;
};

// ��Ƭ����
class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();
	
	// �ڴ������
	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	// ��Ƭ����ѹ��������
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	// ��Ƭ�����������
	const dtTileCacheParams* getParams() const { return &m_params; }
	
	// �����Ƭ����
	inline int getTileCount() const { return m_params.maxTiles; }
	// ���ظ���������ѹ����Ƭ����
	inline const dtCompressedTile* getTile(const int i) const { return &m_tiles[i]; }
	
	// ����ϰ�������
	inline int getObstacleCount() const { return m_params.maxObstacles; }
	// ���ظ�����������Ƭ�����ϰ������
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }
	// ͨ���ϰ���id�������Ƭ�����ϰ������
	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);
	// ͨ��salt��dtTileCacheObstacle�������е��±꣬���һ������id
	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;
	
	// ��ʼ����Ƭ����
	dtStatus init(const dtTileCacheParams* params,
				  struct dtTileCacheAlloc* talloc,
				  struct dtTileCacheCompressor* tcomp,
				  struct dtTileCacheMeshProcess* tmproc);
	
	// ��ָ�����С����ϣ���ȡѹ������Ƭ�б�
	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const ;
	
	// ��ȡָ���С��С��㼶��ѹ����Ƭ
	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	// ͨ��salt��dtCompressedTile�������е��±꣬���һ������id
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	// ͨ��ѹ������Ƭ��id�����ѹ������Ƭ����
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;
	
	// ͨ�����������ݣ����һ����Ƭ����
	dtStatus addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result);
	
	// ͨ��ѹ������Ƭ��id��ɾ��һ����Ƭ���󣬲���¼���Ķ��������ݣ���ŵ������
	dtStatus removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize);
	
	// Cylinder obstacle.
	// ���һ��Բ�������͵��ϰ���
	// ע�⣺�ú������ǻ���һ������ָ�ʵ�ʴ����� update �н���
	dtStatus addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);

	// Aabb obstacle.
	// ���һ�� AABB�������İ�Χ�У����ϰ���
	// ע�⣺�ú������ǻ���һ������ָ�ʵ�ʴ����� update �н���
	dtStatus addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	// ���һ�� OBB�������Χ�У����ϰ���
	// ע�⣺�ú������ǻ���һ������ָ�ʵ�ʴ����� update �н���
	dtStatus addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result);
	
	// ������Ƭɾ���Ĳ���ָ��
	dtStatus removeObstacle(const dtObstacleRef ref);
	
	// ͨ������һ�� AABB��������Χ�У���ѯѹ������Ƭ��id�б�֧�ֲ�ѯ����
	dtStatus queryTiles(const float* bmin, const float* bmax,
						dtCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	/// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	///  @param[in]		dt			The time step size. Currently not used.
	///  @param[in]		navmesh		The mesh to affect when rebuilding tiles.
	///  @param[out]	upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
	///  							If the tile cache is up to date another (immediate) call to update will have no effect;
	///  							otherwise another call will continue processing obstacle requests and tile rebuilds.
	/// 
	/// ͨ������δ��ɵ���Ƭ���ع���������Ƭ���Ը�����Ƭ���档
	dtStatus update(const float dt, class dtNavMesh* navmesh, bool* upToDate = 0);
	
	// ��ָ�����С��й�������������Ƭ
	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	// ͨ��ѹ������Ƭid������������������Ƭ
	dtStatus buildNavMeshTile(const dtCompressedTileRef ref, class dtNavMesh* navmesh);
	
	// ���� AABB ��Χ�еļ�ⷶΧ
	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, float* bmin, float* bmax) const;
	
	// ��ȡ�ϰ���ļ��߽�
	void getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const;
	

	/// Encodes a tile id.
	/// ����һ����Ƭid
	/// it dtCompressedTile �����������е��±꣨��������
	/// ͨ�����±��salt���һ������id���������ⲿʹ��
	inline dtCompressedTileRef encodeTileId(unsigned int salt, unsigned int it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}
	
	/// Decodes a tile salt.
	// ����һ����Ƭ��id
	inline unsigned int decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> m_tileBits) & saltMask);
	}
	
	/// Decodes a tile id.
	/// ����һ����Ƭ��ʶ��
	inline unsigned int decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1<<m_tileBits)-1;
		return (unsigned int)(ref & tileMask);
	}

	/// Encodes an obstacle id.
	/// ����һ���ϰ���id
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}
	
	/// Decodes an obstacle salt.
	/// ����һ���ϰ����id
	/// ���� dtTileCacheObstacle ����� salt ֵ
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)((ref >> 16) & saltMask);
	}
	
	/// Decodes an obstacle id.
	/// ����һ���ϰ���ı�ʶ��
	/// ���� dtTileCacheObstacle �����������е��±꣨��������λ��
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)(ref & tileMask);
	}
	
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&);
	dtTileCache& operator=(const dtTileCache&);

	// ���ϰ��������Ϊ��ö��
	enum ObstacleRequestAction
	{
		// ����ϰ���
		REQUEST_ADD,
		// �Ƴ��ϰ���
		REQUEST_REMOVE
	};
	
	// �ϰ����������Ϊ����ṹ��
	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};
	
	// ��Ƭ��ϣ��ѯ���С��������2���ݴη���
	int m_tileLutSize;						///< Tile hash lookup size (must be pot).
	// ��Ƭ��ʶ����ϣ��ѯ��
	int m_tileLutMask;						///< Tile hash lookup mask.
	
	// ��Ƭ��ϣ��ѯ��
	dtCompressedTile** m_posLookup;			///< Tile hash lookup.
	// ��Ƭ�����б����ڴ��ʵ��
	dtCompressedTile* m_nextFreeTile;		///< Freelist of tiles.
	// ��Ƭ�б��洢������Ƭ������
	dtCompressedTile* m_tiles;				///< List of tiles.
	
	// �ڸ�����Ƭid�У� salt ռ�õı���λ��
	unsigned int m_saltBits;				///< Number of salt bits in the tile ID.
	// �ڸ�����Ƭid�У�tile ռ�õı���λ��
	unsigned int m_tileBits;				///< Number of tile bits in the tile ID.
	
	// ��Ƭ�����������
	dtTileCacheParams m_params;
	// ��Ƭ�����ڴ������
	dtTileCacheAlloc* m_talloc;
	// ��Ƭ����ѹ������
	dtTileCacheCompressor* m_tcomp;
	// ��Ƭ������������󣬾���ʵ��ȡ��������
	dtTileCacheMeshProcess* m_tmproc;
	// ��Ƭ�����ϰ����б�
	dtTileCacheObstacle* m_obstacles;
	// ��Ƭ����Ŀ����ϰ����б��ڴ��
	dtTileCacheObstacle* m_nextFreeObstacle;
	// ������������������
	static const int MAX_REQUESTS = 64;
	// �����ϰ��ﴦ���������
	ObstacleRequest m_reqs[MAX_REQUESTS];
	// ��ǰ�ѻ�����������������
	int m_nreqs;
	
	// ����������
	static const int MAX_UPDATE = 64;
	// ÿ�θ�����ѹ������Ƭid������
	dtCompressedTileRef m_update[MAX_UPDATE];
	// �Ѵ洢����ѹ������Ƭid����
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif
