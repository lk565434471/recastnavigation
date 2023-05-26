#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"

typedef unsigned int dtObstacleRef;
typedef unsigned int dtCompressedTileRef;

/// Flags for addTile
// 添加瓦片的标识符
enum dtCompressedTileFlags
{
	// 导航网格持有瓦片的内存，并负责释放它
	DT_COMPRESSEDTILE_FREE_DATA = 0x01	///< Navmesh owns the tile memory and should free it.
};

// 瓦片的压缩数据
struct dtCompressedTile
{
	// 用于描述瓦片被修改的次数的计数，这个值必须大于等于1
	unsigned int salt;						///< Counter describing modifications to the tile.
	// 瓦片缓存层头信息
	struct dtTileCacheLayerHeader* header;
	// 压缩数据的起始指针位置
	unsigned char* compressed;
	// 压缩数据的长度
	int compressedSize;
	// 未压缩数据
	unsigned char* data;
	// 未压缩数据大小
	int dataSize;
	// 标识符 详见 enum dtCompressedTileFlags
	unsigned int flags;
	// 下一个相邻瓦片数据结构
	dtCompressedTile* next;
};

// 障碍物状态
enum ObstacleState
{
	// 没有障碍物
	DT_OBSTACLE_EMPTY,
	// 处理中
	DT_OBSTACLE_PROCESSING,
	// 已完成处理
	DT_OBSTACLE_PROCESSED,
	// 正在移除
	DT_OBSTACLE_REMOVING
};

// 障碍物类型
enum ObstacleType
{
	// 圆柱体
	DT_OBSTACLE_CYLINDER,
	// AABB（Axis-Aligned Bounding Box） 轴对齐包围盒
	DT_OBSTACLE_BOX, // AABB
	// OBB（Oriented Bounding Box） 有向包围盒
	DT_OBSTACLE_ORIENTED_BOX // OBB
};

// 圆柱体障碍物信息
struct dtObstacleCylinder
{
	// 位置信息
	float pos[ 3 ];
	// 半径
	float radius;
	// 高度
	float height;
};

// AABB 轴对齐包围盒障碍物信息
struct dtObstacleBox
{
	// 描述矩形的范围
	float bmin[ 3 ];
	float bmax[ 3 ];
};

// OBB 有向包围盒障碍物信息
struct dtObstacleOrientedBox
{
	// 中心点
	float center[ 3 ];
	//
	float halfExtents[ 3 ];
	float rotAux[ 2 ]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

static const int DT_MAX_TOUCHED_TILES = 8;


struct dtTileCacheObstacle
{
	// 用于不同障碍物处理的联合体
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
	};

	// touched 存储的是障碍物可以放置在哪个瓦片上，是瓦片的复合id数组
	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	// 待处理的瓦片id列表
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	unsigned short salt;
	unsigned char type;
	unsigned char state;
	unsigned char ntouched;
	unsigned char npending;
	// 指向下一个瓦片缓存障碍物
	dtTileCacheObstacle* next;
};

// 瓦片缓存参数
struct dtTileCacheParams
{
	float orig[3];
	// cs = cell size 单元格尺寸
	// ch = cell height 单元格高度
	float cs, ch;
	// width 和 height 分别表示行和列，即水平面的单元格数量
	int width, height;
	// 可攀爬的高度
	float walkableHeight;
	// 
	float walkableRadius;
	// 可移动物体的攀爬高度
	float walkableClimb;
	//
	float maxSimplificationError;
	// 最大的瓦片数量
	int maxTiles;
	// 最大的障碍物数量
	int maxObstacles;
};

// 瓦片缓存网格处理的抽象接口
struct dtTileCacheMeshProcess
{
	virtual ~dtTileCacheMeshProcess();
	virtual void process(struct dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags) = 0;
};

// 瓦片缓存
class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();
	
	// 内存分配器
	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	// 瓦片缓存压缩处理器
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	// 瓦片缓存参数配置
	const dtTileCacheParams* getParams() const { return &m_params; }
	
	// 最大瓦片数量
	inline int getTileCount() const { return m_params.maxTiles; }
	// 返回给定索引的压缩瓦片对象
	inline const dtCompressedTile* getTile(const int i) const { return &m_tiles[i]; }
	
	// 最大障碍物数量
	inline int getObstacleCount() const { return m_params.maxObstacles; }
	// 返回给定索引的瓦片缓存障碍物对象
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }
	// 通过障碍物id，获得瓦片缓存障碍物对象
	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);
	// 通过salt和dtTileCacheObstacle在数组中的下标，组成一个复合id
	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;
	
	// 初始化瓦片缓存
	dtStatus init(const dtTileCacheParams* params,
				  struct dtTileCacheAlloc* talloc,
				  struct dtTileCacheCompressor* tcomp,
				  struct dtTileCacheMeshProcess* tmproc);
	
	// 在指定的行、列上，获取压缩的瓦片列表
	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const ;
	
	// 获取指定行、列、层级的压缩瓦片
	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	// 通过salt和dtCompressedTile在数组中的下标，组成一个复合id
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	// 通过压缩的瓦片的id，获得压缩的瓦片对象
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;
	
	// 通过二进制数据，添加一个瓦片对象
	dtStatus addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result);
	
	// 通过压缩的瓦片的id，删除一个瓦片对象，并记录它的二进制数据，存放到缓冲池
	dtStatus removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize);
	
	// Cylinder obstacle.
	// 添加一个圆柱体类型的障碍物
	// 注意：该函数仅是缓存一个创建指令，实际创建在 update 中进行
	dtStatus addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);

	// Aabb obstacle.
	// 添加一个 AABB（轴对齐的包围盒）的障碍物
	// 注意：该函数仅是缓存一个创建指令，实际创建在 update 中进行
	dtStatus addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	// 添加一个 OBB（有向包围盒）的障碍物
	// 注意：该函数仅是缓存一个创建指令，实际创建在 update 中进行
	dtStatus addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result);
	
	// 缓存瓦片删除的操作指令
	dtStatus removeObstacle(const dtObstacleRef ref);
	
	// 通过给定一个 AABB（轴对齐包围盒）查询压缩的瓦片的id列表，支持查询上限
	dtStatus queryTiles(const float* bmin, const float* bmax,
						dtCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	/// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	///  @param[in]		dt			The time step size. Currently not used.
	///  @param[in]		navmesh		The mesh to affect when rebuilding tiles.
	///  @param[out]	upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
	///  							If the tile cache is up to date another (immediate) call to update will have no effect;
	///  							otherwise another call will continue processing obstacle requests and tile rebuilds.
	/// 
	/// 通过请求未完成的瓦片来重构触碰的瓦片，以更新瓦片缓存。
	dtStatus update(const float dt, class dtNavMesh* navmesh, bool* upToDate = 0);
	
	// 在指定的行、列构建导航网格瓦片
	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	// 通过压缩的瓦片id，来构建导航网格瓦片
	dtStatus buildNavMeshTile(const dtCompressedTileRef ref, class dtNavMesh* navmesh);
	
	// 计算 AABB 包围盒的检测范围
	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, float* bmin, float* bmax) const;
	
	// 获取障碍物的检测边界
	void getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const;
	

	/// Encodes a tile id.
	/// 编码一个瓦片id
	/// it dtCompressedTile 对象在数组中的下标（即索引）
	/// 通过将下标和salt组合一个复合id，方便在外部使用
	inline dtCompressedTileRef encodeTileId(unsigned int salt, unsigned int it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}
	
	/// Decodes a tile salt.
	// 解码一个瓦片的id
	inline unsigned int decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> m_tileBits) & saltMask);
	}
	
	/// Decodes a tile id.
	/// 解码一个瓦片标识符
	inline unsigned int decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1<<m_tileBits)-1;
		return (unsigned int)(ref & tileMask);
	}

	/// Encodes an obstacle id.
	/// 编码一个障碍物id
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}
	
	/// Decodes an obstacle salt.
	/// 解码一个障碍物的id
	/// 计算 dtTileCacheObstacle 对象的 salt 值
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)((ref >> 16) & saltMask);
	}
	
	/// Decodes an obstacle id.
	/// 解码一个障碍物的标识符
	/// 计算 dtTileCacheObstacle 对象在数组中的下标（即索引）位置
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1<<16)-1;
		return (unsigned int)(ref & tileMask);
	}
	
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&);
	dtTileCache& operator=(const dtTileCache&);

	// 对障碍物操作行为的枚举
	enum ObstacleRequestAction
	{
		// 添加障碍物
		REQUEST_ADD,
		// 移除障碍物
		REQUEST_REMOVE
	};
	
	// 障碍物操作的行为处理结构体
	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};
	
	// 瓦片哈希查询表大小（必须是2的幂次方）
	int m_tileLutSize;						///< Tile hash lookup size (must be pot).
	// 瓦片标识符哈希查询表
	int m_tileLutMask;						///< Tile hash lookup mask.
	
	// 瓦片哈希查询表
	dtCompressedTile** m_posLookup;			///< Tile hash lookup.
	// 瓦片空闲列表，简单内存池实现
	dtCompressedTile* m_nextFreeTile;		///< Freelist of tiles.
	// 瓦片列表，存储所有瓦片的数据
	dtCompressedTile* m_tiles;				///< List of tiles.
	
	// 在复合瓦片id中， salt 占用的比特位数
	unsigned int m_saltBits;				///< Number of salt bits in the tile ID.
	// 在复合瓦片id中，tile 占用的比特位数
	unsigned int m_tileBits;				///< Number of tile bits in the tile ID.
	
	// 瓦片缓存参数配置
	dtTileCacheParams m_params;
	// 瓦片缓存内存分配器
	dtTileCacheAlloc* m_talloc;
	// 瓦片缓存压缩对象
	dtTileCacheCompressor* m_tcomp;
	// 瓦片缓存网格处理对象，具体实现取决于子类
	dtTileCacheMeshProcess* m_tmproc;
	// 瓦片缓存障碍物列表
	dtTileCacheObstacle* m_obstacles;
	// 瓦片缓存的可用障碍物列表，内存池
	dtTileCacheObstacle* m_nextFreeObstacle;
	// 最大处理请求的数量上限
	static const int MAX_REQUESTS = 64;
	// 缓存障碍物处理请求对象
	ObstacleRequest m_reqs[MAX_REQUESTS];
	// 当前已缓存的请求处理对象数量
	int m_nreqs;
	
	// 最大更新数量
	static const int MAX_UPDATE = 64;
	// 每次更新已压缩的瓦片id的数组
	dtCompressedTileRef m_update[MAX_UPDATE];
	// 已存储的已压缩的瓦片id数量
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif
