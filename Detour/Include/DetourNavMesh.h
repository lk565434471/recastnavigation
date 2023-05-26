//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURNAVMESH_H
#define DETOURNAVMESH_H

#include "DetourAlloc.h"
#include "DetourStatus.h"

// Undefine (or define in a build cofnig) the following line to use 64bit polyref.
// Generally not needed, useful for very large worlds.
// Note: tiles build using 32bit refs are not compatible with 64bit refs!
//#define DT_POLYREF64 1

#ifdef DT_POLYREF64
// TODO: figure out a multiplatform version of uint64_t
// - maybe: https://code.google.com/p/msinttypes/
// - or: http://www.azillionmonkeys.com/qed/pstdint.h
#include <stdint.h>
#endif

// Note: If you want to use 64-bit refs, change the types of both dtPolyRef & dtTileRef.
// It is also recommended that you change dtHashRef() to a proper 64-bit hash.

/// A handle to a polygon within a navigation mesh tile.
/// @ingroup detour
#ifdef DT_POLYREF64
static const unsigned int DT_SALT_BITS = 16;
static const unsigned int DT_TILE_BITS = 28;
static const unsigned int DT_POLY_BITS = 20;
typedef uint64_t dtPolyRef;
#else
typedef unsigned int dtPolyRef;
#endif

/// A handle to a tile within a navigation mesh.
/// @ingroup detour
#ifdef DT_POLYREF64
typedef uint64_t dtTileRef;
#else
typedef unsigned int dtTileRef;
#endif

/// The maximum number of vertices per navigation polygon.
/// @ingroup detour
/// 
/// 每个导航多边形最大的顶点数量上限
static const int DT_VERTS_PER_POLYGON = 6;

/// @{
/// @name Tile Serialization Constants
/// These constants are used to detect whether a navigation tile's data
/// and state format is compatible with the current build.
///

/// A magic number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V';

/// A version number used to detect compatibility of navigation tile data.
// 用于检测导航网格数据兼容性的版本号
static const int DT_NAVMESH_VERSION = 7;

/// A magic number used to detect the compatibility of navigation tile states.
// 用于检测导航网格状态兼容性的魔数
static const int DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S';

/// A version number used to detect compatibility of navigation tile states.
// 用于检测导航网格状态兼容性的版本号
static const int DT_NAVMESH_STATE_VERSION = 1;

/// @}

/// A flag that indicates that an entity links to an external entity.
/// (E.g. A polygon edge is a portal that links to another polygon.)
/// 
/// 一个标识指示实体是否链接到一个外部实体
/// 一个多边形的边是指连接到另外一个多边形的门户（门户指的是在寻路中，从一个多
/// 边形移动到另外一个多边形的连接线）
static const unsigned short DT_EXT_LINK = 0x8000;

/// A value that indicates the entity does not link to anything.
/// 一个值指示实体没有连接到任何其它实体
static const unsigned int DT_NULL_LINK = 0xffffffff;

/// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
/// 一个标识指示可以在一个双向导航时穿越一个离线连接。离散连接通常用于跨越不相邻的网格，不需要沿着
/// 寻路网格的路径走，例如，MMO 中从悬崖跳下去后，直接到了一个距离当前网格很远的网格上。
static const unsigned int DT_OFFMESH_CON_BIDIR = 1;

/// The maximum number of user defined area ids.
/// @ingroup detour
/// 
/// 用户定义区域id数量的最大上限
static const int DT_MAX_AREAS = 64;

/// Tile flags used for various functions and fields.
/// For an example, see dtNavMesh::addTile().
/// 
/// 瓦片标识用于各种函数和字段
enum dtTileFlags
{
	/// The navigation mesh owns the tile memory and is responsible for freeing it.
	// 导航网格持有瓦片内存，并负责释放它。
	DT_TILE_FREE_DATA = 0x01
};

/// Vertex flags returned by dtNavMeshQuery::findStraightPath.
/// 通过 dtNavMeshQuery::findStraightPath 函数返回顶点标识
enum dtStraightPathFlags
{
	// 这个顶点是路径的起始位置
	DT_STRAIGHTPATH_START = 0x01,				///< The vertex is the start position in the path.
	// 这个顶点是路径的终止位置
	DT_STRAIGHTPATH_END = 0x02,					///< The vertex is the end position in the path.
	// 这个顶点是一条离散连接的起点
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04	///< The vertex is the start of an off-mesh connection.
};

/// Options for dtNavMeshQuery::findStraightPath.
/// 用于 dtNavMeshQuery::findStraightPath 的选项参数
enum dtStraightPathOptions
{
	// 在每个多边形边缘交叉的位置添加一个顶点，以便在区域变化时进行标识
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,	///< Add a vertex at every polygon edge crossing where area changes.
	// 在每个多边形边界交叉的位置添加一个顶点
	DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02	///< Add a vertex at every polygon edge crossing.
};


/// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
/// 用于 dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath 函数的选项参数
enum dtFindPathOptions
{
	// 在路径规划过程中，使用射线投射来进行快速查找（射线投射仍然需要考虑路径上的成本因素）
	DT_FINDPATH_ANY_ANGLE	= 0x02		///< use raycasts during pathfind to "shortcut" (raycast still consider costs)
};

/// Options for dtNavMeshQuery::raycast
enum dtRaycastOptions
{
	// 射线投射应计算射线沿着路径的移动成本，并填充 RaycastHit::cost
	DT_RAYCAST_USE_COSTS = 0x01		///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
};

enum dtDetailTriEdgeFlags
{
	// 局部三角形的边是多边形边界的一部分
	DT_DETAIL_EDGE_BOUNDARY = 0x01		///< Detail triangle edge is part of the poly boundary
};


/// Limit raycasting during any angle pahfinding
/// The limit is given as a multiple of the character radius
/// 
/// 限制沿着任意角度进行的路径查找期间的射线投射
/// 在任意角度寻路时，限制射线投射的距离是角色半径的倍数
static const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

/// Flags representing the type of a navigation mesh polygon.
// 导航网格多边形的类型标志
enum dtPolyTypes
{
	/// The polygon is a standard convex polygon that is part of the surface of the mesh.
	// 该多边形是网格表面的标准凸多边形。即每条边都是一条直线。
	DT_POLYTYPE_GROUND = 0,
	/// The polygon is an off-mesh connection consisting of two vertices.
	// 该多边形是由两个顶点构成的网格链接，它不是网格表面的一部分，用于链接两个不相邻的区域
	DT_POLYTYPE_OFFMESH_CONNECTION = 1
};


/// Defines a polygon within a dtMeshTile object.
/// @ingroup detour
/// 
/// 定义了一个 dtMeshTile 对象中的多边形
struct dtPoly
{
	/// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	// 在链接列表中，第一个链接的索引
	unsigned int firstLink;

	/// The indices of the polygon's vertices.
	/// The actual vertices are located in dtMeshTile::verts.
	/// 表示该多边形的顶点
	/// 实际上，verts 中存储是多边形的顶点索引，真实的顶点数据位于 dtMeshTile::verts 中。
	unsigned short verts[DT_VERTS_PER_POLYGON];

	/// Packed data representing neighbor polygons references and flags for each edge.
	// 包含多边形相邻关系的压缩数据的数组。
	// 每个多边形有三条边，索引数组中每三个元素表示相邻多边形的引用和相邻关系标志
	unsigned short neis[DT_VERTS_PER_POLYGON];

	/// The user defined polygon flags.
	// 用户定义的多边形标志
	unsigned short flags;

	/// The number of vertices in the polygon.
	// 多边形中顶点的数量
	unsigned char vertCount;

	/// The bit packed area id and polygon type.
	/// @note Use the structure's set and get methods to acess this value.
	/// 位包装的区域id和多边形类型
	/// 区域id < DT_MAX_AREAS, 低6位保存区域id
	/// 多边形类型 enum dtPolyTypes，高2位保存类型id
	unsigned char areaAndtype;

	/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
	// 低6位存储区域id，id 的范围在 [0, 63] 之间
	inline void setArea(unsigned char a) { areaAndtype = (areaAndtype & 0xc0) | (a & 0x3f); }

	/// Sets the polygon type. (See: #dtPolyTypes.)
	// 高2位存储多边形类型，范围 [0, 3] 之间
	inline void setType(unsigned char t) { areaAndtype = (areaAndtype & 0x3f) | (t << 6); }

	/// Gets the user defined area id.
	inline unsigned char getArea() const { return areaAndtype & 0x3f; }

	/// Gets the polygon type. (See: #dtPolyTypes)
	inline unsigned char getType() const { return areaAndtype >> 6; }
};

/// Defines the location of detail sub-mesh data within a dtMeshTile.
// dtPolyDetail 定义了多边形的细节信息，包括在 dtMeshTile 中的位置。
struct dtPolyDetail
{
	// 在 dtMeshTile::detailVerts 数组中的顶点偏移
	unsigned int vertBase;			///< The offset of the vertices in the dtMeshTile::detailVerts array.
	// 在 dtMeshTile::detailTris 数组中的三角形偏移
	unsigned int triBase;			///< The offset of the triangles in the dtMeshTile::detailTris array.
	// 子网格的顶点数量
	unsigned char vertCount;		///< The number of vertices in the sub-mesh.
	// 子网格的三角形数量
	unsigned char triCount;			///< The number of triangles in the sub-mesh.
};

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
/// 
/// 定义两个多边形之间的链接
struct dtLink
{
	// 邻居的引用，指相邻链接的多边形
	dtPolyRef ref;					///< Neighbour reference. (The neighbor that is linked to.)
	// 下一个链接的索引
	unsigned int next;				///< Index of the next link.
	// 拥有此链接的多边形边缘的索引，这个值其实就是顶点数组的下标，这样的注释带给我些许迷惑。
	unsigned char edge;				///< Index of the polygon edge that owns this link.
	// 如果是一个边界链接，定义链接在哪一侧
	unsigned char side;				///< If a boundary link, defines on which side the link is.
	// 如果是一个边界链接，定义最小的子边缘面积
	unsigned char bmin;				///< If a boundary link, defines the minimum sub-edge area.
	// 如果是一个边界链接，定义最大的子边缘面积
	unsigned char bmax;				///< If a boundary link, defines the maximum sub-edge area.
};

/// Bounding volume node.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
/// 
/// 包围体节点，主要用于进行导航网格的构建和优化。
struct dtBVNode
{
	// AABB 包围盒的最小边界
	unsigned short bmin[3];			///< Minimum bounds of the node's AABB. [(x, y, z)]
	// AABB 包围盒的最大边界
	unsigned short bmax[3];			///< Maximum bounds of the node's AABB. [(x, y, z)]
	// 节点的索引，负值表示该节点是一个逃逸节点
	int i;							///< The node's index. (Negative for escape sequence.)
};

/// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
/// An off-mesh connection is a user defined traversable connection made up to two vertices.
/// 
/// dtMeshTile 对象中定义导航网格的离散链接
/// 一个离散链接是用户定义的可遍历链接，由两个顶点组成。
struct dtOffMeshConnection
{
	/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	// 存储离散链接的起点和终点坐标
	float pos[6];

	/// The radius of the endpoints. [Limit: >= 0]
	// 表示起点和终点的半径，用于表示在此链接上移动的空间需求。此值必须大于等于零
	float rad;		

	/// The polygon reference of the connection within the tile.
	// 网格内部链接的多边形id
	unsigned short poly;

	/// Link flags. 
	/// @note These are not the connection's user defined flags. Those are assigned via the 
	/// connection's dtPoly definition. These are link flags used for internal purposes.
	/// 用于内部目的的链接标志，这些标志通过链接的 dtPoly 定义分配。
	unsigned char flags;

	/// End point side.
	// 终点所在的侧面
	unsigned char side;

	/// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	// 离散链接的id，在导航网格构建时，由用户分配的。
	unsigned int userId;
};

/// Provides high level information related to a dtMeshTile object.
/// @ingroup detour
/// 
/// 提供与 dtMeshTile 对象相关的高级信息
struct dtMeshHeader
{
	// 网格（瓦片）魔数，用于定义数据的格式（通常用于解析文件，以此来确定文件的类型格式）
	int magic;				///< Tile magic number. (Used to identify the data format.)
	// 网格（瓦片）数据格式的版本号（每当更新内容时，通常会伴随着文件格式信息的增删，
	// 版本号用于确定解析文件时，区分处理不同的文件版本格式）
	int version;			///< Tile data format version number.
	// 网格（瓦片）在 dtNavMesh 中的空间网格 x 轴坐标
	int x;					///< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// 网格（瓦片）在 dtNavMesh 中的空间网格 y 轴坐标
	int y;					///< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// 网格（瓦片）在 dtNavMesh 中的空间网格所属的层级
	int layer;				///< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	// 用户定义的网格（瓦片）id（编号）
	unsigned int userId;	///< The user defined id of the tile.
	// 网格（瓦片）中包含的多边形数量
	int polyCount;			///< The number of polygons in the tile.
	// 网格（瓦片）中包含的顶点数量
	int vertCount;			///< The number of vertices in the tile.
	// 网格（瓦片）的最大相邻连接数
	int maxLinkCount;		///< The number of allocated links.
	// 细节网格中的子网格数量
	int detailMeshCount;	///< The number of sub-meshes in the detail mesh.
	
	/// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	// 除了多边形顶点以外，在细节网格中独一无二的顶点数量
	int detailVertCount;
	
	// 在细节网格中包含的三角形数量
	int detailTriCount;			///< The number of triangles in the detail mesh.
	// 边界体积节点的数量，如果未启用边界体积，则为零
	int bvNodeCount;			///< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	// 离散连接线的数量
	int offMeshConCount;		///< The number of off-mesh connections.
	// 第一个离散连接多边形的索引
	int offMeshBase;			///< The index of the first polygon which is an off-mesh connection.
	// Agent（代理）在网格（瓦片）中使用的高度信息，用于碰撞检测、避障等功能
	float walkableHeight;		///< The height of the agents using the tile.
	// Agent（代理）在网格（瓦片）中使用的半径信息，用于碰撞检测、避障等功能
	float walkableRadius;		///< The radius of the agents using the tile.
	// Agent（代理）在网格（瓦片）中使用的最大攀爬高度信息，用于碰撞检测、避障等功能
	float walkableClimb;		///< The maximum climb height of the agents using the tile.
	// Agent（代理）在网格（瓦片）中用于 AABB 碰撞检测的最小边界
	float bmin[3];				///< The minimum bounds of the tile's AABB. [(x, y, z)]
	// Agent（代理）在网格（瓦片）中用于 AABB 碰撞检测的最大边界
	float bmax[3];				///< The maximum bounds of the tile's AABB. [(x, y, z)]
	
	/// The bounding volume quantization factor. 
	// 边界体积量化因子
	float bvQuantFactor;
};

/// Defines a navigation mesh tile.
/// @ingroup detour
/// 
/// 定义导航网格（瓦片）
struct dtMeshTile
{
	// 描述对网格（瓦片）进行修改的计数器
	unsigned int salt;					///< Counter describing modifications to the tile.

	// 下一个空闲链表的索引
	unsigned int linksFreeList;			///< Index to the next free link.
	// 网格（瓦片）的头信息
	dtMeshHeader* header;				///< The tile header.
	// 网格（瓦片）内的多边形数据
	dtPoly* polys;						///< The tile polygons. [Size: dtMeshHeader::polyCount]
	// 网格（瓦片）的顶点数组 [(x, y, z) * dtMeshHeader::vertCount]
	float* verts;						///< The tile vertices. [(x, y, z) * dtMeshHeader::vertCount]
	// 网格（瓦片）的连接数组 [(dtMeshHeader) * dtMeshHeader::maxLinkCount]
	dtLink* links;						///< The tile links. [Size: dtMeshHeader::maxLinkCount]
	// 网格（瓦片）的细节网格（瓦片）数组 [(dtMeshHeader) * dtMeshHeader::detailMeshCount]
	dtPolyDetail* detailMeshes;			///< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	
	/// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// 细节网格的唯一顶点数组 [(x, y, z) * dtMeshHeader::detailVertCount]
	float* detailVerts;	

	/// The detail mesh's triangles. [(vertA, vertB, vertC, triFlags) * dtMeshHeader::detailTriCount].
	/// See dtDetailTriEdgeFlags and dtGetDetailTriEdgeFlags.
	/// 
	/// 细节网格的三角形数组，[((vertA, vertB, vertC, triFlags) * dtMeshHeader::detailTriCount]
	unsigned char* detailTris;	

	/// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	/// (Will be null if bounding volumes are disabled.)
	/// 
	/// 网格（瓦片）的边界体积节点
	dtBVNode* bvTree;

	// 网格（瓦片）的离散连接线
	dtOffMeshConnection* offMeshCons;		///< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	
	// 网格（瓦片）的底层数据，通常不会被直接访问
	unsigned char* data;					///< The tile data. (Not directly accessed under normal situations.)
	// 网格（瓦片）数据的大小（即缓冲区长度）
	int dataSize;							///< Size of the tile data.
	// 网格（瓦片）的标志位
	int flags;								///< Tile flags. (See: #dtTileFlags)
	// 下一个空闲网格（瓦片），或者是下一个在空间网格中的网格（瓦片）
	// 通常通过 (x, y, z) 或索引计算，来获取该位置上的网格
	dtMeshTile* next;						///< The next free tile, or the next tile in the spatial grid.
private:
	dtMeshTile(const dtMeshTile&);
	dtMeshTile& operator=(const dtMeshTile&);
};

/// Get flags for edge in detail triangle.
/// @param[in]	triFlags		The flags for the triangle (last component of detail vertices above).
/// @param[in]	edgeIndex		The index of the first vertex of the edge. For instance, if 0,
///								returns flags for edge AB.
/// 
/// 获得局部三角形的边的标识
inline int dtGetDetailTriEdgeFlags(unsigned char triFlags, int edgeIndex)
{
	return (triFlags >> (edgeIndex * 2)) & 0x3;
}

/// Configuration parameters used to define multi-tile navigation meshes.
/// The values are used to allocate space during the initialization of a navigation mesh.
/// @see dtNavMesh::init()
/// @ingroup detour
/// 
/// 多图块导航网格的配置参数，多图块是将导航网格划分位多个图块，以允许处理更大的地图。
struct dtNavMeshParams
{
	// 导航网格所在的世界空间的原点在导航网格的图块空间中的位置
	// 简单理解就是，世界空间和本地空间的映射
	float orig[3];					///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	// 每个网格的宽度，沿 x 轴
	float tileWidth;				///< The width of each tile. (Along the x-axis.)
	// 每个网格的高度，沿 z 轴
	float tileHeight;				///< The height of each tile. (Along the z-axis.)
	// 导航网格可以包含的最大网格数量上限，这个值和maxPolys一起用于计数需要多少位才能唯一地标识每个图块和多边形
	int maxTiles;					///< The maximum number of tiles the navigation mesh can contain. This and maxPolys are used to calculate how many bits are needed to identify tiles and polygons uniquely.
	// 每个网格可以包含的最大多边形数量上限，这个值和maxTiles一起用于计数需要多少位才能唯一地标识每个图块和多边形
	int maxPolys;					///< The maximum number of polygons each tile can contain. This and maxTiles are used to calculate how many bits are needed to identify tiles and polygons uniquely.
};

/// A navigation mesh based on tiles of convex polygons.
/// @ingroup detour
/// 
/// 基于凸多边形的导航网格
class dtNavMesh
{
public:
	dtNavMesh();
	~dtNavMesh();

	/// @{
	/// @name Initialization and Tile Management

	/// Initializes the navigation mesh for tiled use.
	///  @param[in]	params		Initialization parameters.
	/// @return The status flags for the operation.
	/// 
	/// 初始化和网格管理
	dtStatus init(const dtNavMeshParams* params);

	/// Initializes the navigation mesh for single tile use.
	///  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	///  @param[in]	dataSize	The data size of the new tile.
	///  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	/// @return The status flags for the operation.
	///  @see dtCreateNavMeshData
	/// 以二进制数据格式初始化
	dtStatus init(unsigned char* data, const int dataSize, const int flags);
	
	/// The navigation mesh initialization params.
	const dtNavMeshParams* getParams() const;

	/// Adds a tile to the navigation mesh.
	///  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
	///  @param[in]		dataSize	Data size of the new tile mesh.
	///  @param[in]		flags		Tile flags. (See: #dtTileFlags)
	///  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	///  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
	/// @return The status flags for the operation.
	/// 
	/// 添加一个网格到导航网格
	/// data 新网格的数据
	/// dataSize 网格二进制数据的长度
	/// flags 网格标志位，目前仅有一个选项。表示是否有自己维护内存，并释放。
	/// lastRef 大于零值，表示可以从未使用的网格队列中获取，不需要重新创建 dtTileRef 对象
	/// result 网格对象的引用，如果添加成功。即通过二进制数据 new 出来的 dtTileRef 对象
	dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);
	
	/// Removes the specified tile from the navigation mesh.
	///  @param[in]		ref			The reference of the tile to remove.
	///  @param[out]	data		Data associated with deleted tile.
	///  @param[out]	dataSize	Size of the data associated with deleted tile.
	/// @return The status flags for the operation.
	/// 
	/// 从导航网格中移除指定的网格
	dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	/// @}

	/// @{
	/// @name Query Functions

	/// Calculates the tile grid location for the specified world position.
	///  @param[in]	pos  The world position for the query. [(x, y, z)]
	///  @param[out]	tx		The tile's x-location. (x, y)
	///  @param[out]	ty		The tile's y-location. (x, y)
	/// 
	/// 计算指定世界位置的瓦片网格
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	/// Gets the tile at the specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile, or null if the tile does not exist.
	/// 
	/// 获取指定层级上的网格
	const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	/// Gets all tiles at the specified grid location. (All layers.)
	///  @param[in]		x			The tile's x-location. (x, y)
	///  @param[in]		y			The tile's y-location. (x, y)
	///  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	///  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	/// @return The number of tiles returned in the tiles array.
	/// 
	/// 获取在指定网格位置的所有瓦片。（包含所有层级）
	int getTilesAt(const int x, const int y,
				   dtMeshTile const** tiles, const int maxTiles) const;
	
	/// Gets the tile reference for the tile at specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile reference of the tile, or 0 if there is none.
	/// 
	/// 获取在指定网格位置的瓦片id 
	dtTileRef getTileRefAt(int x, int y, int layer) const;

	/// Gets the tile reference for the specified tile.
	///  @param[in]	tile	The tile.
	/// @return The tile reference of the tile.
	/// 
	/// 获取指定瓦片的id
	dtTileRef getTileRef(const dtMeshTile* tile) const;

	/// Gets the tile for the specified tile reference.
	///  @param[in]	ref		The tile reference of the tile to retrieve.
	/// @return The tile for the specified reference, or null if the 
	///		reference is invalid.
	/// 
	/// 通过指定的瓦片id，获取瓦片对象
	const dtMeshTile* getTileByRef(dtTileRef ref) const;
	
	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	/// 
	/// 获取导航网格支持的最大瓦片数量上限
	int getMaxTiles() const;
	
	/// Gets the tile at the specified index.
	///  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	/// @return The tile at the specified index.
	/// 
	/// 获取给定索引位置上的瓦片对象
	const dtMeshTile* getTile(int i) const;

	/// Gets the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		The reference for the a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// @return The status flags for the operation.
	/// 
	/// 通过指定的多边形id，获取瓦片和多边形对象
	/// polygon reference 是一个复合类型的id
	dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;
	
	/// Returns the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		A known valid reference for a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// 
	/// 通过指定的多边形id，获取瓦片和多边形对象
	/// 与 getTileAndPolyByRef 不同的时，它不对解码后的数据进行校验，因此可能导致程序崩溃
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Checks the validity of a polygon reference.
	///  @param[in]	ref		The polygon reference to check.
	/// @return True if polygon reference is valid for the navigation mesh.
	/// 
	/// 检查一个多边形id的合法性
	bool isValidPolyRef(dtPolyRef ref) const;
	
	/// Gets the polygon reference for the tile's base polygon.
	///  @param[in]	tile		The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	/// 
	/// 获取网格的基础多边形的多边形id，基础多边形指的是每个网格的第一个多边形
	dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;
	
	/// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	///  @param[in]		prevRef		The reference of the polygon before the connection.
	///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	/// @return The status flags for the operation.
	/// 
	/// 获取离线链接的端点，按 “行进方向”排序。
	/// “行进方向” 指的是沿着一个端点到另一个端点。
	dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;

	/// Gets the specified off-mesh connection.
	///  @param[in]	ref		The polygon reference of the off-mesh connection.
	/// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	/// 
	/// 获取指定的离散链接
	/// ref 多边形id，通过对其进行解码，获取离散链接
	const dtOffMeshConnection* getOffMeshConnectionByRef(dtPolyRef ref) const;
	
	/// @}

	/// @{
	/// @name State Management
	/// These functions do not effect #dtTileRef or #dtPolyRef's. 

	/// Sets the user defined flags for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	flags	The new flags for the polygon.
	/// @return The status flags for the operation.
	/// 
	/// 将指定的多边形标识符，设置成用户定义的标识符
	dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	/// Gets the user defined flags for the specified polygon.
	///  @param[in]		ref				The polygon reference.
	///  @param[out]	resultFlags		The polygon flags.
	/// @return The status flags for the operation.
	/// 
	/// 获取多边形的标识符
	dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;

	/// Sets the user defined area for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	/// @return The status flags for the operation.
	/// 
	/// 将指定的多边形区域id，设置成用户定义的区域
	dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	/// Gets the user defined area for the specified polygon.
	///  @param[in]		ref			The polygon reference.
	///  @param[out]	resultArea	The area id for the polygon.
	/// @return The status flags for the operation.
	/// 
	/// 获取指定多边形的用户定义区域
	dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;

	/// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	///  @param[in]	tile	The tile.
	/// @return The size of the buffer required to store the state.
	/// 
	/// 获取存储指定瓦片状态所需的缓冲区大小
	int getTileStateSize(const dtMeshTile* tile) const;
	
	/// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	///  @param[in]		tile			The tile.
	///  @param[out]	data			The buffer to store the tile's state in.
	///  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	/// @return The status flags for the operation.
	/// 
	/// 将指定瓦片的非结构性状态（如标识符、区域id等）存储在指定的缓冲区中
	dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;
	
	/// Restores the state of the tile.
	///  @param[in]	tile			The tile.
	///  @param[in]	data			The new state. (Obtained from #storeTileState.)
	///  @param[in]	maxDataSize		The size of the state within the data buffer.
	/// @return The status flags for the operation.
	/// 
	/// 恢复存储在缓冲区中的指定瓦片的非结构性状态，如标识符、区域id等。
	dtStatus restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize);
	
	/// @}

	/// @{
	/// @name Encoding and Decoding
	/// These functions are generally meant for internal use only.

	/// Derives a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	salt	The tile's salt value.
	///  @param[in]	it		The index of the tile.
	///  @param[in]	ip		The index of the polygon within the tile.
	/// 
	/// 对导航网格中的一个多边形，进行唯一性标识，通过位运算生成唯一id
	/// salt
	/// it 网格的索引
	/// ip 多边形在网格内部的索引
	inline dtPolyRef encodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		return ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
#else
		return ((dtPolyRef)salt << (m_polyBits+m_tileBits)) | ((dtPolyRef)it << m_polyBits) | (dtPolyRef)ip;
#endif
	}
	
	/// Decodes a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref   The polygon reference to decode.
	///  @param[out]	salt	The tile's salt value.
	///  @param[out]	it		The index of the tile.
	///  @param[out]	ip		The index of the polygon within the tile.
	///  @see #encodePolyId
	/// 
	/// 解码一个标准多边形的id
	/// 
	/// salt
	/// it 瓦片的索引
	/// ip 多边形在瓦片中的索引
	inline void decodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
		const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
		const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
		salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
		it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
		ip = (unsigned int)(ref & polyMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		salt = (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
		it = (unsigned int)((ref >> m_polyBits) & tileMask);
		ip = (unsigned int)(ref & polyMask);
#endif
	}

	/// Extracts a tile's salt value from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	/// 
	/// 从指定的多边形id中，解出网格的 salt 值
	inline unsigned int decodePolyIdSalt(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
		return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
#endif
	}
	
	/// Extracts the tile's index from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	/// 
	/// 从指定的多边形id中，解出网格的索引
	inline unsigned int decodePolyIdTile(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
		return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
#else
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		return (unsigned int)((ref >> m_polyBits) & tileMask);
#endif
	}
	
	/// Extracts the polygon's index (within its tile) from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	/// 
	/// 从指定的多边形id中，解码出其在瓦片中的索引
	inline unsigned int decodePolyIdPoly(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
		return (unsigned int)(ref & polyMask);
#else
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		return (unsigned int)(ref & polyMask);
#endif
	}

	/// @}
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNavMesh(const dtNavMesh&);
	dtNavMesh& operator=(const dtNavMesh&);

	/// Returns pointer to tile in the tile array.
	// 根据瓦片索引，从瓦片数组中返回瓦片指针
	dtMeshTile* getTile(int i);

	/// Returns neighbour tile based on side.
	// 返回基于边的邻接瓦片
	// x,y 是基于世界坐标系，计算出其在瓦片中的格子所属的行、列（即 x, y）。
	// tiles 用于保存结果集
	// maxTiles 最大存储记录
	// 返回值为实际存储在 tiles 的记录数量
	int getTilesAt(const int x, const int y,
				   dtMeshTile** tiles, const int maxTiles) const;

	/// Returns neighbour tile based on side.
	// 返回基于边的邻接瓦片，与 getTilesAt 不同的是，其返回指定边的邻接瓦片
	int getNeighbourTilesAt(const int x, const int y, const int side,
							dtMeshTile** tiles, const int maxTiles) const;
	
	/// Returns all polygons in neighbour tile based on portal defined by the segment.
	// 基于给定线段定义的门户，返回相邻瓦片中的所有多边形
	int findConnectingPolys(const float* va, const float* vb,
							const dtMeshTile* tile, int side,
							dtPolyRef* con, float* conarea, int maxcon) const;
	
	/// Builds internal polygons links for a tile.
	// 为一个网格构建内部多边形链接
	void connectIntLinks(dtMeshTile* tile);
	/// Builds internal polygons links for a tile.
	// 为一个网格构建内部多边形链接
	void baseOffMeshLinks(dtMeshTile* tile);

	/// Builds external polygon links for a tile.
	// 为一个网格建立外部链接多边形
	void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	/// Builds external polygon links for a tile.
	// 为一个网格建立外部链接多边形
	void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	
	/// Removes external links at specified side.
	// 删除指定一侧的外部链接
	void unconnectLinks(dtMeshTile* tile, dtMeshTile* target);
	

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.
	
	/// Queries polygons within a tile.
	// 查询网格中指定边界内的多边形
	// tile 查询的网格
	// qmin 最小边界
	// qmax 最大边界
	// polys 存储查询到的多边形id
	// maxPolys 查询的多边形数量上限
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							dtPolyRef* polys, const int maxPolys) const;
	/// Find nearest polygon within a tile.
	// 查找网格中与 nearestPt 最近的多边形
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
									const float* halfExtents, float* nearestPt) const;
	/// Returns whether position is over the poly and the height at the position if so.
	// 返回位置是否在多边形上方，如果是，则返回该位置的高度
	bool getPolyHeight(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* height) const;
	/// Returns closest point on polygon.
	// 返回多边形上最近的点
	void closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	// 生成多图块导航网格的配置参数
	dtNavMeshParams m_params;			///< Current initialization params. TODO: do not store this info twice.
	// 网格的原始坐标点
	float m_orig[3];					///< Origin of the tile (0,0)
	// 每一个网格的宽、高
	float m_tileWidth, m_tileHeight;	///< Dimensions of each tile.
	// 最大网格
	int m_maxTiles;						///< Max number of tiles.
	// 网格哈希查询表大小（必须是2的幂）
	int m_tileLutSize;					///< Tile hash lookup size (must be pot).
	// 网格标志位哈希查询表大小
	int m_tileLutMask;					///< Tile hash lookup mask.
	// 网格哈希查询表
	dtMeshTile** m_posLookup;			///< Tile hash lookup.
	// 未使用的网格列表
	dtMeshTile* m_nextFree;				///< Freelist of tiles.
	// 网格数组
	dtMeshTile* m_tiles;				///< List of tiles.
		
#ifndef DT_POLYREF64
	// 网格id中用于存储salt的比特位，即需要几位来存储salt
	unsigned int m_saltBits;			///< Number of salt bits in the tile ID.
	// 网格id中用于存储图块位数的比特位，即需要几位来存储图块的位数
	unsigned int m_tileBits;			///< Number of tile bits in the tile ID.
	// 网格id中用于存储多边形id的比特位，即需要几位来存储多边形的id
	unsigned int m_polyBits;			///< Number of poly bits in the tile ID.
#endif

	friend class dtNavMeshQuery;
};

/// Allocates a navigation mesh object using the Detour allocator.
/// @return A navigation mesh that is ready for initialization, or null on failure.
///  @ingroup detour
dtNavMesh* dtAllocNavMesh();

/// Frees the specified navigation mesh object using the Detour allocator.
///  @param[in]	navmesh		A navigation mesh allocated using #dtAllocNavMesh
///  @ingroup detour
void dtFreeNavMesh(dtNavMesh* navmesh);

#endif // DETOURNAVMESH_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@typedef dtPolyRef
@par

Polygon references are subject to the same invalidate/preserve/restore 
rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
tile changes, the polygon reference becomes invalid.

Changing a polygon's flags, area id, etc. does not impact its polygon
reference.

@typedef dtTileRef
@par

The following changes will invalidate a tile reference:

- The referenced tile has been removed from the navigation mesh.
- The navigation mesh has been initialized using a different set
  of #dtNavMeshParams.

A tile reference is preserved/restored if the tile is added to a navigation 
mesh initialized with the original #dtNavMeshParams and is added at the
original reference location. (E.g. The lastRef parameter is used with
dtNavMesh::addTile.)

Basically, if the storage structure of a tile changes, its associated
tile reference changes.


@var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
@par

Each entry represents data for the edge starting at the vertex of the same index. 
E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].

A value of zero indicates the edge has no polygon connection. (It makes up the 
border of the navigation mesh.)

The information can be extracted as follows: 
@code 
neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.

if (neis[n] & #DT_EX_LINK)
{
    // The edge is an external (portal) edge.
}
@endcode

@var float dtMeshHeader::bvQuantFactor
@par

This value is used for converting between world and bounding volume coordinates.
For example:
@code
const float cs = 1.0f / tile->header->bvQuantFactor;
const dtBVNode* n = &tile->bvTree[i];
if (n->i >= 0)
{
    // This is a leaf node.
    float worldMinX = tile->header->bmin[0] + n->bmin[0]*cs;
    float worldMinY = tile->header->bmin[0] + n->bmin[1]*cs;
    // Etc...
}
@endcode

@struct dtMeshTile
@par

Tiles generally only exist within the context of a dtNavMesh object.

Some tile content is optional.  For example, a tile may not contain any
off-mesh connections.  In this case the associated pointer will be null.

If a detail mesh exists it will share vertices with the base polygon mesh.  
Only the vertices unique to the detail mesh will be stored in #detailVerts.

@warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
For example: The tile at a location might not have been loaded yet, or may have been removed.
In this case, pointers will be null.  So if in doubt, check the polygon count in the 
tile's header to determine if a tile has polygons defined.

@var float dtOffMeshConnection::pos[6]
@par

For a properly built navigation mesh, vertex A will always be within the bounds of the mesh. 
Vertex B is not required to be within the bounds of the mesh.

*/
