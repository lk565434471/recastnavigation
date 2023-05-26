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
/// ÿ��������������Ķ�����������
static const int DT_VERTS_PER_POLYGON = 6;

/// @{
/// @name Tile Serialization Constants
/// These constants are used to detect whether a navigation tile's data
/// and state format is compatible with the current build.
///

/// A magic number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V';

/// A version number used to detect compatibility of navigation tile data.
// ���ڼ�⵼���������ݼ����Եİ汾��
static const int DT_NAVMESH_VERSION = 7;

/// A magic number used to detect the compatibility of navigation tile states.
// ���ڼ�⵼������״̬�����Ե�ħ��
static const int DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S';

/// A version number used to detect compatibility of navigation tile states.
// ���ڼ�⵼������״̬�����Եİ汾��
static const int DT_NAVMESH_STATE_VERSION = 1;

/// @}

/// A flag that indicates that an entity links to an external entity.
/// (E.g. A polygon edge is a portal that links to another polygon.)
/// 
/// һ����ʶָʾʵ���Ƿ����ӵ�һ���ⲿʵ��
/// һ������εı���ָ���ӵ�����һ������ε��Ż����Ż�ָ������Ѱ·�У���һ����
/// �����ƶ�������һ������ε������ߣ�
static const unsigned short DT_EXT_LINK = 0x8000;

/// A value that indicates the entity does not link to anything.
/// һ��ֵָʾʵ��û�����ӵ��κ�����ʵ��
static const unsigned int DT_NULL_LINK = 0xffffffff;

/// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
/// һ����ʶָʾ������һ��˫�򵼺�ʱ��Խһ���������ӡ���ɢ����ͨ�����ڿ�Խ�����ڵ����񣬲���Ҫ����
/// Ѱ·�����·���ߣ����磬MMO �д���������ȥ��ֱ�ӵ���һ�����뵱ǰ�����Զ�������ϡ�
static const unsigned int DT_OFFMESH_CON_BIDIR = 1;

/// The maximum number of user defined area ids.
/// @ingroup detour
/// 
/// �û���������id�������������
static const int DT_MAX_AREAS = 64;

/// Tile flags used for various functions and fields.
/// For an example, see dtNavMesh::addTile().
/// 
/// ��Ƭ��ʶ���ڸ��ֺ������ֶ�
enum dtTileFlags
{
	/// The navigation mesh owns the tile memory and is responsible for freeing it.
	// �������������Ƭ�ڴ棬�������ͷ�����
	DT_TILE_FREE_DATA = 0x01
};

/// Vertex flags returned by dtNavMeshQuery::findStraightPath.
/// ͨ�� dtNavMeshQuery::findStraightPath �������ض����ʶ
enum dtStraightPathFlags
{
	// ���������·������ʼλ��
	DT_STRAIGHTPATH_START = 0x01,				///< The vertex is the start position in the path.
	// ���������·������ֹλ��
	DT_STRAIGHTPATH_END = 0x02,					///< The vertex is the end position in the path.
	// ���������һ����ɢ���ӵ����
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04	///< The vertex is the start of an off-mesh connection.
};

/// Options for dtNavMeshQuery::findStraightPath.
/// ���� dtNavMeshQuery::findStraightPath ��ѡ�����
enum dtStraightPathOptions
{
	// ��ÿ������α�Ե�����λ������һ�����㣬�Ա�������仯ʱ���б�ʶ
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,	///< Add a vertex at every polygon edge crossing where area changes.
	// ��ÿ������α߽罻���λ������һ������
	DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02	///< Add a vertex at every polygon edge crossing.
};


/// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
/// ���� dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath ������ѡ�����
enum dtFindPathOptions
{
	// ��·���滮�����У�ʹ������Ͷ�������п��ٲ��ң�����Ͷ����Ȼ��Ҫ����·���ϵĳɱ����أ�
	DT_FINDPATH_ANY_ANGLE	= 0x02		///< use raycasts during pathfind to "shortcut" (raycast still consider costs)
};

/// Options for dtNavMeshQuery::raycast
enum dtRaycastOptions
{
	// ����Ͷ��Ӧ������������·�����ƶ��ɱ�������� RaycastHit::cost
	DT_RAYCAST_USE_COSTS = 0x01		///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
};

enum dtDetailTriEdgeFlags
{
	// �ֲ������εı��Ƕ���α߽��һ����
	DT_DETAIL_EDGE_BOUNDARY = 0x01		///< Detail triangle edge is part of the poly boundary
};


/// Limit raycasting during any angle pahfinding
/// The limit is given as a multiple of the character radius
/// 
/// ������������ǶȽ��е�·�������ڼ������Ͷ��
/// ������Ƕ�Ѱ·ʱ����������Ͷ��ľ����ǽ�ɫ�뾶�ı���
static const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

/// Flags representing the type of a navigation mesh polygon.
// �����������ε����ͱ�־
enum dtPolyTypes
{
	/// The polygon is a standard convex polygon that is part of the surface of the mesh.
	// �ö�������������ı�׼͹����Ρ���ÿ���߶���һ��ֱ�ߡ�
	DT_POLYTYPE_GROUND = 0,
	/// The polygon is an off-mesh connection consisting of two vertices.
	// �ö���������������㹹�ɵ��������ӣ���������������һ���֣������������������ڵ�����
	DT_POLYTYPE_OFFMESH_CONNECTION = 1
};


/// Defines a polygon within a dtMeshTile object.
/// @ingroup detour
/// 
/// ������һ�� dtMeshTile �����еĶ����
struct dtPoly
{
	/// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	// �������б��У���һ�����ӵ�����
	unsigned int firstLink;

	/// The indices of the polygon's vertices.
	/// The actual vertices are located in dtMeshTile::verts.
	/// ��ʾ�ö���εĶ���
	/// ʵ���ϣ�verts �д洢�Ƕ���εĶ�����������ʵ�Ķ�������λ�� dtMeshTile::verts �С�
	unsigned short verts[DT_VERTS_PER_POLYGON];

	/// Packed data representing neighbor polygons references and flags for each edge.
	// ������������ڹ�ϵ��ѹ�����ݵ����顣
	// ÿ��������������ߣ�����������ÿ����Ԫ�ر�ʾ���ڶ���ε����ú����ڹ�ϵ��־
	unsigned short neis[DT_VERTS_PER_POLYGON];

	/// The user defined polygon flags.
	// �û�����Ķ���α�־
	unsigned short flags;

	/// The number of vertices in the polygon.
	// ������ж��������
	unsigned char vertCount;

	/// The bit packed area id and polygon type.
	/// @note Use the structure's set and get methods to acess this value.
	/// λ��װ������id�Ͷ��������
	/// ����id < DT_MAX_AREAS, ��6λ��������id
	/// ��������� enum dtPolyTypes����2λ��������id
	unsigned char areaAndtype;

	/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
	// ��6λ�洢����id��id �ķ�Χ�� [0, 63] ֮��
	inline void setArea(unsigned char a) { areaAndtype = (areaAndtype & 0xc0) | (a & 0x3f); }

	/// Sets the polygon type. (See: #dtPolyTypes.)
	// ��2λ�洢��������ͣ���Χ [0, 3] ֮��
	inline void setType(unsigned char t) { areaAndtype = (areaAndtype & 0x3f) | (t << 6); }

	/// Gets the user defined area id.
	inline unsigned char getArea() const { return areaAndtype & 0x3f; }

	/// Gets the polygon type. (See: #dtPolyTypes)
	inline unsigned char getType() const { return areaAndtype >> 6; }
};

/// Defines the location of detail sub-mesh data within a dtMeshTile.
// dtPolyDetail �����˶���ε�ϸ����Ϣ�������� dtMeshTile �е�λ�á�
struct dtPolyDetail
{
	// �� dtMeshTile::detailVerts �����еĶ���ƫ��
	unsigned int vertBase;			///< The offset of the vertices in the dtMeshTile::detailVerts array.
	// �� dtMeshTile::detailTris �����е�������ƫ��
	unsigned int triBase;			///< The offset of the triangles in the dtMeshTile::detailTris array.
	// ������Ķ�������
	unsigned char vertCount;		///< The number of vertices in the sub-mesh.
	// �����������������
	unsigned char triCount;			///< The number of triangles in the sub-mesh.
};

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
/// 
/// �������������֮�������
struct dtLink
{
	// �ھӵ����ã�ָ�������ӵĶ����
	dtPolyRef ref;					///< Neighbour reference. (The neighbor that is linked to.)
	// ��һ�����ӵ�����
	unsigned int next;				///< Index of the next link.
	// ӵ�д����ӵĶ���α�Ե�����������ֵ��ʵ���Ƕ���������±꣬������ע�ʹ�����Щ���Ի�
	unsigned char edge;				///< Index of the polygon edge that owns this link.
	// �����һ���߽����ӣ�������������һ��
	unsigned char side;				///< If a boundary link, defines on which side the link is.
	// �����һ���߽����ӣ�������С���ӱ�Ե���
	unsigned char bmin;				///< If a boundary link, defines the minimum sub-edge area.
	// �����һ���߽����ӣ����������ӱ�Ե���
	unsigned char bmax;				///< If a boundary link, defines the maximum sub-edge area.
};

/// Bounding volume node.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
/// 
/// ��Χ��ڵ㣬��Ҫ���ڽ��е�������Ĺ������Ż���
struct dtBVNode
{
	// AABB ��Χ�е���С�߽�
	unsigned short bmin[3];			///< Minimum bounds of the node's AABB. [(x, y, z)]
	// AABB ��Χ�е����߽�
	unsigned short bmax[3];			///< Maximum bounds of the node's AABB. [(x, y, z)]
	// �ڵ����������ֵ��ʾ�ýڵ���һ�����ݽڵ�
	int i;							///< The node's index. (Negative for escape sequence.)
};

/// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
/// An off-mesh connection is a user defined traversable connection made up to two vertices.
/// 
/// dtMeshTile �����ж��嵼���������ɢ����
/// һ����ɢ�������û�����Ŀɱ������ӣ�������������ɡ�
struct dtOffMeshConnection
{
	/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	// �洢��ɢ���ӵ������յ�����
	float pos[6];

	/// The radius of the endpoints. [Limit: >= 0]
	// ��ʾ�����յ�İ뾶�����ڱ�ʾ�ڴ��������ƶ��Ŀռ����󡣴�ֵ������ڵ�����
	float rad;		

	/// The polygon reference of the connection within the tile.
	// �����ڲ����ӵĶ����id
	unsigned short poly;

	/// Link flags. 
	/// @note These are not the connection's user defined flags. Those are assigned via the 
	/// connection's dtPoly definition. These are link flags used for internal purposes.
	/// �����ڲ�Ŀ�ĵ����ӱ�־����Щ��־ͨ�����ӵ� dtPoly ������䡣
	unsigned char flags;

	/// End point side.
	// �յ����ڵĲ���
	unsigned char side;

	/// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	// ��ɢ���ӵ�id���ڵ������񹹽�ʱ�����û�����ġ�
	unsigned int userId;
};

/// Provides high level information related to a dtMeshTile object.
/// @ingroup detour
/// 
/// �ṩ�� dtMeshTile ������صĸ߼���Ϣ
struct dtMeshHeader
{
	// ������Ƭ��ħ�������ڶ������ݵĸ�ʽ��ͨ�����ڽ����ļ����Դ���ȷ���ļ������͸�ʽ��
	int magic;				///< Tile magic number. (Used to identify the data format.)
	// ������Ƭ�����ݸ�ʽ�İ汾�ţ�ÿ����������ʱ��ͨ����������ļ���ʽ��Ϣ����ɾ��
	// �汾������ȷ�������ļ�ʱ�����ִ�����ͬ���ļ��汾��ʽ��
	int version;			///< Tile data format version number.
	// ������Ƭ���� dtNavMesh �еĿռ����� x ������
	int x;					///< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// ������Ƭ���� dtNavMesh �еĿռ����� y ������
	int y;					///< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// ������Ƭ���� dtNavMesh �еĿռ����������Ĳ㼶
	int layer;				///< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	// �û������������Ƭ��id����ţ�
	unsigned int userId;	///< The user defined id of the tile.
	// ������Ƭ���а����Ķ��������
	int polyCount;			///< The number of polygons in the tile.
	// ������Ƭ���а����Ķ�������
	int vertCount;			///< The number of vertices in the tile.
	// ������Ƭ�����������������
	int maxLinkCount;		///< The number of allocated links.
	// ϸ�������е�����������
	int detailMeshCount;	///< The number of sub-meshes in the detail mesh.
	
	/// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	// ���˶���ζ������⣬��ϸ�������ж�һ�޶��Ķ�������
	int detailVertCount;
	
	// ��ϸ�������а���������������
	int detailTriCount;			///< The number of triangles in the detail mesh.
	// �߽�����ڵ�����������δ���ñ߽��������Ϊ��
	int bvNodeCount;			///< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	// ��ɢ�����ߵ�����
	int offMeshConCount;		///< The number of off-mesh connections.
	// ��һ����ɢ���Ӷ���ε�����
	int offMeshBase;			///< The index of the first polygon which is an off-mesh connection.
	// Agent����������������Ƭ����ʹ�õĸ߶���Ϣ��������ײ��⡢���ϵȹ���
	float walkableHeight;		///< The height of the agents using the tile.
	// Agent����������������Ƭ����ʹ�õİ뾶��Ϣ��������ײ��⡢���ϵȹ���
	float walkableRadius;		///< The radius of the agents using the tile.
	// Agent����������������Ƭ����ʹ�õ���������߶���Ϣ��������ײ��⡢���ϵȹ���
	float walkableClimb;		///< The maximum climb height of the agents using the tile.
	// Agent����������������Ƭ�������� AABB ��ײ������С�߽�
	float bmin[3];				///< The minimum bounds of the tile's AABB. [(x, y, z)]
	// Agent����������������Ƭ�������� AABB ��ײ�������߽�
	float bmax[3];				///< The maximum bounds of the tile's AABB. [(x, y, z)]
	
	/// The bounding volume quantization factor. 
	// �߽������������
	float bvQuantFactor;
};

/// Defines a navigation mesh tile.
/// @ingroup detour
/// 
/// ���嵼��������Ƭ��
struct dtMeshTile
{
	// ������������Ƭ�������޸ĵļ�����
	unsigned int salt;					///< Counter describing modifications to the tile.

	// ��һ����������������
	unsigned int linksFreeList;			///< Index to the next free link.
	// ������Ƭ����ͷ��Ϣ
	dtMeshHeader* header;				///< The tile header.
	// ������Ƭ���ڵĶ��������
	dtPoly* polys;						///< The tile polygons. [Size: dtMeshHeader::polyCount]
	// ������Ƭ���Ķ������� [(x, y, z) * dtMeshHeader::vertCount]
	float* verts;						///< The tile vertices. [(x, y, z) * dtMeshHeader::vertCount]
	// ������Ƭ������������ [(dtMeshHeader) * dtMeshHeader::maxLinkCount]
	dtLink* links;						///< The tile links. [Size: dtMeshHeader::maxLinkCount]
	// ������Ƭ����ϸ��������Ƭ������ [(dtMeshHeader) * dtMeshHeader::detailMeshCount]
	dtPolyDetail* detailMeshes;			///< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	
	/// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// ϸ�������Ψһ�������� [(x, y, z) * dtMeshHeader::detailVertCount]
	float* detailVerts;	

	/// The detail mesh's triangles. [(vertA, vertB, vertC, triFlags) * dtMeshHeader::detailTriCount].
	/// See dtDetailTriEdgeFlags and dtGetDetailTriEdgeFlags.
	/// 
	/// ϸ����������������飬[((vertA, vertB, vertC, triFlags) * dtMeshHeader::detailTriCount]
	unsigned char* detailTris;	

	/// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	/// (Will be null if bounding volumes are disabled.)
	/// 
	/// ������Ƭ���ı߽�����ڵ�
	dtBVNode* bvTree;

	// ������Ƭ������ɢ������
	dtOffMeshConnection* offMeshCons;		///< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	
	// ������Ƭ���ĵײ����ݣ�ͨ�����ᱻֱ�ӷ���
	unsigned char* data;					///< The tile data. (Not directly accessed under normal situations.)
	// ������Ƭ�����ݵĴ�С�������������ȣ�
	int dataSize;							///< Size of the tile data.
	// ������Ƭ���ı�־λ
	int flags;								///< Tile flags. (See: #dtTileFlags)
	// ��һ������������Ƭ������������һ���ڿռ������е�������Ƭ��
	// ͨ��ͨ�� (x, y, z) ���������㣬����ȡ��λ���ϵ�����
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
/// ��þֲ������εıߵı�ʶ
inline int dtGetDetailTriEdgeFlags(unsigned char triFlags, int edgeIndex)
{
	return (triFlags >> (edgeIndex * 2)) & 0x3;
}

/// Configuration parameters used to define multi-tile navigation meshes.
/// The values are used to allocate space during the initialization of a navigation mesh.
/// @see dtNavMesh::init()
/// @ingroup detour
/// 
/// ��ͼ�鵼����������ò�������ͼ���ǽ��������񻮷�λ���ͼ�飬��������������ĵ�ͼ��
struct dtNavMeshParams
{
	// �����������ڵ�����ռ��ԭ���ڵ��������ͼ��ռ��е�λ��
	// ��������ǣ�����ռ�ͱ��ؿռ��ӳ��
	float orig[3];					///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	// ÿ������Ŀ��ȣ��� x ��
	float tileWidth;				///< The width of each tile. (Along the x-axis.)
	// ÿ������ĸ߶ȣ��� z ��
	float tileHeight;				///< The height of each tile. (Along the z-axis.)
	// ����������԰�������������������ޣ����ֵ��maxPolysһ�����ڼ�����Ҫ����λ����Ψһ�ر�ʶÿ��ͼ��Ͷ����
	int maxTiles;					///< The maximum number of tiles the navigation mesh can contain. This and maxPolys are used to calculate how many bits are needed to identify tiles and polygons uniquely.
	// ÿ��������԰�������������������ޣ����ֵ��maxTilesһ�����ڼ�����Ҫ����λ����Ψһ�ر�ʶÿ��ͼ��Ͷ����
	int maxPolys;					///< The maximum number of polygons each tile can contain. This and maxTiles are used to calculate how many bits are needed to identify tiles and polygons uniquely.
};

/// A navigation mesh based on tiles of convex polygons.
/// @ingroup detour
/// 
/// ����͹����εĵ�������
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
	/// ��ʼ�����������
	dtStatus init(const dtNavMeshParams* params);

	/// Initializes the navigation mesh for single tile use.
	///  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	///  @param[in]	dataSize	The data size of the new tile.
	///  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	/// @return The status flags for the operation.
	///  @see dtCreateNavMeshData
	/// �Զ��������ݸ�ʽ��ʼ��
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
	/// ����һ�����񵽵�������
	/// data �����������
	/// dataSize ������������ݵĳ���
	/// flags �����־λ��Ŀǰ����һ��ѡ���ʾ�Ƿ����Լ�ά���ڴ棬���ͷš�
	/// lastRef ������ֵ����ʾ���Դ�δʹ�õ���������л�ȡ������Ҫ���´��� dtTileRef ����
	/// result �����������ã�������ӳɹ�����ͨ������������ new ������ dtTileRef ����
	dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);
	
	/// Removes the specified tile from the navigation mesh.
	///  @param[in]		ref			The reference of the tile to remove.
	///  @param[out]	data		Data associated with deleted tile.
	///  @param[out]	dataSize	Size of the data associated with deleted tile.
	/// @return The status flags for the operation.
	/// 
	/// �ӵ����������Ƴ�ָ��������
	dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	/// @}

	/// @{
	/// @name Query Functions

	/// Calculates the tile grid location for the specified world position.
	///  @param[in]	pos  The world position for the query. [(x, y, z)]
	///  @param[out]	tx		The tile's x-location. (x, y)
	///  @param[out]	ty		The tile's y-location. (x, y)
	/// 
	/// ����ָ������λ�õ���Ƭ����
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	/// Gets the tile at the specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile, or null if the tile does not exist.
	/// 
	/// ��ȡָ���㼶�ϵ�����
	const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	/// Gets all tiles at the specified grid location. (All layers.)
	///  @param[in]		x			The tile's x-location. (x, y)
	///  @param[in]		y			The tile's y-location. (x, y)
	///  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	///  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	/// @return The number of tiles returned in the tiles array.
	/// 
	/// ��ȡ��ָ������λ�õ�������Ƭ�����������в㼶��
	int getTilesAt(const int x, const int y,
				   dtMeshTile const** tiles, const int maxTiles) const;
	
	/// Gets the tile reference for the tile at specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile reference of the tile, or 0 if there is none.
	/// 
	/// ��ȡ��ָ������λ�õ���Ƭid 
	dtTileRef getTileRefAt(int x, int y, int layer) const;

	/// Gets the tile reference for the specified tile.
	///  @param[in]	tile	The tile.
	/// @return The tile reference of the tile.
	/// 
	/// ��ȡָ����Ƭ��id
	dtTileRef getTileRef(const dtMeshTile* tile) const;

	/// Gets the tile for the specified tile reference.
	///  @param[in]	ref		The tile reference of the tile to retrieve.
	/// @return The tile for the specified reference, or null if the 
	///		reference is invalid.
	/// 
	/// ͨ��ָ������Ƭid����ȡ��Ƭ����
	const dtMeshTile* getTileByRef(dtTileRef ref) const;
	
	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	/// 
	/// ��ȡ��������֧�ֵ������Ƭ��������
	int getMaxTiles() const;
	
	/// Gets the tile at the specified index.
	///  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	/// @return The tile at the specified index.
	/// 
	/// ��ȡ��������λ���ϵ���Ƭ����
	const dtMeshTile* getTile(int i) const;

	/// Gets the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		The reference for the a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// @return The status flags for the operation.
	/// 
	/// ͨ��ָ���Ķ����id����ȡ��Ƭ�Ͷ���ζ���
	/// polygon reference ��һ���������͵�id
	dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;
	
	/// Returns the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		A known valid reference for a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// 
	/// ͨ��ָ���Ķ����id����ȡ��Ƭ�Ͷ���ζ���
	/// �� getTileAndPolyByRef ��ͬ��ʱ�������Խ��������ݽ���У�飬��˿��ܵ��³������
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Checks the validity of a polygon reference.
	///  @param[in]	ref		The polygon reference to check.
	/// @return True if polygon reference is valid for the navigation mesh.
	/// 
	/// ���һ�������id�ĺϷ���
	bool isValidPolyRef(dtPolyRef ref) const;
	
	/// Gets the polygon reference for the tile's base polygon.
	///  @param[in]	tile		The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	/// 
	/// ��ȡ����Ļ�������εĶ����id�����������ָ����ÿ������ĵ�һ�������
	dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;
	
	/// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	///  @param[in]		prevRef		The reference of the polygon before the connection.
	///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	/// @return The status flags for the operation.
	/// 
	/// ��ȡ�������ӵĶ˵㣬�� ���н���������
	/// ���н����� ָ��������һ���˵㵽��һ���˵㡣
	dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;

	/// Gets the specified off-mesh connection.
	///  @param[in]	ref		The polygon reference of the off-mesh connection.
	/// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	/// 
	/// ��ȡָ������ɢ����
	/// ref �����id��ͨ��������н��룬��ȡ��ɢ����
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
	/// ��ָ���Ķ���α�ʶ�������ó��û�����ı�ʶ��
	dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	/// Gets the user defined flags for the specified polygon.
	///  @param[in]		ref				The polygon reference.
	///  @param[out]	resultFlags		The polygon flags.
	/// @return The status flags for the operation.
	/// 
	/// ��ȡ����εı�ʶ��
	dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;

	/// Sets the user defined area for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	/// @return The status flags for the operation.
	/// 
	/// ��ָ���Ķ��������id�����ó��û����������
	dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	/// Gets the user defined area for the specified polygon.
	///  @param[in]		ref			The polygon reference.
	///  @param[out]	resultArea	The area id for the polygon.
	/// @return The status flags for the operation.
	/// 
	/// ��ȡָ������ε��û���������
	dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;

	/// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	///  @param[in]	tile	The tile.
	/// @return The size of the buffer required to store the state.
	/// 
	/// ��ȡ�洢ָ����Ƭ״̬����Ļ�������С
	int getTileStateSize(const dtMeshTile* tile) const;
	
	/// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	///  @param[in]		tile			The tile.
	///  @param[out]	data			The buffer to store the tile's state in.
	///  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	/// @return The status flags for the operation.
	/// 
	/// ��ָ����Ƭ�ķǽṹ��״̬�����ʶ��������id�ȣ��洢��ָ���Ļ�������
	dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;
	
	/// Restores the state of the tile.
	///  @param[in]	tile			The tile.
	///  @param[in]	data			The new state. (Obtained from #storeTileState.)
	///  @param[in]	maxDataSize		The size of the state within the data buffer.
	/// @return The status flags for the operation.
	/// 
	/// �ָ��洢�ڻ������е�ָ����Ƭ�ķǽṹ��״̬�����ʶ��������id�ȡ�
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
	/// �Ե��������е�һ������Σ�����Ψһ�Ա�ʶ��ͨ��λ��������Ψһid
	/// salt
	/// it ���������
	/// ip ������������ڲ�������
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
	/// ����һ����׼����ε�id
	/// 
	/// salt
	/// it ��Ƭ������
	/// ip ���������Ƭ�е�����
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
	/// ��ָ���Ķ����id�У��������� salt ֵ
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
	/// ��ָ���Ķ����id�У�������������
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
	/// ��ָ���Ķ����id�У������������Ƭ�е�����
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
	// ������Ƭ����������Ƭ�����з�����Ƭָ��
	dtMeshTile* getTile(int i);

	/// Returns neighbour tile based on side.
	// ���ػ��ڱߵ��ڽ���Ƭ
	// x,y �ǻ�����������ϵ�������������Ƭ�еĸ����������С��У��� x, y����
	// tiles ���ڱ�������
	// maxTiles ���洢��¼
	// ����ֵΪʵ�ʴ洢�� tiles �ļ�¼����
	int getTilesAt(const int x, const int y,
				   dtMeshTile** tiles, const int maxTiles) const;

	/// Returns neighbour tile based on side.
	// ���ػ��ڱߵ��ڽ���Ƭ���� getTilesAt ��ͬ���ǣ��䷵��ָ���ߵ��ڽ���Ƭ
	int getNeighbourTilesAt(const int x, const int y, const int side,
							dtMeshTile** tiles, const int maxTiles) const;
	
	/// Returns all polygons in neighbour tile based on portal defined by the segment.
	// ���ڸ����߶ζ�����Ż�������������Ƭ�е����ж����
	int findConnectingPolys(const float* va, const float* vb,
							const dtMeshTile* tile, int side,
							dtPolyRef* con, float* conarea, int maxcon) const;
	
	/// Builds internal polygons links for a tile.
	// Ϊһ�����񹹽��ڲ����������
	void connectIntLinks(dtMeshTile* tile);
	/// Builds internal polygons links for a tile.
	// Ϊһ�����񹹽��ڲ����������
	void baseOffMeshLinks(dtMeshTile* tile);

	/// Builds external polygon links for a tile.
	// Ϊһ���������ⲿ���Ӷ����
	void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	/// Builds external polygon links for a tile.
	// Ϊһ���������ⲿ���Ӷ����
	void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	
	/// Removes external links at specified side.
	// ɾ��ָ��һ����ⲿ����
	void unconnectLinks(dtMeshTile* tile, dtMeshTile* target);
	

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.
	
	/// Queries polygons within a tile.
	// ��ѯ������ָ���߽��ڵĶ����
	// tile ��ѯ������
	// qmin ��С�߽�
	// qmax ���߽�
	// polys �洢��ѯ���Ķ����id
	// maxPolys ��ѯ�Ķ������������
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							dtPolyRef* polys, const int maxPolys) const;
	/// Find nearest polygon within a tile.
	// ������������ nearestPt ����Ķ����
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
									const float* halfExtents, float* nearestPt) const;
	/// Returns whether position is over the poly and the height at the position if so.
	// ����λ���Ƿ��ڶ�����Ϸ�������ǣ��򷵻ظ�λ�õĸ߶�
	bool getPolyHeight(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* height) const;
	/// Returns closest point on polygon.
	// ���ض����������ĵ�
	void closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	// ���ɶ�ͼ�鵼����������ò���
	dtNavMeshParams m_params;			///< Current initialization params. TODO: do not store this info twice.
	// �����ԭʼ�����
	float m_orig[3];					///< Origin of the tile (0,0)
	// ÿһ������Ŀ�����
	float m_tileWidth, m_tileHeight;	///< Dimensions of each tile.
	// �������
	int m_maxTiles;						///< Max number of tiles.
	// �����ϣ��ѯ����С��������2���ݣ�
	int m_tileLutSize;					///< Tile hash lookup size (must be pot).
	// �����־λ��ϣ��ѯ����С
	int m_tileLutMask;					///< Tile hash lookup mask.
	// �����ϣ��ѯ��
	dtMeshTile** m_posLookup;			///< Tile hash lookup.
	// δʹ�õ������б�
	dtMeshTile* m_nextFree;				///< Freelist of tiles.
	// ��������
	dtMeshTile* m_tiles;				///< List of tiles.
		
#ifndef DT_POLYREF64
	// ����id�����ڴ洢salt�ı���λ������Ҫ��λ���洢salt
	unsigned int m_saltBits;			///< Number of salt bits in the tile ID.
	// ����id�����ڴ洢ͼ��λ���ı���λ������Ҫ��λ���洢ͼ���λ��
	unsigned int m_tileBits;			///< Number of tile bits in the tile ID.
	// ����id�����ڴ洢�����id�ı���λ������Ҫ��λ���洢����ε�id
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