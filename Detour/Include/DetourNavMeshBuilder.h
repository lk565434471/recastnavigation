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

#ifndef DETOURNAVMESHBUILDER_H
#define DETOURNAVMESHBUILDER_H

#include "DetourAlloc.h"

/// Represents the source data used to build an navigation mesh tile.
/// @ingroup detour
struct dtNavMeshCreateParams
{

	/// @name Polygon Mesh Attributes
	/// Used to create the base navigation graph.
	/// See #rcPolyMesh for details related to these attributes.
	/// @{

	// 多边形网格顶点
	const unsigned short* verts;			///< The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	// 多边形网格中的顶点数量
	int vertCount;							///< The number vertices in the polygon mesh. [Limit: >= 3]
	// 多边形数据
	const unsigned short* polys;			///< The polygon data. [Size: #polyCount * 2 * #nvp]
	// 用户分配给每个多边形的标志
	const unsigned short* polyFlags;		///< The user defined flags assigned to each polygon. [Size: #polyCount]
	// 用户分配给每个多边形的区域id
	const unsigned char* polyAreas;			///< The user defined area ids assigned to each polygon. [Size: #polyCount]
	// 在导航网格中的多边形数量
	int polyCount;							///< Number of polygons in the mesh. [Limit: >= 1]
	// 每个多边形最多包含的顶点数量
	int nvp;								///< Number maximum number of vertices per polygon. [Limit: >= 3]

	/// @}
	/// @name Height Detail Attributes (Optional)
	/// See #rcPolyMeshDetail for details related to these attributes.
	/// @{

	// 每个子网格的高度详细数据
	const unsigned int* detailMeshes;		///< The height detail sub-mesh data. [Size: 4 * #polyCount]
	// 详细的导航网格顶点信息
	const float* detailVerts;				///< The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	// 高精度网格中的顶点数量
	int detailVertsCount;					///< The number of vertices in the detail mesh.
	// 高精度网格三角形
	const unsigned char* detailTris;		///< The detail mesh triangles. [Size: 4 * #detailTriCount]
	// 在高精度网格中的三角形数量
	int detailTriCount;						///< The number of triangles in the detail mesh.

	/// @}
	/// @name Off-Mesh Connections Attributes (Optional)
	/// Used to define a custom point-to-point edge within the navigation graph, an 
	/// off-mesh connection is a user defined traversable connection made up to two vertices, 
	/// at least one of which resides within a navigation mesh polygon.
	/// @{

	/// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	// 顶点连接线的起始和终点位置
	const float* offMeshConVerts;
	/// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	// 连接线半径
	const float* offMeshConRad;
	/// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	// 用户分配给连接线的标志
	const unsigned short* offMeshConFlags;
	/// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	// 用户分配给连接线的区域id
	const unsigned char* offMeshConAreas;
	/// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	///
	/// 0 = Travel only from endpoint A to endpoint B.<br/>
	/// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	const unsigned char* offMeshConDir;	
	/// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	// 用户分配给连接线的id
	const unsigned int* offMeshConUserID;
	/// The number of off-mesh connections. [Limit: >= 0]
	// 连接线的数量
	int offMeshConCount;

	/// @}
	/// @name Tile Attributes
	/// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
	/// @{

	// 用户定义的网格（瓦片）id
	unsigned int userId;	///< The user defined id of the tile.
	// 多网格目标网格：指的是由一些列网格组成的网格集合
	// 在多网格目标网格中，该网格在 x 轴网格中的位置
	int tileX;				///< The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	// 在多网格目标网格中，该网格在 z 轴网格中的位置
	int tileY;				///< The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
	// 在分层目标网格中，该网格所在的层
	int tileLayer;			///< The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	// 一个三维的长方体，最小、最大检测边界
	// 通常用于碰撞检测、包围盒剔除等操作
	float bmin[3];			///< The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	float bmax[3];			///< The maximum bounds of the tile. [(x, y, z)] [Unit: wu]

	/// @}
	/// @name General Configuration Attributes
	/// @{

	float walkableHeight;	///< The agent height. [Unit: wu]
	float walkableRadius;	///< The agent radius. [Unit: wu]
	float walkableClimb;	///< The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	// 多边形网格的 xz 平面单元格尺寸
	float cs;				///< The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	// 多边形网格的 y 平面单元格尺寸
	float ch;				///< The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]

	/// True if a bounding volume tree should be built for the tile.
	/// @note The BVTree is not normally needed for layered navigation meshes.
	// 表示是否需要构建边界体积树
	bool buildBvTree;

	/// @}
};

/// Builds navigation mesh tile data from the provided tile creation data.
/// @ingroup detour
///  @param[in]		params		Tile creation data.
///  @param[out]	outData		The resulting tile data.
///  @param[out]	outDataSize	The size of the tile data array.
/// @return True if the tile data was successfully created.
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);

/// Swaps the endianess of the tile data's header (#dtMeshHeader).
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int dataSize);

/// Swaps endianess of the tile data.
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int dataSize);

#endif // DETOURNAVMESHBUILDER_H

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@struct dtNavMeshCreateParams
@par

This structure is used to marshal data between the Recast mesh generation pipeline and Detour navigation components.

See the rcPolyMesh and rcPolyMeshDetail documentation for detailed information related to mesh structure.

Units are usually in voxels (vx) or world units (wu). The units for voxels, grid size, and cell size 
are all based on the values of #cs and #ch.

The standard navigation mesh build process is to create tile data using dtCreateNavMeshData, then add the tile 
to a navigation mesh using either the dtNavMesh single tile <tt>init()</tt> function or the dtNavMesh::addTile()
function.

@see dtCreateNavMeshData

*/

