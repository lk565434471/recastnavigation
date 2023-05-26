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

#ifndef DETOURNAVMESHQUERY_H
#define DETOURNAVMESHQUERY_H

#include "DetourNavMesh.h"
#include "DetourStatus.h"


// Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from dtQueryFilter.
// On certain platforms indirect or virtual function call is expensive. The default
// setting is to use non-virtual functions, the actual implementations of the functions
// are declared as inline for maximum speed. 

//#define DT_VIRTUAL_QUERYFILTER 1

/// Defines polygon filtering and traversal costs for navigation mesh query operations.
/// @ingroup detour
/// 
/// 网格查询过滤器
class dtQueryFilter
{
	// 每种区域类型的代价
	float m_areaCost[DT_MAX_AREAS];		///< Cost per area type. (Used by default implementation.)
	// 表示可访问的多边形标识符
	unsigned short m_includeFlags;		///< Flags for polygons that can be visited. (Used by default implementation.)
	// 表示不可访问的多边形标识符
	unsigned short m_excludeFlags;		///< Flags for polygons that should not be visted. (Used by default implementation.)
	
public:
	dtQueryFilter();
	
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual ~dtQueryFilter() { }
#endif
	
	/// Returns true if the polygon can be visited.  (I.e. Is traversable.)
	///  @param[in]		ref		The reference id of the polygon test.
	///  @param[in]		tile	The tile containing the polygon.
	///  @param[in]		poly  The polygon to test.
	/// 
	/// 如果多边形可以被访问，则返回true，否则返回false
	/// ref 多边形id
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual bool passFilter(const dtPolyRef ref,
							const dtMeshTile* tile,
							const dtPoly* poly) const;
#else
	bool passFilter(const dtPolyRef ref,
					const dtMeshTile* tile,
					const dtPoly* poly) const;
#endif

	/// Returns cost to move from the beginning to the end of a line segment
	/// that is fully contained within a polygon.
	///  @param[in]		pa			The start position on the edge of the previous and current polygon. [(x, y, z)]
	///  @param[in]		pb			The end position on the edge of the current and next polygon. [(x, y, z)]
	///  @param[in]		prevRef		The reference id of the previous polygon. [opt]
	///  @param[in]		prevTile	The tile containing the previous polygon. [opt]
	///  @param[in]		prevPoly	The previous polygon. [opt]
	///  @param[in]		curRef		The reference id of the current polygon.
	///  @param[in]		curTile		The tile containing the current polygon.
	///  @param[in]		curPoly		The current polygon.
	///  @param[in]		nextRef		The refernece id of the next polygon. [opt]
	///  @param[in]		nextTile	The tile containing the next polygon. [opt]
	///  @param[in]		nextPoly	The next polygon. [opt]
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual float getCost(const float* pa, const float* pb,
						  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
						  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
						  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#else
	float getCost(const float* pa, const float* pb,
				  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
				  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
				  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#endif

	/// @name Getters and setters for the default implementation data.
	///@{

	/// Returns the traversal cost of the area.
	///  @param[in]		i		The id of the area.
	/// @returns The traversal cost of the area.
	inline float getAreaCost(const int i) const { return m_areaCost[i]; }

	/// Sets the traversal cost of the area.
	///  @param[in]		i		The id of the area.
	///  @param[in]		cost	The new cost of traversing the area.
	inline void setAreaCost(const int i, const float cost) { m_areaCost[i] = cost; } 

	/// Returns the include flags for the filter.
	/// Any polygons that include one or more of these flags will be
	/// included in the operation.
	inline unsigned short getIncludeFlags() const { return m_includeFlags; }

	/// Sets the include flags for the filter.
	/// @param[in]		flags	The new flags.
	/// 
	/// 详见 SamplePolyFlags
	inline void setIncludeFlags(const unsigned short flags) { m_includeFlags = flags; }

	/// Returns the exclude flags for the filter.
	/// Any polygons that include one ore more of these flags will be
	/// excluded from the operation.
	inline unsigned short getExcludeFlags() const { return m_excludeFlags; }

	/// Sets the exclude flags for the filter.
	/// @param[in]		flags		The new flags.
	/// 
	/// 详见 SamplePolyFlags
	inline void setExcludeFlags(const unsigned short flags) { m_excludeFlags = flags; }	

	///@}

};

/// Provides information about raycast hit
/// filled by dtNavMeshQuery::raycast
/// @ingroup detour
/// 
/// 提供命中射线的相关信息
struct dtRaycastHit
{
	/// The hit parameter. (FLT_MAX if no wall hit.)
	// 命中参数，如果没有命中任何物体，则返回 FLT_MAX
	float t; 
	
	/// hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	// 命中物体的法向量，通过这个值可以实现反弹、摩擦力等计算
	float hitNormal[3];

	/// The index of the edge on the final polygon where the wall was hit.
	// 表示射线命中物体时，所命中的最终多边形中，射线所命中的多边形的边的索引
	int hitEdgeIndex;
	
	/// Pointer to an array of reference ids of the visited polygons. [opt]
	// 表示一个指向访问过的多边形的 id 数组的指针
	dtPolyRef* path;
	
	/// The number of visited polygons. [opt]
	// 已访问的多边形数量
	int pathCount;

	/// The maximum number of polygons the @p path array can hold.
	// 可以存储的最大多边形数量
	int maxPath;

	///  The cost of the path until hit.
	// 命中路径的代价
	float pathCost;
};

/// Provides custom polygon query behavior.
/// Used by dtNavMeshQuery::queryPolygons.
/// @ingroup detour
/// 
/// 提供自定的多边形查询行为，用于 dtNavMeshQuery::queryPolygons 函数。
class dtPolyQuery
{
public:
	virtual ~dtPolyQuery();

	/// Called for each batch of unique polygons touched by the search area in dtNavMeshQuery::queryPolygons.
	/// This can be called multiple times for a single query.
	/// 
	/// 在一次查询中，这个函数可以被调用多次。
	virtual void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count) = 0;
};

/// Provides the ability to perform pathfinding related queries against
/// a navigation mesh.
/// @ingroup detour
/// 
/// 提供了针对导航网格执行路径查找相关的查询功能
class dtNavMeshQuery
{
public:
	dtNavMeshQuery();
	~dtNavMeshQuery();
	
	/// Initializes the query object.
	///  @param[in]		nav			Pointer to the dtNavMesh object to use for all queries.
	///  @param[in]		maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
	/// @returns The status flags for the query.
	/// 
	/// 初始化查询对象
	/// nav 指向导航网格对象的指针，用于执行所有查询
	dtStatus init(const dtNavMesh* nav, const int maxNodes);
	
	/// @name Standard Pathfinding Functions
	/// @{

	/// Finds a path from the start polygon to the end polygon.
	///  @param[in]		startRef	The refrence id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
					  const float* startPos, const float* endPos,
					  const dtQueryFilter* filter,
					  dtPolyRef* path, int* pathCount, const int maxPath) const;

	/// Finds the straight path from the start to the end position within the polygon corridor.
	///  @param[in]		startPos			Path start position. [(x, y, z)]
	///  @param[in]		endPos				Path end position. [(x, y, z)]
	///  @param[in]		path				An array of polygon references that represent the path corridor.
	///  @param[in]		pathSize			The number of polygons in the @p path array.
	///  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	///  @param[out]	straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	///  @param[out]	straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	///  @param[out]	straightPathCount	The number of points in the straight path.
	///  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	///  @param[in]		options				Query options. (see: #dtStraightPathOptions)
	/// @returns The status flags for the query.
	/// 
	/// 找到从起始位置到结束位置的直线路径，该路径位于多边形内部。
	/// startPos 路径起始位置
	/// endPos 路径结束位置
	/// path 一个代表路径通道的多边形引用数组
	/// pathSize 在 path 中存储多边形路径的数量
	/// straightPath 描述直线路径的点
	/// straightPathFlags 描述每个点标识符
	/// straightPathRefs 用于唯一标识导航网格中的每个多边形
	/// straightPathCount 直线路径点的数量
	/// maxStraightPath 直线路径点可以容纳的路径点数量上限
	/// options 查询选项，详见 dtStraightPathOptions
	dtStatus findStraightPath(const float* startPos, const float* endPos,
							  const dtPolyRef* path, const int pathSize,
							  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
							  int* straightPathCount, const int maxStraightPath, const int options = 0) const;

	///@}
	/// @name Sliced Pathfinding Functions
	/// Common use case:
	///	-# Call initSlicedFindPath() to initialize the sliced path query.
	///	-# Call updateSlicedFindPath() until it returns complete.
	///	-# Call finalizeSlicedFindPath() to get the path.
	///@{ 

	/// Intializes a sliced path query.
	///  @param[in]		startRef	The refrence id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		options		query options (see: #dtFindPathOptions)
	/// @returns The status flags for the query.
	/// 
	/// 初始化一个路径查询切片（当从起始位置到目标位置需要经过1个及以上的路径点时，每个路径点将作为一个独立的寻路路径）
	/// 这样做可以提升寻路的效率，将每一个路径点作为一个独立的目标
	/// 
	/// startRef 起始多边形的复合id
	/// endRef 目标多边形的复合id
	/// startPos 群集代理在起始多边形中的位置信息
	/// endPos 群集代理在目标多边形中的到达位置信息
	/// filter 应用于路径查询条件的多边形筛选器
	/// options 路径查询选项，详见 dtFindPathOptions
	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
								const float* startPos, const float* endPos,
								const dtQueryFilter* filter, const unsigned int options = 0);

	/// Updates an in-progress sliced path query.
	///  @param[in]		maxIter		The maximum number of iterations to perform.
	///  @param[out]	doneIters	The actual number of iterations completed. [opt]
	/// @returns The status flags for the query.
	/// 
	/// 更新正在进行的切片路径查询
	/// maxIter 要执行的最大迭代次数
	/// doneIters 实际完成的迭代次数，可选项。
	dtStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	/// Finalizes and returns the results of a sliced path query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	/// 
	/// 对切片路径查询进行最终处理并返回结果
	/// path 一个按顺序排列的多边形引用列表。（从起点到终点）
	/// pathCount 返回在 path 中存储的元素数量
	/// maxPath path 可以存储的多边形的数量上限
	dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath);
	
	/// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	/// polygon on the existing path that was visited during the search.
	///  @param[in]		existing		An array of polygon references for the existing path.
	///  @param[in]		existingSize	The number of polygon in the @p existing array.
	///  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.) 
	///  								[(polyRef) * @p pathCount]
	///  @param[out]	pathCount		The number of polygons returned in the @p path array.
	///  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	/// 
	/// 对不完整的切片路径查询进行最终处理并返回结果，其中包括沿着已访问的路径中最远的多边形的路径。
	/// existing 存储已有路径的多边形数组的引用
	/// existingSize 多边形数组的元素数量
	/// path 一个按顺序排列的多边形引用列表。（从起点到终点）
	/// pathCount 返回在 path 中存储的元素数量
	/// maxPath path 可以存储的多边形的数量上限
	dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
										   dtPolyRef* path, int* pathCount, const int maxPath);

	///@}
	/// @name Dijkstra Search Functions
	/// @{ 

	/// Finds the polygons along the navigation graph that touch the specified circle.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found. [opt]
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
								   const dtQueryFilter* filter,
								   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								   int* resultCount, const int maxResult) const;
	
	/// Finds the polygons along the naviation graph that touch the specified convex polygon.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		verts			The vertices describing the convex polygon. (CCW) 
	///  								[(x, y, z) * @p nverts]
	///  @param[in]		nverts			The number of vertices in the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a 
	///  								result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
								  const dtQueryFilter* filter,
								  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								  int* resultCount, const int maxResult) const;
	
	/// Gets a path from the explored nodes in the previous search.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	///  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	///  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	///  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	///  				Otherwise returns DT_SUCCESS.
	///  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	///  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const;

	/// @}
	/// @name Local Query Functions
	///@{

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	///
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	/// @returns The status flags for the query.
	/// 
	/// 找到与指定中心点最近的多边形
	/// 
	/// center 搜索框的中心
	/// halfExtents 沿每个轴的搜索距离
	/// filter 用于查询的多边形过滤器
	/// nearestRef 距离中心点最近的多边形复合id。如果没有找到合适的多边形，则设置为0值。
	/// nearestPt 距离中心点最近的多边形上的最近点。如果没有找到合适的多边形，不对其进行修改。
	dtStatus findNearestPoly(const float* center, const float* halfExtents,
							 const dtQueryFilter* filter,
							 dtPolyRef* nearestRef, float* nearestPt) const;

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	/// 
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	///  @param[out]	isOverPoly 	Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
	/// @returns The status flags for the query.
	/// 
	/// 找到与指定中心点最近的多边形
	/// 
	/// center 搜索框的中心
	/// halfExtents 沿每个轴的搜索距离
	/// filter 用于查询的多边形过滤器
	/// nearestRef 距离中心点最近的多边形复合id。如果没有找到合适的多边形，则设置为0值。
	/// isOverPoly 点的 x/z 坐标是否位于多边形内部。如果没有找到合适的多边形，则该值保持不变。
	dtStatus findNearestPoly(const float* center, const float* halfExtents,
							 const dtQueryFilter* filter,
							 dtPolyRef* nearestRef, float* nearestPt, bool* isOverPoly) const;
	
	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	polys		The reference ids of the polygons that overlap the query box.
	///  @param[out]	polyCount	The number of polygons in the search result.
	///  @param[in]		maxPolys	The maximum number of polygons the search result can hold.
	/// @returns The status flags for the query.
	dtStatus queryPolygons(const float* center, const float* halfExtents,
						   const dtQueryFilter* filter,
						   dtPolyRef* polys, int* polyCount, const int maxPolys) const;

	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	/// 
	/// 查找与搜索框重叠的多边形
	/// center 搜索框的中心点
	/// halfExtents 沿每个轴 [(x, y, z)] 搜索的距离
	/// filter 作用于查询的多边形过滤器（条件筛选）
	/// query 找到的多边形将被批量处理，并作为一个整体传递给这个 query 对象。
	dtStatus queryPolygons(const float* center, const float* halfExtents,
						   const dtQueryFilter* filter, dtPolyQuery* query) const;

	/// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the query circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
									const dtQueryFilter* filter,
									dtPolyRef* resultRef, dtPolyRef* resultParent,
									int* resultCount, const int maxResult) const;

	/// Moves from the start to the end position constrained to the navigation mesh.
	///  @param[in]		startRef		The reference id of the start polygon.
	///  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
	///  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultPos		The result position of the mover. [(x, y, z)]
	///  @param[out]	visited			The reference ids of the polygons visited during the move.
	///  @param[out]	visitedCount	The number of polygons visited during the move.
	///  @param[in]		maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	/// @returns The status flags for the query.
	/// 
	/// 从起始位置移动到目标位置，并受限于导航网格。
	/// 
	/// startRef 起始多边形的复合id
	/// startPos 代理在起始多边形内的一个位置
	/// endPos 代理期望移动到的目标位置
	/// filter 作用于路径查询的多边形筛选器
	/// resultPos 代理最终到达的位置，可能与 endPos 有一定偏差。
	/// visited 在移动过程中，所经过的多边形，以数组的方式保存下来。
	/// visitedCount 在移动过程中，所经过的多边形的数量。
	/// maxVisitedSize 在移动过程中，可以容纳的所经过的多边形的数量上限。
	dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
							  const dtQueryFilter* filter,
							  float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;
	
	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	/// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
	///  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	///  @param[out]	path		The reference ids of the visited polygons. [opt]
	///  @param[out]	pathCount	The number of visited polygons. [opt]
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
	/// @returns The status flags for the query.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
					 const dtQueryFilter* filter,
					 float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const;
	
	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		options		govern how the raycast behaves. See dtRaycastOptions
	///  @param[out]	hit			Pointer to a raycast hit structure which will be filled by the results.
	///  @param[in]		prevRef		parent of start ref. Used during for cost calculation [opt]
	/// @returns The status flags for the query.
	/// 
	/// 在导航网格表面从起点到终点投射一条可行走的射线
	/// startRef 起点的多边形id
	/// startPos 表示在起点多边形内的一个射线起点位置（x, y, z）
	/// endPos 射线投射的目标位置，即射线的终点位置
	/// filter 应用于查询的多边形过滤器
	/// options 决定射线投射的行为，目前仅支持 DT_RAYCAST_USE_COSTS 选项
	/// hit 指向一个射线投射碰撞结构体的指针，用于保存结果
	/// prevRef 起始参考的父节点。用于计算成本（可选）
	/// 返回查询状态标识符
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
					 const dtQueryFilter* filter, const unsigned int options,
					 dtRaycastHit* hit, dtPolyRef prevRef = 0) const;


	/// Finds the distance from the specified position to the nearest polygon wall.
	///  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		maxRadius		The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
	///  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	///  @param[out]	hitNormal		The normalized ray formed from the wall point to the 
	///  								source point. [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
								const dtQueryFilter* filter,
								float* hitDist, float* hitPos, float* hitNormal) const;
	
	/// Returns the segments for the specified polygon, optionally including portals.
	///  @param[in]		ref				The reference id of the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	///  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon. 
	///  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
	///  @param[out]	segmentCount	The number of segments returned.
	///  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
	/// @returns The status flags for the query.
	/// 
	/// 返回指定多边形的边界段，可选择是否包含门户点。
	/// ref 多边形的复合id
	/// filter 作用于多边形的条件筛选器
	/// segmentVerts 线段的顶点数组
	/// segmentRefs 每个线段的相邻多边形的复合id。如果边界段是墙，则为零。
	/// segmentCount 返回线段的数量
	/// maxSegments 结果数组可以容纳的边界段的最大数量
	dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
								 float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
								 const int maxSegments) const;

	/// Returns random location on navmesh.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. 
	/// @returns The status flags for the query.
	dtStatus findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
							 dtPolyRef* randomRef, float* randomPt) const;

	/// Returns random location on navmesh within the reach of specified location.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/// The location is not exactly constrained by the circle, but it limits the visited polygons.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		maxRadius		The radius of the search circle. [Units: wu]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
										 const dtQueryFilter* filter, float (*frand)(),
										 dtPolyRef* randomRef, float* randomPt) const;
	
	/// Finds the closest point on the specified polygon.
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	///  @param[out]	posOverPoly	True of the position is over the polygon.
	/// @returns The status flags for the query.
	dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	/// Returns a point on the boundary closest to the source point if the source point is outside the 
	/// polygon's xz-bounds.
	///  @param[in]		ref			The reference id to the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point. [(x, y, z)]
	/// @returns The status flags for the query.
	/// 
	/// 如果源点在多边形的 xz 平面边界之外，函数将返回距离源点最近的边界上的点。
	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;
	
	/// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	///  @param[out]	height		The height at the surface of the polygon.
	/// @returns The status flags for the query.
	/// 
	/// 使用高度细节信息以获得给定位置处多边形高度信息
	/// 
	/// ref 多边形的复合id
	/// pos 位于多边形xz平面边界内的位置
	/// height 多边形表面的高度
	dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;

	/// @}
	/// @name Miscellaneous Functions
	/// @{

	/// Returns true if the polygon reference is valid and passes the filter restrictions.
	///  @param[in]		ref			The polygon reference to check.
	///  @param[in]		filter		The filter to apply.
	bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	/// Returns true if the polygon reference is in the closed list. 
	///  @param[in]		ref		The reference id of the polygon to check.
	/// @returns True if the polygon is in closed list.
	bool isInClosedList(dtPolyRef ref) const;
	
	/// Gets the node pool.
	/// @returns The node pool.
	class dtNodePool* getNodePool() const { return m_nodePool; }
	
	/// Gets the navigation mesh the query object is using.
	/// @return The navigation mesh the query object is using.
	const dtNavMesh* getAttachedNavMesh() const { return m_nav; }

	/// @}
	
private:
	// Explicitly disabled copy constructor and copy assignment operator
	dtNavMeshQuery(const dtNavMeshQuery&);
	dtNavMeshQuery& operator=(const dtNavMeshQuery&);
	
	/// Queries polygons within a tile.
	/// 查询特定瓦片内的多边形
	void queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							 const dtQueryFilter* filter, dtPolyQuery* query) const;

	/// Returns portal points between two polygons.
	// 返回两个多边形之间的门户点
	dtStatus getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
							 unsigned char& fromType, unsigned char& toType) const;
	dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* left, float* right) const;
	
	/// Returns edge mid point between two polygons.
	/// 返回两个多边形之间的边缘中点
	dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	dtStatus getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* mid) const;
	
	// Appends vertex to a straight path
	dtStatus appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
						  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
						  int* straightPathCount, const int maxStraightPath) const;

	// Appends intermediate portal points to a straight path.
	dtStatus appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
						   float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
						   int* straightPathCount, const int maxStraightPath, const int options) const;

	// Gets the path leading to the specified end node.
	dtStatus getPathToNode(struct dtNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const;
	
	// 导航网格，存储寻路相关数据
	const dtNavMesh* m_nav;				///< Pointer to navmesh data.

	// 导航路径的查询数据
	struct dtQueryData
	{
		// 状态枚举值
		dtStatus status;
		// 最后一个最优节点
		struct dtNode* lastBestNode;
		// 最后一个最优节点的代价（启发式搜索的代价）
		float lastBestNodeCost;
		// 起始和终点位置所属的多边形id
		dtPolyRef startRef, endRef;
		// 起始和终止位置的坐标信息
		float startPos[3], endPos[3];
		// 路径查询过滤器
		const dtQueryFilter* filter;
		// 查询选项
		unsigned int options;
		// 用于判断拐角点是否可以被删除，以直线路径寻路
		float raycastLimitSqr;
	};

	// 分割路径查询状态，将一个大型任务分割成若干个子任务
	dtQueryData m_query;				///< Sliced query state.

	// 指向寻路路径的节点的小内存池
	class dtNodePool* m_tinyNodePool;	///< Pointer to small node pool.
	// 指向寻路路径节点的内存池
	class dtNodePool* m_nodePool;		///< Pointer to node pool.
	// 指向寻路路径的开放队列列表
	class dtNodeQueue* m_openList;		///< Pointer to open list queue.
};

/// Allocates a query object using the Detour allocator.
/// @return An allocated query object, or null on failure.
/// @ingroup detour
dtNavMeshQuery* dtAllocNavMeshQuery();

/// Frees the specified query object using the Detour allocator.
///  @param[in]		query		A query object allocated using #dtAllocNavMeshQuery
/// @ingroup detour
/// 
/// 释放通过 Detour allocator 分配的查询对象内存
void dtFreeNavMeshQuery(dtNavMeshQuery* query);

#endif // DETOURNAVMESHQUERY_H
