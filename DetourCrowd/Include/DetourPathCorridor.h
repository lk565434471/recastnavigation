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

#ifndef DETOUTPATHCORRIDOR_H
#define DETOUTPATHCORRIDOR_H

#include "DetourNavMeshQuery.h"

/// Represents a dynamic polygon corridor used to plan agent movement.
/// @ingroup crowd, detour
/// 
/// 用于规划代理移动的动态多边形路径
class dtPathCorridor
{
	// 当前位置（x,y,z）
	float m_pos[3];
	// 目标位置（x,y,z）
	float m_target[3];
	// 可行走的一系列路径
	dtPolyRef* m_path;
	// 已使用的路径点计数
	int m_npath;
	// 最大存储的路径点数量
	int m_maxPath;
	
public:
	dtPathCorridor();
	~dtPathCorridor();
	
	/// Allocates the corridor's path buffer. 
	///  @param[in]		maxPath		The maximum path size the corridor can handle.
	/// @return True if the initialization succeeded.
	/// 
	/// 初始化导航路径的缓冲区，确保该函数仅被调用一次
	/// maxPath 可以处理的最大导航路径点
	bool init(const int maxPath);
	
	/// Resets the path corridor to the specified position.
	///  @param[in]		ref		The polygon reference containing the position.
	///  @param[in]		pos		The new position in the corridor. [(x, y, z)]
	/// 
	/// 将导航路径重置到指定位置
	/// ref 导航路径中，第一个包含位置信息的多边形路径点引用
	/// pos 当前路径点的坐标，强制 m_pos 和 m_target = pos
	void reset(dtPolyRef ref, const float* pos);
	
	/// Finds the corners in the corridor from the position toward the target. (The straightened path.)
	///  @param[out]	cornerVerts		The corner vertices. [(x, y, z) * cornerCount] [Size: <= maxCorners]
	///  @param[out]	cornerFlags		The flag for each corner. [(flag) * cornerCount] [Size: <= maxCorners]
	///  @param[out]	cornerPolys		The polygon reference for each corner. [(polyRef) * cornerCount] 
	///  								[Size: <= @p maxCorners]
	///  @param[in]		maxCorners		The maximum number of corners the buffers can hold.
	///  @param[in]		navquery		The query object used to build the corridor.
	///  @param[in]		filter			The filter to apply to the operation.
	/// @return The number of corners returned in the corner buffers. [0 <= value <= @p maxCorners]
	/// 
	/// 查询从当前位置到目标位置的导航路径的拐角顶点。（处理路径平滑，直线路径）
	/// cornerVerts 拐角顶点数组 [(x, y, z) * 拐角数量]，小于等于 maxCorners
	/// cornerFlags 每个拐角顶点的标志位信息，[(flag) * 拐角数量]，小于等于 maxCorners
	/// cornerPolys 每个拐角的多边形对象引用 [(polyRef) * 拐角数量]，小于等 maxCorners
	/// maxCorners 最大拐角数量
	/// navquery 用于构建导航路径的导航网格查询对象
	/// filter 导航网格过滤对象，用于筛选导航网格
	/// 返回导航缓冲区中存储的拐角数量
	int findCorners(float* cornerVerts, unsigned char* cornerFlags,
					dtPolyRef* cornerPolys, const int maxCorners,
					dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Attempts to optimize the path if the specified point is visible from the current position.
	///  @param[in]		next					The point to search toward. [(x, y, z])
	///  @param[in]		pathOptimizationRange	The maximum range to search. [Limit: > 0]
	///  @param[in]		navquery				The query object used to build the corridor.
	///  @param[in]		filter					The filter to apply to the operation.	
	/// 		
	/// 如果从当前位置可见指定点，则尝试优化路径。
	void optimizePathVisibility(const float* next, const float pathOptimizationRange,
								dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Attempts to optimize the path using a local area search. (Partial replanning.) 
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.	
	/// 
	/// 使用局部区域搜索来尝试优化路径，也称为部分重新规划。
	/// navquery 用于构建路径的查询对象
	/// filter 应用于路径查询操作的过滤器
	bool optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	// 通过离散连接点进行移动
	// offMeshConRef 多边形的复合id，该多边形为一个离散连接点
	// refs
	// startPos 起始位置
	// endPos 目标位置
	// navquery 导航网格查询
	bool moveOverOffmeshConnection(dtPolyRef offMeshConRef, dtPolyRef* refs,
								   float* startPos, float* endPos,
								   dtNavMeshQuery* navquery);

	// 修复起始路径，区别代理在一个可通行的多边形内的一个有效坐标点
	bool fixPathStart(dtPolyRef safeRef, const float* safePos);

	// 函数没有被使用
	bool trimInvalidPath(dtPolyRef safeRef, const float* safePos,
						 dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Checks the current corridor path to see if its polygon references remain valid. 
	///  @param[in]		maxLookAhead	The number of polygons from the beginning of the corridor to search.
	///  @param[in]		navquery		The query object used to build the corridor.
	///  @param[in]		filter			The filter to apply to the operation.	
	/// 
	/// 对当前走廊路径的多边形引用进行有效性检查。
	/// maxLookAhead 在走廊路径中，从起点开始搜索的多边形数量
	/// navquery 用于构建走廊路径的查询对象
	/// filter 在操作过程中应用的过滤器
	bool isValid(const int maxLookAhead, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Moves the position from the current location to the desired location, adjusting the corridor 
	/// as needed to reflect the change.
	///  @param[in]		npos		The desired new position. [(x, y, z)]
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.
	/// @return Returns true if move succeeded.
	/// 
	/// 将代理从当前位置移动到目标位置，并根据需要调整路径，以翻译这种变化的过程。
	/// 
	/// npos 群集代理的当前位置
	/// navquery 用于构建导航路径的查询对象
	/// filter 根据路径筛选条件不可行走的区域
	bool movePosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	/// Moves the target from the curent location to the desired location, adjusting the corridor
	/// as needed to reflect the change. 
	///  @param[in]		npos		The desired new target position. [(x, y, z)]
	///  @param[in]		navquery	The query object used to build the corridor.
	///  @param[in]		filter		The filter to apply to the operation.
	/// @return Returns true if move succeeded.
	/// 
	/// 未使用函数
	bool moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	/// Loads a new path and target into the corridor.
	///  @param[in]		target		The target location within the last polygon of the path. [(x, y, z)]
	///  @param[in]		path		The path corridor. [(polyRef) * @p npolys]
	///  @param[in]		npath		The number of polygons in the path.
	/// 
	/// 将新的路径和目标点加载到路径走廊中。
	/// 
	/// target 路径中最后一个多边形内的目标位置
	/// polys 路径走廊
	/// 路径中包含的多边形数量
	void setCorridor(const float* target, const dtPolyRef* polys, const int npath);
	
	/// Gets the current position within the corridor. (In the first polygon.)
	/// @return The current position within the corridor.
	/// 
	/// 获取走廊内当前位置。（在第一个多边形内）
	inline const float* getPos() const { return m_pos; }

	/// Gets the current target within the corridor. (In the last polygon.)
	/// @return The current target within the corridor.
	/// 
	/// 获取走廊内当前目标位置。（在最后一个多边形内）
	inline const float* getTarget() const { return m_target; }
	
	/// The polygon reference id of the first polygon in the corridor, the polygon containing the position.
	/// @return The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.)
	/// 
	/// 走廊中第一个多边形的复合id，包含当前位置的多边形。
	inline dtPolyRef getFirstPoly() const { return m_npath ? m_path[0] : 0; }

	/// The polygon reference id of the last polygon in the corridor, the polygon containing the target.
	/// @return The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.)
	/// 
	/// 走廊中最后一个多边形的复合id，包含目标位置的多边形。
	inline dtPolyRef getLastPoly() const { return m_npath ? m_path[m_npath-1] : 0; }
	
	/// The corridor's path.
	/// @return The corridor's path. [(polyRef) * #getPathCount()]
	/// 
	/// 获取走廊的路径
	inline const dtPolyRef* getPath() const { return m_path; }

	/// The number of polygons in the current corridor path.
	/// @return The number of polygons in the current corridor path.
	/// 
	/// 获取当前走廊路径中的多边形数量
	inline int getPathCount() const { return m_npath; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtPathCorridor(const dtPathCorridor&);
	dtPathCorridor& operator=(const dtPathCorridor&);
};

// 将路径中已访问的部分与当前路径进行合并，它们连接成一个更长的路径。
// path 当前路径
// npath 当前路径的长度
// maxPath 最大路径
// visited 已访问路径
// nvisited 已访问路径的长度
int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
							  const dtPolyRef* visited, const int nvisited);

// 未使用函数
int dtMergeCorridorEndMoved(dtPolyRef* path, const int npath, const int maxPath,
							const dtPolyRef* visited, const int nvisited);

// 在路径起点处进行路径的快捷合并。函数通过检查路径起点附近的多边形，找到最长的可行走路径段，并
// 将其替换为一条直接连接起点和终点的直线路径。这样可以减少路径中的多边形数量，简化路径的形状，提
// 高寻路效率。
int dtMergeCorridorStartShortcut(dtPolyRef* path, const int npath, const int maxPath,
								 const dtPolyRef* visited, const int nvisited);

#endif // DETOUTPATHCORRIDOR_H
