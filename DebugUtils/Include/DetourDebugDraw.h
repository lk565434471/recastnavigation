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

#ifndef DETOURDEBUGDRAW_H
#define DETOURDEBUGDRAW_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCacheBuilder.h"

// 导航网格标记枚举值
enum DrawNavMeshFlags
{
	// 绘制连接两个离散点的跨越连接：比如导航路径的移动线
	DU_DRAWNAVMESH_OFFMESHCONS = 0x01,
	// 绘制导航网格时，绘制导航网格闭合列表的标志
	DU_DRAWNAVMESH_CLOSEDLIST = 0x02,
	// 绘制导航网格时，按照 Tile（瓦片）的颜色进行绘制
	DU_DRAWNAVMESH_COLOR_TILES = 0x04
};

// 绘制导航网格
// dd 绘制图形的具体实现
// mesh 包含所有需要绘制的网格（瓦片）信息
// flags 绘制标记 enum DrawNavMeshFlags
void duDebugDrawNavMesh(struct duDebugDraw* dd, const dtNavMesh& mesh, unsigned char flags);
// duDebugDrawNavMeshWithClosedList 与 duDebugDrawNavMesh 的不同之处在于
// 包含一个关闭列表，用于判断绘制的多边形是否包含在关闭列表当中
void duDebugDrawNavMeshWithClosedList(struct duDebugDraw* dd, const dtNavMesh& mesh, const dtNavMeshQuery& query, unsigned char flags);
// 绘制导航网格的查询节点
// 绘制父节点和子节点之间的连接关系，以帮助可视化程序调试寻路算法，检查算法是否
// 按照预期的方式处理节点
void duDebugDrawNavMeshNodes(struct duDebugDraw* dd, const dtNavMeshQuery& query);
// 绘制导航网格中三角形网格中的 BV 树节点边框
void duDebugDrawNavMeshBVTree(struct duDebugDraw* dd, const dtNavMesh& mesh);
// 检查网格边缘是否与当前侧面匹配，如果匹配，则说明这是一个连接边界
// 绘制导航网格的4个边角，以矩形的方式
void duDebugDrawNavMeshPortals(struct duDebugDraw* dd, const dtNavMesh& mesh);
// duDebugDrawNavMeshPolysWithFlags 与 duDebugDrawNavMeshPortals类似，但只检测包含 polyFlags 标志的多边形
void duDebugDrawNavMeshPolysWithFlags(struct duDebugDraw* dd, const dtNavMesh& mesh, const unsigned short polyFlags, const unsigned int col);
// 绘制单个多边形
void duDebugDrawNavMeshPoly(struct duDebugDraw* dd, const dtNavMesh& mesh, dtPolyRef ref, const unsigned int col);

void duDebugDrawTileCacheLayerAreas(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch);
void duDebugDrawTileCacheLayerRegions(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch);
void duDebugDrawTileCacheContours(duDebugDraw* dd, const struct dtTileCacheContourSet& lcset,
								  const float* orig, const float cs, const float ch);
void duDebugDrawTileCachePolyMesh(duDebugDraw* dd, const struct dtTileCachePolyMesh& lmesh,
								  const float* orig, const float cs, const float ch);

#endif // DETOURDEBUGDRAW_H
