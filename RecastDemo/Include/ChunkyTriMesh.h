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

#ifndef CHUNKYTRIMESH_H
#define CHUNKYTRIMESH_H

// 三角形网格块
struct rcChunkyTriMeshNode
{
	// AABB（包围盒）的最小、最大边界
	float bmin[2];
	float bmax[2];
	// 起始节点的索引
	int i;
	// 存储的节点数量
	int n;
};

// 分块三角形网格
// 用于快速查找场景中的静态三角形索引，并在运行时根据查询的点的位置
// 快速确定该点所在的三角形，从而用于导航网格生成过程中。
struct rcChunkyTriMesh
{
	inline rcChunkyTriMesh() : nodes(0), nnodes(0), tris(0), ntris(0), maxTrisPerChunk(0) {}
	inline ~rcChunkyTriMesh() { delete [] nodes; delete [] tris; }

	rcChunkyTriMeshNode* nodes;
	// 实际存储的节点数量，小于 nodes 的长度
	int nnodes;
	// 三角形的三条边的顶点索引数组
	int* tris;
	// 三角形的数量
	int ntris;
	// 每一个 rcChunkyTriMeshNode 最多存储的三角形数量
	int maxTrisPerChunk;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcChunkyTriMesh(const rcChunkyTriMesh&);
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh&);
};

/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
/// 
/// 生成分割的三角形网格，这个三角形网格是一个带有 AABB 树结构的数据结构。
/// 其中每个节点，最多包含 trisPerChunk 个三角形网格。
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
						   int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
// 返回与输入矩形相交的地图块索引列表
// ids 用于存储索引
// maxIds 存储的最大上限
// 函数返回值表示，ids 中存储的元素个数
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

/// Returns the chunk indices which overlap the input segment.
// 返回与输入线段相交的地图块索引列表
// ids 用于存储索引
// maxIds 存储的最大上限
// 函数返回值表示，ids 中存储的元素个数
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, float p[2], float q[2], int* ids, const int maxIds);


#endif // CHUNKYTRIMESH_H
