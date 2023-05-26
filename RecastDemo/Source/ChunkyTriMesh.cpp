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

#include "ChunkyTriMesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// AABB（包围盒）
struct BoundsItem
{
	// 最小、最大边界
	float bmin[2];
	float bmax[2];
	// 节点的索引，从 0 ... n。
	int i;
};

static int compareItemX(const void* va, const void* vb)
{
	const BoundsItem* a = (const BoundsItem*)va;
	const BoundsItem* b = (const BoundsItem*)vb;
	if (a->bmin[0] < b->bmin[0])
		return -1;
	if (a->bmin[0] > b->bmin[0])
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BoundsItem* a = (const BoundsItem*)va;
	const BoundsItem* b = (const BoundsItem*)vb;
	if (a->bmin[1] < b->bmin[1])
		return -1;
	if (a->bmin[1] > b->bmin[1])
		return 1;
	return 0;
}

// 函数通过遍历 items 找到最小和最大边界
static void calcExtends(const BoundsItem* items, const int /*nitems*/,
						const int imin, const int imax,
						float* bmin, float* bmax)
{
	bmin[0] = items[imin].bmin[0];
	bmin[1] = items[imin].bmin[1];
	
	bmax[0] = items[imin].bmax[0];
	bmax[1] = items[imin].bmax[1];
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BoundsItem& it = items[i];
		if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
		if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
		
		if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
		if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
	}
}

// 用于确定 x 和 y 轴，哪个更长
inline int longestAxis(float x, float y)
{
	return y > x ? 1 : 0;
}

// 函数将三角形分布到不同的 rcChunkyTriMeshNode 中，以提高导航网格的计数消耗
static void subdivide(BoundsItem* items, int nitems, int imin, int imax, int trisPerChunk,
					  int& curNode, rcChunkyTriMeshNode* nodes, const int maxNodes,
					  int& curTri, int* outTris, const int* inTris)
{
	// 计算元素的数量
	int inum = imax - imin;
	// 当前节点的索引
	int icur = curNode;
	
	// 检查节点数量大于等于上限
	if (curNode >= maxNodes)
		return;

	rcChunkyTriMeshNode& node = nodes[curNode++];
	
	// 如果 inum <= trisPerChunk，则为叶子节点
	// 否则，发生裂变，我们称之为逃逸节点
	if (inum <= trisPerChunk)
	{
		// Leaf
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		// Copy triangles.
		node.i = curTri;
		node.n = inum;
		
		// 从资源加载对象中，拷贝三角形顶点索引
		for (int i = imin; i < imax; ++i)
		{
			const int* src = &inTris[items[i].i*3];
			int* dst = &outTris[curTri*3];
			curTri++;
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
		}
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax[0] - node.bmin[0],
							   node.bmax[1] - node.bmin[1]);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemY);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		// Right
		subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
						   int trisPerChunk, rcChunkyTriMesh* cm)
{
	// 动态计数，需要多少个 rcChunkyTriMeshNode 节点可以容纳所有的三角形
	int nchunks = (ntris + trisPerChunk-1) / trisPerChunk;
	// 创建 rcChunkyTriMeshNode 数组
	cm->nodes = new rcChunkyTriMeshNode[nchunks*4];
	if (!cm->nodes)
		return false;
	
	// 存储三角形的顶点坐标的索引
	cm->tris = new int[ntris*3];
	if (!cm->tris)
		return false;
	
	// 三角形的数量
	cm->ntris = ntris;

	// Build tree
	// 以数组构建一颗树
	BoundsItem* items = new BoundsItem[ntris];
	if (!items)
		return false;

	for (int i = 0; i < ntris; i++)
	{
		const int* t = &tris[i*3];
		BoundsItem& it = items[i];
		it.i = i;
		// Calc triangle XZ bounds.
		// 计算三角形沿 xz 轴平面的边界
		it.bmin[0] = it.bmax[0] = verts[t[0]*3+0];
		it.bmin[1] = it.bmax[1] = verts[t[0]*3+2];
		for (int j = 1; j < 3; ++j)
		{
			const float* v = &verts[t[j]*3];
			if (v[0] < it.bmin[0]) it.bmin[0] = v[0]; 
			if (v[2] < it.bmin[1]) it.bmin[1] = v[2]; 

			if (v[0] > it.bmax[0]) it.bmax[0] = v[0]; 
			if (v[2] > it.bmax[1]) it.bmax[1] = v[2]; 
		}
	}

	int curTri = 0;
	int curNode = 0;
	subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks*4, curTri, cm->tris, tris);
	
	delete [] items;
	
	cm->nnodes = curNode;
	
	// Calc max tris per node.
	cm->maxTrisPerChunk = 0;
	// 遍历所有子节点，找出拥有最多三角形顶点索引的那个节点
	// 用它的值，作为每一个块可以存储三角形顶点索引的数量上限
	for (int i = 0; i < cm->nnodes; ++i)
	{
		rcChunkyTriMeshNode& node = cm->nodes[i];
		const bool isLeaf = node.i >= 0;
		if (!isLeaf) continue;
		if (node.n > cm->maxTrisPerChunk)
			cm->maxTrisPerChunk = node.n;
	}
	 
	return true;
}


inline bool checkOverlapRect(const float amin[2], const float amax[2],
							 const float bmin[2], const float bmax[2])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	return overlap;
}

int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm,
							   float bmin[2], float bmax[2],
							   int* ids, const int maxIds)
{
	// Traverse tree
	int i = 0;
	int n = 0;
	// 遍历三角形网格块，BVTree 树
	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		// 判断当前节点是否与给定的矩形重叠
		const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;
		
		// 只有当前节点是叶子节点，且相交，且已存储的索引数量小于 maxIds
		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}
		
		// 如果是叶子节点，或相交，则递增
		if (overlap || isLeafNode)
			i++;
		// 计算出裂变的节点位置
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}
	
	return n;
}


// 判断线段是否与矩形框相交
// 计算沿 xz 轴，从p点到q点的向量
static bool checkOverlapSegment(const float p[2], const float q[2],
								const float bmin[2], const float bmax[2])
{
	static const float EPSILON = 1e-6f;

	//tmin 和 tmax，分别代表射线与AABB包围盒的最近点和最远点的距离比例
	float tmin = 0;
	float tmax = 1;
	float d[2];
	d[0] = q[0] - p[0];
	d[1] = q[1] - p[1];
	
	for (int i = 0; i < 2; i++)
	{
		if (fabsf(d[i]) < EPSILON)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			// 这个是一个光线与平面相交的判断过程中的一个特殊情况，即光线与平面平行。
			// 如果光线与平面平行，那么光线不会与平面相交，因此也不会与平面包围的物体相交。
			// 但是如果光线的起点在平面内部，那么这个光线也是可以和平面相交的，因此需要判
			// 断光线的起点是否在平面内部，如果不在，那么就不存在相交。
			// 这里的 “slab” 是指平面所定义的一个矩形区域，即平面的投影区域。
			if (p[i] < bmin[i] || p[i] > bmax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			// 计算射线与平面的交点 t 值
			// 这里判断的内容就是，射线与平面的两个交点，是否在转化后的 AABB（包围盒）内
			// tmin 和 tmax 是 bmin 和 bmax 量化后的值
			float ood = 1.0f / d[i];
			float t1 = (bmin[i] - p[i]) * ood;
			float t2 = (bmax[i] - p[i]) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}
	return true;
}

int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm,
								  float p[2], float q[2],
								  int* ids, const int maxIds)
{
	// Traverse tree
	int i = 0;
	int n = 0;
	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		const bool overlap = checkOverlapSegment(p, q, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;
		
		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}
		
		if (overlap || isLeafNode)
			i++;
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}
	
	return n;
}
