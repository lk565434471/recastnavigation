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

#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourStatus.h"
#include "DetourAssert.h"
#include "DetourTileCacheBuilder.h"
#include <string.h>

dtTileCacheAlloc::~dtTileCacheAlloc()
{
	// Defined out of line to fix the weak v-tables warning
}

dtTileCacheCompressor::~dtTileCacheCompressor()
{
	// Defined out of line to fix the weak v-tables warning
}

template<class T> class dtFixedArray
{
	dtTileCacheAlloc* m_alloc;
	T* m_ptr;
	const int m_size;
	inline void operator=(dtFixedArray<T>& p);
public:
	inline dtFixedArray(dtTileCacheAlloc* a, const int s) : m_alloc(a), m_ptr((T*)a->alloc(sizeof(T)*s)), m_size(s) {}
	inline ~dtFixedArray() { if (m_alloc) m_alloc->free(m_ptr); }
	inline operator T*() { return m_ptr; }
	inline int size() const { return m_size; }
};

inline int getDirOffsetX(int dir)
{
	const int offset[4] = { -1, 0, 1, 0, };
	return offset[dir&0x03];
}

inline int getDirOffsetY(int dir)
{
	const int offset[4] = { 0, 1, 0, -1 };
	return offset[dir&0x03];
}

static const int MAX_VERTS_PER_POLY = 6;	// TODO: use the DT_VERTS_PER_POLYGON
static const int MAX_REM_EDGES = 48;		// TODO: make this an expression.



dtTileCacheContourSet* dtAllocTileCacheContourSet(dtTileCacheAlloc* alloc)
{
	dtAssert(alloc);

	dtTileCacheContourSet* cset = (dtTileCacheContourSet*)alloc->alloc(sizeof(dtTileCacheContourSet));
	memset(cset, 0, sizeof(dtTileCacheContourSet));
	return cset;
}

void dtFreeTileCacheContourSet(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset)
{
	dtAssert(alloc);

	if (!cset) return;
	for (int i = 0; i < cset->nconts; ++i)
		alloc->free(cset->conts[i].verts);
	alloc->free(cset->conts);
	alloc->free(cset);
}

dtTileCachePolyMesh* dtAllocTileCachePolyMesh(dtTileCacheAlloc* alloc)
{
	dtAssert(alloc);

	dtTileCachePolyMesh* lmesh = (dtTileCachePolyMesh*)alloc->alloc(sizeof(dtTileCachePolyMesh));
	memset(lmesh, 0, sizeof(dtTileCachePolyMesh));
	return lmesh;
}

void dtFreeTileCachePolyMesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh)
{
	dtAssert(alloc);
	
	if (!lmesh) return;
	alloc->free(lmesh->verts);
	alloc->free(lmesh->polys);
	alloc->free(lmesh->flags);
	alloc->free(lmesh->areas);
	alloc->free(lmesh);
}


// 图层扫描的跨度信息
struct dtLayerSweepSpan
{
	// 可通行路径的数量（邻居）
	unsigned short ns;	// number samples
	// 连续性区域id
	unsigned char id;	// region id
	// y轴上的邻居的id，值：0xff，表示无效邻居.
	unsigned char nei;	// neighbour id
};

static const int DT_LAYER_MAX_NEIS = 16;

struct dtLayerMonotoneRegion
{
	// 区域计数
	int area;
	// 存储邻居的下标区域连通性的下标
	unsigned char neis[DT_LAYER_MAX_NEIS];
	// 邻居的数量
	unsigned char nneis;
	// 区域连通性id，将多个相邻的多边形划分到同一个 region 中
	unsigned char regId;
	// 区域id
	unsigned char areaId;
};

struct dtTempContour
{
	inline dtTempContour(unsigned char* vbuf, const int nvbuf,
						 unsigned short* pbuf, const int npbuf) :
		verts(vbuf), nverts(0), cverts(nvbuf),
		poly(pbuf), npoly(0), cpoly(npbuf) 
	{
	}
	// 顶点缓冲区
	unsigned char* verts;
	// 顶点数量计数
	int nverts;
	// 顶点数量上限
	int cverts;
	// 多边形数组
	unsigned short* poly;
	// 实际多边形数量
	int npoly;
	// 多边形数量
	int cpoly;
};




inline bool overlapRangeExl(const unsigned short amin, const unsigned short amax,
							const unsigned short bmin, const unsigned short bmax)
{
	return (amin >= bmax || amax <= bmin) ? false : true;
}

static void addUniqueLast(unsigned char* a, unsigned char& an, unsigned char v)
{
	const int n = (int)an;
	if (n > 0 && a[n-1] == v) return;
	a[an] = v;
	an++;
}

inline bool isConnected(const dtTileCacheLayer& layer,
						const int ia, const int ib, const int walkableClimb)
{
	if (layer.areas[ia] != layer.areas[ib]) return false;
	if (dtAbs((int)layer.heights[ia] - (int)layer.heights[ib]) > walkableClimb) return false;
	return true;
}

static bool canMerge(unsigned char oldRegId, unsigned char newRegId, const dtLayerMonotoneRegion* regs, const int nregs)
{
	int count = 0;
	for (int i = 0; i < nregs; ++i)
	{
		const dtLayerMonotoneRegion& reg = regs[i];
		if (reg.regId != oldRegId) continue;
		const int nnei = (int)reg.nneis;
		for (int j = 0; j < nnei; ++j)
		{
			if (regs[reg.neis[j]].regId == newRegId)
				count++;
		}
	}
	return count == 1;
}


dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc* alloc,
								 dtTileCacheLayer& layer,
								 const int walkableClimb)
{
	dtAssert(alloc);
	
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	// 由 w * h 个格子组成的区域
	memset(layer.regs,0xff,sizeof(unsigned char)*w*h);
	
	const int nsweeps = w;
	dtFixedArray<dtLayerSweepSpan> sweeps(alloc, nsweeps);
	if (!sweeps)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(sweeps,0,sizeof(dtLayerSweepSpan)*nsweeps);
	
	// Partition walkable area into monotone regions.
	// prevCount 存储的是y轴的邻居数量
	unsigned char prevCount[256];
	unsigned char regId = 0;
	
	for (int y = 0; y < h; ++y)
	{
		if (regId > 0)
			memset(prevCount,0,sizeof(unsigned char)*regId);
		unsigned char sweepId = 0;
		
		for (int x = 0; x < w; ++x)
		{
			const int idx = x + y*w;
			if (layer.areas[idx] == DT_TILECACHE_NULL_AREA) continue;
			
			unsigned char sid = 0xff;
			
			// -x
			const int xidx = (x-1)+y*w;
			// 连通性测试，可攀爬高度必须在 walkableClimb 以内（包含）
			if (x > 0 && isConnected(layer, idx, xidx, walkableClimb))
			{
				if (layer.regs[xidx] != 0xff)
					sid = layer.regs[xidx];
			}
			
			if (sid == 0xff)
			{
				sid = sweepId++;
				sweeps[sid].nei = 0xff;
				sweeps[sid].ns = 0;
			}
			
			// -y
			const int yidx = x+(y-1)*w;
			// 连通性测试，可攀爬高度必须在 walkableClimb 以内（包含）
			if (y > 0 && isConnected(layer, idx, yidx, walkableClimb))
			{
				const unsigned char nr = layer.regs[yidx];
				if (nr != 0xff)
				{
					// Set neighbour when first valid neighbour is encoutered.
					if (sweeps[sid].ns == 0)
						sweeps[sid].nei = nr;
					
					if (sweeps[sid].nei == nr)
					{
						// Update existing neighbour
						sweeps[sid].ns++;
						prevCount[nr]++;
					}
					else
					{
						// This is hit if there is nore than one neighbour.
						// Invalidate the neighbour.
						sweeps[sid].nei = 0xff;
					}
				}
			}
			
			layer.regs[idx] = sid;
		}
		
		// Create unique ID.
		// 创建区域唯一id
		// sweepId 表示有多少个sid
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			// 如果有邻居被设置，且只有一个到这个单元格的连续连，那么，与前一个网格合并成一个
			// 否则，创建一个新的连续性区域。
			if (sweeps[i].nei != 0xff && (unsigned short)prevCount[sweeps[i].nei] == sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				// 因为 0xff 被用于没有有效的邻居的标识符，因此最多允许254个连续性区域id
				if (regId == 255)
				{
					// Region ID's overflow.
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
		// 将局部扫描ID重新映射为区域ID
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			if (layer.regs[idx] != 0xff)
				layer.regs[idx] = sweeps[layer.regs[idx]].id;
		}
	}
	
	// Allocate and init layer regions.
	// 分配并初始化层的区域信息
	const int nregs = (int)regId;
	dtFixedArray<dtLayerMonotoneRegion> regs(alloc, nregs);
	if (!regs)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	memset(regs, 0, sizeof(dtLayerMonotoneRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = 0xff;
	
	// Find region neighbours.
	// 查询相邻的单元格区域，更新连通性信息，及可连通的单元格数量（即邻居）
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			// 表示没有连通性
			if (ri == 0xff)
				continue;
			
			// Update area.
			regs[ri].area++;
			regs[ri].areaId = layer.areas[idx];
			
			// Update neighbours
			const int ymi = x+(y-1)*w;
			if (y > 0 && isConnected(layer, idx, ymi, walkableClimb))
			{
				const unsigned char rai = layer.regs[ymi];
				if (rai != 0xff && rai != ri)
				{
					// 将对方加入到自己的唯一列表当中
					addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
					addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
				}
			}
		}
	}
	
	// 连通性的区域id，是从0到最大区域数量-1的自增值
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = (unsigned char)i;
	
	// 合并区域
	for (int i = 0; i < nregs; ++i)
	{
		dtLayerMonotoneRegion& reg = regs[i];
		
		int merge = -1;
		int mergea = 0;
		for (int j = 0; j < (int)reg.nneis; ++j)
		{
			const unsigned char nei = reg.neis[j];
			dtLayerMonotoneRegion& regn = regs[nei];
			if (reg.regId == regn.regId)
				continue;
			if (reg.areaId != regn.areaId)
				continue;
			if (regn.area > mergea)
			{
				// 在只有一个邻居的连通性区域id == regn.regId 的情况下，可以进行合并
				if (canMerge(reg.regId, regn.regId, regs, nregs))
				{
					mergea = regn.area;
					merge = (int)nei;
				}
			}
		}
		// 可以合并，设置成新的连通性区域id
		if (merge != -1)
		{
			const unsigned char oldId = reg.regId;
			const unsigned char newId = regs[merge].regId;
			for (int j = 0; j < nregs; ++j)
				if (regs[j].regId == oldId) 
					regs[j].regId = newId;
		}
	}
	
	// Compact ids.
	unsigned char remap[256];
	memset(remap, 0, 256);
	// Find number of unique regions.
	regId = 0;
	for (int i = 0; i < nregs; ++i)
		remap[regs[i].regId] = 1;
	for (int i = 0; i < 256; ++i)
		if (remap[i])
			remap[i] = regId++;
	// Remap ids.
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = remap[regs[i].regId];
	
	layer.regCount = regId;
	
	// 重置连通性区域id
	for (int i = 0; i < w*h; ++i)
	{
		if (layer.regs[i] != 0xff)
			layer.regs[i] = regs[layer.regs[i]].regId;
	}
	
	return DT_SUCCESS;
}


//
// r 可通行区域的id
static bool appendVertex(dtTempContour& cont, const int x, const int y, const int z, const int r)
{
	// Try to merge with existing segments.
	// 尝试合并已经存在的线段，这里直接对 cont.nverts - 2/1 的操作
	// 应该是基于一条线段有2个顶点组成的概念得出的
	if (cont.nverts > 1)
	{
		// 点 a 和 点 b
		unsigned char* pa = &cont.verts[(cont.nverts-2)*4];
		unsigned char* pb = &cont.verts[(cont.nverts-1)*4];
		if ((int)pb[3] == r)
		{
			if (pa[0] == pb[0] && (int)pb[0] == x)
			{
				// The verts are aligned aling x-axis, update z.
				// 沿x轴对齐，然后更新z轴的值。
				pb[1] = (unsigned char)y;
				pb[2] = (unsigned char)z;
				return true;
			}
			else if (pa[2] == pb[2] && (int)pb[2] == z)
			{
				// The verts are aligned aling z-axis, update x.
				// 沿z轴对齐，然后更新x轴的值
				pb[0] = (unsigned char)x;
				pb[1] = (unsigned char)y;
				return true;
			}
		}
	}
	
	// Add new point.
	// 添加新的点
	if (cont.nverts+1 > cont.cverts)
		return false;
	
	unsigned char* v = &cont.verts[cont.nverts*4];
	v[0] = (unsigned char)x;
	v[1] = (unsigned char)y;
	v[2] = (unsigned char)z;
	v[3] = (unsigned char)r;
	cont.nverts++;
	
	return true;
}

// 获得相邻区域的id
// layer 可行走区域数据管理对象
// ax 代表第几列的格子
// ay 代表第几行的格子
// dir 移动方向
static unsigned char getNeighbourReg(dtTileCacheLayer& layer,
									 const int ax, const int ay, const int dir)
{
	// w 表示一行有多少列，即格子的数量
	const int w = (int)layer.header->width;
	// 通过 ax + w * ay，可以计算出格子的下标
	const int ia = ax + ay*w;
	
	// con 表示什么，暂时不确定
	const unsigned char con = layer.cons[ia] & 0xf;
	// portal 表示什么，暂时不确定
	const unsigned char portal = layer.cons[ia] >> 4;
	// mask 盲猜是表示方向
	const unsigned char mask = (unsigned char)(1<<dir);
	
	// 说明无法直接通过，但具体因为啥，我也不知道
	// 因为到目前为止，我还不知道 mask 可以表示的值有哪些
	if ((con & mask) == 0)
	{
		// No connection, return portal or hard edge.
		// 没有连接，返回门户或者硬边？直译的
		// 门户指的是多边形之间的通道，如门。。。
		// 硬边指的是墙壁、建筑物或其它无法穿越的障碍物。
		if (portal & mask)
			return 0xf8 + (unsigned char)dir;
		return 0xff;
	}
	
	/*
	* 
	*				   分别对应：负方向x，正方向y，正方向x，负方向y
	*                  {-1, 1, 1, -1}
	* 
	*                                     +1
	*                                   *
	*                                   *
	*                                   * y轴
	*                                   *
	*                   x轴 ************************ +1
	*                                   *
	*                                   *
	*                                   *
	*                                   *                   
	* 
	*/
	const int bx = ax + getDirOffsetX(dir);
	const int by = ay + getDirOffsetY(dir);
	const int ib = bx + by*w;
	
	return layer.regs[ib];
}

// 可行走轮廓线，沿多边形的边。用来描述从起点到终点的可行走区域的轮廓。
// 我的理解是，可能有多种路径最终可以到达终点，轮廓线包含了所有可能。
static bool walkContour(dtTileCacheLayer& layer, int x, int y, dtTempContour& cont)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	cont.nverts = 0;
	
	int startX = x;
	int startY = y;
	int startDir = -1;
	
	// 定义了4个方向。上下左右
	for (int i = 0; i < 4; ++i)
	{
		const int dir = (i+3)&3;
		// 获得相邻区域的id
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		// 获取从起点出发的朝向
		if (rn != layer.regs[x+y*w])
		{
			startDir = dir;
			break;
		}
	}
	// 如果朝向不需要改变，就直接返回可通行了
	if (startDir == -1)
		return true;
	
	int dir = startDir;
	const int maxIter = w*h;
	
	int iter = 0;
	while (iter < maxIter)
	{
		// 获得相邻区域的id
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		
		int nx = x;
		int ny = y;
		int ndir = dir;
		
		if (rn != layer.regs[x+y*w])
		{
			// Solid edge.
			// 实体边界？表示在两个多边形之间的障碍物或不可穿越的边界
			int px = x;
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			// Try to merge with previous vertex.
			// 尝试与前一个顶点合并
			if (!appendVertex(cont, px, (int)layer.heights[x+y*w], pz,rn))
				return false;
			
			// 沿顺时针方向旋转 cw = clockwise
			ndir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			// Move to next.
			nx = x + getDirOffsetX(dir);
			ny = y + getDirOffsetY(dir);
			// 沿逆时针方向旋转 ccw = counter-clockwise
			ndir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (iter > 0 && x == startX && y == startY && dir == startDir)
			break;
		
		x = nx;
		y = ny;
		dir = ndir;
		
		iter++;
	}
	
	// Remove last vertex if it is duplicate of the first one.
	// 检测第一个顶点和最后一个顶点是否重复（数据一致），如果重复，删除最后一个顶点。
	// 实际上，最后一个顶点的内存还在，通过对顶点计数减1，来掩盖最后一个顶点被删除的事实。
	unsigned char* pa = &cont.verts[(cont.nverts-1)*4];
	unsigned char* pb = &cont.verts[0];
	if (pa[0] == pb[0] && pa[2] == pb[2])
		cont.nverts--;
	
	return true;
}	


static float distancePtSeg(const int x, const int z,
						   const int px, const int pz,
						   const int qx, const int qz)
{
	// 分别计算线段的向量在xz轴上的分量
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	// 分别表示点与线段起点的向量在xz轴上的分量
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	// 线段的长度的平方，即线段的向量的模长的平方
	float d = pqx*pqx + pqz*pqz;
	// 点在线段上的投影在线段方向上的比例
	float t = pqx*dx + pqz*dz;
	// 如果线段长度的平方大于0，则将投影长度除以线段长度的平方，得到比例。
	// 这里将t的值缩放到 [0, 1] 之间
	if (d > 0)
		t /= d;
	// 表示最短距离在线段起点之前。
	if (t < 0)
		t = 0;
	// 表示最大距离在线段终点之后。
	else if (t > 1)
		t = 1;
	
	// 更新后，最短距离的向量在xz轴上的分量
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	// 返回最短距离的平方
	return dx*dx + dz*dz;
}

// 简化导航路径的轮廓线
static void simplifyContour(dtTempContour& cont, const float maxError)
{
	// 每次进入函数，都重置多边形的数量统计
	cont.npoly = 0;
	
	for (int i = 0; i < cont.nverts; ++i)
	{
		int j = (i+1) % cont.nverts;
		// Check for start of a wall segment.
		// 检测墙壁的起始线段？
		// 关于这段代码，我暂时没有任何想法
		// 唯一知道的是，判断两个顶点存储的值是否相等，如果不等，则多边形的计数+1
		// 且记录下顶点在数组中的下标
		unsigned char ra = cont.verts[j*4+3];
		unsigned char rb = cont.verts[i*4+3];
		if (ra != rb)
			cont.poly[cont.npoly++] = (unsigned short)i;
	}
	if (cont.npoly < 2)
	{
		// If there is no transitions at all,
		// create some initial points for the simplification process. 
		// Find lower-left and upper-right vertices of the contour.
		// 如果没有转换点，为简化过程创建一些初始点。
		// 找到轮廓的左上角和右下角的顶点，通过这两个点构建了一个 AABB（轴对齐包围盒）
		int llx = cont.verts[0];
		int llz = cont.verts[2];
		int lli = 0;
		int urx = cont.verts[0];
		int urz = cont.verts[2];
		int uri = 0;
		for (int i = 1; i < cont.nverts; ++i)
		{
			int x = cont.verts[i*4+0];
			int z = cont.verts[i*4+2];
			if (x < llx || (x == llx && z < llz))
			{
				llx = x;
				llz = z;
				lli = i;
			}
			if (x > urx || (x == urx && z > urz))
			{
				urx = x;
				urz = z;
				uri = i;
			}
		}
		cont.npoly = 0;
		cont.poly[cont.npoly++] = (unsigned short)lli;
		cont.poly[cont.npoly++] = (unsigned short)uri;
	}
	
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	// 向简化的图形添加点，直到所有原始点都在误差容忍范围内。
	for (int i = 0; i < cont.npoly; )
	{
		int ii = (i+1) % cont.npoly;
		
		const int ai = (int)cont.poly[i];
		const int ax = (int)cont.verts[ai*4+0];
		const int az = (int)cont.verts[ai*4+2];
		
		const int bi = (int)cont.poly[ii];
		const int bx = (int)cont.verts[bi*4+0];
		const int bz = (int)cont.verts[bi*4+2];
		
		// Find maximum deviation from the segment.
		// 找出线段的最大偏差
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;
		
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		// 按照字典顺序遍历线段，确保在计算最大偏差时得到一致的结果，无论
		// 是正向还是反向遍历线段。
		if (bx > ax || (bx == ax && bz > az))
		{
			cinc = 1;
			ci = (ai+cinc) % cont.nverts;
			endi = bi;
		}
		else
		{
			cinc = cont.nverts-1;
			ci = (bi+cinc) % cont.nverts;
			endi = ai;
		}
		
		// Tessellate only outer edges or edges between areas.
		while (ci != endi)
		{
			// 最短距离
			float d = distancePtSeg(cont.verts[ci*4+0], cont.verts[ci*4+2], ax, az, bx, bz);
			// 记录最大的最短距离，及顶点下标
			if (d > maxd)
			{
				maxd = d;
				maxi = ci;
			}
			ci = (ci+cinc) % cont.nverts;
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		// 如果最大偏差大于可接受的误差阈值，那么就在该线段上添加新的
		// 点来细化线段；否则，跳过该线段，继续处理下一个线段。
		// 通过减少点的数量，可以降低后续处理计算的复杂度和成本。
		// 这个过程基于对轮廓线的形状特征进行分析和判断，并保留重要的
		// 形状细节，并去除不必要的冗余点。
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			cont.npoly++;
			for (int j = cont.npoly-1; j > i; --j)
				cont.poly[j] = cont.poly[j-1];
			cont.poly[i+1] = (unsigned short)maxi;
		}
		else
		{
			++i;
		}
	}
	
	// Remap vertices
	// 重新映射顶点
	int start = 0;
	for (int i = 1; i < cont.npoly; ++i)
		if (cont.poly[i] < cont.poly[start])
			start = i;
	
	// 这里就没什么花头了，基本上就是通过移动拷贝数据，及重新统计
	// 顶点的数量，来实现删除多余的顶点数据
	cont.nverts = 0;
	for (int i = 0; i < cont.npoly; ++i)
	{
		const int j = (start+i) % cont.npoly;
		unsigned char* src = &cont.verts[cont.poly[j]*4];
		unsigned char* dst = &cont.verts[cont.nverts*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		cont.nverts++;
	}
}

// 返回角的高度
static unsigned char getCornerHeight(dtTileCacheLayer& layer,
									 const int x, const int y, const int z,
									 const int walkableClimb,
									 bool& shouldRemove)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	int n = 0;
	
	unsigned char portal = 0xf;
	unsigned char height = 0;
	unsigned char preg = 0xff;
	bool allSameReg = true;
	
	for (int dz = -1; dz <= 0; ++dz)
	{
		for (int dx = -1; dx <= 0; ++dx)
		{
			const int px = x+dx;
			const int pz = z+dz;
			if (px >= 0 && pz >= 0 && px < w && pz < h)
			{
				const int idx  = px + pz*w;
				const int lh = (int)layer.heights[idx];
				if (dtAbs(lh-y) <= walkableClimb && layer.areas[idx] != DT_TILECACHE_NULL_AREA)
				{
					height = dtMax(height, (unsigned char)lh);
					portal &= (layer.cons[idx] >> 4);
					if (preg != 0xff && preg != layer.regs[idx])
						allSameReg = false;
					preg = layer.regs[idx]; 
					n++;
				}
			}
		}
	}
	
	int portalCount = 0;
	for (int dir = 0; dir < 4; ++dir)
		if (portal & (1<<dir))
			portalCount++;
	
	shouldRemove = false;
	if (n > 1 && portalCount == 1 && allSameReg)
	{
		shouldRemove = true;
	}
	
	return height;
}


// TODO: move this somewhere else, once the layer meshing is done.
dtStatus dtBuildTileCacheContours(dtTileCacheAlloc* alloc,
								  dtTileCacheLayer& layer,
								  const int walkableClimb, 	const float maxError,
								  dtTileCacheContourSet& lcset)
{
	dtAssert(alloc);

	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	// 可行走区域的数量
	lcset.nconts = layer.regCount;
	lcset.conts = (dtTileCacheContour*)alloc->alloc(sizeof(dtTileCacheContour)*lcset.nconts);
	if (!lcset.conts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(lcset.conts, 0, sizeof(dtTileCacheContour)*lcset.nconts);
	
	// Allocate temp buffer for contour tracing.
	// 分配临时缓冲区，用于寻路算法数据的追踪
	const int maxTempVerts = (w+h)*2 * 2; // Twice around the layer.
	
	dtFixedArray<unsigned char> tempVerts(alloc, maxTempVerts*4);
	if (!tempVerts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	dtFixedArray<unsigned short> tempPoly(alloc, maxTempVerts);
	if (!tempPoly)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	dtTempContour temp(tempVerts, maxTempVerts, tempPoly, maxTempVerts);
	
	// Find contours.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			// 无法通过的区域，直接跳过
			if (ri == 0xff)
				continue;
			
			dtTileCacheContour& cont = lcset.conts[ri];
			
			// 顶点数大于0，说明已经构建过了？暂时无法理解
			if (cont.nverts > 0)
				continue;
			
			// 可通行区域id
			cont.reg = ri;
			cont.area = layer.areas[idx];
			
			if (!walkContour(layer, x, y, temp))
			{
				// Too complex contour.
				// Note: If you hit here ofte, try increasing 'maxTempVerts'.
				// 如果代码经常执行到这里，试着增加 maxTempVerts 的数量
				// 暂时无法确定是否有效，但直译过滤就是这样子
				return DT_FAILURE | DT_BUFFER_TOO_SMALL;
			}
			
			simplifyContour(temp, maxError);
			
			// Store contour.
			// 保存轮廓线数据
			cont.nverts = temp.nverts;
			if (cont.nverts > 0)
			{
				cont.verts = (unsigned char*)alloc->alloc(sizeof(unsigned char)*4*temp.nverts);
				if (!cont.verts)
					return DT_FAILURE | DT_OUT_OF_MEMORY;
				
				for (int i = 0, j = temp.nverts-1; i < temp.nverts; j=i++)
				{
					unsigned char* dst = &cont.verts[j*4];
					unsigned char* v = &temp.verts[j*4];
					unsigned char* vn = &temp.verts[i*4];
					// 在每个线段的顶点上存储相邻的区域信息，来加快路径规划和导航过程中的区域转换和连接
					unsigned char nei = vn[3]; // The neighbour reg is stored at segment vertex of a segment. 
					bool shouldRemove = false;
					unsigned char lh = getCornerHeight(layer, (int)v[0], (int)v[1], (int)v[2],
													   walkableClimb, shouldRemove);
					
					dst[0] = v[0];
					dst[1] = lh;
					dst[2] = v[2];
					
					// Store portal direction and remove status to the fourth component.
					// 将门户的方向信息存储在顶点的第4个元素中，并移除状态信息。
					dst[3] = 0x0f;
					if (nei != 0xff && nei >= 0xf8)
						dst[3] = nei - 0xf8;
					if (shouldRemove)
						dst[3] |= 0x80;
				}
			}
		}
	}
	
	return DT_SUCCESS;
}	



static const int VERTEX_BUCKET_COUNT2 = (1<<8);

inline int computeVertexHash2(int x, int y, int z)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	return (int)(n & (VERTEX_BUCKET_COUNT2-1));
}

static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, unsigned short* firstVert, unsigned short* nextVert, int& nv)
{
	int bucket = computeVertexHash2(x, 0, z);
	unsigned short i = firstVert[bucket];
	
	while (i != DT_TILECACHE_NULL_IDX)
	{
		const unsigned short* v = &verts[i*3];
		if (v[0] == x && v[2] == z && (dtAbs(v[1] - y) <= 2))
			return i;
		i = nextVert[i]; // next
	}
	
	// Could not find, create new.
	i = (unsigned short)nv; nv++;
	unsigned short* v = &verts[i*3];
	v[0] = x;
	v[1] = y;
	v[2] = z;
	nextVert[i] = firstVert[bucket];
	firstVert[bucket] = i;
	
	return (unsigned short)i;
}


struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static bool buildMeshAdjacency(dtTileCacheAlloc* alloc,
							   unsigned short* polys, const int npolys,
							   const unsigned short* verts, const int nverts,
							   const dtTileCacheContourSet& lcset)
{
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php
	
	const int maxEdgeCount = npolys*MAX_VERTS_PER_POLY;
	dtFixedArray<unsigned short> firstEdge(alloc, nverts + maxEdgeCount);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	dtFixedArray<rcEdge> edges(alloc, maxEdgeCount);
	if (!edges)
		return false;
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = DT_TILECACHE_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*MAX_VERTS_PER_POLY*2];
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
		{
			if (t[j] == DT_TILECACHE_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= MAX_VERTS_PER_POLY || t[j+1] == DT_TILECACHE_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0xff;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*MAX_VERTS_PER_POLY*2];
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
		{
			if (t[j] == DT_TILECACHE_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= MAX_VERTS_PER_POLY || t[j+1] == DT_TILECACHE_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				bool found = false;
				for (unsigned short e = firstEdge[v1]; e != DT_TILECACHE_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						found = true;
						break;
					}
				}
				if (!found)
				{
					// Matching edge not found, it is an open edge, add it.
					rcEdge& edge = edges[edgeCount];
					edge.vert[0] = v1;
					edge.vert[1] = v0;
					edge.poly[0] = (unsigned short)i;
					edge.polyEdge[0] = (unsigned short)j;
					edge.poly[1] = (unsigned short)i;
					edge.polyEdge[1] = 0xff;
					// Insert edge
					nextEdge[edgeCount] = firstEdge[v1];
					firstEdge[v1] = (unsigned short)edgeCount;
					edgeCount++;
				}
			}
		}
	}
	
	// Mark portal edges.
	for (int i = 0; i < lcset.nconts; ++i)
	{
		dtTileCacheContour& cont = lcset.conts[i];
		if (cont.nverts < 3)
			continue;
		
		for (int j = 0, k = cont.nverts-1; j < cont.nverts; k=j++)
		{
			const unsigned char* va = &cont.verts[k*4];
			const unsigned char* vb = &cont.verts[j*4];
			const unsigned char dir = va[3] & 0xf;
			if (dir == 0xf)
				continue;
			
			if (dir == 0 || dir == 2)
			{
				// Find matching vertical edge
				const unsigned short x = (unsigned short)va[0];
				unsigned short zmin = (unsigned short)va[2];
				unsigned short zmax = (unsigned short)vb[2];
				if (zmin > zmax)
					dtSwap(zmin, zmax);
				
				for (int m = 0; m < edgeCount; ++m)
				{
					rcEdge& e = edges[m];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[0] == x && evb[0] == x)
					{
						unsigned short ezmin = eva[2];
						unsigned short ezmax = evb[2];
						if (ezmin > ezmax)
							dtSwap(ezmin, ezmax);
						if (overlapRangeExl(zmin,zmax, ezmin, ezmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
			else
			{
				// Find matching vertical edge
				const unsigned short z = (unsigned short)va[2];
				unsigned short xmin = (unsigned short)va[0];
				unsigned short xmax = (unsigned short)vb[0];
				if (xmin > xmax)
					dtSwap(xmin, xmax);
				for (int m = 0; m < edgeCount; ++m)
				{
					rcEdge& e = edges[m];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[2] == z && evb[2] == z)
					{
						unsigned short exmin = eva[0];
						unsigned short exmax = evb[0];
						if (exmin > exmax)
							dtSwap(exmin, exmax);
						if (overlapRangeExl(xmin,xmax, exmin, exmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
		}
	}
	
	
	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*MAX_VERTS_PER_POLY*2];
			unsigned short* p1 = &polys[e.poly[1]*MAX_VERTS_PER_POLY*2];
			p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = e.poly[1];
			p1[MAX_VERTS_PER_POLY + e.polyEdge[1]] = e.poly[0];
		}
		else if (e.polyEdge[1] != 0xff)
		{
			unsigned short* p0 = &polys[e.poly[0]*MAX_VERTS_PER_POLY*2];
			p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = 0x8000 | (unsigned short)e.polyEdge[1];
		}
		
	}
	
	return true;
}


// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) - ((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]);
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
// 返回 true，当 c 严格位于有向线段 ab 的左侧时。
inline bool left(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) < 0;
}

// 返回 true，c 位于有向线段的左侧，或者在线段上。
inline bool leftOn(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) <= 0;
}

// 判断是否为共线，当三个点的有向面积等于0时，就说明是共线。
inline bool collinear(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
// 返回 true， 线段 ab 和 cd 在内部相交，它们在两个线段间共享一个点。
// 通过使用严格的左侧相交来确保交点的适用性。
static bool intersectProp(const unsigned char* a, const unsigned char* b,
						  const unsigned char* c, const unsigned char* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return xorb(left(a,b,c), left(a,b,d)) && xorb(left(c,d,a), left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies 
// on the closed segement ab.
static bool between(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	// 如果线段 ab 不是垂直线，在 x 轴上进行介数检查；否则，在 y 轴上进行介数的检查。
	// 来判断线段的相交性
	if (a[0] != b[0])
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
// 返回 true，如果线段 ab 和 cd 相交，无论是适当相交（内部交点）还是不适当相交（边界交点）
// 内部交点：表示在空间中相互穿插或交织
// 边界交点：表示在边界上有交叉时
static bool intersect(const unsigned char* a, const unsigned char* b,
					  const unsigned char* c, const unsigned char* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const unsigned char* a, const unsigned char* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
// 如果线段 (v_i, v_j) 是多边形 P 的一个有效的内部或外部对角线，返回 T。（需要排除与顶点 v_i 和 v_j 相邻的边） 
static bool diagonalie(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* d0 = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* d1 = &verts[(indices[j] & 0x7fff) * 4];
	
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i or j
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const unsigned char* p0 = &verts[(indices[k] & 0x7fff) * 4];
			const unsigned char* p1 = &verts[(indices[k1] & 0x7fff) * 4];
			
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;
			
			if (intersect(d0, d1, p0, p1))
				return false;
		}
	}
	return true;
}

// Returns true iff the diagonal (i,j) is strictly internal to the 
// polygon P in the neighborhood of the i endpoint.
// 当对角线（i，j）严格地位于端点附近的多边形P内部时，返回 true
static bool	inCone(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* pi = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* pj = &verts[(indices[j] & 0x7fff) * 4];
	const unsigned char* pi1 = &verts[(indices[next(i, n)] & 0x7fff) * 4];
	const unsigned char* pin1 = &verts[(indices[prev(i, n)] & 0x7fff) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	// 如果 p[i] 是一个凸点，i+1 在线段左边 or 在线段 (i-1,i) 上。
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	// 假设 (i-1,i,i+1) 不共线，否则 P[i] 是凹点
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
// 返回 true，如果线段 (v_i, v_j) 是 P 的一个内部对角线
static bool diagonal(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

static int triangulate(int n, const unsigned char* verts, unsigned short* indices, unsigned short* tris)
{
	int ntris = 0;
	unsigned short* dst = tris;
	
	// The last bit of the index is used to indicate if the vertex can be removed.
	// 索引的最后一个比特位，用于表示顶点是否可以被移除
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n);
		int i2 = next(i1, n);
		// 检查内部对角线
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x8000; // 内部对角线标识符
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			// 内部对角线
			if (indices[i1] & 0x8000)
			{
				const unsigned char* p0 = &verts[(indices[i] & 0x7fff) * 4];
				const unsigned char* p2 = &verts[(indices[next(i1, n)] & 0x7fff) * 4];
				
				const int dx = (int)p2[0] - (int)p0[0];
				const int dz = (int)p2[2] - (int)p0[2];
				const int len = dx*dx + dz*dz;
				if (minLen < 0 || len < minLen)
				{
					minLen = len;
					mini = i;
				}
			}
		}
		// 作者认为这是一个意外，不应该进入下面的流程处理
		if (mini == -1)
		{
			// Should not happen.
			/*			printf("mini == -1 ntris=%d n=%d\n", ntris, n);
			 for (int i = 0; i < n; i++)
			 {
			 printf("%d ", indices[i] & 0x0fffffff);
			 }
			 printf("\n");*/
			return -ntris;
		}
		
		int i = mini;
		int i1 = next(i, n);
		int i2 = next(i1, n);
		
		*dst++ = indices[i] & 0x7fff;
		*dst++ = indices[i1] & 0x7fff;
		*dst++ = indices[i2] & 0x7fff;
		ntris++;
		
		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		// 通过移动拷贝来删除索引
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		// Update diagonal flags.
		// 更新对角线标识符
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x8000;
		else
			indices[i] &= 0x7fff;
		// 更新对角线标识符
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x8000;
		else
			indices[i1] &= 0x7fff;
	}
	
	// Append the remaining triangle.
	// 附加剩余的三角形
	*dst++ = indices[0] & 0x7fff;
	*dst++ = indices[1] & 0x7fff;
	*dst++ = indices[2] & 0x7fff;
	ntris++;
	
	return ntris;
}

// 返回多边形的顶点数
static int countPolyVerts(const unsigned short* p)
{
	for (int i = 0; i < MAX_VERTS_PER_POLY; ++i)
		if (p[i] == DT_TILECACHE_NULL_IDX)
			return i;
	return MAX_VERTS_PER_POLY;
}

// 返回 true，点 c 在有向线段 ab 的左侧
inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
	((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

// 返回两个顶点之间的平方距离
static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb)
{
	// 获得多边形 a 和 b 的顶点数
	const int na = countPolyVerts(pa);
	const int nb = countPolyVerts(pb);
	
	// If the merged polygon would be too big, do not merge.
	// 如果两个多边形的顶点总数减2大于MAX_VERTS_PER_POLY，不执行合并操作
	if (na+nb-2 > MAX_VERTS_PER_POLY)
		return -1;
	
	// Check if the polygons share an edge.
	// 检查多边形是否存在一条共享边，ea 和 eb 两个顶点，组成一条共享边
	// 存储的是两个顶点的下标
	ea = -1;
	eb = -1;
	
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		// 为什么对当前下标+1后，求模运算？
		// 处理多边形的闭合性：多边形的最后一个顶点后面通常是第一个顶点，通过对索引进行求模运算
		// 可以确保在达到最后一个顶点时，下一个索引会回到第一个顶点。
		// 兼容性和通用性：不需要考虑多边形顶点的奇偶数的情况，逻辑上统一，易于理解。
		// 简化边界条件处理：不需要额外的条件判断来处理最后一个顶点后面的情况，降低了代码复杂度。
		unsigned short va1 = pa[(i+1) % na];
		if (va0 > va1)
			dtSwap(va0, va1);
		for (int j = 0; j < nb; ++j)
		{
			unsigned short vb0 = pb[j];
			unsigned short vb1 = pb[(j+1) % nb];
			if (vb0 > vb1)
				dtSwap(vb0, vb1);
			if (va0 == vb0 && va1 == vb1)
			{
				ea = i;
				eb = j;
				break;
			}
		}
	}
	
	// No common edge, cannot merge.
	// 没有共享边，不能合并，即不相邻
	if (ea == -1 || eb == -1)
		return -1;
	
	// Check to see if the merged polygon would be convex.
	// 检查合并后的多边形是否为凸多边形
	unsigned short va, vb, vc;
	
	va = pa[(ea+na-1) % na];
	vb = pa[ea];
	vc = pb[(eb+2) % nb];
	// 判断点c，在有向线段ab的左侧
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pb[(eb+nb-1) % nb];
	vb = pb[eb];
	vc = pa[(ea+2) % na];
	// 判断点c，在有向线段ab的左侧
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pa[ea];
	vb = pa[(ea+1)%na];
	
	int dx = (int)verts[va*3+0] - (int)verts[vb*3+0];
	int dy = (int)verts[va*3+2] - (int)verts[vb*3+2];
	
	return dx*dx + dy*dy;
}

// 合并多边
// pa 和 pb 是待合并的两个多边形
// ea 和 eb 是两条最优边的下标
static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb)
{
	unsigned short tmp[MAX_VERTS_PER_POLY*2];
	
	const int na = countPolyVerts(pa);
	const int nb = countPolyVerts(pb);
	
	// Merge polygons.
	memset(tmp, 0xff, sizeof(unsigned short)*MAX_VERTS_PER_POLY*2);
	int n = 0;
	// Add pa
	for (int i = 0; i < na-1; ++i)
		tmp[n++] = pa[(ea+1+i) % na];
	// Add pb
	for (int i = 0; i < nb-1; ++i)
		tmp[n++] = pb[(eb+1+i) % nb];
	
	memcpy(pa, tmp, sizeof(unsigned short)*MAX_VERTS_PER_POLY);
}


static void pushFront(unsigned short v, unsigned short* arr, int& an)
{
	an++;
	for (int i = an-1; i > 0; --i)
		arr[i] = arr[i-1];
	arr[0] = v;
}

static void pushBack(unsigned short v, unsigned short* arr, int& an)
{
	arr[an] = v;
	an++;
}

// 返回 true，顶点可以被删除
static bool canRemoveVertex(dtTileCachePolyMesh& mesh, const unsigned short rem)
{
	// Count number of polygons to remove.
	// 计算删除的多边形数量
	int numTouchedVerts = 0;
	// 剩余的边数
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// 获得多边形 p 的顶点数量
		const int nv = countPolyVerts(p);
		int numRemoved = 0;
		int numVerts = 0;
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
			{
				numTouchedVerts++;
				numRemoved++;
			}
			numVerts++;
		}
		if (numRemoved)
		{
			numRemainingEdges += numVerts-(numRemoved+1);
		}
	}
	
	// There would be too few edges remaining to create a polygon.
	// This can happen for example when a tip of a triangle is marked
	// as deletion, but there are no other polys that share the vertex.
	// In this case, the vertex should not be removed.
	// 剩下的边数太少，无法创建多边形。
	// 例如，当三角形的顶点被标记为删除，但没有其它多边形共享该顶点时，就会发生这种情况.
	// 在这种情况下，顶点不能被删除。
	if (numRemainingEdges <= 2)
		return false;
	
	// Check that there is enough memory for the test.
	// 检测是否有足够的内存用于测试
	const int maxEdges = numTouchedVerts*2;
	if (maxEdges > MAX_REM_EDGES)
		return false;
	
	// Find edges which share the removed vertex.
	// 找到与被移除顶点相邻的边
	unsigned short edges[MAX_REM_EDGES];
	int nedges = 0;
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// 获得多边形p的顶点数量
		const int nv = countPolyVerts(p);
		
		// Collect edges which touches the removed vertex.
		// 收集与被删除顶点相邻的边
		for (int j = 0, k = nv-1; j < nv; k = j++)
		{
			if (p[j] == rem || p[k] == rem)
			{
				// Arrange edge so that a=rem.
				// 将边重新排列，a = 被移除的顶点
				int a = p[j], b = p[k];
				if (b == rem)
					dtSwap(a,b);
				
				// Check if the edge exists
				// 检测边是否存在
				bool exists = false;
				for (int m = 0; m < nedges; ++m)
				{
					unsigned short* e = &edges[m*3];
					if (e[1] == b)
					{
						// Exists, increment vertex share count.
						// 存在，增加顶点共享计数
						e[2]++;
						exists = true;
					}
				}
				// Add new edge.
				// 边不存在，创建一条新边
				if (!exists)
				{
					unsigned short* e = &edges[nedges*3];
					e[0] = (unsigned short)a;
					e[1] = (unsigned short)b;
					e[2] = 1;
					nedges++;
				}
			}
		}
	}
	
	// There should be no more than 2 open edges.
	// This catches the case that two non-adjacent polygons
	// share the removed vertex. In that case, do not remove the vertex.
	// 确保多边形没有超过两条开放边。
	// 这种情况发生在两个非相邻的多边形共享被移除的顶点的情况。
	// 在这种情况下，不能删除顶点，否则会发生断裂。
	// 为什么不相邻的多边形会拥有共享顶点，我们可以想象我们有3个多边形，它们两两相邻。
	// 在这种情况下，为了优化，中间的多边形会被删除掉，因此就出现了非相邻多边形，共享顶点的情况。
	int numOpenEdges = 0;
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+2] < 2)
			numOpenEdges++;
	}
	if (numOpenEdges > 2)
		return false;
	
	return true;
}

// 删除顶点
// mesh
// rem 待删除的顶点下标
// maxTris 三角形数量上限
static dtStatus removeVertex(dtTileCachePolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	// Count number of polygons to remove.
	// 计算被删除的多边形数量
	int numRemovedVerts = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// 获得多边形p的顶点数量
		const int nv = countPolyVerts(p);
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
				numRemovedVerts++;
		}
	}
	
	int nedges = 0;
	unsigned short edges[MAX_REM_EDGES*3];
	int nhole = 0;
	unsigned short hole[MAX_REM_EDGES];
	int nharea = 0;
	unsigned short harea[MAX_REM_EDGES];
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// 获得多边形p的顶点数量
		const int nv = countPolyVerts(p);
		bool hasRem = false;
		for (int j = 0; j < nv; ++j)
			if (p[j] == rem) hasRem = true;
		if (hasRem)
		{
			// Collect edges which does not touch the removed vertex.
			// 收集不与被删除顶点相接触的边
			for (int j = 0, k = nv-1; j < nv; k = j++)
			{
				if (p[j] != rem && p[k] != rem)
				{
					if (nedges >= MAX_REM_EDGES)
						return DT_FAILURE | DT_BUFFER_TOO_SMALL;
					unsigned short* e = &edges[nedges*3];
					e[0] = p[k];
					e[1] = p[j];
					e[2] = mesh.areas[i];
					nedges++;
				}
			}
			// Remove the polygon.
			// 删除多边形，通过移动内存数据来实现，然后减少多边形计数
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*MAX_VERTS_PER_POLY*2];
			memcpy(p,p2,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			memset(p+MAX_VERTS_PER_POLY,0xff,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			mesh.areas[i] = mesh.areas[mesh.npolys-1];
			mesh.npolys--;
			--i;
		}
	}
	
	// Remove vertex.
	// 删除多边形，通过移动内存数据来实现，然后减少顶点计数
	for (int i = (int)rem; i < mesh.nverts - 1; ++i)
	{
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0];
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1];
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2];
	}
	mesh.nverts--;
	
	// Adjust indices to match the removed vertex layout.
	// 调整索引以匹配移除顶点后的顶点布局
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	// 递减边引用计数
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+0] > rem) edges[i*3+0]--;
		if (edges[i*3+1] > rem) edges[i*3+1]--;
	}
	
	if (nedges == 0)
		return DT_SUCCESS;
	
	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	// 从一个顶点开始，不断将连接的线段添加到孔洞的起点和终点。
	// 这里 “孔洞” 指的是由于简化多边形后，许多顶点可能已经被删除
	// 从而，导致多边形不在是封闭的状态，通过线段将这些连接区域填补
	// 后，再次形成新的封闭式的多边形。
	pushBack(edges[0], hole, nhole);
	pushBack(edges[2], harea, nharea);
	
	while (nedges)
	{
		bool match = false;
		
		for (int i = 0; i < nedges; ++i)
		{
			const unsigned short ea = edges[i*3+0];
			const unsigned short eb = edges[i*3+1];
			const unsigned short a = edges[i*3+2];
			bool add = false;
			if (hole[0] == eb)
			{
				// The segment matches the beginning of the hole boundary.
				// 该线段与孔洞的边界的起点部分匹配
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushFront(ea, hole, nhole);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				// The segment matches the end of the hole boundary.
				// 该线段与孔洞的边界的终点部分匹配
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushBack(eb, hole, nhole);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				// The edge segment was added, remove it.
				// 边缘段（多边形边界的线段）已经被添加到合适的位置后，线段
				// 就可以从数据结构中移除了。
				edges[i*3+0] = edges[(nedges-1)*3+0];
				edges[i*3+1] = edges[(nedges-1)*3+1];
				edges[i*3+2] = edges[(nedges-1)*3+2];
				--nedges;
				match = true;
				--i;
			}
		}
		
		if (!match)
			break;
	}
	
	
	unsigned short tris[MAX_REM_EDGES*3];
	unsigned char tverts[MAX_REM_EDGES*3];
	unsigned short tpoly[MAX_REM_EDGES*3];
	
	// Generate temp vertex array for triangulation.
	// 创建三角形的临时顶点数据
	for (int i = 0; i < nhole; ++i)
	{
		const unsigned short pi = hole[i];
		tverts[i*4+0] = (unsigned char)mesh.verts[pi*3+0];
		tverts[i*4+1] = (unsigned char)mesh.verts[pi*3+1];
		tverts[i*4+2] = (unsigned char)mesh.verts[pi*3+2];
		tverts[i*4+3] = 0;
		tpoly[i] = (unsigned short)i;
	}
	
	// Triangulate the hole.
	// 将孔洞进行剖分，将其分解为一系列三角形，表示为一组连接。
	int ntris = triangulate(nhole, tverts, tpoly, tris);
	if (ntris < 0)
	{
		// TODO: issue warning!
		ntris = -ntris;
	}
	
	if (ntris > MAX_REM_EDGES)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	unsigned short polys[MAX_REM_EDGES*MAX_VERTS_PER_POLY];
	unsigned char pareas[MAX_REM_EDGES];
	
	// Build initial polygons.
	// 构建初始多边形
	int npolys = 0;
	memset(polys, 0xff, ntris*MAX_VERTS_PER_POLY*sizeof(unsigned short));
	for (int j = 0; j < ntris; ++j)
	{
		unsigned short* t = &tris[j*3];
		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
		{
			polys[npolys*MAX_VERTS_PER_POLY+0] = hole[t[0]];
			polys[npolys*MAX_VERTS_PER_POLY+1] = hole[t[1]];
			polys[npolys*MAX_VERTS_PER_POLY+2] = hole[t[2]];
			pareas[npolys] = (unsigned char)harea[t[0]];
			npolys++;
		}
	}
	if (!npolys)
		return DT_SUCCESS;
	
	// Merge polygons.
	// 合并多边形
	int maxVertsPerPoly = MAX_VERTS_PER_POLY;
	if (maxVertsPerPoly > 3)
	{
		for (;;)
		{
			// Find best polygons to merge.
			// 找到最优多边形，进行合并操作
			int bestMergeVal = 0;
			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
			
			for (int j = 0; j < npolys-1; ++j)
			{
				unsigned short* pj = &polys[j*MAX_VERTS_PER_POLY];
				for (int k = j+1; k < npolys; ++k)
				{
					unsigned short* pk = &polys[k*MAX_VERTS_PER_POLY];
					int ea, eb;
					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
					if (v > bestMergeVal)
					{
						bestMergeVal = v;
						bestPa = j;
						bestPb = k;
						bestEa = ea;
						bestEb = eb;
					}
				}
			}
			
			if (bestMergeVal > 0)
			{
				// Found best, merge.
				unsigned short* pa = &polys[bestPa*MAX_VERTS_PER_POLY];
				unsigned short* pb = &polys[bestPb*MAX_VERTS_PER_POLY];
				mergePolys(pa, pb, bestEa, bestEb);
				memcpy(pb, &polys[(npolys-1)*MAX_VERTS_PER_POLY], sizeof(unsigned short)*MAX_VERTS_PER_POLY);
				pareas[bestPb] = pareas[npolys-1];
				npolys--;
			}
			else
			{
				// Could not merge any polygons, stop.
				break;
			}
		}
	}
	
	// Store polygons.
	// 存储多边形数据结构
	for (int i = 0; i < npolys; ++i)
	{
		if (mesh.npolys >= maxTris) break;
		unsigned short* p = &mesh.polys[mesh.npolys*MAX_VERTS_PER_POLY*2];
		memset(p,0xff,sizeof(unsigned short)*MAX_VERTS_PER_POLY*2);
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
			p[j] = polys[i*MAX_VERTS_PER_POLY+j];
		mesh.areas[mesh.npolys] = pareas[i];
		mesh.npolys++;
		if (mesh.npolys > maxTris)
			return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	}
	
	return DT_SUCCESS;
}


dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc* alloc,
								  dtTileCacheContourSet& lcset,
								  dtTileCachePolyMesh& mesh)
{
	dtAssert(alloc);
	
	// 顶点的总数
	int maxVertices = 0;
	// 三角形总数
	int maxTris = 0;
	// 每个轮廓可持有的顶点数上限
	int maxVertsPerCont = 0;
	for (int i = 0; i < lcset.nconts; ++i)
	{
		// Skip null contours.
		if (lcset.conts[i].nverts < 3) continue;
		maxVertices += lcset.conts[i].nverts;
		maxTris += lcset.conts[i].nverts - 2;
		maxVertsPerCont = dtMax(maxVertsPerCont, lcset.conts[i].nverts);
	}

	// TODO: warn about too many vertices?
	
	mesh.nvp = MAX_VERTS_PER_POLY;
	
	dtFixedArray<unsigned char> vflags(alloc, maxVertices);
	if (!vflags)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(vflags, 0, maxVertices);
	
	mesh.verts = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxVertices*3);
	if (!mesh.verts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	mesh.polys = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxTris*MAX_VERTS_PER_POLY*2);
	if (!mesh.polys)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	mesh.areas = (unsigned char*)alloc->alloc(sizeof(unsigned char)*maxTris);
	if (!mesh.areas)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	mesh.flags = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxTris);
	if (!mesh.flags)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// Just allocate and clean the mesh flags array. The user is resposible for filling it.
	memset(mesh.flags, 0, sizeof(unsigned short) * maxTris);
		
	mesh.nverts = 0;
	mesh.npolys = 0;
	
	memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices*3);
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*MAX_VERTS_PER_POLY*2);
	memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);
	
	unsigned short firstVert[VERTEX_BUCKET_COUNT2];
	for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
		firstVert[i] = DT_TILECACHE_NULL_IDX;
	
	dtFixedArray<unsigned short> nextVert(alloc, maxVertices);
	if (!nextVert)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(nextVert, 0, sizeof(unsigned short)*maxVertices);
	
	// 顶点的索引数组
	dtFixedArray<unsigned short> indices(alloc, maxVertsPerCont);
	if (!indices)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// 三角形数组
	dtFixedArray<unsigned short> tris(alloc, maxVertsPerCont*3);
	if (!tris)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// 多边形数组
	dtFixedArray<unsigned short> polys(alloc, maxVertsPerCont*MAX_VERTS_PER_POLY);
	if (!polys)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		dtTileCacheContour& cont = lcset.conts[i];
		
		// Skip null contours.
		if (cont.nverts < 3)
			continue;
		
		// Triangulate contour
		// 三角形轮廓
		for (int j = 0; j < cont.nverts; ++j)
			indices[j] = (unsigned short)j;
		
		int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
		if (ntris <= 0)
		{
			// TODO: issue warning!
			ntris = -ntris;
		}
		
		// Add and merge vertices.
		for (int j = 0; j < cont.nverts; ++j)
		{
			const unsigned char* v = &cont.verts[j*4];
			// 添加一个新顶点，或找到一个顶点，返回其索引
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			// 0x80 删除标识符
			if (v[3] & 0x80)
			{
				// This vertex should be removed.
				vflags[indices[j]] = 1;
			}
		}
		
		// Build initial polygons.
		// 构建初始多边形
		int npolys = 0;
		memset(polys, 0xff, sizeof(unsigned short) * maxVertsPerCont * MAX_VERTS_PER_POLY);
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned short* t = &tris[j*3];
			if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
			{
				polys[npolys*MAX_VERTS_PER_POLY+0] = indices[t[0]];
				polys[npolys*MAX_VERTS_PER_POLY+1] = indices[t[1]];
				polys[npolys*MAX_VERTS_PER_POLY+2] = indices[t[2]];
				npolys++;
			}
		}
		if (!npolys)
			continue;
		
		// Merge polygons.
		int maxVertsPerPoly =MAX_VERTS_PER_POLY ;
		if (maxVertsPerPoly > 3)
		{
			for(;;)
			{
				// Find best polygons to merge.
				// 找到最优多边形进行合并
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*MAX_VERTS_PER_POLY];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*MAX_VERTS_PER_POLY];
						int ea, eb;
						// v = 两个顶点之间的平方距离
						int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
						if (v > bestMergeVal)
						{
							bestMergeVal = v;
							bestPa = j;
							bestPb = k;
							bestEa = ea;
							bestEb = eb;
						}
					}
				}
				
				// 找到最佳合并的多边形
				if (bestMergeVal > 0)
				{
					// Found best, merge.
					unsigned short* pa = &polys[bestPa*MAX_VERTS_PER_POLY];
					unsigned short* pb = &polys[bestPb*MAX_VERTS_PER_POLY];
					// 合并后的数据，存储在 pa 上
					mergePolys(pa, pb, bestEa, bestEb);
					memcpy(pb, &polys[(npolys-1)*MAX_VERTS_PER_POLY], sizeof(unsigned short)*MAX_VERTS_PER_POLY);
					npolys--;
				}
				else
				{
					// Could not merge any polygons, stop.
					// 不能合并任何多边形，退出循环
					break;
				}
			}
		}
		
		// Store polygons.
		// 存储多边形
		for (int j = 0; j < npolys; ++j)
		{
			unsigned short* p = &mesh.polys[mesh.npolys*MAX_VERTS_PER_POLY*2];
			unsigned short* q = &polys[j*MAX_VERTS_PER_POLY];
			for (int k = 0; k < MAX_VERTS_PER_POLY; ++k)
				p[k] = q[k];
			mesh.areas[mesh.npolys] = cont.area;
			mesh.npolys++;
			if (mesh.npolys > maxTris)
				return DT_FAILURE | DT_BUFFER_TOO_SMALL;
		}
	}
	
	
	// Remove edge vertices.
	// 删除边的顶点
	for (int i = 0; i < mesh.nverts; ++i)
	{
		// vflags[i] != 0，那么就需要删除
		if (vflags[i])
		{
			// 判断顶点是否可以被删除
			if (!canRemoveVertex(mesh, (unsigned short)i))
				continue;
			dtStatus status = removeVertex(mesh, (unsigned short)i, maxTris);
			if (dtStatusFailed(status))
				return status;
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			for (int j = i; j < mesh.nverts; ++j)
				vflags[j] = vflags[j+1];
			--i;
		}
	}
	
	// Calculate adjacency.
	if (!buildMeshAdjacency(alloc, mesh.polys, mesh.npolys, mesh.verts, mesh.nverts, lcset))
		return DT_FAILURE | DT_OUT_OF_MEMORY;
		
	return DT_SUCCESS;
}

dtStatus dtMarkCylinderArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
							const float* pos, const float radius, const float height, const unsigned char areaId)
{
	// 在圆柱体区域检查中使用 AABB（轴对齐包围盒）检测来近似表示
	// 是因为矩形边界更容易进行碰撞和重叠检测。但这种表示在某些情
	// 况下，并不完全精准。这是在效率和速度上的一种取舍。

	// 这里的 bmin 和 bmax 表示的是物体的 AABB（轴对齐包围盒）
	// 边界，是一个近似值，通过半径计算获得
	float bmin[3], bmax[3];
	bmin[0] = pos[0] - radius;
	bmin[1] = pos[1];
	bmin[2] = pos[2] - radius;
	bmax[0] = pos[0] + radius;
	bmax[1] = pos[1] + height;
	bmax[2] = pos[2] + radius;
	const float r2 = dtSqr(radius/cs + 0.5f);
	// 图层的尺寸，可以理解为画布、地图等类似概念的大小（如像素宽高：1024 * 1024等等）
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	// 缩放因子
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;
	
	// 在 xz 轴平面上距离包围盒起点（矩形的左上角）的位置的缩放
	const float px = (pos[0]-orig[0])*ics;
	const float pz = (pos[2]-orig[2])*ics;
	
	int minx = (int)dtMathFloorf((bmin[0]-orig[0])*ics);
	int miny = (int)dtMathFloorf((bmin[1]-orig[1])*ich);
	int minz = (int)dtMathFloorf((bmin[2]-orig[2])*ics);
	int maxx = (int)dtMathFloorf((bmax[0]-orig[0])*ics);
	int maxy = (int)dtMathFloorf((bmax[1]-orig[1])*ich);
	int maxz = (int)dtMathFloorf((bmax[2]-orig[2])*ics);

	// 表示圆柱体在 AABB（包围盒）的左侧，这是基于网格的小标从零开始的原因
	if (maxx < 0) return DT_SUCCESS;
	// 表示圆柱体在 AABB（包围盒）的右侧，超出了有效范围
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	// 下面的代码使用了更为精准的圆柱体检测
	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const float dx = (float)(x+0.5f) - px;
			const float dz = (float)(z+0.5f) - pz;
			if (dx*dx + dz*dz > r2)
				continue;
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* bmin, const float* bmax, const unsigned char areaId)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;

	int minx = (int)floorf((bmin[0]-orig[0])*ics);
	int miny = (int)floorf((bmin[1]-orig[1])*ich);
	int minz = (int)floorf((bmin[2]-orig[2])*ics);
	int maxx = (int)floorf((bmax[0]-orig[0])*ics);
	int maxy = (int)floorf((bmax[1]-orig[1])*ich);
	int maxz = (int)floorf((bmax[2]-orig[2])*ics);
	
	if (maxx < 0) return DT_SUCCESS;
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* center, const float* halfExtents, const float* rotAux, const unsigned char areaId)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;

	float cx = (center[0] - orig[0])*ics;
	float cz = (center[2] - orig[2])*ics;
	
	float maxr = 1.41f*dtMax(halfExtents[0], halfExtents[2]);
	int minx = (int)floorf(cx - maxr*ics);
	int maxx = (int)floorf(cx + maxr*ics);
	int minz = (int)floorf(cz - maxr*ics);
	int maxz = (int)floorf(cz + maxr*ics);
	int miny = (int)floorf((center[1]-halfExtents[1]-orig[1])*ich);
	int maxy = (int)floorf((center[1]+halfExtents[1]-orig[1])*ich);

	if (maxx < 0) return DT_SUCCESS;
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	float xhalf = halfExtents[0]*ics + 0.5f;
	float zhalf = halfExtents[2]*ics + 0.5f;

	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{			
			float x2 = 2.0f*(float(x) - cx);
			float z2 = 2.0f*(float(z) - cz);
			float xrot = rotAux[1]*x2 + rotAux[0]*z2;
			if (xrot > xhalf || xrot < -xhalf)
				continue;
			float zrot = rotAux[1]*z2 - rotAux[0]*x2;
			if (zrot > zhalf || zrot < -zhalf)
				continue;
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor* comp,
							   dtTileCacheLayerHeader* header,
							   const unsigned char* heights,
							   const unsigned char* areas,
							   const unsigned char* cons,
							   unsigned char** outData, int* outDataSize)
{
	// 以4字节对齐
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	// 计算格子的大小
	const int gridSize = (int)header->width * (int)header->height;
	// 计算数据的最大缓冲区大小
	const int maxDataSize = headerSize + comp->maxCompressedSize(gridSize*3);
	unsigned char* data = (unsigned char*)dtAlloc(maxDataSize, DT_ALLOC_PERM);
	if (!data)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(data, 0, maxDataSize);
	
	// Store header
	// 存储瓦片缓存的层级的头部信息
	memcpy(data, header, sizeof(dtTileCacheLayerHeader));
	
	// Concatenate grid data for compression.
	const int bufferSize = gridSize*3;
	// 分配网格数据
	unsigned char* buffer = (unsigned char*)dtAlloc(bufferSize, DT_ALLOC_TEMP);
	if (!buffer)
	{
		dtFree(data);
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	// 二进制数据分为三个部分：高度信息、区域信息、连接信息
	memcpy(buffer, heights, gridSize);
	memcpy(buffer+gridSize, areas, gridSize);
	memcpy(buffer+gridSize*2, cons, gridSize);
	
	// Compress
	// 跳过瓦片缓存的头部信息
	unsigned char* compressed = data + headerSize;
	// 计算出压缩数据的长度信息
	const int maxCompressedSize = maxDataSize - headerSize;
	int compressedSize = 0;
	// 实际压缩的数据大小
	dtStatus status = comp->compress(buffer, bufferSize, compressed, maxCompressedSize, &compressedSize);
	if (dtStatusFailed(status))
	{
		dtFree(buffer);
		dtFree(data);
		return status;
	}

	// 压缩后的数据，及实际压缩数据的大小
	*outData = data;
	*outDataSize = headerSize + compressedSize;
	
	dtFree(buffer);
	
	return DT_SUCCESS;
}

// 释放瓦片缓存的层级数据所占用的内存
void dtFreeTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer)
{
	dtAssert(alloc);
	// The layer is allocated as one conitguous blob of data.
	alloc->free(layer);
}

dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheCompressor* comp,
									unsigned char* compressed, const int compressedSize,
									dtTileCacheLayer** layerOut)
{
	dtAssert(alloc);
	dtAssert(comp);

	// 检查 layerOut 是否已经在外部分配了内存（即初始化过的）
	if (!layerOut)
		return DT_FAILURE | DT_INVALID_PARAM;
	// 检查 compressed 是否有效，待解压缩的数据
	if (!compressed)
		return DT_FAILURE | DT_INVALID_PARAM;

	*layerOut = 0;
	// 获取瓦片缓存的层级头部信息
	dtTileCacheLayerHeader* compressedHeader = (dtTileCacheLayerHeader*)compressed;
	// 检查文件头格式、数据版本号
	if (compressedHeader->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (compressedHeader->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	const int layerSize = dtAlign4(sizeof(dtTileCacheLayer));
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = (int)compressedHeader->width * (int)compressedHeader->height;
	const int bufferSize = layerSize + headerSize + gridSize*4;
	
	// 分配足够大的缓冲区以容纳高度、区域、连接信息
	unsigned char* buffer = (unsigned char*)alloc->alloc(bufferSize);
	if (!buffer)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(buffer, 0, bufferSize);

	dtTileCacheLayer* layer = (dtTileCacheLayer*)buffer;
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)(buffer + layerSize);
	unsigned char* grids = buffer + layerSize + headerSize;
	const int gridsSize = bufferSize - (layerSize + headerSize); 
	
	// Copy header
	// 拷贝头部信息
	memcpy(header, compressedHeader, headerSize);
	// Decompress grid.
	int size = 0;
	dtStatus status = comp->decompress(compressed+headerSize, compressedSize-headerSize,
									   grids, gridsSize, &size);
	if (dtStatusFailed(status))
	{
		alloc->free(buffer);
		return status;
	}
	
	layer->header = header;
	layer->heights = grids;
	layer->areas = grids + gridSize;
	layer->cons = grids + gridSize*2;
	layer->regs = grids + gridSize*3;
	
	*layerOut = layer;
	
	return DT_SUCCESS;
}


// 瓦片数据结构的存储格式
bool dtTileCacheHeaderSwapEndian(unsigned char* data, const int dataSize)
{
	dtIgnoreUnused(dataSize);
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
	
	int swappedMagic = DT_TILECACHE_MAGIC;
	int swappedVersion = DT_TILECACHE_VERSION;
	dtSwapEndian(&swappedMagic);
	dtSwapEndian(&swappedVersion);
	
	if ((header->magic != DT_TILECACHE_MAGIC || header->version != DT_TILECACHE_VERSION) &&
		(header->magic != swappedMagic || header->version != swappedVersion))
	{
		return false;
	}
	
	dtSwapEndian(&header->magic);
	dtSwapEndian(&header->version);
	dtSwapEndian(&header->tx);
	dtSwapEndian(&header->ty);
	dtSwapEndian(&header->tlayer);
	dtSwapEndian(&header->bmin[0]);
	dtSwapEndian(&header->bmin[1]);
	dtSwapEndian(&header->bmin[2]);
	dtSwapEndian(&header->bmax[0]);
	dtSwapEndian(&header->bmax[1]);
	dtSwapEndian(&header->bmax[2]);
	dtSwapEndian(&header->hmin);
	dtSwapEndian(&header->hmax);
	
	// width, height, minx, maxx, miny, maxy are unsigned char, no need to swap.
	
	return true;
}

