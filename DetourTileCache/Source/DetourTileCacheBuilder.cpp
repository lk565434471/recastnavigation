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


// ͼ��ɨ��Ŀ����Ϣ
struct dtLayerSweepSpan
{
	// ��ͨ��·�����������ھӣ�
	unsigned short ns;	// number samples
	// ����������id
	unsigned char id;	// region id
	// y���ϵ��ھӵ�id��ֵ��0xff����ʾ��Ч�ھ�.
	unsigned char nei;	// neighbour id
};

static const int DT_LAYER_MAX_NEIS = 16;

struct dtLayerMonotoneRegion
{
	// �������
	int area;
	// �洢�ھӵ��±�������ͨ�Ե��±�
	unsigned char neis[DT_LAYER_MAX_NEIS];
	// �ھӵ�����
	unsigned char nneis;
	// ������ͨ��id����������ڵĶ���λ��ֵ�ͬһ�� region ��
	unsigned char regId;
	// ����id
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
	// ���㻺����
	unsigned char* verts;
	// ������������
	int nverts;
	// ������������
	int cverts;
	// ���������
	unsigned short* poly;
	// ʵ�ʶ��������
	int npoly;
	// ���������
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
	
	// �� w * h ��������ɵ�����
	memset(layer.regs,0xff,sizeof(unsigned char)*w*h);
	
	const int nsweeps = w;
	dtFixedArray<dtLayerSweepSpan> sweeps(alloc, nsweeps);
	if (!sweeps)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(sweeps,0,sizeof(dtLayerSweepSpan)*nsweeps);
	
	// Partition walkable area into monotone regions.
	// prevCount �洢����y����ھ�����
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
			// ��ͨ�Բ��ԣ��������߶ȱ����� walkableClimb ���ڣ�������
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
			// ��ͨ�Բ��ԣ��������߶ȱ����� walkableClimb ���ڣ�������
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
		// ��������Ψһid
		// sweepId ��ʾ�ж��ٸ�sid
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			// ������ھӱ����ã���ֻ��һ���������Ԫ�������������ô����ǰһ������ϲ���һ��
			// ���򣬴���һ���µ�����������
			if (sweeps[i].nei != 0xff && (unsigned short)prevCount[sweeps[i].nei] == sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				// ��Ϊ 0xff ������û����Ч���ھӵı�ʶ��������������254������������id
				if (regId == 255)
				{
					// Region ID's overflow.
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
		// ���ֲ�ɨ��ID����ӳ��Ϊ����ID
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			if (layer.regs[idx] != 0xff)
				layer.regs[idx] = sweeps[layer.regs[idx]].id;
		}
	}
	
	// Allocate and init layer regions.
	// ���䲢��ʼ�����������Ϣ
	const int nregs = (int)regId;
	dtFixedArray<dtLayerMonotoneRegion> regs(alloc, nregs);
	if (!regs)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	memset(regs, 0, sizeof(dtLayerMonotoneRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = 0xff;
	
	// Find region neighbours.
	// ��ѯ���ڵĵ�Ԫ�����򣬸�����ͨ����Ϣ��������ͨ�ĵ�Ԫ�����������ھӣ�
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			// ��ʾû����ͨ��
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
					// ���Է����뵽�Լ���Ψһ�б���
					addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
					addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
				}
			}
		}
	}
	
	// ��ͨ�Ե�����id���Ǵ�0�������������-1������ֵ
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = (unsigned char)i;
	
	// �ϲ�����
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
				// ��ֻ��һ���ھӵ���ͨ������id == regn.regId ������£����Խ��кϲ�
				if (canMerge(reg.regId, regn.regId, regs, nregs))
				{
					mergea = regn.area;
					merge = (int)nei;
				}
			}
		}
		// ���Ժϲ������ó��µ���ͨ������id
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
	
	// ������ͨ������id
	for (int i = 0; i < w*h; ++i)
	{
		if (layer.regs[i] != 0xff)
			layer.regs[i] = regs[layer.regs[i]].regId;
	}
	
	return DT_SUCCESS;
}


//
// r ��ͨ�������id
static bool appendVertex(dtTempContour& cont, const int x, const int y, const int z, const int r)
{
	// Try to merge with existing segments.
	// ���Ժϲ��Ѿ����ڵ��߶Σ�����ֱ�Ӷ� cont.nverts - 2/1 �Ĳ���
	// Ӧ���ǻ���һ���߶���2��������ɵĸ���ó���
	if (cont.nverts > 1)
	{
		// �� a �� �� b
		unsigned char* pa = &cont.verts[(cont.nverts-2)*4];
		unsigned char* pb = &cont.verts[(cont.nverts-1)*4];
		if ((int)pb[3] == r)
		{
			if (pa[0] == pb[0] && (int)pb[0] == x)
			{
				// The verts are aligned aling x-axis, update z.
				// ��x����룬Ȼ�����z���ֵ��
				pb[1] = (unsigned char)y;
				pb[2] = (unsigned char)z;
				return true;
			}
			else if (pa[2] == pb[2] && (int)pb[2] == z)
			{
				// The verts are aligned aling z-axis, update x.
				// ��z����룬Ȼ�����x���ֵ
				pb[0] = (unsigned char)x;
				pb[1] = (unsigned char)y;
				return true;
			}
		}
	}
	
	// Add new point.
	// ����µĵ�
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

// ������������id
// layer �������������ݹ������
// ax ����ڼ��еĸ���
// ay ����ڼ��еĸ���
// dir �ƶ�����
static unsigned char getNeighbourReg(dtTileCacheLayer& layer,
									 const int ax, const int ay, const int dir)
{
	// w ��ʾһ���ж����У������ӵ�����
	const int w = (int)layer.header->width;
	// ͨ�� ax + w * ay�����Լ�������ӵ��±�
	const int ia = ax + ay*w;
	
	// con ��ʾʲô����ʱ��ȷ��
	const unsigned char con = layer.cons[ia] & 0xf;
	// portal ��ʾʲô����ʱ��ȷ��
	const unsigned char portal = layer.cons[ia] >> 4;
	// mask ä���Ǳ�ʾ����
	const unsigned char mask = (unsigned char)(1<<dir);
	
	// ˵���޷�ֱ��ͨ������������Ϊɶ����Ҳ��֪��
	// ��Ϊ��ĿǰΪֹ���һ���֪�� mask ���Ա�ʾ��ֵ����Щ
	if ((con & mask) == 0)
	{
		// No connection, return portal or hard edge.
		// û�����ӣ������Ż�����Ӳ�ߣ�ֱ���
		// �Ż�ָ���Ƕ����֮���ͨ�������š�����
		// Ӳ��ָ����ǽ�ڡ�������������޷���Խ���ϰ��
		if (portal & mask)
			return 0xf8 + (unsigned char)dir;
		return 0xff;
	}
	
	/*
	* 
	*				   �ֱ��Ӧ��������x��������y��������x��������y
	*                  {-1, 1, 1, -1}
	* 
	*                                     +1
	*                                   *
	*                                   *
	*                                   * y��
	*                                   *
	*                   x�� ************************ +1
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

// �����������ߣ��ض���εıߡ�������������㵽�յ�Ŀ����������������
// �ҵ�����ǣ������ж���·�����տ��Ե����յ㣬�����߰��������п��ܡ�
static bool walkContour(dtTileCacheLayer& layer, int x, int y, dtTempContour& cont)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	cont.nverts = 0;
	
	int startX = x;
	int startY = y;
	int startDir = -1;
	
	// ������4��������������
	for (int i = 0; i < 4; ++i)
	{
		const int dir = (i+3)&3;
		// ������������id
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		// ��ȡ���������ĳ���
		if (rn != layer.regs[x+y*w])
		{
			startDir = dir;
			break;
		}
	}
	// ���������Ҫ�ı䣬��ֱ�ӷ��ؿ�ͨ����
	if (startDir == -1)
		return true;
	
	int dir = startDir;
	const int maxIter = w*h;
	
	int iter = 0;
	while (iter < maxIter)
	{
		// ������������id
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		
		int nx = x;
		int ny = y;
		int ndir = dir;
		
		if (rn != layer.regs[x+y*w])
		{
			// Solid edge.
			// ʵ��߽磿��ʾ�����������֮����ϰ���򲻿ɴ�Խ�ı߽�
			int px = x;
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			// Try to merge with previous vertex.
			// ������ǰһ������ϲ�
			if (!appendVertex(cont, px, (int)layer.heights[x+y*w], pz,rn))
				return false;
			
			// ��˳ʱ�뷽����ת cw = clockwise
			ndir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			// Move to next.
			nx = x + getDirOffsetX(dir);
			ny = y + getDirOffsetY(dir);
			// ����ʱ�뷽����ת ccw = counter-clockwise
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
	// ����һ����������һ�������Ƿ��ظ�������һ�£�������ظ���ɾ�����һ�����㡣
	// ʵ���ϣ����һ��������ڴ滹�ڣ�ͨ���Զ��������1�����ڸ����һ�����㱻ɾ������ʵ��
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
	// �ֱ�����߶ε�������xz���ϵķ���
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	// �ֱ��ʾ�����߶�����������xz���ϵķ���
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	// �߶εĳ��ȵ�ƽ�������߶ε�������ģ����ƽ��
	float d = pqx*pqx + pqz*pqz;
	// �����߶��ϵ�ͶӰ���߶η����ϵı���
	float t = pqx*dx + pqz*dz;
	// ����߶γ��ȵ�ƽ������0����ͶӰ���ȳ����߶γ��ȵ�ƽ�����õ�������
	// ���ｫt��ֵ���ŵ� [0, 1] ֮��
	if (d > 0)
		t /= d;
	// ��ʾ��̾������߶����֮ǰ��
	if (t < 0)
		t = 0;
	// ��ʾ���������߶��յ�֮��
	else if (t > 1)
		t = 1;
	
	// ���º���̾����������xz���ϵķ���
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	// ������̾����ƽ��
	return dx*dx + dz*dz;
}

// �򻯵���·����������
static void simplifyContour(dtTempContour& cont, const float maxError)
{
	// ÿ�ν��뺯���������ö���ε�����ͳ��
	cont.npoly = 0;
	
	for (int i = 0; i < cont.nverts; ++i)
	{
		int j = (i+1) % cont.nverts;
		// Check for start of a wall segment.
		// ���ǽ�ڵ���ʼ�߶Σ�
		// ������δ��룬����ʱû���κ��뷨
		// Ψһ֪�����ǣ��ж���������洢��ֵ�Ƿ���ȣ�������ȣ������εļ���+1
		// �Ҽ�¼�¶����������е��±�
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
		// ���û��ת���㣬Ϊ�򻯹��̴���һЩ��ʼ�㡣
		// �ҵ����������ϽǺ����½ǵĶ��㣬ͨ���������㹹����һ�� AABB��������Χ�У�
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
	// ��򻯵�ͼ����ӵ㣬ֱ������ԭʼ�㶼��������̷�Χ�ڡ�
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
		// �ҳ��߶ε����ƫ��
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;
		
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		// �����ֵ�˳������߶Σ�ȷ���ڼ������ƫ��ʱ�õ�һ�µĽ��������
		// �������Ƿ�������߶Ρ�
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
			// ��̾���
			float d = distancePtSeg(cont.verts[ci*4+0], cont.verts[ci*4+2], ax, az, bx, bz);
			// ��¼������̾��룬�������±�
			if (d > maxd)
			{
				maxd = d;
				maxi = ci;
			}
			ci = (ci+cinc) % cont.nverts;
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		// ������ƫ����ڿɽ��ܵ������ֵ����ô���ڸ��߶�������µ�
		// ����ϸ���߶Σ������������߶Σ�����������һ���߶Ρ�
		// ͨ�����ٵ�����������Խ��ͺ����������ĸ��ӶȺͳɱ���
		// ������̻��ڶ������ߵ���״�������з������жϣ���������Ҫ��
		// ��״ϸ�ڣ���ȥ������Ҫ������㡣
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
	// ����ӳ�䶥��
	int start = 0;
	for (int i = 1; i < cont.npoly; ++i)
		if (cont.poly[i] < cont.poly[start])
			start = i;
	
	// �����ûʲô��ͷ�ˣ������Ͼ���ͨ���ƶ��������ݣ�������ͳ��
	// �������������ʵ��ɾ������Ķ�������
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

// ���ؽǵĸ߶�
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
	
	// ���������������
	lcset.nconts = layer.regCount;
	lcset.conts = (dtTileCacheContour*)alloc->alloc(sizeof(dtTileCacheContour)*lcset.nconts);
	if (!lcset.conts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(lcset.conts, 0, sizeof(dtTileCacheContour)*lcset.nconts);
	
	// Allocate temp buffer for contour tracing.
	// ������ʱ������������Ѱ·�㷨���ݵ�׷��
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
			// �޷�ͨ��������ֱ������
			if (ri == 0xff)
				continue;
			
			dtTileCacheContour& cont = lcset.conts[ri];
			
			// ����������0��˵���Ѿ��������ˣ���ʱ�޷����
			if (cont.nverts > 0)
				continue;
			
			// ��ͨ������id
			cont.reg = ri;
			cont.area = layer.areas[idx];
			
			if (!walkContour(layer, x, y, temp))
			{
				// Too complex contour.
				// Note: If you hit here ofte, try increasing 'maxTempVerts'.
				// ������뾭��ִ�е������������ maxTempVerts ������
				// ��ʱ�޷�ȷ���Ƿ���Ч����ֱ����˾���������
				return DT_FAILURE | DT_BUFFER_TOO_SMALL;
			}
			
			simplifyContour(temp, maxError);
			
			// Store contour.
			// ��������������
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
					// ��ÿ���߶εĶ����ϴ洢���ڵ�������Ϣ�����ӿ�·���滮�͵��������е�����ת��������
					unsigned char nei = vn[3]; // The neighbour reg is stored at segment vertex of a segment. 
					bool shouldRemove = false;
					unsigned char lh = getCornerHeight(layer, (int)v[0], (int)v[1], (int)v[2],
													   walkableClimb, shouldRemove);
					
					dst[0] = v[0];
					dst[1] = lh;
					dst[2] = v[2];
					
					// Store portal direction and remove status to the fourth component.
					// ���Ż��ķ�����Ϣ�洢�ڶ���ĵ�4��Ԫ���У����Ƴ�״̬��Ϣ��
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
// ���� true���� c �ϸ�λ�������߶� ab �����ʱ��
inline bool left(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) < 0;
}

// ���� true��c λ�������߶ε���࣬�������߶��ϡ�
inline bool leftOn(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) <= 0;
}

// �ж��Ƿ�Ϊ���ߣ���������������������0ʱ����˵���ǹ��ߡ�
inline bool collinear(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
// ���� true�� �߶� ab �� cd ���ڲ��ཻ�������������߶μ乲��һ���㡣
// ͨ��ʹ���ϸ������ཻ��ȷ������������ԡ�
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
	// ����߶� ab ���Ǵ�ֱ�ߣ��� x ���Ͻ��н�����飻������ y ���Ͻ��н����ļ�顣
	// ���ж��߶ε��ཻ��
	if (a[0] != b[0])
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
// ���� true������߶� ab �� cd �ཻ���������ʵ��ཻ���ڲ����㣩���ǲ��ʵ��ཻ���߽罻�㣩
// �ڲ����㣺��ʾ�ڿռ����໥�����֯
// �߽罻�㣺��ʾ�ڱ߽����н���ʱ
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
// ����߶� (v_i, v_j) �Ƕ���� P ��һ����Ч���ڲ����ⲿ�Խ��ߣ����� T������Ҫ�ų��붥�� v_i �� v_j ���ڵıߣ� 
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
// ���Խ��ߣ�i��j���ϸ��λ�ڶ˵㸽���Ķ����P�ڲ�ʱ������ true
static bool	inCone(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* pi = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* pj = &verts[(indices[j] & 0x7fff) * 4];
	const unsigned char* pi1 = &verts[(indices[next(i, n)] & 0x7fff) * 4];
	const unsigned char* pin1 = &verts[(indices[prev(i, n)] & 0x7fff) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	// ��� p[i] ��һ��͹�㣬i+1 ���߶���� or ���߶� (i-1,i) �ϡ�
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	// ���� (i-1,i,i+1) �����ߣ����� P[i] �ǰ���
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
// ���� true������߶� (v_i, v_j) �� P ��һ���ڲ��Խ���
static bool diagonal(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

static int triangulate(int n, const unsigned char* verts, unsigned short* indices, unsigned short* tris)
{
	int ntris = 0;
	unsigned short* dst = tris;
	
	// The last bit of the index is used to indicate if the vertex can be removed.
	// ���������һ������λ�����ڱ�ʾ�����Ƿ���Ա��Ƴ�
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n);
		int i2 = next(i1, n);
		// ����ڲ��Խ���
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x8000; // �ڲ��Խ��߱�ʶ��
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			// �ڲ��Խ���
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
		// ������Ϊ����һ�����⣬��Ӧ�ý�����������̴���
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
		// ͨ���ƶ�������ɾ������
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		// Update diagonal flags.
		// ���¶Խ��߱�ʶ��
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x8000;
		else
			indices[i] &= 0x7fff;
		// ���¶Խ��߱�ʶ��
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x8000;
		else
			indices[i1] &= 0x7fff;
	}
	
	// Append the remaining triangle.
	// ����ʣ���������
	*dst++ = indices[0] & 0x7fff;
	*dst++ = indices[1] & 0x7fff;
	*dst++ = indices[2] & 0x7fff;
	ntris++;
	
	return ntris;
}

// ���ض���εĶ�����
static int countPolyVerts(const unsigned short* p)
{
	for (int i = 0; i < MAX_VERTS_PER_POLY; ++i)
		if (p[i] == DT_TILECACHE_NULL_IDX)
			return i;
	return MAX_VERTS_PER_POLY;
}

// ���� true���� c �������߶� ab �����
inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
	((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

// ������������֮���ƽ������
static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb)
{
	// ��ö���� a �� b �Ķ�����
	const int na = countPolyVerts(pa);
	const int nb = countPolyVerts(pb);
	
	// If the merged polygon would be too big, do not merge.
	// �����������εĶ���������2����MAX_VERTS_PER_POLY����ִ�кϲ�����
	if (na+nb-2 > MAX_VERTS_PER_POLY)
		return -1;
	
	// Check if the polygons share an edge.
	// ��������Ƿ����һ������ߣ�ea �� eb �������㣬���һ�������
	// �洢��������������±�
	ea = -1;
	eb = -1;
	
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		// Ϊʲô�Ե�ǰ�±�+1����ģ���㣿
		// �������εıպ��ԣ�����ε����һ���������ͨ���ǵ�һ�����㣬ͨ��������������ģ����
		// ����ȷ���ڴﵽ���һ������ʱ����һ��������ص���һ�����㡣
		// �����Ժ�ͨ���ԣ�����Ҫ���Ƕ���ζ������ż����������߼���ͳһ��������⡣
		// �򻯱߽�������������Ҫ����������ж����������һ��������������������˴��븴�Ӷȡ�
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
	// û�й���ߣ����ܺϲ�����������
	if (ea == -1 || eb == -1)
		return -1;
	
	// Check to see if the merged polygon would be convex.
	// ���ϲ���Ķ�����Ƿ�Ϊ͹�����
	unsigned short va, vb, vc;
	
	va = pa[(ea+na-1) % na];
	vb = pa[ea];
	vc = pb[(eb+2) % nb];
	// �жϵ�c���������߶�ab�����
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pb[(eb+nb-1) % nb];
	vb = pb[eb];
	vc = pa[(ea+2) % na];
	// �жϵ�c���������߶�ab�����
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pa[ea];
	vb = pa[(ea+1)%na];
	
	int dx = (int)verts[va*3+0] - (int)verts[vb*3+0];
	int dy = (int)verts[va*3+2] - (int)verts[vb*3+2];
	
	return dx*dx + dy*dy;
}

// �ϲ����
// pa �� pb �Ǵ��ϲ������������
// ea �� eb ���������űߵ��±�
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

// ���� true��������Ա�ɾ��
static bool canRemoveVertex(dtTileCachePolyMesh& mesh, const unsigned short rem)
{
	// Count number of polygons to remove.
	// ����ɾ���Ķ��������
	int numTouchedVerts = 0;
	// ʣ��ı���
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// ��ö���� p �Ķ�������
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
	// ʣ�µı���̫�٣��޷���������Ρ�
	// ���磬�������εĶ��㱻���Ϊɾ������û����������ι���ö���ʱ���ͻᷢ���������.
	// ����������£����㲻�ܱ�ɾ����
	if (numRemainingEdges <= 2)
		return false;
	
	// Check that there is enough memory for the test.
	// ����Ƿ����㹻���ڴ����ڲ���
	const int maxEdges = numTouchedVerts*2;
	if (maxEdges > MAX_REM_EDGES)
		return false;
	
	// Find edges which share the removed vertex.
	// �ҵ��뱻�Ƴ��������ڵı�
	unsigned short edges[MAX_REM_EDGES];
	int nedges = 0;
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// ��ö����p�Ķ�������
		const int nv = countPolyVerts(p);
		
		// Collect edges which touches the removed vertex.
		// �ռ��뱻ɾ���������ڵı�
		for (int j = 0, k = nv-1; j < nv; k = j++)
		{
			if (p[j] == rem || p[k] == rem)
			{
				// Arrange edge so that a=rem.
				// �����������У�a = ���Ƴ��Ķ���
				int a = p[j], b = p[k];
				if (b == rem)
					dtSwap(a,b);
				
				// Check if the edge exists
				// �����Ƿ����
				bool exists = false;
				for (int m = 0; m < nedges; ++m)
				{
					unsigned short* e = &edges[m*3];
					if (e[1] == b)
					{
						// Exists, increment vertex share count.
						// ���ڣ����Ӷ��㹲�����
						e[2]++;
						exists = true;
					}
				}
				// Add new edge.
				// �߲����ڣ�����һ���±�
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
	// ȷ�������û�г����������űߡ�
	// ����������������������ڵĶ���ι����Ƴ��Ķ���������
	// ����������£�����ɾ�����㣬����ᷢ�����ѡ�
	// Ϊʲô�����ڵĶ���λ�ӵ�й����㣬���ǿ�������������3������Σ������������ڡ�
	// ����������£�Ϊ���Ż����м�Ķ���λᱻɾ��������˾ͳ����˷����ڶ���Σ�������������
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

// ɾ������
// mesh
// rem ��ɾ���Ķ����±�
// maxTris ��������������
static dtStatus removeVertex(dtTileCachePolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	// Count number of polygons to remove.
	// ���㱻ɾ���Ķ��������
	int numRemovedVerts = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		// ��ö����p�Ķ�������
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
		// ��ö����p�Ķ�������
		const int nv = countPolyVerts(p);
		bool hasRem = false;
		for (int j = 0; j < nv; ++j)
			if (p[j] == rem) hasRem = true;
		if (hasRem)
		{
			// Collect edges which does not touch the removed vertex.
			// �ռ����뱻ɾ��������Ӵ��ı�
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
			// ɾ������Σ�ͨ���ƶ��ڴ�������ʵ�֣�Ȼ����ٶ���μ���
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*MAX_VERTS_PER_POLY*2];
			memcpy(p,p2,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			memset(p+MAX_VERTS_PER_POLY,0xff,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			mesh.areas[i] = mesh.areas[mesh.npolys-1];
			mesh.npolys--;
			--i;
		}
	}
	
	// Remove vertex.
	// ɾ������Σ�ͨ���ƶ��ڴ�������ʵ�֣�Ȼ����ٶ������
	for (int i = (int)rem; i < mesh.nverts - 1; ++i)
	{
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0];
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1];
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2];
	}
	mesh.nverts--;
	
	// Adjust indices to match the removed vertex layout.
	// ����������ƥ���Ƴ������Ķ��㲼��
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	// �ݼ������ü���
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+0] > rem) edges[i*3+0]--;
		if (edges[i*3+1] > rem) edges[i*3+1]--;
	}
	
	if (nedges == 0)
		return DT_SUCCESS;
	
	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	// ��һ�����㿪ʼ�����Ͻ����ӵ��߶���ӵ��׶��������յ㡣
	// ���� ���׶��� ָ�������ڼ򻯶���κ���ඥ������Ѿ���ɾ��
	// �Ӷ������¶���β����Ƿ�յ�״̬��ͨ���߶ν���Щ���������
	// ���ٴ��γ��µķ��ʽ�Ķ���Ρ�
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
				// ���߶���׶��ı߽����㲿��ƥ��
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushFront(ea, hole, nhole);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				// The segment matches the end of the hole boundary.
				// ���߶���׶��ı߽���յ㲿��ƥ��
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushBack(eb, hole, nhole);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				// The edge segment was added, remove it.
				// ��Ե�Σ�����α߽���߶Σ��Ѿ�����ӵ����ʵ�λ�ú��߶�
				// �Ϳ��Դ����ݽṹ���Ƴ��ˡ�
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
	// ���������ε���ʱ��������
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
	// ���׶������ʷ֣�����ֽ�Ϊһϵ�������Σ���ʾΪһ�����ӡ�
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
	// ������ʼ�����
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
	// �ϲ������
	int maxVertsPerPoly = MAX_VERTS_PER_POLY;
	if (maxVertsPerPoly > 3)
	{
		for (;;)
		{
			// Find best polygons to merge.
			// �ҵ����Ŷ���Σ����кϲ�����
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
	// �洢��������ݽṹ
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
	
	// ���������
	int maxVertices = 0;
	// ����������
	int maxTris = 0;
	// ÿ�������ɳ��еĶ���������
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
	
	// �������������
	dtFixedArray<unsigned short> indices(alloc, maxVertsPerCont);
	if (!indices)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// ����������
	dtFixedArray<unsigned short> tris(alloc, maxVertsPerCont*3);
	if (!tris)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// ���������
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
		// ����������
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
			// ���һ���¶��㣬���ҵ�һ�����㣬����������
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			// 0x80 ɾ����ʶ��
			if (v[3] & 0x80)
			{
				// This vertex should be removed.
				vflags[indices[j]] = 1;
			}
		}
		
		// Build initial polygons.
		// ������ʼ�����
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
				// �ҵ����Ŷ���ν��кϲ�
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*MAX_VERTS_PER_POLY];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*MAX_VERTS_PER_POLY];
						int ea, eb;
						// v = ��������֮���ƽ������
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
				
				// �ҵ���Ѻϲ��Ķ����
				if (bestMergeVal > 0)
				{
					// Found best, merge.
					unsigned short* pa = &polys[bestPa*MAX_VERTS_PER_POLY];
					unsigned short* pb = &polys[bestPb*MAX_VERTS_PER_POLY];
					// �ϲ�������ݣ��洢�� pa ��
					mergePolys(pa, pb, bestEa, bestEb);
					memcpy(pb, &polys[(npolys-1)*MAX_VERTS_PER_POLY], sizeof(unsigned short)*MAX_VERTS_PER_POLY);
					npolys--;
				}
				else
				{
					// Could not merge any polygons, stop.
					// ���ܺϲ��κζ���Σ��˳�ѭ��
					break;
				}
			}
		}
		
		// Store polygons.
		// �洢�����
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
	// ɾ���ߵĶ���
	for (int i = 0; i < mesh.nverts; ++i)
	{
		// vflags[i] != 0����ô����Ҫɾ��
		if (vflags[i])
		{
			// �ж϶����Ƿ���Ա�ɾ��
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
	// ��Բ������������ʹ�� AABB��������Χ�У���������Ʊ�ʾ
	// ����Ϊ���α߽�����׽�����ײ���ص���⡣�����ֱ�ʾ��ĳЩ��
	// ���£�������ȫ��׼��������Ч�ʺ��ٶ��ϵ�һ��ȡ�ᡣ

	// ����� bmin �� bmax ��ʾ��������� AABB��������Χ�У�
	// �߽磬��һ������ֵ��ͨ���뾶������
	float bmin[3], bmax[3];
	bmin[0] = pos[0] - radius;
	bmin[1] = pos[1];
	bmin[2] = pos[2] - radius;
	bmax[0] = pos[0] + radius;
	bmax[1] = pos[1] + height;
	bmax[2] = pos[2] + radius;
	const float r2 = dtSqr(radius/cs + 0.5f);
	// ͼ��ĳߴ磬�������Ϊ��������ͼ�����Ƹ���Ĵ�С�������ؿ�ߣ�1024 * 1024�ȵȣ�
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	// ��������
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;
	
	// �� xz ��ƽ���Ͼ����Χ����㣨���ε����Ͻǣ���λ�õ�����
	const float px = (pos[0]-orig[0])*ics;
	const float pz = (pos[2]-orig[2])*ics;
	
	int minx = (int)dtMathFloorf((bmin[0]-orig[0])*ics);
	int miny = (int)dtMathFloorf((bmin[1]-orig[1])*ich);
	int minz = (int)dtMathFloorf((bmin[2]-orig[2])*ics);
	int maxx = (int)dtMathFloorf((bmax[0]-orig[0])*ics);
	int maxy = (int)dtMathFloorf((bmax[1]-orig[1])*ich);
	int maxz = (int)dtMathFloorf((bmax[2]-orig[2])*ics);

	// ��ʾԲ������ AABB����Χ�У�����࣬���ǻ��������С����㿪ʼ��ԭ��
	if (maxx < 0) return DT_SUCCESS;
	// ��ʾԲ������ AABB����Χ�У����Ҳ࣬��������Ч��Χ
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	// ����Ĵ���ʹ���˸�Ϊ��׼��Բ������
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
	// ��4�ֽڶ���
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	// ������ӵĴ�С
	const int gridSize = (int)header->width * (int)header->height;
	// �������ݵ���󻺳�����С
	const int maxDataSize = headerSize + comp->maxCompressedSize(gridSize*3);
	unsigned char* data = (unsigned char*)dtAlloc(maxDataSize, DT_ALLOC_PERM);
	if (!data)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(data, 0, maxDataSize);
	
	// Store header
	// �洢��Ƭ����Ĳ㼶��ͷ����Ϣ
	memcpy(data, header, sizeof(dtTileCacheLayerHeader));
	
	// Concatenate grid data for compression.
	const int bufferSize = gridSize*3;
	// ������������
	unsigned char* buffer = (unsigned char*)dtAlloc(bufferSize, DT_ALLOC_TEMP);
	if (!buffer)
	{
		dtFree(data);
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	// ���������ݷ�Ϊ�������֣��߶���Ϣ��������Ϣ��������Ϣ
	memcpy(buffer, heights, gridSize);
	memcpy(buffer+gridSize, areas, gridSize);
	memcpy(buffer+gridSize*2, cons, gridSize);
	
	// Compress
	// ������Ƭ�����ͷ����Ϣ
	unsigned char* compressed = data + headerSize;
	// �����ѹ�����ݵĳ�����Ϣ
	const int maxCompressedSize = maxDataSize - headerSize;
	int compressedSize = 0;
	// ʵ��ѹ�������ݴ�С
	dtStatus status = comp->compress(buffer, bufferSize, compressed, maxCompressedSize, &compressedSize);
	if (dtStatusFailed(status))
	{
		dtFree(buffer);
		dtFree(data);
		return status;
	}

	// ѹ��������ݣ���ʵ��ѹ�����ݵĴ�С
	*outData = data;
	*outDataSize = headerSize + compressedSize;
	
	dtFree(buffer);
	
	return DT_SUCCESS;
}

// �ͷ���Ƭ����Ĳ㼶������ռ�õ��ڴ�
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

	// ��� layerOut �Ƿ��Ѿ����ⲿ�������ڴ棨����ʼ�����ģ�
	if (!layerOut)
		return DT_FAILURE | DT_INVALID_PARAM;
	// ��� compressed �Ƿ���Ч������ѹ��������
	if (!compressed)
		return DT_FAILURE | DT_INVALID_PARAM;

	*layerOut = 0;
	// ��ȡ��Ƭ����Ĳ㼶ͷ����Ϣ
	dtTileCacheLayerHeader* compressedHeader = (dtTileCacheLayerHeader*)compressed;
	// ����ļ�ͷ��ʽ�����ݰ汾��
	if (compressedHeader->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (compressedHeader->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	const int layerSize = dtAlign4(sizeof(dtTileCacheLayer));
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = (int)compressedHeader->width * (int)compressedHeader->height;
	const int bufferSize = layerSize + headerSize + gridSize*4;
	
	// �����㹻��Ļ����������ɸ߶ȡ�����������Ϣ
	unsigned char* buffer = (unsigned char*)alloc->alloc(bufferSize);
	if (!buffer)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(buffer, 0, bufferSize);

	dtTileCacheLayer* layer = (dtTileCacheLayer*)buffer;
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)(buffer + layerSize);
	unsigned char* grids = buffer + layerSize + headerSize;
	const int gridsSize = bufferSize - (layerSize + headerSize); 
	
	// Copy header
	// ����ͷ����Ϣ
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


// ��Ƭ���ݽṹ�Ĵ洢��ʽ
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

