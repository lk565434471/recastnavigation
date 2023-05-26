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

//////////////////////////////////////////////////////////////////////////////////////////

void dtClosestPtPointTriangle(float* closest, const float* p,
							  const float* a, const float* b, const float* c)
{
	// Check if P in vertex region outside A
	float ab[3], ac[3], ap[3];
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	float d1 = dtVdot(ab, ap);
	float d2 = dtVdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	float bp[3];
	dtVsub(bp, p, b);
	float d3 = dtVdot(ab, bp);
	float d4 = dtVdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}
	
	// Check if P in vertex region outside C
	float cp[3];
	dtVsub(cp, p, c);
	float d5 = dtVdot(ab, cp);
	float d6 = dtVdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

// 与二维多边形相交的线段
bool dtIntersectSegmentPoly2D(const float* p0, const float* p1,
							  const float* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax)
{
	static const float EPS = 0.000001f;
	
	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	
	float dir[3];
	dtVsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		float edge[3], diff[3];
		dtVsub(edge, &verts[i*3], &verts[j*3]);
		dtVsub(diff, p0, &verts[j*3]);
		// 计算射线起点到多边形的最近焦点处的坐标
		const float n = dtVperp2D(edge, diff);
		const float d = dtVperp2D(dir, edge);
		if (fabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			// S(Segment) 线段几乎与多边形的边向量平行。
			if (n < 0)
				return false;
			else
				continue;
		}
		const float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

// 计算点到线段的最近点
float dtDistancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t)
{
	// 计算线段的向量pq的分量
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	// 计算点到起点p的向量分量
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	// 计算线段pq的长度的平方
	float d = pqx*pqx + pqz*pqz;
	// 计算点乘的结果，并存储到变量t中
	t = pqx*dx + pqz*dz;
	if (d > 0) t /= d;
	// 表示最近距离在起点p处
	if (t < 0) t = 0;
	// 表示最近距离在终点q处
	else if (t > 1) t = 1;
	// 计算最近点到点pt的向量dx、dz
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	// 返回最近距离的平方根
	return dx*dx + dz*dz;
}

void dtCalcPolyCenter(float* tc, const unsigned short* idx, int nidx, const float* verts)
{
	tc[0] = 0.0f;
	tc[1] = 0.0f;
	tc[2] = 0.0f;
	for (int j = 0; j < nidx; ++j)
	{
		const float* v = &verts[idx[j]*3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const float s = 1.0f / nidx;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}

bool dtClosestHeightPointTriangle(const float* p, const float* a, const float* b, const float* c, float& h)
{
	const float EPS = 1e-6f;
	float v0[3], v1[3], v2[3];

	dtVsub(v0, c, a);
	dtVsub(v1, b, a);
	dtVsub(v2, p, a);

	// Compute scaled barycentric coordinates
	// 计算缩放的重心坐标的过程
	// 重心坐标是相对于三角形内部位置的坐标系统
	// denmo 叉乘的垂直分量
	float denom = v0[0] * v1[2] - v0[2] * v1[0];
	if (fabsf(denom) < EPS)
		return false;

	// 分别计算给定点与三角形的两个边向量的叉乘的垂直分量
	float u = v1[2] * v2[0] - v1[0] * v2[2];
	float v = v0[0] * v2[2] - v0[2] * v2[0];

	if (denom < 0) {
		denom = -denom;
		u = -u;
		v = -v;
	}

	// If point lies inside the triangle, return interpolated ycoord.
	// 如果给定的点位于三角形内部，则返回插值计算得到的y坐标（高度）
	if (u >= 0.0f && v >= 0.0f && (u + v) <= denom) {
		h = a[1] + (v0[1] * u + v1[1] * v) / denom;
		return true;
	}
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
/// 
/// 所有的点都被投射到xz平面上，因此忽略了y值。
bool dtPointInPolygon(const float* pt, const float* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	// 替换 pnpoly 为 triArea2D 测试？
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const float* vi = &verts[i*3];
		const float* vj = &verts[j*3];
		// ((vi[2] > pt[2]) != (vj[2] > pt[2])) 如果多边形的两个顶点的高度都大于给定点的高度
		// 那么，给定点的投影一定位于多边形的边界上或外部，因为多边形是一个封闭的形状，其边界上的点
		// 不可能在多边形的内部
		// (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) 计算给定点的x坐标投影到
		// 多边形的投影平面上，并计算出投影平面上给定点的x坐标
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
	}
	return c;
}

bool dtDistancePtPolyEdgesSqr(const float* pt, const float* verts, const int nverts,
							  float* ed, float* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	// 替换 pnpoly 算法为 triArea2D 算法后，进行测试？
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const float* vi = &verts[i*3];
		const float* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const float* axis, const float* poly, const int npoly,
						float& rmin, float& rmax)
{
	rmin = rmax = dtVdot2D(axis, &poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const float d = dtVdot2D(axis, &poly[i*3]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

inline bool overlapRange(const float amin, const float amax,
						 const float bmin, const float bmax,
						 const float eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
/// 
/// 所有顶点投射到xz平面上，并忽略y值。
bool dtOverlapPolyPoly2D(const float* polya, const int npolya,
						 const float* polyb, const int npolyb)
{
	const float eps = 1e-4f;
	
	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const float* va = &polya[j*3];
		const float* vb = &polya[i*3];
		const float n[3] = { vb[2]-va[2], 0, -(vb[0]-va[0]) };
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);

		// 在进行碰撞检测时，找到了一个分离轴，该轴可以将两个物体的投影分离开来
		// 表明它们之间没有碰撞。
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const float* va = &polyb[j*3];
		const float* vb = &polyb[i*3];
		const float n[3] = { vb[2]-va[2], 0, -(vb[0]-va[0]) };
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
// 返回在凸多边形中的一个随机点
// pts 存储顶点的数组，[[x, y, z]]
// npts 顶点的数量
// areas 存储每个三角形的面积
// s 和 t 分别为两个在 [0, 1] 的随机数
// out 存储随机点 [x, y, z]
void dtRandomPointInConvexPoly(const float* pts, const int npts, float* areas,
							   const float s, const float t, float* out)
{
	// Calc triangle araes
	// 计算三角形区域
	// 存储累加三角形的面积
	float areasum = 0.0f;
	// 子三角形都是以多边形的第一个顶点为起点，其它顶点两两相邻。
	// 所以，当我们遍历子三角形时，需要从第二个顶点开始。
	for (int i = 2; i < npts; i++) {
		// 存储每个三角形的面积
		areas[i] = dtTriArea2D(&pts[0], &pts[(i-1)*3], &pts[i*3]);
		// 确保面积不为0
		areasum += dtMax(0.001f, areas[i]);
	}
	// Find sub triangle weighted by area.
	// 随机面积变量 thr 的范围在 [0, areasum] 之间
	const float thr = s*areasum;
	// 临时存储每个三角形面积的累加值
	float acc = 0.0f;
	// 随机点在所选中三角形内的权重
	float u = 1.0f;
	// 表示最后一个三角形的索引
	int tri = npts - 1;
	// 子三角形都是以多边形的第一个顶点为起点，其它顶点两两相邻。
	// 所以，当我们遍历子三角形时，需要从第二个顶点开始。
	for (int i = 2; i < npts; i++) {
		const float dacc = areas[i];
		// 当 thr 大于等于 acc，且小于 (acc + dacc) 时，我们就选中了包含随机点的三角形
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			// 更新选中三角形的索引，然后退出循环
			tri = i;
			break;
		}
		acc += dacc;
	}
	
	// 计算另外一个随机数 t 的平方根
	float v = dtMathSqrtf(t);
	// 计算 ABC 三个顶点的权重因子
	const float a = 1 - v;
	const float b = (1 - u) * v;
	const float c = u * v;
	// 获取三个顶点的坐标
	const float* pa = &pts[0];
	const float* pb = &pts[(tri-1)*3];
	const float* pc = &pts[tri*3];
	// 通过加权平均计算随机点，分别对应 x、y、z
	out[0] = a*pa[0] + b*pb[0] + c*pc[0];
	out[1] = a*pa[1] + b*pb[1] + c*pc[1];
	out[2] = a*pa[2] + b*pb[2] + c*pc[2];
}

inline float vperpXZ(const float* a, const float* b) { return a[0]*b[2] - a[2]*b[0]; }

bool dtIntersectSegSeg2D(const float* ap, const float* aq,
						 const float* bp, const float* bq,
						 float& s, float& t)
{
	float u[3], v[3], w[3];
	dtVsub(u,aq,ap);
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	float d = vperpXZ(u,v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}

