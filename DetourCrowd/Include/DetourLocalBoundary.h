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

#ifndef DETOURLOCALBOUNDARY_H
#define DETOURLOCALBOUNDARY_H

#include "DetourNavMeshQuery.h"


class dtLocalBoundary
{
	static const int MAX_LOCAL_SEGS = 8;
	static const int MAX_LOCAL_POLYS = 16;
	
	// 线段的结构体
	struct Segment
	{
		// 线段的起始和终止位置
		float s[6];	///< Segment start/end
		// 经过修剪的距离
		float d;	///< Distance for pruning.
	};
	
	// 中心点坐标
	float m_center[3];
	// 被认为是障碍物的线段
	Segment m_segs[MAX_LOCAL_SEGS];
	// 线段的实际数量
	int m_nsegs;
	
	// 被认为是障碍物的多边形
	dtPolyRef m_polys[MAX_LOCAL_POLYS];
	// 实际的多边形数量
	int m_npolys;

	// 根据代理之间的距离，将符合一定条件的代理插入到当前代理的邻居列表中。
	void addSegment(const float dist, const float* s);
	
public:
	dtLocalBoundary();
	~dtLocalBoundary();
	
	// 重置数据
	void reset();
	
	// 更新局部搜索边界，用于限制路径查询或导航操作的范围，以避免代理进入不可访问的区域
	void update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
				dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	// 在进行路径查询或导航网格操作时，使用多边形查询过滤器检测所有多边形
	// 确保它们仍然符合查询的条件
	bool isValid(dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	// 获得中心点位置
	inline const float* getCenter() const { return m_center; }

	// 获得线段的数量
	inline int getSegmentCount() const { return m_nsegs; }

	// 获得一条线段的起始和终止位置
	inline const float* getSegment(int i) const { return m_segs[i].s; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtLocalBoundary(const dtLocalBoundary&);
	dtLocalBoundary& operator=(const dtLocalBoundary&);
};

#endif // DETOURLOCALBOUNDARY_H
