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
	
	// �߶εĽṹ��
	struct Segment
	{
		// �߶ε���ʼ����ֹλ��
		float s[6];	///< Segment start/end
		// �����޼��ľ���
		float d;	///< Distance for pruning.
	};
	
	// ���ĵ�����
	float m_center[3];
	// ����Ϊ���ϰ�����߶�
	Segment m_segs[MAX_LOCAL_SEGS];
	// �߶ε�ʵ������
	int m_nsegs;
	
	// ����Ϊ���ϰ���Ķ����
	dtPolyRef m_polys[MAX_LOCAL_POLYS];
	// ʵ�ʵĶ��������
	int m_npolys;

	// ���ݴ���֮��ľ��룬������һ�������Ĵ�����뵽��ǰ������ھ��б��С�
	void addSegment(const float dist, const float* s);
	
public:
	dtLocalBoundary();
	~dtLocalBoundary();
	
	// ��������
	void reset();
	
	// ���¾ֲ������߽磬��������·����ѯ�򵼺������ķ�Χ���Ա��������벻�ɷ��ʵ�����
	void update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
				dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	// �ڽ���·����ѯ�򵼺��������ʱ��ʹ�ö���β�ѯ������������ж����
	// ȷ��������Ȼ���ϲ�ѯ������
	bool isValid(dtNavMeshQuery* navquery, const dtQueryFilter* filter);
	
	// ������ĵ�λ��
	inline const float* getCenter() const { return m_center; }

	// ����߶ε�����
	inline int getSegmentCount() const { return m_nsegs; }

	// ���һ���߶ε���ʼ����ֹλ��
	inline const float* getSegment(int i) const { return m_segs[i].s; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtLocalBoundary(const dtLocalBoundary&);
	dtLocalBoundary& operator=(const dtLocalBoundary&);
};

#endif // DETOURLOCALBOUNDARY_H
