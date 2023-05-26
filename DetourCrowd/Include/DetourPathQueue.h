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

#ifndef DETOURPATHQUEUE_H
#define DETOURPATHQUEUE_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

static const unsigned int DT_PATHQ_INVALID = 0;

typedef unsigned int dtPathQueueRef;

class dtPathQueue
{
	struct PathQuery
	{
		dtPathQueueRef ref;
		/// Path find start and end location.
		// �����յ�� (x, y, z)
		float startPos[3], endPos[3];
		// �����յ������Ķ����id
		dtPolyRef startRef, endRef;
		/// Result.
		// ��������Ķ����id
		dtPolyRef* path;
		// ·���ڵ�����
		int npath;
		/// State.
		// ·��״̬
		dtStatus status;
		int keepAlive;
		// ·����ѯ������
		const dtQueryFilter* filter; ///< TODO: This is potentially dangerous!
	};
	
	// ����ѯ·����������
	static const int MAX_QUEUE = 8;
	// ��ѯ·������
	PathQuery m_queue[MAX_QUEUE];
	// ��һ����ѯ����id
	dtPathQueueRef m_nextHandle;
	// ·���������
	int m_maxPathSize;
	// ��ѯ���е�ͷ
	int m_queueHead;
	// ���������ѯ��
	dtNavMeshQuery* m_navquery;
	
	// ��Դ���ٺ������ͷŵ������������ڴ�
	void purge();
	
public:
	dtPathQueue();
	~dtPathQueue();
	
	// ��ʼ������
	// maxPathSize ���·������
	// maxSearchNodeCount ��������ڵ�����
	// nav �����������
	bool init(const int maxPathSize, const int maxSearchNodeCount, dtNavMesh* nav);
	
	// maxIters �������Ĵ���
	void update(const int maxIters);
	
	// startRef ������εĸ���id
	// endRef �յ����εĸ���id
	// startPos �������
	// endPos �յ�����
	// filter ·����ѯ����ɸѡ��
	dtPathQueueRef request(dtPolyRef startRef, dtPolyRef endRef,
						   const float* startPos, const float* endPos, 
						   const dtQueryFilter* filter);
	
	dtStatus getRequestStatus(dtPathQueueRef ref) const;
	
	// ����·�������Ľ��
	dtStatus getPathResult(dtPathQueueRef ref, dtPolyRef* path, int* pathSize, const int maxPath);
	
	inline const dtNavMeshQuery* getNavQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtPathQueue(const dtPathQueue&);
	dtPathQueue& operator=(const dtPathQueue&);
};

#endif // DETOURPATHQUEUE_H
