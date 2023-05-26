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
		// 起点和终点的 (x, y, z)
		float startPos[3], endPos[3];
		// 起点和终点所属的多边形id
		dtPolyRef startRef, endRef;
		/// Result.
		// 搜索结果的多边形id
		dtPolyRef* path;
		// 路径节点数量
		int npath;
		/// State.
		// 路径状态
		dtStatus status;
		int keepAlive;
		// 路径查询过滤器
		const dtQueryFilter* filter; ///< TODO: This is potentially dangerous!
	};
	
	// 最大查询路径数量上限
	static const int MAX_QUEUE = 8;
	// 查询路径数组
	PathQuery m_queue[MAX_QUEUE];
	// 下一个查询队列id
	dtPathQueueRef m_nextHandle;
	// 路径最大数量
	int m_maxPathSize;
	// 查询队列的头
	int m_queueHead;
	// 导航网格查询器
	dtNavMeshQuery* m_navquery;
	
	// 资源销毁函数，释放导航网格分配的内存
	void purge();
	
public:
	dtPathQueue();
	~dtPathQueue();
	
	// 初始化函数
	// maxPathSize 最大路径数量
	// maxSearchNodeCount 最大搜索节点数量
	// nav 导航网格对象
	bool init(const int maxPathSize, const int maxSearchNodeCount, dtNavMesh* nav);
	
	// maxIters 最大遍历的次数
	void update(const int maxIters);
	
	// startRef 起点多边形的复合id
	// endRef 终点多边形的复合id
	// startPos 起点坐标
	// endPos 终点坐标
	// filter 路径查询条件筛选器
	dtPathQueueRef request(dtPolyRef startRef, dtPolyRef endRef,
						   const float* startPos, const float* endPos, 
						   const dtQueryFilter* filter);
	
	dtStatus getRequestStatus(dtPathQueueRef ref) const;
	
	// 拷贝路径搜索的结果
	dtStatus getPathResult(dtPathQueueRef ref, dtPolyRef* path, int* pathSize, const int maxPath);
	
	inline const dtNavMeshQuery* getNavQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtPathQueue(const dtPathQueue&);
	dtPathQueue& operator=(const dtPathQueue&);
};

#endif // DETOURPATHQUEUE_H
