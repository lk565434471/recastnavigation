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

#ifndef DETOURNODE_H
#define DETOURNODE_H

#include "DetourNavMesh.h"

// 寻路节点的标识符
enum dtNodeFlags
{
	// 节点在开放列表中
	DT_NODE_OPEN = 0x01,
	// 节点在关闭列表中
	DT_NODE_CLOSED = 0x02,
	// 父节点不相邻，使用射线投影找到的
	DT_NODE_PARENT_DETACHED = 0x04 // parent of the node is not adjacent. Found using raycast.
};

typedef unsigned short dtNodeIndex;
static const dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;

static const int DT_NODE_PARENT_BITS = 24;
static const int DT_NODE_STATE_BITS = 2;

// 寻路路径节点
struct dtNode
{
	// 坐标
	float pos[3];								///< Position of the node.
	// 单个路径的寻路代价
	float cost;									///< Cost from previous node to current node.
	// 总的寻路路径代价
	float total;								///< Cost up to the node.
	// 父节点的索引
	unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	// 额外的状态信息。一个多边形可以拥有多个不同扩展信息的节点
	unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	// 节点标识符
	unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.
	// 节点引用的多边形id
	dtPolyRef id;								///< Polygon ref the node corresponds to.
};

static const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;	// number of extra states per node. See dtNode::state

// 实现一个哈希节点池对象
class dtNodePool
{
public:
	// maxNodes 最大节点数
	// hashSize 哈希表大小
	dtNodePool(int maxNodes, int hashSize);
	~dtNodePool();

	// 重置哈希表，将已使用的哈希节点重置为零
	// 重置哈希桶对应的节点索引
	void clear();

	// Get a dtNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	// 查找某个桶内的节点所属多边形id 与 id 相同的节点。
	// 如果已使用节点没有达到最大节点数，则 new 一个新节点，否则返回一个空指针
	dtNode* getNode(dtPolyRef id, unsigned char state=0);
	// 查找某个桶内的节点所属多边形id 与 id 相同的节点。如果节点不存在，则返回一个空指针
	dtNode* findNode(dtPolyRef id, unsigned char state);
	// 查找某个桶内的节点所属多边形id 与 id 相同的节点列表，通过 maxNodes 指定列表大小
	unsigned int findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes);
	// （通过节点地址 - 节点数组地址） + 1，取得节点对应的索引
	inline unsigned int getNodeIdx(const dtNode* node) const
	{
		if (!node) return 0;
		return (unsigned int)(node - m_nodes) + 1;
	}

	// 获取指定索引位置的节点指针，idx 必须大于等于1，小于等于 m_maxNodes
	inline dtNode* getNodeAtIdx(unsigned int idx)
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	// 获取指定索引位置的节点指针，idx 必须大于等于1，小于等于 m_maxNodes
	inline const dtNode* getNodeAtIdx(unsigned int idx) const
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}
	
	// 获得当前节点池的内存使用量
	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode)*m_maxNodes +
			sizeof(dtNodeIndex)*m_maxNodes +
			sizeof(dtNodeIndex)*m_hashSize;
	}
	
	// 获得最大节点数
	inline int getMaxNodes() const { return m_maxNodes; }
	
	// 获得哈希桶的元素最大数量上限
	inline int getHashSize() const { return m_hashSize; }
	// 获取某个哈希桶存储的节点索引
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	// 通过元素下标获取节点的索引
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	// 获取已使用的节点数量
	inline int getNodeCount() const { return m_nodeCount; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodePool(const dtNodePool&);
	dtNodePool& operator=(const dtNodePool&);
	
	// 存储所有节点的数组
	dtNode* m_nodes;
	// 初始化 m_hashSize 个节点索引
	dtNodeIndex* m_first;
	// 初始化 m_maxNodes 个节点索引
	dtNodeIndex* m_next;
	// 节点数量上限
	const int m_maxNodes;
	// 每个哈希桶存储的节点数量
	const int m_hashSize;
	// 已使用的节点数量
	int m_nodeCount;
};

// 节点队列
class dtNodeQueue
{
public:
	// n 队列的容量
	dtNodeQueue(int n);
	~dtNodeQueue();
	
	// 重置保存的元素数量计数
	inline void clear() { m_size = 0; }
	
	// 获取队列的第一个元素
	inline dtNode* top() { return m_heap[0]; }
	
	// 弹出第一个元素
	inline dtNode* pop()
	{
		dtNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	// 插入一个元素
	inline void push(dtNode* node)
	{
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	// 修改队列中的元素
	inline void modify(dtNode* node)
	{
		for (int i = 0; i < m_size; ++i)
		{
			if (m_heap[i] == node)
			{
				bubbleUp(i, node);
				return;
			}
		}
	}
	
	// 判断队列是否为空
	inline bool empty() const { return m_size == 0; }
	
	// 获取队列的内存使用量
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtNode*) * (m_capacity + 1);
	}
	
	// 获取队列容量上限
	inline int getCapacity() const { return m_capacity; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodeQueue(const dtNodeQueue&);
	dtNodeQueue& operator=(const dtNodeQueue&);

	// 冒泡排序，把 node 放入合适的位置
	void bubbleUp(int i, dtNode* node);
	// 最小堆的下滤操作 
	void trickleDown(int i, dtNode* node);
	
	// 存储节点指针的数组
	dtNode** m_heap;
	// 队列的容量上限
	const int m_capacity;
	// 队列中存储的元素数量
	int m_size;
};		


#endif // DETOURNODE_H
