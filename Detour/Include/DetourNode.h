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

// Ѱ·�ڵ�ı�ʶ��
enum dtNodeFlags
{
	// �ڵ��ڿ����б���
	DT_NODE_OPEN = 0x01,
	// �ڵ��ڹر��б���
	DT_NODE_CLOSED = 0x02,
	// ���ڵ㲻���ڣ�ʹ������ͶӰ�ҵ���
	DT_NODE_PARENT_DETACHED = 0x04 // parent of the node is not adjacent. Found using raycast.
};

typedef unsigned short dtNodeIndex;
static const dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;

static const int DT_NODE_PARENT_BITS = 24;
static const int DT_NODE_STATE_BITS = 2;

// Ѱ··���ڵ�
struct dtNode
{
	// ����
	float pos[3];								///< Position of the node.
	// ����·����Ѱ·����
	float cost;									///< Cost from previous node to current node.
	// �ܵ�Ѱ··������
	float total;								///< Cost up to the node.
	// ���ڵ������
	unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	// �����״̬��Ϣ��һ������ο���ӵ�ж����ͬ��չ��Ϣ�Ľڵ�
	unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	// �ڵ��ʶ��
	unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.
	// �ڵ����õĶ����id
	dtPolyRef id;								///< Polygon ref the node corresponds to.
};

static const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;	// number of extra states per node. See dtNode::state

// ʵ��һ����ϣ�ڵ�ض���
class dtNodePool
{
public:
	// maxNodes ���ڵ���
	// hashSize ��ϣ���С
	dtNodePool(int maxNodes, int hashSize);
	~dtNodePool();

	// ���ù�ϣ������ʹ�õĹ�ϣ�ڵ�����Ϊ��
	// ���ù�ϣͰ��Ӧ�Ľڵ�����
	void clear();

	// Get a dtNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	// ����ĳ��Ͱ�ڵĽڵ����������id �� id ��ͬ�Ľڵ㡣
	// �����ʹ�ýڵ�û�дﵽ���ڵ������� new һ���½ڵ㣬���򷵻�һ����ָ��
	dtNode* getNode(dtPolyRef id, unsigned char state=0);
	// ����ĳ��Ͱ�ڵĽڵ����������id �� id ��ͬ�Ľڵ㡣����ڵ㲻���ڣ��򷵻�һ����ָ��
	dtNode* findNode(dtPolyRef id, unsigned char state);
	// ����ĳ��Ͱ�ڵĽڵ����������id �� id ��ͬ�Ľڵ��б�ͨ�� maxNodes ָ���б��С
	unsigned int findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes);
	// ��ͨ���ڵ��ַ - �ڵ������ַ�� + 1��ȡ�ýڵ��Ӧ������
	inline unsigned int getNodeIdx(const dtNode* node) const
	{
		if (!node) return 0;
		return (unsigned int)(node - m_nodes) + 1;
	}

	// ��ȡָ������λ�õĽڵ�ָ�룬idx ������ڵ���1��С�ڵ��� m_maxNodes
	inline dtNode* getNodeAtIdx(unsigned int idx)
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	// ��ȡָ������λ�õĽڵ�ָ�룬idx ������ڵ���1��С�ڵ��� m_maxNodes
	inline const dtNode* getNodeAtIdx(unsigned int idx) const
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}
	
	// ��õ�ǰ�ڵ�ص��ڴ�ʹ����
	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode)*m_maxNodes +
			sizeof(dtNodeIndex)*m_maxNodes +
			sizeof(dtNodeIndex)*m_hashSize;
	}
	
	// ������ڵ���
	inline int getMaxNodes() const { return m_maxNodes; }
	
	// ��ù�ϣͰ��Ԫ�������������
	inline int getHashSize() const { return m_hashSize; }
	// ��ȡĳ����ϣͰ�洢�Ľڵ�����
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	// ͨ��Ԫ���±��ȡ�ڵ������
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	// ��ȡ��ʹ�õĽڵ�����
	inline int getNodeCount() const { return m_nodeCount; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodePool(const dtNodePool&);
	dtNodePool& operator=(const dtNodePool&);
	
	// �洢���нڵ������
	dtNode* m_nodes;
	// ��ʼ�� m_hashSize ���ڵ�����
	dtNodeIndex* m_first;
	// ��ʼ�� m_maxNodes ���ڵ�����
	dtNodeIndex* m_next;
	// �ڵ���������
	const int m_maxNodes;
	// ÿ����ϣͰ�洢�Ľڵ�����
	const int m_hashSize;
	// ��ʹ�õĽڵ�����
	int m_nodeCount;
};

// �ڵ����
class dtNodeQueue
{
public:
	// n ���е�����
	dtNodeQueue(int n);
	~dtNodeQueue();
	
	// ���ñ����Ԫ����������
	inline void clear() { m_size = 0; }
	
	// ��ȡ���еĵ�һ��Ԫ��
	inline dtNode* top() { return m_heap[0]; }
	
	// ������һ��Ԫ��
	inline dtNode* pop()
	{
		dtNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	// ����һ��Ԫ��
	inline void push(dtNode* node)
	{
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	// �޸Ķ����е�Ԫ��
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
	
	// �ж϶����Ƿ�Ϊ��
	inline bool empty() const { return m_size == 0; }
	
	// ��ȡ���е��ڴ�ʹ����
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtNode*) * (m_capacity + 1);
	}
	
	// ��ȡ������������
	inline int getCapacity() const { return m_capacity; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodeQueue(const dtNodeQueue&);
	dtNodeQueue& operator=(const dtNodeQueue&);

	// ð�����򣬰� node ������ʵ�λ��
	void bubbleUp(int i, dtNode* node);
	// ��С�ѵ����˲��� 
	void trickleDown(int i, dtNode* node);
	
	// �洢�ڵ�ָ�������
	dtNode** m_heap;
	// ���е���������
	const int m_capacity;
	// �����д洢��Ԫ������
	int m_size;
};		


#endif // DETOURNODE_H
