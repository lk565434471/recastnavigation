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

#ifndef DETOURPROXIMITYGRID_H
#define DETOURPROXIMITYGRID_H

class dtProximityGrid
{
	// ��Ԫ���С
	float m_cellSize;
	// ��Ԫ���С�ĵ���
	float m_invCellSize;
	
	// �ڴ�ص�Ԫ����
	// ��Ԫ����洢����Ⱥ�������ָ����Ϣ
	// ���磺Ⱥ��������Ⱥ�����������е��±꣬��洢��id�ֶ��ϡ�
	struct Item
	{
		// Ⱥ�������������е�����
		unsigned short id;
		// ��Ԫ���������Ϣ
		short x,y;
		// ָ����һ�� Item ������
		unsigned short next;
	};

	// Ⱥ��������ڴ��
	Item* m_pool;
	// m_pool ��ƫ��������ʾ����ӵ�Ԫ������
	int m_poolHead;
	// m_pool ����������
	int m_poolSize;
	
	// ��ϣͰ������
	unsigned short* m_buckets;
	// ��ϣͰ������
	int m_bucketsSize;
	
	// ��ȡ���д����е���С������������ֵ����Ϊ�߽��ķ�Χ
	int m_bounds[4];
	
public:
	dtProximityGrid();
	~dtProximityGrid();
	
	// ��ʼ������
	// poolSize �ڴ�ش�С
	// cellSize ��Ԫ���С
	bool init(const int poolSize, const float cellSize);
	
	// ���� m_bounds��m_buckets��m_poolHead ����
	void clear();
	
	void addItem(const unsigned short id,
				 const float minx, const float miny,
				 const float maxx, const float maxy);
	
	// ��ѯ�������η�Χ�ڵ�Ԫ��
	// minx��miny��maxx��maxy ȷ����Ҫ����������Χ
	// ids �洢����Ⱥ��������Ⱥ�����������е�����
	// maxIds ���������ھӵ���������
	int queryItems(const float minx, const float miny,
				   const float maxx, const float maxy,
				   unsigned short* ids, const int maxIds) const;
	
	// ��ȡ��ָ�������а�����Ⱥ����������
	int getItemCountAt(const int x, const int y) const;
	
	// ���������������䷶Χ
	inline const int* getBounds() const { return m_bounds; }
	// ��õ�Ԫ���С
	inline float getCellSize() const { return m_cellSize; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtProximityGrid(const dtProximityGrid&);
	dtProximityGrid& operator=(const dtProximityGrid&);
};

dtProximityGrid* dtAllocProximityGrid();
void dtFreeProximityGrid(dtProximityGrid* ptr);


#endif // DETOURPROXIMITYGRID_H

