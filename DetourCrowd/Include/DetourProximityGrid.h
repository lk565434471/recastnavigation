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
	// 单元格大小
	float m_cellSize;
	// 单元格大小的倒数
	float m_invCellSize;
	
	// 内存池的元素项
	// 该元素项存储的是群集代理的指引信息
	// 例如：群集代理在群集代理数组中的下标，则存储在id字段上。
	struct Item
	{
		// 群集代理在数组中的索引
		unsigned short id;
		// 单元格的行列信息
		short x,y;
		// 指向下一个 Item 的索引
		unsigned short next;
	};

	// 群集代理的内存池
	Item* m_pool;
	// m_pool 的偏移量，表示已添加的元素数量
	int m_poolHead;
	// m_pool 的容量上限
	int m_poolSize;
	
	// 哈希桶缓冲区
	unsigned short* m_buckets;
	// 哈希桶的容量
	int m_bucketsSize;
	
	// 它取所有代理中的最小点和最大点的坐标值，作为边界框的范围
	int m_bounds[4];
	
public:
	dtProximityGrid();
	~dtProximityGrid();
	
	// 初始化函数
	// poolSize 内存池大小
	// cellSize 单元格大小
	bool init(const int poolSize, const float cellSize);
	
	// 重置 m_bounds、m_buckets、m_poolHead 数据
	void clear();
	
	void addItem(const unsigned short id,
				 const float minx, const float miny,
				 const float maxx, const float maxy);
	
	// 查询给定矩形范围内的元素
	// minx、miny、maxx、maxy 确定需要遍历的网格范围
	// ids 存储的是群集代理在群集代理数组中的索引
	// maxIds 可以容纳邻居的容量上限
	int queryItems(const float minx, const float miny,
				   const float maxx, const float maxy,
				   unsigned short* ids, const int maxIds) const;
	
	// 获取在指定网格中包含的群集代理数量
	int getItemCountAt(const int x, const int y) const;
	
	// 获得网格遍历的区间范围
	inline const int* getBounds() const { return m_bounds; }
	// 获得单元格大小
	inline float getCellSize() const { return m_cellSize; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtProximityGrid(const dtProximityGrid&);
	dtProximityGrid& operator=(const dtProximityGrid&);
};

dtProximityGrid* dtAllocProximityGrid();
void dtFreeProximityGrid(dtProximityGrid* ptr);


#endif // DETOURPROXIMITYGRID_H

