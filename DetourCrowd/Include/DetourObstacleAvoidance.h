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

#ifndef DETOUROBSTACLEAVOIDANCE_H
#define DETOUROBSTACLEAVOIDANCE_H

// 圆柱形的障碍物
struct dtObstacleCircle
{
	// 障碍物的位置
	float p[3];				///< Position of the obstacle
	// 障碍物的移动速度
	float vel[3];			///< Velocity of the obstacle
	// 障碍物的期望移动速度
	float dvel[3];			///< Velocity of the obstacle
	// 障碍物的半径
	float rad;				///< Radius of the obstacle
	// 在进行采样过程中，用于选择侧面的用途
	// dp 代理与障碍物的距离
	float dp[3], np[3];		///< Use for side selection during sampling.
};

// 代表障碍物的线段
struct dtObstacleSegment
{
	// 障碍物线段的两个端点
	float p[3], q[3];		///< End points of the obstacle segment
	// 标识是否已经触碰到了障碍物
	bool touch;
};

// 用于避障调试的数据
class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();
	
	// 初始化，分配内存
	bool init(const int maxSamples);
	// 重置函数
	void reset();

	// 添加采样数据
	// vel 采样的速度向量
	// ssize 在采样过程中调整速度的向量
	// pen vpen + vcpen + spen + tpen 的代价总和
	// vpen（vertex penalty） 顶点代价项，表示起点或目标点于当前节点的距离代价。使用欧几里距离或曼哈顿距离等方式计算
	// vcpen（vertex cost penalty） 顶点成本代价项，表示起点或目标点与当前节点的成本代价。这是基于顶点的附加代价，可能包括阻碍物、地形坡度等信息。
	// spen（straight penalty） 直线路径代价项，表示当前节点到相邻节点之间的直线路径代价。这是基于直线路径的附加代价，可以用于对曲线路径进行惩罚。
	// tpen（turn penalty） 转弯代价项，表示在当前节点处进行转弯的代价。转弯通常会增加路径的长度或时间，因此可以通过该项代价对转弯进行惩罚。
	void addSample(const float* vel, const float ssize, const float pen,
				   const float vpen, const float vcpen, const float spen, const float tpen);
	
	// 对样本做归一化处理
	void normalizeSamples();
	
	inline int getSampleCount() const { return m_nsamples; }
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i*3]; }
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData&);
	dtObstacleAvoidanceDebugData& operator=(const dtObstacleAvoidanceDebugData&);

	// 样本数量
	int m_nsamples;
	// 最大样本数量
	int m_maxSamples;
	// 采样的速度向量
	float* m_vel;
	// 在采样过程中调整速度的向量
	float* m_ssize;
	// 总的代价（顶点代价项 + 顶点成本代价项 + 直线路径代价项 + 转弯代价项）
	float* m_pen;
	// 顶点代价项
	float* m_vpen;
	// 顶点成本代价项
	float* m_vcpen;
	// 直线路径代价项
	float* m_spen;
	// 转弯代价项
	float* m_tpen;
};

dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);

// 在路径平滑过程中，用于自适应分割的最大数量
// 较少的分割点会使路径更接近于原始路径，但可能不够平滑
// 相反，更多的分割点可使路径更加平滑，但可能对原始的路径偏移较大
static const int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.

// 在路径平滑过程中用于自适应环插值的最大数量。其作用与DT_MAX_PATTERN_DIVS类似
static const int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

// 避障配置参数
struct dtObstacleAvoidanceParams
{
	// 速度偏差因子
	float velBias;
	// 期望速度的权重
	float weightDesVel;
	// 当前速度的权重
	float weightCurVel;
	float weightSide;
	// Toi（time of impact）碰撞发生的时间
	// 碰撞时间的权重
	float weightToi;
	// horional time 在平面（水平）方向上移动的时间或成本
	float horizTime;
	// 格子的数量，在 recastnavigation 中表示行、列，记作：m * n
	unsigned char gridSize;	///< grid
	// 自适应分割，用于控制搜索过程中网格分割的细化程度
	unsigned char adaptiveDivs;	///< adaptive
	// 自适应环，用于控制搜索过程中扩展的环的数量和范围
	unsigned char adaptiveRings;	///< adaptive
	// 自适应深度，用于控制搜索过程中的递归深度
	unsigned char adaptiveDepth;	///< adaptive
};

// 避障查询
class dtObstacleAvoidanceQuery
{
public:
	dtObstacleAvoidanceQuery();
	~dtObstacleAvoidanceQuery();
	
	// 避障查询初始化函数
	// 分配圆柱体、线段数组的内存
	bool init(const int maxCircles, const int maxSegments);
	
	// 重置圆柱体、线段的偏移量（代表已存储的元素数量）
	void reset();

	// 添加圆柱体障碍物
	// pos 中心点
	// rad 碰撞检测半径
	// vel 移动速度向量
	// dvel 期望的移动速度向量
	void addCircle(const float* pos, const float rad,
				   const float* vel, const float* dvel);
	
	// 添加线段障碍物
	// p 线段的起点
	// q 线段的终点
	void addSegment(const float* p, const float* q);

	// 基于格子的采样点
	// pos 当前位置
	// rad 碰撞检测半径
	// vmax 期望的移动速度
	// vel 移动速度向量
	// dvel 期望的移动速度向量
	// nvel 经过障碍物避让调整后的期望速度
	// params 避障配置参数
	// debug 避障调试数据对象
	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
						   const float* vel, const float* dvel, float* nvel,
						   const dtObstacleAvoidanceParams* params,
						   dtObstacleAvoidanceDebugData* debug = 0);

	// 自适应速度采样点
	// pos 当前位置
	// rad 碰撞检测半径
	// vmax 期望的移动速度
	// vel 移动速度向量
	// dvel 期望的移动速度向量
	// nvel 经过障碍物避让调整后的期望速度
	// params 避障配置参数
	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
							   const float* vel, const float* dvel, float* nvel,
							   const dtObstacleAvoidanceParams* params, 
							   dtObstacleAvoidanceDebugData* debug = 0);
	
	inline int getObstacleCircleCount() const { return m_ncircles; }
	const dtObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	inline int getObstacleSegmentCount() const { return m_nsegments; }
	const dtObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery&);
	dtObstacleAvoidanceQuery& operator=(const dtObstacleAvoidanceQuery&);

	// 预处理
	void prepare(const float* pos, const float* dvel);

	/*
	*
	* 计算给定速度向量的碰撞代价
	*
	* vcand 采样的速度向量
	* cs 在采样过程中调整速度的向量
	* pos 代理的当前位置
	* rad 代理的半径
	* vel 代理的移动速度向量
	* dvel 期望的速度向量
	* minPenalty 最小代价
	*
	*/
	float processSample(const float* vcand, const float cs,
						const float* pos, const float rad,
						const float* vel, const float* dvel,
						const float minPenalty,
						dtObstacleAvoidanceDebugData* debug);

	// 用于避障查询的配置参数
	dtObstacleAvoidanceParams m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	// 圆柱体的最大数量上限
	int m_maxCircles;
	// 用于容纳圆柱体的数组
	dtObstacleCircle* m_circles;
	// 当前存储的圆柱体数量
	int m_ncircles;

	// 线段的最大数量上限
	int m_maxSegments;
	// 用于容纳线段的数组
	dtObstacleSegment* m_segments;
	// 当前存储的线段数量
	int m_nsegments;
};

dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);


#endif // DETOUROBSTACLEAVOIDANCE_H
