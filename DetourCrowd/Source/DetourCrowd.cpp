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

#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <new>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"


dtCrowd* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(dtCrowd), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowd;
}

void dtFreeCrowd(dtCrowd* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowd();
	dtFree(ptr);
}


static const int MAX_ITERS_PER_UPDATE = 100;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;

inline float tween(const float t, const float t0, const float t1)
{
	return dtClamp((t-t0) / (t1-t0), 0.0f, 1.0f);
}

// 通过增量时间、加速度、移动速度等参数，计算出群集代理的新位置
// 这个函数在 update 中进行调用
static void integrate(dtCrowdAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->params.maxAcceleration * dt;
	float dv[3];
	dtVsub(dv, ag->nvel, ag->vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(ag->vel, ag->vel, dv);
	
	// Integrate
	if (dtVlen(ag->vel) > 0.0001f)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel,0,0,0);
}

// 穿过离散连接测试
static bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
{
	if (!ag->ncorners)
		return false;
	
	// 检测是否是离线连接
	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}
	
	return false;
}

static float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);
	
	return range;
}

/*
* 
* 用于计算平滑的转向方向。它会考虑当前位置和目标位置之间的路径，并尽量平滑地转向以沿着路径移动。
* 它使用了路径的曲率和当前位置与路径的夹角等信息来计算转向方向，以实现更自然和平滑的转向行为。
* 
*/
static void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	
	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners-1);
	const float* p0 = &ag->cornerVerts[ip0*3];
	const float* p1 = &ag->cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;
	
	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);
	
	dir[0] = dir0[0] - dir1[0]*len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2]*len0*0.5f;
	
	dtVnormalize(dir);
}

/*
* 
* 用于计算直接的转向方向。它简单地计算当前位置指向目标位置的直线方向，并直接转向该方向。
* 它不考虑路径的曲率和当前位置与路径的夹角，因此转向行为可能会更直接和快速。
* 
*/
static void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

// 添加邻居
// idx 在群集代理数组中的下标
// dist 群集代理之间的半径距离
// neis 存储当前群集代理的邻居的数组
// nneis 邻居在 neis 数组中的下标
// maxNeis neis 可以容纳的元素数量上限
static int addNeighbour(const int idx, const float dist,
						dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	// 根据距离插入邻居信息
	dtCrowdNeighbour* nei = 0;
	// 如果没有现有的邻居，插入到第一个位置
	if (!nneis)
	{
		nei = &neis[nneis];
	}
	// 如果距离大于等于最后一个邻居的距离，则在最后一个位置插入（如果有空间）
	else if (dist >= neis[nneis-1].dist)
	{
		if (nneis >= maxNeis)
			return nneis;
		nei = &neis[nneis];
	}
	// 否则，根据距离找到合适的位置插入新邻居
	else
	{
		int i;
		for (i = 0; i < nneis; ++i)
			if (dist <= neis[i].dist)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nneis-i, maxNeis-tgt);
		
		dtAssert(tgt+n <= maxNeis);
		
		// 如果有邻居需要移动，将它们移动到新的位置
		if (n > 0)
			memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour)*n);
		nei = &neis[i];
	}
	
	// 使用给定的索引和距离初始化新邻居
	memset(nei, 0, sizeof(dtCrowdNeighbour));
	
	nei->idx = idx;
	nei->dist = dist;
	
	// 返回更新后的邻居数（受最大邻居数限制）
	return dtMin(nneis+1, maxNeis);
}

// pos 当前代理的坐标
// height 当前代理的高度
// range 当前代理的碰撞检查范围
// skip 不需要检测的代理（即当前代理）
// result 存储邻居代理的数组
// maxResult result 可以容纳群集代理元素的数量上限
// agents 群集代理数组
// nagents 群集代理数组的元素数量
// grid 邻近网格对象
static int getNeighbours(const float* pos, const float height, const float range,
						 const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
						 dtCrowdAgent** agents, const int /*nagents*/, dtProximityGrid* grid)
{
	int n = 0;
	
	// 最大可以容纳邻居的数量上限
	static const int MAX_NEIS = 32;
	// ids 中存储的是群集代理在 agents 中的下标
	unsigned short ids[MAX_NEIS];
	// 确定一个矩形的检测范围，并把符合条件的元素存储到ids数组
	int nids = grid->queryItems(pos[0]-range, pos[2]-range,
								pos[0]+range, pos[2]+range,
								ids, MAX_NEIS);
	
	for (int i = 0; i < nids; ++i)
	{
		const dtCrowdAgent* ag = agents[ids[i]];
		
		// 如果代理不需要处理，则跳过
		if (ag == skip) continue;
		
		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		// 判断代理之间是否会发生垂直方向上的穿透或重叠。以维持代理之间的安全间隔。
		if (dtMathFabsf(diff[1]) >= (height+ag->params.height)/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		// 检测邻居间的半径距离
		if (distSqr > dtSqr(range))
			continue;
		
		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}

// 将新代理插入到操作队列
static int addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	// 根据最大时间插入相邻节点
	int slot = 0;
	// 如果当前路径队列中没有代理，则将新代理插入到队列末尾
	if (!nagents)
	{
		slot = nagents;
	}
	// 如果新代理的拓扑结构操作时间小于等于代理队列中最后一个代理的拓扑结构操作时间
	// 则将新代理插入到队列的末尾
	else if (newag->topologyOptTime <= agents[nagents-1]->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	// 找到合适的位置插入新代理
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->topologyOptTime >= agents[i]->topologyOptTime)
				break;
		
		// 计算需要移动的代理数量
		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);
		
		dtAssert(tgt+n <= maxAgents);
		
		// 判断是否需要移动已经存在的代理
		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		// 新代理插入的位置
		slot = i;
	}
	
	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}

// 将新代理插入到路径队列
// newag 新插入的代理
// agents 代理数组
// nagents 当前路径队列中的代理数量
// maxAgents 路径队列可以存储的代理最大数量

// 返回当前路径队列中的代理数量
static int addToPathQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	// 根据最大时间插入相邻节点
	int slot = 0;
	// 如果当前路径队列中没有代理，则将新代理插入到队列末尾
	if (!nagents)
	{
		slot = nagents;
	}
	// 如果新代理的目标重新规划时间小于等于路径队列中最后一个代理的目标重新规划时间
	// 则将新代理插入到队列的末尾
	else if (newag->targetReplanTime <= agents[nagents-1]->targetReplanTime)
	{
		// 这里还检查了路径队列中的代理是否已被填满
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	// 找到合适的位置插入新代理
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->targetReplanTime >= agents[i]->targetReplanTime)
				break;
		
		// 计算需要移动的代理数量
		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);
		
		dtAssert(tgt+n <= maxAgents);
		
		// 判断是否需要移动已经存在的代理
		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		// 新代理插入的位置
		slot = i;
	}
	
	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}


/**
@class dtCrowd
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes: 

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #dtCrowdAgent::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.  
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent

*/

dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
	m_obstacleQuery(0),
	m_grid(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_maxAgentRadius(0),
	m_velocitySampleCount(0),
	m_navquery(0)
{
}

dtCrowd::~dtCrowd()
{
	purge();
}


void dtCrowd::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent();
	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;
	
	dtFree(m_activeAgents);
	m_activeAgents = 0;

	dtFree(m_agentAnims);
	m_agentAnims = 0;
	
	dtFree(m_pathResult);
	m_pathResult = 0;
	
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;
	
	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
/// 可以多次调用以重新初始化和清除群集代理对象
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	// 清除群集代理，重置成员变量
	purge();
	
	// 设置初始的群集代理数量上限
	m_maxAgents = maxAgents;
	// 设置群集代理的最大半径
	m_maxAgentRadius = maxAgentRadius;

	// Larger than agent radius because it is also used for agent recovery.
	// 大于群集代理半径，因为它用于代理在某些情况下的异常恢复。
	// 比如：陷入无法行走的区域
	dtVset(m_agentPlacementHalfExtents, m_maxAgentRadius*2.0f, m_maxAgentRadius*1.5f, m_maxAgentRadius*2.0f);
	
	m_grid = dtAllocProximityGrid();
	if (!m_grid)
		return false;
	if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	if (!m_obstacleQuery)
		return false;
	if (!m_obstacleQuery->init(6, 8))
		return false;

	// Init obstacle query params.
	memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
	for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
	{
		dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[i];
		params->velBias = 0.4f;
		params->weightDesVel = 2.0f;
		params->weightCurVel = 0.75f;
		params->weightSide = 0.75f;
		params->weightToi = 2.5f;
		params->horizTime = 2.5f;
		params->gridSize = 33;
		params->adaptiveDivs = 7;
		params->adaptiveRings = 2;
		params->adaptiveDepth = 5;
	}
	
	// Allocate temp buffer for merging paths.
	// 分配临时缓冲区，用于合并路径
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathResult, DT_ALLOC_PERM);
	if (!m_pathResult)
		return false;
	
	if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
		return false;
	
	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;
	
	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;

	m_agentAnims = (dtCrowdAgentAnimation*)dtAlloc(sizeof(dtCrowdAgentAnimation)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agentAnims)
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = false;
		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	// dtNavMeshQuery 主要用于局部搜索，不需要使用大型的节点池。
	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;
	
	return true;
}

void dtCrowd::setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params)
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		memcpy(&m_obstacleQueryParams[idx], params, sizeof(dtObstacleAvoidanceParams));
}

const dtObstacleAvoidanceParams* dtCrowd::getObstacleAvoidanceParams(const int idx) const
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		return &m_obstacleQueryParams[idx];
	return 0;
}

int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
const dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

/// 
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
dtCrowdAgent* dtCrowd::getEditableAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

void dtCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx >= m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
/// 
/// 代理的位置将被限制在导航网格的表面上。（即平面，因为寻路算法是基于2d的）
int dtCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
{
	// Find empty slot.
	// 在群集代理池中，找到一个未被使用的插槽的索引
	// 如果索引为负数，说明内存池已满，无法添加群集代理
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	dtCrowdAgent* ag = &m_agents[idx];		
	// 更新群集代理的配置参数
	updateAgentParameters(idx, params);
	
	// Find nearest position on navmesh and place the agent there.
	// 找到距离 pos 最近的位置，并将群集代理放置在该位置。
	float nearest[3];
	// 群集代理找到的位置所属的多边形复合id
	dtPolyRef ref = 0;
	// 默认把群集代理放置在 pos 的位置上
	dtVcopy(nearest, pos);
	dtStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
	if (dtStatusFailed(status))
	{
		dtVcopy(nearest, pos);
		ref = 0;
	}
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();
	ag->partial = false;

	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	
	ag->desiredSpeed = 0;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
	
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	ag->active = true;

	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void dtCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = false;
	}
}

bool dtCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = true;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
	
	return true;
}

/// @par
/// 
/// This method is used when a new target is set.
/// 
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
/// 
/// 这个方法用于设置一个新的移动目标。
/// pos 将被限制在导航网格的平面上。
/// 移动请求将会在下一次 update 函数调用时进行处理
bool dtCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	if (!ref)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool dtCrowd::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;
	
	return true;
}

bool dtCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	dtCrowdAgent* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0,0,0);
	dtVset(ag->dvel, 0,0,0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	return true;
}

int dtCrowd::getActiveAgents(dtCrowdAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}


void dtCrowd::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;
	
	// Fire off new requests.
	// 处理所有处于激活状态且已发送移动请求的代理的移动请求
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		// 代理处于无效状态，直接跳过
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		// 目标无法到达，或者群集代理处于基于目标速度的移动方式进行移动
		// 在这两种情况下，不需要更新群集代理的移动请求
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// 代理处于移动请求状态，即已收到移动请求操作，但还没有处理请求的状态
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			// 导航系统中计算的从当前位置到请求位置的移动路径
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick search towards the goal.
			// 快速搜索到达移动请求目标的路径
			static const int MAX_ITER = 20;
			m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filters[ag->params.queryFilterType]);
			// 搜索切片路径，更新已开放列表
			m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				// 当需要重新规划路径时，尝试重用当前的路径，如果可行的话，避免在每一帧都需要完全重新规划路径。
				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				// 当目标改变时，尽可能尝试向新的目标点移动
				status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount-1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					// 对部分路径进行处理，将目标位置限制在最后一个多边形内部
					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount-1], ag->targetPos, reqPos, 0);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}
				
			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				// 在无法找到路径时，从当前位置开始重新发起路径请求。
				dtVcopy(reqPos, ag->npos);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			ag->boundary.reset();
			ag->partial = false;

			if (reqPath[reqPathCount-1] == ag->targetRef)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				ag->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				// 路径较长或潜在无法到达的情况下，进行完整规划。
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}
		
		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->targetPathqRef = m_pathq.request(ag->corridor.getLastPoly(), ag->targetRef,
											 ag->corridor.getTarget(), ag->targetPos, &m_filters[ag->params.queryFilterType]);
		if (ag->targetPathqRef != DT_PATHQ_INVALID)
			ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}

	
	// Update requests.
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	dtStatus status;

	// Process path results.
	// 处理路径查询结果
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		// 不需要更新移动
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			// 从路径队列中获取路径请求并进行处理
			status = m_pathq.getRequestStatus(ag->targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				// 当路径搜索失败时，如果目标位置仍然有效，则进行重试。
				ag->targetPathqRef = DT_PATHQ_INVALID;
				if (ag->targetRef)
					ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				ag->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);
				
				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, ag->targetPos);
				
				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				// true 路径没有到达期望的位置
				if (dtStatusDetail(status, DT_PARTIAL_RESULT))
					ag->partial = true;
				else
					ag->partial = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.
				
				// The last ref in the old path should be the same as
				// the location where the request was issued..

				// 将新的路径结果与已存在的路径进行合并。
				// 在请求处理的过程中，代理可能已经移动了，因此路径可能发生了变化。
				// 我们假设路径的终点与发出请求的位置相同。
				// 
				if (valid && path[npath-1] != res[0])
					valid = false;
				
				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath-1)+nres > m_maxPathResult)
							nres = m_maxPathResult - (npath-1);
						
						memmove(res+npath-1, res, sizeof(dtPolyRef)*nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef)*(npath-1));
						nres += npath-1;
						
						// Remove trackbacks
						for (int j = 0; j < nres; ++j)
						{
							if (j-1 >= 0 && j+1 < nres)
							{
								if (res[j-1] == res[j+1])
								{
									memmove(res+(j-1), res+(j+1), sizeof(dtPolyRef)*(nres-(j+1)));
									nres -= 2;
									j -= 2;
								}
							}
						}
						
					}
					
					// Check for partial path.
					if (res[nres-1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						status = m_navquery->closestPointOnPoly(res[nres-1], targetPos, nearest, 0);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}
				
				if (valid)
				{
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				ag->targetReplanTime = 0.0;
			}
		}
	}
	
}


void dtCrowd::updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt)
{
	if (!nagents)
		return;
	
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
			continue;
		ag->topologyOptTime += dt;
		// 拓扑结构操作的时间阈值？
		if (ag->topologyOptTime >= OPT_TIME_THR)
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navquery, &m_filters[ag->params.queryFilterType]);
		ag->topologyOptTime = 0;
	}

}

// 检测代理是否具有有效的移动路径
void dtCrowd::checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
			
		ag->targetReplanTime += dt;

		bool replan = false;

		// First check that the current location is valid.
		// 首先，检测当前位置是否有效
		const int idx = getAgentIndex(ag);
		float agentPos[3];
		// 获取移动路径的第一个多边形符合id
		dtPolyRef agentRef = ag->corridor.getFirstPoly();
		// 存储群集代理当前的位置到 agentPos
		dtVcopy(agentPos, ag->npos);
		if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			// 当前位置无效，尝试重新获取位置信息。
			float nearest[3];
			dtVcopy(nearest, agentPos);
			agentRef = 0;
			m_navquery->findNearestPoly(ag->npos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &agentRef, nearest);
			dtVcopy(agentPos, nearest);

			if (!agentRef)
			{
				// Could not find location in navmesh, set state to invalid.
				// 在导航网格中，无法找到合适的位置，将移动状态设置为无效。
				ag->corridor.reset(0, agentPos);
				ag->partial = false;
				ag->boundary.reset();
				ag->state = DT_CROWDAGENT_STATE_INVALID;
				continue;
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			// 确保第一个多边形是有效的，尽管如此，其它有效的多边形也需要被在路径中，以便重
			// 新规划器可以更好的调整路径。
			ag->corridor.fixPathStart(agentRef, agentPos);
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag->boundary.reset();
			dtVcopy(ag->npos, agentPos);

			replan = true;
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
		// 如果群集代理没有移动目标，或者由速度控制，那么无需恢复目标或重新规划。
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Try to recover move request position.
		// 尝试恢复移动请求的目标位置
		if (ag->targetState != DT_CROWDAGENT_TARGET_NONE && ag->targetState != DT_CROWDAGENT_TARGET_FAILED)
		{
			if (!m_navquery->isValidPolyRef(ag->targetRef, &m_filters[ag->params.queryFilterType]))
			{
				// Current target is not valid, try to reposition.
				// 当前目标位置无效，尝试重新定位目标位置。
				float nearest[3];
				dtVcopy(nearest, ag->targetPos);
				ag->targetRef = 0;
				m_navquery->findNearestPoly(ag->targetPos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, nearest);
				dtVcopy(ag->targetPos, nearest);
				replan = true;
			}
			if (!ag->targetRef)
			{
				// Failed to reposition target, fail moverequest.
				// 重新定位移动请求目标位置失败，移动请求失败。
				ag->corridor.reset(agentRef, agentPos);
				ag->partial = false;
				ag->targetState = DT_CROWDAGENT_TARGET_NONE;
			}
		}

		// If nearby corridor is not valid, replan.
		// 如果附近的路径无效，重新规划
		if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			// Fix current path.
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
//			ag->boundary.reset();
			replan = true;
		}
		
		// If the end of the path is near and it is not the requested location, replan.
		// 如果在路径的终点附近，且它不是请求的目标位置，则重新规划
		// 如果在同一个多边形内，其实并不会重新规划，没有要求坐标一致。
		if (ag->targetState == DT_CROWDAGENT_TARGET_VALID)
		{
			if (ag->targetReplanTime > TARGET_REPLAN_DELAY &&
				ag->corridor.getPathCount() < CHECK_LOOKAHEAD &&
				ag->corridor.getLastPoly() != ag->targetRef)
				replan = true;
		}

		// Try to replan path to goal.
		// 尝试重新规划目标路径
		if (replan)
		{
			if (ag->targetState != DT_CROWDAGENT_TARGET_NONE)
			{
				requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
			}
		}
	}
}
	
void dtCrowd::update(const float dt, dtCrowdAgentDebugInfo* debug)
{
	m_velocitySampleCount = 0;
	
	const int debugIdx = debug ? debug->idx : -1;
	
	dtCrowdAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);

	// Check that all agents still have valid paths.
	// 检测所有代理是否仍然具有有效的移动路径
	checkPathValidity(agents, nagents, dt);
	
	// Update async move request and path finder.
	// 更新异步移动请求和路径查询器
	updateMoveRequest(dt);

	// Optimize path topology.
	// 对路径的拓扑结构进行优化
	updateTopologyOptimization(agents, nagents, dt);
	
	// Register agents to proximity grid.
	// 将代理注册到邻近网格
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->params.radius;
		// 存储代理在 m_activeAgents 数组中的下标，以及所属的网格块的行列位置信息
		m_grid->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}
	
	// Get nearby navmesh segments and agents to collide with.
	// 获取与代理发生碰撞的附近导航网格段（指网格中的一部分区域）和其它代理。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		// 通过一段距离后，或者它已变为无效状态时，更新碰撞检测的边界。

		// 计算碰撞检测的阈值
		const float updateThr = ag->params.collisionQueryRange*0.25f;
		// 代理的当前位置距离中心点的距离大于碰撞检测的阈值
		// 或者，查询条件筛选器无效
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
								m_navquery, &m_filters[ag->params.queryFilterType]);
		}
		// Query neighbour agents
		// 查询群集代理的邻居代理
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
								  ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
								  agents, nagents, m_grid);
		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
	}
	
	// Find next corner to steer to.
	// 寻找下一个要转向的角点（角点是路径上的拐点或转弯点，代表需要改变方向或转向的位置）
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		// 判断群集代理是否处于可移动状态下
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		// Find corners for steering
		// 寻找适合转向的角点
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
												DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filters[ag->params.queryFilterType]);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		// 检查下一个角点是否可以直接可见，并直接到达这个捷径。
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);
			
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVcopy(debug->optStart, ag->corridor.getPos());
				dtVcopy(debug->optEnd, target);
			}
		}
		else
		{
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVset(debug->optStart, 0,0,0);
				dtVset(debug->optEnd, 0,0,0);
			}
		}
	}
	
	// Trigger off-mesh connections (depends on corners).
	// 触发离线连接（取决于角点）
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		// Check 
		// 群集代理触发半径
		const float triggerRadius = ag->params.radius*2.25f;
		// 检测是否穿过离散连接
		if (overOffmeshConnection(ag, triggerRadius))
		{
			// Prepare to off-mesh connection.
			const int idx = (int)(ag - m_agents);
			dtCrowdAgentAnimation* anim = &m_agentAnims[idx];
			
			// Adjust the path over the off-mesh connection.
			// 调整穿越离散连接的路径
			// refs[0] 存储的是可以通往离散连接点的多边形
			// refs[1] 存储的是离散连接点
			dtPolyRef refs[2];
			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners-1], refs,
													   anim->startPos, anim->endPos, m_navquery))
			{
				dtVcopy(anim->initPos, ag->npos);
				anim->polyRef = refs[1];
				anim->active = true;
				anim->t = 0.0f;
				anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;
				
				ag->state = DT_CROWDAGENT_STATE_OFFMESH;
				ag->ncorners = 0;
				ag->nneis = 0;
				continue;
			}
			else
			{
				// Path validity check will ensure that bad/blocked connections will be replanned.
			}
		}
	}
		
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;
		
		float dvel[3] = {0,0,0};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			// 计算转向方向的方式
			if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
				// 平滑的转向
				calcSmoothSteerDirection(ag, dvel);
			else
				// 直接的转向
				calcStraightSteerDirection(ag, dvel);
			
			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			// 计算速度缩放因子，该因子告诉代理在路径的末尾减速。
			// 对代码或实现进行改进，使其更加规范、可维护和可理解，减少使用"hack"或"hacky"的技巧或临时解决方案。
			const float slowDownRadius = ag->params.radius*2;	// TODO: make less hacky.
			const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;
				
			ag->desiredSpeed = ag->params.maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}

		// Separation
		// 在移动过程中，保持一定的距离，避免相互碰撞或过于靠近
		// 群集代理拥有该标识符，表示在移动过程中需要处理群集代理之间的距离
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			// 初始的碰撞检测查询范围、分离力量、分离力量因子及权重
			const float separationDist = ag->params.collisionQueryRange; 
			const float invSeparationDist = 1.0f / separationDist; 
			const float separationWeight = ag->params.separationWeight;
			
			float w = 0;
			float disp[3] = {0,0,0};
			
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				const float distSqr = dtVlenSqr(diff);
				// 如果距离足够大，则不需要进一步处理
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > dtSqr(separationDist))
					continue;
				// 使用分离权重乘以分离距离的比例来衰减权重，以实现距离越近的代理受到更大的分离力量
				const float dist = dtMathSqrtf(distSqr);
				const float weight = separationWeight * (1.0f - dtSqr(dist*invSeparationDist));
				
				dtVmad(disp, disp, diff, weight/dist);
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				// 调整期望的速度
				dtVmad(dvel, dvel, disp, 1.0f/w);
				// Clamp desired velocity to desired speed.
				// 将期望的速度限制在期望的速度之内
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = dtSqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dtVscale(dvel, dvel, desiredSqr/speedSqr);
			}
		}
		
		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
	}
	
	// Velocity planning.
	// 速度规划
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		// 检测代理是否处于移动状态下
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// 代理是否需要考虑避障
		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			// 重置避障查询
			m_obstacleQuery->reset();
			
			// Add neighbours as obstacles.
			// 将相邻的群集代理作为障碍物添加到避障查询对象
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			// 将邻近的路径段作为障碍物添加到避障查询对象
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				// 检测是否属于逆时针三角形或左转三角形，两个名字描述的是同一种说法，仅是叫法不同而已
				if (dtTriArea2D(ag->npos, s, s+3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s+3);
			}

			dtObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i) 
				vod = debug->vod;
			
			// Sample new safe velocity.
			// 通过为adaptive设置不同的值，以使用不同的安全速度采样方式
			bool adaptive = true;
			int ns = 0;

			const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];
				
			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
															 ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
														 ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			m_velocitySampleCount += ns;
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			// 如果没有使用速度规划，新的移动速度将直接等于期望速度
			dtVcopy(ag->nvel, ag->dvel);
		}
	}

	// Integrate.
	// 更新群集代理的当前位置信息
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}
	
	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;
	
	// 处理碰撞检测
	// 最外层遍历的次数，可能仅是一个经验值，用于有效避免群集代理之间的碰撞。
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			const int idx0 = getAgentIndex(ag);
			
			// 判断当前群集代理是否处于可移动状态下
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ag->disp, 0,0,0);
			
			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				// 计算 ag 到 nei 的方向和距离
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				// 如果距离小于两个代理的半径之和，则可能发生重叠或接触，即发生了碰撞。
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->params.radius + nei->params.radius))
					continue;
				dist = dtMathSqrtf(dist);
				// 计算代理之间的间距，间距在这里被认为是碰撞代价
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					// 当多个代理彼此重叠或位于相同位置时，尝试选择分离方向以避免碰撞。
					if (idx0 > idx1)
						dtVset(diff, -ag->dvel[2],0,ag->dvel[0]);
					else
						dtVset(diff, ag->dvel[2],0,-ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					// 计算碰撞代价的公式
					pen = (1.0f/dist) * (pen*0.5f) * COLLISION_RESOLVE_FACTOR;
				}
				
				// 更新群集代理在当前帧的坐标偏移量
				dtVmad(ag->disp, ag->disp, diff, pen);			
				
				w += 1.0f;
			}
			
			// 缩放当前帧群集代理的坐标偏移量，以减少碰撞
			if (w > 0.0001f)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->disp, ag->disp, iw);
			}
		}
		
		// 更新群集代理的当前坐标位置
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;
			
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Move along navmesh.
		// 沿着导航网格移动，从起始位置到目标位置
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
		// Get valid constrained position back.
		// 获得有效且受限制的位置，意思是，这个点可能不是最终的目标点，但趋于无限接近，且保证可以位置在正确的
		// 导航网格上。
		dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		// 如果不使用路径，则将路径截断为只有一个多边形。
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
			ag->partial = false;
		}

	}
	
	// Update agents using off-mesh connection.
	// 使用离散连接更新代理
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const int idx = (int)(ag - m_agents);
		dtCrowdAgentAnimation* anim = &m_agentAnims[idx];
		if (!anim->active)
			continue;
		

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			anim->active = false;
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}
		
		// Update position
		const float ta = anim->tmax*0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}
			
		// Update velocity.
		dtVset(ag->vel, 0,0,0);
		dtVset(ag->dvel, 0,0,0);
	}
	
}
