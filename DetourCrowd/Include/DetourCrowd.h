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

#ifndef DETOURCROWD_H
#define DETOURCROWD_H

// 导航网格查询
#include "DetourNavMeshQuery.h"
// 避障
#include "DetourObstacleAvoidance.h"
// 本地边界
#include "DetourLocalBoundary.h"
// 路径规划
#include "DetourPathCorridor.h"
// 邻近网格
#include "DetourProximityGrid.h"
// 路径查询
#include "DetourPathQueue.h"

/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
/// 
/// 群集代理在进行转向决策时，考虑的最大邻居数量
static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

/// The maximum number of corners a crowd agent will look ahead in the path.
/// This value is used for sizing the crowd agent corner buffers.
/// Due to the behavior of the crowd manager, the actual number of useful
/// corners will be one less than this number.
/// @ingroup crowd
/// 
/// 群集代理在路径规划和导航过程中，影响转弯能力和对环境的感知范围。
/// 这个值用于确定群集代理转角缓冲区的大小。
/// 由于群集管理器的行为，实际上有效的转角数量为比这个数字小1。
static const int DT_CROWDAGENT_MAX_CORNERS = 4;

/// The maximum number of crowd avoidance configurations supported by the
/// crowd manager.
/// @ingroup crowd
/// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
///		 dtCrowdAgentParams::obstacleAvoidanceType
/// 
/// 群集管理器支持的最大避让配置数量
static const int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

/// The maximum number of query filter types supported by the crowd manager.
/// @ingroup crowd
/// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
///		dtCrowdAgentParams::queryFilterType
/// 
/// 群集管理器支持的最大查询过滤器类型数量。查询过滤器用于定义群集代理在路径规划和导
/// 航查询过程中要考虑的限制条件，例如避免通过特定类型的区域或避免与特定类型的障碍物碰
/// 撞等。
static const int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
/// 
/// 提供了由群集管理器管理的代理的邻居数据。
struct dtCrowdNeighbour
{
	// 在群集管理器中群集代理的索引（下标）
	int idx;		///< The index of the neighbor in the crowd.
	// 当前群集代理和邻居之间的距离
	float dist;		///< The distance between the current agent and the neighbor.
};

/// The type of navigation mesh polygon the agent is currently traversing.
/// @ingroup crowd
/// 
/// 代理当前正在遍历的导航网格多边形的类型
enum CrowdAgentState
{
	// 群集代理处于无效状态下
	DT_CROWDAGENT_STATE_INVALID,		///< The agent is not in a valid state.
	// 群集代理正在路径规划或导航过程中
	DT_CROWDAGENT_STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
	// 群集代理正在遍历离散连接（离散连接是指两个不相连的点之间的直接路径，通常用于描述不同高度或不同类型的障碍物）
	DT_CROWDAGENT_STATE_OFFMESH 		///< The agent is traversing an off-mesh connection.
};

/// Configuration parameters for a crowd agent.
/// @ingroup crowd
/// 
/// 为集群代理提供配置参数
struct dtCrowdAgentParams
{
	// 群集代理的半径
	float radius;						///< Agent radius. [Limit: >= 0]
	// 群集代理的高度
	float height;						///< Agent height. [Limit: > 0]
	// 群集代理允许的最大加速度
	float maxAcceleration;				///< Maximum allowed acceleration. [Limit: >= 0]
	// 群集代理允许的最大速度
	float maxSpeed;						///< Maximum allowed speed. [Limit: >= 0]

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	// 定义在进行导航代理的行为决策时，考虑哪些与代理足够近的碰撞元素。这个参数通常用来控制代理与障碍物直接的距离
	// 只有小于这个距离的碰撞元素，才会将其纳入代理的行为决策中。
	float collisionQueryRange;

	// 路径可见性优化范围。用于减少导航代理在计算路径时的开销。
	float pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]

	/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	// 代理管理器避免与该代理发生碰撞时的策略。这个数值越大，代理就越能较早的进行路径修正或方向、速度的调整。
	float separationWeight;

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	// 代理控制行为的标志位，这些标志位用于调整代理的行为方式。
	unsigned char updateFlags;

	/// The index of the avoidance configuration to use for the agent. 
	/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	/// 代理所使用的避让配置的索引，用于不同避让策略的选择。
	unsigned char obstacleAvoidanceType;	

	/// The index of the query filter used by this agent.
	// 代理所使用的路径搜索过滤器配置的索引，用于控制代理在路径搜索和避让行为中所考虑的导航网格区域的设置。
	// 它定义了一些规则和条件，用于过滤掉不适合的行走区域。
	unsigned char queryFilterType;

	/// User defined data attached to the agent.
	// 用户定义到代理的附加数据，用于某些扩展行为。
	void* userData;
};

// 代理的移动请求状态
enum MoveRequestState
{
	// 群集代理的初始移动状态
	DT_CROWDAGENT_TARGET_NONE = 0,
	// 群集代理的移动请求失败
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	// 群集代理的移动请求正在处理中
	DT_CROWDAGENT_TARGET_REQUESTING,
	// 路径较长或潜在无法到达的情况下，进行完整规划。
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	// 基于群集代理的目标速度来控制移动
	DT_CROWDAGENT_TARGET_VELOCITY
};

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
/// 
/// 集群代理，由  dtCrowd 对象管理的一组代理对象
struct dtCrowdAgent
{
	/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	// 表示代理的活动状态，true 表示代理处于活动状态；false 表示代理处于代理对象池未使用的插槽中。
	bool active;

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	// 代理正在穿越的网格多边形的类型，详见 CrowdAgentState 枚举值描述。
	unsigned char state;

	/// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	// 如果 targetState == DT_CROWDAGENT_TARGET_VALID，partial 为 true。
	// true 表示代理具有一个有效的路径，意味着代理已经成功规划了一条可行走的路径，且路径不会导向请求的位置。
	// 然而，即使路径有效，导航路径也可能不直接指向请求的位置。这是因为，代理无法直接到达请求位置。
	// 通过这个字段，我们可以得知代理在具有有效路径的情况下，是否可以直接到达请求位置。
	bool partial;

	/// The path corridor the agent is using.
	// 控制代理的移动路径（建立在路径规划的基础之上）
	dtPathCorridor corridor;

	/// The local boundary data for the agent.
	// 代理的局部边界数据
	dtLocalBoundary boundary;
	
	/// Time since the agent's path corridor was optimized.
	// 自上一次移动路径优化以来所经过的时间
	float topologyOptTime;
	
	/// The known neighbors of the agent.
	// 代理已知的邻居代理
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

	/// The number of neighbors.
	// 代理周围的邻居数量，对 neis 的描述
	int nneis;
	
	/// The desired speed.
	// 代理的期望移动速度
	float desiredSpeed;

	// 代理的当前位置
	float npos[3];		///< The current agent position. [(x, y, z)]
	// 在迭代碰撞解决过程中累积代理的位移，通过累计位移，确保代理能够避开障碍物或其他代理
	float disp[3];		///< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	// 代理的期望速度。它基于当前路径进行计算，并在每帧重新计算。
	float dvel[3];		///< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	// 经过障碍物避让调整后的期望速度。它是根据当前路径和障碍物信息等因素，每帧重新计算得到的。
	float nvel[3];		///< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	// 代理的实际速度。它表示代理当前的速度状态，并且在更新时受到最大加速度的限制。（由期望速度和最大加速度计算得出）
	float vel[3];		///< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

	/// The agent's configuration parameters.
	// 代理的配置参数
	dtCrowdAgentParams params;

	/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	// 存储了代理的局部路径拐角信息，它是代理移动所需经过的一系列点
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS*3];

	/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	// 存储了代理局部路径拐角点的标志信息。详见 dtStraightPathFlags
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	/// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	// 存储了代理在路径拐角点进入的多边形的复合id（这个id在导航网格中具有唯一性）
	// 通过这个id，代理可以快速确定自己进入了哪个多边形，进行相应的行为和导航决策。
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	/// The number of corners.
	// cornerVerts 中实际存储的路径拐角点数量
	int ncorners;
	
	// 代理的移动状态 详见 MoveRequestState
	unsigned char targetState;			///< State of the movement request.
	// 代理移动请求的目标多边形复合id
	dtPolyRef targetRef;				///< Target polyref of the movement request.
	// 代理移动请求的目标位置或目标速度，当 targetState == DT_CROWDAGENT_TARGET_VELOCITY 时，表示移动速度
	float targetPos[3];					///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	// 路径查询器的复合id
	dtPathQueueRef targetPathqRef;		///< Path finder ref.
	// 表示当前路径是否正在重新规划
	bool targetReplan;					///< Flag indicating that the current path is being replanned.
	// 代理目标路径新规划之后经过的时间
	float targetReplanTime;				/// <Time since the agent's target was replanned.
};

struct dtCrowdAgentAnimation
{
	// 表示群集代理是否处于激活状态
	bool active;
	// 分别表示初始位置、起点位置和终点位置
	float initPos[3], startPos[3], endPos[3];
	// 多边形复合id
	dtPolyRef polyRef;
	float t, tmax;
};

/// Crowd agent update flags.
/// @ingroup crowd
/// @see dtCrowdAgentParams::updateFlags
/// 
/// 群集代理的更新标志位
enum UpdateFlags
{
	// 以期望的方式进行行为转向
	DT_CROWD_ANTICIPATE_TURNS = 1,
	// 代理在移动过程中，是否需要考虑避障
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	// 群集行为中的分离力量，在移动的过程中保持一定的距离，避免相互碰撞或过于靠近。
	DT_CROWD_SEPARATION = 4,
	// 使用 dtPathCorridor::optimizePathVisibility() 函数来优化代理路径
	// 包含该标志位，将进行捷径检查
	DT_CROWD_OPTIMIZE_VIS = 8,			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	// 使用 dtPathCorridor::optimizePathTopology() 函数来优化代理路径
	DT_CROWD_OPTIMIZE_TOPO = 16 		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
};

// 群集代理的调试信息
struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	// 避障调试信息
	dtObstacleAvoidanceDebugData* vod;
};

/// Provides local steering behaviors for a group of agents. 
/// @ingroup crowd
/// 
/// 为一组群集代理提供本地行为控制
class dtCrowd
{
	// Agent（代理）的最大数量上限
	int m_maxAgents;
	// Agent（代理）的数组
	dtCrowdAgent* m_agents;
	// 处于激活状态下的 Agent（代理）
	dtCrowdAgent** m_activeAgents;
	// Agent（代理）的动画
	dtCrowdAgentAnimation* m_agentAnims;
	// 路径请求处理队列，按照先后顺序处理请求，以避免冲突和混乱
	dtPathQueue m_pathq;

	// 避障参数
	dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
	// 避障查询
	dtObstacleAvoidanceQuery* m_obstacleQuery;
	
	// 管理代理之间的邻近关系的数据结构
	dtProximityGrid* m_grid;
	
	// 临时缓冲区，用于合并路径操作
	dtPolyRef* m_pathResult;
	int m_maxPathResult;
	
	float m_agentPlacementHalfExtents[3];

	// 路径查询和导航过滤的筛选器
	dtQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];

	float m_maxAgentRadius;

	int m_velocitySampleCount;

	// 搜索最优路径，基于 dtQueryFilter 筛选最近的可行走区域
	dtNavMeshQuery* m_navquery;

	// 优化路径拓扑结构
	// agents 代理数组
	// nagents 数组的元素数量
	// dt 模拟更新的时间，以秒为单位
	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);

	// 更新移动请求
	// dt 模拟更新的时间，以秒为单位。
	void updateMoveRequest(const float dt);

	// 检测代理是否具有有效的移动路径
	// agents 待检测的群集代理指针数组
	// nagents 待检测的群集代理数量，描述 agents 数组的元素数量
	// dt 模拟更新的时间，以秒为单位。
	void checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt);

	inline int getAgentIndex(const dtCrowdAgent* agent) const  { return (int)(agent - m_agents); }

	// 请求重新规划移动目标路径
	// idx 群集代理在数组中的下标
	// ref 目标多边形的复合id
	// pos 移动请求的目标位置
	bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos);

	// 清除群集代理，并重置成员变量
	void purge();
	
public:
	dtCrowd();
	~dtCrowd();
	
	/// Initializes the crowd.  
	///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
	///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	///  @param[in]		nav				The navigation mesh to use for planning.
	/// @return True if the initialization succeeded.
	/// 
	/// 初始化群集代理
	/// maxAgents 群体管理器可以管理的群集代理的最大数量上限
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);
	
	/// Sets the shared avoidance configuration for the specified index.
	///  @param[in]		idx		The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	///  @param[in]		params	The new configuration.
	/// 
	/// 设置指定索引的共享避障配置参数
	void setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params);

	/// Gets the shared avoidance configuration for the specified index.
	///  @param[in]		idx		The index of the configuration to retreive. 
	///							[Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	/// @return The requested configuration.
	/// 
	/// 获得指定索引的共享避障配置参数
	const dtObstacleAvoidanceParams* getObstacleAvoidanceParams(const int idx) const;
	
	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	/// 
	/// 通过指定索引，从群集代理池中获得群集代理对象
	const dtCrowdAgent* getAgent(const int idx);

	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	/// 
	/// 与 getAgent 功能一致，唯一的区别是返回可编辑的群集代理对象
	dtCrowdAgent* getEditableAgent(const int idx);

	/// The maximum number of agents that can be managed by the object.
	/// @return The maximum number of agents.
	/// 
	/// 返回群集代理管理器管理的群集代理的最大数量
	int getAgentCount() const;
	
	/// Adds a new agent to the crowd.
	///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	///  @param[in]		params	The configutation of the agent.
	/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	/// 
	/// 添加一个群集代理到群集管理器
	int addAgent(const float* pos, const dtCrowdAgentParams* params);

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	/// 
	/// 更新指定群集代理的配置参数
	/// idx 群集代理在数组中的下标
	/// params 新的群集代理配置
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// 
	/// 从群集代理管理器中删除群集代理对象
	/// idx 群集代理在数组中的下标
	/// 函数不会真正的是否群集代理的内存，通过群集代理的 active 标识为不可用
	void removeAgent(const int idx);
	
	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		ref		The position's polygon reference.
	///  @param[in]		pos		The position within the polygon. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	/// 
	/// 提交指定群集代理的新的移动请求
	/// idx 群集代理在数组中的下标
	/// ref 请求移动的目标位置所属的多边形符合id
	/// pos 移动请求在多边形中的位置
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		vel		The movement velocity. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	/// 
	/// 指定的群集代理提交一个新的移动请求
	/// idx 群集代理在数组中的下标
	/// vel 移动速度（包含方向和作用力）
	bool requestMoveVelocity(const int idx, const float* vel);

	/// Resets any request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return True if the request was successfully reseted.
	/// 
	/// 重置指定的群集代理的移动请求
	/// idx 群集代理在数组中的下标
	bool resetMoveTarget(const int idx);

	/// Gets the active agents int the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	///  @param[in]		maxAgents	The size of the crowd agent array.
	/// @return The number of agents returned in @p agents.
	/// 
	/// 获得群集代理池中处于激活状态下的群集代理列表
	/// 
	/// 返回处于激活状态下的群集代理的数量
	/// agents 一个群集代理指针数组，用于保持处于激活状态下的群集代理
	/// maxAgents 群集代理数组的大小
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	/// 
	/// 更新所有群集代理的转向和位置
	/// dt 更新模拟的时间，以秒为单位。
	/// debug 一个调试对象，用于加载调试信息
	void update(const float dt, dtCrowdAgentDebugInfo* debug);
	
	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
	/// 
	/// 获得群集管理器使用的路径查询过滤器对象
	inline const dtQueryFilter* getFilter(const int i) const { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }
	
	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
	/// 
	/// 与 getFilter 功能一致，区别在于返回一个可编辑的路径查询过滤器对象
	inline dtQueryFilter* getEditableFilter(const int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	/// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations. 
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	/// 
	/// 获取群集代理管理器在查询操作中使用的搜索半径
	const float* getQueryHalfExtents() const { return m_agentPlacementHalfExtents; }

	/// Same as getQueryHalfExtents. Left to maintain backwards compatibility.
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	/// 
	/// 与 getQueryHalfExtents 函数功能类似，保留该函数是为了向后兼容。
	const float* getQueryExtents() const { return m_agentPlacementHalfExtents; }
	
	/// Gets the velocity sample count.
	/// @return The velocity sample count.
	/// 
	/// 获取加速度采样数
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }
	
	/// Gets the crowd's proximity grid.
	/// @return The crowd's proximity grid.
	/// 
	/// 获得群集管理器的邻近网格管理对象
	const dtProximityGrid* getGrid() const { return m_grid; }

	/// Gets the crowd's path request queue.
	/// @return The crowd's path request queue.
	/// 
	/// 获得群集管理器的路径查询队列
	const dtPathQueue* getPathQueue() const { return &m_pathq; }

	/// Gets the query object used by the crowd.
	/// 
	/// 获得群集管理器的导航网格查询对象
	const dtNavMeshQuery* getNavMeshQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtCrowd(const dtCrowd&);
	dtCrowd& operator=(const dtCrowd&);
};

/// Allocates a crowd object using the Detour allocator.
/// @return A crowd object that is ready for initialization, or null on failure.
///  @ingroup crowd
dtCrowd* dtAllocCrowd();

/// Frees the specified crowd object using the Detour allocator.
///  @param[in]		ptr		A crowd object allocated using #dtAllocCrowd
///  @ingroup crowd
void dtFreeCrowd(dtCrowd* ptr);


#endif // DETOURCROWD_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@defgroup crowd Crowd

Members in this module implement local steering and dynamic avoidance features.

The crowd is the big beast of the navigation features. It not only handles a 
lot of the path management for you, but also local steering and dynamic 
avoidance between members of the crowd. I.e. It can keep your agents from 
running into each other.

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy 
to use path planning features. But in the end they only give you points that 
your navigation client should be moving toward. When it comes to deciding things 
like agent velocity and steering to avoid other agents, that is up to you to 
implement. Unless, of course, you decide to use #dtCrowd.

Basically, you add an agent to the crowd, providing various configuration 
settings such as maximum speed and acceleration. You also provide a local 
target to more toward. The crowd manager then provides, with every update, the 
new agent position and velocity for the frame. The movement will be 
constrained to the navigation mesh, and steering will be applied to ensure 
agents managed by the crowd do not collide with each other.

This is very powerful feature set. But it comes with limitations.

The biggest limitation is that you must give control of the agent's position 
completely over to the crowd manager. You can update things like maximum speed 
and acceleration. But in order for the crowd manager to do its thing, it can't 
allow you to constantly be giving it overrides to position and velocity. So 
you give up direct control of the agent's movement. It belongs to the crowd.

The second biggest limitation revolves around the fact that the crowd manager 
deals with local planning. So the agent's target should never be more than 
256 polygons aways from its current position. If it is, you risk 
your agent failing to reach its target. So you may still need to do long 
distance planning and provide the crowd manager with intermediate targets.

Other significant limitations:

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd 
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.

@note This is a summary list of members.  Use the index or search 
feature to find minor members.

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value 
is the index of the #dtObstacleAvoidanceParams within the crowd.

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), 
	 dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.

This value is often based on the agent radius. E.g. radius * 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at 
the cost of more difficult steering in tight spaces.

*/

