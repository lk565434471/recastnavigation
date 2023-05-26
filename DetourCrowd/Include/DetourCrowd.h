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

// ���������ѯ
#include "DetourNavMeshQuery.h"
// ����
#include "DetourObstacleAvoidance.h"
// ���ر߽�
#include "DetourLocalBoundary.h"
// ·���滮
#include "DetourPathCorridor.h"
// �ڽ�����
#include "DetourProximityGrid.h"
// ·����ѯ
#include "DetourPathQueue.h"

/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
/// 
/// Ⱥ�������ڽ���ת�����ʱ�����ǵ�����ھ�����
static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

/// The maximum number of corners a crowd agent will look ahead in the path.
/// This value is used for sizing the crowd agent corner buffers.
/// Due to the behavior of the crowd manager, the actual number of useful
/// corners will be one less than this number.
/// @ingroup crowd
/// 
/// Ⱥ��������·���滮�͵��������У�Ӱ��ת�������ͶԻ����ĸ�֪��Χ��
/// ���ֵ����ȷ��Ⱥ������ת�ǻ������Ĵ�С��
/// ����Ⱥ������������Ϊ��ʵ������Ч��ת������Ϊ���������С1��
static const int DT_CROWDAGENT_MAX_CORNERS = 4;

/// The maximum number of crowd avoidance configurations supported by the
/// crowd manager.
/// @ingroup crowd
/// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
///		 dtCrowdAgentParams::obstacleAvoidanceType
/// 
/// Ⱥ��������֧�ֵ���������������
static const int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

/// The maximum number of query filter types supported by the crowd manager.
/// @ingroup crowd
/// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
///		dtCrowdAgentParams::queryFilterType
/// 
/// Ⱥ��������֧�ֵ�����ѯ������������������ѯ���������ڶ���Ⱥ��������·���滮�͵�
/// ����ѯ������Ҫ���ǵ������������������ͨ���ض����͵������������ض����͵��ϰ�����
/// ײ�ȡ�
static const int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
/// 
/// �ṩ����Ⱥ������������Ĵ�����ھ����ݡ�
struct dtCrowdNeighbour
{
	// ��Ⱥ����������Ⱥ��������������±꣩
	int idx;		///< The index of the neighbor in the crowd.
	// ��ǰȺ��������ھ�֮��ľ���
	float dist;		///< The distance between the current agent and the neighbor.
};

/// The type of navigation mesh polygon the agent is currently traversing.
/// @ingroup crowd
/// 
/// ����ǰ���ڱ����ĵ����������ε�����
enum CrowdAgentState
{
	// Ⱥ����������Ч״̬��
	DT_CROWDAGENT_STATE_INVALID,		///< The agent is not in a valid state.
	// Ⱥ����������·���滮�򵼺�������
	DT_CROWDAGENT_STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
	// Ⱥ���������ڱ�����ɢ���ӣ���ɢ������ָ�����������ĵ�֮���ֱ��·����ͨ������������ͬ�߶Ȼ�ͬ���͵��ϰ��
	DT_CROWDAGENT_STATE_OFFMESH 		///< The agent is traversing an off-mesh connection.
};

/// Configuration parameters for a crowd agent.
/// @ingroup crowd
/// 
/// Ϊ��Ⱥ�����ṩ���ò���
struct dtCrowdAgentParams
{
	// Ⱥ������İ뾶
	float radius;						///< Agent radius. [Limit: >= 0]
	// Ⱥ������ĸ߶�
	float height;						///< Agent height. [Limit: > 0]
	// Ⱥ����������������ٶ�
	float maxAcceleration;				///< Maximum allowed acceleration. [Limit: >= 0]
	// Ⱥ���������������ٶ�
	float maxSpeed;						///< Maximum allowed speed. [Limit: >= 0]

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	// �����ڽ��е����������Ϊ����ʱ��������Щ������㹻������ײԪ�ء��������ͨ���������ƴ������ϰ���ֱ�ӵľ���
	// ֻ��С������������ײԪ�أ��ŻὫ������������Ϊ�����С�
	float collisionQueryRange;

	// ·���ɼ����Ż���Χ�����ڼ��ٵ��������ڼ���·��ʱ�Ŀ�����
	float pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]

	/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	// ���������������ô�������ײʱ�Ĳ��ԡ������ֵԽ�󣬴����Խ�ܽ���Ľ���·�����������ٶȵĵ�����
	float separationWeight;

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	// ���������Ϊ�ı�־λ����Щ��־λ���ڵ����������Ϊ��ʽ��
	unsigned char updateFlags;

	/// The index of the avoidance configuration to use for the agent. 
	/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	/// ������ʹ�õı������õ����������ڲ�ͬ���ò��Ե�ѡ��
	unsigned char obstacleAvoidanceType;	

	/// The index of the query filter used by this agent.
	// ������ʹ�õ�·���������������õ����������ڿ��ƴ�����·�������ͱ�����Ϊ�������ǵĵ���������������á�
	// ��������һЩ��������������ڹ��˵����ʺϵ���������
	unsigned char queryFilterType;

	/// User defined data attached to the agent.
	// �û����嵽����ĸ������ݣ�����ĳЩ��չ��Ϊ��
	void* userData;
};

// ������ƶ�����״̬
enum MoveRequestState
{
	// Ⱥ������ĳ�ʼ�ƶ�״̬
	DT_CROWDAGENT_TARGET_NONE = 0,
	// Ⱥ��������ƶ�����ʧ��
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	// Ⱥ��������ƶ��������ڴ�����
	DT_CROWDAGENT_TARGET_REQUESTING,
	// ·���ϳ���Ǳ���޷����������£����������滮��
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	// ����Ⱥ�������Ŀ���ٶ��������ƶ�
	DT_CROWDAGENT_TARGET_VELOCITY
};

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
/// 
/// ��Ⱥ������  dtCrowd ��������һ��������
struct dtCrowdAgent
{
	/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	// ��ʾ����Ļ״̬��true ��ʾ�����ڻ״̬��false ��ʾ�����ڴ�������δʹ�õĲ���С�
	bool active;

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	// �������ڴ�Խ���������ε����ͣ���� CrowdAgentState ö��ֵ������
	unsigned char state;

	/// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	// ��� targetState == DT_CROWDAGENT_TARGET_VALID��partial Ϊ true��
	// true ��ʾ�������һ����Ч��·������ζ�Ŵ����Ѿ��ɹ��滮��һ�������ߵ�·������·�����ᵼ�������λ�á�
	// Ȼ������ʹ·����Ч������·��Ҳ���ܲ�ֱ��ָ�������λ�á�������Ϊ�������޷�ֱ�ӵ�������λ�á�
	// ͨ������ֶΣ����ǿ��Ե�֪�����ھ�����Ч·��������£��Ƿ����ֱ�ӵ�������λ�á�
	bool partial;

	/// The path corridor the agent is using.
	// ���ƴ�����ƶ�·����������·���滮�Ļ���֮�ϣ�
	dtPathCorridor corridor;

	/// The local boundary data for the agent.
	// ����ľֲ��߽�����
	dtLocalBoundary boundary;
	
	/// Time since the agent's path corridor was optimized.
	// ����һ���ƶ�·���Ż�������������ʱ��
	float topologyOptTime;
	
	/// The known neighbors of the agent.
	// ������֪���ھӴ���
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

	/// The number of neighbors.
	// ������Χ���ھ��������� neis ������
	int nneis;
	
	/// The desired speed.
	// ����������ƶ��ٶ�
	float desiredSpeed;

	// ����ĵ�ǰλ��
	float npos[3];		///< The current agent position. [(x, y, z)]
	// �ڵ�����ײ����������ۻ������λ�ƣ�ͨ���ۼ�λ�ƣ�ȷ�������ܹ��ܿ��ϰ������������
	float disp[3];		///< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	// ����������ٶȡ������ڵ�ǰ·�����м��㣬����ÿ֡���¼��㡣
	float dvel[3];		///< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	// �����ϰ�����õ�����������ٶȡ����Ǹ��ݵ�ǰ·�����ϰ�����Ϣ�����أ�ÿ֡���¼���õ��ġ�
	float nvel[3];		///< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	// �����ʵ���ٶȡ�����ʾ����ǰ���ٶ�״̬�������ڸ���ʱ�ܵ������ٶȵ����ơ����������ٶȺ������ٶȼ���ó���
	float vel[3];		///< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

	/// The agent's configuration parameters.
	// ��������ò���
	dtCrowdAgentParams params;

	/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	// �洢�˴���ľֲ�·���ս���Ϣ�����Ǵ����ƶ����辭����һϵ�е�
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS*3];

	/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	// �洢�˴���ֲ�·���սǵ�ı�־��Ϣ����� dtStraightPathFlags
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	/// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	// �洢�˴�����·���սǵ����Ķ���εĸ���id�����id�ڵ��������о���Ψһ�ԣ�
	// ͨ�����id��������Կ���ȷ���Լ��������ĸ�����Σ�������Ӧ����Ϊ�͵������ߡ�
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	/// The number of corners.
	// cornerVerts ��ʵ�ʴ洢��·���սǵ�����
	int ncorners;
	
	// ������ƶ�״̬ ��� MoveRequestState
	unsigned char targetState;			///< State of the movement request.
	// �����ƶ������Ŀ�����θ���id
	dtPolyRef targetRef;				///< Target polyref of the movement request.
	// �����ƶ������Ŀ��λ�û�Ŀ���ٶȣ��� targetState == DT_CROWDAGENT_TARGET_VELOCITY ʱ����ʾ�ƶ��ٶ�
	float targetPos[3];					///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	// ·����ѯ���ĸ���id
	dtPathQueueRef targetPathqRef;		///< Path finder ref.
	// ��ʾ��ǰ·���Ƿ��������¹滮
	bool targetReplan;					///< Flag indicating that the current path is being replanned.
	// ����Ŀ��·���¹滮֮�󾭹���ʱ��
	float targetReplanTime;				/// <Time since the agent's target was replanned.
};

struct dtCrowdAgentAnimation
{
	// ��ʾȺ�������Ƿ��ڼ���״̬
	bool active;
	// �ֱ��ʾ��ʼλ�á����λ�ú��յ�λ��
	float initPos[3], startPos[3], endPos[3];
	// ����θ���id
	dtPolyRef polyRef;
	float t, tmax;
};

/// Crowd agent update flags.
/// @ingroup crowd
/// @see dtCrowdAgentParams::updateFlags
/// 
/// Ⱥ������ĸ��±�־λ
enum UpdateFlags
{
	// �������ķ�ʽ������Ϊת��
	DT_CROWD_ANTICIPATE_TURNS = 1,
	// �������ƶ������У��Ƿ���Ҫ���Ǳ���
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	// Ⱥ����Ϊ�еķ������������ƶ��Ĺ����б���һ���ľ��룬�����໥��ײ����ڿ�����
	DT_CROWD_SEPARATION = 4,
	// ʹ�� dtPathCorridor::optimizePathVisibility() �������Ż�����·��
	// �����ñ�־λ�������нݾ����
	DT_CROWD_OPTIMIZE_VIS = 8,			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	// ʹ�� dtPathCorridor::optimizePathTopology() �������Ż�����·��
	DT_CROWD_OPTIMIZE_TOPO = 16 		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
};

// Ⱥ������ĵ�����Ϣ
struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	// ���ϵ�����Ϣ
	dtObstacleAvoidanceDebugData* vod;
};

/// Provides local steering behaviors for a group of agents. 
/// @ingroup crowd
/// 
/// Ϊһ��Ⱥ�������ṩ������Ϊ����
class dtCrowd
{
	// Agent�������������������
	int m_maxAgents;
	// Agent������������
	dtCrowdAgent* m_agents;
	// ���ڼ���״̬�µ� Agent������
	dtCrowdAgent** m_activeAgents;
	// Agent�������Ķ���
	dtCrowdAgentAnimation* m_agentAnims;
	// ·����������У������Ⱥ�˳���������Ա����ͻ�ͻ���
	dtPathQueue m_pathq;

	// ���ϲ���
	dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
	// ���ϲ�ѯ
	dtObstacleAvoidanceQuery* m_obstacleQuery;
	
	// �������֮����ڽ���ϵ�����ݽṹ
	dtProximityGrid* m_grid;
	
	// ��ʱ�����������ںϲ�·������
	dtPolyRef* m_pathResult;
	int m_maxPathResult;
	
	float m_agentPlacementHalfExtents[3];

	// ·����ѯ�͵������˵�ɸѡ��
	dtQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];

	float m_maxAgentRadius;

	int m_velocitySampleCount;

	// ��������·�������� dtQueryFilter ɸѡ����Ŀ���������
	dtNavMeshQuery* m_navquery;

	// �Ż�·�����˽ṹ
	// agents ��������
	// nagents �����Ԫ������
	// dt ģ����µ�ʱ�䣬����Ϊ��λ
	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);

	// �����ƶ�����
	// dt ģ����µ�ʱ�䣬����Ϊ��λ��
	void updateMoveRequest(const float dt);

	// �������Ƿ������Ч���ƶ�·��
	// agents ������Ⱥ������ָ������
	// nagents ������Ⱥ���������������� agents �����Ԫ������
	// dt ģ����µ�ʱ�䣬����Ϊ��λ��
	void checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt);

	inline int getAgentIndex(const dtCrowdAgent* agent) const  { return (int)(agent - m_agents); }

	// �������¹滮�ƶ�Ŀ��·��
	// idx Ⱥ�������������е��±�
	// ref Ŀ�����εĸ���id
	// pos �ƶ������Ŀ��λ��
	bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos);

	// ���Ⱥ�����������ó�Ա����
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
	/// ��ʼ��Ⱥ������
	/// maxAgents Ⱥ����������Թ����Ⱥ������������������
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);
	
	/// Sets the shared avoidance configuration for the specified index.
	///  @param[in]		idx		The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	///  @param[in]		params	The new configuration.
	/// 
	/// ����ָ�������Ĺ���������ò���
	void setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params);

	/// Gets the shared avoidance configuration for the specified index.
	///  @param[in]		idx		The index of the configuration to retreive. 
	///							[Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	/// @return The requested configuration.
	/// 
	/// ���ָ�������Ĺ���������ò���
	const dtObstacleAvoidanceParams* getObstacleAvoidanceParams(const int idx) const;
	
	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	/// 
	/// ͨ��ָ����������Ⱥ��������л��Ⱥ���������
	const dtCrowdAgent* getAgent(const int idx);

	/// Gets the specified agent from the pool.
	///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return The requested agent.
	/// 
	/// �� getAgent ����һ�£�Ψһ�������Ƿ��ؿɱ༭��Ⱥ���������
	dtCrowdAgent* getEditableAgent(const int idx);

	/// The maximum number of agents that can be managed by the object.
	/// @return The maximum number of agents.
	/// 
	/// ����Ⱥ����������������Ⱥ��������������
	int getAgentCount() const;
	
	/// Adds a new agent to the crowd.
	///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	///  @param[in]		params	The configutation of the agent.
	/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	/// 
	/// ���һ��Ⱥ������Ⱥ��������
	int addAgent(const float* pos, const dtCrowdAgentParams* params);

	/// Updates the specified agent's configuration.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		params	The new agent configuration.
	/// 
	/// ����ָ��Ⱥ����������ò���
	/// idx Ⱥ�������������е��±�
	/// params �µ�Ⱥ����������
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	/// Removes the agent from the crowd.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// 
	/// ��Ⱥ�������������ɾ��Ⱥ���������
	/// idx Ⱥ�������������е��±�
	/// ���������������Ƿ�Ⱥ��������ڴ棬ͨ��Ⱥ������� active ��ʶΪ������
	void removeAgent(const int idx);
	
	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		ref		The position's polygon reference.
	///  @param[in]		pos		The position within the polygon. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	/// 
	/// �ύָ��Ⱥ��������µ��ƶ�����
	/// idx Ⱥ�������������е��±�
	/// ref �����ƶ���Ŀ��λ�������Ķ���η���id
	/// pos �ƶ������ڶ�����е�λ��
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	/// Submits a new move request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	///  @param[in]		vel		The movement velocity. [(x, y, z)]
	/// @return True if the request was successfully submitted.
	/// 
	/// ָ����Ⱥ�������ύһ���µ��ƶ�����
	/// idx Ⱥ�������������е��±�
	/// vel �ƶ��ٶȣ������������������
	bool requestMoveVelocity(const int idx, const float* vel);

	/// Resets any request for the specified agent.
	///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	/// @return True if the request was successfully reseted.
	/// 
	/// ����ָ����Ⱥ��������ƶ�����
	/// idx Ⱥ�������������е��±�
	bool resetMoveTarget(const int idx);

	/// Gets the active agents int the agent pool.
	///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	///  @param[in]		maxAgents	The size of the crowd agent array.
	/// @return The number of agents returned in @p agents.
	/// 
	/// ���Ⱥ��������д��ڼ���״̬�µ�Ⱥ�������б�
	/// 
	/// ���ش��ڼ���״̬�µ�Ⱥ�����������
	/// agents һ��Ⱥ������ָ�����飬���ڱ��ִ��ڼ���״̬�µ�Ⱥ������
	/// maxAgents Ⱥ����������Ĵ�С
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);

	/// Updates the steering and positions of all agents.
	///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	///  @param[out]	debug	A debug object to load with debug information. [Opt]
	/// 
	/// ��������Ⱥ�������ת���λ��
	/// dt ����ģ���ʱ�䣬����Ϊ��λ��
	/// debug һ�����Զ������ڼ��ص�����Ϣ
	void update(const float dt, dtCrowdAgentDebugInfo* debug);
	
	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
	/// 
	/// ���Ⱥ��������ʹ�õ�·����ѯ����������
	inline const dtQueryFilter* getFilter(const int i) const { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }
	
	/// Gets the filter used by the crowd.
	/// @return The filter used by the crowd.
	/// 
	/// �� getFilter ����һ�£��������ڷ���һ���ɱ༭��·����ѯ����������
	inline dtQueryFilter* getEditableFilter(const int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	/// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations. 
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	/// 
	/// ��ȡȺ������������ڲ�ѯ������ʹ�õ������뾶
	const float* getQueryHalfExtents() const { return m_agentPlacementHalfExtents; }

	/// Same as getQueryHalfExtents. Left to maintain backwards compatibility.
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	/// 
	/// �� getQueryHalfExtents �����������ƣ������ú�����Ϊ�������ݡ�
	const float* getQueryExtents() const { return m_agentPlacementHalfExtents; }
	
	/// Gets the velocity sample count.
	/// @return The velocity sample count.
	/// 
	/// ��ȡ���ٶȲ�����
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }
	
	/// Gets the crowd's proximity grid.
	/// @return The crowd's proximity grid.
	/// 
	/// ���Ⱥ�����������ڽ�����������
	const dtProximityGrid* getGrid() const { return m_grid; }

	/// Gets the crowd's path request queue.
	/// @return The crowd's path request queue.
	/// 
	/// ���Ⱥ����������·����ѯ����
	const dtPathQueue* getPathQueue() const { return &m_pathq; }

	/// Gets the query object used by the crowd.
	/// 
	/// ���Ⱥ���������ĵ��������ѯ����
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

