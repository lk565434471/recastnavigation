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

// Բ���ε��ϰ���
struct dtObstacleCircle
{
	// �ϰ����λ��
	float p[3];				///< Position of the obstacle
	// �ϰ�����ƶ��ٶ�
	float vel[3];			///< Velocity of the obstacle
	// �ϰ���������ƶ��ٶ�
	float dvel[3];			///< Velocity of the obstacle
	// �ϰ���İ뾶
	float rad;				///< Radius of the obstacle
	// �ڽ��в��������У�����ѡ��������;
	// dp �������ϰ���ľ���
	float dp[3], np[3];		///< Use for side selection during sampling.
};

// �����ϰ�����߶�
struct dtObstacleSegment
{
	// �ϰ����߶ε������˵�
	float p[3], q[3];		///< End points of the obstacle segment
	// ��ʶ�Ƿ��Ѿ����������ϰ���
	bool touch;
};

// ���ڱ��ϵ��Ե�����
class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();
	
	// ��ʼ���������ڴ�
	bool init(const int maxSamples);
	// ���ú���
	void reset();

	// ��Ӳ�������
	// vel �������ٶ�����
	// ssize �ڲ��������е����ٶȵ�����
	// pen vpen + vcpen + spen + tpen �Ĵ����ܺ�
	// vpen��vertex penalty�� ����������ʾ����Ŀ����ڵ�ǰ�ڵ�ľ�����ۡ�ʹ��ŷ�������������پ���ȷ�ʽ����
	// vcpen��vertex cost penalty�� ����ɱ��������ʾ����Ŀ����뵱ǰ�ڵ�ĳɱ����ۡ����ǻ��ڶ���ĸ��Ӵ��ۣ����ܰ����谭������¶ȵ���Ϣ��
	// spen��straight penalty�� ֱ��·���������ʾ��ǰ�ڵ㵽���ڽڵ�֮���ֱ��·�����ۡ����ǻ���ֱ��·���ĸ��Ӵ��ۣ��������ڶ�����·�����гͷ���
	// tpen��turn penalty�� ת��������ʾ�ڵ�ǰ�ڵ㴦����ת��Ĵ��ۡ�ת��ͨ��������·���ĳ��Ȼ�ʱ�䣬��˿���ͨ��������۶�ת����гͷ���
	void addSample(const float* vel, const float ssize, const float pen,
				   const float vpen, const float vcpen, const float spen, const float tpen);
	
	// ����������һ������
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

	// ��������
	int m_nsamples;
	// �����������
	int m_maxSamples;
	// �������ٶ�����
	float* m_vel;
	// �ڲ��������е����ٶȵ�����
	float* m_ssize;
	// �ܵĴ��ۣ���������� + ����ɱ������� + ֱ��·�������� + ת������
	float* m_pen;
	// ���������
	float* m_vpen;
	// ����ɱ�������
	float* m_vcpen;
	// ֱ��·��������
	float* m_spen;
	// ת�������
	float* m_tpen;
};

dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);

// ��·��ƽ�������У���������Ӧ�ָ���������
// ���ٵķָ���ʹ·�����ӽ���ԭʼ·���������ܲ���ƽ��
// �෴������ķָ���ʹ·������ƽ���������ܶ�ԭʼ��·��ƫ�ƽϴ�
static const int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.

// ��·��ƽ����������������Ӧ����ֵ�������������������DT_MAX_PATTERN_DIVS����
static const int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

// �������ò���
struct dtObstacleAvoidanceParams
{
	// �ٶ�ƫ������
	float velBias;
	// �����ٶȵ�Ȩ��
	float weightDesVel;
	// ��ǰ�ٶȵ�Ȩ��
	float weightCurVel;
	float weightSide;
	// Toi��time of impact����ײ������ʱ��
	// ��ײʱ���Ȩ��
	float weightToi;
	// horional time ��ƽ�棨ˮƽ���������ƶ���ʱ���ɱ�
	float horizTime;
	// ���ӵ��������� recastnavigation �б�ʾ�С��У�������m * n
	unsigned char gridSize;	///< grid
	// ����Ӧ�ָ���ڿ�����������������ָ��ϸ���̶�
	unsigned char adaptiveDivs;	///< adaptive
	// ����Ӧ�������ڿ���������������չ�Ļ��������ͷ�Χ
	unsigned char adaptiveRings;	///< adaptive
	// ����Ӧ��ȣ����ڿ������������еĵݹ����
	unsigned char adaptiveDepth;	///< adaptive
};

// ���ϲ�ѯ
class dtObstacleAvoidanceQuery
{
public:
	dtObstacleAvoidanceQuery();
	~dtObstacleAvoidanceQuery();
	
	// ���ϲ�ѯ��ʼ������
	// ����Բ���塢�߶�������ڴ�
	bool init(const int maxCircles, const int maxSegments);
	
	// ����Բ���塢�߶ε�ƫ�����������Ѵ洢��Ԫ��������
	void reset();

	// ���Բ�����ϰ���
	// pos ���ĵ�
	// rad ��ײ���뾶
	// vel �ƶ��ٶ�����
	// dvel �������ƶ��ٶ�����
	void addCircle(const float* pos, const float rad,
				   const float* vel, const float* dvel);
	
	// ����߶��ϰ���
	// p �߶ε����
	// q �߶ε��յ�
	void addSegment(const float* p, const float* q);

	// ���ڸ��ӵĲ�����
	// pos ��ǰλ��
	// rad ��ײ���뾶
	// vmax �������ƶ��ٶ�
	// vel �ƶ��ٶ�����
	// dvel �������ƶ��ٶ�����
	// nvel �����ϰ�����õ�����������ٶ�
	// params �������ò���
	// debug ���ϵ������ݶ���
	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
						   const float* vel, const float* dvel, float* nvel,
						   const dtObstacleAvoidanceParams* params,
						   dtObstacleAvoidanceDebugData* debug = 0);

	// ����Ӧ�ٶȲ�����
	// pos ��ǰλ��
	// rad ��ײ���뾶
	// vmax �������ƶ��ٶ�
	// vel �ƶ��ٶ�����
	// dvel �������ƶ��ٶ�����
	// nvel �����ϰ�����õ�����������ٶ�
	// params �������ò���
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

	// Ԥ����
	void prepare(const float* pos, const float* dvel);

	/*
	*
	* ��������ٶ���������ײ����
	*
	* vcand �������ٶ�����
	* cs �ڲ��������е����ٶȵ�����
	* pos ����ĵ�ǰλ��
	* rad ����İ뾶
	* vel ������ƶ��ٶ�����
	* dvel �������ٶ�����
	* minPenalty ��С����
	*
	*/
	float processSample(const float* vcand, const float cs,
						const float* pos, const float rad,
						const float* vel, const float* dvel,
						const float minPenalty,
						dtObstacleAvoidanceDebugData* debug);

	// ���ڱ��ϲ�ѯ�����ò���
	dtObstacleAvoidanceParams m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	// Բ����������������
	int m_maxCircles;
	// ��������Բ���������
	dtObstacleCircle* m_circles;
	// ��ǰ�洢��Բ��������
	int m_ncircles;

	// �߶ε������������
	int m_maxSegments;
	// ���������߶ε�����
	dtObstacleSegment* m_segments;
	// ��ǰ�洢���߶�����
	int m_nsegments;
};

dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);


#endif // DETOUROBSTACLEAVOIDANCE_H
