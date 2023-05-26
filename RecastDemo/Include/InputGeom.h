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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"

static const int MAX_CONVEXVOL_PTS = 12;

struct ConvexVolume
{
	float verts[MAX_CONVEXVOL_PTS*3];
	float hmin, hmax;
	int nverts;
	int area;
};

struct BuildSettings
{
	// Cell size in world units
	// ��Ԫ��ߴ磨�����絥λ���㣩
	float cellSize;
	// Cell height in world units
	// ��Ԫ��߶ȣ������絥λ���㣩
	float cellHeight;
	// Agent height in world units
	// ����߶ȣ������絥λ���㣩
	float agentHeight;
	// Agent radius in world units
	// ����뾶�������絥λ���㣩
	float agentRadius;
	// Agent max climb in world units
	// ����������������߶ȣ������絥λ���㣩
	float agentMaxClimb;
	// Agent max slope in degrees
	// // �����������������¶�
	float agentMaxSlope;
	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	float regionMinSize;
	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	// �ڽ��߶�ͼ����Ϊһϵ������ʱ�������ٸ�����ϲ�Ϊһ�����������
	// ���Լ����������ɵĵ��������е����������Ӷ���������ٶȺ͵���Ч��
	float regionMergeSize;
	// Edge max length in world units
	// �ߵ���󳤶ȣ������絥λ���㣩
	float edgeMaxLen;
	// Edge max error in voxels
	// Ѱ·�������ɵı�Ե������ֵ����λΪ�����أ���������ά���ϵ���Ϣ����ɫ��͸���ȡ��ܶȵȣ�
	float edgeMaxError;
	// ����ε���󶥵���
	float vertsPerPoly;
	// Detail sample distance in voxels
	float detailSampleDist;
	// Detail sample max error in voxel heights.
	// ϸ�ڲ��õĸ߶�����������ֵ
	float detailSampleMaxError;
	// Partition type, see SamplePartitionType
	// ���񻮷ֵķ�ʽ����ö�٣���ˮ���㷨�������㷨���ֲ��㷨
	int partitionType;
	// Bounds of the area to mesh
	// ���񻯵�������С�����߽�
	float navMeshBMin[3];
	float navMeshBMax[3];
	// Size of the tiles in voxels
	// ��Ƭ�Ĵ�С����λ������
	float tileSize;
};

// ������ṩ�� Mesh �����������ݵļ��أ��Լ� BVTree �Ĺ�����
class InputGeom
{
	// �ֿ�����������
	rcChunkyTriMesh* m_chunkyMesh;
	// ������άģ���ļ�
	rcMeshLoaderObj* m_mesh;
	// ������С�����߽�
	float m_meshBMin[3], m_meshBMax[3];
	// �������������һЩ����
	BuildSettings m_buildSettings;
	// �Ƿ�Ӧ�� m_buildSettings �Ĳ�������
	bool m_hasBuildSettings;
	
	/// @name Off-Mesh connections.
	///@{
	///
	/// �����ɢ���ӵ�
	static const int MAX_OFFMESH_CONNECTIONS = 256;
	// ��ɢ���ӵĶ�������
	float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*3*2];
	// ��ɢ���ӵİ뾶����
	float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
	// ��ɢ���ӵĳ�������
	unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
	// ��ɢ���ӵ���������
	unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
	// ��ɢ���ӵı�ʶ������
	unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
	// ��ɢ���ӵ�id����
	unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
	// ʵ�ʵ���ɢ������
	int m_offMeshConCount;
	///@}

	/// @name Convex Volumes.
	///@{
	/// 
	/// ͹����������������
	static const int MAX_VOLUMES = 256;
	// ͹�������������
	ConvexVolume m_volumes[MAX_VOLUMES];
	// ͹������������
	int m_volumeCount;
	///@}
	
	// ���ص��������ļ������� BVTree
	bool loadMesh(class rcContext* ctx, const std::string& filepath);
	// ��������Ǽ���һϵ�еĵ��������ļ����ڲ��� loadMesh ʵ�֡�
	// .gset �ļ��� recastnavigation ˽�е��ļ���ʽ���Ƕ� .obj �ļ��ĵİ���
	// ��ʵ����������Զ�������ѡ��
	bool loadGeomSet(class rcContext* ctx, const std::string& filepath);
public:
	InputGeom();
	~InputGeom();
	
	// ���ص��������ļ����ڲ��򵥵�ͨ���ļ���׺�������� ���� loadMesh
	// �� loadGeomSet ����������������Ϊ����ļ��غ�����
	bool load(class rcContext* ctx, const std::string& filepath);
	bool saveGeomSet(const BuildSettings* settings);
	
	/// Method to return static mesh data.
	const rcMeshLoaderObj* getMesh() const { return m_mesh; }
	const float* getMeshBoundsMin() const { return m_meshBMin; }
	const float* getMeshBoundsMax() const { return m_meshBMax; }
	const float* getNavMeshBoundsMin() const { return m_hasBuildSettings ? m_buildSettings.navMeshBMin : m_meshBMin; }
	const float* getNavMeshBoundsMax() const { return m_hasBuildSettings ? m_buildSettings.navMeshBMax : m_meshBMax; }
	const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }
	const BuildSettings* getBuildSettings() const { return m_hasBuildSettings ? &m_buildSettings : 0; }

	// �����������ε��ཻ���ԣ��������Ƿ��������ཻ
	bool raycastMesh(float* src, float* dst, float& tmin);

	/// @name Off-Mesh connections.
	///@{
	int getOffMeshConnectionCount() const { return m_offMeshConCount; }
	const float* getOffMeshConnectionVerts() const { return m_offMeshConVerts; }
	const float* getOffMeshConnectionRads() const { return m_offMeshConRads; }
	const unsigned char* getOffMeshConnectionDirs() const { return m_offMeshConDirs; }
	const unsigned char* getOffMeshConnectionAreas() const { return m_offMeshConAreas; }
	const unsigned short* getOffMeshConnectionFlags() const { return m_offMeshConFlags; }
	const unsigned int* getOffMeshConnectionId() const { return m_offMeshConId; }
	void addOffMeshConnection(const float* spos, const float* epos, const float rad,
							  unsigned char bidir, unsigned char area, unsigned short flags);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
	///@}

	/// @name Box Volumes.
	///@{
	int getConvexVolumeCount() const { return m_volumeCount; }
	const ConvexVolume* getConvexVolumes() const { return m_volumes; }
	void addConvexVolume(const float* verts, const int nverts,
						 const float minh, const float maxh, unsigned char area);
	void deleteConvexVolume(int i);
	void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
	///@}
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	InputGeom(const InputGeom&);
	InputGeom& operator=(const InputGeom&);
};

#endif // INPUTGEOM_H
