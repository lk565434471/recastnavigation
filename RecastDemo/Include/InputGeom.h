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
	// 单元格尺寸（以世界单位计算）
	float cellSize;
	// Cell height in world units
	// 单元格高度（以世界单位计算）
	float cellHeight;
	// Agent height in world units
	// 代理高度（以世界单位计算）
	float agentHeight;
	// Agent radius in world units
	// 代理半径（以世界单位计算）
	float agentRadius;
	// Agent max climb in world units
	// 代理可以攀爬的最大高度（以世界单位计算）
	float agentMaxClimb;
	// Agent max slope in degrees
	// // 代理可以攀爬的最大坡度
	float agentMaxSlope;
	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	float regionMinSize;
	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	// 在将高度图划分为一系列区域时，将多少个区域合并为一个更大的区域
	// 用以减少最终生成的导航网格中的连接数，从而提高生成速度和导航效率
	float regionMergeSize;
	// Edge max length in world units
	// 边的最大长度（以世界单位计算）
	float edgeMaxLen;
	// Edge max error in voxels
	// 寻路网格生成的边缘最大误差值，单位为：体素（包含三个维度上的信息，颜色、透明度、密度等）
	float edgeMaxError;
	// 多边形的最大顶点数
	float vertsPerPoly;
	// Detail sample distance in voxels
	float detailSampleDist;
	// Detail sample max error in voxel heights.
	// 细节采用的高度体素最大误差值
	float detailSampleMaxError;
	// Partition type, see SamplePartitionType
	// 网格划分的方式类型枚举：分水岭算法、单调算法、分层算法
	int partitionType;
	// Bounds of the area to mesh
	// 网格化的区域最小、最大边界
	float navMeshBMin[3];
	float navMeshBMax[3];
	// Size of the tiles in voxels
	// 瓦片的大小，单位：体素
	float tileSize;
};

// 这个类提供了 Mesh 导航网格数据的加载，以及 BVTree 的构建。
class InputGeom
{
	// 分块三角形网格
	rcChunkyTriMesh* m_chunkyMesh;
	// 解析三维模型文件
	rcMeshLoaderObj* m_mesh;
	// 网格最小、最大边界
	float m_meshBMin[3], m_meshBMax[3];
	// 构建导航网格的一些参数
	BuildSettings m_buildSettings;
	// 是否应用 m_buildSettings 的参数配置
	bool m_hasBuildSettings;
	
	/// @name Off-Mesh connections.
	///@{
	///
	/// 最大离散连接点
	static const int MAX_OFFMESH_CONNECTIONS = 256;
	// 离散连接的顶点数组
	float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*3*2];
	// 离散连接的半径数组
	float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
	// 离散连接的朝向数组
	unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
	// 离散连接的区域数组
	unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
	// 离散连接的标识符数组
	unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
	// 离散连接的id数组
	unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
	// 实际的离散连接数
	int m_offMeshConCount;
	///@}

	/// @name Convex Volumes.
	///@{
	/// 
	/// 凸多边形区域最大上限
	static const int MAX_VOLUMES = 256;
	// 凸多边形区域数组
	ConvexVolume m_volumes[MAX_VOLUMES];
	// 凸多边形区域计数
	int m_volumeCount;
	///@}
	
	// 加载导航网格文件，构建 BVTree
	bool loadMesh(class rcContext* ctx, const std::string& filepath);
	// 这个函数是加载一系列的导航网格文件，内部由 loadMesh 实现。
	// .gset 文件是 recastnavigation 私有的文件格式，是对 .obj 文件的的包裹
	// 以实现其特殊的自定义配置选项
	bool loadGeomSet(class rcContext* ctx, const std::string& filepath);
public:
	InputGeom();
	~InputGeom();
	
	// 加载导航网格文件，内部简单的通过文件后缀名称区分 调用 loadMesh
	// 或 loadGeomSet 函数，这两个函数为具体的加载函数。
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

	// 射线与三角形的相交测试，即射线是否与网格相交
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
