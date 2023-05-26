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

#ifndef DETOURTILECACHEBUILDER_H
#define DETOURTILECACHEBUILDER_H

#include "DetourAlloc.h"
#include "DetourStatus.h"

static const int DT_TILECACHE_MAGIC = 'D'<<24 | 'T'<<16 | 'L'<<8 | 'R'; ///< 'DTLR';
static const int DT_TILECACHE_VERSION = 1;

static const unsigned char DT_TILECACHE_NULL_AREA = 0;
static const unsigned char DT_TILECACHE_WALKABLE_AREA = 63;
static const unsigned short DT_TILECACHE_NULL_IDX = 0xffff;

// 瓦片缓存层的头信息
struct dtTileCacheLayerHeader
{
	// 魔数数据
	int magic;								///< Data magic
	// 数据版本号，多用于解析文件格式，及随着版本迭代的数据差异性兼容
	int version;							///< Data version
	// tx、ty 代表行列，tlayer 代表层级，多用于 BVTree 中，可以加速判断、筛选操作等
	int tx,ty,tlayer;
	// AABB 包围盒的范围
	float bmin[3], bmax[3];
	// 高度范围
	unsigned short hmin, hmax;				///< Height min/max range
	// 图层的尺寸（即横向和纵向有多少个格子）
	unsigned char width, height;			///< Dimension of the layer.
	// 用于自区域的范围检查
	unsigned char minx, maxx, miny, maxy;	///< Usable sub-region.
};

// 瓦片缓存层级信息
struct dtTileCacheLayer
{
	// 瓦片缓存层的头信息
	dtTileCacheLayerHeader* header;
	// 连通性区域数量
	unsigned char regCount;					///< Region count.
	// 高度信息，用于判断连通性
	unsigned char* heights;
	unsigned char* areas;
	unsigned char* cons;
	// x轴方向，如果当前单元格与前一个单元格是可连通的，那么他们的id相同
	// 否则，存储自增式的连通性id
	unsigned char* regs;
};

struct dtTileCacheContour
{
	// 顶点数量
	int nverts;
	// 顶点数据缓冲区
	unsigned char* verts;
	// 可通行区域
	unsigned char reg;
	// 网格区域
	unsigned char area;
};

struct dtTileCacheContourSet
{
	// 可行走区域的数量
	int nconts;
	// 可行走区域数据结构
	dtTileCacheContour* conts;
};

// 其中 flags 和 areas 的元素数量上限是相等的，他们的上限都是三角形的数量上限
struct dtTileCachePolyMesh
{
	int nvp;
	// 顶点的数量
	int nverts;				///< Number of vertices.
	// 多边形的数量
	int npolys;				///< Number of polygons.
	// 网格的顶点，每个顶点由3个元素组成
	unsigned short* verts;	///< Vertices of the mesh, 3 elements per vertex.
	// 网格的多边形，每个多边形由2个元素组成
	unsigned short* polys;	///< Polygons of the mesh, nvp*2 elements per polygon.
	// 每个多边形的标识符
	unsigned short* flags;	///< Per polygon flags.
	// 每个多边形所属的区域id
	unsigned char* areas;	///< Area ID of polygons.
};

// 瓦片缓存的内存分配器
struct dtTileCacheAlloc
{
	virtual ~dtTileCacheAlloc();

	virtual void reset() {}
	
	virtual void* alloc(const size_t size)
	{
		return dtAlloc(size, DT_ALLOC_TEMP);
	}
	
	virtual void free(void* ptr)
	{
		dtFree(ptr);
	}
};

// 瓦片缓存的压缩对象
struct dtTileCacheCompressor
{
	virtual ~dtTileCacheCompressor();

	virtual int maxCompressedSize(const int bufferSize) = 0;
	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int maxCompressedSize, int* compressedSize) = 0;
	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize) = 0;
};


// 构建瓦片缓存的层级数据
// comp 瓦片缓存数据压缩的具体实现对象
// header 瓦片缓存的层级数据，描述数据的格式、内容等
// heights 网格的高度信息
// areas 区域信息
// cons 连接信息
// outData 压缩后的二进制数据
// outDataSize 压缩后的二进制数据大小
dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor* comp,
							   dtTileCacheLayerHeader* header,
							   const unsigned char* heights,
							   const unsigned char* areas,
							   const unsigned char* cons,
							   unsigned char** outData, int* outDataSize);

// 释放瓦片缓存的层级数据所占用的内存
void dtFreeTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer);

// 解压缩瓦片缓存的层级数据
// alloc 内存分配器
// comp 瓦片缓存数据压缩的具体实现对象
// compressed 已压缩的二进制数据
// compressedSize 已压缩的二进制数据大小
// layerOut 解压后的瓦片缓存层级对象
dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheCompressor* comp,
									unsigned char* compressed, const int compressedSize,
									dtTileCacheLayer** layerOut);

// 创建 dtTileCacheContourSet 对象，并初始化其内部成员
dtTileCacheContourSet* dtAllocTileCacheContourSet(dtTileCacheAlloc* alloc);

// 首先，释放由 dtTileCacheContourSet 管理的对象的内存。然后，释放自己的内存。
void dtFreeTileCacheContourSet(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset);

// 创建 dtTileCachePolyMesh 对象，并初始化其内部成员
dtTileCachePolyMesh* dtAllocTileCachePolyMesh(dtTileCacheAlloc* alloc);

// 首先，释放由 dtTileCachePolyMesh 管理的对象的内存。然后，释放自己的内存。
void dtFreeTileCachePolyMesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh);

// 在导航网格中标记一个圆柱体的区域
dtStatus dtMarkCylinderArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
							const float* pos, const float radius, const float height, const unsigned char areaId);

// 在导航网格中标记一个 AABB（包围盒）的区域
dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* bmin, const float* bmax, const unsigned char areaId);

// 在导航网格中标记一个 OBB（包围盒）的区域
dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* center, const float* halfExtents, const float* rotAux, const unsigned char areaId);

// 构建导航网格寻路算法的可行走区域数据
dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc* alloc,
								 dtTileCacheLayer& layer,
								 const int walkableClimb);

dtStatus dtBuildTileCacheContours(dtTileCacheAlloc* alloc,
								  dtTileCacheLayer& layer,
								  const int walkableClimb, 	const float maxError,
								  dtTileCacheContourSet& lcset);

dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc* alloc,
								  dtTileCacheContourSet& lcset,
								  dtTileCachePolyMesh& mesh);

/// Swaps the endianess of the compressed tile data's header (#dtTileCacheLayerHeader).
/// Tile layer data does not need endian swapping as it consits only of bytes.
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtTileCacheHeaderSwapEndian(unsigned char* data, const int dataSize);


#endif // DETOURTILECACHEBUILDER_H
