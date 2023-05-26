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

// ��Ƭ������ͷ��Ϣ
struct dtTileCacheLayerHeader
{
	// ħ������
	int magic;								///< Data magic
	// ���ݰ汾�ţ������ڽ����ļ���ʽ�������Ű汾���������ݲ����Լ���
	int version;							///< Data version
	// tx��ty �������У�tlayer ����㼶�������� BVTree �У����Լ����жϡ�ɸѡ������
	int tx,ty,tlayer;
	// AABB ��Χ�еķ�Χ
	float bmin[3], bmax[3];
	// �߶ȷ�Χ
	unsigned short hmin, hmax;				///< Height min/max range
	// ͼ��ĳߴ磨������������ж��ٸ����ӣ�
	unsigned char width, height;			///< Dimension of the layer.
	// ����������ķ�Χ���
	unsigned char minx, maxx, miny, maxy;	///< Usable sub-region.
};

// ��Ƭ����㼶��Ϣ
struct dtTileCacheLayer
{
	// ��Ƭ������ͷ��Ϣ
	dtTileCacheLayerHeader* header;
	// ��ͨ����������
	unsigned char regCount;					///< Region count.
	// �߶���Ϣ�������ж���ͨ��
	unsigned char* heights;
	unsigned char* areas;
	unsigned char* cons;
	// x�᷽�������ǰ��Ԫ����ǰһ����Ԫ���ǿ���ͨ�ģ���ô���ǵ�id��ͬ
	// ���򣬴洢����ʽ����ͨ��id
	unsigned char* regs;
};

struct dtTileCacheContour
{
	// ��������
	int nverts;
	// �������ݻ�����
	unsigned char* verts;
	// ��ͨ������
	unsigned char reg;
	// ��������
	unsigned char area;
};

struct dtTileCacheContourSet
{
	// ���������������
	int nconts;
	// �������������ݽṹ
	dtTileCacheContour* conts;
};

// ���� flags �� areas ��Ԫ��������������ȵģ����ǵ����޶��������ε���������
struct dtTileCachePolyMesh
{
	int nvp;
	// ���������
	int nverts;				///< Number of vertices.
	// ����ε�����
	int npolys;				///< Number of polygons.
	// ����Ķ��㣬ÿ��������3��Ԫ�����
	unsigned short* verts;	///< Vertices of the mesh, 3 elements per vertex.
	// ����Ķ���Σ�ÿ���������2��Ԫ�����
	unsigned short* polys;	///< Polygons of the mesh, nvp*2 elements per polygon.
	// ÿ������εı�ʶ��
	unsigned short* flags;	///< Per polygon flags.
	// ÿ�����������������id
	unsigned char* areas;	///< Area ID of polygons.
};

// ��Ƭ������ڴ������
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

// ��Ƭ�����ѹ������
struct dtTileCacheCompressor
{
	virtual ~dtTileCacheCompressor();

	virtual int maxCompressedSize(const int bufferSize) = 0;
	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int maxCompressedSize, int* compressedSize) = 0;
	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize) = 0;
};


// ������Ƭ����Ĳ㼶����
// comp ��Ƭ��������ѹ���ľ���ʵ�ֶ���
// header ��Ƭ����Ĳ㼶���ݣ��������ݵĸ�ʽ�����ݵ�
// heights ����ĸ߶���Ϣ
// areas ������Ϣ
// cons ������Ϣ
// outData ѹ����Ķ���������
// outDataSize ѹ����Ķ��������ݴ�С
dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor* comp,
							   dtTileCacheLayerHeader* header,
							   const unsigned char* heights,
							   const unsigned char* areas,
							   const unsigned char* cons,
							   unsigned char** outData, int* outDataSize);

// �ͷ���Ƭ����Ĳ㼶������ռ�õ��ڴ�
void dtFreeTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer);

// ��ѹ����Ƭ����Ĳ㼶����
// alloc �ڴ������
// comp ��Ƭ��������ѹ���ľ���ʵ�ֶ���
// compressed ��ѹ���Ķ���������
// compressedSize ��ѹ���Ķ��������ݴ�С
// layerOut ��ѹ�����Ƭ����㼶����
dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheCompressor* comp,
									unsigned char* compressed, const int compressedSize,
									dtTileCacheLayer** layerOut);

// ���� dtTileCacheContourSet ���󣬲���ʼ�����ڲ���Ա
dtTileCacheContourSet* dtAllocTileCacheContourSet(dtTileCacheAlloc* alloc);

// ���ȣ��ͷ��� dtTileCacheContourSet ����Ķ�����ڴ档Ȼ���ͷ��Լ����ڴ档
void dtFreeTileCacheContourSet(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset);

// ���� dtTileCachePolyMesh ���󣬲���ʼ�����ڲ���Ա
dtTileCachePolyMesh* dtAllocTileCachePolyMesh(dtTileCacheAlloc* alloc);

// ���ȣ��ͷ��� dtTileCachePolyMesh ����Ķ�����ڴ档Ȼ���ͷ��Լ����ڴ档
void dtFreeTileCachePolyMesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh);

// �ڵ��������б��һ��Բ���������
dtStatus dtMarkCylinderArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
							const float* pos, const float radius, const float height, const unsigned char areaId);

// �ڵ��������б��һ�� AABB����Χ�У�������
dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* bmin, const float* bmax, const unsigned char areaId);

// �ڵ��������б��һ�� OBB����Χ�У�������
dtStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* center, const float* halfExtents, const float* rotAux, const unsigned char areaId);

// ������������Ѱ·�㷨�Ŀ�������������
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
