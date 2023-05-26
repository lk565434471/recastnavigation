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

#ifndef CHUNKYTRIMESH_H
#define CHUNKYTRIMESH_H

// �����������
struct rcChunkyTriMeshNode
{
	// AABB����Χ�У�����С�����߽�
	float bmin[2];
	float bmax[2];
	// ��ʼ�ڵ������
	int i;
	// �洢�Ľڵ�����
	int n;
};

// �ֿ�����������
// ���ڿ��ٲ��ҳ����еľ�̬��������������������ʱ���ݲ�ѯ�ĵ��λ��
// ����ȷ���õ����ڵ������Σ��Ӷ����ڵ����������ɹ����С�
struct rcChunkyTriMesh
{
	inline rcChunkyTriMesh() : nodes(0), nnodes(0), tris(0), ntris(0), maxTrisPerChunk(0) {}
	inline ~rcChunkyTriMesh() { delete [] nodes; delete [] tris; }

	rcChunkyTriMeshNode* nodes;
	// ʵ�ʴ洢�Ľڵ�������С�� nodes �ĳ���
	int nnodes;
	// �����ε������ߵĶ�����������
	int* tris;
	// �����ε�����
	int ntris;
	// ÿһ�� rcChunkyTriMeshNode ���洢������������
	int maxTrisPerChunk;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcChunkyTriMesh(const rcChunkyTriMesh&);
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh&);
};

/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
/// 
/// ���ɷָ���������������������������һ������ AABB ���ṹ�����ݽṹ��
/// ����ÿ���ڵ㣬������ trisPerChunk ������������
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
						   int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
// ��������������ཻ�ĵ�ͼ�������б�
// ids ���ڴ洢����
// maxIds �洢���������
// ��������ֵ��ʾ��ids �д洢��Ԫ�ظ���
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

/// Returns the chunk indices which overlap the input segment.
// �����������߶��ཻ�ĵ�ͼ�������б�
// ids ���ڴ洢����
// maxIds �洢���������
// ��������ֵ��ʾ��ids �д洢��Ԫ�ظ���
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, float p[2], float q[2], int* ids, const int maxIds);


#endif // CHUNKYTRIMESH_H
