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

#ifndef DETOURSTATUS_H
#define DETOURSTATUS_H

typedef unsigned int dtStatus;

// High level status.
// 操作失败
static const unsigned int DT_FAILURE = 1u << 31;			// Operation failed.
// 操作成功
static const unsigned int DT_SUCCESS = 1u << 30;			// Operation succeed.
// 操作仍在进行中
static const unsigned int DT_IN_PROGRESS = 1u << 29;		// Operation still in progress.
// 以上这些值会存储在操作结果的高位

// Detail information for status.
// 操作的详细信息，存储在操作结果的低位

// 表示已到达移动请求目的位置？
static const unsigned int DT_STATUS_DETAIL_MASK = 0x0ffffff;
// 无法识别输入数据
static const unsigned int DT_WRONG_MAGIC = 1 << 0;		// Input data is not recognized.
// 输入数据的版本号错误
static const unsigned int DT_WRONG_VERSION = 1 << 1;	// Input data is in wrong version.
// 操作内存不足
static const unsigned int DT_OUT_OF_MEMORY = 1 << 2;	// Operation ran out of memory.
// 输入参数无效
static const unsigned int DT_INVALID_PARAM = 1 << 3;	// An input parameter was invalid.
// 查询的结果缓冲区太小，无法存储所有结果
static const unsigned int DT_BUFFER_TOO_SMALL = 1 << 4;	// Result buffer for the query was too small to store all results.
// 在搜索过程中，查询耗尽了所有节点
static const unsigned int DT_OUT_OF_NODES = 1 << 5;		// Query ran out of nodes during search.
// 路径搜索没有达到最终位置，而是返回了最佳猜测结果
static const unsigned int DT_PARTIAL_RESULT = 1 << 6;	// Query did not reach the end location, returning best guess. 
// 给定的 x,y 坐标已经被分配给了一个网格块
static const unsigned int DT_ALREADY_OCCUPIED = 1 << 7;	// A tile has already been assigned to the given x,y coordinate


// Returns true of status is success.
inline bool dtStatusSucceed(dtStatus status)
{
	return (status & DT_SUCCESS) != 0;
}

// Returns true of status is failure.
inline bool dtStatusFailed(dtStatus status)
{
	return (status & DT_FAILURE) != 0;
}

// Returns true of status is in progress.
inline bool dtStatusInProgress(dtStatus status)
{
	return (status & DT_IN_PROGRESS) != 0;
}

// Returns true if specific detail is set.
inline bool dtStatusDetail(dtStatus status, unsigned int detail)
{
	return (status & detail) != 0;
}

#endif // DETOURSTATUS_H
