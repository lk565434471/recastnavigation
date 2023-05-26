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

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

// Some math headers don't have PI defined.
static const float DU_PI = 3.14159265f;

// 用于 Debug 调试的基本图形类型
enum duDebugDrawPrimitives
{
	DU_DRAW_POINTS, // 绘制点（单个像素）
	DU_DRAW_LINES, // 绘制线段
	DU_DRAW_TRIS, // 绘制三角形
	DU_DRAW_QUADS // 绘制四边形
};

/// Abstract debug draw interface.
// 绘制 Debug 辅助图形的抽象接口类
struct duDebugDraw
{
	virtual ~duDebugDraw() = 0;
	
	virtual void depthMask(bool state) = 0;

	virtual void texture(bool state) = 0;

	/// Begin drawing primitives.
	///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
	///  @param size [in] size of a primitive, applies to point size and line width only.
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float* pos, unsigned int color) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float x, const float y, const float z, unsigned int color) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param uv [in] the uv coordinates of the verts.
	virtual void vertex(const float* pos, unsigned int color, const float* uv) = 0;
	
	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	///  @param u,v [in] the uv coordinates of the verts.
	virtual void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v) = 0;
	
	/// End drawing primitives.
	virtual void end() = 0;

	/// Compute a color for given area.
	virtual unsigned int areaToCol(unsigned int area);
};

// A Alpha 透明度
inline unsigned int duRGBA(int r, int g, int b, int a)
{
	return ((unsigned int)r) | ((unsigned int)g << 8) | ((unsigned int)b << 16) | ((unsigned int)a << 24);
}

// F 需要额外存储折射、反射信息时会用到
inline unsigned int duRGBAf(float fr, float fg, float fb, float fa)
{
	unsigned char r = (unsigned char)(fr*255.0f);
	unsigned char g = (unsigned char)(fg*255.0f);
	unsigned char b = (unsigned char)(fb*255.0f);
	unsigned char a = (unsigned char)(fa*255.0f);
	return duRGBA(r,g,b,a);
}

// 将一个32位整型值和代表透明度的a值，转换成 RGBA 值
unsigned int duIntToCol(int i, int a);
// 将 RGB 的每个分量值存储到 col 数组中
// col[0] = r, col[1] = g, col[2] = b
void duIntToCol(int i, float* col);

// 参数 col 表示一个 RGBA 值
// 低24位分别表示：r、g、b，低32位表示 alpha，即透明度
// 参数 d 有可能会改变 r、g、b 的值，最终和 a 组合成一个新的 RGBA 值
inline unsigned int duMultCol(const unsigned int col, const unsigned int d)
{
	const unsigned int r = col & 0xff;
	const unsigned int g = (col >> 8) & 0xff;
	const unsigned int b = (col >> 16) & 0xff;
	const unsigned int a = (col >> 24) & 0xff;
	return duRGBA((r*d) >> 8, (g*d) >> 8, (b*d) >> 8, a);
}

// 这个函数的作用是将 RGBA 的颜色变暗，最后返回新的 RGBA 值
inline unsigned int duDarkenCol(unsigned int col)
{
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000);
}

// 计算两个 RGBA 颜色值之间的差值
// u 表示差值的比例，取值范围在 [0,255] 之间，u 越接近 255，最终的 RGBA 颜色值就越接近 cb
inline unsigned int duLerpCol(unsigned int ca, unsigned int cb, unsigned int u)
{
	const unsigned int ra = ca & 0xff;
	const unsigned int ga = (ca >> 8) & 0xff;
	const unsigned int ba = (ca >> 16) & 0xff;
	const unsigned int aa = (ca >> 24) & 0xff;
	const unsigned int rb = cb & 0xff;
	const unsigned int gb = (cb >> 8) & 0xff;
	const unsigned int bb = (cb >> 16) & 0xff;
	const unsigned int ab = (cb >> 24) & 0xff;
	
	unsigned int r = (ra*(255-u) + rb*u)/255;
	unsigned int g = (ga*(255-u) + gb*u)/255;
	unsigned int b = (ba*(255-u) + bb*u)/255;
	unsigned int a = (aa*(255-u) + ab*u)/255;
	return duRGBA(r,g,b,a);
}

// 将 RGBA 的颜色值的 Alpha 值，设定为指定的值
// a Alpha 的取值范围在 [0,255] 之间
inline unsigned int duTransCol(unsigned int c, unsigned int a)
{
	return (a<<24) | (c & 0x00ffffff);
}

// 计算一个长方体的六个面的颜色值，包括顶部和侧面
void duCalcBoxColors(unsigned int* colors, unsigned int colTop, unsigned int colSide);

// 绘制调试使用的空心圆柱体
void duDebugDrawCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, unsigned int col, const float lineWidth);
// 绘制调试使用的包围盒
void duDebugDrawBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, unsigned int col, const float lineWidth);

// 绘制一个用调试的弧线
// x0、y0、z0、x1、y1、z1，表示弧线的起点和终点
// as0、as1，表示起点和终点的箭头尺寸
// h 表示弧线的高度
// col 表示弧线的颜色
void duDebugDrawArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					const float x1, const float y1, const float z1, const float h,
					const float as0, const float as1, unsigned int col, const float lineWidth);

// 绘制箭头
void duDebugDrawArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					  const float x1, const float y1, const float z1,
					  const float as0, const float as1, unsigned int col, const float lineWidth);

// 绘制圆
void duDebugDrawCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					   const float r, unsigned int col, const float lineWidth);

// 绘制一个十字架
// x、y、z，表示十字架中心的位置
// s，表示十字架的大小
void duDebugDrawCross(struct duDebugDraw* dd, const float x, const float y, const float z,
					  const float size, unsigned int col, const float lineWidth);

// 绘制立方体
// minx、miny、minz、maxx、maxy、maxz 表示立方体的最小和最大顶点坐标
// fcol 表示立方体的六个面的颜色
void duDebugDrawBox(struct duDebugDraw* dd, float minx, float miny, float minz,
					float maxx, float maxy, float maxz, const unsigned int* fcol);

// 绘制圆柱体
// minx、miny、minz、maxx、maxy、maxz 表示最小和最大顶点坐标
// col 圆柱体的颜色
void duDebugDrawCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
						 float maxx, float maxy, float maxz, unsigned int col);


// 在 X、Z 轴上绘制一个网格
// ox, oy, oz 表示起始坐标
// size 表示网格大小
// w 和 h 表示网格的宽度和高度
// col 网格的颜色
// lineWidth 线段的宽度
void duDebugDrawGridXZ(struct duDebugDraw* dd, const float ox, const float oy, const float oz,
					   const int w, const int h, const float size,
					   const unsigned int col, const float lineWidth);


// Versions without begin/end, can be used to draw multiple primitives.
// 绘制一个空心的圆柱体
void duAppendCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						  float maxx, float maxy, float maxz, unsigned int col);
// 绘制一个空心的包围盒
void duAppendBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
					 float maxx, float maxy, float maxz, unsigned int col);

// 绘制包围盒的顶面和底面
void duAppendBoxPoints(struct duDebugDraw* dd, float minx, float miny, float minz,
					   float maxx, float maxy, float maxz, unsigned int col);

// 绘制一个用调试的弧线
// x0、y0、z0、x1、y1、z1，表示弧线的起点和终点
// as0、as1，表示起点和终点的箭头尺寸
// h 表示弧线的高度
// col 表示弧线的颜色
void duAppendArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				 const float x1, const float y1, const float z1, const float h,
				 const float as0, const float as1, unsigned int col);

void duAppendArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				   const float x1, const float y1, const float z1,
				   const float as0, const float as1, unsigned int col);

void duAppendCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					const float r, unsigned int col);

void duAppendCross(struct duDebugDraw* dd, const float x, const float y, const float z,
				   const float size, unsigned int col);

void duAppendBox(struct duDebugDraw* dd, float minx, float miny, float minz,
				 float maxx, float maxy, float maxz, const unsigned int* fcol);

void duAppendCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
					  float maxx, float maxy, float maxz, unsigned int col);


// duDebugDraw 仅记录用于绘制图形的数据
class duDisplayList : public duDebugDraw
{
	// m_pos 存储图形绘制的顶点坐标信息（x,y,z）
	float* m_pos;
	// m_color 每条线段的颜色信息
	unsigned int* m_color;
	// 已存储的顶点数量
	int m_size;
	// 存储顶点的数量上限
	int m_cap;

	// 绘制基本图像的类型枚举值
	duDebugDrawPrimitives m_prim;
	// 线段的宽度
	float m_primSize;
	// 深度缓冲区开启、关闭控制变量
	bool m_depthMask;
	
	// 重新分配顶点的数量上限
	// 函数做2件事情： 1.重新分配内存 2.拷贝原有数据到新分配的内存
	void resize(int cap);
	
public:
	// 默认分配512个顶点
	duDisplayList(int cap = 512);
	virtual ~duDisplayList();
	virtual void depthMask(bool state);
	// 标记开始绘制
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	// 记录顶点的坐标，颜色
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void vertex(const float* pos, unsigned int color);
	// 标记结束绘制
	virtual void end();
	// 重置已使用的顶点，将顶点下标归零
	void clear();
	// 通过 duDebugDraw 的具体实现来绘制图形
	void draw(struct duDebugDraw* dd);
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	duDisplayList(const duDisplayList&);
	duDisplayList& operator=(const duDisplayList&);
};


#endif // DEBUGDRAW_H
