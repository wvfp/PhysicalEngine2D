// math_utils.hpp
#pragma once
#include "vector2d.hpp"
#include <vector>
#define PI 3.14159265358979323846f

namespace PE2D {
	// 计算两点之间的距离
	float distanceBetweenPoints(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2);

	// 计算点到直线的距离
	float distanceFromPointToLine(const PE2D::Vector2D& point, const PE2D::Vector2D& lineStart, const PE2D::Vector2D& lineEnd);

	// 判断点是否在多边形内
	bool isPointInPolygon(const PE2D::Vector2D& point, const std::vector<PE2D::Vector2D>& polygon);

	// 判断点是否在矩形内
	bool isPointInRectangle(const PE2D::Vector2D& point, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight);

	// 判断两条直线是否相交
	bool isLineIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4);

	// 判断两条线段是否相交
	bool isSegmentIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4);

	// 判断两个圆是否相交
	bool isCircleIntersect(const PE2D::Vector2D& center1, float radius1, const PE2D::Vector2D& center2, float radius2);

	// 判断圆与矩形是否相交
	bool isCircleRectangleIntersect(const PE2D::Vector2D& center, float radius, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight);

	// 判断圆形与三角形是否相交
	bool isCircleTriangleIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& trianglePoints);

	// 判断圆形与多边形是否相交
	bool isCirclePolygonIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& polygonVertices);
	// 计算多边形的重心
	Vector2D calculateCentroid(const std::vector<Vector2D>& polygon);
	// 三角函数相关
	float sin(float angle);
	float cos(float angle);
	float tan(float angle);
	float asin(float value);
	float acos(float value);
	float atan(float value);
	float atan2(float y, float x);

	// 角度转换相关
	float radiansToDegrees(float radians);
	float degreesToRadians(float degrees);

	// 其他数学操作
	float clamp(float value, float min, float max);
	float lerp(float a, float b, float t);
	float smoothstep(float edge0, float edge1, float x);
	float abs(float value);
	float sqrt(float value);
	float pow(float base, float exponent);
}