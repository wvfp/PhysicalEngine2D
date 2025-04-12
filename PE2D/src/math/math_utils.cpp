// math_utils.cpp
#include "math/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace PE2D {
	// 计算两点之间的距离
	float distanceBetweenPoints(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2) {
		return std::sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]));
	}

	// 计算点到直线的距离
	float distanceFromPointToLine(const PE2D::Vector2D& point, const PE2D::Vector2D& lineStart, const PE2D::Vector2D& lineEnd) {
		float A = point[0] - lineStart[0];
		float B = point[1] - lineStart[1];
		float C = lineEnd[0] - lineStart[0];
		float D = lineEnd[1] - lineStart[1];
		float dot = A * C + B * D;
		float len_sq = C * C + D * D;
		float param = dot / len_sq;
		float xx, yy;

		if (param < 0 || (lineStart == lineEnd)) {
			xx = lineStart[0];
			yy = lineStart[1];
		}
		else if (param > 1) {
			xx = lineEnd[0];
			yy = lineEnd[1];
		}
		else {
			xx = lineStart[0] + param * C;
			yy = lineStart[1] + param * D;
		}

		return distanceBetweenPoints(point, PE2D::Vector2D(xx, yy));
	}

	// 判断点是否在多边形内
	bool isPointInPolygon(const PE2D::Vector2D& point, const std::vector<PE2D::Vector2D>& polygon) {
		int n = polygon.size();
		bool inside = false;
		for (int i = 0, j = n - 1; i < n; j = i++) {
			float xi = polygon[i][0], yi = polygon[i][1];
			float xj = polygon[j][0], yj = polygon[j][1];
			bool intersect = ((yi > point[1]) != (yj > point[1])) && (point[0] < (xj - xi) * (point[1] - yi) / (yj - yi) + xi);
			if (intersect) inside = !inside;
		}
		return inside;
	}

	// 判断点是否在矩形内
	bool isPointInRectangle(const PE2D::Vector2D& point, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight) {
		return point[0] >= topLeft[0] && point[0] <= bottomRight[0] && point[1] >= topLeft[1] && point[1] <= bottomRight[1];
	}

	// 判断两条直线是否相交
	bool isLineIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4) {
		float denom = (p4[1] - p3[1]) * (p2[0] - p1[0]) - (p4[0] - p3[0]) * (p2[1] - p1[1]);
		if (denom == 0.0f) return false;
		float ua = ((p4[0] - p3[0]) * (p1[1] - p3[1]) - (p4[1] - p3[1]) * (p1[0] - p3[0])) / denom;
		float ub = ((p2[0] - p1[0]) * (p1[1] - p3[1]) - (p2[1] - p1[1]) * (p1[0] - p3[0])) / denom;
		return ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f;
	}

	// 判断两条线段是否相交
	bool isSegmentIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4) {
		auto ccw = [](const PE2D::Vector2D& a, const PE2D::Vector2D& b, const PE2D::Vector2D& c) {
			return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]);
			};
		return ccw(p1, p3, p4) != ccw(p2, p3, p4) && ccw(p1, p2, p3) != ccw(p1, p2, p4);
	}

	// 判断两个圆是否相交
	bool isCircleIntersect(const PE2D::Vector2D& center1, float radius1, const PE2D::Vector2D& center2, float radius2) {
		float distSq = (center1[0] - center2[0]) * (center1[0] - center2[0]) + (center1[1] - center2[1]) * (center1[1] - center2[1]);
		float radiusSumSq = (radius1 + radius2) * (radius1 + radius2);
		return distSq <= radiusSumSq;
	}

	// 判断圆与矩形是否相交
	bool isCircleRectangleIntersect(const PE2D::Vector2D& center, float radius, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight) {
		float closestX = std::clamp(center[0], topLeft[0], bottomRight[0]);
		float closestY = std::clamp(center[1], topLeft[1], bottomRight[1]);
		float distanceX = center[0] - closestX;
		float distanceY = center[1] - closestY;
		return (distanceX * distanceX + distanceY * distanceY) < (radius * radius);
	}

	// 判断圆形与三角形是否相交
	bool isCircleTriangleIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& trianglePoints) {
		if (trianglePoints.size() != 3) return false;

		// 检查圆心是否在三角形内部
		if (isPointInPolygon(center, trianglePoints)) return true;

		// 检查圆心到三角形各边的距离是否小于等于半径
		for (int i = 0; i < 3; ++i) {
			int j = (i + 1) % 3;
			if (distanceFromPointToLine(center, trianglePoints[i], trianglePoints[j]) <= radius) {
				return true;
			}
		}

		return false;
	}
	// 计算多边形的重心
	Vector2D calculateCentroid(const std::vector<Vector2D>& polygon) {
		double cx = 0, cy = 0;
		size_t n = polygon.size();
		for (const auto& point : polygon) {
			cx += point.x();
			cy += point.y();
		}
		cx /= n;
		cy /= n;
		return Vector2D(cx, cy);
	}
	// 判断圆形与多边形是否相交
	bool isCirclePolygonIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& polygonVertices) {
		if (polygonVertices.size() < 3) return false;

		// 检查圆心是否在多边形内部
		if (isPointInPolygon(center, polygonVertices)) return true;

		// 检查圆心到多边形各边的距离是否小于等于半径
		for (size_t i = 0; i < polygonVertices.size(); ++i) {
			size_t j = (i + 1) % polygonVertices.size();
			if (distanceFromPointToLine(center, polygonVertices[i], polygonVertices[j]) <= radius) {
				return true;
			}
		}

		return false;
	}
	// 三角函数相关
	float sin(float angle) { return std::sin(angle); }
	float cos(float angle) { return std::cos(angle); }
	float tan(float angle) { return std::tan(angle); }
	float asin(float value) { return std::asin(value); }
	float acos(float value) { return std::acos(value); }
	float atan(float value) { return std::atan(value); }
	float atan2(float y, float x) { return std::atan2(y, x); }

	// 角度转换相关
	float radiansToDegrees(float radians) { return radians * (180.0f / PI); }
	float degreesToRadians(float degrees) { return degrees * (PI / 180.0f); }

	// 其他数学操作
	float clamp(float value, float min, float max) {
		return std::max(min, std::min(value, max));
	}

	float lerp(float a, float b, float t) {
		return a + t * (b - a);
	}

	float smoothstep(float edge0, float edge1, float x) {
		x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
		return x * x * (3 - 2 * x);
	}

	float abs(float value) { return std::fabs(value); }
	float sqrt(float value) { return std::sqrt(value); }
	float pow(float base, float exponent) { return std::pow(base, exponent); }
}