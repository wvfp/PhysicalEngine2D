// math_utils.hpp
#pragma once
#include "vector2d.hpp"
#include <vector>
#define PI 3.14159265358979323846f

namespace PE2D {
	// ��������֮��ľ���
	float distanceBetweenPoints(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2);

	// ����㵽ֱ�ߵľ���
	float distanceFromPointToLine(const PE2D::Vector2D& point, const PE2D::Vector2D& lineStart, const PE2D::Vector2D& lineEnd);

	// �жϵ��Ƿ��ڶ������
	bool isPointInPolygon(const PE2D::Vector2D& point, const std::vector<PE2D::Vector2D>& polygon);

	// �жϵ��Ƿ��ھ�����
	bool isPointInRectangle(const PE2D::Vector2D& point, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight);

	// �ж�����ֱ���Ƿ��ཻ
	bool isLineIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4);

	// �ж������߶��Ƿ��ཻ
	bool isSegmentIntersect(const PE2D::Vector2D& p1, const PE2D::Vector2D& p2, const PE2D::Vector2D& p3, const PE2D::Vector2D& p4);

	// �ж�����Բ�Ƿ��ཻ
	bool isCircleIntersect(const PE2D::Vector2D& center1, float radius1, const PE2D::Vector2D& center2, float radius2);

	// �ж�Բ������Ƿ��ཻ
	bool isCircleRectangleIntersect(const PE2D::Vector2D& center, float radius, const PE2D::Vector2D& topLeft, const PE2D::Vector2D& bottomRight);

	// �ж�Բ�����������Ƿ��ཻ
	bool isCircleTriangleIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& trianglePoints);

	// �ж�Բ���������Ƿ��ཻ
	bool isCirclePolygonIntersect(const Vector2D& center, float radius, const std::vector<Vector2D>& polygonVertices);
	// �������ε�����
	Vector2D calculateCentroid(const std::vector<Vector2D>& polygon);
	// ���Ǻ������
	float sin(float angle);
	float cos(float angle);
	float tan(float angle);
	float asin(float value);
	float acos(float value);
	float atan(float value);
	float atan2(float y, float x);

	// �Ƕ�ת�����
	float radiansToDegrees(float radians);
	float degreesToRadians(float degrees);

	// ������ѧ����
	float clamp(float value, float min, float max);
	float lerp(float a, float b, float t);
	float smoothstep(float edge0, float edge1, float x);
	float abs(float value);
	float sqrt(float value);
	float pow(float base, float exponent);
}