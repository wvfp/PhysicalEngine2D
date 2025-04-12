#include "math/geometry2d.hpp"
#include <cmath>
#include <stdexcept>

namespace PE2D {
	// Triangle ��ʵ��
	Triangle::Triangle(const Vector2D& p1, const Vector2D& p2, const Vector2D& p3) {
		// ��ʼ�������ε���������
		points.push_back(p1);
		points.push_back(p2);
		points.push_back(p3);
		// ������״����Ϊ������
		shapeType = ShapeType::TRIANGLE;
		pos = getCentroid();
	}

	float Triangle::area() const {
		// ʹ����������������������
		Vector2D v1 = points[1] - points[0];
		Vector2D v2 = points[2] - points[0];
		return 0.5f * std::abs(v1.x() * v2.y() - v1.y() * v2.x());
	}

	// Polygon ��ʵ��
	Polygon::Polygon(const std::vector<Vector2D>& vertices) : vertices(vertices) {
		// ������εĶ��������Ƿ���ڵ���3
		if (vertices.size() < 3) {
			throw std::invalid_argument("A polygon must have at least 3 vertices.");
		}
		// ������״����Ϊ�����
		shapeType = ShapeType::POLYGON;
		pos = getCentroid();
	}

	float Polygon::area() const {
		float area = 0.0f;
		int n = vertices.size();
		for (int i = 0; i < n; ++i) {
			int j = (i + 1) % n;
			area += vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y();
		}
		return 0.5f * std::abs(area);
	}

	// Rectangle ��ʵ��
	Rectangle::Rectangle(const Vector2D& topLeft, const Vector2D& bottomRight)
		: topLeft(topLeft), bottomRight(bottomRight) {
		// ������״����Ϊ����
		shapeType = ShapeType::RECTANGLE;
		pos = getCentroid();
	}

	float Rectangle::area() const {
		float width = std::abs(bottomRight.x() - topLeft.x());
		float height = std::abs(bottomRight.y() - topLeft.y());
		return width * height;
	}

	// Circle ��ʵ��
	Circle::Circle(const Vector2D& center, float radius) : center(center), radius(radius) {
		// ������״����ΪԲ��
		shapeType = ShapeType::CIRCLE;
		pos = center;
	}

	float Circle::area() const {
		return static_cast<float>(PI) * radius * radius;
	}
	//��������ʵ��
	Capsule::Capsule(const Vector2D& center, float radius, float height)
		: center(center), radius(radius), height(height) {
		// ������״����Ϊ������
		shapeType = ShapeType::CAPSULE;
		pos = getCentroid();
	}
	float Capsule::area() const {
		// ���������� = Բ����� + ���ε����
		return 2 * static_cast<float>(PI) * radius * radius + height * (2 * radius);
	}

}