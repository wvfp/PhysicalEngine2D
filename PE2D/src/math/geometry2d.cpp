#include "math/geometry2d.hpp"
#include <cmath>
#include <stdexcept>

namespace PE2D {
	// Triangle 类实现
	Triangle::Triangle(const Vector2D& p1, const Vector2D& p2, const Vector2D& p3) {
		// 初始化三角形的三个顶点
		points.push_back(p1);
		points.push_back(p2);
		points.push_back(p3);
		// 设置形状类型为三角形
		shapeType = ShapeType::TRIANGLE;
		pos = getCentroid();
	}

	float Triangle::area() const {
		// 使用向量叉积计算三角形面积
		Vector2D v1 = points[1] - points[0];
		Vector2D v2 = points[2] - points[0];
		return 0.5f * std::abs(v1.x() * v2.y() - v1.y() * v2.x());
	}

	// Polygon 类实现
	Polygon::Polygon(const std::vector<Vector2D>& vertices) : vertices(vertices) {
		// 检查多边形的顶点数量是否大于等于3
		if (vertices.size() < 3) {
			throw std::invalid_argument("A polygon must have at least 3 vertices.");
		}
		// 设置形状类型为多边形
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

	// Rectangle 类实现
	Rectangle::Rectangle(const Vector2D& topLeft, const Vector2D& bottomRight)
		: topLeft(topLeft), bottomRight(bottomRight) {
		// 设置形状类型为矩形
		shapeType = ShapeType::RECTANGLE;
		pos = getCentroid();
	}

	float Rectangle::area() const {
		float width = std::abs(bottomRight.x() - topLeft.x());
		float height = std::abs(bottomRight.y() - topLeft.y());
		return width * height;
	}

	// Circle 类实现
	Circle::Circle(const Vector2D& center, float radius) : center(center), radius(radius) {
		// 设置形状类型为圆形
		shapeType = ShapeType::CIRCLE;
		pos = center;
	}

	float Circle::area() const {
		return static_cast<float>(PI) * radius * radius;
	}
	//胶囊体类实现
	Capsule::Capsule(const Vector2D& center, float radius, float height)
		: center(center), radius(radius), height(height) {
		// 设置形状类型为胶囊体
		shapeType = ShapeType::CAPSULE;
		pos = getCentroid();
	}
	float Capsule::area() const {
		// 胶囊体的面积 = 圆的面积 + 矩形的面积
		return 2 * static_cast<float>(PI) * radius * radius + height * (2 * radius);
	}

}