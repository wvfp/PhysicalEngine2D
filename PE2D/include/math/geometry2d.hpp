#pragma once
#include "vector2d.hpp"
#include <vector>
#define PI 3.14159265358979323846f
#include <iostream>
#include "math/matrix3x3.hpp"
namespace PE2D {
	enum class ShapeType
	{
		TRIANGLE,
		POLYGON,
		RECTANGLE,
		CIRCLE,
		CAPSULE,
		UNKNOWN
	};

	// 虚拟基类 Shape
	class Shape {
	public:
		virtual float area() const = 0;
		virtual ~Shape() = default;
		ShapeType type() const { return shapeType; }
		bool isTriangle() const { return shapeType == ShapeType::TRIANGLE; }
		bool isPolygon() const { return shapeType == ShapeType::POLYGON; }
		bool isRectangle() const { return shapeType == ShapeType::RECTANGLE; }
		bool isCircle() const { return shapeType == ShapeType::CIRCLE; }
		bool isCapsule() const { return shapeType == ShapeType::CAPSULE; }
		bool isUnknown() const { return shapeType == ShapeType::UNKNOWN; }
		virtual Vector2D getCentroid() const { return Vector2D(0.0f, 0.0f); } // 默认实现返回原点
		virtual const std::vector<Vector2D> getVertices()const { return std::vector<Vector2D>(); };
		virtual void setPosition(Vector2D v) = 0;
		virtual Vector2D getPosition() { return pos; }
		virtual void rotate(const Matrix3x3& mat) {};
	protected:
		Vector2D pos = Vector2D(0, 0);
		ShapeType shapeType = ShapeType::UNKNOWN;
	};

	// 三角形类，继承自 Shape
	class Triangle : public Shape {
	public:
		Triangle(const Vector2D& p1, const Vector2D& p2, const Vector2D& p3);
		float area() const override;
		const std::vector<Vector2D> getVertices() const { return points; }
		Vector2D getCentroid() {
			return (points[0] + points[1] + points[2]) / 3;
		}
		void setPosition(Vector2D v) {
			Vector2D dV = v - pos;
			pos = v;
			for (auto& i : points)
				i += dV;
		}
		void rotate(const Matrix3x3 mat) {
			for (auto& i : points) {
				i = mat * i;
			}
		};
	private:
		std::vector<Vector2D> points;
	};

	// 多边形类，继承自 Shape, 允许任意数量的顶点,至少3个
	// 顶点顺序应为逆时针或顺时针
	class Polygon : public Shape {
	public:
		Polygon(const std::vector<Vector2D>& vertices);
		Polygon(const std::vector<Vector2D>&& vertices);
		float area() const override;
		const std::vector<Vector2D> getVertices() const { return vertices; }
		Vector2D getCentroid() {
			if (vertices.size() < 3) {
				std::cerr << "多边形最少得包含3个顶点" << std::endl;
				return Vector2D(0.0f, 0.0f);
			}
			float area = 0.0f;
			float cx = 0.0f;
			float cy = 0.0f;
			for (size_t i = 0; i < vertices.size(); ++i) {
				size_t j = (i + 1) % vertices.size(); // 下一个顶点的索引
				float crossProduct = vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y();
				area += crossProduct;
				cx += (vertices[i].x() + vertices[j].x()) * crossProduct;
				cy += (vertices[i].y() + vertices[j].y()) * crossProduct;
			}
			area *= 0.5f;
			if (area == 0.0f) {
				std::cerr << "多边形面积为0，没有质心" << std::endl;
				return Vector2D(0.0f, 0.0f);
			}
			cx /= (6.0f * area);
			cy /= (6.0f * area);
			return Vector2D(cx, cy);
		}
		void setPosition(Vector2D v) {
			Vector2D dV = v - pos;
			pos = v;
			for (auto& i : vertices)
				i += dV;
		}
		void rotate(const Matrix3x3 mat) {
			for (auto& i : vertices) {
				i = mat * i;
			}
		}
	private:
		std::vector<Vector2D> vertices;
	};

	// 矩形类，继承自 Shape
	class Rectangle : public Shape {
	public:
		Rectangle(const Vector2D& topLeft, const Vector2D& bottomRight);
		float area() const override;
		Vector2D getTopLeft() const { return topLeft; }
		Vector2D getBottomRight() const { return bottomRight; }
		const std::vector<Vector2D> getVertices() const {
			std::vector<Vector2D> v;
			v.push_back(Vector2D(topLeft.x(), bottomRight.y()));
			v.push_back(topLeft);
			v.push_back(Vector2D(bottomRight.x(), topLeft.y()));
			v.push_back(bottomRight);
			return v;
		}
		Vector2D getCentroid() const {
			return Vector2D((topLeft.x() + bottomRight.x()) / 2, (topLeft.y() + bottomRight.y()) / 2);
		}
		void setPosition(Vector2D v) {
			Vector2D dV = v - pos;
			pos = v;
			topLeft += dV;
			bottomRight += dV;
		}
		void rotate(const Matrix3x3 mat) {
			topLeft = mat * topLeft;
			bottomRight = mat * bottomRight;
		}
	private:
		Vector2D topLeft;
		Vector2D bottomRight;
	};

	// 圆形类，继承自 Shape
	class Circle : public Shape {
	public:
		Circle(const Vector2D& center, float radius);
		Vector2D getCenter() const { return center; }
		float getRadius() const { return radius; }
		void setRadius(float r) { radius = r; }
		float area() const override;
		Vector2D getCentroid() const { return center; }
		void setPosition(Vector2D v) {
			Vector2D dV = v - pos;
			pos = v;
			center = pos;
		}
		void rotate(const Matrix3x3 mat) {}
	private:
		Vector2D center;
		float radius;
	};
	//胶囊类，继承自 Shape
	class Capsule : public Shape {
	public:
		Capsule(const Vector2D& center, float radius, float height);
		Vector2D getCenter() const { return center; }
		float getRadius() const { return radius; }
		float getHeight() const { return height; }
		float area() const override;
		Vector2D getCentroid() const {
			return Vector2D(center.x(), center.y() + height / 2);
		}
		void setPosition(Vector2D v) {
			Vector2D dV = v - pos;
			pos = v;
			center = pos;
		}
		void rotate(const Matrix3x3 mat) {}
	private:
		Vector2D center;
		float radius;
		float height;
	};
}