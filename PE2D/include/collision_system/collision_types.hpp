#pragma once
#include <math/vector2d.hpp>
#include <math/geometry2d.hpp>
#include <stdexcept>
//AABB 检测
namespace PE2D {
	// collision_detector.hpp
	struct CollisionInfo {
		bool isColliding = false;
		Vector2D normal;          // 碰撞法向量（从a指向b）
		float penetration = 0.0f; // 穿透深度
		Vector2D contactPoint;    // 接触点（可选）
	};
	class AABB {
	public:
		// 构造函数，使用左上角和右下角的点来初始化 AABB
		AABB(const Vector2D& topLeft, const Vector2D& bottomRight)
			: m_topLeft(topLeft), m_bottomRight(bottomRight) {
		}
		AABB() :m_topLeft(0), m_bottomRight(0) {};
		AABB(const Shape* shap) {
			if (shap->isCircle()) {
				const Circle* circle = static_cast<const Circle*>(shap);
				m_topLeft = Vector2D(circle->getCenter().x() - circle->getRadius(), circle->getCenter().y() - circle->getRadius());
				m_bottomRight = Vector2D(circle->getCenter().x() + circle->getRadius(), circle->getCenter().y() + circle->getRadius());
			}
			else if (shap->isRectangle()) {
				const Rectangle* rect = static_cast<const Rectangle*>(shap);
				m_topLeft = rect->getTopLeft();
				m_bottomRight = rect->getBottomRight();
			}
			else if (shap->isPolygon()) {
				const Polygon* polygon = static_cast<const Polygon*>(shap);
				//根据多边形的顶点计算 AABB，顺时针或逆时针遍历多边形的顶点
				const std::vector<Vector2D> vertices = polygon->getVertices();
				float minX = vertices[0].x();
				float maxX = vertices[0].x();
				float minY = vertices[0].y();
				float maxY = vertices[0].y();
				for (const Vector2D& vertex : vertices) {
					if (vertex.x() < minX) {
						minX = vertex.x();
					}
					if (vertex.x() > maxX) {
						maxX = vertex.x();
					}
					if (vertex.y() < minY) {
						minY = vertex.y();
					}
					if (vertex.y() > maxY) {
						maxY = vertex.y();
					}
				}
				m_topLeft = Vector2D(minX, minY);
				m_bottomRight = Vector2D(maxX, maxY);
			}
			else if (shap->isTriangle()) {
				const Triangle* triangle = static_cast<const Triangle*>(shap);
				//根据三角形的顶点计算 AABB，顺时针或逆时针遍历三角形的顶点
				const std::vector<Vector2D> points = triangle->getVertices();
				float minX = points[0].x();
				float maxX = points[0].x();
				float minY = points[0].y();
				float maxY = points[0].y();
				for (const Vector2D& point : points) {
					if (point.x() < minX) {
						minX = point.x();
					}
					if (point.x() > maxX) {
						maxX = point.x();
					}
					if (point.y() < minY) {
						minY = point.y();
					}
					if (point.y() > maxY) {
						maxY = point.y();
					}
				}
				m_topLeft = Vector2D(minX, minY);
				m_bottomRight = Vector2D(maxX, maxY);
			}
			else if (shap->isCapsule()) {
				const Capsule* capsule = static_cast<const Capsule*>(shap);
				//根据胶囊体的中心点和半径计算 AABB
				float minX = capsule->getCenter().x() - capsule->getRadius();
				float maxX = capsule->getCenter().x() + capsule->getRadius();
				float minY = capsule->getCenter().y() - capsule->getHeight() / 2;
				float maxY = capsule->getCenter().y() + capsule->getHeight() / 2;
				m_topLeft = Vector2D(minX, minY);
				m_bottomRight = Vector2D(maxX, maxY);
			}
			else {
				throw std::invalid_argument("Unsupported shape type for AABB.");
			}
		}
		// 计算 AABB
		static AABB calculateAABB(Shape* shape) {
			AABB a=AABB(shape);
			return a;
		}
		void setObj_ID(unsigned int id) {
			Obj_ID = id;
		}
		unsigned int getObj_ID() const {
			return Obj_ID;
		}
		// 获取左上角的点
		Vector2D getTopLeft() const {
			return m_topLeft;
		}
		void setTopLeft(Vector2D tl) { m_topLeft = tl; }
		// 获取右下角的点
		Vector2D getBottomRight() const {
			return m_bottomRight;
		}
		void setButtomRight(Vector2D br) { m_bottomRight = br; }
		// 获取 AABB 的宽度
		float getWidth() const {
			return m_bottomRight.x() - m_topLeft.x();
		}

		// 获取 AABB 的高度
		float getHeight() const {
			return m_bottomRight.y() - m_topLeft.y();
		}

		// 获取 AABB 的中心
		Vector2D getCenter() const {
			return Vector2D((m_topLeft.x() + m_bottomRight.x()) / 2, (m_topLeft.y() + m_bottomRight.y()) / 2);
		}

		// 检查一个点是否在 AABB 内
		bool isPointInside(const Vector2D& point) const {
			return (point.x() >= m_topLeft.x() && point.x() <= m_bottomRight.x() &&
				point.y() >= m_topLeft.y() && point.y() <= m_bottomRight.y());
		}

		// 检查两个 AABB 是否相交
		bool isIntersecting(const AABB& other) const {
			return !(other.m_bottomRight.x() < m_topLeft.x() ||
				other.m_topLeft.x() > m_bottomRight.x() ||
				other.m_bottomRight.y() < m_topLeft.y() ||
				other.m_topLeft.y() > m_bottomRight.y());
		}
		//基于分离轴算法的AABB碰撞检测

		// 扩展 AABB 以包含某个点
		void expandToInclude(const Vector2D& point) {
			if (point.x() < m_topLeft.x()) m_topLeft.setX(point.x());
			if (point.y() < m_topLeft.y()) m_topLeft.setY(point.y());
			if (point.x() > m_bottomRight.x()) m_bottomRight.setX(point.x());
			if (point.y() > m_bottomRight.y()) m_bottomRight.setY(point.y());
		}
		// 批量碰撞检测
		std::vector<std::pair<unsigned int, unsigned int>>
			batchAABBIntersection(const std::vector<AABB>& aabbs) {
			std::vector<std::pair<unsigned int, unsigned int>> intersections;
			int n = aabbs.size();
			for (int i = 0; i < n; ++i) {
				for (int j = i + 1; j < n; ++j) {
					if (aabbs[i].isIntersecting(aabbs[j])) {
						intersections.push_back(std::make_pair(aabbs[i].getObj_ID(),
							aabbs[j].getObj_ID()));
					}
				}
			}
			return intersections;
		}
		// 基于分离轴算法的两个AABB碰撞检测
		bool isAABBIntersect(const AABB& aabb1, const AABB& aabb2) {
			// 检查X轴上的投影是否重叠
			bool overlapX = (aabb1.getBottomRight().x() >= aabb2.getTopLeft().x()) && (aabb2.getBottomRight().x() >= aabb1.getTopLeft().x());
			// 检查Y轴上的投影是否重叠
			bool overlapY = (aabb1.getBottomRight().y() >= aabb2.getTopLeft().y()) && (aabb2.getBottomRight().y() >= aabb1.getTopLeft().y());

			// 只有当X轴和Y轴上的投影都重叠时，两个AABB才相交
			return overlapX && overlapY;
		}

		// 多个AABB的碰撞检测
		std::vector<std::pair<unsigned int, unsigned int>>
			areMultipleAABBsIntersecting(const std::vector<AABB>& aabbs) {
			std::vector<std::pair<unsigned int, unsigned int>> result;
			size_t numAABBs = aabbs.size();
			for (size_t i = 0; i < numAABBs; ++i) {
				for (size_t j = i + 1; j < numAABBs; ++j) {
					if (isAABBIntersect(aabbs[i], aabbs[j])) {
						result.push_back(std::make_pair(aabbs[i].getObj_ID(), aabbs[j].getObj_ID()));
					}
				}
			}
			return result;
		}
	private:
		Vector2D m_topLeft;
		Vector2D m_bottomRight;
		unsigned int Obj_ID=0;
	};
}