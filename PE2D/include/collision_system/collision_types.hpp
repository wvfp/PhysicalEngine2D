#pragma once
#include <math/vector2d.hpp>
#include <math/geometry2d.hpp>
#include <stdexcept>
//AABB ���
namespace PE2D {
	// collision_detector.hpp
	struct CollisionInfo {
		bool isColliding = false;
		Vector2D normal;          // ��ײ����������aָ��b��
		float penetration = 0.0f; // ��͸���
		Vector2D contactPoint;    // �Ӵ��㣨��ѡ��
	};
	class AABB {
	public:
		// ���캯����ʹ�����ϽǺ����½ǵĵ�����ʼ�� AABB
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
				//���ݶ���εĶ������ AABB��˳ʱ�����ʱ���������εĶ���
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
				//���������εĶ������ AABB��˳ʱ�����ʱ����������εĶ���
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
				//���ݽ���������ĵ�Ͱ뾶���� AABB
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
		// ���� AABB
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
		// ��ȡ���Ͻǵĵ�
		Vector2D getTopLeft() const {
			return m_topLeft;
		}
		void setTopLeft(Vector2D tl) { m_topLeft = tl; }
		// ��ȡ���½ǵĵ�
		Vector2D getBottomRight() const {
			return m_bottomRight;
		}
		void setButtomRight(Vector2D br) { m_bottomRight = br; }
		// ��ȡ AABB �Ŀ��
		float getWidth() const {
			return m_bottomRight.x() - m_topLeft.x();
		}

		// ��ȡ AABB �ĸ߶�
		float getHeight() const {
			return m_bottomRight.y() - m_topLeft.y();
		}

		// ��ȡ AABB ������
		Vector2D getCenter() const {
			return Vector2D((m_topLeft.x() + m_bottomRight.x()) / 2, (m_topLeft.y() + m_bottomRight.y()) / 2);
		}

		// ���һ�����Ƿ��� AABB ��
		bool isPointInside(const Vector2D& point) const {
			return (point.x() >= m_topLeft.x() && point.x() <= m_bottomRight.x() &&
				point.y() >= m_topLeft.y() && point.y() <= m_bottomRight.y());
		}

		// ������� AABB �Ƿ��ཻ
		bool isIntersecting(const AABB& other) const {
			return !(other.m_bottomRight.x() < m_topLeft.x() ||
				other.m_topLeft.x() > m_bottomRight.x() ||
				other.m_bottomRight.y() < m_topLeft.y() ||
				other.m_topLeft.y() > m_bottomRight.y());
		}
		//���ڷ������㷨��AABB��ײ���

		// ��չ AABB �԰���ĳ����
		void expandToInclude(const Vector2D& point) {
			if (point.x() < m_topLeft.x()) m_topLeft.setX(point.x());
			if (point.y() < m_topLeft.y()) m_topLeft.setY(point.y());
			if (point.x() > m_bottomRight.x()) m_bottomRight.setX(point.x());
			if (point.y() > m_bottomRight.y()) m_bottomRight.setY(point.y());
		}
		// ������ײ���
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
		// ���ڷ������㷨������AABB��ײ���
		bool isAABBIntersect(const AABB& aabb1, const AABB& aabb2) {
			// ���X���ϵ�ͶӰ�Ƿ��ص�
			bool overlapX = (aabb1.getBottomRight().x() >= aabb2.getTopLeft().x()) && (aabb2.getBottomRight().x() >= aabb1.getTopLeft().x());
			// ���Y���ϵ�ͶӰ�Ƿ��ص�
			bool overlapY = (aabb1.getBottomRight().y() >= aabb2.getTopLeft().y()) && (aabb2.getBottomRight().y() >= aabb1.getTopLeft().y());

			// ֻ�е�X���Y���ϵ�ͶӰ���ص�ʱ������AABB���ཻ
			return overlapX && overlapY;
		}

		// ���AABB����ײ���
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