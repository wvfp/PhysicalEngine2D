#include "collision_system/detection/vertix_detection.hpp"
#include "math/geometry2d.hpp"
#include "math/vector2d.hpp"
#include "math/math_utils.hpp"
#include <limits>
#include <algorithm>

namespace PE2D {
	// ������������
	static float distanceBetweenPoints(const Vector2D& point, const std::vector<Vector2D>& polygon);
	static Vector2D GetEdgeNormal(const Vector2D& current, const Vector2D& next);
	static Vector2D FindClosestPointOnSegment(const Vector2D& point,
		const Vector2D& segA,
		const Vector2D& segB);
	static bool ClipSegmentToLine(const std::vector<Vector2D>& contactsIn,
		std::vector<Vector2D>& contactsOut,
		const Vector2D& c0,
		const Vector2D& c1);
	// �����Ƿ��ڶ������
	static bool IsPointInsidePolygon(const Vector2D& point, const std::vector<Vector2D>& polygon);
	// ��ȡ�㵽����ε�����ߵķ�����
	static Vector2D GetClosestEdgeNormal(const Vector2D& point, const std::vector<Vector2D>& polygon);
	// ʵ�ֲ���
	bool CollisionDetector::FindAxisLeastPenetration(
		const std::vector<Vector2D>& aVerts,
		const std::vector<Vector2D>& bVerts,
		Vector2D& normal,
		float& penetration)
	{
		penetration = -std::numeric_limits<float>::max();
		size_t normalIndex = 0;

		// �������A�����б߷���
		for (size_t i = 0; i < aVerts.size(); ++i) {
			size_t j = (i + 1) % aVerts.size();
			Vector2D edge = aVerts[j] - aVerts[i];
			Vector2D axis(-edge.y(), edge.x()); // ���߷���
			axis = axis.normalize();

			// ͶӰ�����B����ǰ��
			float minB = axis.dot(bVerts[0]);
			float maxB = minB;
			for (const auto& vert : bVerts) {
				float proj = axis.dot(vert);
				minB = std::min(minB, proj);
				maxB = std::max(maxB, proj);
			}

			// ͶӰ�����A����ǰ��
			float minA = axis.dot(aVerts[0]);
			float maxA = minA;
			for (const auto& vert : aVerts) {
				float proj = axis.dot(vert);
				minA = std::min(minA, proj);
				maxA = std::max(maxA, proj);
			}

			// �����ص�
			float overlap = std::min(maxA, maxB) - std::max(minA, minB);
			if (overlap < 0) return false; // ���������

			// ��¼��С��͸
			if (overlap < penetration) {
				penetration = overlap;
				normal = axis;
				// ȷ�����߷����Aָ��B
				Vector2D centroidA = calculateCentroid(aVerts);
				Vector2D centroidB = calculateCentroid(bVerts);
				Vector2D dir = centroidB - centroidA;
				if (dir.dot(normal) < 0)
					normal = -normal;
			}
		}
		return true;
	}
	// �����ײ
	std::optional<CollisionInfo> CollisionDetector::CheckCollision(const Shape& a, const Shape& b) {
		return Dispatch(a, b);
	}
	// ��״���ͷַ�
	std::optional<CollisionInfo> CollisionDetector::Dispatch(const Shape& a, const Shape& b) {
		if (a.isCircle() && b.isCircle()) {
			return CircleVsCircle(static_cast<const Circle&>(a), static_cast<const Circle&>(b));
		}
		else if (a.isCircle() && b.isPolygon()) {
			return CircleVsPolygon(static_cast<const Circle&>(a), static_cast<const Polygon&>(b));
		}
		else if (a.isPolygon() && b.isCircle()) {
			return PolygonVsCircle(static_cast<const Polygon&>(a), static_cast<const Circle&>(b));
		}
		else if (a.isPolygon() && b.isPolygon()) {
			return PolygonVsPolygon(static_cast<const Polygon&>(a), static_cast<const Polygon&>(b));
		}
		else if (b.isCapsule() && a.isPolygon()) {
			return CapsuleVsPolygon(static_cast<const Capsule&>(a), static_cast<const Polygon&>(b));
		}
		else if (a.isPolygon() && b.isCapsule()) {
			return CapsuleVsPolygon(static_cast<const Capsule&>(b), static_cast<const Polygon&>(a));
		}
		return std::nullopt; // δʵ�ֵ���ײ����
	}

	// Բ��Բ����ײ���
	std::optional<CollisionInfo> CollisionDetector::CircleVsCircle(const Circle& a, const Circle& b) {
		Vector2D normal = b.getCenter() - a.getCenter();
		float distance = normal.magnitude();
		float radiusSum = a.getRadius() + b.getRadius();
		if (distance >= radiusSum) {
			return std::nullopt; // û����ײ
		}
		CollisionInfo info;
		info.isColliding = true;
		info.normal = normal.normalize();
		info.penetration = radiusSum - distance;
		info.contactPoint = a.getCenter() + info.normal * a.getRadius();
		return info;
	}
	// Բ�����ε���ײ���
	std::optional<CollisionInfo> CollisionDetector::CircleVsPolygon(const Circle& circle, const Polygon& polygon) {
		CollisionInfo info;
		const auto& vertices = polygon.getVertices();
		Vector2D center = circle.getCenter();
		float radius = circle.getRadius();
		Vector2D normal;
		float penetration;
		FindAxisLeastPenetration(vertices, { center }, normal, penetration);
		if (penetration < 0) {
			return std::nullopt; // û����ײ
		}
		info.isColliding = true;
		info.normal = normal;
		info.penetration = penetration;
		info.contactPoint = center + normal * (radius - penetration);
		return info;
	}
	// �������Բ����ײ���
	std::optional<CollisionInfo> CollisionDetector::PolygonVsCircle(const Polygon& polygon, const Circle& circle) {
		CollisionInfo info;
		const auto& vertices = polygon.getVertices();
		Vector2D center = circle.getCenter();
		float radius = circle.getRadius();
		Vector2D normal;
		float penetration;
		FindAxisLeastPenetration(vertices, { center }, normal, penetration);
		if (penetration < 0) {
			return std::nullopt; // û����ײ
		}
		info.isColliding = true;
		info.normal = normal;
		info.penetration = penetration;
		info.contactPoint = center + normal * (radius - penetration);
		return info;
	}
	// ����������ε���ײ���
	std::optional<CollisionInfo> CollisionDetector::PolygonVsPolygon(const Polygon& a, const Polygon& b) {
		CollisionInfo info;
		const auto& verticesA = a.getVertices();
		const auto& verticesB = b.getVertices();
		Vector2D normal;
		float penetration;
		FindAxisLeastPenetration(verticesA, verticesB, normal, penetration);
		if (penetration < 0) {
			return std::nullopt; // û����ײ
		}
		Vector2D normalB = -normal;
		float penetrationB;
		FindAxisLeastPenetration(verticesB, verticesA, normalB, penetrationB);
		if (penetrationB < 0) {
			return std::nullopt; // û����ײ
		}
		if (penetrationB > penetration) {
			penetration = penetrationB;
			normal = normalB;
		}
		info.isColliding = true;
		info.normal = normal;
		info.penetration = penetration;
		FindContactPoints(verticesA, verticesB, info);
		return info;
	}
	// �����������ε���ײ���
	std::optional<CollisionInfo> CollisionDetector::CapsuleVsPolygon(const Capsule& capsule, const Polygon& polygon) {
		CollisionInfo info;
		const auto& vertices = polygon.getVertices();
		Vector2D center = capsule.getCenter();
		float height = capsule.getHeight();
		float radius = capsule.getRadius();
		// ���㽺����������˵�
		Vector2D halfHeight = Vector2D(0, height / 2);
		Vector2D top = center + halfHeight;
		Vector2D bottom = center - halfHeight;
		Vector2D normal;
		float penetration;
		FindAxisLeastPenetration(vertices, { top, bottom }, normal, penetration);
		if (penetration < 0) {
			return std::nullopt; // û����ײ
		}
		info.isColliding = true;
		info.normal = normal;
		info.penetration = penetration;
		info.contactPoint = center + normal * (radius - penetration);
		return info;
	}

	// �Ӵ�����㣨�����vs����Σ�
	static void FindContactPoints(
		const std::vector<Vector2D>& vertsA,
		const std::vector<Vector2D>& vertsB,
		CollisionInfo& info)
	{
		std::vector<Vector2D> contacts;

		// ���A�Ķ����Ƿ���B�ڲ�
		for (const auto& v : vertsA) {
			if (isPointInPolygon(v, vertsB)) {
				contacts.push_back(v);
			}
		}

		// ���B�Ķ����Ƿ���A�ڲ�
		for (const auto& v : vertsB) {
			if (isPointInPolygon(v, vertsA)) {
				contacts.push_back(v);
			}
		}

		// �����ཻ
		for (size_t i = 0; i < vertsA.size(); ++i) {
			Vector2D a1 = vertsA[i];
			Vector2D a2 = vertsA[(i + 1) % vertsA.size()];
			for (size_t j = 0; j < vertsB.size(); ++j) {
				Vector2D b1 = vertsB[j];
				Vector2D b2 = vertsB[(j + 1) % vertsB.size()];

				if (isSegmentIntersect(a1, a2, b1, b2)) {
					Vector2D aDir = a2 - a1;
					Vector2D bDir = b2 - b1;
					Vector2D contact;
					bool found = false;

					const float epsilon = 1e-6f;
					float cross = aDir.cross(bDir);

					if (std::abs(cross) < epsilon) { // ƽ�����
						// ���߼��
						if (std::abs((b1 - a1).cross(aDir)) < epsilon) {
							// ����������
							float t0 = (b1 - a1).dot(aDir) / aDir.dot(aDir);
							float t1 = (b2 - a1).dot(aDir) / aDir.dot(aDir);

							float tMin = std::max(0.0f, std::min(t0, t1));
							float tMax = std::min(1.0f, std::max(t0, t1));

							if (tMin <= tMax) {
								contact = a1 + aDir * ((tMin + tMax) * 0.5f);
								found = true;
							}
						}
					}
					else { // ��ƽ�����
						Vector2D cToA = a1 - b1;
						float t = cToA.cross(bDir) / cross;
						float s = cToA.cross(aDir) / cross;

						if (t >= -epsilon && t <= 1.0f + epsilon &&
							s >= -epsilon && s <= 1.0f + epsilon)
						{
							contact = a1 + aDir * std::clamp(t, 0.0f, 1.0f);
							Vector2D verify = b1 + bDir * std::clamp(s, 0.0f, 1.0f);

							if ((contact - verify).magnitudeSq() < epsilon * epsilon) {
								found = true;
							}
						}
					}

					if (found) {
						// ȥ�ؼ��
						bool isUnique = true;
						for (const auto& existing : contacts) {
							if ((contact - existing).magnitudeSq() < epsilon * epsilon) {
								isUnique = false;
								break;
							}
						}
						if (isUnique) {
							contacts.push_back(contact);
						}
					}
				}
			}
		}

		// ��������Ӵ��㣨��ƽ����
		if (!contacts.empty()) {
			info.contactPoint = Vector2D(0, 0);
			//ƽ��
			for (auto& i : contacts) {
				info.contactPoint += i;
			}
			info.contactPoint /= contacts.size();
		}
	}
	// ����������

	// ���������������߶������
	static Vector2D FindClosestPointOnSegment(
		const Vector2D& point,
		const Vector2D& segA,
		const Vector2D& segB)
	{
		Vector2D ab = segB - segA;
		float t = (point - segA).dot(ab) / ab.dot(ab);
		t = clamp(t, 0.0f, 1.0f);
		return segA + ab * t;
	}

	static bool GetIntersection(const Vector2D& a0, const Vector2D& a1, const Vector2D& b0, const Vector2D& b1, Vector2D& intersection) {
		Vector2D r = a1 - a0;
		Vector2D s = b1 - b0;

		double rCrossS = r.x() * s.y() - r.y() * s.x();
		if (rCrossS == 0) {
			return false; // ƽ�л���
		}

		Vector2D q = b0 - a0;
		double t = (q.x() * s.y() - q.y() * s.x()) / rCrossS;
		double u = (q.x() * r.y() - q.y() * r.x()) / rCrossS;

		if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
			intersection = a0 + r * t;
			return true;
		}

		return false;
	}

	// ClipSegmentToLine ����ʵ��
	static bool ClipSegmentToLine(const std::vector<Vector2D>& contactsIn,
		std::vector<Vector2D>& contactsOut,
		const Vector2D& c0,
		const Vector2D& c1) {
		contactsOut.clear();

		for (size_t i = 0; i < contactsIn.size(); ++i) {
			size_t j = (i + 1) % contactsIn.size();
			Vector2D intersection;
			if (GetIntersection(contactsIn[i], contactsIn[j], c0, c1, intersection)) {
				contactsOut.push_back(intersection);
			}
		}

		return !contactsOut.empty();
	}
	// ��ȡ�ߵķ�����
	static Vector2D GetEdgeNormal(const Vector2D& current, const Vector2D& next) {
		Vector2D edge = next - current;
		Vector2D normal = Vector2D(-edge.y(), edge.x()); // ��ʱ����ת90�ȵõ�������
		normal.normalize(); // ��һ��������
		return normal;
	}
	// �����⺯��
	bool VertexDetection(const std::vector<Vector2D>& polygonA, const std::vector<Vector2D>& polygonB, CollisionInfo& collisionInfo) {
		collisionInfo.isColliding = false;
		collisionInfo.normal = Vector2D(0.0f, 0.0f);
		collisionInfo.penetration = std::numeric_limits<float>::max();
		// ��� polygonA ��ÿ�������Ƿ��� polygonB ��
		for (const auto& vertex : polygonA) {
			if (IsPointInsidePolygon(vertex, polygonB)) {
				collisionInfo.isColliding = true;
				collisionInfo.normal = GetClosestEdgeNormal(vertex, polygonB);
				collisionInfo.penetration = distanceBetweenPoints(vertex, polygonB);
				return true;
			}
		}
		// ��� polygonB ��ÿ�������Ƿ��� polygonA ��
		for (const auto& vertex : polygonB) {
			if (IsPointInsidePolygon(vertex, polygonA)) {
				collisionInfo.isColliding = true;
				collisionInfo.normal = GetClosestEdgeNormal(vertex, polygonA);
				collisionInfo.penetration = distanceBetweenPoints(vertex, polygonA);
				return true;
			}
		}
		return false;
	}
	// �����Ƿ��ڶ������
	static bool IsPointInsidePolygon(const Vector2D& point, const std::vector<Vector2D>& polygon) {
		int n = polygon.size();
		bool inside = false;
		for (int i = 0, j = n - 1; i < n; j = i++) {
			const Vector2D& vi = polygon[i];
			const Vector2D& vj = polygon[j];
			if (((vi.y() > point.y()) != (vj.y() > point.y()) &&
				(point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x()))) {
				inside = !inside;
			}
		}
		return inside;
	}
	// ��ȡ�㵽����ε�����ߵķ�����
	static Vector2D GetClosestEdgeNormal(const Vector2D& point, const std::vector<Vector2D>& polygon) {
		Vector2D closestNormal;
		float minDistance = std::numeric_limits<float>::max();
		for (size_t i = 0; i < polygon.size(); ++i) {
			const Vector2D& current = polygon[i];
			const Vector2D& next = polygon[(i + 1) % polygon.size()];
			Vector2D edge = next - current;
			Vector2D normal = GetEdgeNormal(current, next);
			float distance = distanceFromPointToLine(point, current, next);
			if (distance < minDistance) {
				minDistance = distance;
				closestNormal = normal;
			}
		}
		return closestNormal;
	}
	// ����㵽�߶εľ���
	static float distanceFromPointToLine(const Vector2D& point, const Vector2D& lineStart, const Vector2D& lineEnd) {
		Vector2D line = lineEnd - lineStart;
		Vector2D pointToLineStart = point - lineStart;
		float lineLength = line.magnitude();
		if (lineLength == 0.0f) {
			return pointToLineStart.magnitude();
		}
		float projection = pointToLineStart.dot(line) / lineLength;
		projection = std::max(0.0f, std::min(projection, lineLength));
		Vector2D closestPoint = lineStart + line * (projection / lineLength);
		return (point - closestPoint).magnitude();
	}
	// ����㵽����εľ���
	static float distanceBetweenPoints(const Vector2D& point, const std::vector<Vector2D>& polygon) {
		float minDistance = std::numeric_limits<float>::max();
		for (size_t i = 0; i < polygon.size(); ++i) {
			const Vector2D& current = polygon[i];
			const Vector2D& next = polygon[(i + 1) % polygon.size()];
			float distance = distanceFromPointToLine(point, current, next);
			if (distance < minDistance) {
				minDistance = distance;
			}
		}
		return minDistance;
	}
}