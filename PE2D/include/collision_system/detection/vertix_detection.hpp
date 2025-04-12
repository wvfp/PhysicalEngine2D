#pragma once
#include <optional>
#include <math/geometry2d.hpp>
#include <math/vector2d.hpp>
#include <collision_system/collision_types.hpp>
//为了获得更精确的碰撞信息，对于多边形和三角形的碰撞检测，使用了更复杂的算法
//先用AABB进行初步检测，如果AABB相交，则使用更复杂的算法进行精确检测

namespace PE2D {
	// 接触点计算（多边形vs多边形）
	static void FindContactPoints(
		const std::vector<Vector2D>& vertsA,
		const std::vector<Vector2D>& vertsB,
		CollisionInfo& info);
	static Vector2D FindClosestPointOnSegment(
		const Vector2D& point,
		const Vector2D& segA,
		const Vector2D& segB);

	bool VertexDetection(const std::vector<Vector2D>& polygonA, const std::vector<Vector2D>& polygonB, CollisionInfo& collisionInfo);
	class CollisionDetector {
	public:
		static std::optional<CollisionInfo> CheckCollision(const Shape& a, const Shape& b);

	private:
		// 形状类型分发
		static std::optional<CollisionInfo> Dispatch(const Shape& a, const Shape& b);

		// 具体检测方法
		static std::optional<CollisionInfo> CircleVsCircle(const Circle& a, const Circle& b);
		static std::optional<CollisionInfo> CircleVsPolygon(const Circle& circle, const Polygon& polygon);
		static std::optional<CollisionInfo> PolygonVsCircle(const Polygon& polygon, const Circle& circle);
		static std::optional<CollisionInfo> PolygonVsPolygon(const Polygon& a, const Polygon& b);
		static std::optional<CollisionInfo> CapsuleVsPolygon(const Capsule& capsule, const Polygon& polygon);

		// SAT 辅助方法
		static bool FindAxisLeastPenetration(const std::vector<Vector2D>& aVertices,
			const std::vector<Vector2D>& bVertices,
			Vector2D& normal,
			float& penetration);
	};
} // namespace PE2D 