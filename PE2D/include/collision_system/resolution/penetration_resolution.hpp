#pragma once
#include "math/vector2d.hpp"
#include "types/common_types.hpp"
#include "collision_system/collision_types.hpp"

namespace PE2D {
	class PenetrationResolution {
	public:
		// 解决穿透问题
		static void ResolvePenetration(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo);
	};
}