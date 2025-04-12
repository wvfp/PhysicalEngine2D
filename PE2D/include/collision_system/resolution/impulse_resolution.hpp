#pragma once
#include "math/vector2d.hpp"
#include "collision_system/collision_types.hpp"
#include "types/common_types.hpp"

namespace PE2D {
	class ImpulseResolution {
	public:
		// 应用冲量
		static void ApplyImpulse(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo);
	private:
		// 计算恢复系数（ restitution ）
		static float CalculateRestitution(const Object& bodyA, const Object& bodyB);
		// 计算摩擦力
		static float CalculateFriction(const Object& bodyA, const Object& bodyB);
	};
}