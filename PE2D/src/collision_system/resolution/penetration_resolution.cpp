#include "collision_system/resolution/penetration_resolution.hpp"
#include "types/common_types.hpp"
#include "math/vector2d.hpp"

namespace PE2D {
	void PenetrationResolution::ResolvePenetration(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo) {
		if (!collisionInfo.isColliding) {
			return; // 如果没有碰撞，直接返回
		}

		// 计算分离向量
		Vector2D separationVector = collisionInfo.normal * collisionInfo.penetration;

		// 计算质量比
		float totalMass = bodyA.getMass() + bodyB.getMass();
		float massRatioA = bodyB.getMass() / totalMass;
		float massRatioB = bodyA.getMass() / totalMass;

		// 移动刚体以解决穿透问题
		bodyA.setPosition(bodyA.getPosition() - separationVector * massRatioA);
		bodyB.setPosition(bodyB.getPosition() + separationVector * massRatioB);
	}
}