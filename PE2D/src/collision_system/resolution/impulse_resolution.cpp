#include "collision_system/resolution/impulse_resolution.hpp"
#include "types/common_types.hpp"
#include "math/vector2d.hpp"
#include "collision_system/collision_types.hpp"

namespace PE2D {
	void ImpulseResolution::ApplyImpulse(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo) {
		if (!collisionInfo.isColliding) {
			return; // 如果没有碰撞，直接返回
		}

		Vector2D normal = collisionInfo.normal;
		Vector2D contactPoint = collisionInfo.contactPoint;

		// 计算相对速度
		Vector2D relativeVelocity = bodyB.getVelocity() - bodyA.getVelocity();

		// 计算法向相对速度
		float velocityAlongNormal = relativeVelocity.dot(normal);

		// 如果物体已经分离或没有相对速度，则不需要应用冲量
		if (velocityAlongNormal > 0) {
			return;
		}

		// 计算恢复系数（ restitution ）
		float restitution = CalculateRestitution(bodyA, bodyB);

		// 计算冲量的大小
		float numerator = -(1 + restitution) * velocityAlongNormal;
		float denominator = bodyA.getInvMass() + bodyB.getInvMass();
		float impulseMagnitude = numerator / denominator;

		// 计算冲量
		Vector2D impulse = normal * impulseMagnitude;

		// 应用冲量
		bodyA.applyImpulse(-impulse, contactPoint);
		bodyB.applyImpulse(impulse, contactPoint);

		// 计算摩擦力
		float friction = CalculateFriction(bodyA, bodyB);
		Vector2D tangent = relativeVelocity - normal * velocityAlongNormal;
		tangent.normalize();

		float tangentImpulseMagnitude = -relativeVelocity.dot(tangent) / denominator;
		Vector2D tangentImpulse = tangent * tangentImpulseMagnitude;

		// 应用摩擦冲量
		if (std::abs(tangentImpulseMagnitude) < impulseMagnitude * friction) {
			bodyA.applyImpulse(-tangentImpulse, contactPoint);
			bodyB.applyImpulse(tangentImpulse, contactPoint);
		}
		else {
			bodyA.applyImpulse(-tangent * impulseMagnitude * friction, contactPoint);
			bodyB.applyImpulse(tangent * impulseMagnitude * friction, contactPoint);
		}
	}

	float ImpulseResolution::CalculateRestitution(const Object& bodyA, const Object& bodyB) {
		// 返回两个刚体中较大的恢复系数
		return std::max(bodyA.getRestitution(), bodyB.getRestitution());
	}

	float ImpulseResolution::CalculateFriction(const Object& bodyA, const Object& bodyB) {
		// 返回两个刚体摩擦力的几何平均值
		return std::sqrt(bodyA.getFriction() * bodyB.getFriction());
	}
}