#include "collision_system/resolution/impulse_resolution.hpp"
#include "types/common_types.hpp"
#include "math/vector2d.hpp"
#include "collision_system/collision_types.hpp"

namespace PE2D {
	void ImpulseResolution::ApplyImpulse(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo) {
		if (!collisionInfo.isColliding) {
			return; // ���û����ײ��ֱ�ӷ���
		}

		Vector2D normal = collisionInfo.normal;
		Vector2D contactPoint = collisionInfo.contactPoint;

		// ��������ٶ�
		Vector2D relativeVelocity = bodyB.getVelocity() - bodyA.getVelocity();

		// ���㷨������ٶ�
		float velocityAlongNormal = relativeVelocity.dot(normal);

		// ��������Ѿ������û������ٶȣ�����ҪӦ�ó���
		if (velocityAlongNormal > 0) {
			return;
		}

		// ����ָ�ϵ���� restitution ��
		float restitution = CalculateRestitution(bodyA, bodyB);

		// ��������Ĵ�С
		float numerator = -(1 + restitution) * velocityAlongNormal;
		float denominator = bodyA.getInvMass() + bodyB.getInvMass();
		float impulseMagnitude = numerator / denominator;

		// �������
		Vector2D impulse = normal * impulseMagnitude;

		// Ӧ�ó���
		bodyA.applyImpulse(-impulse, contactPoint);
		bodyB.applyImpulse(impulse, contactPoint);

		// ����Ħ����
		float friction = CalculateFriction(bodyA, bodyB);
		Vector2D tangent = relativeVelocity - normal * velocityAlongNormal;
		tangent.normalize();

		float tangentImpulseMagnitude = -relativeVelocity.dot(tangent) / denominator;
		Vector2D tangentImpulse = tangent * tangentImpulseMagnitude;

		// Ӧ��Ħ������
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
		// �������������нϴ�Ļָ�ϵ��
		return std::max(bodyA.getRestitution(), bodyB.getRestitution());
	}

	float ImpulseResolution::CalculateFriction(const Object& bodyA, const Object& bodyB) {
		// ������������Ħ�����ļ���ƽ��ֵ
		return std::sqrt(bodyA.getFriction() * bodyB.getFriction());
	}
}