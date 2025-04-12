#include "collision_system/resolution/penetration_resolution.hpp"
#include "types/common_types.hpp"
#include "math/vector2d.hpp"

namespace PE2D {
	void PenetrationResolution::ResolvePenetration(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo) {
		if (!collisionInfo.isColliding) {
			return; // ���û����ײ��ֱ�ӷ���
		}

		// �����������
		Vector2D separationVector = collisionInfo.normal * collisionInfo.penetration;

		// ����������
		float totalMass = bodyA.getMass() + bodyB.getMass();
		float massRatioA = bodyB.getMass() / totalMass;
		float massRatioB = bodyA.getMass() / totalMass;

		// �ƶ������Խ����͸����
		bodyA.setPosition(bodyA.getPosition() - separationVector * massRatioA);
		bodyB.setPosition(bodyB.getPosition() + separationVector * massRatioB);
	}
}