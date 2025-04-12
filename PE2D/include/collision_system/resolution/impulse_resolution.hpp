#pragma once
#include "math/vector2d.hpp"
#include "collision_system/collision_types.hpp"
#include "types/common_types.hpp"

namespace PE2D {
	class ImpulseResolution {
	public:
		// Ӧ�ó���
		static void ApplyImpulse(Object& bodyA, Object& bodyB, const CollisionInfo& collisionInfo);
	private:
		// ����ָ�ϵ���� restitution ��
		static float CalculateRestitution(const Object& bodyA, const Object& bodyB);
		// ����Ħ����
		static float CalculateFriction(const Object& bodyA, const Object& bodyB);
	};
}