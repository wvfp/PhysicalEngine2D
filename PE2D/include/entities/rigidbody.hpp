#pragma once
#include "types/common_types.hpp"

namespace PE2D {
	class RigidBody : virtual public Object {
	private:
		// ��ֹ�û�ֱ��ʹ��Ĭ�Ϲ��캯��
		RigidBody();
	public:
		// Factory method to create a new RigidBody instance
		// Returns the ID of the created RigidBody
		//�������������µĸ���ʵ�������ش����ĸ����ID
		static unsigned int makeRigidBody();
		void init() override;
		void reset() override { init(); }

		void update(float deltaTime) override;
	};
}