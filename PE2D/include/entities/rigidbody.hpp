#pragma once
#include "types/common_types.hpp"

namespace PE2D {
	class RigidBody : virtual public Object {
	private:
		// 禁止用户直接使用默认构造函数
		RigidBody();
	public:
		// Factory method to create a new RigidBody instance
		// Returns the ID of the created RigidBody
		//工厂方法创建新的刚体实例并返回创建的刚体的ID
		static unsigned int makeRigidBody();
		void init() override;
		void reset() override { init(); }

		void update(float deltaTime) override;
	};
}