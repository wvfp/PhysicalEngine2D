#include "types/common_types.hpp"
#include "entities/rigidbody.hpp"
#include "entities/softbody.hpp"
#include "entities/collider.hpp"
#include "constraints/constraint_types.hpp"
#include "constraints/AngleConstraint/AngleConstraint.hpp"
#include "constraints/CollisionConstraint/CollisionConstraint.hpp"
#include "constraints/ConnectionConstraint/ConnectionConstraint.hpp"
#include "constraints/PositionConstraint/PositionConstraint.hpp"
#include "constraints/constraint_types.hpp"
#include "collision_system/detection/aabb_detection.hpp"
#include "collision_system/detection/vertix_detection.hpp"
#include "collision_system/resolution/penetration_resolution.hpp"
#include "collision_system/resolution/impulse_resolution.hpp"
#include "utils/timer.hpp"
#include <vector>

namespace PE2D {
	class World {
	public:
		//g=9.80,time_step=0.2
		World();
		World(Vector2D gravity, float time_step);
		virtual ~World();
		//更新世界状态
		virtual void update();
		//添加物体
		void addObject(unsigned int id);
		//移除物体
		void removeObject(unsigned int id);
		//添加约束
		void addConstraint(unsigned int id);
		//移除约束
		void removeConstraint(unsigned int id);
	private:
		//工具函数，用于碰撞
		void collisionHandle();
	private:
		//time-step 时间步长
		float m_time_step;
		Timer timer;
		//重力
		Vector2D m_gravity;
		//世界管理的物体的ID
		std::vector<unsigned int> m_Object_IDs;
		//世界管理的约束的ID
		std::vector<unsigned int> m_Constraint_IDs;
	};

}
