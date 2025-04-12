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
		//��������״̬
		virtual void update();
		//�������
		void addObject(unsigned int id);
		//�Ƴ�����
		void removeObject(unsigned int id);
		//���Լ��
		void addConstraint(unsigned int id);
		//�Ƴ�Լ��
		void removeConstraint(unsigned int id);
	private:
		//���ߺ�����������ײ
		void collisionHandle();
	private:
		//time-step ʱ�䲽��
		float m_time_step;
		Timer timer;
		//����
		Vector2D m_gravity;
		//�������������ID
		std::vector<unsigned int> m_Object_IDs;
		//��������Լ����ID
		std::vector<unsigned int> m_Constraint_IDs;
	};

}
