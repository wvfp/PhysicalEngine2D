#include "PhysicalEngine2D.hpp"
#include <map>
namespace PE2D {
	//���캯��
	World::World(Vector2D gravity, float time_step) :
		m_gravity(gravity), m_time_step(time_step) {
	}
	World::World() :m_gravity(0, 9.80f), m_time_step(0.02) {}
	World::~World() {}
	void World::addObject(unsigned int id) {
		if (Object::isValidID(id))
			m_Object_IDs.push_back(id);
	}
	void World::removeObject(unsigned int id) {
		if (Object::isValidID(id)) {
			auto newEnd = std::remove(m_Object_IDs.begin(), m_Object_IDs.end(), id);
			m_Object_IDs.erase(newEnd, m_Object_IDs.end());
		}
	}
	void World::addConstraint(unsigned int id) {
		if (Constraint::isValidID(id))
			m_Constraint_IDs.push_back(id);
	}
	void World::removeConstraint(unsigned int id) {
		if (Constraint::isValidID(id)) {
			auto newEnd = std::remove(m_Constraint_IDs.begin(), m_Constraint_IDs.end(), id);
			m_Constraint_IDs.erase(newEnd, m_Constraint_IDs.end());
		}
	}
	void World::update() {
		//��ÿ������ʩ������
		for (auto i : m_Object_IDs) {
			auto obj = Object::ID_Map[i];
			obj->applyForce(m_gravity * obj->getMass(), obj->getCentroid());
		}
		//��ÿ�������״̬���и���,�ٶȣ�λ�ã����ٶȵ�״̬���и���
		for (unsigned int id : m_Object_IDs) {
			Object::ID_Map[id]->update(m_time_step);
		}
		//��ײ����
		this->collisionHandle();
		//��ÿ���������Լ������
		for (auto i : m_Constraint_IDs) {
			Constraint::ID_Map[i]->update(m_time_step);
		}
		//������֡����
		timer.capFrameRateWithDeltaTime(m_time_step);
	}
	void World::collisionHandle() {
		//����AABB
		for (unsigned int id : m_Object_IDs) {
			Object::ID_Map[id]->calculateAABB();//����AABB
		}
		//�����Ĳ���
		QuadTree Q_Tree(QuadTree::getRootbyAABBS(m_Object_IDs), 5, 10);
		for (auto i : m_Object_IDs) {
			Q_Tree.insert(*(Object::ID_Map[i]->getAABB()));
		}
		//������ײ���
		//���Լ�⣬ÿ������������ܿ�����ײ�������б�
		std::map<unsigned int, std::vector<unsigned int>> collisions;
		for (auto i : m_Object_IDs) {
			std::vector<unsigned int> c = Q_Tree.retrieve(*Object::ID_Map[i]->getAABB());
			collisions[i] = c;
		}
		//��ϸ���������������ײ��λ�������,ID������ײ��Ϣ
		std::map<unsigned int, std::vector<std::pair<unsigned int, CollisionInfo>>> CollisionMap;
		for (auto& i : collisions) {
			const Shape* shape1 = Object::ID_Map[i.first]->getShape();
			for (auto& j : i.second) {
				const Shape* shape2 = Object::ID_Map[j]->getShape();
				auto coll = CollisionDetector::CheckCollision(*shape1, *shape2);
				if (coll.has_value())
					CollisionMap[i.first].push_back(std::make_pair(j, coll.value()));
			}
		}
		//�Բ�����ײ������Ӧ�ó����������������ܵ���
		for (auto& i : CollisionMap) {
			for (auto& j : i.second) {
				auto obj1 = Object::ID_Map[i.first];
				auto obj2 = Object::ID_Map[j.first];
				//Ӧ�ó�������ײ�������ĳ���
				ImpulseResolution::ApplyImpulse(*obj1, *obj2, j.second);
				//����͸�������崩͸������д���
				PenetrationResolution::ResolvePenetration(*obj1, *obj2, j.second);
			}
		}
	}
}