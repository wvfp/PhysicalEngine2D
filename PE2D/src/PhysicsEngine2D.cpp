#include "PhysicalEngine2D.hpp"
#include <map>
namespace PE2D {
	//构造函数
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
		//对每个物体施加重力
		for (auto i : m_Object_IDs) {
			auto obj = Object::ID_Map[i];
			obj->applyForce(m_gravity * obj->getMass(), obj->getCentroid());
		}
		//对每个物体的状态进行更新,速度，位置，角速度等状态进行更新
		for (unsigned int id : m_Object_IDs) {
			Object::ID_Map[id]->update(m_time_step);
		}
		//碰撞处理
		this->collisionHandle();
		//对每个物体进行约束处理
		for (auto i : m_Constraint_IDs) {
			Constraint::ID_Map[i]->update(m_time_step);
		}
		//最后进行帧控制
		timer.capFrameRateWithDeltaTime(m_time_step);
	}
	void World::collisionHandle() {
		//计算AABB
		for (unsigned int id : m_Object_IDs) {
			Object::ID_Map[id]->calculateAABB();//计算AABB
		}
		//构造四叉树
		QuadTree Q_Tree(QuadTree::getRootbyAABBS(m_Object_IDs), 5, 10);
		for (auto i : m_Object_IDs) {
			Q_Tree.insert(*(Object::ID_Map[i]->getAABB()));
		}
		//进行碰撞检测
		//粗略检测，每个物体与其可能可能碰撞的物体列表
		std::map<unsigned int, std::vector<unsigned int>> collisions;
		for (auto i : m_Object_IDs) {
			std::vector<unsigned int> c = Q_Tree.retrieve(*Object::ID_Map[i]->getAABB());
			collisions[i] = c;
		}
		//仔细检测物体与物体碰撞的位置与深度,ID与其碰撞信息
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
		//对产生碰撞的物体应用冲量，更新物体所受的力
		for (auto& i : CollisionMap) {
			for (auto& j : i.second) {
				auto obj1 = Object::ID_Map[i.first];
				auto obj2 = Object::ID_Map[j.first];
				//应用冲量，碰撞所产生的冲量
				ImpulseResolution::ApplyImpulse(*obj1, *obj2, j.second);
				//处理穿透，若物体穿透，则进行处理
				PenetrationResolution::ResolvePenetration(*obj1, *obj2, j.second);
			}
		}
	}
}