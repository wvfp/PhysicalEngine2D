#include "types/common_types.hpp"
#include "math/math_utils.hpp"
#include <iterator>
#include <iostream>
#include <numeric>

namespace PE2D {
	Object::Object() : m_ID(m_nextID), m_isAlive(true), m_isActive(true),
		m_Position(0.0f, 0.0f), m_Rotation(Matrix3x3::rotate(0.0f)), m_Scale(Matrix3x3::scale(1.0f)),
		m_Mass(1.0f), m_fixed(false), m_enableSleep(false), m_fixedRotation(false),
		m_GravityScale(1.0f), m_LinearDamping(0.0f), m_AngularDamping(0.0f),
		m_Restitution(0.5f), m_Friction(0.5f), m_Density(1.0f),
		m_Velocity(0.0f, 0.0f), m_AngularVelocity(0.0f, 0.0f),
		m_Force(0.0f, 0.0f), m_Torque(Vector2D(0, 0)),
		m_sleepTime(0.0f), m_sleepThreshold(1e-4f),
		m_islandPrev(0), m_islandNext(0) {
		// Add the object to the ID map
		if (m_ID == ID_Map.size())
			ID_Map.push_back(std::shared_ptr<Object>(this));
		else
			ID_Map[m_ID] = std::shared_ptr<Object>(this);
		m_nextID = getNextID();
		this->setShape(new Rectangle(Vector2D(0, 1), Vector2D(1, 0)));
	}
	Object::~Object() {
		// Remove the object from the ID map
		if (m_shape) {
			delete m_shape;
			m_shape = nullptr;
		}
		if (m_aabb) {
			delete m_aabb;
			m_aabb = nullptr;
		}
	}
	// destructor
	void Object::destroyObject(unsigned int id) {
		// Remove the object from the ID map
		if (id < ID_Map.size()) {
			ID_Map[id] = nullptr;
		}
		else {
			std::cerr << "Object ID out of range" << std::endl;
		}
	}
	//Initialization list to set static values for the object properties
	unsigned int Object::m_nextID = 1;
	std::vector<std::shared_ptr<Object>> Object::ID_Map{ nullptr };
	void Object::removeAllObjFromMap() {
		for (auto i = ID_Map.begin(); i < ID_Map.end(); i++) {
			if (i->get() == nullptr) {
				ID_Map.erase(i);
				break;
			}
		}
		ID_Map.clear();
		m_nextID = 1;
	}
	void Object::removeObjFromMap(unsigned int id) {
		// Remove the object from the ID map
		//减少对象的引用计数
		ID_Map[id] = nullptr;
	}
	bool Object::isValidID(unsigned int id) {
		//判断ID代表的物体是否有效，物体是否存在
		if (id < ID_Map.size()) {
			if (ID_Map[id] == nullptr)
				return false;
			else
				return true;
		}
		else {
			return false;
		}
	}
	bool Object::isAlive() const {
		return m_isAlive;
	}

	bool Object::isActive() const {
		return m_isActive;
	}

	void Object::setActive(bool active) {
		m_isActive = active;
	}

	void Object::setAlive(bool alive) {
		m_isAlive = alive;
	}

	void Object::setPosition(float x, float y) {
		m_Position = Vector2D(x, y);
	}

	void Object::setPosition(const Vector2D& position) {
		Vector2D deltaVec = position - m_Position;
		m_Position = position;
		if (m_shape->type() == ShapeType::CIRCLE)
			dynamic_cast<Circle*>(m_shape)->setPosition(position);
	}

	void Object::setRotation(float angle) {
		m_Rotation = Matrix3x3::rotate(angle);
	}

	void Object::setRotation(const Matrix3x3& rotation) {
		m_shape->rotate(rotation);
		m_Rotation = rotation;
	}

	void Object::setScale(float x, float y) {
		m_Scale = Matrix3x3::scale(x, y);
	}

	void Object::setScale(const Matrix3x3& scale) {
		m_Scale = scale;
	}

	void Object::setScale(float scale) {
		m_Scale = Matrix3x3::scale(scale, scale);
	}

	Vector2D Object::getPosition() const {
		return m_Position;
	}

	Matrix3x3 Object::getRotation() const {
		return m_Rotation;
	}

	Matrix3x3 Object::getScale() const {
		return m_Scale;
	}

	unsigned int Object::getID() const {
		return m_ID;
	}

	void Object::setMass(float mass) {
		m_Mass = mass;
	}

	float Object::getMass() const {
		return m_Mass;
	}
	void Object::updateInvMass() {
		// Update the inverse mass
		if (m_Mass > 1e-6f) {
			m_InvMass = 1.0f / m_Mass;
		}
		else {
			m_InvMass = 0.0f; // Infinite mass
		}
	}
	float Object::getInvMass() const {
		return m_InvMass;
	}

	void Object::setID(unsigned int id) {
		m_ID = id;
	}

	unsigned int Object::getNextID() const {
		unsigned int nextID = 1;
		while (nextID < ID_Map.size() && ID_Map[nextID] != nullptr) {
			++nextID;
		}
		return nextID;
	}

	void Object::setFixed(bool fixed) {
		m_fixed = fixed;
	}

	bool Object::isFixed() const {
		return m_fixed;
	}

	void Object::setEnableSleep(bool enable) {
		m_enableSleep = enable;
	}

	bool Object::isEnableSleep() const {
		return m_enableSleep;
	}

	void Object::setFixedRotation(bool fixed) {
		m_fixedRotation = fixed;
	}

	bool Object::isFixedRotation() const {
		return m_fixedRotation;
	}

	void Object::setGravityScale(float scale) {
		m_GravityScale = scale;
	}

	float Object::getGravityScale() const {
		return m_GravityScale;
	}

	void Object::setLinearDamping(float damping) {
		m_LinearDamping = damping;
	}

	float Object::getLinearDamping() const {
		return m_LinearDamping;
	}

	void Object::setAngularDamping(float damping) {
		m_AngularDamping = damping;
	}

	float Object::getAngularDamping() const {
		return m_AngularDamping;
	}

	void Object::setRestitution(float restitution) {
		m_Restitution = restitution;
	}

	float Object::getRestitution() const {
		return m_Restitution;
	}

	void Object::setFriction(float friction) {
		m_Friction = friction;
	}

	float Object::getFriction() const {
		return m_Friction;
	}

	void Object::setDensity(float density) {
		m_Density = density;
	}

	float Object::getDensity() const {
		return m_Density;
	}

	void Object::setVelocity(const Vector2D& velocity) {
		m_Velocity = velocity;
	}

	Vector2D Object::getVelocity() const {
		return m_Velocity;
	}

	void Object::setAngularVelocity(const Vector2D& angularVelocity) {
		m_AngularVelocity = angularVelocity;
	}

	Vector2D Object::getAngularVelocity() const {
		return m_AngularVelocity;
	}

	void Object::setForce(const Vector2D& force) {
		m_Force = force;
	}

	Vector2D Object::getForce() const {
		return m_Force;
	}

	void Object::setTorque(Vector2D torque) {
		m_Torque = torque;
	}

	Vector2D Object::getTorque() const {
		return m_Torque;
	}

	void Object::setSleepTime(float time) {
		m_sleepTime = time;
	}

	float Object::getSleepTime() const {
		return m_sleepTime;
	}

	void Object::setSleepThreshold(float threshold) {
		m_sleepThreshold = threshold;
	}

	float Object::getSleepThreshold() const {
		return m_sleepThreshold;
	}

	void Object::setIslandPrev(unsigned int island) {
		m_islandPrev = island;
	}

	unsigned int Object::getIslandPrev() const {
		return m_islandPrev;
	}

	void Object::setIslandNext(unsigned int island) {
		m_islandNext = island;
	}

	unsigned int Object::getIslandNext() const {
		return m_islandNext;
	}
	void Object::setObjectType(ObjectType type) {
		m_objectType = type;
	}
	ObjectType Object::getObjectType() const {
		return m_objectType;
	}
	void Object::setShape(Shape* shape) {
		if (m_shape)
			delete m_shape;
		m_shape = shape;
		calculateAABB();
		m_Centroid = shape->getCentroid();
		m_Position = m_Centroid;
		calculateInertia();
	}
	const Shape* Object::getShape() const {
		return m_shape;
	}
	void Object::setCentroid(const Vector2D& centroid) {
		m_Centroid = centroid;
	}
	Vector2D Object::getCentroid() const {
		return m_Centroid;
	}
	void Object::calculateAABB() {
		if (m_shape) {
			m_aabb = new AABB(m_shape);
			m_aabb->setObj_ID(m_ID);
		}
	}
	//计算转动惯量
	void Object::calculateInertia() {
		if (m_shape->type() == ShapeType::UNKNOWN) {
			m_Intertia = 1.0;
			m_InvIntertia = 1.0;
			return;
		}
		if (m_shape->type() == ShapeType::CIRCLE) {
			m_Intertia = 0.5 * m_Mass * pow(static_cast<Circle*>(m_shape)->getRadius(), 2);
			if (m_Intertia < 1e-6)
				m_InvIntertia = 1.0 / m_Intertia;
			return;
		}
		if (m_shape->type() == ShapeType::CAPSULE) {
			float r = static_cast<Capsule*>(m_shape)->getRadius();
			float h = static_cast<Capsule*>(m_shape)->getHeight();
			float m1 = (PI * r * r / (PI * r * r + 2 * r * h)) * m_Mass;
			float m2 = m_Mass - m1;
			m_Intertia = 0.5 * m1 * r * r + (1.0 / 12.0) * m2 * (4 * r * r + h * h);
			if (m_Intertia < 1e-6)
				m_InvIntertia = 1.0 / m_Intertia;
			return;
		}
		//计算三角形，矩形，多边形的转动惯量
		float area = 0.0f;
		float inertia = 0.0f;
		auto vertices = m_shape->getVertices();
		Vector2D centroid = m_Centroid;
		for (size_t i = 0; i < vertices.size(); ++i) {
			size_t j = (i + 1) % vertices.size(); // 下一个顶点的索引
			float crossProduct = vertices[i].x() * vertices[j].y() - vertices[j].x() * vertices[i].y();
			area += crossProduct;
			Vector2D v1 = vertices[i] - centroid;
			Vector2D v2 = vertices[j] - centroid;
			float term = v1.x() * v1.x() + v1.x() * v2.x() + v2.x() * v2.x() + v1.y() * v1.y() + v1.y() * v2.y() + v2.y() * v2.y();
			inertia += crossProduct * term;
		}
		area = m_shape->area();
		if (area < 1e-7f) {
			std::cerr << "The polygon area is zero, cannot compute inertia." << std::endl;
			return;
		}
		inertia *= 1.0f / (12.0f * area);
		m_Intertia = inertia;
		m_InvIntertia = 1.0 / inertia;
	}
	Vector2D Object::calTorque() {
		float totalTorque = 0.0f;
		// 计算力的力矩贡献
		for (const auto& [force, applicationPoint] : Force) {
			Vector2D r = applicationPoint - m_Centroid;
			// 2D叉积公式：r × F = rx*Fy - ry*Fx
			totalTorque += r.x() * force.y() - r.y() * force.x();
		}

		// 计算冲量的力矩贡献
		for (const auto& [impulse, applicationPoint] : Impulse) {
			Vector2D r = applicationPoint - m_Centroid;
			totalTorque += r.x() * impulse.y() - r.y() * impulse.x();
		}
		float angle = 45 * PI / 180;
		Vector2D Torque;
		Torque.setX(sinf(angle));
		Torque.setX(sinf(angle));
		return Torque;
	}
	Parameter Object::getParameter() {
		Parameter Parm;
		//变量
		//合力
		for (auto i = Force.begin(); i != Force.end(); i++) {
			Parm.force += i->first;
		}
		//合冲量
		for (auto i = Impulse.begin(); i != Impulse.end(); i++) {
			Parm.impulse += i->first;
		}
		Parm.velocity = m_Velocity;
		Parm.angular_velocity = m_AngularVelocity;
		Parm.position = m_Position;
		Parm.inertia = m_Intertia;
		Parm.acceleration = m_Force / m_Mass;
		//计算力矩
		m_Torque = calTorque();
		Parm.shape = m_shape;
		Parm.angular_acceleration = m_Torque / m_Intertia;
		//常量
		Parm.mass = m_Mass;
		Parm.friction = m_Friction;
		Parm.damping = m_LinearDamping;
		Parm.angularDamping = m_AngularDamping;
		Parm.restitution = m_Restitution;
		return Parm;
	}
	void Object::setWithParameter(const Parameter& parm) {
		m_Velocity = parm.velocity;
		m_AngularVelocity = parm.angular_velocity;
		this->setPosition(parm.position);
		m_Intertia = parm.inertia;
		m_Torque = parm.torque;
	}
}