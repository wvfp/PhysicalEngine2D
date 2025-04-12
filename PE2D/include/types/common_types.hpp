#pragma once
#include "math/vector2d.hpp"
#include "math/matrix3x3.hpp"
#include "math/geometry2d.hpp"
#include "collision_system/collision_types.hpp"
#include "motion_system/motion_types.hpp"
#include "motion_system/integrators/integrator.hpp"
#include <vector>
#include <memory>
namespace PE2D {
	// �����������
	enum class ObjectType {
		OT_NONE = 0,//δ֪
		OT_RIGIDBODY,//����
		OT_ELASTICBODY,//������
		OT_SOFTBODY,//����
		OT_PARTICLE,//����
		OT_FLUID,//����
	};
	//Physics Object class
	//��������࣬�����������Ļ���
	class Object {
		Object(const Object&) = delete; // Disable copy constructor ���ø��ƹ��캯��
		Object(Object&&) = delete; // Disable move constructor �����ƶ����캯��
		Object& operator=(const Object&) = delete; // Disable copy assignment operator ���ø��Ƹ�ֵ�����
		Object& operator=(Object&&) = delete; // Disable move assignment operator �����ƶ���ֵ�����
	public:
		// Constructor and Destructor
		Object();
		virtual ~Object();
		// Factory method to create a new Object instance
		//�������������µĶ���ʵ��
		// ��������᷵��һ��ID,���ID��Ψһ��,����������ʶ�������
		//���������������ʱ,���ID�ᱻ�ͷ�,���Բ���������ʶ�������
		//static unsigned int makeObject() = 0;

		// Function to destroy the object
		//���ٶ���ĺ���
		// ������������������ID_Map��ɾ��
		static void destroyObject(unsigned int);
		// Pure virtual functions to be implemented by derived classes
		virtual void update(float deltaTime) = 0;
		virtual void init() = 0;
		virtual void reset() = 0;
		// Getters and Setters
		bool isAlive() const;
		bool isActive() const;
		void setActive(bool active);
		void setAlive(bool alive);
		void setPosition(float x, float y);
		void setPosition(const Vector2D& position);
		void setRotation(float angle);
		void setRotation(const Matrix3x3& rotation);
		void setScale(float x, float y);
		void setScale(const Matrix3x3& scale);
		void setScale(float scale);
		Vector2D getPosition() const;
		Matrix3x3 getRotation() const;
		Matrix3x3 getScale() const;
		unsigned int getID() const;
		void setMass(float mass);
		float getMass() const;
		void updateInvMass();
		float getInvMass() const;
		void setID(unsigned int id);
		unsigned int getNextID() const;
		void setFixed(bool fixed);
		bool isFixed() const;
		void setEnableSleep(bool enable);
		bool isEnableSleep() const;
		void setFixedRotation(bool fixed);
		bool isFixedRotation() const;
		void setGravityScale(float scale);
		float getGravityScale() const;
		void setLinearDamping(float damping);
		float getLinearDamping() const;
		void setAngularDamping(float damping);
		float getAngularDamping() const;
		void setRestitution(float restitution);
		float getRestitution() const;
		void setFriction(float friction);
		float getFriction() const;
		void setDensity(float density);
		float getDensity() const;
		void setVelocity(const Vector2D& velocity);
		Vector2D getVelocity() const;
		void setAngularVelocity(const Vector2D& angularVelocity);
		Vector2D getAngularVelocity() const;
		void setForce(const Vector2D& force);
		Vector2D getForce() const;
		void setTorque(Vector2D torque);
		Vector2D getTorque() const;
		void setSleepTime(float time);
		float getSleepTime() const;
		void setSleepThreshold(float threshold);
		float getSleepThreshold() const;
		void setIslandPrev(unsigned int island);
		unsigned int getIslandPrev() const;
		void setIslandNext(unsigned int island);
		unsigned int getIslandNext() const;
		void setObjectType(ObjectType type);
		void setShape(Shape* shape);
		void setCentroid(const Vector2D& centroid);
		Vector2D getCentroid() const;
		const Shape* getShape() const;
		ObjectType getObjectType() const;
		void calculateAABB();
		void calculateInertia();
		Parameter getParameter();
		void setWithParameter(const Parameter&);
		void clearForce() { Force.clear(); }
		void clearImpulse() { Impulse.clear(); }
		const AABB* getAABB()const { return m_aabb; }
		Vector2D calTorque();
		void applyForce(Vector2D f, Vector2D p) {
			std::pair<Vector2D, Vector2D> a = std::make_pair(f, p);
			Force.push_back(a);
		}
		void applyImpulse(Vector2D i, Vector2D p) {
			Impulse.push_back(std::make_pair(i, p));
		}
	public:
		// Static Map to keep track of all objects by ID
		static std::vector<std::shared_ptr<Object>> ID_Map;
		static void removeAllObjFromMap();
		static void removeObjFromMap(unsigned int);
		static bool isValidID(unsigned int);
	protected:
		// Object type (used for object management)
		ObjectType m_objectType = ObjectType::OT_NONE; // ��������
		AABB* m_aabb = nullptr; //AABB
		Shape* m_shape = nullptr; // ������״
		// Static variable to keep track of the next ID,starting from 1
		static unsigned int m_nextID;
		unsigned int m_ID = 0; // Unique ID for the object ΨһID
		// Previous and next object in the island (used for island management)
		// �����е�ǰһ������һ�����壨���ڵ������
		unsigned int m_islandPrev = 0;
		unsigned int m_islandNext = 0;
		// Is the object alive (used for object pooling) �����Ƿ�����ڶ���أ�
		bool m_isAlive = true;
		// Is the object active (used for physics simulation) �����Ƿ��ڻ״̬����������ģ�⣩
		bool m_isActive = true;
		float m_Mass = 1.0f; // Mass of the object ����
		float m_InvMass = 1.0f; //Inverse
		float m_Intertia = 1.0f; // Inertia of the object ת������
		float m_InvIntertia = 1.0f; // Inertia of the object 1/ת������
		float m_Restitution = 0.0f; // Restitution (bounciness) �ظ�ϵ��
		float m_Friction = 0.0f; // Friction Ħ��ϵ��
		float m_Density = 1.0f; // Density �ܶ�
		Vector2D m_Centroid = { 0,0 };//Centroid ����
		Vector2D m_Velocity = { 0.0f, 0.0f }; // Velocity of the object �ٶ�
		Vector2D m_AngularVelocity = { 0.0f, 0.0f }; // Angular velocity of the object ���ٶ�
		Vector2D m_Force = 0.0f; // Force applied to the object �������������
		Vector2D m_Torque = { 0,0 }; // Torque applied to the object ���������������
		float m_LinearDamping = 0.0f; // Linear damping ��������
		float m_AngularDamping = 0.0f; // Angular damping ������
		float m_GravityScale = 1.0f; // Gravity scale ��������
		bool m_fixed = false; // Fixed object (not affected by physics) �̶�����
		bool m_enableSleep = false; // Enable sleep mode ��������ģʽ
		float m_sleepTime = 0.0f; // Time to sleep ����ʱ��
		float m_sleepThreshold = 0.0f; // Sleep threshold ������ֵ
		bool m_fixedRotation = false; // Fixed rotation (does not rotate) �̶���ת
		Vector2D m_Position = { 0.0f, 0.0f }; // Position of the object �����λ��
		// Rotation and scale matrices ��ת�����ž���
		Matrix3x3 m_Rotation = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		Matrix3x3 m_Scale = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		std::vector<std::pair<Vector2D, Vector2D>> Force;
		std::vector<std::pair<Vector2D, Vector2D>> Impulse;
	};
}