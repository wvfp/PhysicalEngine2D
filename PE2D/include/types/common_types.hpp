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
	// 物理对象类型
	enum class ObjectType {
		OT_NONE = 0,//未知
		OT_RIGIDBODY,//刚体
		OT_ELASTICBODY,//弹性体
		OT_SOFTBODY,//软体
		OT_PARTICLE,//粒子
		OT_FLUID,//流体
	};
	//Physics Object class
	//物理对象类，所有物理对象的基类
	class Object {
		Object(const Object&) = delete; // Disable copy constructor 禁用复制构造函数
		Object(Object&&) = delete; // Disable move constructor 禁用移动构造函数
		Object& operator=(const Object&) = delete; // Disable copy assignment operator 禁用复制赋值运算符
		Object& operator=(Object&&) = delete; // Disable move assignment operator 禁用移动赋值运算符
	public:
		// Constructor and Destructor
		Object();
		virtual ~Object();
		// Factory method to create a new Object instance
		//工厂方法创建新的对象实例
		// 这个函数会返回一个ID,这个ID是唯一的,可以用来标识这个对象
		//但当这个对象被销毁时,这个ID会被释放,所以不能用来标识这个对象
		//static unsigned int makeObject() = 0;

		// Function to destroy the object
		//销毁对象的函数
		// 这个函数会把这个对象从ID_Map中删除
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
		ObjectType m_objectType = ObjectType::OT_NONE; // 物体类型
		AABB* m_aabb = nullptr; //AABB
		Shape* m_shape = nullptr; // 物体形状
		// Static variable to keep track of the next ID,starting from 1
		static unsigned int m_nextID;
		unsigned int m_ID = 0; // Unique ID for the object 唯一ID
		// Previous and next object in the island (used for island management)
		// 岛屿中的前一个和下一个物体（用于岛屿管理）
		unsigned int m_islandPrev = 0;
		unsigned int m_islandNext = 0;
		// Is the object alive (used for object pooling) 物体是否存活（用于对象池）
		bool m_isAlive = true;
		// Is the object active (used for physics simulation) 物体是否处于活动状态（用于物理模拟）
		bool m_isActive = true;
		float m_Mass = 1.0f; // Mass of the object 质量
		float m_InvMass = 1.0f; //Inverse
		float m_Intertia = 1.0f; // Inertia of the object 转动惯量
		float m_InvIntertia = 1.0f; // Inertia of the object 1/转动惯量
		float m_Restitution = 0.0f; // Restitution (bounciness) 回复系数
		float m_Friction = 0.0f; // Friction 摩擦系数
		float m_Density = 1.0f; // Density 密度
		Vector2D m_Centroid = { 0,0 };//Centroid 质心
		Vector2D m_Velocity = { 0.0f, 0.0f }; // Velocity of the object 速度
		Vector2D m_AngularVelocity = { 0.0f, 0.0f }; // Angular velocity of the object 角速度
		Vector2D m_Force = 0.0f; // Force applied to the object 作用于物体的力
		Vector2D m_Torque = { 0,0 }; // Torque applied to the object 作用于物体的力矩
		float m_LinearDamping = 0.0f; // Linear damping 线性阻尼
		float m_AngularDamping = 0.0f; // Angular damping 角阻尼
		float m_GravityScale = 1.0f; // Gravity scale 重力缩放
		bool m_fixed = false; // Fixed object (not affected by physics) 固定物体
		bool m_enableSleep = false; // Enable sleep mode 启用休眠模式
		float m_sleepTime = 0.0f; // Time to sleep 休眠时间
		float m_sleepThreshold = 0.0f; // Sleep threshold 休眠阈值
		bool m_fixedRotation = false; // Fixed rotation (does not rotate) 固定旋转
		Vector2D m_Position = { 0.0f, 0.0f }; // Position of the object 物体的位置
		// Rotation and scale matrices 旋转和缩放矩阵
		Matrix3x3 m_Rotation = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		Matrix3x3 m_Scale = { 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		std::vector<std::pair<Vector2D, Vector2D>> Force;
		std::vector<std::pair<Vector2D, Vector2D>> Impulse;
	};
}