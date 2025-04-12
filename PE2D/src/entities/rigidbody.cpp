#include "entities/rigidbody.hpp"
#include "math/vector2d.hpp"
#include "math/matrix3x3.hpp"
#include "motion_system/motion_types.hpp"
#include "motion_system/integrators/integrator.hpp"


namespace PE2D {
	RigidBody::RigidBody() : Object() {
		// Constructor implementation
		init();
	}
	unsigned int RigidBody::makeRigidBody() {
		// Factory method to create a new RigidBody instance
		// Returns the ID of the created RigidBody
		std::shared_ptr<RigidBody> rb = static_cast<std::shared_ptr<RigidBody>>(new RigidBody());
		return rb->getID();
	}
	void RigidBody::init() {
		// Initialization logic
		// Set default values for the rigid body properties
		setActive(true);
		setAlive(true);
		setPosition(0.0f, 0.0f);
		setRotation(Matrix3x3::rotate(0.0f));
		setScale(1.0f, 1.0f);
		setMass(1.0f);
		setFixed(false);
		setEnableSleep(false);
		setFixedRotation(false);
		setGravityScale(1.0f);
		setLinearDamping(0.0f);
		setAngularDamping(0.0f);
		setRestitution(0.5f);
		setFriction(0.5f);
		setDensity(1.0f);
		setVelocity(Vector2D());
		setAngularVelocity(Vector2D());
		setForce(Vector2D());
		setTorque(0.0f);
		setSleepTime(0.0f);
		setSleepThreshold(1e-4f);
		setIslandPrev(0);
		setIslandNext(0);
	}

	void RigidBody::update(float deltaTime) {
		// Update logic with deltaTime
		// Update the rigid body state using the selected integration method
		Parameter params=this->getParameter();
		params.delta_time = deltaTime;
		Parameter newParams;
		IntegrationMethod* Integrator =new RungeKuttaIntegration();
		newParams=Integrator->operator()(params);
		newParams.force = Vector2D(0,0);
		newParams.impulse = Vector2D(0,0);
		float rotate =deltaTime*(newParams.angular_velocity.magnitude()+
			params.angular_velocity.magnitude())/2.0;
		this->setRotation(getRotation()*Matrix3x3::rotate(rotate));
		this->setWithParameter(params);
		this->clearForce();
		this->clearImpulse();

	}
}