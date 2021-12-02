#include "../../include/dynamics/world.h"

namespace Physics2D
{
	PhysicsWorld::~PhysicsWorld()
	{
		clearAllBodies();

		clearAllJoints();
	}

	void PhysicsWorld::prepareVelocityConstraint(const real& dt)
	{
		for (auto& joint : m_jointList)
			if (joint->active())
				joint->prepare(dt);
	}

	void PhysicsWorld::stepVelocity(const real& dt)
	{
		const Vector2 g = m_enableGravity ? m_gravity : Vector2{0.0, 0.0};
		real lvd = 1.0f;
		real avd = 1.0f;
		if(m_enableDamping)
		{
			lvd = 1.0f / (1.0f + dt * m_linearVelocityDamping);
			avd = 1.0f / (1.0f + dt * m_angularVelocityDamping);
		}
		for (auto& body : m_bodyList)
		{
			switch (body->type())
			{
			case Body::BodyType::Static:
			{
				body->velocity().clear();
				body->angularVelocity() = 0;
				break;
			}
			case Body::BodyType::Dynamic:
			{
				body->forces() += body->mass() * g;

				body->velocity() += body->inverseMass() * body->forces() * dt;
				body->angularVelocity() += body->inverseInertia() * body->torques() * dt;

				
				body->velocity() *= lvd;
				body->angularVelocity() *= avd;

				break;
			}
			case Body::BodyType::Kinematic:
			{
				body->velocity() += body->inverseMass() * body->forces() * dt;
				body->angularVelocity() += body->inverseInertia() * body->torques() * dt;

				body->velocity() *= lvd;
				body->angularVelocity() *= avd;
				break;
			}
			case Body::BodyType::Bullet:
			{
				break;
			}
			}
		}
	}
	void PhysicsWorld::solveVelocityConstraint(real dt)
	{
		for (auto& joint : m_jointList)
			if (joint->active())
				joint->solveVelocity(dt);
	}
	void PhysicsWorld::solvePositionConstraint(real dt)
	{
		for (auto& joint : m_jointList)
			if (joint->active())
				joint->solvePosition(dt);
	}

	void PhysicsWorld::stepPosition(const real& dt)
	{

		for (auto& body : m_bodyList)
		{
			switch (body->type())
			{
			case Body::BodyType::Static:
				break;
			case Body::BodyType::Dynamic:
			{
				body->position() += body->velocity() * dt;
				body->rotation() += body->angularVelocity() * dt;

				body->forces().clear();
				body->clearTorque();
				break;
			}
			case Body::BodyType::Kinematic:
			{
				body->position() += body->velocity() * dt;
				body->rotation() += body->angularVelocity() * dt;

				body->forces().clear();
				body->clearTorque();
				break;
			}
			case Body::BodyType::Bullet:
			{
				break;
			}
			}
		}

	}
	
	real PhysicsWorld::bias() const
	{
		return m_bias;
	}

	void PhysicsWorld::setBias(const real& bias)
	{
		m_bias = bias;
	}

	int PhysicsWorld::velocityIteration() const
	{
		return m_velocityIteration;
	}

	void PhysicsWorld::setVelocityIteration(const int& velocityIteration)
	{
		m_velocityIteration = velocityIteration;
	}

	int PhysicsWorld::positionIteration() const
	{
		return m_positionIteration;
	}

	void PhysicsWorld::setPositionIteration(const int& positionIteration)
	{
		m_positionIteration = positionIteration;
	}
	
	std::vector<std::unique_ptr<Body>>& PhysicsWorld::bodyList()
	{
		return m_bodyList;
	}

	std::vector<std::unique_ptr<Joint>>& PhysicsWorld::jointList()
	{
		return m_jointList;
	}
	
	Vector2 PhysicsWorld::gravity() const
	{
		return m_gravity;
	}

	void PhysicsWorld::setGravity(const Vector2& gravity)
	{
		m_gravity = gravity;
	}

	real PhysicsWorld::linearVelocityDamping() const
	{
		return m_linearVelocityDamping;
	}

	void PhysicsWorld::setLinearVelocityDamping(const real& linearVelocityDamping)
	{
		m_linearVelocityDamping = linearVelocityDamping;
	}

	real PhysicsWorld::angularVelocityDamping() const
	{
		return m_angularVelocityDamping;
	}

	void PhysicsWorld::setAngularVelocityDamping(const real& angularVelocityDamping)
	{
		m_angularVelocityDamping = angularVelocityDamping;
	}

	real PhysicsWorld::linearVelocityThreshold() const
	{
		return m_linearVelocityThreshold;
	}

	void PhysicsWorld::setLinearVelocityThreshold(const real& linearVelocityThreshold)
	{
		m_linearVelocityThreshold = linearVelocityThreshold;
	}

	real PhysicsWorld::angularVelocityThreshold() const
	{
		return m_angularVelocityThreshold;
	}

	void PhysicsWorld::setAngularVelocityThreshold(const real& angularVelocityThreshold)
	{
		m_angularVelocityThreshold = angularVelocityThreshold;
	}

	real PhysicsWorld::airFrictionCoefficient() const
	{
		return m_airFrictionCoefficient;
	}

	void PhysicsWorld::setAirFrictionCoefficient(const real& airFrictionCoefficient)
	{
		m_airFrictionCoefficient = airFrictionCoefficient;
	}

	bool PhysicsWorld::enableGravity() const
	{
		return m_enableGravity;
	}

	void PhysicsWorld::setEnableGravity(bool enableGravity)
	{
		m_enableGravity = enableGravity;
	}

	bool PhysicsWorld::enableDamping() const
	{
		return m_enableDamping;
	}

	void PhysicsWorld::setEnableDamping(bool enableDamping)
	{
		m_enableDamping = enableDamping;
	}


	Body* PhysicsWorld::createBody()
	{
		auto body = std::make_unique<Body>();
		Body* temp = body.get();
		temp->setId(RandomGenerator::unique());
		m_bodyList.emplace_back(std::move(body));
		return temp;
	}

	void PhysicsWorld::removeBody(Body* body)
	{
		for(auto iter = m_bodyList.begin(); iter != m_bodyList.end(); ++iter)
		{
			if(iter->get() == body)
			{
				RandomGenerator::pop(body->id());
				iter->release();
				m_bodyList.erase(iter);
				break;
			}
		}
	}

	void PhysicsWorld::removeJoint(Joint* joint)
	{
		for (auto iter = m_jointList.begin(); iter != m_jointList.end(); ++iter)
		{
			if (iter->get() == joint)
			{
				RandomGenerator::pop(joint->id());
				iter->release();
				m_jointList.erase(iter);
				break;
			}
		}
	}

	void PhysicsWorld::clearAllBodies()
	{
		for (auto& body : m_bodyList)
			body.release();
		m_bodyList.clear();
	}

	void PhysicsWorld::clearAllJoints()
	{
		for (auto& joint : m_jointList)
			joint.release();
		m_jointList.clear();
	}

	RotationJoint* PhysicsWorld::createJoint(const RotationJointPrimitive& primitive)
	{
		auto joint = std::make_unique<RotationJoint>(primitive);
		RotationJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	PointJoint* PhysicsWorld::createJoint(const PointJointPrimitive& primitive)
	{
		auto joint = std::make_unique<PointJoint>(primitive);
		PointJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	DistanceJoint* PhysicsWorld::createJoint(const DistanceJointPrimitive& primitive)
	{
		auto joint = std::make_unique<DistanceJoint>(primitive);
		DistanceJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}
	
	PulleyJoint* PhysicsWorld::createJoint(const PulleyJointPrimitive& primitive)
	{
		auto joint = std::make_unique<PulleyJoint>(primitive);
		PulleyJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	RevoluteJoint* PhysicsWorld::createJoint(const RevoluteJointPrimitive& primitive)
	{
		auto joint = std::make_unique<RevoluteJoint>(primitive);
		RevoluteJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	OrientationJoint* PhysicsWorld::createJoint(const OrientationJointPrimitive& primitive)
	{
		auto joint = std::make_unique<OrientationJoint>(primitive);
		OrientationJoint* temp = joint.get();
		temp->setId(RandomGenerator::unique());
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}
	
}
