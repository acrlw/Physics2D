#include "include/dynamics/world.h"

namespace Physics2D
{
	World::~World()
	{
		for (auto& body : m_bodyList)
			body.release();

		for (auto& joint : m_jointList)
			joint.release();
	}

	void World::prepareVelocityConstraint(const real& dt)
	{
		for (auto& joint : m_jointList)
			if (joint->active())
				joint->prepare(dt);
	}

	void World::stepVelocity(const real& dt)
	{
		const Vector2 g = m_enableGravity ? m_gravity : Vector2{0.0, 0.0};
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

					//damping
				body->velocity() *= 1.0 / (1.0 + dt * m_linearVelocityDamping);
				body->angularVelocity() *= 1.0 / (1.0 + dt * m_angularVelocityDamping);

				break;
			}
			case Body::BodyType::Kinematic:
			{
				body->velocity() += body->inverseMass() * body->forces() * dt;
				body->angularVelocity() += body->inverseInertia() * body->torques() * dt;

				body->velocity() *= 1.0 / (1.0 + dt * m_linearVelocityDamping);
				body->angularVelocity() *= 1.0 / (1.0 + dt * m_angularVelocityDamping);
				break;
			}
			case Body::BodyType::Bullet:
			{
				break;
			}
			}
		}
	}
	void World::solveVelocityConstraint(real dt)
	{
		for (int i = 0; i < m_velocityIteration; i++)
			for (auto& joint : m_jointList)
				if (joint->active())
					joint->solveVelocity(dt);
	}
	void World::solvePositionConstraint(real dt)
	{
		for (int i = 0; i < m_positionIteration; i++)
			for (auto& joint : m_jointList)
				if (joint->active())
					joint->solvePosition(dt);
	}

	void World::stepPosition(const real& dt)
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
	
	real World::bias() const
	{
		return m_bias;
	}

	void World::setBias(const real& bias)
	{
		m_bias = bias;
	}

	real World::velocityIteration() const
	{
		return m_velocityIteration;
	}

	void World::setVelocityIteration(const real& velocityIteration)
	{
		m_velocityIteration = velocityIteration;
	}

	real World::positionIteration() const
	{
		return m_positionIteration;
	}

	void World::setPositionIteration(const real& positionIteration)
	{
		m_positionIteration = positionIteration;
	}
	
	std::vector<std::unique_ptr<Body>>& World::bodyList()
	{
		return m_bodyList;
	}

	std::vector<std::unique_ptr<Joint>>& World::jointList()
	{
		return m_jointList;
	}
	
	Vector2 World::gravity() const
	{
		return m_gravity;
	}

	void World::setGravity(const Vector2& gravity)
	{
		m_gravity = gravity;
	}

	real World::linearVelocityDamping() const
	{
		return m_linearVelocityDamping;
	}

	void World::setLinearVelocityDamping(const real& linearVelocityDamping)
	{
		m_linearVelocityDamping = linearVelocityDamping;
	}

	real World::angularVelocityDamping() const
	{
		return m_angularVelocityDamping;
	}

	void World::setAngularVelocityDamping(const real& angularVelocityDamping)
	{
		m_angularVelocityDamping = angularVelocityDamping;
	}

	real World::linearVelocityThreshold() const
	{
		return m_linearVelocityThreshold;
	}

	void World::setLinearVelocityThreshold(const real& linearVelocityThreshold)
	{
		m_linearVelocityThreshold = linearVelocityThreshold;
	}

	real World::angularVelocityThreshold() const
	{
		return m_angularVelocityThreshold;
	}

	void World::setAngularVelocityThreshold(const real& angularVelocityThreshold)
	{
		m_angularVelocityThreshold = angularVelocityThreshold;
	}

	real World::airFrictionCoefficient() const
	{
		return m_airFrictionCoefficient;
	}

	void World::setAirFrictionCoefficient(const real& airFrictionCoefficient)
	{
		m_airFrictionCoefficient = airFrictionCoefficient;
	}

	bool World::enableGravity() const
	{
		return m_enableGravity;
	}

	void World::setEnableGravity(bool enableGravity)
	{
		m_enableGravity = enableGravity;
	}
	

	Body* World::createBody()
	{
		//Body* body = new Body;
		auto body = std::make_unique<Body>();
		Body* temp = body.get();
		temp->setId(RandomGenerator::unique());
		m_bodyList.emplace_back(std::move(body));
		return temp;
	}

	void World::removeBody(Body* body)
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

	RotationJoint* World::createJoint(const RotationJointPrimitive& primitive)
	{
		auto joint = std::make_unique<RotationJoint>(primitive);
		RotationJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	PointJoint* World::createJoint(const PointJointPrimitive& primitive)
	{
		auto joint = std::make_unique<PointJoint>(primitive);
		PointJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	DistanceJoint* World::createJoint(const DistanceJointPrimitive& primitive)
	{
		auto joint = std::make_unique<DistanceJoint>(primitive);
		DistanceJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}
	
	PulleyJoint* World::createJoint(const PulleyJointPrimitive& primitive)
	{
		auto joint = std::make_unique<PulleyJoint>(primitive);
		PulleyJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	RevoluteJoint* World::createJoint(const RevoluteJointPrimitive& primitive)
	{
		auto joint = std::make_unique<RevoluteJoint>(primitive);
		RevoluteJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}

	OrientationJoint* World::createJoint(const OrientationJointPrimitive& primitive)
	{
		auto joint = std::make_unique<OrientationJoint>(primitive);
		OrientationJoint* temp = joint.get();
		m_jointList.emplace_back(std::move(joint));
		return temp;
	}
}
