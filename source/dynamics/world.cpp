#include "include/dynamics/world.h"

namespace Physics2D
{
	World::~World()
	{
		for (Body* body : m_bodyList)
			delete body;
	}

	Vector2 World::screenToWorld(const Vector2& pos) const
	{
		return screenToWorld(m_leftTop, m_rightBottom, pos);
	}

	Vector2 World::worldToScreen(const Vector2& pos) const
	{
		return worldToScreen(m_leftTop, m_rightBottom, pos);
	}
	void World::stepVelocity(const real& dt)
	{
		const Vector2 g = m_enableGravity ? m_gravity : (0, 0);
		for (Body* body : m_bodyList)
		{
			switch (body->type())
			{
			case Body::BodyType::Static:
				break;
			case Body::BodyType::Dynamic:
			{
				body->forces() += g - 0.5 * m_airFrictionCoefficient * body->velocity();
				body->velocity() += body->inverseMass() * body->forces() * dt * m_velocityIteration;
				body->angularVelocity() += body->inverseInertia() * body->torques() * dt * m_velocityIteration;
				
				break;
			}
			case Body::BodyType::Kinematic:
			{
				body->velocity() += body->inverseMass() * body->forces() * dt * m_velocityIteration;
				body->angularVelocity() += body->inverseInertia() * body->torques() * dt * m_velocityIteration;
				break;
			}
			case Body::BodyType::Bullet:
			{
				break;
			}
			}
		}
	}
	void World::stepPosition(const real& dt)
	{
		for (Body* body : m_bodyList)
		{
			switch (body->type())
			{
			case Body::BodyType::Static:
				break;
			case Body::BodyType::Dynamic:
			{
				body->position() += body->velocity() * dt * m_positionIteration;
				body->angle() += body->angularVelocity() * dt * m_positionIteration;

				body->forces().clear();
				body->clearTorque();
				break;
			}
			case Body::BodyType::Kinematic:
			{
				body->position() += body->velocity() * dt * m_positionIteration;
				body->angle() += body->angularVelocity() * dt * m_positionIteration;

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
	void World::step(const real& dt)
	{
		const Vector2 g = m_enableGravity ? m_gravity : (0, 0);
		for (Body* body : m_bodyList)
		{
			switch (body->type())
			{
			case Body::BodyType::Static:
				break;
			case Body::BodyType::Dynamic:
				{
					const Vector2 forces = g + body->forces();
					const Vector2 a = body->inverseMass() * forces;
					const Vector2 v = (body->velocity() + a * dt * m_velocityIteration) * m_linearVelocityDamping;
					const Vector2 p = body->position() + v * dt * m_positionIteration;

					const real b = body->inverseInertia() * body->torques();
					const real av = (body->angularVelocity() + b * dt * m_velocityIteration) * m_angularVelocityDamping;
					const real angle = body->angle() + av * dt * m_positionIteration;

					body->velocity() = v;
					body->position() = p;
					body->angularVelocity() = av;
					body->angle() = angle;

					body->forces().clear();
					body->clearTorque();
					break;
				}
			case Body::BodyType::Kinematic:
				{
					break;
				}
			case Body::BodyType::Bullet:
				{
					break;
				}
			}
		}
	}

	void World::setGeometry(const Vector2& leftTop, const Vector2& rightBottom)
	{
		m_leftTop = leftTop;
		m_rightBottom = rightBottom;
	}

	Vector2 World::worldToScreen(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos)
	{
		//real screen origin
		const real origin_y = (rightBottom.y + leftTop.y) * (0.5f);
		const real origin_x = (leftTop.x + rightBottom.x) * (0.5f);
		return Vector2(origin_x + pos.x * Constant::meterToPixel, origin_y - pos.y * Constant::meterToPixel);
	}

	Vector2 World::screenToWorld(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos)
	{
		//real screen origin
		const real origin_y = (rightBottom.y + leftTop.y) * (0.5f);
		const real origin_x = (leftTop.x + rightBottom.x) * (0.5f);
		Vector2 result = pos - Vector2(origin_x, origin_y);
		result.y = -result.y;
		result *= Constant::pixelToMeter;
		return result;
	}

	std::vector<Body*> World::bodyList() const
	{
		return m_bodyList;
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

	Integrator World::integrator() const
	{
		return m_integrator;
	}

	void World::setIntegrator(const Integrator& integrator)
	{
		m_integrator = integrator;
	}

	Vector2 World::leftTop() const
	{
		return m_leftTop;
	}

	void World::setLeftTop(const Vector2& leftTop)
	{
		m_leftTop = leftTop;
	}

	Vector2 World::rightBottom() const
	{
		return m_rightBottom;
	}

	void World::setRightBottom(const Vector2& rightBottom)
	{
		m_rightBottom = rightBottom;
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

	void World::addBody(Body* body)
	{
		m_bodyList.emplace_back(body);
	}

	void World::removeBody(Body* body)
	{
		m_bodyList.erase(std::remove(m_bodyList.begin(), m_bodyList.end(), body),
		                 m_bodyList.end());
		delete body;
	}

	Body* World::createBody()
	{
		Body* body = new Body;
		m_bodyList.emplace_back(body);
		return body;
	}

	real World::width()
	{
		return m_rightBottom.x - m_leftTop.x;
	}

	real World::height()
	{
		return m_rightBottom.y - m_leftTop.y;
	}
}
