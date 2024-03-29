#ifndef PHYSICS2D_WORLD_H
#define PHYSICS2D_WORLD_H
#include "physics2d_common.h"
#include "physics2d_body.h"
#include "physics2d_math.h"
#include "physics2d_integrator.h"
#include "physics2d_joints.h"
#include "physics2d_random.h"
#include "physics2d_contact.h"
#include "physics2d_weld_joint.h"

namespace Physics2D
{
	class PHYSICS2D_API PhysicsWorld
	{
	public:
		PhysicsWorld() : m_gravity(0, -9.8f), m_linearVelocityDamping(0.9f), m_angularVelocityDamping(0.9f),
		                 m_linearVelocityThreshold(0.02f), m_angularVelocityThreshold(0.02f),
		                 m_airFrictionCoefficient(0.7f), m_bias(0.8f)
		{
		}

		~PhysicsWorld();
		//disable copy
		PhysicsWorld(const PhysicsWorld&) = delete;
		PhysicsWorld& operator=(const PhysicsWorld&) = delete;
		void prepareVelocityConstraint(const real& dt);
		void stepVelocity(const real& dt);
		void solveVelocityConstraint(real dt);
		void stepPosition(const real& dt);
		void solvePositionConstraint(real dt);


		Vector2 gravity() const;
		void setGravity(const Vector2& gravity);

		real linearVelocityDamping() const;
		void setLinearVelocityDamping(const real& linearVelocityDamping);

		real angularVelocityDamping() const;
		void setAngularVelocityDamping(const real& angularVelocityDamping);

		real linearVelocityThreshold() const;
		void setLinearVelocityThreshold(const real& linearVelocityThreshold);

		real angularVelocityThreshold() const;
		void setAngularVelocityThreshold(const real& angularVelocityThreshold);

		real airFrictionCoefficient() const;
		void setAirFrictionCoefficient(const real& airFrictionCoefficient);

		bool enableGravity() const;
		void setEnableGravity(bool enableGravity);

		bool enableDamping() const;
		void setEnableDamping(bool enableDamping);

		Body* createBody();
		void removeBody(Body* body);

		void removeJoint(Joint* joint);

		void clearAllBodies();
		void clearAllJoints();

		RotationJoint* createJoint(const RotationJointPrimitive& primitive);
		PointJoint* createJoint(const PointJointPrimitive& primitive);
		DistanceJoint* createJoint(const DistanceJointPrimitive& primitive);
		PulleyJoint* createJoint(const PulleyJointPrimitive& primitive);
		RevoluteJoint* createJoint(const RevoluteJointPrimitive& primitive);
		WeldJoint* createJoint(const WeldJointPrimitive& primitive);
		OrientationJoint* createJoint(const OrientationJointPrimitive& primitive);

		real bias() const;
		void setBias(const real& bias);

		Container::Vector<std::unique_ptr<Body>>& bodyList();

		Container::Vector<std::unique_ptr<Joint>>& jointList();

		bool& enableSleep();

	private:
		Vector2 m_gravity;
		real m_linearVelocityDamping;
		real m_angularVelocityDamping;
		real m_linearVelocityThreshold;
		real m_angularVelocityThreshold;
		real m_airFrictionCoefficient;

		real m_bias;

		bool m_enableGravity = true;
		bool m_enableDamping = true;
		bool m_enableSleep = false;
		Container::Vector<std::unique_ptr<Body>> m_bodyList;
		Container::Vector<std::unique_ptr<Joint>> m_jointList;
	};

	class PHYSICS2D_API DiscreteWorld
	{
	public:
		using ObjectID = uint32_t;
		ObjectID createBody(const ShapePrimitive& primitive);
		ObjectID createJoint();

		void step(real dt);
		void stepPosition(real dt);
		void stepVelocity(real dt);

		void removeBody(const ObjectID& id);
		void removeJoint(const ObjectID& id);

		void solveVelocity(real dt);
		void solvePosition(real dt);

	private:
		Container::Vector<ObjectID> m_bodyList;
		Container::Vector<ObjectID> m_jointList;
		Container::Vector<bool> m_sleepList;

		Vector2 m_gravity;
		real m_linearVelocityDamping = 0.9f;
		real m_angularVelocityDamping = 0.9f;
		real m_linearVelocityThreshold = 0.02f;
		real m_angularVelocityThreshold = 0.02f;
		real m_airFrictionCoefficient = 0.7f;
	};
}
#endif
