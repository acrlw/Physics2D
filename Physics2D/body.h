#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
#include "aabb.h"
#include "math.h"
#include "common.h"
#include "shape.h"
#include "integrator.h"

#include "capsule.h"
#include "circle.h"
#include "curve.h"
#include "edge.h"
#include "ellipse.h"
#include "point.h"
#include "polygon.h"
#include "rectangle.h"
#include "sector.h"

namespace Physics2D
{
	class Body
	{
	public:

		struct Relation
		{
			using RelationID = uint64_t;
			static RelationID generateRelationID(Body* bodyA, Body* bodyB);
			static Relation generateRelation(Body* bodyA, Body* bodyB);
			RelationID relationID;
			Body* bodyA;
			Body* bodyB;
		};

		enum class BodyType
		{
			Kinematic,
			Static,
			Dynamic,
			Bullet
		};

		struct PhysicsAttribute
		{
			Vector2 position;
			Vector2 velocity;
			real rotation = 0;
			real angularVelocity = 0;
			void step(const real& dt);
		};

		Body() = default;
		Vector2& position();

		Vector2& velocity();

		real& rotation();

		real& angularVelocity();

		Vector2& forces();
		void clearTorque();

		real& torques();

		Vector2& lastPosition();
		real& lastRotation();
		uint32_t& sleepCountdown();

		Shape* shape() const;
		void setShape(Shape* shape);

		BodyType type() const;
		void setType(const BodyType& type);

		real mass() const;
		void setMass(const real& mass);

		real inertia() const;

		AABB aabb(const real& factor = Constant::AABBExpansionFactor) const;

		real friction() const;
		void setFriction(const real& friction);

		bool sleep() const;
		void setSleep(bool sleep);

		real inverseMass() const;
		real inverseInertia() const;

		PhysicsAttribute physicsAttribute() const;
		void setPhysicsAttribute(const PhysicsAttribute& info);

		void stepPosition(const real& dt);

		void applyImpulse(const Vector2& impulse, const Vector2& r);
		Vector2 toLocalPoint(const Vector2& point) const;
		Vector2 toWorldPoint(const Vector2& point) const;
		Vector2 toActualPoint(const Vector2& point) const;

		uint32_t id()const;
		void setId(const uint32_t& id);

		uint32_t bitmask()const;
		void setBitmask(const uint32_t& bitmask);

		real restitution()const;
		void setRestitution(const real& restitution);


	private:
		void calcInertia();

		uint32_t m_id;
		uint32_t m_bitmask = 1;

		real m_mass = 0;
		real m_inertia = 0;
		real m_invMass = 0;
		real m_invInertia = 0;

		Vector2 m_position;
		Vector2 m_velocity;
		real m_rotation = 0;
		real m_angularVelocity = 0;

		Vector2 m_lastPosition;
		real m_lastRotation = 0;

		Vector2 m_forces;
		real m_torques = 0;

		Shape* m_shape;
		BodyType m_type = BodyType::Static;

		bool m_sleep = false;
		real m_friction = 0.1f;
		real m_restitution = 0.0f;
		
		uint32_t m_sleepCountdown = 0;

		
	};
}
#endif
