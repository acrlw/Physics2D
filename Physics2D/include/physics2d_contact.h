#ifndef PHYSICS2D_CONSTRAINT_CONTACT_H
#define PHYSICS2D_CONSTRAINT_CONTACT_H
#include <string>

#include "physics2d_body.h"
#include "physics2d_random.h"
#include "physics2d_detector.h"

namespace Physics2D
{
	struct PHYSICS2D_API VelocityConstraintPoint
	{
		Vector2 localA;
		Vector2 localB;
		Vector2 ra;
		Vector2 rb;
		Vector2 va;
		Vector2 vb;
		Vector2 normal;
		Vector2 tangent;
		Vector2 velocityBias;
		real bias = 0;
		real penetration = 0.0f;
		real restitution = 0.8f;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real accumulatedNormalImpulse = 0;
		real accumulatedTangentImpulse = 0;
	};

	struct PHYSICS2D_API ContactConstraintPoint
	{
		ContactConstraintPoint() = default;
		Body::BodyPair::BodyPairID relation = 0;
		real friction = 0.2f;
		bool active = true;
		Vector2 localA;
		Vector2 localB;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstraintPoint vcp;
		Matrix2x2 k;
		Matrix2x2 normalMass;
	};

	class PHYSICS2D_API ContactMaintainer
	{
	public:
		void clearAll();
		void solveVelocity(real dt);
		void solvePosition(real dt);
		void add(const Collision& collision);
		void prepare(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision);
		void clearInactivePoints();
		void deactivateAllPoints();
		real m_maxPenetration = 0.005f;
		real m_biasFactor = 0.2f;
		bool m_warmStart = true;
		bool m_velocityBlockSolver = false;
		bool m_positionBlockSolver = false;
		Container::Map<Body::BodyPair::BodyPairID, Container::Vector<ContactConstraintPoint>> m_contactTable;

	private:
	};
}
#endif
