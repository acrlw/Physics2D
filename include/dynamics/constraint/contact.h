#ifndef PHYSICS2D_CONSTRAINT_CONTACT_H
#define PHYSICS2D_CONSTRAINT_CONTACT_H
#include <string>

#include "include/dynamics/body.h"
#include "include/utils/random.h"
#include "include/collision/detector.h"
namespace Physics2D
{
	using RelationID = long;
	static RelationID generateRelation(Body* bodyA, Body* bodyB);
	struct VelocityConstraintPoint
	{
		Vector2 ra;
		Vector2 rb;
		Vector2 va;
		Vector2 vb;
		Vector2 normal;
		Vector2 tangent;
		real bias = 0;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real accumulatedNormalImpulse = 0;
		real accumulatedTangentImpulse = 0;
	};
	
	struct ContactConstraintPoint
	{
		ContactConstraintPoint() = default;
		RelationID relation;
		int contactId = RandomGenerator::unique(1, 99999);
		real friction = 0.2;
		bool active = true;
		Vector2 localA;
		Vector2 localB;
		Body* bodyA;
		Body* bodyB;
		VelocityConstraintPoint vcp;
	};
	class ContactMaintainer
	{
	public:
		void solve(real dt);
		void add(const Collision& collision);
		std::map<RelationID, std::vector<ContactConstraintPoint>> m_contactTable;
		void renewCcp(ContactConstraintPoint& ccp, const PointPair& pair, const Collision& collision);
		real m_maxPenetration = 0.02;
		real m_biasFactor = 0.2;
	private:
	};
	
}
#endif
