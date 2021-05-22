#ifndef PHYSICS2D_CONSTRAINT_CONTACT_H
#define PHYSICS2D_CONSTRAINT_CONTACT_H
#include "include/dynamics/body.h"

namespace Physics2D
{
	struct VelocityConstraintPoint
	{
		Vector2 ra;
		Vector2 rb;
		Vector2 vpa;
		Vector2 vpb;
		real bias = 0;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real accumulatedNormalImpulse = 0;
		real accumulatedTangentImpulse = 0;
	};

	struct ContactInfo
	{
		Collision result;
		std::vector<VelocityConstraintPoint> points;
		Matrix2x2 effectiveMassNormal;
		Matrix2x2 effectiveMassTangent;
	};

	class ContactConstraintSolver
	{
	public:
		void add(const ContactInfo& info)
		{
			m_list.emplace_back(info);
		}

		void prepare()
		{
			for (auto& info : m_list)
			{
				switch (info.result.contactList.size())
				{
				case 1:
					{
						Vector2 ra = info.result.contactList[0].pointA - info.result.bodyA->position();
						Vector2 rb = info.result.contactList[0].pointB - info.result.bodyB->position();
						Vector2 n = info.result.normal;
						Vector2 t = n.perpendicular();

						real im_a = info.result.bodyA->inverseMass();
						real im_b = info.result.bodyB->inverseMass();
						real ii_a = info.result.bodyA->inverseInertia();
						real ii_b = info.result.bodyB->inverseInertia();

						real rn_a = ra.cross(n);
						real rn_b = rb.cross(n);

						real rt_a = ra.cross(t);
						real rt_b = rb.cross(t);

						real kNormal = im_a + ii_a * rn_a * rn_a +
							im_b + ii_b * rn_b * rn_b;

						real kTangent = im_a + ii_a * rt_a * rt_a +
							im_b + ii_b * rt_b * rt_b;

						VelocityConstraintPoint vcp;

						vcp.ra = ra;
						vcp.rb = rb;
						vcp.vpa = info.result.bodyA->velocity() +
							Vector2::crossProduct(info.result.bodyA->angularVelocity(), ra);
						vcp.vpb = info.result.bodyB->velocity() +
							Vector2::crossProduct(info.result.bodyB->angularVelocity(), rb);
						vcp.effectiveMassNormal = realEqual(kNormal, 0.0) ? 0 : 1.0 / kNormal;
						vcp.effectiveMassTangent = realEqual(kTangent, 0.0) ? 0 : 1.0 / kTangent;

						vcp.bias = 0.0;

						real v_rel = n.dot(vcp.vpb - vcp.vpa);
						vcp.bias = - 0.6 * v_rel;

						info.points.emplace_back(vcp);
						break;
					}
				case 2:
					{
						Vector2 ra1 = info.result.contactList[0].pointA - info.result.bodyA->position();
						Vector2 rb1 = info.result.contactList[0].pointB - info.result.bodyB->position();
						Vector2 ra2 = info.result.contactList[1].pointA - info.result.bodyA->position();
						Vector2 rb2 = info.result.contactList[1].pointB - info.result.bodyB->position();

						Vector2 n = info.result.normal;
						Vector2 t = n.perpendicular();

						real im_a = info.result.bodyA->inverseMass();
						real im_b = info.result.bodyB->inverseMass();
						real ii_a = info.result.bodyA->inverseInertia();
						real ii_b = info.result.bodyB->inverseInertia();

						real rn_a1 = ra1.cross(n);
						real rn_b1 = rb1.cross(n);

						real rt_a1 = ra1.cross(t);
						real rt_b1 = rb1.cross(t);

						real rn_a2 = ra2.cross(n);
						real rn_b2 = rb2.cross(n);

						real rt_a2 = ra2.cross(t);
						real rt_b2 = rb2.cross(t);

						info.effectiveMassNormal.e11() = im_a + ii_a * rn_a1 * rn_a1 + im_b + ii_b * rn_b1 * rn_b1;
						info.effectiveMassNormal.e12() = im_a + ii_a * rn_a1 * rn_a2 + im_b + ii_b * rn_b1 * rn_b2;
						info.effectiveMassNormal.e21() = info.effectiveMassNormal.e12();
						info.effectiveMassNormal.e22() = im_a + ii_a * rn_a2 * rn_a2 + im_b + ii_b * rn_b2 * rn_b2;
						info.effectiveMassNormal.invert();

						info.effectiveMassTangent.e11() = im_a + ii_a * rt_a1 * rt_a1 + im_b + ii_b * rt_b1 * rt_b1;
						info.effectiveMassTangent.e12() = im_a + ii_a * rt_a1 * rt_a2 + im_b + ii_b * rt_b1 * rt_b2;
						info.effectiveMassTangent.e21() = info.effectiveMassTangent.e12();
						info.effectiveMassTangent.e22() = im_a + ii_a * rt_a2 * rt_a2 + im_b + ii_b * rt_b2 * rt_b2;
						info.effectiveMassTangent.invert();

						real kNormal1 = im_a + ii_a * rn_a1 * rn_a1 +
							im_b + ii_b * rn_b1 * rn_b1;
						
						real kNormal2 = im_a + ii_a * rn_a2 * rn_a2 +
							im_b + ii_b * rn_b2 * rn_b2;
						

						VelocityConstraintPoint vcp1, vcp2;

						vcp1.ra = ra1;
						vcp1.rb = rb1;
						vcp1.vpa = info.result.bodyA->velocity() +
							Vector2::crossProduct(info.result.bodyA->angularVelocity(), ra1);
						vcp1.vpb = info.result.bodyB->velocity() +
							Vector2::crossProduct(info.result.bodyB->angularVelocity(), rb1);
						vcp1.effectiveMassNormal = kNormal1;
						vcp1.effectiveMassTangent = 0;


						vcp2.ra = ra2;
						vcp2.rb = rb2;
						vcp2.vpa = info.result.bodyA->velocity() +
							Vector2::crossProduct(info.result.bodyA->angularVelocity(), ra2);
						vcp2.vpb = info.result.bodyB->velocity() +
							Vector2::crossProduct(info.result.bodyB->angularVelocity(), rb2);
						vcp2.effectiveMassNormal = kNormal2;
						vcp2.effectiveMassTangent = 0;


						real v_rel1 = n.dot(vcp1.vpb - vcp1.vpa);
						vcp1.bias = -0.8 * v_rel1;

						real v_rel2 = n.dot(vcp2.vpb - vcp2.vpa);
						vcp2.bias = -0.8 * v_rel2;

						info.points.emplace_back(vcp1);
						info.points.emplace_back(vcp2);
						break;
					}
				}
			}
		}

		void solveVelocity(const real& dt)
		{
			for(int i = 0;i < 2;i++)
			{
				for (auto& info : m_list)
				{
					switch (info.points.size())
					{
					case 1:
					{
						VelocityConstraintPoint* vcp = &info.points[0];
						Vector2 dv = vcp->vpb - vcp->vpa;
						real jv = info.result.normal.dot(dv);
						real jvb = jv + vcp->bias;
						real lambda_n = -vcp->effectiveMassNormal * jvb;
						//lambda_n = -lambda_n;
						
						real newImpulse = Math::max(vcp->accumulatedNormalImpulse - lambda_n, 0);
						lambda_n = newImpulse - vcp->accumulatedNormalImpulse;
						vcp->accumulatedNormalImpulse = newImpulse;
						
						Vector2 impulse_n = lambda_n * info.result.normal;
						info.result.bodyA->applyImpulse(impulse_n, vcp->ra);
						info.result.bodyB->applyImpulse(-impulse_n, vcp->rb);
						break;
					}
					case 2:
					{
						VelocityConstraintPoint* vcp1 = &info.points[0];
						VelocityConstraintPoint* vcp2 = &info.points[1];
						Vector2 dv1 = vcp1->vpb - vcp1->vpa;
						Vector2 dv2 = vcp2->vpb - vcp2->vpa;


						real jv1 = info.result.normal.dot(dv1);
						real jvb1 = jv1 + vcp1->bias;
						real lambda_n1 = -vcp1->effectiveMassNormal * jvb1;
						if (lambda_n1 < 0)
							lambda_n1 = -lambda_n1;

						Vector2 impulse_n1 = lambda_n1 * info.result.normal;
						
						real jv2 = info.result.normal.dot(dv2);
						real jvb2 = jv2 + vcp2->bias;
						real lambda_n2 = -vcp2->effectiveMassNormal * jvb2;
						if (lambda_n2 < 0)
							lambda_n2 = -lambda_n2;

						Vector2 impulse_n2 = lambda_n2 * info.result.normal;
						
							
						info.result.bodyA->applyImpulse(impulse_n1, vcp1->ra);
						info.result.bodyB->applyImpulse(-impulse_n1, vcp1->rb);
						info.result.bodyA->applyImpulse(impulse_n2, vcp2->ra);
						info.result.bodyB->applyImpulse(-impulse_n2, vcp2->rb);
						
						break;
					}
					}
				}
			}
			m_list.clear();
		}

		void solvePosition(const real& dt)
		{
		}

	private:
		std::vector<ContactInfo> m_list;
	};
}
#endif
