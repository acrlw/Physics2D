#ifndef PHYSICS2D_CONTACT_H
#define PHYSICS2D_CONTACT_H
#include "include/math/linear/linear.h"
#include "include/dynamics/body.h"
#include "include/dynamics/constraint/constraint.h"
namespace Physics2D
{
	struct Contact
	{
		bool isColliding = false;
		Vector2 contactA;
		Vector2 contactB;
		Vector2 normal;
		real penetration = 0;
	};
	struct CollisionInfo
	{
        Body* bodyA = nullptr;
        Body* bodyB = nullptr;
        Contact info;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real bias = 0;
		real accumulatedLambdaNormal = 0;
		real accumulatedLambdaTangent = 0;
	};
	class CollisionSolver
	{
	public:
		void add(const CollisionInfo& resolve)
		{
			m_list.emplace_back(resolve);
		}
		void solve(const real& dt)
		{
			for(int i = 0;i < m_iteration;i++)
			{
				for(CollisionInfo& resolve: m_list)
				{
					Vector2 ra = resolve.info.contactA - resolve.bodyA->position();
					Vector2 rb = resolve.info.contactB - resolve.bodyB->position();
					//bodyB.position() += penetration -> separate two body
					Vector2 n = resolve.info.normal;
					Vector2 t = Vector2::crossProduct(n, 1.0);
					
					Vector2 rv = resolve.bodyB->velocity() + Vector2::crossProduct(resolve.bodyB->angularVelocity(), rb) -
						resolve.bodyA->velocity() + Vector2::crossProduct(resolve.bodyA->angularVelocity(), ra);

					real rv_n = rv.dot(resolve.info.normal);
					real lambda_n = resolve.effectiveMassNormal * (rv_n + resolve.bias);
					
					real old_lambda_n = resolve.accumulatedLambdaNormal;
					resolve.accumulatedLambdaNormal = Math::max(old_lambda_n + lambda_n, 0.0);
					lambda_n = resolve.accumulatedLambdaNormal - old_lambda_n;

					Vector2 impulseNormal = lambda_n * resolve.info.normal;
					resolve.bodyA->velocity() += resolve.bodyA->inverseMass() * impulseNormal;
					resolve.bodyA->angularVelocity() += resolve.bodyA->inverseInertia() * ra.cross(impulseNormal);
				}
			}
			m_list.clear();
		}
		void initialize(const real& dt)
		{
			for (CollisionInfo& resolve : m_list)
			{
				Vector2 ra = resolve.info.contactA - resolve.bodyA->position();
				Vector2 rb = resolve.info.contactB - resolve.bodyB->position();
				//bodyA.position() += penetration -> separate two body
				Vector2 n = resolve.info.normal;
				Vector2 t = Vector2::crossProduct(n, 1.0);
				
				real im_a = resolve.bodyA->inverseMass();
				real im_b = resolve.bodyB->inverseMass();
				real ii_a = resolve.bodyA->inverseInertia();
				real ii_b = resolve.bodyB->inverseInertia();

				real ra_n = ra.cross(n);
				real rb_n = rb.cross(n);

				real ra_t = ra.cross(t);
				real rb_t = rb.cross(t);
				//effective mass = 1.0 / k-matrix
				resolve.effectiveMassNormal = 1.0 /
					(im_a + ii_a * ra_n * ra_n + im_b + ii_b * rb_n * rb_n);

				resolve.effectiveMassTangent = 1.0 /
					(im_a + ii_a * ra_t * ra_t + im_b + ii_b * rb_t * rb_t);

				resolve.bias = m_bias * (1.0 / dt) * Math::max(0.0, resolve.info.penetration - m_allowedPenetration);

				
				Vector2 impulseNormal = resolve.accumulatedLambdaNormal * resolve.info.normal;
				resolve.bodyA->velocity() += resolve.bodyA->inverseMass() * impulseNormal;
				resolve.bodyA->angularVelocity() += resolve.bodyA->inverseInertia() * ra.cross(impulseNormal);
			}
		}
	private:
        std::vector<CollisionInfo> m_list;
		real m_allowedPenetration = 0.02;
		real m_bias = 0.2;
		real m_iteration = 1;
	};
}
#endif
