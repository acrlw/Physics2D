#ifndef PHYSICS2D_INCLUDE_COLLISION_SOLVER_H
#define PHYSICS2D_INCLUDE_COLLISION_SOLVER_H
#include "detector.h"
namespace Physics2D
{
	struct SolverData
	{
		Collision info;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real bias = 0;
		real accumulatedNormal = 0;
		real accumulatedTangent = 0;
		Vector2 ra;
		Vector2 rb;
	};
	class CollisionSolver
	{
	public:
		void add(const Collision& info)
		{
			m_list.emplace_back(info);
		}
		void prepare(const real& dt)
		{
			for (Collision& info : m_list)
			{
				auto& contact = info.contactList[0];
				for (auto& contact : info.contactList)
				{
					Vector2 ra = contact.pointA - info.bodyA->position();
					Vector2 rb = contact.pointB - info.bodyB->position();
					Vector2 normal = info.normal;
					Vector2 tangent = Vector2::crossProduct(normal, 1.0);

					real im_a = info.bodyA->inverseMass();
					real im_b = info.bodyB->inverseMass();
					real ii_a = info.bodyA->inverseInertia();
					real ii_b = info.bodyB->inverseInertia();

					real ra_n = ra.cross(normal);
					real rb_n = rb.cross(normal);

					real ra_t = ra.cross(tangent);
					real rb_t = rb.cross(tangent);
					//effective mass = 1.0 / k-matrix
					SolverData data;
					data.bias = -m_bias * (1.0 / dt) * Math::min(0, m_allowedPenetration - info.penetration);
					//data.bias = 0;
					data.effectiveMassNormal = 1.0 /
						(im_a + ii_a * ra_n * ra_n + im_b + ii_b * rb_n * rb_n);

					data.effectiveMassTangent = 1.0 /
						(im_a + ii_a * ra_t * ra_t + im_b + ii_b * rb_t * rb_t);

					data.info = info;
					data.ra = ra;
					data.rb = rb;

					m_dataList.emplace_back(data);
				}
			}
		}
		void solveVelocityConstraint(const real& dt)
		{
			
		}
		void solvePositionConstraint(const real& dt)
		{
			
		}
		void clear()
		{
			m_dataList.clear();
			m_list.clear();
		}
		void solve(const real& dt)
		{
			for(int i = 0;i < m_iteration;i++)
			{
				for (SolverData& data : m_dataList)
				{
					//bodyB.position() += penetration -> separate two body
					Vector2 vB = data.info.bodyB->velocity() + Vector2::crossProduct(data.info.bodyB->angularVelocity(), data.rb);
					Vector2 vA = data.info.bodyA->velocity() + Vector2::crossProduct(data.info.bodyA->angularVelocity(), data.ra);
					Vector2 rv = vB - vA;
					Vector2 tangent = Vector2::crossProduct(data.info.normal, 1.0);
					
					real rv_n = rv.dot(data.info.normal);
					real rv_t = rv.dot(tangent);
					real lambda_n = data.effectiveMassNormal * (rv_n + data.bias);
					
					real old_lambda_n = data.accumulatedNormal;
					data.accumulatedNormal = Math::max(lambda_n + old_lambda_n, 0);
					lambda_n = data.accumulatedNormal - old_lambda_n;


					real lambda_t = data.effectiveMassTangent * (-rv_t);
					real max_lambda_t = data.info.bodyA->damping() * data.accumulatedNormal;
					// Clamp friction
					real old_lambda_t = data.accumulatedTangent;
					data.accumulatedTangent = Math::clamp(lambda_t + old_lambda_t, -max_lambda_t, max_lambda_t);
					lambda_t = data.accumulatedTangent - old_lambda_t;

					//real lambda_t = 0;
					
					Vector2 impulseNormal = lambda_n * data.info.normal;
					Vector2 impulseTangent = lambda_t * tangent;
					Vector2 dv = data.info.bodyA->inverseMass() * (impulseNormal + impulseTangent);
					real dw = data.info.bodyA->inverseInertia() * data.ra.cross(impulseNormal + impulseTangent);

					data.info.bodyA->applyImpulse(impulseNormal, data.ra);
					data.info.bodyB->applyImpulse(-impulseNormal, data.rb);

					vB = data.info.bodyB->velocity() + Vector2::crossProduct(data.info.bodyB->angularVelocity(), data.rb);
					vA = data.info.bodyA->velocity() + Vector2::crossProduct(data.info.bodyA->angularVelocity(), data.ra);
					rv = vB - vA; 
				}
			}

			m_dataList.clear();
			m_list.clear();
		}
	private:
		std::vector<Collision> m_list;
		std::vector<SolverData> m_dataList;
		real m_allowedPenetration = 0.01;
		real m_bias = 0.2;
		int m_iteration = 8;
		
	};
}
#endif