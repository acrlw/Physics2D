#ifndef PHYSICS2D_CONTACT_H
#define PHYSICS2D_CONTACT_H
#include "include/math/linear/linear.h"
#include "include/dynamics/body.h"
namespace Physics2D
{
    struct ContactInfo
    {
            Vector2 contactA;
            Vector2 contactB;
            bool isCollide = false;
            Vector2 penetration;
    };
	struct CollisionInfo
	{
        Body* bodyA;
        Body* bodyB;
        ContactInfo info;
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
			for(CollisionInfo& resolve: m_list)
			{

				
			}
			m_list.clear();
		}
	private:
        std::vector<CollisionInfo> m_list;
		real m_allowedPenetration = 0.01f;
		real m_bias = 0.2f;
		real m_iteration = 10;
	};
}
#endif
