#ifndef PHYSICS2D_WORLD_H
#define PHYSICS2D_WORLD_H
#include "include/common/common.h"
#include "include/dynamics/body.h"
#include "include/math/math.h"
#include "include/math/integrator.h"
#include "include/profile/profiler.h"

namespace Physics2D
{
    class World
    {
		public:
            World()
            {
	            
            }
    		Vector2 screenToWorld(const Vector2& pos)const
    		{
                return screenToWorld(m_leftTop, m_rightBottom, pos);
    		}
    		Vector2 worldToScreen(const Vector2& pos)const
            {
                return worldToScreen(m_leftTop, m_rightBottom, pos);
            }
    		void step(const number& dt)
    		{
    			
    		}
			static Vector2 worldToScreen(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos)
			{

			}
			static Vector2 screenToWorld(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos)
			{
				
			}
        private:
			Vector2 m_leftTop;
			Vector2 m_rightBottom;
            Vector2 m_gravity;
            Vector2 m_linearVelocityDamping;
            number m_angularVelocityDamping;
            Vector2 m_linearVelocityThreshold;
            number m_angularVelocityThreshold;
    		bool m_enableGravity;
    		
    };
}
#endif
