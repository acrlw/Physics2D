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
            {}
            World(const Vector2& leftTop, const Vector2& rightBottom) : m_leftTop(leftTop), m_rightBottom(rightBottom)
            {}
            Vector2 screenToWorld(const Vector2& pos)const;
            Vector2 worldToScreen(const Vector2& pos)const;
            void step(const number& dt);

            void setGeometry(const Vector2& leftTop, const Vector2& rightBottom);
            Vector2 leftTop() const;
            void setLeftTop(const Vector2 &leftTop);

            Vector2 rightBottom() const;
            void setRightBottom(const Vector2 &rightBottom);

            Vector2 gravity() const;
            void setGravity(const Vector2 &gravity);

            Vector2 linearVelocityDamping() const;
            void setLinearVelocityDamping(const Vector2 &linearVelocityDamping);

            number angularVelocityDamping() const;
            void setAngularVelocityDamping(const number &angularVelocityDamping);

            Vector2 linearVelocityThreshold() const;
            void setLinearVelocityThreshold(const Vector2 &linearVelocityThreshold);

            number angularVelocityThreshold() const;
            void setAngularVelocityThreshold(const number &angularVelocityThreshold);

            bool enableGravity() const;
            void setEnableGravity(bool enableGravity);

            void addBody(Body* body);
            void removeBody(Body* body);

            number width();
            number height();
    	
            static Vector2 worldToScreen(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos);
            static Vector2 screenToWorld(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos);
        private:
            Vector2 m_leftTop;
            Vector2 m_rightBottom;
    	
            Vector2 m_gravity;
            Vector2 m_linearVelocityDamping;
            number m_angularVelocityDamping;
            Vector2 m_linearVelocityThreshold;
            number m_angularVelocityThreshold;
    		bool m_enableGravity;
            std::vector<Body*> m_bodyList;
    		
    };
}
#endif
