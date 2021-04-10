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
            World() : m_gravity(0, 9.8), m_linearVelocityDamping(0.25, 0.25), m_angularVelocityDamping(0.25), m_bias(0.8),
    			m_enableGravity(true), m_linearVelocityThreshold(0.02, 0.02), m_angularVelocityThreshold(0.02),
    			m_velocityIteration(6), m_positionIteration(8)
            {}
            World(const Vector2& leftTop, const Vector2& rightBottom) : m_leftTop(leftTop), m_rightBottom(rightBottom), m_gravity(0, 9.8),
    			m_linearVelocityDamping(0.25, 0.25), m_angularVelocityDamping(0.25),
    			m_bias(0.8), m_enableGravity(true),m_linearVelocityThreshold(0.02, 0.02),
    			m_angularVelocityThreshold(0.02), m_velocityIteration(6), m_positionIteration(8)
            {}
            ~World();
            Vector2 screenToWorld(const Vector2& pos)const;
            Vector2 worldToScreen(const Vector2& pos)const;
            void step(const real& dt);

            void setGeometry(const Vector2& leftTop, const Vector2& rightBottom);
            Vector2 leftTop() const;
            void setLeftTop(const Vector2 &leftTop);

            Vector2 rightBottom() const;
            void setRightBottom(const Vector2 &rightBottom);

            Vector2 gravity() const;
            void setGravity(const Vector2 &gravity);

            Vector2 linearVelocityDamping() const;
            void setLinearVelocityDamping(const Vector2 &linearVelocityDamping);

            real angularVelocityDamping() const;
            void setAngularVelocityDamping(const real &angularVelocityDamping);

            Vector2 linearVelocityThreshold() const;
            void setLinearVelocityThreshold(const Vector2 &linearVelocityThreshold);

            real angularVelocityThreshold() const;
            void setAngularVelocityThreshold(const real &angularVelocityThreshold);

            bool enableGravity() const;
            void setEnableGravity(bool enableGravity);

            void addBody(Body* body);
            void removeBody(Body* body);

            real width();
            real height();
    	
            static Vector2 worldToScreen(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos);
            static Vector2 screenToWorld(const Vector2& leftTop, const Vector2& rightBottom, const Vector2& pos);

            std::vector<Body*> bodyList()const;

            real bias() const;
            void setBias(const real &bias);

            real velocityIteration() const;
            void setVelocityIteration(const real &velocityIteration);

            real positionIteration() const;
            void setPositionIteration(const real &positionIteration);

        private:
            Vector2 m_leftTop;
            Vector2 m_rightBottom;

            Vector2 m_gravity;
            Vector2 m_linearVelocityDamping;
            real m_angularVelocityDamping;
            Vector2 m_linearVelocityThreshold;
            real m_angularVelocityThreshold;

            real m_bias;
            real m_velocityIteration;
            real m_positionIteration;
    		
    		bool m_enableGravity;
            std::vector<Body*> m_bodyList;
    		
    };
}
#endif
