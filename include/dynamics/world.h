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
            World() : m_gravity(0, -0.15), m_linearVelocityDamping(0.9), m_angularVelocityDamping(0.9), m_bias(0.8),
    			m_enableGravity(true), m_linearVelocityThreshold(0.02), m_angularVelocityThreshold(0.02), m_airFrictionCoefficient(0.7),
    			m_velocityIteration(6), m_positionIteration(8)
            {}
            World(const Vector2& leftTop, const Vector2& rightBottom) : m_leftTop(leftTop), m_rightBottom(rightBottom), m_gravity(0, -0.15),
    			m_linearVelocityDamping(0.9), m_angularVelocityDamping(0.9), m_airFrictionCoefficient(0.7),
    			m_bias(0.8), m_enableGravity(true),m_linearVelocityThreshold(0.02),
    			m_angularVelocityThreshold(0.02), m_velocityIteration(6), m_positionIteration(8)
            {}
            ~World();
            Vector2 screenToWorld(const Vector2& pos)const;
            Vector2 worldToScreen(const Vector2& pos)const;
            void stepVelocity(const real& dt);
            void stepPosition(const real& dt);
            void step(const real& dt);

            void setGeometry(const Vector2& leftTop, const Vector2& rightBottom);
            Vector2 leftTop() const;
            void setLeftTop(const Vector2 &leftTop);

            Vector2 rightBottom() const;
            void setRightBottom(const Vector2 &rightBottom);

            Vector2 gravity() const;
            void setGravity(const Vector2 &gravity);

            real linearVelocityDamping() const;
            void setLinearVelocityDamping(const real&linearVelocityDamping);

            real angularVelocityDamping() const;
            void setAngularVelocityDamping(const real &angularVelocityDamping);

            real linearVelocityThreshold() const;
            void setLinearVelocityThreshold(const real&linearVelocityThreshold);

            real angularVelocityThreshold() const;
            void setAngularVelocityThreshold(const real &angularVelocityThreshold);

            real airFrictionCoefficient()const;
    		void setAirFrictionCoefficient(const real& airFrictionCoefficient);

            bool enableGravity() const;
            void setEnableGravity(bool enableGravity);

            void addBody(Body* body);
            void removeBody(Body* body);
            Body* createBody();

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

            Integrator integrator()const;
            void setIntegrator(const Integrator& integrator);


        private:
            Vector2 m_leftTop;
            Vector2 m_rightBottom;

            Vector2 m_gravity;
            real m_linearVelocityDamping;
            real m_angularVelocityDamping;
            real m_linearVelocityThreshold;
            real m_angularVelocityThreshold;
            real m_airFrictionCoefficient;

            real m_bias;
            real m_velocityIteration;
            real m_positionIteration;
    		
    		bool m_enableGravity;
            std::vector<Body*> m_bodyList;
            Integrator m_integrator;

    		
    		
    };
}
#endif
