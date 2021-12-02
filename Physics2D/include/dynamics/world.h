#ifndef PHYSICS2D_WORLD_H
#define PHYSICS2D_WORLD_H
#include "../common/common.h"
#include "../dynamics/body.h"
#include "../math/math.h"
#include "../math/integrator.h"
#include "../dynamics/joint/joints.h"
#include "../utils/random.h"
#include "../dynamics/constraint/contact.h"
namespace Physics2D
{
    class PhysicsWorld
    {
		public:
            PhysicsWorld() : m_gravity(0, -1), m_linearVelocityDamping(0.9f), m_angularVelocityDamping(0.9f), m_bias(0.8f),
    			m_enableGravity(true), m_linearVelocityThreshold(0.02f), m_angularVelocityThreshold(0.02f), m_airFrictionCoefficient(0.7f),
    			m_velocityIteration(1), m_positionIteration(1)
            {}
            ~PhysicsWorld();
            void prepareVelocityConstraint(const real& dt);
            void stepVelocity(const real& dt);
            void solveVelocityConstraint(real dt);
            void stepPosition(const real& dt);
            void solvePositionConstraint(real dt);
            

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

            bool enableDamping() const;
            void setEnableDamping(bool enableDamping);
            
            Body* createBody();
            void removeBody(Body* body);

            void removeJoint(Joint* joint);

            void clearAllBodies();
            void clearAllJoints();
    	
            RotationJoint* createJoint(const RotationJointPrimitive& primitive);
            PointJoint* createJoint(const PointJointPrimitive& primitive);
            DistanceJoint* createJoint(const DistanceJointPrimitive& primitive);
            PulleyJoint* createJoint(const PulleyJointPrimitive& primitive);
            RevoluteJoint* createJoint(const RevoluteJointPrimitive& primitive);
            OrientationJoint* createJoint(const OrientationJointPrimitive& primitive);
            
            real bias() const;
            void setBias(const real &bias);

            int velocityIteration() const;
            void setVelocityIteration(const int&velocityIteration);

            int positionIteration() const;
            void setPositionIteration(const int&positionIteration);

            std::vector<std::unique_ptr<Body>>& bodyList();
    	
            std::vector<std::unique_ptr<Joint>>& jointList();
        private:

            Vector2 m_gravity;
            real m_linearVelocityDamping;
            real m_angularVelocityDamping;
            real m_linearVelocityThreshold;
            real m_angularVelocityThreshold;
            real m_airFrictionCoefficient;

            real m_bias;
            int m_velocityIteration;
            int m_positionIteration;
    		
    		bool m_enableGravity = true;
    		bool m_enableDamping = true;
            std::vector<std::unique_ptr<Body>> m_bodyList;
            std::vector<std::unique_ptr<Joint>> m_jointList;

    		
    		
    };
    
}
#endif
