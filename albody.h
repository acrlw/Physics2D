#ifndef ALBODY_H
#define ALBODY_H

#include <almath.h>
#include <alsettings.h>
#include <vector>
#include <QDebug>

enum BodyType
{
    None,
    Circle,
    Polygon,
    Wall,
    Ellipse,
    Segment,
    Curve
};


class alBody
{
public:
    alBody(const BodyType type = BodyType::None, const float m = 1.0f, const bool sleep = true, const bool touched = false):
        m_sleep(sleep), m_isTouched(touched), m_mass(m), m_torque(0), m_angle(0),
        m_angularAcceleration(0), m_angularVelocity(0), m_type(type), m_invMass(1.0f / m), m_invInertia(0)
    {
        m_lastAngle = 0;
        m_lastAngularVelocity = 0;
    }

    alVector2& velocity()
    {
        return m_velocity;
    }

    void setVelocity(const alVector2& velocity)
    {
        m_velocity = velocity;
    }

    alVector2& acceleration()
    {
        return m_acceleration;
    }

    void setAcceleration(const alVector2& acceleration)
    {
        m_acceleration = acceleration;
    }

    alVector2& position()
    {
        return m_position;
    }

    void setPosition(const alVector2& position)
    {
        m_position = position;
        if(m_lastPosition.lengthSquare() == 0)
            m_lastPosition = m_position;
    }

    float mass() const
    {
        return m_mass;
    }

    void setMass(float mass)
    {
        m_mass = mass;
        if(mass != 0.0f)
            m_invMass = 1.0f / mass;
    }

    float& angle()
    {
        return m_angle;
    }

    void setAngle(float angle)
    {
        m_angle = angle;
        if(m_lastAngle == 0.0f)
            m_lastAngle = angle;
    }

    float& angularAcceleration()
    {
        return m_angularAcceleration;
    }

    void setAngularAcceleration(float angularAcceleration)
    {
        m_angularAcceleration = angularAcceleration;
    }

    bool sleep() const
    {
        return m_sleep;
    }

    void setSleep(bool sleep)
    {
        m_sleep = sleep;
    }

    float& angularVelocity()
    {
        return m_angularVelocity;
    }

    void setAngularVelocity(float angularVelocity)
    {
        m_angularVelocity = angularVelocity;
    }

    BodyType type() const
    {
        return m_type;
    }

    void setType(const BodyType& type)
    {
        m_type = type;
    }

    bool isTouched() const
    {
        return m_isTouched;
    }

    void setIsTouched(bool isTouched)
    {
        m_isTouched = isTouched;
    }

    float inertia() const
    {
        return m_inertia;
    }

    void setInertia(float inertia)
    {
        m_inertia = inertia;
        if(inertia != 0.0f)
            m_invInertia = 1.0f / inertia;
    }

    float density() const
    {
        return m_density;
    }

    void setDensity(float density)
    {
        m_density = density;
    }

    alVector2 massPosition() const
    {
        return m_massPosition;
    }

    void setMassPosition(const alVector2& massPosition)
    {
        m_massPosition = massPosition;
    }

    float torque() const
    {
        return m_torque;
    }

    void setTorque(float torque)
    {
        m_torque = torque;
        if(torque != 0.0f)
            setAngularAcceleration(invInertia() * m_torque);
    }

    alVector2& forces()
    {
        return m_forces;
    }

    void setForces(const alVector2& forces)
    {
        m_forces = forces;
    }

    void clearForce()
    {
        m_forces.set(0, 0);
    }

    void addForce(const alVector2& force)
    {
        m_forces += force;
    }

    float invMass() const
    {
        return m_invMass;
    }

    float invInertia() const
    {
        return m_invInertia;
    }
    void addForce(const alVector2& force, const alVector2& point)
    {

    }
    void applyImpulse(const alVector2& force, const alVector2& point)
    {
        //translate coordinate system
        alVector2 real_contact = point - m_position;
        alVector2 trans_force = real_contact + force;
        setTorque(alCross2(real_contact, trans_force));
    }

    float aabbLength() const
    {
        return m_AabbLength;
    }

    alVector2& lastPosition()
    {
        return m_lastPosition;
    }

    void setLastPosition(const alVector2 &lastPosition)
    {
        m_lastPosition = lastPosition;
    }


    float& lastAngle()
    {
        return m_lastAngle;
    }

    void setLastAngle(float lastAngle)
    {
        m_lastAngle = lastAngle;
    }

    float& lastAngularVelocity()
    {
        return m_lastAngularVelocity;
    }

    void setLastAngularVelocity(float lastAngularVelocity)
    {
        m_lastAngularVelocity = lastAngularVelocity;
    }

protected:
    bool m_sleep;
    bool m_isTouched;
    float m_mass;
    float m_density;
    float m_inertia;
    float m_torque;
    float m_angle;
    float m_angularAcceleration;
    float m_angularVelocity;
    alVector2 m_massPosition;
    alVector2 m_velocity;
    alVector2 m_acceleration;
    alVector2 m_position;
    alVector2 m_forces;
    std::vector<alVector2> m_vertices;
    float m_radius;
    bool m_isConvex;
    BodyType m_type;
    float m_invMass;
    float m_invInertia;
    float m_AabbLength;
    alVector2 m_lastPosition;
    alVector2 m_lastVelocity;
    float m_lastAngle;
    float m_lastAngularVelocity;
};

class alCircle : public alBody
{
public:
    alCircle(const float radius = 120, const float angle = 0, const float m = 1):
        alBody(BodyType::Circle)
    {
        m_angle = angle;
        m_mass = m;
        m_massPosition = m_position;
        m_inertia = m_mass * m_radius * m_radius * 0.5;
        m_radius = radius;
        calculateAabb();
    }
    void scale(const float& factor)
    {
        m_radius *= factor;
    }
    void setPosition(const alVector2& position)
    {
        m_position = position;
        m_massPosition = m_position;
    }

    float radius() const
    {
        return m_radius;
    }

    void setRadius(float radius)
    {
        m_radius = radius;
        setInertia(m_mass * m_radius * m_radius * 0.5);
        calculateAabb();
    }
    std::vector<alVector2>& getAabb()
    {
        return m_Aabb;
    }
    void calculateAabb()
    {
        m_AabbLength = pow(2 ,2) * 2 * m_radius;
    }
private:
    std::vector<alVector2> m_Aabb;
};

class alPolygon : public alBody
{
public:
    alPolygon():
        alBody(BodyType::Polygon)
    {
        m_isConvex = true;
        m_AabbLength = 0.0f;
    }

    std::vector<alVector2>& vertices()
    {
        return m_vertices;
    }

    void setVertices(const std::vector<alVector2>& vertices)
    {
        m_vertices = vertices;
        calculateAabb();
    }

    /// \brief
    /// return the rotated vertices of polygon
    /// \return
    std::vector<alVector2> getRotatedVertices() const
    {
        std::vector<alVector2> actual;
        foreach(alVector2 v, m_vertices)
        {
            alVector2 va = alRotation(m_angle) * v;
            actual.push_back(va);
        }
        return actual;
    }

    void addVertex(const alVector2& v)
    {
        m_vertices.push_back(v);
        updateMassPosition();
        calculateAabb();
    }
    void scale(const float& factor)
    {
        for(size_t i = 0;i < m_vertices.size(); i++)
            m_vertices[i] *= factor;
    }
    alVector2 triangleGravityPoint(const alVector2& a1, const alVector2& a2, const alVector2& a3) const
    {
        return alVector2(a1 + a2 + a3) / 3;
    }

    float triangleArea(const alVector2& a1, const alVector2& a2, const alVector2& a3) const
    {
        return abs(alCross2(a1 - a2, a1 - a3)) / 2;
    }
    void calculateAabb()
    {
        float x_max = FLT_MAX;
        float y_max = FLT_MAX;
        float x_min = FLT_MIN;
        float y_min = FLT_MIN;
        foreach(auto p, m_vertices)
        {
            if(x_max < p.x())
                x_max = p.x();

            if(x_min > p.x())
                x_min = p.x();


            if(y_max < p.y())
                y_max = p.y();


            if(y_min > p.y())
                y_min = p.y();
        }
        m_AabbLength = sqrt(pow(x_max - x_min, 2) + pow(y_max - y_min, 2));
    }
protected:
    ///
    /// \brief updateMassPosition
    /// update the mass position
    void updateMassPosition()
    {
        if (m_vertices.size() >= 3)
        {
            if (m_type == Polygon)
            {
                alVector2 pos;
                float area = 0;
                for (uint32_t i = 0; i < m_vertices.size() - 1; i++)
                {
                    float a = triangleArea(alVector2(0, 0), m_vertices[i], m_vertices[i + 1]);
                    alVector2 p = triangleGravityPoint(alVector2(0, 0), m_vertices[i], m_vertices[i + 1]);
                    pos += p * a;
                    area += a;
                }
                pos /= area;
                m_massPosition = pos;
                updateInertia();
            }
        }
    }

    ///
    /// \brief
    /// the vertices come from when body stay the origin static status, which means this vertices will not participate the transformation


private:
    void updateInertia()
    {
        float sum1 = 0.0;
        float sum2 = 0.0;
        for (uint32_t i = 0; i < m_vertices.size() - 1; i++)
        {
            alVector2 n1 = m_vertices[i] - m_massPosition;
            alVector2 n2 = m_vertices[i + 1] - m_massPosition;
            float cross = abs(alCross2(n1, n2));
            float dot = n2 * n2 + n2 * n1 + n1 * n1;
            sum1 += cross * dot;
            sum2 += cross;
        }
        setInertia((m_mass / 6) * sum1 / sum2);
    }
};

class alRectangle : public alPolygon
{
public:
    alRectangle(const float width = 50, const float height = 50):
        alPolygon(), m_width(width), m_height(height)
    {
        m_massPosition = m_position;
        updateVertices();
        calculateAabb();
    }

    float width() const
    {
        return m_width;
    }

    void setWidth(float width)
    {
        m_width = width;
        updateVertices();
        calculateAabb();
    }

    float height() const
    {
        return m_height;
    }

    void setHeight(float height)
    {
        m_height = height;
        updateVertices();
        calculateAabb();
    }
    void scale(const float& factor)
    {
        m_width *= factor;
        m_height *= factor;
        updateVertices();
    }
    void updateVertices()
    {
        m_vertices.clear();
        addVertex(alVector2(-m_width / 2, m_height / 2)); 
        addVertex(alVector2(-m_width / 2, -m_height / 2));
        addVertex(alVector2(m_width / 2, -m_height / 2));
        addVertex(alVector2(m_width / 2, m_height / 2));
        addVertex(alVector2(-m_width / 2, m_height / 2));
        m_massPosition = m_position;
        updateInertia();
        calculateAabb();
    }
    void calculateAabb()
    {
        m_AabbLength = sqrt(pow(m_width, 2) + pow(m_height, 2));
    }
protected:
    float m_width;
    float m_height;
private:
    void updateInertia()
    {
        m_inertia = m_mass * (m_width * m_width + m_height * m_height) / 12;
    }

};
class alEllipse : public alBody
{
public:
    alEllipse(const alVector2& topLeft, const alVector2& bottomRight) : m_topLeft(topLeft), m_bottomRight(bottomRight){
        m_type = BodyType::Ellipse;
        m_AabbLength = 0.0f;
        calculateAabb();
    }
    alEllipse()
    {
        m_type = BodyType::Ellipse;
    }
    alVector2 getGeometricCenter()const{
        return alVector2((m_topLeft + m_bottomRight) / 2);
    }
    alVector2 topLeft() const
    {
        return m_topLeft;
    }

    void setTopLeft(const alVector2 &topLeft)
    {
        m_topLeft = topLeft;
        updateInertia();
        calculateAabb();
    }

    alVector2 bottomRight() const
    {
        return m_bottomRight;
    }

    void setBottomRight(const alVector2 &bottomRight)
    {
        m_bottomRight = bottomRight;
        updateInertia();
        calculateAabb();
    }
    void scale(const float& factor)
    {
        m_topLeft *= factor;
        m_bottomRight *= factor;
    }
    float getA()const{
        return abs(m_bottomRight.x() - m_topLeft.x()) / 2;
    }
    float getB()const{
        return abs(m_bottomRight.y() - m_topLeft.y()) / 2;
    }
    float getC()const{
        float a = getA();
        float b = getB();
        return sqrt(a*a - b*b);
    }
    void calculateAabb()
    {
        m_AabbLength = sqrt(pow(2 * getA(), 2) + pow(2 * getB(), 2));
    }
private:
    alVector2 m_topLeft;
    alVector2 m_bottomRight;
    void updateInertia(){
        float a = getA();
        float b = getB();
        float a2 = a * a;
        float b2 = b * b;
        setInertia(m_mass * (a2 + b2) / 4.0f);
    }
};
class alEdge: public alBody
{
public:
    alEdge() {
        m_type = BodyType::Segment;
    }
    void translate(alVector2& offset)
    {
        m_startPoint += offset;
        m_endPoint += offset;
    }
    void translate(const float& offsetX, const float offsetY)
    {
        alVector2 temp(offsetX, offsetY);
        m_startPoint += temp;
        m_endPoint += temp;
    }
    void setStartPoint(const alVector2 &startPoint)
    {
        m_startPoint = startPoint;
    }
    void setEndPoint(const alVector2 &endPoint)
    {
        m_endPoint = endPoint;
    }
    alVector2 startPoint()const
    {
        return m_startPoint;
    }
    alVector2 endPoint()const
    {
        return m_endPoint;
    }
    void scale(const float& factor)
    {
        m_startPoint *= factor;
        m_endPoint *= factor;
    }
    void set(const alVector2& start, const alVector2& end)
    {
        m_startPoint = start;
        m_endPoint = end;
    }

private:
    alVector2 m_startPoint;
    alVector2 m_endPoint;
};
class alCurveEdge: public alEdge
{
public:
    alCurveEdge(){
        m_type = BodyType::Curve;
    }

    alVector2 startPoint() const
    {
        return m_startPoint;
    }

    void setStartPoint(const alVector2 &startPoint)
    {
        m_startPoint = startPoint;
    }

    alVector2 endPoint() const
    {
        return m_endPoint;
    }

    void setEndPoint(const alVector2 &endPoint)
    {
        m_endPoint = endPoint;
    }
    void set(const alVector2& start,const alVector2& control1,const alVector2& control2, const alVector2& end)
    {
        m_startPoint = start;
        m_control1 = control1;
        m_control2 = control2;
        m_endPoint = end;
    }

    void translate(alVector2& offset)
    {
        m_startPoint += offset;
        m_endPoint += offset;
        m_control1 += offset;
        m_control2 += offset;
    }
    void translate(const float& offsetX, const float offsetY)
    {
        alVector2 temp(offsetX, offsetY);
        m_startPoint += temp;
        m_endPoint += temp;
        m_control1 += temp;
        m_control2 += temp;
    }
    void scale(const float& factor)
    {
        m_startPoint *= factor;
        m_endPoint *= factor;
        m_control1 *= factor;
        m_control2 *= factor;
    }
    alVector2 control1() const
    {
        return m_control1;
    }

    void setControl1(const alVector2 &control1)
    {
        m_control1 = control1;
    }

    alVector2 control2() const
    {
        return m_control2;
    }

    void setControl2(const alVector2 &control2)
    {
        m_control2 = control2;
    }


private:
    alVector2 m_startPoint;
    alVector2 m_endPoint;
    alVector2 m_control1;
    alVector2 m_control2;
};
class alWall : public alRectangle
{
public:
    alWall(const float elasticCoefficient = 0.8, const float width = 40): alRectangle(),
        m_elasticCoefficient(elasticCoefficient)
    {
        m_type = Wall;
        m_width = width;
        m_height = width;
        m_massPosition = m_position;
    }

    float elasticCoefficient() const
    {
        return m_elasticCoefficient;
    }

    void setElasticCoefficient(float elasticCoefficient)
    {
        m_elasticCoefficient = elasticCoefficient;
    }

private:
    float m_elasticCoefficient;
};
#endif // ALBODY_H
