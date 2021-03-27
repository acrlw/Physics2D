#ifndef ALCOLLISION_H
#define ALCOLLISION_H

#include <albody.h>
#include <map>
#include <tuple>
#include <QPainter>
///
/// \brief
/// This is the collision detection base class

///
/// \brief
/// This class is used for Minkowski sum calculation
///
struct Minkowski_t
{
    Minkowski_t(const alVector2& a, const alVector2& b): m_A(a), m_B(b), m_result(m_A - m_B)
    {}
    Minkowski_t(){}

public:
    inline bool operator ==(const Minkowski_t& v)
    {
        return m_A == v.m_A && m_B == v.m_B;
    }
    inline bool operator !=(const Minkowski_t& v)
    {
        return !(m_A == v.m_A && m_B == v.m_B);
    }
    alVector2 A() const
    {
        return m_A;
    }

    void setA(const alVector2 &A)
    {
        m_A = A;
    }

    alVector2 B() const
    {
        return m_B;
    }

    void setB(const alVector2 &B)
    {
        m_B = B;
    }

    alVector2 result() const
    {
        return m_result;
    }

    void setResult(const alVector2 &result)
    {
        m_result = result;
    }

private:
    alVector2 m_A, m_B, m_result;
};

struct ContactInfo{

public:
    ContactInfo(const alVector2& contact1, const alVector2& contact2, const alVector2& penetration, const bool isCollide = false):
    m_body1(nullptr), m_body2(nullptr), m_body1ContactPoint(contact1), m_body2ContactPoint(contact2), m_penetrationVector(penetration), m_isCollide(isCollide)
    {

    }
    ContactInfo(): m_body1(nullptr), m_body2(nullptr), m_isCollide(false)
    {

    }
    alVector2 getBody1ContactPoint() const
    {
        return m_body1ContactPoint;
    }

    void setBody1ContactPoint(const alVector2 &body1ContactPoint)
    {
        m_body1ContactPoint = body1ContactPoint;
    }
    alVector2 getBody2ContactPoint() const
    {
        return m_body2ContactPoint;
    }

    void setBody2ContactPoint(const alVector2 &body2ContactPoint)
    {
        m_body2ContactPoint = body2ContactPoint;
    }
    alVector2 getPenetrationVector() const
    {
        return m_penetrationVector;
    }

    void setPenetrationVector(const alVector2 &penetrationVector)
    {
        m_penetrationVector = penetrationVector;
    }


    bool getIsCollide() const
    {
        return m_isCollide;
    }

    void setIsCollide(bool isCollide)
    {
        m_isCollide = isCollide;
    }
    alBody *getBody1()
    {
        return m_body1;
    }

    alBody *getBody2()
    {
        return m_body2;
    }


    void setBody1(alBody *body1)
    {
        m_body1 = body1;
    }

    void setBody2(alBody *body2)
    {
        m_body2 = body2;
    }


private:


    alBody *m_body1;
    alBody *m_body2;
    alVector2 m_body1ContactPoint;
    alVector2 m_body2ContactPoint;
    alVector2 m_penetrationVector;
    bool m_isCollide;
};

class alSimplex{
public:

    std::vector<Minkowski_t>& vertices()
    {
        return m_vertices;
    }

    bool containOrigin();
    alVector2 getLastVertex()const{
        return m_vertices[m_vertices.size() - 1].result();
    }
    void insertVertex(alSimplex &edge, const Minkowski_t &vertex);
private:
    std::vector<Minkowski_t> m_vertices;
};
class alCircleCollisionDetector
{
public:
    alCircleCollisionDetector() {}
    std::pair<bool, alVector2> detect(QPainter *painter, alBody *body1, alBody *body2);
private:

    alVector2 m_minimumPenetration;
    float m_penetrateLength;
};

class alGJKCollisionDetector
{
public:
    alGJKCollisionDetector() {
        m_curveSample = 25;
    }


    ContactInfo detect(QPainter *painter, alBody *body1, alBody *body2) ;
    static alVector2 findFarthestPoint(alBody *body, const alVector2& direction);
    static Minkowski_t support(alBody *body1, alBody *body2, const alVector2& direction);

    static alVector2 getDirection(alSimplex &simplex, bool towardsOrigin);
    static alSimplex findClosestEdge(alSimplex simplex);
    static alSimplex findEdge(alSimplex simplex);
    std::vector<ContactInfo> curveDetection(QPainter *painter, alBody *body, alCurveEdge *curve);
private:

    alVector2 m_minimumPenetration;

    void doEPA(QPainter *painter, const alVector2 &origin, alBody *body1, alBody *body2, alSimplex &simplex);

    bool doGJKDetection(QPainter *painter, alBody *body1, alBody *body2);
    void scaleBody(alBody *body);
    void drawSimplex(const alVector2 &origin, alSimplex &simplex, QPainter *painter, QPen pen);
    alVector2 contact1;
    alVector2 contact2;
    float m_curveSample;
};
class alMPRCollisionDetector
{
public:
    alMPRCollisionDetector(){}
    std::pair<bool, alVector2> detect(QPainter *painter, alBody *body1, alBody *body2);
private:
    void drawMinkowski(alBody *body1, alBody *body2);

    alVector2 m_minimumPenetration;
    float m_penetrateLength;
};

class alSATCollisionDetector
{
public:
    alSATCollisionDetector() {}
    std::pair<bool, alVector2> detect(QPainter *painter, alBody *body1, alBody *body2);

private:
    int polygonSATDetection(alPolygon *p1, alPolygon *p2);
    bool circleSATDetection(alCircle *body1, alPolygon *body2);

    alVector2 m_minimumPenetration;
    float m_penetrateLength;
};

#endif // ALCOLLISION_H
