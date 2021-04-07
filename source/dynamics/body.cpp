#include "include/dynamics/body.h"

namespace Physics2D {

    void Body::calcInertia()
    {
        switch (m_shape->type()) {
            case Shape::Type::Circle:
            {
                Circle * circle = dynamic_cast<Circle*>(m_shape);

                m_inertia = m_mass * circle->radius() * circle->radius() * 0.5f;
                break;
            }
            case Shape::Type::Polygon:
            {
                Polygon * polygon = dynamic_cast<Polygon*>(m_shape);

                Vector2 center = polygon->center();
                number sum1 = 0.0f;
                number sum2 = 0.0f;

                for (uint32_t i = 0; i < polygon->vertices().size() - 1; i++)
                {
                    Vector2 n1 = polygon->vertices()[i] - center;
                    Vector2 n2 = polygon->vertices()[i + 1] - center;
                    number cross = abs(n1.cross(n2));
                    number dot = n2.dot(n2) + n2.dot(n1) + n1.dot(n1);
                    sum1 += cross * dot;
                    sum2 += cross;
                }

                m_inertia = (m_mass / 6) * sum1 / sum2;
                break;
            }
            case Shape::Type::Ellipse:
            {
                Ellipse * ellipse = dynamic_cast<Ellipse*>(m_shape);

                number a = ellipse->A();
                number b = ellipse->B();
                m_inertia = m_mass * (a*a + b*b) / 4.0f;

                break;
            }
            default:
                break;
        }
    }

}
