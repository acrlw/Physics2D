#include "alcollision.h"



std::pair<bool, alVector2> alCircleCollisionDetector::detect(QPainter *painter, alBody *body1, alBody *body2)
{
    bool result = false;
    if(body1 == nullptr || body2 == nullptr)
        return std::pair<bool, alVector2>(result, alVector2());
    alCircle * circle1 = static_cast<alCircle*>(body1);
    alCircle * circle2 = static_cast<alCircle*>(body2);

    alVector2 dp = circle1->position() - circle2->position();
    float dps = dp.length();
    float distance = circle1->radius() + circle2->radius();
    if(dps > distance * 1.2)
        result = false;
    if(dps <= distance)
    {
        dp.normalize();
        dp *= dps - distance;
        m_penetrateLength = dp.length();
        m_minimumPenetration = dp;
        result = true;
    }
    return std::pair<bool, alVector2>(result, alVector2());
}

std::pair<bool, alVector2> alSATCollisionDetector::detect(QPainter *painter, alBody *body1, alBody *body2)
{
    bool result = false;
    m_penetrateLength = 0;
    m_minimumPenetration.set(0, 0);

    if(body1 == nullptr || body2 == nullptr)
        return std::pair<bool, alVector2>(result, alVector2());


    if(body1->type() == BodyType::Circle)
        result = circleSATDetection(static_cast<alCircle*>(body1), static_cast<alPolygon*>(body2));

    else if(body2->type() == BodyType::Circle)
        result = circleSATDetection(static_cast<alCircle*>(body2), static_cast<alPolygon*>(body1));
    else
    {
        alPolygon * polygon1 = static_cast<alPolygon*>(body1);
        alPolygon * polygon2 = static_cast<alPolygon*>(body2);
        uint32_t contact1 = polygonSATDetection(polygon1, polygon2);
        uint32_t contact2 = polygonSATDetection(polygon2, polygon1);
        if(contact1 == polygon1->vertices().size() - 1 && contact2 == polygon2->vertices().size() - 1)
        {
            alVector2 positionDirection = polygon2->position() - polygon1->position();
            if(m_minimumPenetration * positionDirection / abs(m_minimumPenetration * positionDirection) == 1)
                m_minimumPenetration *= -1;
            result = true;
        }
    }
    return std::pair<bool, alVector2>(result, m_minimumPenetration);
}


int alSATCollisionDetector::polygonSATDetection(alPolygon *p1, alPolygon *p2)
{
    std::vector<alVector2> b1vertices = p1->getRotatedVertices();
    std::vector<alVector2> b2vertices = p2->getRotatedVertices();
    alVector2 relativePosition = p2->position() - p1->position();
    int contactAxis = 0;
    float shortestLength = 0;
    alVector2 shortestST;
    alVector2 shortestED;
    for(uint32_t i = 0; i < b1vertices.size() - 1; i++)
    {
        alVector2 edge = b1vertices[i + 1] - b1vertices[i];
        alVector2 perpendicular = alVector2(edge.y(), -edge.x()).getNormalizedVector();

        float b1_min = (b1vertices[0]) * (perpendicular), b1_max = (b1vertices[0]) * (perpendicular);
        for(uint32_t j = 0;j < b1vertices.size(); j++)
        {
            float temp = (b1vertices[j]) * (perpendicular);

            if(b1_min > temp)
                b1_min = temp;
            if(b1_max < temp)
                b1_max = temp;
        }
        float b2_min = (b2vertices[0] + relativePosition) * (perpendicular), b2_max = (b2vertices[0] + relativePosition) * (perpendicular);
        for(uint32_t j = 0;j < b2vertices.size(); j++)
        {
            float temp = (b2vertices[j] + relativePosition) * (perpendicular);



            if(b2_min > temp)
                b2_min = temp;
            if(b2_max < temp)
                b2_max = temp;
        }

        alVector2 minVector1 = perpendicular * b1_min;
        alVector2 maxVector1 = perpendicular * b1_max;
        alVector2 minVector2 = perpendicular * b2_min;
        alVector2 maxVector2 = perpendicular * b2_max;

        float dt1 = b1_max - b2_min;
        float dt2 = b2_max - b1_min;
        float min = abs(dt1) > abs(dt2) ? dt2 : dt1;

        if((b2_min > b1_min && b2_min < b1_max) || (b2_max > b1_min && b2_max < b1_max) ||
                (b1_min > b2_min && b1_max < b2_max) || (b2_min > b1_min && b1_max > b2_max))
            contactAxis++;


        if(i == 0)
        {
            shortestLength = abs(min);
            if(abs(dt1) > abs(dt2))
            {
                shortestST = alVector2(maxVector2.x(), maxVector2.y());
                shortestED = alVector2(minVector1.x(), minVector1.y());
            }
            else
            {
                shortestST = alVector2(maxVector1.x(), maxVector1.y());
                shortestED = alVector2(minVector2.x(), minVector2.y());
            }
        }
        else
        {
            if(shortestLength > abs(dt2))
            {
                shortestLength = abs(dt2);
                shortestST = alVector2(maxVector2.x(), maxVector2.y());
                shortestED = alVector2(minVector1.x(), minVector1.y());
            }
            if(shortestLength > abs(dt1))
            {
                shortestLength = abs(dt1);
                shortestST = alVector2(maxVector1.x(), maxVector1.y());
                shortestED = alVector2(minVector2.x(), minVector2.y());
            }
        }
    }
    if(m_penetrateLength == 0)
    {
        m_minimumPenetration = shortestED - shortestST;
        m_penetrateLength = shortestLength;
    }
    else
    {
        if(m_penetrateLength > shortestLength)
        {
            m_minimumPenetration = shortestED - shortestST;
            m_penetrateLength = shortestLength;
        }
    }
    return contactAxis;
}

bool alSATCollisionDetector::circleSATDetection(alCircle *body1, alPolygon *body2)
{
    if(body1 == nullptr || body2 == nullptr)
        return false;
    ///Attention the body order!
    alPolygon polygon = *body2;
    alCircle circle = *body1;
    alVector2 relativePosition = circle.position() - polygon.position();
    std::vector<alVector2> b1vertices = polygon.getRotatedVertices();
    uint32_t contactAxis = 0;
    float minimumDistance = 0;
    uint32_t minimumIndex = 0;
    polygon.position().set(0, 0);
    circle.position() = polygon.position() + relativePosition;
    float shortestLength = 0;
    alVector2 shortestST;
    alVector2 shortestED;
    for(uint32_t i = 0; i < b1vertices.size() - 1; i++)
    {
        alVector2 edge = b1vertices[i + 1] - b1vertices[i];
        alVector2 perpendicular = alVector2(edge.y(), -edge.x()).getNormalizedVector();
        float b1_min = (b1vertices[0]) * (perpendicular), b1_max = (b1vertices[0]) * (perpendicular);
        for(uint32_t j = 0;j < b1vertices.size(); j++)
        {
            float temp = (b1vertices[j]) * (perpendicular);
            if(b1_min > temp)
                b1_min = temp;
            if(b1_max < temp)
                b1_max = temp;
        }
        //draw min max point
        alVector2 minVector1 = perpendicular * b1_min;
        alVector2 maxVector1 = perpendicular * b1_max;
        //draw end

        float project = perpendicular * circle.position();
        float b2_min = project - circle.radius();
        float b2_max = project + circle.radius();

        //draw min max point of circle projection
        alVector2 minVector2 = perpendicular * b2_min;
        alVector2 maxVector2 = perpendicular * b2_max;


        if((b2_min > b1_min && b2_min < b1_max) || (b2_max > b1_min && b2_max < b1_max) ||
                (b1_min > b2_min && b1_max < b2_max) || (b2_min > b1_min && b1_max > b2_max))
            contactAxis++;
        float dt1 = b1_max - b2_min;
        float dt2 = b2_max - b1_min;
        float min = abs(dt1) > abs(dt2) ? dt2 : dt1;



        if(i == 0)
        {
            shortestLength = abs(min);
            if(abs(dt1) > abs(dt2))
            {
                shortestST = alVector2(maxVector2.x(), maxVector2.y());
                shortestED = alVector2(minVector1.x(), minVector1.y());
            }
            else
            {
                shortestST = alVector2(maxVector1.x(), maxVector1.y());
                shortestED = alVector2(minVector2.x(), minVector2.y());
            }
        }
        else
        {
            if(shortestLength > abs(dt2))
            {
                shortestLength = abs(dt2);
                shortestST = alVector2(maxVector2.x(), maxVector2.y());
                shortestED = alVector2(minVector1.x(), minVector1.y());
            }
            if(shortestLength > abs(dt1))
            {
                shortestLength = abs(dt1);
                shortestST = alVector2(maxVector1.x(), maxVector1.y());
                shortestED = alVector2(minVector2.x(), minVector2.y());
            }
        }
    }
    //circle projection
    for(uint32_t i = 0; i < b1vertices.size(); i++)
    {
        float temp = (b1vertices[i] - circle.position()).length();
        if(minimumDistance == 0)
        {
            minimumDistance = temp;
            minimumIndex = 0;
        }
        if(minimumDistance > temp)
        {
            minimumIndex = i;
            minimumDistance = temp;
        }
    }
    //projection axis of circle
    alVector2 edge = b1vertices[minimumIndex] - circle.position();
    edge.normalize();

    float b1_min = (b1vertices[0]) * (edge), b1_max = (b1vertices[0]) * (edge);
    for(uint32_t j = 0;j < b1vertices.size(); j++)
    {
        float temp = (b1vertices[j]) * (edge);

        if(b1_min > temp)
            b1_min = temp;
        if(b1_max < temp)
            b1_max = temp;
    }

    alVector2 minVector1 = edge * b1_min;
    alVector2 maxVector1 = edge * b1_max;

    float project = edge * circle.position();
    float b2_min = project - circle.radius();
    float b2_max = project + circle.radius();

    alVector2 minVector2 = edge * b2_min;
    alVector2 maxVector2 = edge * b2_max;



    float dt1 = b1_max - b2_min;
    float dt2 = b2_max - b1_min;
    float min = abs(dt1) > abs(dt2) ? dt2 : dt1;
    if((b2_min > b1_min && b2_min < b1_max) || (b2_max > b1_min && b2_max < b1_max))
        contactAxis++;


    if(shortestLength == 0)
    {
        shortestLength = abs(min);
        if(abs(dt1) > abs(dt2))
        {
            shortestST = alVector2(maxVector2.x(), maxVector2.y());
            shortestED = alVector2(minVector1.x(), minVector1.y());
        }
        else
        {
            shortestST = alVector2(maxVector1.x(), maxVector1.y());
            shortestED = alVector2(minVector2.x(), minVector2.y());
        }
    }
    else
    {
        if(shortestLength > abs(dt2))
        {
            shortestLength = abs(dt2);
            shortestST = alVector2(maxVector2.x(), maxVector2.y());
            shortestED = alVector2(minVector1.x(), minVector1.y());
        }
        if(shortestLength > abs(dt1))
        {
            shortestLength = abs(dt1);
            shortestST = alVector2(maxVector1.x(), maxVector1.y());
            shortestED = alVector2(minVector2.x(), minVector2.y());
        }
    }
    if(contactAxis == polygon.vertices().size()){
        m_minimumPenetration = shortestED - shortestST;
        alVector2 positionDirection = body1->position() - body2->position();
        if(m_minimumPenetration * positionDirection / abs(m_minimumPenetration * positionDirection) == 1)
            m_minimumPenetration *= -1;
        m_penetrateLength = m_minimumPenetration.length();
        return true;
    }
    return false;
}
std::vector<ContactInfo> alGJKCollisionDetector::curveDetection(QPainter* painter, alBody *body, alCurveEdge *curve)
{
    QPen p_edge(Qt::darkGreen, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p_p(Qt::gray, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    std::vector<ContactInfo> result;
    m_minimumPenetration.set(0, 0);
    alEdge temp;
    temp.setStartPoint(curve->startPoint());
    for(float t = 0, i = 0; i <= m_curveSample; t += 1 / m_curveSample, i++)
    {
        alVector2 point = pow((1.0f - t), 3) * curve->startPoint() + 3 * t * pow(1.0f - t, 2) * curve->control1() + 3 * pow(t, 2) * (1.0f - t) * curve->control2() + pow(t, 3) * curve->endPoint();
        temp.setEndPoint(point);
//        painter->setPen(p_edge);
//        painter->drawLine(temp.startPoint().toPointF(), temp.endPoint().toPointF());
//        painter->setPen(p_p);
//        painter->drawPoint(point.toPointF());
        bool isCollide = doGJKDetection(painter, body, &temp);
        ContactInfo info(contact1, contact2, m_minimumPenetration, isCollide);
        info.setBody1(body);
        info.setBody2(curve);
        result.push_back(info);
        temp.setStartPoint(point);
    }
    return result;
}
ContactInfo alGJKCollisionDetector::detect(QPainter* painter, alBody *body1, alBody *body2)
{
    ContactInfo result;
    result.setBody1(body1);
    result.setBody2(body2);
    m_minimumPenetration.set(0, 0);

    if(body1 == nullptr || body2 == nullptr)
        return result;


    bool isCollide = doGJKDetection(painter, body1, body2);
    return ContactInfo(contact1, contact2, m_minimumPenetration, isCollide);

}

alVector2 alGJKCollisionDetector::findFarthestPoint(alBody *body, const alVector2 &direction)
{
    alVector2 maxVector;
    switch (body->type()) {
    case BodyType::Polygon:
    {
        std::vector<alVector2> vertices = static_cast<alPolygon*>(body)->getRotatedVertices();
        float max = 0.0f;
        foreach(alVector2 vertex, vertices)
        {
            if(max == 0.0f)
            {
                max = vertex * direction;
                maxVector = vertex;
            }
            else
            {
                if(max < vertex * direction){
                    max = vertex * direction;
                    maxVector = vertex;
                }
            }
        }
        break;
    }
    case BodyType::Circle:
    {
        alCircle * circle = static_cast<alCircle*>(body);
        maxVector = direction.getNormalizedVector() * circle->radius();
        break;
    }
    case BodyType::Ellipse:
    {
        alEllipse * ellipse = static_cast<alEllipse*>(body);
        alRotation r_matrix(-1 * ellipse->angle());
        float a = ellipse->getA();
        float b = ellipse->getB();
        if(direction.x() == 0)
        {
            float sgn = direction.y() < 0 ? -1.0f : 1.0f;
            maxVector.set(0, sgn * b);
        }
        else if(direction.y() == 0)
        {
            float sgn = direction.x() < 0 ? -1.0f : 1.0f;
            maxVector.set(sgn * a, 0);
        }
        else
        {
            alVector2 dir = r_matrix * direction;
            float k = dir.y() / dir.x();
            //line offset constant d
            float a2 = pow(a, 2);
            float b2 = pow(b, 2);
            float k2 = pow(k, 2);
            float d = sqrt((a2 + b2 * k2) / k2);
            float x1, y1;
            if(alVector2(0, d) * dir < 0)
                d = d * -1;
            x1 = k*d - (b2 * k2*k * d)/(a2 + b2 * k2);
            y1 = (b2 * k2 * d)/(a2 + b2 * k2);
            maxVector.set(x1, y1);
            //rotate the final vector
            r_matrix.setAngle(ellipse->angle());
        }
        maxVector = r_matrix * maxVector;
        break;
    }
    case BodyType::Segment:
    {
        alEdge * edge = static_cast<alEdge*>(body);
        float dot1, dot2;
        dot1 = edge->startPoint() * direction;
        dot2 = edge->endPoint() * direction;
        maxVector = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
    }
    case BodyType::Curve:
    {

    }
    default:
        break;
    }

    return maxVector;
}

Minkowski_t alGJKCollisionDetector::support(alBody *body1, alBody *body2, const alVector2 &direction)
{
    alVector2 p1 = findFarthestPoint(body1, direction) + body1->position();
    alVector2 p2 = findFarthestPoint(body2, direction * -1) + body2->position();
    return Minkowski_t(p1, p2);
}


bool alGJKCollisionDetector::doGJKDetection(QPainter * painter, alBody *body1, alBody *body2)
{
    alSimplex simplex;
    //20210306: copy assignment operation causes  information loss: infomation of ellipse vertex
    alVector2 pos1 = body1->position();
    alVector2 pos2 = body2->position();
    alVector2 direction = pos2 - pos1;
    body1->position().set(0, 0);
    body2->position() = direction;
    //alBody rigidBody1 = *body1;
    //alBody rigidBody2 = *body2;
    //rigidBody1.position().set(0, 0);
    //rigidBody2.position() = direction;

    //start gjk
    //scale vertices in order to optimize the epa iteration result
    //scaleBody(&rigidBody1);
    //scaleBody(&rigidBody2);

    int iteration = 0;
    Minkowski_t result = support(body1, body2, direction);
    simplex.vertices().push_back(result);
    direction.negate();
    //iteration start
    while(iteration <= alGJKIteration)
    {
        result = support(body1, body2, direction);

        simplex.vertices().push_back(result);

        if (simplex.getLastVertex() * direction <= 0) {
            body1->position() = pos1;
            body2->position() = pos2;
            return false;
        } else {
            if(simplex.containOrigin())
            {
                simplex.vertices().push_back(simplex.vertices()[0]);
                doEPA(painter, pos1, body1, body2, simplex);
                body1->position() = pos1;
                body2->position() = pos2;
                return true;
            }
            else
            {
                direction = getDirection(simplex, true);
            }

        }
        iteration++;
    }
    body1->position() = pos1;
    body2->position() = pos2;
    return false;
}

void alGJKCollisionDetector::doEPA(QPainter* painter,const alVector2& origin, alBody *body1, alBody *body2, alSimplex &simplex)
{
    QPen p_origin(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p_edge(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p_finalSimplex(Qt::darkGreen, 1, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p_A(Qt::red, 8, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    QPen p_B(Qt::blue, 8, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    //drawSimplex(origin, simplex, painter, p_origin);
    BodyType body1_type = body1->type();
    BodyType body2_type = body2->type();
    alSimplex edge;
    alVector2 normal, witness, mirror;
    alVector2 A_s1, A_s2, B_s1, B_s2;
    Minkowski_t p;
    float originToEdge;
    if(simplex.vertices().size() == 4)
    {
        //epa iteration start
        uint32_t iteration = 0;
        while(iteration <= alEPAIteration)
        {
            edge = findEdge(simplex);


            normal = getDirection(edge, false).getNormalizedVector();

            originToEdge = abs(normal * edge.vertices()[0].result());
            p = support(body1, body2, normal);

            if(body1_type != BodyType::Polygon || body2_type != BodyType::Polygon)
            {

                float d = p.result() * normal;

                float difference = d - originToEdge;
                if(abs(difference) < alEPAEpsilon)
                {
                    m_minimumPenetration = normal * originToEdge * -1;

                    A_s1 = edge.vertices()[0].A();
                    A_s2 = edge.vertices()[1].A();
                    B_s1 = edge.vertices()[0].B();
                    B_s2 = edge.vertices()[1].B();
                    int dir = 1;
                    if((A_s1 - A_s2).lengthSquare() < (B_s1 - B_s2).lengthSquare())
                    {
                        witness = (A_s1 + A_s2) / 2;
                    }
                    else
                    {
                        witness = (B_s1 + B_s2) / 2;
                        dir = dir * -1;
                    }
                    A_s1 += origin;
                    A_s2 += origin;
                    B_s1 += origin;
                    B_s2 += origin;
                    witness += origin;
                    mirror = witness + m_minimumPenetration * dir;
                    //QPen ed(Qt::blue, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                    //painter->setPen(ed);
                    QPolygonF points;
                    points << A_s1.toPointF() << A_s2.toPointF();
                    painter->setPen(p_A);
                    painter->drawPoints(points);
                    points.clear();
                    points << B_s1.toPointF() << B_s2.toPointF();
                    painter->setPen(p_B);
                    painter->drawPoints(points);
                    painter->setPen(p_edge);
                    alVector2 o = origin;
                    body1->position();
                    painter->drawLine(o.toPointF(), o.toPointF() - m_minimumPenetration.toPointF());
                    //painter->setPen(p_origin);

                    //painter->drawLine(witness.toPointF(), mirror.toPointF());
                	

                    drawSimplex(origin, edge, painter, p_edge);

                    drawSimplex(origin, simplex, painter, p_finalSimplex);
                    contact1 = mirror;
                    contact2 = witness;
                    return;
                }
            }
            //optimization
            //float even = sqrt(d * originToEdge);
            //check if we have already saved the same Minkowski Difference
            bool isExisted = false;
            foreach(Minkowski_t v, simplex.vertices())
            {
                if(v == p)
                {
                    isExisted = true;
                    break;
                }
            }
            if(isExisted)
            {

                m_minimumPenetration = normal * originToEdge * -1;

                A_s1 = edge.vertices()[0].A();
                A_s2 = edge.vertices()[1].A();
                B_s1 = edge.vertices()[0].B();
                B_s2 = edge.vertices()[1].B();
                int dir = 1;
                if((A_s1 - A_s2).lengthSquare() < (B_s1 - B_s2).lengthSquare())
                {
                    witness = (A_s1 + A_s2) / 2;
                }
                else
                {
                    witness = (B_s1 + B_s2) / 2;
                    dir = dir * -1;
                }
                A_s1 += origin;
                A_s2 += origin;
                B_s1 += origin;
                B_s2 += origin;
                witness += origin;
                mirror = witness + m_minimumPenetration * dir;
                if(dir < 0)
                {
                    alVector2 temp = witness;
                    witness = mirror;
                    mirror = temp;
                }
            	
                QPen w(Qt::blue, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                painter->setPen(w);
                painter->drawPoint(witness.toPointF());
                QPen m(Qt::blue, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                painter->setPen(m);
                painter->drawPoint(mirror.toPointF());
                painter->setPen(p_origin);
                painter->drawLine(witness.toPointF(), mirror.toPointF());

                contact1 = witness;
                contact2 = mirror;
                
                drawSimplex(origin, edge, painter, p_edge);


                return;
            }
            else{
                simplex.insertVertex(edge, p);
            }

            iteration++;
        }

    }
}

void alGJKCollisionDetector::drawSimplex(const alVector2 &origin, alSimplex& simplex, QPainter* painter, QPen pen)
{
    QPen p_point(Qt::gray, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	
    if(simplex.vertices().size() >= 2)
    {
        for(size_t i = 0;i < simplex.vertices().size() - 1; i++)
        {
            QPointF start(simplex.vertices()[i].result().x(),simplex.vertices()[i].result().y());
            QPointF end(simplex.vertices()[i + 1].result().x(),simplex.vertices()[i + 1].result().y());
            QPointF pos(origin.x(), origin.y());
            painter->setPen(p_point);
            painter->drawPoint(pos + start);
            painter->drawPoint(pos + end);
            painter->setPen(pen);
            painter->drawLine(pos + start, pos + end);
        }
    }
}

alVector2 alGJKCollisionDetector::getDirection(alSimplex &simplex, bool towardsOrigin = true)
{
    alVector2 result;
    int count = simplex.vertices().size();
    if(count == 3)
    {
        //planning the area, filtering area that the origin must not exist
        //using dot product, for example, if AO * AB >= 0, it means that the origin lay in this area
        //if the origin is found, remaining the two point of line, removing the third point
        //return the normal vector of this line vector
        //
        simplex = findClosestEdge(simplex);
    }
    //make the vector always point to origin
    //generate perpendicular vector of line, making it point to the origin
    alVector2 ao = simplex.vertices()[0].result() * -1;
    alVector2 ab = simplex.vertices()[1].result() - simplex.vertices()[0].result();
    alVector2 perpendicularOfAB = ab.perpendicularVector();
    result = perpendicularOfAB;
    if((ao * perpendicularOfAB < 0 && towardsOrigin) || (ao * perpendicularOfAB > 0 && !towardsOrigin))
        result.negate();
    return result;
}
alSimplex alGJKCollisionDetector::findClosestEdge(alSimplex simplex)
{
    int minimumDistance = INT_MAX;
    int minimumIndex1 = 0;
    int minimumIndex2 = 0;
    for(uint32_t i = 0;i < simplex.vertices().size(); i++)
    {
        uint32_t j = i == simplex.vertices().size() - 1 ? 0 : i + 1;
        alVector2 a = simplex.vertices()[i].result();
        alVector2 b = simplex.vertices()[j].result();
        alVector2 ab = b - a; // a -> b
        alVector2 ao = a * -1;
        alVector2 perpendicularOfAB = ab.perpendicularVector();
        alVector2 e = perpendicularOfAB;
        if(ao * perpendicularOfAB < 0)//perpendicular vector point to origin
            e = e * -1;
        float projection = e.getNormalizedVector() * ao;

        if(minimumDistance > projection)
        {
            minimumIndex1 = i;
            minimumIndex2 = j;
            minimumDistance = projection;
        }
    }
    alSimplex result;
    result.vertices().push_back(simplex.vertices()[minimumIndex1]);
    result.vertices().push_back(simplex.vertices()[minimumIndex2]);
    return result;
}

alSimplex alGJKCollisionDetector::findEdge(alSimplex simplex)
{
    int minimumDistance = INT_MAX;
    int minimumIndex1 = 0;
    int minimumIndex2 = 0;
    for(uint32_t i = 0;i < simplex.vertices().size() - 1; i++)
    {
        uint32_t j = i + 1;
        alVector2 a = simplex.vertices()[i].result();
        alVector2 b = simplex.vertices()[j].result();
        alVector2 ab = b - a; // a -> b
        alVector2 ao = a * -1;
        alVector2 perpendicularOfAB = ab.perpendicularVector();
        alVector2 e = perpendicularOfAB;
        if(ao * perpendicularOfAB < 0)//perpendicular vector point to origin
            e = e * -1;
        float projection = e.getNormalizedVector() * ao;

        if(minimumDistance > projection)
        {
            minimumIndex1 = i;
            minimumIndex2 = j;
            minimumDistance = projection;
        }
    }
    alSimplex result;
    result.vertices().push_back(simplex.vertices()[minimumIndex1]);
    result.vertices().push_back(simplex.vertices()[minimumIndex2]);
    return result;
}

void alGJKCollisionDetector::scaleBody(alBody *body)
{
    if(body->type() == BodyType::Circle)
    {
        alCircle * circle = static_cast<alCircle*>(body);
        circle->setRadius(circle->radius() * alEPAScale);
    }
    else if(body->type() == BodyType::Polygon)
    {
        alPolygon * polygon = static_cast<alPolygon*>(body);
        foreach(alVector2 v, polygon->vertices())
            v *= alEPAScale;
    }
}
bool alSimplex::containOrigin(){
    if(m_vertices.size() == 3)
    {
        float a = 0, b = 0, c = 0;
        alVector2 origin;
        alVector2 oa = origin - m_vertices[0].result();
        alVector2 ob = origin - m_vertices[1].result();
        alVector2 oc = origin - m_vertices[2].result();

        a = alCross2(oa, ob);
        b = alCross2(ob, oc);
        c = alCross2(oc, oa);

        if((a <= 0 && b <= 0 && c <= 0)||
                (a >= 0 && b >= 0 && c >= 0))
            return true;
        return false;
    }
    else if(m_vertices.size() == 2)
    {
        alVector2 origin;
        alVector2 oa = origin - m_vertices[0].result();
        alVector2 ob = origin - m_vertices[1].result();
        return alCross2(oa, ob) == 0;
    }
    else
        return false;

}

void alSimplex::insertVertex(alSimplex &edge, const Minkowski_t &vertex)
{
    //insert into vertices list before the index
    uint32_t targetIndex = 0;
    for(uint32_t i = 0;i < m_vertices.size() - 1;i++)
    {
        if(m_vertices[i].result() == edge.vertices()[0].result())
        {
            targetIndex = i;
        }
    }

    m_vertices.insert(m_vertices.begin() + targetIndex + 1, vertex);
}






