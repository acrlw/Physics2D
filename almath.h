#ifndef ALMATH_H
#define ALMATH_H

#include <math.h>
#include <qmath.h>
#include <stddef.h>
#include <assert.h>
#include <float.h>
#include <iostream>
#include <QPointF>
#include <QVector2D>
///Fast Inverse Square Root Algorithm From Quake III
static inline float alFastInverseSqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    return x;
}
///
/// \brief The alVector2 class
/// the 2d vector contains x and y element.
/// support the vector calculation
class alVector2{

public:
    alVector2(): m_x(0), m_y(0), m_length(0){

    }
    alVector2(const float& x, const float& y)
    {
        m_x = x;
        m_y = y;
    }
    alVector2(const alVector2& v)
    {
        m_x = v.x();
        m_y = v.y();
    }
    ///
    /// \brief operator =
    /// copy the x and y from another vector to this vector
    /// \param v target vector
    ///
    inline void operator = (const alVector2& v)
    {
        m_x = v.x();
        m_y = v.y();
    }

    ///
    /// \brief operator + vector addition calculation
    /// \param v target vector
    /// \return
    ///
    inline alVector2 operator + (const alVector2& v) const
    {
        return alVector2(m_x + v.x(), m_y + v.y());
    }
    inline alVector2 operator - (const alVector2& v) const
    {
        return alVector2(m_x - v.x(), m_y - v.y());
    }
    inline alVector2 operator * (const float f) const {
        return alVector2(m_x * f, m_y * f);
    }
    inline float operator * (const alVector2& v) const {
        return m_x * v.x() + m_y * v.y();
    }

    inline alVector2 operator / (const float f) const{
        return alVector2(m_x / f, m_y / f);
    }
    inline alVector2& operator += (const alVector2& v) {
        m_x += v.m_x;
        m_y += v.m_y;
        return *this;
    }
    inline alVector2& operator -= (const alVector2& v) {
        m_x -= v.m_x;
        m_y -= v.m_y;
        return *this;
    }
    inline alVector2& operator *= (const float f) {
        m_x *= f;
        m_y *= f;
        return *this;
    }
    inline alVector2& operator /= (const float f){
        m_x /= f;
        m_y /= f;
        return *this;
    }

    inline bool operator ==(const alVector2& v)
    {
        return m_x == v.m_x && m_y == v.m_y;
    }

    ///
    /// \brief
    /// return the length of the vector
    /// \return
    ///
    inline float length()
    {
        m_length = sqrt(m_x * m_x + m_y * m_y);
        return m_length;
    }
    inline float lengthSquare()
    {
        return m_x * m_x + m_y * m_y;
    }
    inline float x() const
    {
        return m_x;
    }
    inline void setX(float x)
    {
        m_x = x;
    }
    inline float y() const
    {
        return m_y;
    }
    inline void setY(float y)
    {
        m_y = y;
    }
    inline alVector2& normalize() {
        float r = alFastInverseSqrt(m_x * m_x + m_y * m_y);
        m_x *= r;
        m_y *= r;
        return *this;
    }
    inline const alVector2 getNormalizedVector()const
    {
        float r = alFastInverseSqrt(m_x * m_x + m_y * m_y);
        return alVector2(m_x, m_y) * r;
    }
    inline void set(const float x, const float y)
    {
        m_x = x;
        m_y = y;
    }
    inline void set(const alVector2& v)
    {
        m_x = v.m_x;
        m_y = v.m_y;
    }
    inline alVector2& negate()
    {
        m_x = -m_x;
        m_y = -m_y;
        return *this;
    }
    inline const alVector2 perpendicularVector()
    {
        return alVector2(-m_y, m_x);
    }
    inline const QVector2D toVector2D(){
        return QVector2D(m_x, m_y);
    }
    inline const QPointF toPointF()
    {
        return QPointF(m_x, m_y);
    }
private:
    float m_x;
    float m_y;
    float m_length;
};
class alVector3{

public:
    alVector3(): m_x(0), m_y(0), m_z(0), m_length(0){

    }
    alVector3(const float& x, const float& y, const float& z)
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }
    alVector3(const alVector3& v)
    {
        m_x = v.m_x;
        m_y = v.m_y;
        m_z = v.m_z;
    }

    inline void operator= (const alVector3& v)
    {
        m_x = v.m_x;
        m_y = v.m_y;
        m_z = v.m_z;
    }
    inline alVector3 operator+ (const alVector3& v) const
    {
        return alVector3(m_x + v.m_x, m_y + v.m_y, m_z + v.m_z);
    }

    inline alVector3 operator- (const alVector3& v) const
    {
        return alVector3(m_x - v.m_x, m_y - v.m_y, m_z - v.m_z);
    }
    inline alVector3 operator* (const float f) const
    {
        return alVector3(m_x * f, m_y * f, m_z * f);
    }
    inline float operator* (const alVector3& v) const
    {
        return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
    }
    inline alVector3& operator+= (const alVector3& v)
    {
        m_x += v.m_x;
        m_y += v.m_y;
        m_z += v.m_z;
        return *this;
    }
    inline alVector3& operator-= (const alVector3& v)
    {
        m_x -= v.m_x;
        m_y -= v.m_y;
        m_z -= v.m_z;
        return *this;
    }
    inline alVector3& operator*= (const float f)
    {
        m_x *= f;
        m_y *= f;
        m_z *= f;
        return *this;
    }
    inline alVector3& operator/= (const float f)
    {
        m_x /= f;
        m_y /= f;
        m_z /= f;
        return *this;
    }
    inline bool operator ==(const alVector3& v)
    {
        return m_x == v.m_x && m_y == v.m_y && m_z == v.m_z;
    }
    inline float lengthSquare()
    {
        return m_x * m_x + m_y * m_y + m_z * m_z;
    }
    inline float length()
    {
        m_length = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        return m_length;
    }
    inline float x() const
    {
        return m_x;
    }
    inline void setX(float x)
    {
        m_x = x;
    }
    inline float y() const{
        return m_y;
    }
    inline void setY(float y)
    {
        m_y = y;
    }
    inline float z() const
    {
        return m_z;
    }
    inline void setZ(float z)
    {
        m_z = z;
    }
    inline alVector3& normalize()
    {
        float r = alFastInverseSqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        m_x *= r;
        m_y *= r;
        m_z *= r;
        return *this;
    }
    inline void set(const float x, const float y, const float z)
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }
    inline alVector3 getNormalizedVector() const
    {
        float r = alFastInverseSqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        return alVector3(m_x, m_y, m_z) * r;
    }
    inline alVector3& negate()
    {
        m_x = -m_x;
        m_y = -m_y;
        m_z = -m_z;
        return *this;
    }
private:
    float m_x;
    float m_y;
    float m_z;
    float m_length;
};
class alMatrix2x2{

public:
    alMatrix2x2(){}
    alMatrix2x2(const alVector2& col1, const alVector2& col2)
    {
        m_column1 = col1;
        m_column2 = col2;
    }
    alMatrix2x2(const float& a11,const float& a12,const float& a21,const float& a22)
    {
        m_column1 = alVector2(a11, a12);
        m_column2 = alVector2(a21, a22);
    }

    inline void operator= (const alMatrix2x2& m)
    {
        m_column1 = m.m_column1;
        m_column2 = m.m_column2;
    }
    inline alMatrix2x2 operator+ (const alMatrix2x2& v) const
    {
        return alMatrix2x2(m_column1 + v.m_column1 , m_column2 + v.m_column2);
    }
    inline alMatrix2x2 operator- (const alMatrix2x2& v) const
    {
        return alMatrix2x2(m_column1 - v.m_column1 , m_column2 - v.m_column2);
    }
    inline alVector2 operator* (const alVector2& v) const
    {
        return alVector2(v.x() * m_column1.x() + v.y() * m_column2.x(), v.x() * m_column1.y() + v.y() * m_column2.y());
    }
    inline alMatrix2x2 operator* (const alMatrix2x2& v) const
    {
        return alMatrix2x2(*this * v.m_column1, *this * v.m_column2);
    }
    inline alMatrix2x2 operator* (const float f) const
    {
        return alMatrix2x2(m_column1 * f, m_column2 * f);
    }
    inline alMatrix2x2& operator*= (const float f)
    {
        m_column1 *= f;
        m_column2 *= f;
        return *this;
    }

    inline alVector2 column1() const{
        return m_column1;
    }
    inline void setColumn1(const alVector2 &column1){
        m_column1 = column1;
    }
    inline alVector2 column2() const{
        return m_column2;
    }
    inline void setColumn2(const alVector2 &column2){
        m_column2 = column2;
    }
    inline alMatrix2x2& transpose(){
        m_column1 = alVector2(m_column1.x() , m_column2.x());
        m_column2 = alVector2(m_column1.y() , m_column2.y());
        return *this;
    }
private:
    alVector2 m_column1;
    alVector2 m_column2;
};
class alMatrix3x3{

public:

    alMatrix3x3(){}
    alMatrix3x3(const alVector3& col1, const alVector3& col2, const alVector3& col3)
    {
        m_column1 = col1;
        m_column2 = col2;
        m_column3 = col3;
    }
    alMatrix3x3(const float& a11,const float& a12,const float& a13,const float& a21,const float& a22,const float& a23,const float& a31,const float& a32,const float& a33)
    {
        m_column1 = alVector3(a11, a12, a13);
        m_column2 = alVector3(a21, a22, a23);
        m_column3 = alVector3(a31, a32, a33);
    }

    inline void operator= (const alMatrix3x3& m)
    {
        m_column1 = m.m_column1;
        m_column2 = m.m_column2;
        m_column3 = m.m_column3;
    }
    inline alMatrix3x3 operator+ (const alMatrix3x3& v) const
    {
        return alMatrix3x3(m_column1 + v.m_column1 , m_column2 + v.m_column2, m_column3 + v.m_column3);
    }

    inline alMatrix3x3 operator- (const alMatrix3x3& v) const
    {
        return alMatrix3x3(m_column1 - v.m_column1 , m_column2 - v.m_column2, m_column3 - v.m_column3);
    }
    inline alVector3 operator* (const alVector3& v) const
    {
        return alVector3(v.x() * m_column1.x() + v.y() * m_column2.x() + v.z() * m_column3.x()
                      , v.x() * m_column1.y() + v.y() * m_column2.y() + v.z() * m_column3.y()
                      , v.x() * m_column1.z() + v.y() * m_column2.z() + v.z() * m_column3.z());
    }
    inline alMatrix3x3 operator* (const alMatrix3x3& v) const
    {
        return alMatrix3x3(*this * v.m_column1, *this * v.m_column2, *this * v.m_column3);
    }
    inline alMatrix3x3& operator*= (const float f)
    {
        m_column1 *= f;
        m_column2 *= f;
        m_column3 *= f;
        return *this;
    }

    inline alVector3 column1() const
    {
        return m_column1;
    }
    inline void setColumn1(const alVector3 &column1){
        m_column1 = column1;
    }

    inline alVector3 column2() const{
        return m_column2;
    }
    inline void setColumn2(const alVector3 &column2){
        m_column2 = column2;
    }
    inline alVector3 column3() const{
        return m_column3;
    }

    inline void setColumn3(const alVector3 &column3){
        m_column3 = column3;
    }
    inline alMatrix3x3& transpose(){
        m_column1 = alVector3(m_column1.x() , m_column2.x(), m_column3.x());
        m_column2 = alVector3(m_column1.y() , m_column2.y(), m_column3.y());
        m_column3 = alVector3(m_column1.z() , m_column2.z(), m_column3.z());
        return *this;
    }
private:
    alVector3 m_column1;
    alVector3 m_column2;
    alVector3 m_column3;
};
class alRotation
{
public:

    alRotation(const float angle){
        m_angle = angle;
        m_arc = m_angle * M_PI / 180;
        update();
    }

    inline alVector2 operator* (const alVector2& v) const
    {
        return m_rotMat * v;
    }
    inline alMatrix2x2 operator* (const float f) const
    {
        return m_rotMat * f;
    }
    inline float angle() const{
        return m_angle;
    }
    inline void setAngle(float angle){
        m_angle = angle;
        m_arc = m_angle * M_PI / 180;
        update();
    }
    inline float arc() const{
        return m_arc;
    }
    inline void setArc(float arc){
        m_arc = arc;
        m_angle = m_angle * 180 / M_PI;
        update();
    }
private:
    inline void update()
    {
        m_rotMat = alMatrix2x2(cos(m_arc), sin(m_arc), -sin(m_arc), cos(m_arc));
    }
    float m_angle;
    float m_arc;
    alMatrix2x2 m_rotMat;
};
inline std::ostream &operator << (std::ostream& os, const alVector2& v)
{
    return os << '(' << v.x() << ',' << v.y() << ')';
}
inline std::ostream &operator << (std::ostream& os, const alVector3& v)
{
    return os << '(' << v.x() << ',' << v.y() << ',' << v.z() << ')';
}
static inline float alCross2(const alVector2& l, const alVector2& r)
{
    return l.x() * r.y() - r.x() * l.y();
}
///3d vector cross product
///
static inline alVector3 alCross3(const alVector3& l, const alVector3& r)
{
    return alVector3(l.y() * r.z() - l.z() * r.y(), l.z() * r.x() - l.x() * r.z(), l.x() * r.y() - l.y() * r.x());
}
static inline alVector2 alPolarToCartesian(const float arc, const float length)
{
    return alVector2(cos(arc), sin(arc)) * length;
}
inline alVector2 operator*(const float f, const alVector2& v)
{
    return v * f;
}
inline bool floatEqual(const float& lhs, const float& rhs, const float epsilon = 0.0000001)
{
    return abs(lhs - rhs) < epsilon;
}
#endif // ALMATH_H
