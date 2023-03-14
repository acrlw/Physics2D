#include "curve.h"
namespace Physics2D
{
	Curve::Curve()
	{
		m_type = Type::Curve;
	}

	void Curve::set(const Vector2& start, const Vector2& control1, const Vector2& control2, const Vector2& end)
	{
		m_startPoint = start;
		m_control1 = control1;
		m_control2 = control2;
		m_endPoint = end;
	}

	Vector2 Curve::startPoint() const
	{
		return m_startPoint;
	}

	void Curve::setStartPoint(const Vector2& startPoint)
	{
		m_startPoint = startPoint;
	}

	Vector2 Curve::control1() const
	{
		return m_control1;
	}

	void Curve::setControl1(const Vector2& control1)
	{
		m_control1 = control1;
	}

	Vector2 Curve::control2() const
	{
		return m_control2;
	}

	void Curve::setControl2(const Vector2& control2)
	{
		m_control2 = control2;
	}

	Vector2 Curve::endPoint() const
	{
		return m_endPoint;
	}

	void Curve::setEndPoint(const Vector2& endPoint)
	{
		m_endPoint = endPoint;
	}

	void Curve::scale(const real& factor)
	{
		m_startPoint *= factor;
		m_control1 *= factor;
		m_control2 *= factor;
		m_endPoint *= factor;
	}
	bool Curve::contains(const Vector2& point, const real& epsilon)
	{
		return false;
	}
	Vector2 Curve::center() const
	{
		return Vector2();
	}

}