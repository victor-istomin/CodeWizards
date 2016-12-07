#pragma once
#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES
#include <cmath>

class Vec2d
{
public:
	double m_x, m_y;

	Vec2d()                   : m_x(0), m_y(0) {}
	Vec2d(double x, double y) : m_x(x), m_y(y) {}
	Vec2d(const Vec2d& v)     : m_x(v.m_x), m_y(v.m_y) {}

	template <typename Point> static Vec2d fromPoint(const Point& p) { return Vec2d(p.m_x, p.m_y); }
	template <typename Point>        Point toPoint() const           { return Point(m_x, m_y); }

	Vec2d& operator=(const Vec2d& v)       { m_x = v.m_x; m_y = v.m_y; return *this; }
	Vec2d  operator+(const Vec2d& v) const { return Vec2d(m_x + v.m_x, m_y + v.m_y); }
	Vec2d  operator-(const Vec2d& v) const { return Vec2d(m_x - v.m_x, m_y - v.m_y); }

	Vec2d& operator+=(Vec2d& v) { m_x += v.m_x; m_y += v.m_y; return *this; }
	Vec2d& operator-=(Vec2d& v) { m_x -= v.m_x; m_y -= v.m_y; return *this; }

	Vec2d operator+(double s) const { return Vec2d(m_x + s, m_y + s); }
	Vec2d operator-(double s) const { return Vec2d(m_x - s, m_y - s); }
	Vec2d operator*(double s) const { return Vec2d(m_x * s, m_y * s); }
	Vec2d operator/(double s) const { return Vec2d(m_x / s, m_y / s); }

	Vec2d& operator+=(double s) { m_x += s; m_y += s; return *this; }
	Vec2d& operator-=(double s) { m_x -= s; m_y -= s; return *this; }
	Vec2d& operator*=(double s) { m_x *= s; m_y *= s; return *this; }
	Vec2d& operator/=(double s) { m_x /= s; m_y /= s; return *this; }

	Vec2d& set(double x, double y) { m_x = x; m_y = y; return *this; }

	Vec2d& rotate(double radians)
	{
		double c = cos(radians);
		double s = sin(radians);

		double x = m_x, y = m_y;

		m_x = x * c - y * s;
		m_y = x * s + y * c;

		return *this;
	}

	Vec2d& normalize()
	{
		if (length() == 0)
			return *this;

		*this /= length();
		return *this;
	}

	double dist(const Vec2d& v) const { return (*this - v).length(); }
	double length()             const { return std::hypot(m_x, m_y); }   // note: std::hypot() might be slower than naive sqrd(x*x + y*y);

	Vec2d& truncate(double length)
	{
		double a = angle();

		double cos = std::cos(a);
		double sin = std::sin(a);

		return *this = truncate(length, cos, sin);
	}

	static Vec2d truncate(double length, double cos, double sin)
	{
		return Vec2d{ length * cos, length * sin };
	}

	double angle() const
	{
		return std::atan2(m_y, m_x);
	}

	Vec2d ortho() const { return Vec2d(m_y, -m_x); }

	static double dot(const Vec2d& v1, const Vec2d& v2) { return v1.m_x * v2.m_x + v1.m_y * v2.m_y; }
	static double cross(const Vec2d& v1, const Vec2d& v2) { return (v1.m_x * v2.m_y) - (v1.m_y * v2.m_x); }

	static double angleBetween(const Vec2d& a, const Vec2d& b)
	{
		double cosine = dot(a, b) / a.length() / b.length();
		return std::acos(cosine);
	}
};
