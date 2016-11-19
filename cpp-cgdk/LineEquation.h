#pragma once

#include "Vec.h"
#include "model//Unit.h"

#include <cmath>

struct Point2D
{
	double m_x;
	double m_y;

	static const double k_epsilon;
	static double pow2(double d) { return d*d; }

	// unused:
	bool operator==(const Point2D& right) const { return std::abs(m_x - right.m_x) < k_epsilon && std::abs(m_y - right.m_y) < k_epsilon; }
	// bool operator!=(const Point2D& right) const { return !this->operator ==(right); }
	bool isZero() const { return *this == Point2D(0, 0); }

	Point2D(double x, double y) : m_x(x), m_y(y) {}
	Point2D(const model::Unit& u) : Point2D(u.getX(), u.getY()) {}

	double getDistanceTo(const Point2D& other)     const { return std::hypot(m_x - other.m_x, m_y - other.m_y); }
	double getSquareDistance(const Point2D& other) const { return pow2(m_x - other.m_x) + pow2(m_y - other.m_y); }  // sometimes we could compare Distance² with Radius² to omit expensive sqrt() and/or more expensive hypot()

	Point2D operator+(const Point2D& right) const { return Point2D(m_x + right.m_x, m_y + right.m_y); }
	Point2D operator-(const Point2D& right) const { return Point2D(m_x - right.m_x, m_y - right.m_y); }

	template <typename Numeric> 
	Point2D operator/(Numeric divisor) const { return Point2D(m_x / divisor, m_y / divisor); }
};

struct LineEquation  // m_a * x + m_b * y + m_c = 0;
{
	double m_a;
	double m_b;
	double m_c;

	LineEquation(double a, double b, double c)
		: m_a(a), m_b(b), m_c(c) {}

	// line from direction vector and point
	static LineEquation fromDirectionVector(const Point2D& a, const Vec2d& v)
	{
		/*
		* Line from vector:
		*   have: A{x,y}, v{x, y}
		*         (x - A.x) / v.x = (y - A.y) / v.y
		*
		*   A = vy, B = -vx, C = (vx * Ay - vy * Ax)
		*/
		return LineEquation(v.m_y, -v.m_x, v.m_x * a.m_y - v.m_y * a.m_x);
	}

private:
	static double matrixDeterminant(double a, double b, double c, double d) { return a * d - b * c; }

public:

	bool isIntersects(const LineEquation& another, Point2D& result) const
	{
		static const double EPSILON = 1e-5; // TODO - move

											// using Cramer's rule
		double zn = matrixDeterminant(m_a, m_b, another.m_a, another.m_b);
		if (std::abs(zn) < EPSILON)
			return false;

		result.m_x = -matrixDeterminant(m_c, m_b, another.m_c, another.m_b) / zn;
		result.m_y = -matrixDeterminant(m_a, m_c, another.m_a, another.m_c) / zn;
		return true;
	}

	bool isContains(const Point2D& point) const
	{
		return std::abs(m_a*point.m_x + m_b*point.m_y + m_c) < Point2D::k_epsilon;
	}

	bool isParallel(const LineEquation& another) const
	{
		return std::abs(matrixDeterminant(m_a, m_b, another.m_a, another.m_b)) < Point2D::k_epsilon;
	}

	bool isEquivalent(const LineEquation& another)
	{
		return std::abs(matrixDeterminant(m_a, m_b, another.m_a, another.m_b)) < Point2D::k_epsilon
			&& std::abs(matrixDeterminant(m_a, m_c, another.m_a, another.m_c)) < Point2D::k_epsilon
			&& std::abs(matrixDeterminant(m_b, m_c, another.m_b, another.m_c)) < Point2D::k_epsilon;
	}

};
