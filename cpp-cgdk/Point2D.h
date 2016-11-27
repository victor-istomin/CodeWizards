#pragma once

#include "model/Unit.h"
#include <cmath>

struct Point2D
{
	double m_x;
	double m_y;

	static const double k_epsilon;
	static double pow2(double d) { return d*d; }

	bool operator==(const Point2D& right) const { return std::abs(m_x - right.m_x) < k_epsilon && std::abs(m_y - right.m_y) < k_epsilon; }

	// unused:
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


