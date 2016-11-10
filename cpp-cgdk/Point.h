#pragma once
#include "model/Unit.h"
#include <cmath>

struct Point2D
{
	double m_x;
	double m_y;

	static const double k_epsilon;

	Point2D(double x, double y) : m_x(x), m_y(y) {}
	Point2D(const model::Unit& u) : Point2D(u.getX(), u.getY()) {}

	double getDistanceTo(const Point2D& other) const { return std::hypot(m_x - other.m_x, m_y - other.m_y); }
};

