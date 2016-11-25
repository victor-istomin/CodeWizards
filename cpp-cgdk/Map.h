#pragma once
#include <vector>
#include <list>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <limits>

#include "model/World.h"
#include "model/Game.h"
#include "model/Wizard.h"

#include "LineEquation.h"
#include "Utils.h"

struct TileState
{
	enum State
	{
		FREE,
		OCCUPIED,
	};

	State m_state;
	bool  m_isVisible;

	TileState(State state, bool isVisible) : m_state(state), m_isVisible(isVisible) {}
	bool isOccupied() const { return m_state == OCCUPIED; }
};

// Base map implementation
class Map : NonCopyable
{
public:
	struct TileIndex;

	typedef std::vector<TileState> TilesRow;
	typedef std::vector<TilesRow>  TilesMatrix;
	typedef std::list<Point2D>     PointPath;
	typedef std::list<TileIndex>   TilesPath;

	struct TileIndex
	{
		int m_x;
		int m_y;

		struct Hasher
		{ 
			size_t operator()(const TileIndex& t) const
			{
				std::hash<size_t> hasher;

				assert(t.m_x <= 0xFFFF && t.m_y <= 0xFFFF);  // not critical, but may be not optimal when assertion fails
				return hasher(((size_t)t.m_x << 16) + ((size_t)t.m_x >> 16) + (size_t)t.m_y);
			} 
		};

		TileIndex(int x = -1, int y = -1) : m_x(x), m_y(y) {}
		bool operator==(const TileIndex& right) const { return m_x == right.m_x && m_y == right.m_y; }
		bool operator!=(const TileIndex& right) const { return !(*this == right); }

		bool operator>(const TileIndex& right) const { return m_y > right.m_y || (m_y == right.m_y && m_x > right.m_x); }

		bool isValid(const Map& map) const 
		{
			return m_x >= 0 && m_x < static_cast<int>(map.getTilesYX().size())
				&& m_y >= 0 && m_y < static_cast<int>(map.getTilesYX().size());
		}

		double manhattanDistance(const TileIndex& to) const { return std::abs(to.m_x - m_x) + std::abs(to.m_y - m_y); }
	};

	virtual void initTiles(const model::Game& game, const model::World& world, const model::Wizard& self) = 0;
	virtual ~Map() {}

	size_t getTileSize() const { return m_tileSize; }
	const TilesMatrix& getTilesYX() const { return m_tilesYX; }

	TileIndex getTileIndex(const Point2D& mapPoint) const
	{
		return TileIndex(static_cast<int>(mapPoint.m_x / m_tileSize), static_cast<int>(mapPoint.m_y / m_tileSize));
	}

	Point2D   getTileCenter(const TileIndex& tileIndex) const { return Point2D(tileIndex.m_x * m_tileSize + m_tileSize / 2, tileIndex.m_y * m_tileSize + m_tileSize / 2); }
	TileState getTileState(const TileIndex& tileIndex) const { return getTilesYX()[tileIndex.m_y][tileIndex.m_x]; }

	PointPath tilesToPoints(const TilesPath& tilesPath) const
	{
		PointPath path;
		std::transform(tilesPath.begin(), tilesPath.end(), std::back_inserter(path), [this](const TileIndex& index) { return getTileCenter(index); });
		return std::move(path);
	}

	bool canMove(const model::World& world, const Point2D& from, const Point2D& to) const
	{
		// can move if segment from -> to does not intersect any visible obstacle, and does not intersect any occupied tile under the fog of war.

		auto isIntersectsUnit = [&from, &to, this](const model::CircularUnit& u)
		{
			return isSectionIntersects(from, to, u, u.getRadius() + m_selfRadius);
		};

		// TODO - could became a woodcutter and slowly move though tree...
		bool canMove = std::find_if(world.getTrees().begin(), world.getTrees().end(), isIntersectsUnit) == world.getTrees().end();
		canMove = canMove && std::find_if(world.getBuildings().begin(), world.getBuildings().end(), isIntersectsUnit) == world.getBuildings().end();
		
		if (canMove)
		{
			raytrace(from, to, [&canMove](const TileState& tile) 
			{
				canMove = !tile.isOccupied();
				return canMove; // if can't - abort raytracing
			});
		}

		return canMove;
	}

	PointPath smoothPath(const model::World& world, const PointPath& path) const
	{
		PointPath smoothPath;
		if (path.empty())
			return path;

		auto current = path.begin();
		auto last = path.end(); --last;

		smoothPath.push_back(*current);
		while (current != last)
		{
			// TODO - don't cut a point with a bonus

			auto middle = current; ++middle;
			auto next   = middle; ++next;
			while (next != path.end() && canMove(world, *current, *next))
			{
				++middle;
				++next;
			}

			smoothPath.push_back(*middle);
			current = middle;
		}

		return smoothPath;
	}

	static bool isSectionIntersects(const Point2D& from, const Point2D& to, const Point2D& unitCenter, double unitRadius)
	{
		// coordinates are relative to unit's center
		Point2D fromRelative = from - unitCenter;
		Point2D toRelative = to - unitCenter;
		Point2D d = toRelative - fromRelative;

		double a = d.m_x * d.m_x + d.m_y * d.m_y;
		double b = 2.0 *(fromRelative.m_x * d.m_x + fromRelative.m_y * d.m_y);
		double c = fromRelative.m_x * fromRelative.m_x + fromRelative.m_y * fromRelative.m_y - unitRadius * unitRadius;

		bool isIntersects = false;

		if (-b < 0)
			isIntersects = (c < 0);
		else if (-b < (2.0 * a))
			isIntersects = ((4.*a*c - b*b) < 0);

		if (!isIntersects)
			isIntersects = (a + b + c < 0);

		return isIntersects;
	}

private:
	TilesMatrix m_tilesYX;
	size_t      m_tileSize;
	double      m_selfRadius;

protected:
	// non-const version for derived classes
	TilesMatrix& getTilesYX() { return m_tilesYX; }

	template <typename UnitClass>
	using FilterCallback = bool(const UnitClass&);

	template <typename UnitClass, typename Filter = FilterCallback<UnitClass> >
	void fillWith(const std::vector<UnitClass>& units, Filter filter = [](const UnitClass& u){return true;})
	{
		for (const UnitClass& unit : units)
		{
			if (!filter(unit))
				continue;

			fillCircle(unit, unit.getRadius() + m_selfRadius);
		}
	}

	template <typename Setter = void(TileState&)>
	void fillCircle(Point2D center, double radius, Setter setter = [](TileState& s) { s.m_state = TileState::OCCUPIED; })
	{
		Point2D r = Point2D(radius, radius);
		Point2D topLeft     = center - r;
		Point2D bottomRight = center + r;

		int rowsCount    = m_tilesYX.size();
		int columnsCount = m_tilesYX.front().size();

		for (int y = static_cast<int>(topLeft.m_y / m_tileSize); y != static_cast<int>(bottomRight.m_y / m_tileSize + 1); ++y)
		{
			if (y < 0 || y >= rowsCount)
				continue;  // trees and projectiles won't fall through the edge of the world, but AI process will

			int xStart = std::max(static_cast<int>(topLeft.m_x / m_tileSize), 0);
			int xFinish = std::min(static_cast<int>(bottomRight.m_x / m_tileSize + 1), columnsCount);

			for (int x = xStart; x < xFinish; ++x)
			{
				if (intersects(center, radius, x, y))
					setter(m_tilesYX[y][x]);
			}
		}
	}

	Map(size_t tileSize, const model::Game& game, const model::World& world, const model::Wizard& self)
		: m_tilesYX()
		, m_tileSize(tileSize)
		, m_selfRadius(self.getRadius() * 1.3)  // TODO - remove temporary hack
	{
		size_t xSize = static_cast<size_t>(world.getWidth() / tileSize);
		size_t ySize = static_cast<size_t>(world.getHeight() / tileSize);

		m_tilesYX.reserve(ySize);
		for (size_t i = 0; i < ySize; ++i)
			m_tilesYX.emplace_back(xSize, TileState(TileState::FREE, false));
	}

	// true if unit intersects with tile
	bool intersects(const Point2D& unitCenter, double radius, size_t tileX, size_t tileY)
	{
		Point2D tileCenter = Point2D(tileX * m_tileSize, tileY * m_tileSize) + Point2D(m_tileSize / 2.0, m_tileSize / 2.0);

		Point2D centersDistance = tileCenter - unitCenter;
		centersDistance.m_x = std::abs(centersDistance.m_x);
		centersDistance.m_y = std::abs(centersDistance.m_y);

		double halfTile = m_tileSize / 2.0;

		if (   centersDistance.m_x > (halfTile + radius)
			|| centersDistance.m_y > (halfTile + radius))
		{ 
			return false; // too far to collide
		} 

		if (centersDistance.m_x <= halfTile || centersDistance.m_y <= halfTile) 
		{ 
			return true; // lays inside
		}

		double cornerDistance = std::hypot(centersDistance.m_x - halfTile, centersDistance.m_y - halfTile);
		return cornerDistance <= radius;
	}


	// cast a ray 'from' -> 'to'
	// variant of "A Fast Voxel Traversal Algorithm for Ray Tracing"
	// http://www.cse.yorku.ca/~amana/research/grid.pdf
	// http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
	// 
	// TileVisitor is bool f(tile) which returns false if visiting should be interrupted
	template <typename TileVisitor = bool(const TileState& s)>
	void raytrace(const Point2D& from, const Point2D& to, const TileVisitor& visit) const
	{
		// todo: use this one when navigating through the for of war 
		Point2D tileFrom = from / m_tileSize;
		Point2D tileTo   = to   / m_tileSize;

		double dx = std::abs(tileTo.m_x - tileFrom.m_x);
		double dy = std::abs(tileTo.m_y - tileFrom.m_y);

		int x = int(std::floor(tileFrom.m_x));
		int y = int(std::floor(tileFrom.m_y));

		int n = 1;
		int x_inc, y_inc;
		double error;

		if (std::abs(dx) < Point2D::k_epsilon)
		{
			x_inc = 0;
			error = std::numeric_limits<double>::infinity();
		}
		else if (tileTo.m_x > tileFrom.m_x)
		{
			x_inc = 1;
			n    += int(std::floor(tileTo.m_x)) - x;
			error = (std::floor(tileFrom.m_x) + 1 - tileFrom.m_x) * dy;
		}
		else
		{
			x_inc = -1;
			n    += x - int(std::floor(tileTo.m_x));
			error = (tileFrom.m_x - floor(tileFrom.m_x)) * dy;
		}

		if (std::abs(dy) < Point2D::k_epsilon)
		{
			y_inc = 0;
			error -= std::numeric_limits<double>::infinity();
		}
		else if (tileTo.m_y > tileFrom.m_y)
		{
			y_inc  = 1;
			n     += int(std::floor(tileTo.m_y)) - y;
			error -= (std::floor(tileFrom.m_y) + 1 - tileFrom.m_y) * dx;
		}
		else
		{
			y_inc  = -1;
			n     += y - int(std::floor(tileTo.m_y));
			error -= (tileFrom.m_y - std::floor(tileFrom.m_y)) * dx;
		}

		for (; n > 0; --n)
		{
			if (!visit(getTileState(TileIndex(x, y))))
				break;

			if (error > 0)
			{
				y += y_inc;
				error -= dx;
			}
			else
			{
				x += x_inc;
				error += dy;
			}
		}
	}
};

