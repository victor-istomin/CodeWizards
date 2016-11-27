#pragma once

#include "Map.h"
#include "model/Wizard.h"
#include "MyStrategy.h"

#if _DEBUG
#include "../russian-ai-cup-visual/clients/cpp/Debug.h"
#include "PathFinder.h"
#include <memory>

	class DebugVisualizer
	{
		Debug m_debug;

	public:
		DebugVisualizer() {}
		void beginPre() { m_debug.beginPre();  }
		void endPre() { m_debug.endPre(); }

		void drawWaypoint(const model::Wizard& me, const Point2D& waypoint)
		{
			Point2D selfPoint = me;

			const int32_t color = 0x555555;

			m_debug.fillCircle(waypoint.m_x, waypoint.m_y, me.getRadius(), color);
			m_debug.line(selfPoint.m_x, selfPoint.m_y, waypoint.m_x, waypoint.m_y, color);
		}

		void drawPath(const Map* map, const PathFinder::TilesPath& path, const Map::PointPath& smooth)
		{
			const int32_t color = 0x55FF55;

			Point2D previous    = Point2D(0, 0);
			bool    hasPresious = false;

			// path itself

			for (const auto& tile : path)
			{
				Point2D next = map->getTileCenter(tile);

				if (hasPresious)
					m_debug.line(previous.m_x, previous.m_y, next.m_x, next.m_y, color);

				m_debug.circle(next.m_x, next.m_y, map->getTileSize() / 2, color);
				previous = next;
				hasPresious = true;
			}

			// and then smooth path 
			const int32_t smoothColor = 0x5555FF;
			previous = Point2D(0, 0);
			hasPresious = false;
			for (const auto& next : smooth)
			{
				if (hasPresious)
					m_debug.line(previous.m_x, previous.m_y, next.m_x, next.m_y, smoothColor);

				m_debug.fillCircle(next.m_x, next.m_y, map->getTileSize() / 2, smoothColor);
				previous = next;
				hasPresious = true;
			}
		}

		void drawMap(const Map& map)
		{
			const Map::TilesMatrix rows = map.getTilesYX();
			const int32_t tileColor = 0x99BBBB;
			const int32_t gridColor = 0xDDDDDD;
			const size_t tileSize = map.getTileSize();

			// draw occupied tiles

			for (size_t y = 0; y < rows.size(); ++y)
			{
				const Map::TilesRow& row = rows[y];
				for (size_t x = 0; x < row.size(); ++x)
				{
					if (row[x].isOccupied())
					{
						Point2D tileTopLeft = Point2D(x * tileSize, y * tileSize);
						Point2D tileBottomRight = tileTopLeft + Point2D(tileSize, tileSize);
						m_debug.fillRect(tileTopLeft.m_x, tileTopLeft.m_y, tileBottomRight.m_x, tileBottomRight.m_y, tileColor);
					}
				}
			}

			// draw grid
			for (size_t y = 0; y < rows.size(); ++y)
				m_debug.line(0, y * tileSize, 4000, y * tileSize, gridColor);

			for (size_t x = 0; x < rows.front().size(); ++x)
				m_debug.line(x * tileSize, 0, x * tileSize, 4000, gridColor);
		}

		void drawWaypointsMap(const MyStrategy::TWaypointsMap& waypointsMap)
		{
			for (const auto& laneWaypoints : waypointsMap)
			{
				const auto& waypoints = laneWaypoints.second;
				for (const Point2D& point : waypoints)
					m_debug.fillCircle(point.m_x, point.m_y, 20, 0x222255);
			}
		}

		void drawBonuses(const BonusSpawns& bonuses)
		{
			int32_t color = 0xFF5555;
			int32_t confirmedColor = 0xFF0000;
			for (const BonusSpawn& bonus : bonuses)
			{
				m_debug.circle(bonus.m_point.m_x, bonus.m_point.m_y, bonus.m_state == BonusSpawn::HAS_BONUS ? confirmedColor : color);
			}
		}

		void drawPredictions(const State::PredictedUnits& predictions)
		{
			const int color = 0x888833;
			for (const PredictedUnit& unit : predictions)
				m_debug.circle(unit.getX(), unit.getY(), unit.getRadius(), color);
		}
	};

	class DebugMessage
	{
		DebugVisualizer& m_render;

		const model::Wizard&      m_self;
		const model::World&       m_world;
		std::list<const Map*>     m_maps;
 		std::unique_ptr<Point2D>  m_nextWaypoint;

		const MyStrategy::TWaypointsMap* m_waypoints;
		const BonusSpawns*               m_bonuses;
		const State::PredictedUnits*     m_predictions;
 		std::tuple<PathFinder::TilesPath, Map::PointPath, const Map*> m_path;

	public:
		DebugMessage(DebugVisualizer& render, const model::Wizard& self, const model::World& world)
			: m_render(render), m_self(self), m_world(world), m_waypoints(nullptr), m_bonuses(nullptr), m_predictions(nullptr)
		{}

		void setNextWaypoint(const Point2D& waypoint)                         { m_nextWaypoint = std::make_unique<Point2D>(waypoint); }
		void visualizeMap(const Map* map)                                     { m_maps.push_back(map);  }
		void visualizePath(const PathFinder::TilesPath& path, const Map* map) { m_path = std::make_tuple(path, map->smoothPath(m_world, map->tilesToPoints(path)), map); }
		void visualizeWaypoints(const MyStrategy::TWaypointsMap& waypoints)   { m_waypoints = &waypoints; }
		void visualizeBonuses(const BonusSpawns& bonuses)                     { m_bonuses = &bonuses; }
		void visualizePredictions(const State::PredictedUnits& predictions)   { m_predictions = &predictions; }

		~DebugMessage()
		{
			// commit
			m_render.beginPre();

			for(const Map* map : m_maps)
				m_render.drawMap(*map);

			if (m_waypoints != nullptr)
				m_render.drawWaypointsMap(*m_waypoints);

			const auto& path   = std::get<0>(m_path);
			const auto& smooth = std::get<1>(m_path);
			const Map*  map = std::get<2>(m_path);
			if (!path.empty())
				m_render.drawPath(map, path, smooth);

			if (m_nextWaypoint)
				m_render.drawWaypoint(m_self, *m_nextWaypoint);

			if (m_bonuses)
				m_render.drawBonuses(*m_bonuses);

			if (m_predictions)
				m_render.drawPredictions(*m_predictions);

			m_render.endPre();
		}

	};

#else

	class DebugVisualizer
	{

	};

	class DebugMessage
	{
	public:

		DebugMessage(...) {}
		~DebugMessage() { }

		void setNextWaypoint(...)    {}
		void visualizeMap(...)       {}
		void visualizePath(...)      {}
		void visualizeWaypoints(...) {}
		void visualizeBonuses(...)   {}
		void visualizePredictions(...) {}
	};

#endif

