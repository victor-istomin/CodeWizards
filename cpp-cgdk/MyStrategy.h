#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "model/LaneType.h"
#include "LineEquation.h"

#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <array>

struct State
{
	static const double LOW_HP_FACTOR;

	const model::Wizard& m_self;
	const model::World&  m_world;
	const model::Game&   m_game;
	const model::Move&   m_move;

	bool m_isEnemyAround;
	bool m_isUnderMissile;
	bool m_isLowHP;

	std::array<int, model::_ACTION_COUNT_> m_cooldownTicks;

	State(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move);

	bool isReadyForAction(model::ActionType action) const              { return m_cooldownTicks[action] == 0; }
};

class Map;
class DebugVisualizer;
class DebugMessage;
class PathFinder;
class MapsManager;

class MyStrategy : public Strategy 
{
public:
	typedef std::vector<Point2D>      TWaypoints;
	typedef std::map<model::LaneType, TWaypoints> TWaypointsMap;

private:

	static const double WAYPOINT_RADIUS;
	static const int    STRAFE_CHANGE_INTERVAL = 4; // don't change strafe too often

	static Point2D      TOP_GUARD_POINT;
	static Point2D      MID_GUARD_POINT;
	static Point2D      BOTTOM_GUARD_POINT;

	static Point2D      BONUS_POINTS[];

	static TWaypointsMap g_waypointsMap;

	std::unique_ptr<DebugVisualizer> m_visualizer;
	std::unique_ptr<State>           m_state;
	std::unique_ptr<MapsManager>     m_maps;
	std::unique_ptr<PathFinder>      m_pathFinder;
	std::unique_ptr<Point2D>         m_spawnPoint;

	TWaypoints m_waypoints;
	size_t     m_currentWaypointIndex; 

	int m_lastStrafeChangeTick;
	double m_lastStrafe;
	Point2D* m_guardPoint;

	void initialSetup();
	void initState(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move);

	Point2D getNextWaypoint();
	Point2D getPreviousWaypoint();

	const model::LivingUnit* getNearestTarget();

	// actions
	void goTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);
	void retreatTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);

public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;
};

#endif
