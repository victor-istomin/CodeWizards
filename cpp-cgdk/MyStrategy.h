#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "model/LaneType.h"
#include "LineEquation.h"
#include "DebugVisualizer.h"

#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <array>

struct State
{
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

	static const double LOW_HP_FACTOR;
};

class MyStrategy : public Strategy 
{
	static const double WAYPOINT_RADIUS;
	static const int    STRAFE_CHANGE_INTERVAL = 4; // don't change strafe too ofter
	static Point2D      TOP_GUARD_POINT;
	static Point2D      MID_GUARD_POINT;
	static Point2D      BOTTOM_GUARD_POINT;

	typedef std::vector<Point2D> TWaypoints;

	std::unique_ptr<DebugVisualizer> m_visualizer;
	std::unique_ptr<State> m_state;
	TWaypoints m_waypoints;
	int m_lastStrafeChangeTick;
	double m_lastStrafe;
	Point2D* m_guardPoint;

	void initialSetup();
	void initState(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move);

	Point2D getNextWaypoint();
	Point2D getPreviousWaypoint();

	const model::LivingUnit* getNearestTarget();

	// actions
	void goTo(const Point2D& point, model::Move& move);
	void retreatTo(const Point2D& point, model::Move& move);



public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;
};

#endif
