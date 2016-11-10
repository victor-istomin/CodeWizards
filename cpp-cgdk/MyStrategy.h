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
	const model::Wizard& m_self;
	const model::World&  m_world;
	const model::Game&   m_game;
	const model::Move&   m_move;

	bool m_isEnemyAround;
	bool m_isLowHP;

	const model::Projectile* m_attackingProjectile;

	std::array<int, model::_ACTION_COUNT_> m_cooldownTicks;

	State(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move);

	bool isReadyForAction(model::ActionType action) const              { return m_cooldownTicks[action] == 0; }
	bool isUnderMissile() const                                         { return m_attackingProjectile != nullptr;  }

	static const double LOW_HP_FACTOR;
};

class MyStrategy : public Strategy 
{
	static const double WAYPOINT_RADIUS;
	static const int    STRAFE_CHANGE_INTERVAL = 4; // don't change strafe too ofter

	typedef std::vector<Point2D> TWaypoints;

	std::unique_ptr<State> m_state;
	TWaypoints m_waypoints;
	int m_lastStrafeChangeTick;
	double m_lastStrafe;

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
