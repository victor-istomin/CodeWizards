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
#include <cassert>

struct StorableState
{
	std::unique_ptr<model::Move> m_previousMove;
};

struct State
{
	static const double LOW_HP_FACTOR;

	const model::Wizard& m_self;
	const model::World&  m_world;
	const model::Game&   m_game;
	const model::Move&   m_move;
	const StorableState& m_oldState;

	bool m_isEnemyAround;
	bool m_isUnderMissile;
	bool m_isLowHP;
	bool m_isGoingToBonus;  // not yet implemented, always false

	std::array<int, model::_ACTION_COUNT_> m_cooldownTicks;

	State(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move, const StorableState& m_oldState);

	bool isReadyForAction(model::ActionType action) const              { return m_cooldownTicks[action] == 0; }
	bool isGotStuck() const
	{
		return  std::hypot(m_self.getSpeedX(), m_self.getSpeedY()) < Point2D::k_epsilon
			&&  m_oldState.m_previousMove != nullptr
			&& (m_oldState.m_previousMove->getSpeed() > Point2D::k_epsilon || m_oldState.m_previousMove->getStrafeSpeed() > Point2D::k_epsilon); 
	}

	struct HistoryWriter
	{
		const State&   m_state;
		StorableState& m_storableState;

		HistoryWriter(const State& state, StorableState& oldState) : m_state(state), m_storableState(oldState) {}
		~HistoryWriter()                                           { m_storableState.m_previousMove = std::make_unique<model::Move>(m_state.m_move); }
	};
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
	StorableState                    m_oldState;
	std::unique_ptr<MapsManager>     m_maps;
	std::unique_ptr<PathFinder>      m_pathFinder;
	std::unique_ptr<Point2D>         m_spawnPoint;

	TWaypoints m_waypoints;
	int        m_currentWaypointIndex;

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

	auto getWizard(const model::Unit* unit)   const { return dynamic_cast<const model::Wizard*>(unit); }
	auto getMinion(const model::Unit* unit)   const { return dynamic_cast<const model::Minion*>(unit); }
	auto getBuilding(const model::Unit* unit) const { return dynamic_cast<const model::Building*>(unit); }

	double getSafeDistance(const model::Unit& enemy);
	template <typename UnitType> double getMaxDamage(const UnitType& u) const      { return u.getDamage(); }
	template <>                  double getMaxDamage(const model::Wizard& u) const { return m_state->m_game.getMagicMissileDirectDamage(); };  // TODO - calculate

	double getMaxDamage(const model::Unit* u) const
	{
		auto wizard = getWizard(u);
		if (wizard)
			return getMaxDamage(*wizard);
		
		auto minion = getMinion(u);
		if (minion)
			return getMaxDamage(*minion);

		auto builing = getBuilding(u);
		if (builing)
			return getMaxDamage(*builing);

		assert(false && "unknown unit type");
		return m_state->m_game.getMagicMissileDirectDamage();
	}

	bool isEnemy(const model::Unit& u) const  { return u.getFaction() != model::FACTION_NEUTRAL && u.getFaction() != m_state->m_self.getFaction(); }

public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;
};

#endif
