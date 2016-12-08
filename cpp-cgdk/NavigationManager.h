#pragma once
#include "model/Move.h"
#include "model/LivingUnit.h"

#include "BonusSpawn.h"
#include "Map.h"
#include "PathFinder.h"
#include "DebugVisualizer.h"

#include <vector>
#include <cmath>

struct State;
class MyStrategy;

class NavigationManager
{
public:
	typedef std::vector<Point2D> Waypoints;

	explicit NavigationManager(const MyStrategy& strategy, const State& state, const Waypoints& waypoints, const Point2D& guardPoint, PathFinder& pathFinder, DebugMessage& debugMessage);

	void pursuitEnemy(const model::LivingUnit* unit)           { m_pursuitList.push_back(unit); }
	void setForcePreserveAngle(bool shouldPreserve)            { m_forcePreserveAngle = true; }
	void setInCombatMode(bool isInCombat)                      { m_isInCombat = isInCombat; }
	void setBonusPoint(const BonusSpawn* bonus)                { m_bonus = bonus; }
	void setRetreatMode(bool isRetreating)                     { m_isRetreating = isRetreating; }

	void makeMove(model::Move& move);

	Map::PointPath getSmoothPathTo(const Point2D& point, PathFinder::TilesPath& tiles);


private:
	typedef std::vector<const model::LivingUnit*> LivingUnits;

	const MyStrategy&  m_strategy;
	const State&       m_state;
	const Waypoints&   m_waypoints;
	const Point2D&     m_guardPoint;
	PathFinder&        m_pathFinder;
	const MapsManager& m_maps;
	DebugMessage&      m_debugMessage;

	LivingUnits        m_pursuitList;
	const BonusSpawn*  m_bonus;
	bool               m_isRetreating;
	bool               m_forcePreserveAngle;
	bool               m_isInCombat;


	bool stageAvoidProjectiles(model::Move& move);
	bool stagePursuit(model::Move& move);  // try pursuit enemy
	bool stageBonus(model::Move& move);    // try getting bonus
	bool stageRetreat(model::Move& move);  // try retreat if needed
	bool stagePushLine(model::Move& move);
	bool stageInCombat(model::Move& move); // in-combat moving, except retreating

	bool goTo(const Point2D& point, model::Move& move, bool preserveAngle = false, bool usePathfinding = true);
	bool isPathAcceptable(const Vec2d& moveVector, const Map::PointPath& smoothPath);

	Vec2d getAlternateMoveVector(const Vec2d& suggestion);

	Point2D getNextWaypoint();

	size_t GetCurrentWaypointIndex();

	Point2D getPreviousWaypoint();

	long long getTeammateIdToHelp() const;

	struct Limits
	{
		double forward, backward, strafe;

		void multiply(double factor)
		{
			forward  *= factor;
			backward *= factor;
			strafe   *= factor;
		}

		double hypot(bool isForward) const { return std::hypot((isForward ? forward : backward), strafe); }
	};

	Limits getMaxSpeed(const model::Wizard* wizard);
	void applySpeedLimit(Vec2d& moveVector, const Limits& speedLimit);
};

