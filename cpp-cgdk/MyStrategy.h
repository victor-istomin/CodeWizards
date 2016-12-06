#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#ifndef _DEBUG
#  ifndef NDEBUG
#    define NDEBUG
#  endif
#endif

#include "Strategy.h"
#include "model/LaneType.h"
#include "model/Projectile.h"
#include "LineEquation.h"
#include "Map.h"
#include "PathFinder.h"
#include "vec.h"

#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <array>
#include <cassert>
#include <typeinfo>
#include <typeindex>
#include "BonusSpawn.h"
#include "State.h"



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

	static const std::vector<model::SkillType> SKILLS_TO_LEARN;
	static const double WAYPOINT_RADIUS;

private:

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
	const BonusSpawn* m_reasonableBonus;

	int m_lastStrafeChangeTick;
	double m_lastStrafe;
	Point2D* m_guardPoint;

	void initialSetup();
	void initState(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move);

	Point2D getNextWaypoint();
	Point2D getPreviousWaypoint();

	const model::LivingUnit* getNearestTarget();
	const BonusSpawn* getReasonableBonus();

	// actions
	void goTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);
	void retreatTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);

	Map::PointPath getSmoothPathTo(const Point2D& point, const Map* map, PathFinder::TilesPath& tiles);
	double getPathLength(const Map::PointPath& path) const;

	void tryDisengage(model::Move &move);


	double getMaxDamage(const model::Unit* u) const;
	template <typename UnitType> double getMaxDamage(const UnitType& u) const      { return u.getDamage(); }
	// also, there is a specialization out of class scope

	bool isEnemy(const model::Unit& u) const  { return u.getFaction() != model::FACTION_NEUTRAL && u.getFaction() != m_state->m_self.getFaction(); }

	Vec2d getAlternateMoveVector(const Vec2d& suggestion);
	std::vector<const model::LivingUnit*> getDangerousEnemies() const;

	void learnSkill(model::Move& move);

// 	struct SpeedLimits 
// 	void getWizardMaxSpeed();

public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;

	double getSafeDistance(const model::Unit& enemy) const;

	static auto getWizard(const model::Unit* unit)    { return dynamic_cast<const model::Wizard*>(unit); }
	static auto getMinion(const model::Unit* unit)    { return dynamic_cast<const model::Minion*>(unit); }
	static auto getBuilding(const model::Unit* unit) { return dynamic_cast<const model::Building*>(unit); }
	static auto getTree(const model::Unit* unit)     { return dynamic_cast<const model::Tree*>(unit); }
	static auto getPredicted(const model::Unit* unit) { return dynamic_cast<const PredictedUnit*>(unit); }

	static bool isUnitSeeing(const model::Unit* unit, const Point2D& point);

	static bool hasStatus(const model::Wizard* unit, model::StatusType status)
	{ 
		const auto& ss = unit->getStatuses(); 
		return ss.end() != std::find_if(ss.begin(), ss.end(), [status](const auto& active) { return active.getType() == status; });
	}
};

template <> inline double MyStrategy::getMaxDamage<model::Wizard>(const model::Wizard& u) const { return m_state->m_game.getMagicMissileDirectDamage(); };  // TODO - calculate

inline bool operator==(const StorableState::ProjectileInfo& a, const model::Projectile& b)
{
	return a.m_id == b.getId();
}

inline bool operator==(const model::Projectile& a, const StorableState::ProjectileInfo& b)
{
	return b.m_id == a.getId();
}


#endif
