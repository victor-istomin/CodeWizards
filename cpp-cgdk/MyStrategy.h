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
class NavigationManager;

class MyStrategy : public Strategy 
{
public:
	typedef std::vector<Point2D>      TWaypoints;
	typedef std::map<model::LaneType, TWaypoints> TWaypointsMap;

	static const std::vector<model::SkillType> SKILLS_TO_LEARN;

	static const double WAYPOINT_RADIUS;
	static Point2D      TOP_GUARD_POINT;
	static Point2D      MID_GUARD_POINT;
	static Point2D      BOTTOM_GUARD_POINT;

private:

	static Point2D      BONUS_POINTS[];

	static TWaypointsMap g_waypointsMap;

	std::unique_ptr<DebugVisualizer>   m_visualizer;
	std::unique_ptr<State>             m_state;
	StorableState                      m_oldState;
	std::unique_ptr<MapsManager>       m_maps;
	std::unique_ptr<PathFinder>        m_pathFinder;
	std::unique_ptr<Point2D>           m_spawnPoint;
	std::unique_ptr<NavigationManager> m_navigation;

	int        m_currentWaypointIndex;
	const BonusSpawn* m_reasonableBonus;

	int      m_lastStrafeChangeTick;
	double   m_lastStrafe;
	Point2D* m_guardPoint;
	const model::Game*      m_game;
	mutable model::LaneType m_laneType;

	void initialSetup();
	void initState(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move, DebugMessage& debugMessage);


	const model::LivingUnit* getNearestTarget();
	const BonusSpawn* getReasonableBonus();

	// actions
// 	void goTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);
// 	void retreatTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage);

	double getPathLength(const Map::PointPath& path) const;

	void tryDisengage(model::Move &move);
	void learnSkill(model::Move& move);

public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;

	bool considerRetreat(model::Move& move, DebugMessage& debugMessage);
	bool considerAttack (model::Move &move, bool isRetreating, DebugMessage& debugMessage);
	void considerAnotherLane();

	double getSafeDistance(const model::Unit& enemy) const;

	double getMaxDamage(const model::Unit* u) const;
	template <typename UnitType> double getMaxDamage(const UnitType& u) const { return u.getDamage(); }
	// also, there is a specialization out of class scope

	const TWaypoints& getWaypoints() const                     { return g_waypointsMap[m_laneType]; }
	const TWaypoints& getWaypoints(model::LaneType lane) const { return g_waypointsMap[lane]; }

	void suggestLaneType(model::LaneType lane) const;  // TODO!!!
	int getTimeToChooseLane() const;

	MapsManager& getMaps() const { return *m_maps; }


	static auto getWizard(const model::Unit* unit)    { return dynamic_cast<const model::Wizard*>(unit); }
	static auto getMinion(const model::Unit* unit)    { return dynamic_cast<const model::Minion*>(unit); }
	static auto getBuilding(const model::Unit* unit) { return dynamic_cast<const model::Building*>(unit); }
	static auto getTree(const model::Unit* unit)     { return dynamic_cast<const model::Tree*>(unit); }
	static auto getPredicted(const model::Unit* unit) { return dynamic_cast<const PredictedUnit*>(unit); }

	static bool isUnitSeeing(const model::Unit* unit, const Point2D& point);
	static bool isEnemy(const model::Unit& u, const model::Wizard& self) { return u.getFaction() != model::FACTION_NEUTRAL && u.getFaction() != self.getFaction(); }

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
