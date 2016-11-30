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
#include "LineEquation.h"
#include "Map.h"
#include "PathFinder.h"

#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <array>
#include <cassert>

struct BonusSpawn
{
	enum BonusState
	{
		UNKNOWN = 0,   // don't know whether there is a bonus
		HAS_BONUS,
		NO_BONUS,
	};

	struct WizardsHealth
	{
		double enemies;
		double teammates;

		WizardsHealth() : enemies(0), teammates(0) {}
	};

	Point2D    m_point;
	BonusState m_state;          // true if someone has collected bonus
	int        m_lastCheckTick;
	int        m_teamateCompetitors;
	double     m_dangerHandicap;

	Map::PointPath m_smoothPathCache;
	Map::TilesPath m_tilesPathCache;
	WizardsHealth  m_wizardsHp;

	static const size_t  COUNT = 2;
	static const Point2D RESPAWN_POINTS[];
	static const double  DANGER_HANDICAP;

	BonusSpawn(const Point2D& point, BonusState state, double handicap = 0) 
		: m_point(point), m_state(state), m_lastCheckTick(0), m_dangerHandicap(handicap), m_teamateCompetitors(0) {}
};

typedef std::array<BonusSpawn, BonusSpawn::COUNT> BonusSpawns;

struct StorableState
{
	std::unique_ptr<model::Move>  m_previousMove;
	BonusSpawns m_bonuses; 

	StorableState() 
		: m_previousMove()
		, m_bonuses{ BonusSpawn(BonusSpawn::RESPAWN_POINTS[0], BonusSpawn::NO_BONUS, BonusSpawn::DANGER_HANDICAP),   // because it's closer to enemy's tower on mid lane
					 BonusSpawn(BonusSpawn::RESPAWN_POINTS[1], BonusSpawn::NO_BONUS) }
	{}
};

class MyStrategy;
struct State
{
	static const double LOW_HP_FACTOR;
	static const int    COOLDOWN_INF = 0xFFFF;

	struct Disposition
	{
		int enemyWizards;
		int enemyMinions;
		int enemyBuildings;
		int teammateWizards;
		int teammateMinions;
		int teammateBuildings;

		double movableEnemyHP;
		double movebleTeammatesHP;

		Disposition() 
			: enemyWizards(0), enemyMinions(0), enemyBuildings(0), movableEnemyHP(0.0)
			, teammateWizards(0), teammateMinions(0), teammateBuildings(0), movebleTeammatesHP(0.0) {}

		int enemiesTotal() const   { return enemyWizards + enemyBuildings + enemyMinions; }
		int teammatesCount() const { return teammateWizards + teammateBuildings + teammateMinions; }
	};


	typedef std::vector<const model::Unit*> PointsVector;
	typedef std::vector<PredictedUnit>      PredictedUnits;
	typedef std::vector<model::SkillType>   Skills;

	const model::Wizard& m_self;
	const model::World&  m_world;
	const model::Game&   m_game;
	const model::Move&   m_move;
	const StorableState& m_storedState;
	BonusSpawns          m_bonuses;
	PredictedUnits       m_enemySpawnPredictions;
	Skills               m_learnedSkills;
	Disposition          m_disposionAround;
	const MyStrategy*    m_strategy;

	int    m_nextMinionRespawnTick;
	double m_estimatedHP;
	bool   m_isUnderMissile;
	bool   m_isLowHP;
	bool   m_isHastened;
	bool   m_isGoingToBonus;  // not yet implemented, always false

	std::array<int, model::_ACTION_COUNT_> m_cooldownTicks;

	State(const MyStrategy* strategy, const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move, const StorableState& m_oldState);

	void updateProjectiles();
	void updateBonuses();
	void updatePredictions();
	void updateSkillsAndActions();
	void updateDispositionAround();

	int lastBonusSpawnTick() const { return (m_world.getTickIndex() / m_game.getBonusAppearanceIntervalTicks()) * m_game.getBonusAppearanceIntervalTicks(); }
	int nextBonusSpawnTick() const { return lastBonusSpawnTick() + m_game.getBonusAppearanceIntervalTicks(); }

	bool isReadyForAction(model::ActionType action) const              { return m_cooldownTicks[action] == 0; }
	bool isGotStuck() const
	{
		return  std::hypot(m_self.getSpeedX(), m_self.getSpeedY()) < Point2D::k_epsilon
			&&  m_storedState.m_previousMove != nullptr
			&& (m_storedState.m_previousMove->getSpeed() > Point2D::k_epsilon || m_storedState.m_previousMove->getStrafeSpeed() > Point2D::k_epsilon); 
	}

	struct HistoryWriter
	{
		const State&   m_state;
		StorableState& m_storableState;

		HistoryWriter(const State& state, StorableState& oldState) : m_state(state), m_storableState(oldState) {}
		~HistoryWriter()
		{ 
			m_storableState.m_previousMove = std::make_unique<model::Move>(m_state.m_move); 
			std::copy(m_state.m_bonuses.begin(), m_state.m_bonuses.end(), m_storableState.m_bonuses.begin());
		}
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

	static const model::SkillType SKILLS_TO_LEARN[];
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

	void learnSkill(model::Move& move);

public:
    MyStrategy();

    void move(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) override;

	double getSafeDistance(const model::Unit& enemy) const;

	static auto getWizard(const model::Unit* unit)    { return dynamic_cast<const model::Wizard*>(unit); }
	static auto getMinion(const model::Unit* unit)    { return dynamic_cast<const model::Minion*>(unit); }
	static auto getBuilding(const model::Unit* unit)  { return dynamic_cast<const model::Building*>(unit); }
	static auto getPredicted(const model::Unit* unit) { return dynamic_cast<const PredictedUnit*>(unit); }

	static bool isUnitSeeing(const model::Unit* unit, const Point2D& point);

	static bool hasStatus(const model::Wizard* unit, model::StatusType status)
	{ 
		const auto& ss = unit->getStatuses(); 
		return ss.end() != std::find_if(ss.begin(), ss.end(), [status](const auto& active) { return active.getType() == status; });
	}
};

template <> inline double MyStrategy::getMaxDamage<model::Wizard>(const model::Wizard& u) const { return m_state->m_game.getMagicMissileDirectDamage(); };  // TODO - calculate




#endif
