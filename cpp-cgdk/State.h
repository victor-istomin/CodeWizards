#pragma once
#include "BonusSpawn.h"
#include "Point2D.h"
#include "Vec.h"
#include "model/Move.h"

#include <vector>
#include <array>
#include <memory>
#include <typeindex>
#include <typeinfo>


typedef std::array<BonusSpawn, BonusSpawn::COUNT> BonusSpawns;
typedef std::vector<long long>                    Ids;

struct StorableState
{
	struct ProjectileInfo
	{
		long long      m_id;
		long long      m_ownerUnitId;
		model::Faction m_faction;
		int            m_detectionTick;
		Point2D        m_detectionPoint;
		double         m_radius;
		Vec2d          m_speed;
		Ids            m_possibleTargets;

		ProjectileInfo(const model::Projectile& p, int tick)
			: m_id(p.getId()), m_ownerUnitId(p.getOwnerUnitId()), m_faction(p.getFaction())
			, m_detectionTick(tick), m_detectionPoint(p), m_radius(p.getRadius()), m_speed(p.getSpeedX(), p.getSpeedY()) 
			, m_possibleTargets()
		{}

		friend bool operator==(const ProjectileInfo& a, const model::Projectile& b);
		friend bool operator==(const model::Projectile& a, const ProjectileInfo& b);
	};

	typedef std::vector<ProjectileInfo>  Projectiles;
	typedef std::unique_ptr<model::Move> MovePtr;

	MovePtr     m_previousMove;
	BonusSpawns m_bonuses; 
	Projectiles m_projectiles;

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

	typedef std::map<long long, const model::Unit*>   UnitById;
	typedef std::map<std::type_index, UnitById>       UnitByType;
	typedef std::vector<const model::Unit*> PointsVector;
	typedef std::vector<PredictedUnit>      PredictedUnits;
	typedef std::vector<model::SkillType>   Skills;
	typedef StorableState::Projectiles      Projectiles;

	const model::Wizard& m_self;
	const model::World&  m_world;
	const model::Game&   m_game;
	const model::Move&   m_move;
	const StorableState& m_storedState;
	BonusSpawns          m_bonuses;
	PredictedUnits       m_enemySpawnPredictions;
	Skills               m_learnedSkills;
	Disposition          m_disposionAround;
	Projectiles          m_projectileInfos;
	Projectiles          m_dangerousProjectiles;
	UnitByType           m_units;
	const MyStrategy*    m_strategy;

	int    m_nextMinionRespawnTick;
	double m_estimatedHP;
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
	bool isUnderMissile() const                                        { return !m_dangerousProjectiles.empty(); }
	bool isGotStuck() const
	{
		return  std::hypot(m_self.getSpeedX(), m_self.getSpeedY()) < Point2D::k_epsilon
			&&  m_storedState.m_previousMove != nullptr
			&& (m_storedState.m_previousMove->getSpeed() > Point2D::k_epsilon || m_storedState.m_previousMove->getStrafeSpeed() > Point2D::k_epsilon); 
	}

	template <class Type> const Type* getUnit(long long id) const
	{
		const Type* unit = nullptr;
		auto byTypeIt = m_units.find(std::type_index(typeid(Type)));
		assert(byTypeIt != m_units.end() && "unknown unit type");

		if (byTypeIt != m_units.end())
		{
			auto byIdIt = byTypeIt->second.find(id);
			unit = byIdIt != byTypeIt->second.end() ? dynamic_cast<const  Type*>(byIdIt->second) : nullptr;
		}

		return unit;
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

			m_storableState.m_projectiles = m_state.m_projectileInfos;
		}
	};
};

