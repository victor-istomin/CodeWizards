﻿#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include "MyStrategy.h"
#include "model/LivingUnit.h"
#include "DebugVisualizer.h"
#include "PathFinder.h"
#include "MapsManager.h"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <list>
#include <algorithm>
#include <iterator>
#include <limits>
#include <numeric>

#include "Fringe.h"


using namespace model;
using namespace std;

const double MyStrategy::WAYPOINT_RADIUS = 150.0;

Point2D MyStrategy::MID_GUARD_POINT    = Point2D(2000 + 200, 2000 + 200);
Point2D MyStrategy::TOP_GUARD_POINT    = Point2D(35, 2000 - 400 + 35);
Point2D MyStrategy::BOTTOM_GUARD_POINT = Point2D(2000 + 400 - 35, 4000 - 400 + 35);

// order is importand due to handicap
const Point2D BonusSpawn::RESPAWN_POINTS[] = { Point2D(4000 * 0.3, 4000 * 0.3), Point2D(4000 * 0.7, 4000 * 0.7) };
const double  BonusSpawn::DANGER_HANDICAP = 500;

// currently, these skill branches only
const model::SkillType MyStrategy::SKILLS_TO_LEARN[] = 
{ 
	/* MR damage and frost-bolt */ SKILL_MAGICAL_DAMAGE_BONUS_PASSIVE_1, SKILL_MAGICAL_DAMAGE_BONUS_AURA_1, SKILL_MAGICAL_DAMAGE_BONUS_PASSIVE_2, SKILL_MAGICAL_DAMAGE_BONUS_AURA_2, SKILL_FROST_BOLT,
	/* haste */ SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_1, SKILL_MOVEMENT_BONUS_FACTOR_AURA_1, SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_2, SKILL_MOVEMENT_BONUS_FACTOR_AURA_2, SKILL_HASTE,
	/* range */SKILL_RANGE_BONUS_PASSIVE_1, SKILL_RANGE_BONUS_AURA_1, SKILL_RANGE_BONUS_PASSIVE_2, SKILL_RANGE_BONUS_AURA_2, SKILL_ADVANCED_MAGIC_MISSILE 
};

// TODO - remove this from globals!
MyStrategy::TWaypointsMap MyStrategy::g_waypointsMap;

const double State::LOW_HP_FACTOR = 0.3;
const double Point2D::k_epsilon   = 0.0001;

#include <iostream>


void MyStrategy::move(const Wizard& self, const World& world, const Game& game, Move& move) 
{
	Timer timer(__FUNCTION__, world.getMyPlayer().getName(), self.getOwnerPlayerId());

	initState(self, world, game, move);
	State::HistoryWriter updater(*m_state, m_oldState);
	DebugMessage debugMessage(*m_visualizer, self, world);
	debugMessage.visualizePredictions(m_state->m_enemySpawnPredictions);

	learnSkill(move);

	/* test */
	static bool bIsMapNeeded = false;
	if (bIsMapNeeded)
	{
		Timer pathTimer("getPath");
		const Map* map = m_maps->getMap(MapsManager::MT_WORLD_MAP);

 		debugMessage.visualizeMap(map);
		debugMessage.visualizeWaypoints(g_waypointsMap);
		debugMessage.visualizeBonuses(m_state->m_bonuses);
	}

	const LivingUnit* nearestTarget = getNearestTarget();

	auto dangerousEnemies = getDangerousEnemies();
	double totalEnemiesDamage = std::accumulate(dangerousEnemies.begin(), dangerousEnemies.end(), 0.0, 
		[this](double sum, const model::Unit* enemy) {return sum + getMaxDamage(enemy); });

	bool isNearOrk = dangerousEnemies.end() != std::find_if(dangerousEnemies.begin(), dangerousEnemies.end(),
		[this](const Unit* u) { return getMinion(u) != nullptr && getMinion(u)->getType() == model::MINION_ORC_WOODCUTTER; });

	bool isTooCloseToEnemy = isNearOrk || self.getLife() < totalEnemiesDamage;
	if (m_state->m_estimatedHP < totalEnemiesDamage)
	{
		m_state->m_isLowHP = true;  // this also activates "don't retreat too far" feature
	}

	const State::Disposition& around = m_state->m_disposionAround;
	double relativeEnemiesAmount = around.movableEnemyHP / around.movebleTeammatesHP;
	const double rushHpThreshold = around.teammateMinions != 0 ? 3.0 : 2.0;
	bool isEnemyRushing = relativeEnemiesAmount >= 2.0 && around.enemyWizards != 0;  // TODO - take teammate towers into account?

	// Если осталось мало жизненной энергии, отступаем к предыдущей ключевой точке на линии.
	bool isRetreating = isTooCloseToEnemy || m_state->m_isLowHP || isEnemyRushing;
	if (isRetreating) 
	{
		// TODO - consider retreating to most safe point both in case of bonus or not bonus

		Point2D previousWaypoint = getPreviousWaypoint();
		if (m_reasonableBonus)
		{
			bool canGet = !m_state->m_isLowHP;
			if (m_state->m_isLowHP)
			{
				const Point2D& nextPathPoint = m_reasonableBonus->m_smoothPathCache.empty() ? m_reasonableBonus->m_point : m_reasonableBonus->m_smoothPathCache.front();
				auto enemies = filterPointers<const model::Unit*>([this](const model::Unit& u) {return isEnemy(u); }, world.getBuildings(), world.getWizards(), world.getMinions());

				bool noEnemiesThatWay = enemies.end() == std::find_if(enemies.begin(), enemies.end(), 
					[&nextPathPoint, &self](const model::Unit* enemy) 
				{
					return std::abs(self.getAngleTo(*enemy) - self.getAngleTo(nextPathPoint.m_x, nextPathPoint.m_y)) > (PI / 2);
				});

				canGet = noEnemiesThatWay;
			}

			if (canGet)
				previousWaypoint = m_reasonableBonus->m_point;
		}

		retreatTo(previousWaypoint, move, debugMessage);
		debugMessage.setNextWaypoint(previousWaypoint);
	}

	// Если видим противника ...
	if (nearestTarget != nullptr) 
	{
		double distance = self.getDistanceTo(*nearestTarget);

		// ... и он в пределах досягаемости наших заклинаний, ...
		const double shootingDistance = self.getCastRange();
		if (distance <= shootingDistance) 
		{
			double angle = self.getAngleTo(*nearestTarget);

			// ... то поворачиваемся к цели.
			move.setTurn(angle);

			bool isWizard = getWizard(nearestTarget) != nullptr;
			if (isWizard && !isRetreating)
			{
				// might escape. This is workaround
				double enenyAngle = nearestTarget->getAngleTo(self);
				if (std::abs(enenyAngle) > game.getStaffSector())
				{
					double ticksToTurn = 1 + (std::abs(enenyAngle) - game.getStaffSector()) / game.getWizardMaxTurnAngle();
					double desiredDistance = shootingDistance - ticksToTurn * game.getWizardStrafeSpeed();

					if (self.getDistanceTo(*nearestTarget) > desiredDistance)
					{
						goTo(*nearestTarget, move, debugMessage);
					}
				}
			}

			// Если цель перед нами, ...

			// TODO - more accurate mana regeneration
			const double damageMm      = m_state->m_game.getMagicMissileDirectDamage();
			const double damageFB      = m_state->m_game.getFrostBoltDirectDamage();
			const double shootingAngle = game.getStaffSector() / 2.0;

			double frostCooldownMana = m_state->m_game.getFrostBoltCooldownTicks() * m_state->m_game.getWizardBaseManaRegeneration()
			                         - m_state->m_game.getMagicMissileManacost(); // usually, we're firing MM while FB cools down

			bool isLastFrostBolt = ((m_state->m_self.getMana() + frostCooldownMana) / m_state->m_game.getFrostBoltManacost()) < 2.0;

			auto targetMinion = getMinion(nearestTarget);
			auto targetWizard = getWizard(nearestTarget);
			bool canTakedownWithFBMM = nearestTarget->getLife() > damageMm && nearestTarget->getLife() < (damageFB + damageMm);

			bool shouldShootLastFB = isLastFrostBolt &&
				(isRetreating
					|| (targetMinion != nullptr && targetMinion->getType() == model::MINION_FETISH_BLOWDART && canTakedownWithFBMM)
					|| (targetWizard != nullptr && canTakedownWithFBMM)
					/*TODO: || isKamikazeMode - shoot before die*/);

			if (m_state->isReadyForAction(ACTION_FROST_BOLT) && shouldShootLastFB)
			{
				std::vector<const model::LivingUnit*> candidates;
				candidates.reserve(16);

				std::copy_if(dangerousEnemies.begin(), dangerousEnemies.end(), std::back_inserter(candidates), 
					[this, shootingDistance, shootingAngle](const LivingUnit* unit)
				{
					return m_state->m_self.getDistanceTo(*unit) <= shootingDistance
						&& std::abs(m_state->m_self.getAngleTo(*unit)) < shootingAngle;
				});

				std::sort(candidates.begin(), candidates.end(), [](const auto* left, const auto& right) {return left->getLife() < right->getLife(); });

				// reverse iteration, because rightmost candidate has more health

				auto canKillWizardIt = std::find_if(candidates.rbegin(), candidates.rend(),
					[this, damageFB](const model::LivingUnit* u) {return getWizard(u) && u->getLife() < damageFB; });

				auto canAlmostKillWizardIt = std::find_if(candidates.rbegin(), candidates.rend(),
					[this, damageFB, damageMm](const model::LivingUnit* u) {return getWizard(u) && u->getLife() < (damageFB + damageMm); });

				auto hasBonusIt = std::find_if(candidates.rbegin(), candidates.rend(),
					[this, damageFB, damageMm](const model::LivingUnit* u) 
				{
					auto wizard = getWizard(u); 
					return wizard &&
						(hasStatus(wizard, model::STATUS_EMPOWERED) || hasStatus(wizard, model::STATUS_HASTENED) || hasStatus(wizard, model::STATUS_SHIELDED));
				});

				auto canKillAnyIt = std::find_if(candidates.rbegin(), candidates.rend(), 
					[this, damageFB](const model::LivingUnit* u) { return u->getLife() < damageFB; });

				auto canFreezeWizardIt = std::find_if(candidates.rbegin(), candidates.rend(), 
					[this](const model::LivingUnit* u) { return getWizard(u) != nullptr; });

				if (canKillWizardIt != candidates.rend())
					nearestTarget = *canKillWizardIt;
				else if (canAlmostKillWizardIt != candidates.rend())
					nearestTarget = *canAlmostKillWizardIt;
				else if (hasBonusIt != candidates.rend())
					nearestTarget = *hasBonusIt;
				else if (canKillAnyIt != candidates.rend())
					nearestTarget = *canKillAnyIt;
				else if (canFreezeWizardIt != candidates.rend())
					nearestTarget = *canFreezeWizardIt;  // freeze most healthy enemy wizard when retreating (less healthy may be shoot soon)
				else if (!candidates.empty())
					nearestTarget = candidates.front();  // with minimal health

				angle = m_state->m_self.getAngleTo(*nearestTarget);
				distance = m_state->m_self.getDistanceTo(*nearestTarget);

				move.setAction(ActionType::ACTION_FROST_BOLT);
				move.setCastAngle(angle);
				move.setMinCastDistance(distance - nearestTarget->getRadius() + game.getFrostBoltRadius());
			}
			else if (m_state->isReadyForAction(ACTION_FROST_BOLT) && !isLastFrostBolt)
			{
				if (abs(angle) < shootingAngle)
				{
					// ... то атакуем.
					move.setAction(ActionType::ACTION_FROST_BOLT);
					move.setCastAngle(angle);
					move.setMinCastDistance(distance - nearestTarget->getRadius() + game.getFrostBoltRadius());
				}
			}
			else if (m_state->isReadyForAction(ActionType::ACTION_MAGIC_MISSILE))
			{
				if (abs(angle) < game.getStaffSector() / 2.0)
				{
					// ... то атакуем.
					move.setAction(ActionType::ACTION_MAGIC_MISSILE);
					move.setCastAngle(angle);
					move.setMinCastDistance(distance - nearestTarget->getRadius() + game.getMagicMissileRadius());
				}
			}
			else if (m_state->isReadyForAction(ActionType::ACTION_STAFF))
			{
				if (distance < game.getStaffRange() + nearestTarget->getRadius())
				{
					move.setAction(ActionType::ACTION_STAFF);
				}
			}

			if (m_reasonableBonus)
			{
				retreatTo(m_reasonableBonus->m_point, move, debugMessage);
			}

			return;
		}
	}

	// Если нет других действий, просто продвигаемся вперёд.
	if (move.getSpeed() < Point2D::k_epsilon && std::abs(move.getTurn() < PI/1000) && !isRetreating)
	{
		Point2D nextWaypoint = getNextWaypoint();

		if (m_reasonableBonus)
			nextWaypoint = m_reasonableBonus->m_point;

		// don't push too early
		const int prepareTicks = m_state->m_game.getFactionMinionAppearanceIntervalTicks()
			+ (m_guardPoint->getDistanceTo(MID_GUARD_POINT) < Point2D::k_epsilon ? 200 : 100);

		bool isRushTime = (m_state->m_world.getTickIndex() > prepareTicks);
		if (!isRushTime && m_guardPoint->getDistanceTo(self) < WAYPOINT_RADIUS)
			nextWaypoint = *m_guardPoint;

		if (nextWaypoint.getDistanceTo(self) > self.getRadius())
		{
			goTo(nextWaypoint, move, debugMessage);
		}
		else
		{
			nextWaypoint = getNextWaypoint(); 
			double angle = m_state->m_self.getAngleTo(nextWaypoint.m_x, nextWaypoint.m_y);

			// turn, but not move
			move.setTurn(angle);
		}
	}
}

std::vector<const model::LivingUnit*> MyStrategy::getDangerousEnemies() const
{
	const Point2D enemyBase{ m_state->m_world.getWidth() - 400, 400 };
	bool isBetweenSpawnAndCorner = false;
	for (const PredictedUnit& spawn : m_state->m_enemySpawnPredictions)
		isBetweenSpawnAndCorner = isBetweenSpawnAndCorner || enemyBase.getDistanceTo(m_state->m_self) < enemyBase.getDistanceTo(spawn)*1.2/*hack*/;

	double spawnRadius = m_state->m_enemySpawnPredictions.empty() ? 1 : m_state->m_enemySpawnPredictions.front().getRadius();
	int spawnTraversalTicks = static_cast<int>((isBetweenSpawnAndCorner ? spawnRadius * 2 : spawnRadius) / m_state->m_game.getWizardStrafeSpeed());
	int noSpawnSafeTicks = m_state->m_nextMinionRespawnTick - m_state->m_world.getTickIndex() - spawnTraversalTicks;

	auto emptyPredictions = std::vector<PredictedUnit>();
	auto dangerousEnemies = filterPointers<const model::LivingUnit*>(
		[this](const model::Unit& u) {return isEnemy(u) && m_state->m_self.getDistanceTo(u) < getSafeDistance(u); },
		m_state->m_world.getBuildings(), m_state->m_world.getWizards(), m_state->m_world.getMinions(),
		(noSpawnSafeTicks > 0 ? emptyPredictions : m_state->m_enemySpawnPredictions));

	return std::move(dangerousEnemies);
}

bool MyStrategy::isUnitSeeing(const model::Unit* unit, const Point2D& point)
{
	double distance = point.getDistanceTo(*unit);
	auto wizard = getWizard(unit);
	auto minion = getMinion(unit);
	auto builing = getBuilding(unit);
	return (wizard && wizard->getVisionRange() > distance)
		|| (minion && minion->getVisionRange() > distance)
		|| (builing && builing->getVisionRange() > distance);  // other units have no eyes
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void MyStrategy::initialSetup()
{
	// TODO - add some waypoints to TOP/BOTTOM lane to avoid going out of lane like in game# 42857

	m_spawnPoint = std::make_unique<Point2D>(m_state->m_self);

	m_pathFinder = std::make_unique<PathFinder>();

	srand((unsigned)time(nullptr));

	double mapSize = m_state->m_game.getMapSize();
	double wizardSize = m_state->m_self.getRadius();

	MyStrategy::MID_GUARD_POINT = Point2D(mapSize * 0.5 - 200, mapSize * 0.5 + 300);
	g_waypointsMap[LaneType::LANE_MIDDLE] = TWaypoints
	{ 
		Point2D(100.0, mapSize - 100.0),
//		Point2D(600.0, mapSize - 200.0),
		Point2D(800.0, mapSize - 800.0),
		MID_GUARD_POINT,
		Point2D(2400.0, 1500.0),
		Point2D(mapSize - 100.0, 100.0),
	};

	MyStrategy::TOP_GUARD_POINT = Point2D(wizardSize, mapSize * 0.5 - 400 + wizardSize);
	g_waypointsMap[LaneType::LANE_TOP] = TWaypoints
	{
		Point2D(100.0, mapSize - 100.0),
		Point2D(300.0, mapSize - 1200.0),
		TOP_GUARD_POINT,
		Point2D(1200, 100.0),
		Point2D(mapSize - 100.0, 100.0)
	};

	MyStrategy::BOTTOM_GUARD_POINT = Point2D(mapSize * 0.5 + 400 - wizardSize, mapSize - 330 + wizardSize);
	g_waypointsMap[LaneType::LANE_BOTTOM] = TWaypoints
	{
		Point2D(100.0, mapSize - 100.0),
		Point2D(1400.0, mapSize - 100.0),
		BOTTOM_GUARD_POINT,
		Point2D(mapSize - 200.0, mapSize * 0.5),
		Point2D(mapSize - 100.0, 100.0)
	};

	LaneType line = LaneType::_LANE_UNKNOWN_;
	switch (m_state->m_self.getId()) 
	{
	case 1:
	case 2:
	case 6:
	case 7:
		line = LANE_TOP;
		m_guardPoint = &TOP_GUARD_POINT;
		break;

	case 3:
	case 8:
		line = LANE_MIDDLE;
		m_guardPoint = &MID_GUARD_POINT;
		break;

	case 4:
	case 5:
	case 9:
	case 10:
		line = LANE_BOTTOM;
		m_guardPoint = &BOTTOM_GUARD_POINT;
		break;

	default:
		break;
	}

	m_waypoints = g_waypointsMap[line];
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void MyStrategy::initState(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move)
{
	m_state = std::make_unique<State>(this, self, world, game, move, m_oldState);
	m_state->updateDispositionAround();

	if (m_waypoints.empty())
	{
		// initial setup
		initialSetup();
	}

	if (!m_maps)
	{
		m_maps = std::make_unique<MapsManager>(game, world, self, *m_pathFinder);
	}
	else
	{
		m_maps->update(game, world, self, *m_pathFinder);
	}

	// respawn detect
	if (*m_spawnPoint == self)
	{
		// maybe, add additional clock tick checks for more complex actions
		m_currentWaypointIndex = 1;   // [0] waypoint is for retreating only
	}

	m_reasonableBonus = getReasonableBonus();
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
Point2D MyStrategy::getNextWaypoint()
{
	// assume that waypoint are sorted by-distance !!!

	int lastWaypointIndex = m_waypoints.empty() ? 0 : m_waypoints.size() - 1;

	Point2D currentWaypoint = m_waypoints[m_currentWaypointIndex];
	if (currentWaypoint.getDistanceTo(m_state->m_self) < WAYPOINT_RADIUS && m_currentWaypointIndex < lastWaypointIndex)
	{
		currentWaypoint = m_waypoints[++m_currentWaypointIndex];
	}

	return currentWaypoint;
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
Point2D MyStrategy::getPreviousWaypoint()
{
	// assume that waypoint are sorted by-distance !!!

	Point2D previousWaypoint = m_waypoints[std::max(0, m_currentWaypointIndex - 1)];

	if (previousWaypoint.getDistanceTo(m_state->m_self) <= WAYPOINT_RADIUS && m_currentWaypointIndex > 0)
		previousWaypoint = m_waypoints[std::max(0, (--m_currentWaypointIndex) - 1)];

	return previousWaypoint;
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
const model::LivingUnit* MyStrategy::getNearestTarget()
{
	const World& world = m_state->m_world;
	const Wizard& self = m_state->m_self;

	auto targets = filterPointers<const model::LivingUnit*>([&self](const model::LivingUnit& u) { return u.getFaction() != self.getFaction(); },
		world.getBuildings(), world.getWizards(), world.getMinions());

	const LivingUnit* nearestTarget = nullptr;

	double nearestTargetDistance = std::numeric_limits<double>::max();
	int minEnemyHealth = std::numeric_limits<int>::max();
	int maxEnemyHealth = std::numeric_limits<int>::min();

	for (const LivingUnit* target : targets)
	{
		if (target->getFaction() == Faction::FACTION_NEUTRAL)
		{
			continue;
		}

		minEnemyHealth = std::min(minEnemyHealth, target->getLife());
		maxEnemyHealth = std::max(maxEnemyHealth, target->getLife());
	}

	for (const LivingUnit* target : targets) 
	{
		if (target->getFaction() == Faction::FACTION_NEUTRAL)
		{
			continue;
		}

		double distance =  self.getDistanceTo(*target);
		if (target->getLife() == minEnemyHealth)
		{
			distance /= 2;  // this is priority hack
		}
		if (target->getLife() == maxEnemyHealth)
		{
			distance *= 1.5;  // this is priority hack
		}

		if (distance < nearestTargetDistance) 
		{
			nearestTarget = target;
			nearestTargetDistance = distance;
		}
	}

	return nearestTarget;
}

const BonusSpawn* MyStrategy::getReasonableBonus()
{
	const double MAX_TRAVEL_DISTANCE = m_state->m_game.getWizardVisionRange() * 3 + BonusSpawn::DANGER_HANDICAP;

	const model::Wizard& self = m_state->m_self;
	const int thisTick = m_state->m_world.getTickIndex();

	const BonusSpawn* nearest = nullptr;
	double minPathDistance = std::numeric_limits<double>::infinity();

	auto enemiesAround = getDangerousEnemies();

	for (BonusSpawn& spawn : m_state->m_bonuses)
	{
		if (spawn.m_point.getDistanceTo(self) > MAX_TRAVEL_DISTANCE)
			continue;  // path can't be shorter than straight line

		const Map* map = m_maps->getMap(MapsManager::MT_WORLD_MAP);
		spawn.m_smoothPathCache = getSmoothPathTo(spawn.m_point, map, spawn.m_tilesPathCache);
		const Point2D nextStepPoint = spawn.m_smoothPathCache.empty() ? spawn.m_point : spawn.m_smoothPathCache.front();

		auto isEnemyInBetweenPredicate = [&self, &nextStepPoint](const model::LivingUnit* enemy)
		{
			static const double THRESHOLD = PI / 3.0;
			return std::abs(self.getAngleTo(*enemy) - self.getAngleTo(nextStepPoint.m_x, nextStepPoint.m_y)) < THRESHOLD;
		};

		double betweenEnemiesHp = std::accumulate(enemiesAround.begin(), enemiesAround.end(), 0.0,
			[&isEnemyInBetweenPredicate](double summ, const model::LivingUnit* enemy) { return summ + (isEnemyInBetweenPredicate(enemy) ? enemy->getLife() : 0.0); });

		if (self.getLife() < betweenEnemiesHp)
			continue;  // "do it or die" is not too reasonable

		double currentDistance = getPathLength(spawn.m_smoothPathCache) + spawn.m_dangerHandicap;

		if (currentDistance < MAX_TRAVEL_DISTANCE
			&& (nearest == nullptr || currentDistance < minPathDistance)
			&& spawn.m_teamateCompetitors == 0)   // not too reasonable to compete with teammate
		{
			nearest = &spawn;
			minPathDistance = currentDistance;
		}
	}

	const double MAX_TRAVEL_TIME = std::min(400, m_state->m_world.getTickCount() - m_state->m_world.getTickIndex());

	if (nearest != nullptr && minPathDistance < MAX_TRAVEL_DISTANCE)
	{
		double distance = nearest->m_point.getDistanceTo(self);
		double eta = distance / m_state->m_game.getWizardStrafeSpeed();
		double timeToAppear = nearest->m_state == BonusSpawn::NO_BONUS ? m_state->nextBonusSpawnTick() - thisTick : 0;

		if (timeToAppear < eta && eta < MAX_TRAVEL_TIME)
		{
			const double WAIT_DISTANCE = self.getRadius() * 2.5;
			if (distance < WAIT_DISTANCE && nearest->m_state == BonusSpawn::NO_BONUS)
			{
				static BonusSpawn hack = BonusSpawn(*nearest);
				hack = *nearest;
				hack.m_point = self;  // wait point (TODO: remove dirty hack)

				return &hack; // waif for appearing
			}
			else
			{
				return nearest;
			}
		}
	}

	return nullptr;
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void MyStrategy::goTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage)
{
	Timer timer(__FUNCTION__);

	Point2D target = point;

	PathFinder::TilesPath path;
	const Map* map = m_maps->getMap(MapsManager::MT_WORLD_MAP);
	Map::PointPath smoothPath; 
	
	if (m_reasonableBonus != nullptr && m_reasonableBonus->m_point == point)
	{
		smoothPath = m_reasonableBonus->m_smoothPathCache;
		path = m_reasonableBonus->m_tilesPathCache;

		if (!m_state->m_isHastened && m_state->isReadyForAction(ACTION_HASTE))
		{
			move.setAction(ACTION_HASTE);
			move.setStatusTargetId(m_state->m_self.getId());
		}
	}
	else
	{
		smoothPath = getSmoothPathTo(target, map, path);
	}

	debugMessage.setNextWaypoint(point);
	debugMessage.visualizePath(path, map);

	// this may occur when navigating to the enemy in occupied cell
	//assert(!smoothPath.empty());
	if (!smoothPath.empty())
	{
		target = smoothPath.front();
	}
	
	double angle = m_state->m_self.getAngleTo(target.m_x, target.m_y);

	move.setTurn(angle);

	double forwardSpeed = m_state->m_game.getWizardForwardSpeed();
	if (m_state->m_isHastened)
		forwardSpeed += forwardSpeed * m_state->m_game.getHastenedMovementBonusFactor();

	if (abs(angle) < m_state->m_game.getStaffSector() / 4.0)
	{
		Vec2d vect = Vec2d(forwardSpeed * std::cos(angle), forwardSpeed * std::sin(angle));
		Vec2d alt = getAlternateMoveVector(vect);
		move.setSpeed(alt.m_x);
		move.setStrafeSpeed(alt.m_y);
	}
	else
	{
		double angleTo = m_state->m_self.getAngleTo(target.m_x, target.m_y);
		Vec2d vect = Vec2d(forwardSpeed * std::cos(angleTo), forwardSpeed * std::sin(angleTo));

		Vec2d alt = getAlternateMoveVector(vect);
		move.setSpeed(alt.m_x);
		move.setStrafeSpeed(alt.m_y);
	}

	tryDisengage(move);
}

void MyStrategy::retreatTo(const Point2D& point, model::Move& move, DebugMessage& debugMessage)
{
	bool isToBonus = m_reasonableBonus != nullptr && m_reasonableBonus->m_point == point;

	if (m_state->m_isLowHP)
	{
		const model::Wizard& self  = m_state->m_self;
		const model::World& world = m_state->m_world;

		const model::Unit* enemy = nullptr;
		auto findNearestEnemy = [&self, &world, &enemy, this](const model::Unit& unit) 
		{
			if (unit.getFaction() == self.getFaction())  
				return;

			if (enemy == nullptr)
			{
				enemy = &unit;
			}
			else
			{
				double safeGap    = self.getDistanceTo(unit) - getSafeDistance(unit);
				double oldSafeGap = self.getDistanceTo(*enemy) - getSafeDistance(*enemy);

				if (safeGap < oldSafeGap)
					enemy = &unit;
			}
		};

		std::for_each(world.getWizards().begin(), world.getWizards().end(),     findNearestEnemy);
		std::for_each(world.getMinions().begin(), world.getMinions().end(),     findNearestEnemy);
		std::for_each(world.getBuildings().begin(), world.getBuildings().end(), findNearestEnemy);

		if (m_state->m_nextMinionRespawnTick - world.getTickIndex() < 100)
			std::for_each(m_state->m_enemySpawnPredictions.begin(), m_state->m_enemySpawnPredictions.end(), findNearestEnemy);

		bool isAlreadySafe = enemy == nullptr || enemy->getDistanceTo(self) > getSafeDistance(*enemy);
		if (isAlreadySafe && !isToBonus)
			return;  // don't retreat too far, except getting a bonus
	}

	if (!m_state->m_isHastened && m_state->isReadyForAction(ACTION_HASTE) && m_state->m_self.getLife() != m_state->m_self.getMaxLife())
	{
		move.setAction(ACTION_HASTE);
		move.setStatusTargetId(m_state->m_self.getId());
	}

	double forwardAngle = m_state->m_self.getAngleTo(point.m_x, point.m_y);
	double angle = (forwardAngle < 0 ? 2*PI - forwardAngle : forwardAngle) - PI;

	Point2D target = point;

	PathFinder::TilesPath tiles;
	const Map* map = m_maps->getMap(MapsManager::MT_WORLD_MAP);
	Map::PointPath smoothPath = getSmoothPathTo(point, map, tiles);

	debugMessage.visualizePath(tiles, map);

	//assert(!smoothPath.empty());
	if (!smoothPath.empty())
	{
		target = smoothPath.front();
	}

	double speed = m_state->m_game.getWizardForwardSpeed();
	if (m_state->m_isHastened)
		speed += speed * m_state->m_game.getHastenedMovementBonusFactor();

	double angleTo = m_state->m_self.getAngleTo(target.m_x, target.m_y);
	Vec2d vect = Vec2d(speed * std::cos(angleTo), speed * std::sin(angleTo));

	Vec2d alt = getAlternateMoveVector(vect);
	move.setSpeed(alt.m_x);
	move.setStrafeSpeed(alt.m_y);

	tryDisengage(move);
}

Map::PointPath MyStrategy::getSmoothPathTo(const Point2D& point, const Map* map, PathFinder::TilesPath& tiles)
{
	tiles = m_pathFinder->getPath(m_state->m_self, point, *map);
	Map::PointPath pointPath = map->tilesToPoints(tiles); pointPath.push_front(m_state->m_self);
	Map::PointPath smoothPath = map->smoothPath(m_state->m_world, pointPath);

	// remove 'self' from path after smoothing
	if (!smoothPath.empty())
		smoothPath.pop_front();

	return smoothPath;
}

double MyStrategy::getPathLength(const Map::PointPath& path) const
{
	double length = 0;
	Point2D previous = m_state->m_self;
	for (const Point2D& step : path)
	{
		length  += previous.getDistanceTo(step);
		previous = step;
	}

	return length;
}

void MyStrategy::tryDisengage(model::Move &move)
{
	if (m_state->isGotStuck())
	{
		// try freeing oneself
		move.setStrafeSpeed((rand() % 2 == 0 ? -1 : 1) * m_state->m_game.getWizardStrafeSpeed());
		if (rand() % 2 == 0)
			move.setSpeed(0.1);
	}
}

double MyStrategy::getSafeDistance(const model::Unit& enemy) const
{
	double safeDistance = m_state->m_game.getFactionBaseAttackRange();
	double selfRadius   = m_state->m_self.getRadius();

	// todo: take enemy's cooldown into account?
	// todo: take enemies count into account?

	// TODO - carefully test condition when enemy-attack-range < distance < exp-getting-radius
	if (getWizard(&enemy))
		safeDistance = getWizard(&enemy)->getCastRange() + selfRadius + m_state->m_game.getMagicMissileRadius() + 2 * m_state->m_game.getWizardForwardSpeed();

	const model::Minion* minion = getMinion(&enemy);
	if (minion)
	{
		// TODO - carefully test condition when minion-attack-range < distance < self-attack-range 
		const static double SAFE_GAP = minion->getType() == model::MINION_ORC_WOODCUTTER ? 2 * selfRadius : selfRadius;
		safeDistance = SAFE_GAP +
			(minion->getType() == model::MINION_ORC_WOODCUTTER
			? (m_state->m_game.getOrcWoodcutterAttackRange() + selfRadius)
			: (m_state->m_game.getFetishBlowdartAttackRange() + selfRadius));
	}

	const model::Building* building = getBuilding(&enemy);
	if (building)
	{
		safeDistance = selfRadius + building->getAttackRange();
	}

	const PredictedUnit* predicted = getPredicted(&enemy);
	if (predicted)
	{
		safeDistance = predicted->safeDistance();
	}

	return safeDistance;
}

double MyStrategy::getMaxDamage(const model::Unit* u) const
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

	auto predictedUnit = getPredicted(u);
	if (predictedUnit)
		return predictedUnit->predictedDamage();

	assert(false && "unknown unit type");
	return m_state->m_game.getMagicMissileDirectDamage();
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
Vec2d MyStrategy::getAlternateMoveVector(const Vec2d& suggestion)
{
	Timer timer(__FUNCTION__);

	const model::Wizard& self = m_state->m_self;
	const model::World& world = m_state->m_world;
	const Point2D selfPoint = self;

	const double LOOKUP_DISTANCE = m_state->m_game.getWizardRadius() * 3;

	std::vector<std::unique_ptr<model::Tree>> fakes;
	auto obstacles = filterPointers<const model::CircularUnit*>([&selfPoint, &LOOKUP_DISTANCE, &self](const model::CircularUnit& u)
		{ return self.getId() != u.getId() && selfPoint.getDistanceTo(u) < LOOKUP_DISTANCE; }
		, world.getWizards(), world.getMinions(), world.getBuildings(), world.getTrees());     // TODO - add non-traversable tiles too?

	// TODO: remove this hack
	// add tree to non-traversable cells to avoid problems with path-finder
	const Map* map = m_maps->getMap(MapsManager::MT_WORLD_MAP);
	const Point2D tilesGap = Point2D(self.getRadius() * 4, self.getRadius() * 4);
	Map::TileIndex topLeft = map->getTileIndex(selfPoint - tilesGap);
	Map::TileIndex bottomRight = map->getTileIndex(selfPoint + tilesGap);

	// todo: remove this in next sandbox iteration
	for (int y = topLeft.m_y; y <= bottomRight.m_y; ++y)
	{
		for (int x = topLeft.m_x; x <= bottomRight.m_x; ++x)
		{
			Map::TileIndex index = Map::TileIndex(x, y);
			if (!index.isValid(*map))
				continue;

			if (map->getTileState(index).isOccupied() && !map->getTileState(index).m_isVisible)
			{
				Point2D center = map->getTileCenter(index);
				fakes.emplace_back(std::make_unique<model::Tree>(-1, center.m_x, center.m_y, 0, 0, 0, FACTION_OTHER, double(map->getTileSize()), 999, 999, std::vector<model::Status>()));
				obstacles.push_back(fakes.back().get());
			}
		}
	}

	const Point2D worldTopleft     = Point2D(self.getRadius(), self.getRadius());
	const Point2D worldBottomRight = Point2D(m_state->m_world.getWidth() - self.getRadius(), m_state->m_world.getHeight() - self.getRadius());

	auto isVectorValid = [&obstacles, &self, &worldTopleft, &worldBottomRight](const Vec2d& v)
	{
		Vec2d absDirection = Vec2d(v).rotate(self.getAngle());
		Vec2d newPosition  = absDirection.normalize() * self.getRadius() + absDirection;
		Vec2d worldVector  = newPosition + Vec2d::fromPoint<Point2D>(self);

		if (worldVector.m_x < worldTopleft.m_x || worldVector.m_x > worldBottomRight.m_x || worldVector.m_y < worldTopleft.m_y || worldVector.m_y > worldBottomRight.m_y)
		{
			return false;
		}

		return obstacles.end() == std::find_if(obstacles.begin(), obstacles.end(), [&worldVector, &self](const model::CircularUnit* obstacle)
		{
			return Map::isSectionIntersects(self, worldVector.toPoint<Point2D>(), *obstacle, obstacle->getRadius() + self.getRadius());
		});
	};

	if (isVectorValid(suggestion))
		return suggestion;

	static const int    ALTERNATIVES_COUNT = 180;
	static const double MAX_DEVIATION = PI / 2 + PI / 8;
	static const double STEP = 2 * MAX_DEVIATION / ALTERNATIVES_COUNT;

	std::vector<Vec2d> alternatives;
	alternatives.reserve(ALTERNATIVES_COUNT);

	for (double angle = -MAX_DEVIATION; angle < +MAX_DEVIATION; angle += STEP)
	{
		Vec2d rotated = Vec2d(suggestion).rotate(angle);
		if (isVectorValid(rotated))
		{
			alternatives.push_back(rotated);
		}
	}

	std::sort(alternatives.begin(), alternatives.end(), 
		[&suggestion](const Vec2d& a, const Vec2d& b) { return std::abs(Vec2d::angleBetween(suggestion, a)) < std::abs(Vec2d::angleBetween(suggestion, b)); });

	if (alternatives.empty())
	{
		return suggestion;
	}
	else
	{
		return alternatives.front();
	}
}

void MyStrategy::learnSkill(model::Move& move)
{
	const int level  = m_state->m_self.getLevel();
	const int skillsTotal = std::extent<decltype(SKILLS_TO_LEARN)>::value;

	int nextSkill = level - 1;
	if (nextSkill > 0 && int(m_state->m_self.getSkills().size()) < nextSkill)
	{
		// for case when got 2 levelups at onCe
		int previousSkill = std::max(0, nextSkill - 1);
		move.setSkillToLearn(SKILLS_TO_LEARN[previousSkill]);
	}
	else if (nextSkill >= 0 && nextSkill < skillsTotal)
	{
		move.setSkillToLearn(SKILLS_TO_LEARN[nextSkill]);
	}
}

MyStrategy::MyStrategy()
	: m_lastStrafeChangeTick(0)
	, m_lastStrafe(0.0)
	, m_visualizer(make_unique<DebugVisualizer>())
	, m_guardPoint(nullptr)
	, m_spawnPoint(nullptr)
	, m_currentWaypointIndex(1) // [0] waypoint is for retreating only
	, m_oldState()
	, m_reasonableBonus(nullptr)
{
}

State::State(const MyStrategy* strategy, const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move, const StorableState& oldState)
	: m_self(self), m_world(world), m_game(game), m_move(move)
	, m_isUnderMissile(false)
	, m_isLowHP(false)
	, m_cooldownTicks()
	, m_isGoingToBonus(false)
	, m_storedState(oldState)
	, m_bonuses(oldState.m_bonuses)
	, m_strategy(strategy)
	, m_estimatedHP(self.getLife())
	, m_nextMinionRespawnTick(0)
	, m_isHastened(false)
{
	const auto& wizards = m_world.getWizards();
	const Point2D selfPoint = self;

	updateProjectiles(); // is under missile calculation
	updateBonuses();
	updatePredictions();
	updateSkillsAndActions();

	auto statuses = m_self.getStatuses();
	m_isHastened = statuses.end() != std::find_if(statuses.begin(), statuses.end(), [](const model::Status& s) { return s.getType() == model::STATUS_HASTENED; });

	// todo: take into account all missiles
	m_estimatedHP -= 0; // TODO - not yet ready     //m_isUnderMissile ? game.getMagicMissileDirectDamage() : 0;
	m_isLowHP      = m_estimatedHP < (self.getMaxLife() * State::LOW_HP_FACTOR);
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void State::updateSkillsAndActions()
{
	int learnedCount = std::min<int>(m_self.getLevel(), std::extent<decltype(MyStrategy::SKILLS_TO_LEARN)>::value);
	std::copy_n(std::begin(MyStrategy::SKILLS_TO_LEARN), learnedCount, std::back_inserter(m_learnedSkills));

	// set cooldown ticks, taking into account both specific and common cooldown
	std::copy(m_self.getRemainingCooldownTicksByAction().begin(), m_self.getRemainingCooldownTicksByAction().end(), m_cooldownTicks.begin());
	std::transform(m_cooldownTicks.begin(), m_cooldownTicks.end(), m_cooldownTicks.begin(), 
		[this](int cooldown) { return std::max(cooldown, m_self.getRemainingActionCooldownTicks()); });

	struct ActionInfo
	{
		model::SkillType  skill;
		model::ActionType action;
		int               manaCost;
	};

	const ActionInfo actionSkills[] =
	{
		{model::SKILL_FROST_BOLT, model::ACTION_FROST_BOLT, m_game.getFrostBoltManacost()},
		{model::SKILL_FIREBALL,   model::ACTION_FIREBALL  , m_game.getFireballManacost()},
		{model::SKILL_HASTE,      model::ACTION_HASTE     , m_game.getHasteManacost()},
		{model::SKILL_SHIELD,     model::ACTION_SHIELD    , m_game.getShieldManacost()},
	};

	// set cooldown ticks to COOLDOWN_INF if action is not available at all
	for (const ActionInfo& actionInfo : actionSkills)
	{
		assert(actionInfo.action < model::_ACTION_COUNT_ && "invalid action");
		int& cooldown = m_cooldownTicks[actionInfo.action];

		if (!has(m_learnedSkills, actionInfo.skill))
			cooldown = COOLDOWN_INF;

		if (m_self.getMana() < actionInfo.manaCost)
			cooldown = std::max<int>(cooldown, (actionInfo.manaCost - m_self.getMana()) / m_game.getWizardBaseManaRegeneration() + 1);  // TODO - take levelup-ed mana regeneration speed into account 
	}
}


void State::updateDispositionAround()
{
	Point2D selfPoint = m_self;

	auto isUnitNear = [this, &selfPoint](const model::LivingUnit& unit)
	{
		double distance = selfPoint.getDistanceTo(unit);
		if (unit.getFaction() == m_self.getFaction())
		{
			// teammates has penalty if they are closer to base than me
			bool isAtFrontOfMe = std::abs(m_self.getAngleTo(unit)) <= (PI / 2);

			const double BACK_TEAMMATE_PENALTY = 3;

			if (!isAtFrontOfMe)
				distance *= BACK_TEAMMATE_PENALTY;
		}

		return distance < m_strategy->getSafeDistance(unit);
	};

	auto applyDisposition = [this, isUnitNear](const model::LivingUnit& unit)
	{
		if (!isUnitNear(unit))
			return;

		auto wizard = m_strategy->getWizard(&unit);
		auto minion = m_strategy->getMinion(&unit);
		auto builing = m_strategy->getBuilding(&unit);
		bool isTeammate = m_self.getFaction() == unit.getFaction();

		if (wizard != nullptr)
			(isTeammate ? m_disposionAround.teammateWizards : m_disposionAround.enemyWizards) += 1;
		else if (minion != nullptr)
			(isTeammate ? m_disposionAround.teammateMinions : m_disposionAround.enemyMinions) += 1;
		else if (builing != nullptr)
			(isTeammate ? m_disposionAround.teammateBuildings : m_disposionAround.enemyBuildings) += 1;

		if (wizard != nullptr || minion != nullptr)
			(isTeammate ? m_disposionAround.movebleTeammatesHP : m_disposionAround.movableEnemyHP) += unit.getLife();
	};

	std::for_each(m_world.getWizards().begin(), m_world.getWizards().end(), applyDisposition);
	std::for_each(m_world.getBuildings().begin(), m_world.getBuildings().end(), applyDisposition);
	std::for_each(m_world.getMinions().begin(), m_world.getMinions().end(), applyDisposition);
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void State::updateProjectiles()
{
	const auto& projectiles = m_world.getProjectiles();
	const Point2D selfPoint = m_self;

	auto itProjectile = std::find_if(std::begin(projectiles), std::end(projectiles), [&selfPoint, this](const model::Projectile& projectile)
	{
		Vec2d projectileSpeed = Vec2d(projectile.getSpeedX(), projectile.getSpeedY());
		LineEquation firingLine = LineEquation::fromDirectionVector(projectile, projectileSpeed);

		if (firingLine.isContains(selfPoint))
		{
			return true;   // hit just into center
		}

		double collitionRadius = m_self.getRadius() + projectile.getRadius();

		// http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
		// compute the direction vector D from A to B
		Vec2d D = projectileSpeed;
		D.normalize();

		// TODO - it looks like this code doesn't care on projectile flying direction and distance
		// however, this may be somehow used to prevent staying on firing line like: me -> teammate's back -> enemy

		// Now the fire line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
		// where D is destination vector, A is a starting point, C is a circle center

		// compute the value t of the closest point to the circle center (Cx, Cy)
		Point2D projectilePoint = projectile;
		double t = D.m_x*(selfPoint.m_x - projectilePoint.m_x) + D.m_y*(selfPoint.m_y - projectilePoint.m_y);

		// This is the projection of C on the line from A to B.
		// compute the coordinates of the point E on line and closest to C
		Point2D E = Point2D(t*D.m_x + projectilePoint.m_x, t*D.m_y + projectilePoint.m_y);

		// compute the euclidean distance from E to C
		double LEC = E.getDistanceTo(selfPoint);

		// test if the line intersects the circle
		if (LEC <= collitionRadius)
		{
			// compute distance from t to circle intersection point
			double dt = sqrt(collitionRadius*collitionRadius - LEC * LEC);

			// compute first intersection point
			Point2D F = Point2D((t - dt)*D.m_x + projectilePoint.m_x, (t - dt)*D.m_y + projectilePoint.m_y);
			double test = F.getDistanceTo(selfPoint);

			// compute second intersection point
			// Gx = (t + dt)*Dx + Ax
			// Gy = (t + dt)*Dy + Ay
		}

		return LEC <= collitionRadius;
	});

	if (itProjectile != std::end(projectiles))
	{
		// fire in the hall!
		m_isUnderMissile = true;
	}
}

//////////////////////////////////////////////////////////////////////////
//
// Method:
//
// Desc:
//
// Params:
//
// Return:
//
///////////////////////////////////////////////////////////////////////////
void State::updateBonuses()
{
	auto teammates = filterPointers<const model::Unit*>([this](const model::Unit& u) {return u.getFaction() == m_self.getFaction(); },
		m_world.getBuildings(), m_world.getWizards(), m_world.getMinions());

	const auto& bonusUnits = m_world.getBonuses();
	int thisTick = m_world.getTickIndex();

	for (BonusSpawn& spawn : m_bonuses)
	{
		const double NEAR_DISTANCE = std::min(m_game.getWizardRadius() * 4, spawn.m_point.getDistanceTo(m_self));
		spawn.m_teamateCompetitors = std::count_if(teammates.begin(), teammates.end(), [&spawn, this, NEAR_DISTANCE](const model::Unit* u)
		{ 
			auto wizard = m_strategy->getWizard(u); 
			return wizard 
				&& wizard->getId() != m_self.getId() 
				&& spawn.m_point.getDistanceTo(*wizard) < NEAR_DISTANCE;
		});

// maybe, bad idea
// 		spawn.m_wizardsHp = BonusSpawn::WizardsHealth();
// 		for (const model::Wizard& wizard : m_world.getWizards())
// 		{
// 			if (wizard.getFaction() == m_self.getFaction())
// 				spawn.m_wizardsHp.teammates += wizard.getLife();
// 			else 
// 				spawn.m_wizardsHp.enemies += wizard.getLife();
// 		}

		spawn.m_smoothPathCache.clear();
		spawn.m_tilesPathCache.clear();
	}

	if (lastBonusSpawnTick() == 0)
	{
		for (BonusSpawn& bonusSpawn : m_bonuses)
		{
			bonusSpawn.m_state         = BonusSpawn::NO_BONUS;
			bonusSpawn.m_lastCheckTick = thisTick;
		}
		return;
	}

	for (BonusSpawn& bonusSpawn : m_bonuses)
	{
		const Point2D& spawnPoint = bonusSpawn.m_point;

		if (bonusUnits.end() != std::find_if(bonusUnits.begin(), bonusUnits.end(), [&spawnPoint](const model::Bonus& b) {return spawnPoint == b; }))
		{
			bonusSpawn.m_state = BonusSpawn::HAS_BONUS;
		}
		else
		{
			bool isStateUnknown = bonusSpawn.m_lastCheckTick < lastBonusSpawnTick() || bonusSpawn.m_state == BonusSpawn::UNKNOWN;
			if (isStateUnknown || bonusSpawn.m_state == BonusSpawn::HAS_BONUS)
			{
				// if bonus spawn is visible but bonus is not, then there is no bonus, otherwise - bonus state is unknown
				bool isVisible = teammates.end() != std::find_if(teammates.begin(), teammates.end(),
					[&spawnPoint](const model::Unit* unit) { return MyStrategy::isUnitSeeing(unit, spawnPoint); });

				bonusSpawn.m_state = isVisible ? BonusSpawn::NO_BONUS : BonusSpawn::UNKNOWN;
			}
		}

		bonusSpawn.m_lastCheckTick = thisTick;
	}
}

void State::updatePredictions()
{
	const double minionSpawnDxDy   = 800;
	const double halfTileDxDy      = 200;
	const double minionSpawnRadius = 250;
	const double diagonal          = 1.3; // close to sqrt(2)
	
	const double predictedDamage = 2 * (m_game.getDartDirectDamage() + 3 * m_game.getOrcWoodcutterDamage());
	const double safeDistance = m_game.getOrcWoodcutterAttackRange() + minionSpawnRadius + m_self.getRadius() + 2 * m_self.getRadius()/*safe gap*/;  // actually, there is also a dart, but it's not too dangerous
	
	model::Faction selfFaction = m_self.getFaction();
	model::Faction enemyFaction = selfFaction == model::FACTION_ACADEMY ? model::FACTION_RENEGADES : model::FACTION_ACADEMY;

	m_nextMinionRespawnTick = m_game.getFactionMinionAppearanceIntervalTicks() * (m_world.getTickIndex() / m_game.getFactionMinionAppearanceIntervalTicks() + 1);

	m_enemySpawnPredictions.reserve(3);

	const Point2D enemyCorner{ m_world.getWidth(), 0 };
	m_enemySpawnPredictions.emplace_back(PredictedUnit(enemyCorner + Point2D(-minionSpawnDxDy - halfTileDxDy, halfTileDxDy), minionSpawnRadius, enemyFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));
	m_enemySpawnPredictions.emplace_back(PredictedUnit(enemyCorner + Point2D(-halfTileDxDy, minionSpawnDxDy + halfTileDxDy), minionSpawnRadius, enemyFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));
	m_enemySpawnPredictions.emplace_back(PredictedUnit(enemyCorner + Point2D(-minionSpawnDxDy / diagonal - halfTileDxDy, minionSpawnDxDy / diagonal + halfTileDxDy), minionSpawnRadius, enemyFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));

// 	const Point2D teamCorner{ 0, m_world.getHeight() };
// 	m_predictions.emplace_back(PredictedUnit(teamCorner + Point2D(minionSpawnDxDy + halfTileDxDy, -halfTileDxDy), minionSpawnRadius, selfFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));
// 	m_predictions.emplace_back(PredictedUnit(teamCorner + Point2D(halfTileDxDy, -minionSpawnDxDy - halfTileDxDy), minionSpawnRadius, selfFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));
// 	m_predictions.emplace_back(PredictedUnit(teamCorner + Point2D(minionSpawnDxDy / diagonal + halfTileDxDy, -minionSpawnDxDy / diagonal - halfTileDxDy), minionSpawnRadius, selfFaction, m_nextMinionRespawnTick, predictedDamage, safeDistance));
}

