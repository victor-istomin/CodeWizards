#include "NavigationManager.h"
#include "MyStrategy.h"
#include "DebugVisualizer.h"
#include "MapsManager.h"
#include "WorldMap.h"

#include "model/ActionType.h"

NavigationManager::NavigationManager(const MyStrategy& strategy, const State& state, const Waypoints& waypoints, const Point2D& guardPoint, PathFinder& pathFinder,
                                     DebugMessage& debugMessage)
	: m_strategy(strategy)
	, m_state(state)
	, m_waypoints(waypoints)
	, m_guardPoint(guardPoint)
	, m_pathFinder(pathFinder)
	, m_maps(strategy.getMaps())
	, m_debugMessage(debugMessage)
	, m_pursuitList()
	, m_bonus(nullptr)
	, m_isRetreating(false)
	, m_forcePreserveAngle(false)
	, m_isInCombat(false)
{

}

void NavigationManager::makeMove(model::Move& move)
{
	using StageFunc = decltype(&NavigationManager::stagePursuit);
	StageFunc stages[] = 
	{ 
		&NavigationManager::stagePursuit,
		&NavigationManager::stageBonus,
		&NavigationManager::stageRetreat,    // 'stageBonus' placed before because it's responsible of retreating to bonus
		&NavigationManager::stageInCombat,
		&NavigationManager::stagePushLine,
	};

	for (StageFunc stage : stages)
		if ((this->*stage)(move))
			break;
}

Map::PointPath NavigationManager::getSmoothPathTo(const Point2D& point, PathFinder::TilesPath& tiles)
{
	const Map* map = m_maps.getMap(MapsManager::MT_WORLD_MAP);
	const model::Wizard& self = m_state.m_self;

	tiles = m_pathFinder.getPath(self, point, *map);
	Map::PointPath pointPath = map->tilesToPoints(tiles); pointPath.push_front(self);
	Map::PointPath smoothPath = map->smoothPath(m_state.m_world, pointPath);

	// remove 'self' from path after smoothing
	if (!smoothPath.empty())
		smoothPath.pop_front();

	return smoothPath;
}

bool NavigationManager::stagePursuit(model::Move& move)
{
	if (m_isRetreating || m_pursuitList.empty())
		return false;

	const auto& game = m_state.m_game;
	const auto& self = m_state.m_self;

	bool isMoveChanged = false;

	for (const model::LivingUnit* target : m_pursuitList)
	{
		double enemyAngle = target->getAngleTo(self);
		if (std::abs(enemyAngle) > game.getStaffSector())
		{
			const double shootingDistance = self.getCastRange();   // TODO - accurate calculation

			double ticksToTurn = 1 + (std::abs(enemyAngle) - game.getStaffSector()) / game.getWizardMaxTurnAngle();
			double desiredDistance = shootingDistance - ticksToTurn * game.getWizardStrafeSpeed();

			if (self.getDistanceTo(*target) > desiredDistance)
			{
				if (goTo(*target, move))
				{
					isMoveChanged = true;
					break;
				}
			}
		}
	}

	return isMoveChanged;
}

bool NavigationManager::stageBonus(model::Move& move)
{
	if (m_bonus == nullptr)
		return false;

	const model::Wizard& self = m_state.m_self;
	const model::World& world = m_state.m_world;

	bool canGet = !m_state.m_isLowHP;
	if (m_state.m_isLowHP)
	{
		const Point2D& nextPathPoint = m_bonus->m_smoothPathCache.empty() ? m_bonus->m_point : m_bonus->m_smoothPathCache.front();

		auto enemies = filterPointers<const model::Unit*>([this, &self](const model::Unit& u) {return MyStrategy::isEnemy(u, self); }, 
			world.getBuildings(), world.getWizards(), world.getMinions());

		bool noEnemiesThatWay = enemies.end() == std::find_if(enemies.begin(), enemies.end(),
			[&nextPathPoint, &self](const model::Unit* enemy)
		{
			return std::abs(self.getAngleTo(*enemy) - self.getAngleTo(nextPathPoint.m_x, nextPathPoint.m_y)) > (PI / 2)
				|| self.getDistanceTo(*enemy) > 2 * self.getVisionRange();
		});

		canGet = noEnemiesThatWay;
	}

	bool isMoveApplied = false;
	if (canGet)
	{
		isMoveApplied = goTo(m_bonus->m_point, move);
	}

	return isMoveApplied;
}

bool NavigationManager::stageRetreat(model::Move& move)
{
	if (!m_isRetreating)
		return false;

	// TODO - port "don't retreat too far" feature from MyStrategy::retreatTo()
	const model::Wizard& self  = m_state.m_self;
	const model::World&  world = m_state.m_world;

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
			double safeGap    = self.getDistanceTo(unit) -   m_strategy.getSafeDistance(unit);
			double oldSafeGap = self.getDistanceTo(*enemy) - m_strategy.getSafeDistance(*enemy);

			if (safeGap < oldSafeGap)
				enemy = &unit;
		}
	};

	std::for_each(world.getWizards().begin(), world.getWizards().end(),     findNearestEnemy);
	std::for_each(world.getMinions().begin(), world.getMinions().end(),     findNearestEnemy);
	std::for_each(world.getBuildings().begin(), world.getBuildings().end(), findNearestEnemy);

	if (m_state.m_nextMinionRespawnTick - world.getTickIndex() < 100)
		std::for_each(m_state.m_enemySpawnPredictions.begin(),m_state.m_enemySpawnPredictions.end(), findNearestEnemy);

	bool isAlreadySafe = enemy == nullptr || enemy->getDistanceTo(self) > m_strategy.getSafeDistance(*enemy);
	if (isAlreadySafe)
		return true;  // don't retreat too far, except getting a bonus

	if (!m_state.m_isHastened && m_state.isReadyForAction(model::ACTION_HASTE) && self.getLife() != self.getMaxLife())
	{
		move.setAction(model::ACTION_HASTE);
		move.setStatusTargetId(getTeammateIdToHelp());
	}

	Point2D previousWaypoint = getPreviousWaypoint();
	
	bool aimForward = !m_state.m_dangerousEnemies.empty();
	return goTo(previousWaypoint, move, aimForward);
}

bool NavigationManager::stagePushLine(model::Move& move)
{
	if (m_isInCombat)
	{
		// this code doesn't cover moves in combat
		return false;
	}

	// don't push too early
	const int prepareTicks = m_state.m_game.getFactionMinionAppearanceIntervalTicks()
		/* debug code*/
		+ 0; 
		/* debug code end, uncomment below*/
	//+ (m_guardPoint.getDistanceTo(MyStrategy::MID_GUARD_POINT) < Point2D::k_epsilon ? 200 : 100);

	Point2D currentWaypoint = m_waypoints[GetCurrentWaypointIndex()];
	bool    isPushTime      = m_state.m_world.getTickIndex() > prepareTicks;
	if (!isPushTime && currentWaypoint == m_guardPoint)
	{
		bool isMoveApplied = false;

		const double STAYING_DISTANCE = m_state.m_game.getWizardRadius();
		if (currentWaypoint.getDistanceTo(m_state.m_self) < STAYING_DISTANCE)
		{
			// turn, but not move
			Point2D nextWaypoint = getNextWaypoint();
			double angle = m_state.m_self.getAngleTo(nextWaypoint.m_x, nextWaypoint.m_y);
			move.setTurn(angle);
			isMoveApplied = true;
		}
		else
		{
			// go to guard point
			isMoveApplied = goTo(currentWaypoint, move);
		}

		return isMoveApplied;
	}

	// time to push

	return goTo(getNextWaypoint(), move);
}

bool NavigationManager::stageInCombat(model::Move& move)
{
	return false; // not yet implemented
}

bool NavigationManager::goTo(const Point2D& point, model::Move& move, bool preserveAngle /*= false*/)
{
	// TODO - add 2 modes: preserve unit angle or rotate self
	Timer timer(__FUNCTION__);

	Point2D target = point;

	PathFinder::TilesPath path;
	const Map* map = m_maps.getMap(MapsManager::MT_WORLD_MAP);
	const model::Wizard& self = m_state.m_self;

	double distanceTo = point.getDistanceTo(self);
	if (distanceTo < 1)
		return true; // already here

	Map::PointPath smoothPath;

	if (m_bonus != nullptr && m_bonus->m_point == point)
	{
		smoothPath = m_bonus->m_smoothPathCache;
		path = m_bonus->m_tilesPathCache;

		if (!m_state.m_isHastened && m_state.isReadyForAction(model::ACTION_HASTE))
		{
			// TODO - don't burl all mana for haste?
			move.setAction(model::ACTION_HASTE);
			move.setStatusTargetId(getTeammateIdToHelp());
		}
	}
	else
	{
		smoothPath = getSmoothPathTo(target, path);
	}

	m_debugMessage.setNextWaypoint(point);
	m_debugMessage.visualizePath(path, map);

	// this may occur when navigating to the enemy in occupied cell
	//assert(!smoothPath.empty());
	if (!smoothPath.empty())
	{
		target = smoothPath.front();
	}

	double angle = self.getAngleTo(target.m_x, target.m_y);

	if (!preserveAngle && !m_forcePreserveAngle)
		move.setTurn(angle);

	double cos = std::cos(angle);
	double sin = std::sin(angle);
	bool isForward = cos >= 0.0;

	const model::Game& game = m_state.m_game;
	Limits speedLimit = getMaxSpeed(&self);
	double hypotSpeed = speedLimit.hypot(isForward);

	if (forwardSpeed > distanceTo && !m_isRetreating)
		forwardSpeed = distanceTo;


	Vec2d moveVector = Vec2d(speedLimit.forward * cos, forwardSpeed * sin);
	moveVector.truncate(speedLimit.hypot(isForward));

	moveVector = getAlternateMoveVector(moveVector);
	move.setSpeed(moveVector.m_x);
	move.setStrafeSpeed(moveVector.m_y);

	if (m_state.isGotStuck())
	{
		// try freeing oneself
		move.setStrafeSpeed((rand() % 2 == 0 ? -1 : 1) * game.getWizardStrafeSpeed());
		if (rand() % 2 == 0)
			move.setSpeed(0.1);
	}

	return isPathAcceptable(moveVector, smoothPath);
}

bool NavigationManager::isPathAcceptable(const Vec2d& moveVector, const Map::PointPath& smoothPath)
{
	return true; // TODO: don't step into missile, etc.
}

Vec2d NavigationManager::getAlternateMoveVector(const Vec2d& suggestion)
{
	Timer timer(__FUNCTION__);

	const model::Wizard& self = m_state.m_self;
	const model::World& world = m_state.m_world;
	const Point2D selfPoint = self;

	const double LOOKUP_DISTANCE = m_state.m_game.getWizardRadius() * 3;

	std::vector<std::unique_ptr<model::Tree>> fakes;
	auto obstacles = filterPointers<const model::CircularUnit*>(
		[&selfPoint, &LOOKUP_DISTANCE, &self](const model::CircularUnit& u) { return self.getId() != u.getId() && selfPoint.getDistanceTo(u) < LOOKUP_DISTANCE; }, 
		world.getWizards(), world.getMinions(), world.getBuildings(), world.getTrees());

	// TODO: remove this hack
	// add tree to non-traversable cells to avoid problems with path-finder
	const Map* map = m_maps.getMap(MapsManager::MT_WORLD_MAP);
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
				fakes.emplace_back(std::make_unique<model::Tree>(-1, center.m_x, center.m_y, 0, 0, 0, model::FACTION_OTHER, double(map->getTileSize()), 999, 999, std::vector<model::Status>()));
				obstacles.push_back(fakes.back().get());
			}
		}
	}

	const Point2D worldTopleft = Point2D(self.getRadius(), self.getRadius());
	const Point2D worldBottomRight = Point2D(m_state.m_world.getWidth() - self.getRadius(), m_state.m_world.getHeight() - self.getRadius());

	auto isVectorValid = [&obstacles, &self, &worldTopleft, &worldBottomRight](const Vec2d& v)
	{
		Vec2d absDirection = Vec2d(v).rotate(self.getAngle());
		Vec2d newPosition = absDirection.normalize() * self.getRadius() + absDirection;
		Vec2d worldVector = newPosition + Vec2d::fromPoint<Point2D>(self);

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

Point2D NavigationManager::getNextWaypoint()
{
	size_t lastWaypointIndex = m_waypoints.empty() ? 0 : m_waypoints.size() - 1;
	size_t currentIndex = GetCurrentWaypointIndex();

	const auto& self = m_state.m_self;

	if (currentIndex < lastWaypointIndex)
	{
		const Point2D& current = m_waypoints[currentIndex];
		const Point2D& next = m_waypoints[currentIndex + 1];

		if (current.getDistanceTo(self) > MyStrategy::WAYPOINT_RADIUS && std::abs(self.getAngleTo(current.m_x, current.m_y) - self.getAngleTo(next.m_x, next.m_y)) <= PI / 2)
			return current;
		else
			return next;
	}

	return m_waypoints[lastWaypointIndex];
}

size_t NavigationManager::GetCurrentWaypointIndex()
{
	size_t lastWaypointIndex = m_waypoints.empty() ? 0 : m_waypoints.size() - 1;
	const auto& self = m_state.m_self;

	size_t currentIndex = lastWaypointIndex;
	double currentDistance = m_waypoints[currentIndex].getDistanceTo(self);

	for (size_t i = 0; i < lastWaypointIndex; ++i)
	{
		double distanceTo = m_waypoints[i].getDistanceTo(self);
		if (distanceTo < currentDistance)
		{
			currentIndex = i;
			currentDistance = distanceTo;
		}
	}

	return currentIndex;
}

Point2D NavigationManager::getPreviousWaypoint()
{
	size_t currentIndex = GetCurrentWaypointIndex();
	const auto& self = m_state.m_self;

	if (currentIndex > 0)
	{
		const Point2D& current  = m_waypoints[currentIndex];
		const Point2D& previous = m_waypoints[currentIndex - 1];

		if (std::abs(self.getAngleTo(current.m_x, current.m_y) - self.getAngleTo(previous.m_x, previous.m_y)) <= PI / 2)
			return current;
		else
			return previous;
	}

	return m_waypoints[0];
}

long long NavigationManager::getTeammateIdToHelp() const
{
	const auto& self = m_state.m_self;

	auto teammatesAround = filterPointers<const model::Wizard*>(
		[&self](const model::Wizard& w) { return w.getFaction() == self.getFaction() && self.getDistanceTo(w) < self.getCastRange(); },
		m_state.m_world.getWizards());

	std::sort(teammatesAround.begin(), teammatesAround.end(), [](const auto* a, const auto* b) {return a->getLife() < b->getLife(); });

	return teammatesAround.empty() ? self.getId() : teammatesAround.front()->getId();
}

NavigationManager::Limits NavigationManager::getMaxSpeed(const model::Wizard* wizard)
{
	const model::Game& game   = m_state.m_game;

	Limits limits;
	limits.forward  = game.getWizardForwardSpeed();
	limits.backward = -game.getWizardBackwardSpeed();
	limits.strafe   = game.getWizardStrafeSpeed();

	int learnedSkills = std::count_if(m_state.m_learnedSkills.begin(), m_state.m_learnedSkills.end(),
		[](const model::SkillType& s) { return s == model::SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_1 || s == model::SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_2
		|| s == model::SKILL_MOVEMENT_BONUS_FACTOR_AURA_1 || s == s == model::SKILL_MOVEMENT_BONUS_FACTOR_AURA_2; });

	auto nearTeammates = filterPointers<const model::Wizard*>(
		[this, wizard, &game](const model::Wizard& w) {return w.getFaction() == wizard->getFaction() && wizard->getDistanceTo(w) < game.getAuraSkillRange(); },
		m_state.m_world.getWizards());

	bool hasAura1 = false, hasAura2 = false;
	for (const auto* teammate : nearTeammates)
	{
		const std::vector<model::SkillType>& auras = teammate->getSkills;
		hasAura1 = hasAura1 || auras.end() != std::find(auras.begin(), auras.end(), model::SKILL_MOVEMENT_BONUS_FACTOR_AURA_1);
		hasAura2 = hasAura2 || auras.end() != std::find(auras.begin(), auras.end(), model::SKILL_MOVEMENT_BONUS_FACTOR_AURA_2);
	}

	int  aurasCount = (hasAura1 ? 1 : 0) + (hasAura2 ? 1 : 0);
	bool isHastened = m_state.m_isHastened;

	double factor = 1 + learnedSkills * game.getMovementBonusFactorPerSkillLevel()
	                  + aurasCount * game.getMovementBonusFactorPerSkillLevel()
	                  + (isHastened ? game.getHastenedMovementBonusFactor() : 0);

	limits.increaseBy(factor);
	return limits;
}