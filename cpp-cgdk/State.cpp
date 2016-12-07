#include "State.h"
#include "model/World.h"
#include "MyStrategy.h"

#include <iostream>

State::State(const MyStrategy* strategy, const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move, const StorableState& oldState)
	: m_self(self), m_world(world), m_game(game), m_move(move)
	, m_isLowHP(false)
	, m_cooldownTicks()
	, m_isGoingToBonus(false)
	, m_storedState(oldState)
	, m_bonuses(oldState.m_bonuses)
	, m_strategy(strategy)
	, m_estimatedHP(self.getLife())
	, m_nextMinionRespawnTick(0)
	, m_isHastened(false)
	, m_projectileInfos(oldState.m_projectiles)
	, m_dangerousProjectiles()
	, m_dangerousEnemies()
{
	auto allUnits = filterPointers<const model::Unit*>([](...) {return true; }, 
		m_world.getWizards(), m_world.getMinions(), m_world.getBuildings(), m_world.getBonuses(), m_world.getProjectiles(), m_world.getTrees());

	for (const auto* unit : allUnits)
	{
		auto& sameType = m_units[std::type_index(typeid(*unit))];
		sameType[unit->getId()] = unit;
	}

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
	m_isLowHP = m_estimatedHP < (self.getMaxLife() * State::LOW_HP_FACTOR);
}

void State::updateSkillsAndActions()
{
	int learnedCount = std::min<int>(m_self.getLevel(), MyStrategy::SKILLS_TO_LEARN.size());
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
		{ model::SKILL_FROST_BOLT, model::ACTION_FROST_BOLT, m_game.getFrostBoltManacost() },
		{ model::SKILL_FIREBALL,   model::ACTION_FIREBALL  , m_game.getFireballManacost() },
		{ model::SKILL_HASTE,      model::ACTION_HASTE     , m_game.getHasteManacost() },
		{ model::SKILL_SHIELD,     model::ACTION_SHIELD    , m_game.getShieldManacost() },
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

void State::updateProjectiles()
{
	const auto& projectiles = m_world.getProjectiles();
	const Point2D selfPoint = m_self;

	auto disappearedIt = std::remove_if(m_projectileInfos.begin(), m_projectileInfos.end(), [&projectiles](const StorableState::ProjectileInfo& info)
	{
		// not exists or gone out of visible range
		return projectiles.end() == std::find(projectiles.begin(), projectiles.end(), info);
	});

	if (disappearedIt != m_projectileInfos.end())
		m_projectileInfos.erase(disappearedIt, m_projectileInfos.end());

	for (const model::Projectile& newProjectile : projectiles)
	{
		auto foundIt = std::find(m_projectileInfos.begin(), m_projectileInfos.end(), newProjectile);
		if (foundIt == m_projectileInfos.end() && newProjectile.getType() != model::PROJECTILE_DART)
		{
			m_projectileInfos.emplace_back(newProjectile, m_world.getTickIndex());
			const model::Wizard* owner = getUnit<model::Wizard>(newProjectile.getOwnerUnitId());
			if (owner != nullptr)
				m_projectileInfos.back().m_detectionPoint = *owner;
		}
	}

	const std::vector<const model::LivingUnit*> livingUnits = filterPointers<const model::LivingUnit*>([](const model::LivingUnit&) {return true; },
		m_world.getWizards(), m_world.getMinions(), m_world.getBuildings(), m_world.getTrees());

	std::vector<const model::LivingUnit*> unitsMightHit;
	unitsMightHit.reserve(16);

	for (StorableState::ProjectileInfo& projectileInfo : m_projectileInfos)
	{
		Vec2d direction = projectileInfo.m_speed;
		direction.normalize();

		const model::Projectile* projectile = getUnit<model::Projectile>(projectileInfo.m_id);

		// TODO - more accurate flight distance prediction?
		// e.g. we could track max projectile distance for each unit
		double  flightDistance = m_game.getWizardCastRange();
		Point2D flightFinish = projectileInfo.m_detectionPoint + (direction * flightDistance).toPoint<Point2D>();

		auto mightHit = [&projectile, flightFinish, this](const model::LivingUnit* unit)
		{
			return Map::isSectionIntersects(*projectile, flightFinish, *unit, unit->getRadius() + projectile->getRadius())
				&& (MyStrategy::getTree(unit) != nullptr || unit->getFaction() != projectile->getFaction());
		};

		unitsMightHit.clear();
		std::copy_if(livingUnits.begin(), livingUnits.end(), std::back_inserter(unitsMightHit), mightHit);
		std::sort(unitsMightHit.begin(), unitsMightHit.end(), [&projectile](const auto* a, const auto* b) { return projectile->getDistanceTo(*a) < projectile->getDistanceTo(*b); });

		auto treeIt = std::find_if(unitsMightHit.begin(), unitsMightHit.end(), [](const auto* unit) { return MyStrategy::getTree(unit) != nullptr; });
		auto cantHitIt = treeIt == unitsMightHit.end() ? unitsMightHit.end() : (treeIt + 1);
		if (cantHitIt != unitsMightHit.end())
		{
			// projectile can't flight through tree
			unitsMightHit.erase(cantHitIt, unitsMightHit.end());
		}

		projectileInfo.m_possibleTargets.clear();
		projectileInfo.m_possibleTargets.reserve(16);
		for (const auto* unit : unitsMightHit)
		{
			projectileInfo.m_possibleTargets.push_back(unit->getId());
			if (unit->getId() == m_self.getId())
			{
				m_dangerousProjectiles.push_back(projectileInfo);
			}
		}
	}

	// 	// TODO - get obstacles on projectile path :)
	// 
	// 	auto itProjectile = std::find_if(std::begin(projectiles), std::end(projectiles), [&selfPoint, this](const model::Projectile& projectile)
	// 	{
	// 		Vec2d projectileSpeed = Vec2d(projectile.getSpeedX(), projectile.getSpeedY());
	// 		LineEquation firingLine = LineEquation::fromDirectionVector(projectile, projectileSpeed);
	// 
	// 		double collisionRadius = m_self.getRadius() + projectile.getRadius();
	// 
	// 		// http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
	// 		// compute the direction vector D from A to B
	// 		Vec2d D = projectileSpeed;
	// 		D.normalize();
	// 
	// 		// TODO - it looks like this code doesn't care on projectile flying direction and distance
	// 		// however, this may be somehow used to prevent staying on firing line like: me -> teammate's back -> enemy
	// 
	// 		// Now the fire line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
	// 		// where D is destination vector, A is a starting point, C is a circle center
	// 
	// 		// compute the value 't' of the closest point to the circle center (Cx, Cy)
	// 		Point2D projectilePoint = projectile;
	// 		double t = D.m_x*(selfPoint.m_x - projectilePoint.m_x) + D.m_y*(selfPoint.m_y - projectilePoint.m_y);
	// 		if (t < 0)
	// 			return false;  // DEBUG me: projectile direction is from 'self'
	// 
	// 		// This is the projection of C on the line from A to B.
	// 		// compute the coordinates of the point E on line and closest to C
	// 		Point2D E = Point2D(t*D.m_x + projectilePoint.m_x, t*D.m_y + projectilePoint.m_y);
	// 
	// 		// compute the euclidean distance from E to C
	// 		double LEC = E.getDistanceTo(selfPoint);
	// 
	// 		// test if the line intersects the circle
	// 		bool willHit = LEC < collisionRadius;
	// 		if (willHit)
	// 		{
	// 			// compute distance from t to circle intersection point
	// 			double dt = sqrt(collisionRadius*collisionRadius - LEC * LEC);
	// 
	// 			// compute first intersection point
	// 			Point2D F = Point2D((t - dt)*D.m_x + projectilePoint.m_x, (t - dt)*D.m_y + projectilePoint.m_y);
	// 			double test = F.getDistanceTo(selfPoint);
	// 
	// 			// compute second intersection point
	// 			// Gx = (t + dt)*Dx + Ax
	// 			// Gy = (t + dt)*Dy + Ay
	// 		}
	// 
	// 		return willHit;
	// 	});
	// 
	// 	if (itProjectile != std::end(projectiles))
	// 	{
	// 		// fire in the hall!
	// 		m_isUnderMissile = true;
	// 	}
}

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
			bonusSpawn.m_state = BonusSpawn::NO_BONUS;
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
	const double minionSpawnDxDy = 800;
	const double halfTileDxDy = 200;
	const double minionSpawnRadius = 250;
	const double diagonal = 1.3; // close to sqrt(2)

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

void State::updateDangerousEnemies()
{
	const Point2D enemyBase{ m_world.getWidth() - 400, 400 };
	bool isBetweenSpawnAndCorner = false;
	for (const PredictedUnit& spawn : m_enemySpawnPredictions)
		isBetweenSpawnAndCorner = isBetweenSpawnAndCorner || enemyBase.getDistanceTo(m_self) < enemyBase.getDistanceTo(spawn)*1.2/*hack*/;

	double spawnRadius = m_enemySpawnPredictions.empty() ? 1 : m_enemySpawnPredictions.front().getRadius();
	int spawnTraversalTicks = static_cast<int>((isBetweenSpawnAndCorner ? spawnRadius * 2 : spawnRadius) / m_game.getWizardStrafeSpeed());
	int noSpawnSafeTicks = m_nextMinionRespawnTick - m_world.getTickIndex() - spawnTraversalTicks;

	auto emptyPredictions = std::vector<PredictedUnit>();
	m_dangerousEnemies = filterPointers<const model::LivingUnit*>(
		[this](const model::Unit& u) {return MyStrategy::isEnemy(u, m_self) && m_self.getDistanceTo(u) < m_strategy->getSafeDistance(u); },
		m_world.getBuildings(), m_world.getWizards(), m_world.getMinions(),
		(noSpawnSafeTicks > 0 ? emptyPredictions : m_enemySpawnPredictions));
}
