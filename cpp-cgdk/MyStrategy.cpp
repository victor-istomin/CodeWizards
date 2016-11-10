#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include "MyStrategy.h"
#include "model/LivingUnit.h"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <list>
#include <algorithm>
#include <iterator>
#include <limits>
#include <cassert>

using namespace model;
using namespace std;

const double MyStrategy::WAYPOINT_RADIUS = 100.0;
const double State::LOW_HP_FACTOR        = 0.3;

const double Point2D::k_epsilon = 0.0001;


void MyStrategy::move(const Wizard& self, const World& world, const Game& game, Move& move) 
{
	initState(self, world, game, move);

	// TODO - remove dirty hack inside !!!
	const LivingUnit* nearestTarget = getNearestTarget();

	double tooCloseDistance = 3 * game.getStaffRange() + self.getRadius() + (m_state->m_isLowHP ? game.getStaffRange() : 0);
	bool isTooCloseToEnemy = nearestTarget && self.getDistanceTo(*nearestTarget) < tooCloseDistance;

	// Если осталось мало жизненной энергии, отступаем к предыдущей ключевой точке на линии.
	bool isRetreating = isTooCloseToEnemy || m_state->m_isLowHP;
	if (isRetreating) 
	{
		retreatTo(getPreviousWaypoint(), move);
	}

// 	// retreat also may be missile avoidance maneuver
// 	// todo - looks impossible without bonus
// 	if (m_state->isUnderMissile())
// 	{
// 		const Projectile* projectile = m_state->m_attackingProjectile;
// 		assert(nullptr != projectile);
// 
// 		double distance = projectile->getDistanceTo(self);
// 		double speed = std::hypot(self.getSpeedX() - projectile->getSpeedX(), self.getSpeedY() - projectile->getSpeedY());
// 
// 		int ticksToHit = std::floor(distance / speed);
// 
// 		//move.setSpeed(-10);
// 	}


	if (m_state->m_isEnemyAround && isRetreating)
	{
		// Постоянно двигаемся из-стороны в сторону, чтобы по нам было сложнее попасть.
		// Считаете, что сможете придумать более эффективный алгоритм уклонения? Попробуйте! ;)

		if ((world.getTickIndex() - m_lastStrafeChangeTick) > STRAFE_CHANGE_INTERVAL)
		{
			m_lastStrafe = std::rand() % 2 ? game.getWizardStrafeSpeed() : -game.getWizardStrafeSpeed();
			m_lastStrafeChangeTick = world.getTickIndex();
		}
		
		move.setStrafeSpeed(m_lastStrafe);   // TODO - don't strafe to the enemy tower :)
	}

	// Если видим противника ...
	if (nearestTarget != nullptr) 
	{
		double distance = self.getDistanceTo(*nearestTarget);

		// ... и он в пределах досягаемости наших заклинаний, ...
		if (distance < self.getCastRange())
		{
			double angle = self.getAngleTo(*nearestTarget);

			// ... то поворачиваемся к цели.
			move.setTurn(angle);

			// Если цель перед нами, ...
			if (m_state->isReadyForAction(ActionType::ACTION_MAGIC_MISSILE))
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

			return;
		}
	}

	// Если нет других действий, просто продвигаемся вперёд.
	if (move.getSpeed() == 0 && std::abs(move.getTurn() < PI/1000))
	{
		goTo(getNextWaypoint(), move);
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
void MyStrategy::initialSetup()
{
	srand(time(nullptr));

	double mapSize = m_state->m_game.getMapSize();

	std::map<model::LaneType, TWaypoints> waypointsMap;

	waypointsMap[LaneType::LANE_MIDDLE] = TWaypoints 
	{ 
		Point2D(100.0, mapSize - 100.0),
		(rand() % 2 == 1 ? Point2D(600.0, mapSize - 200.0) : Point2D(600.0, mapSize - 200.0)),
		Point2D(800.0, mapSize - 800.0),
		Point2D(mapSize - 600.0, 600.0),
	};

	waypointsMap[LaneType::LANE_TOP] = TWaypoints 
	{
		Point2D(100.0, mapSize - 100.0),
		Point2D(100.0, mapSize - 400.0),
		Point2D(200.0, mapSize - 800.0),
		Point2D(200.0, mapSize * 0.75),
		Point2D(200.0, mapSize * 0.5),
		Point2D(200.0, mapSize * 0.25),
		Point2D(200.0, 200.0),
		Point2D(mapSize * 0.25, 200.0),
		Point2D(mapSize * 0.5, 200.0),
		Point2D(mapSize * 0.75, 200.0),
		Point2D(mapSize - 200.0, 200.0)
	};

	waypointsMap[LaneType::LANE_BOTTOM] = TWaypoints
	{
		Point2D(100.0, mapSize - 100.0),
		Point2D(400.0, mapSize - 100.0),
		Point2D(800.0, mapSize - 200.0),
		Point2D(mapSize * 0.25, mapSize - 200.0),
		Point2D(mapSize * 0.5, mapSize - 200.0),
		Point2D(mapSize * 0.75, mapSize - 200.0),
		Point2D(mapSize - 200.0, mapSize - 200.0),
		Point2D(mapSize - 200.0, mapSize * 0.75),
		Point2D(mapSize - 200.0, mapSize * 0.5),
		Point2D(mapSize - 200.0, mapSize * 0.25),
		Point2D(mapSize - 200.0, 200.0)
	};

	LaneType line = LaneType::_LANE_UNKNOWN_;
	switch (m_state->m_self.getId()) 
	{
	case 1:
	case 2:
	case 6:
	case 7:
		line = LANE_TOP;
		break;

	case 3:
	case 8:
		line = LANE_MIDDLE;
		break;

	case 4:
	case 5:
	case 9:
	case 10:
		line = LANE_BOTTOM;
		break;

	default:
		break;
	}

	m_waypoints = waypointsMap[line];

	// Наша стратегия исходит из предположения, что заданные нами ключевые точки упорядочены по убыванию
	// дальности до последней ключевой точки. Сейчас проверка этого факта отключена, однако вы можете
	// написать свою проверку, если решите изменить координаты ключевых точек.

	/*Point2D lastWaypoint = waypoints[waypoints.length - 1];

	Preconditions.checkState(ArrayUtils.isSorted(waypoints, (waypointA, waypointB) -> Double.compare(
	waypointB.getDistanceTo(lastWaypoint), waypointA.getDistanceTo(lastWaypoint)
	)));*/
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
	m_state = std::make_unique<State>(self, world, game, move);

	if (m_waypoints.empty())
	{
		// initial setup
		initialSetup();
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
Point2D MyStrategy::getNextWaypoint()
{
	// assume that waypoint are sorted by-distance !!!

	int lastWaypointIndex = m_waypoints.size() - 1;
	Point2D lastWaypoint = m_waypoints[lastWaypointIndex];

	for (int waypointIndex = 0; waypointIndex < lastWaypointIndex; ++waypointIndex) 
	{
		Point2D waypoint = m_waypoints[waypointIndex];

		if (waypoint.getDistanceTo(m_state->m_self) <= WAYPOINT_RADIUS)
			return m_waypoints[waypointIndex + 1];

		if (lastWaypoint.getDistanceTo(waypoint) < lastWaypoint.getDistanceTo(m_state->m_self))
			return waypoint;
	}

	return lastWaypoint;
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

	Point2D firstWaypoint = m_waypoints[0];

	for (int waypointIndex = m_waypoints.size() - 1; waypointIndex > 0; --waypointIndex) 
	{
		Point2D waypoint = m_waypoints[waypointIndex];

		if (waypoint.getDistanceTo(m_state->m_self) <= WAYPOINT_RADIUS)
			return m_waypoints[waypointIndex - 1];

		if (firstWaypoint.getDistanceTo(waypoint) < firstWaypoint.getDistanceTo(m_state->m_self))
			return waypoint;
	}

	return firstWaypoint;
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
	std::list<LivingUnit> targets;

	const World& world = m_state->m_world;
	const Wizard& self = m_state->m_self;

	std::copy(world.getBuildings().begin(), world.getBuildings().end(), std::back_inserter(targets));
	std::copy(world.getWizards().begin(), world.getWizards().end(), std::back_inserter(targets));
	std::copy(world.getMinions().begin(), world.getMinions().end(), std::back_inserter(targets));

	const LivingUnit* nearestTarget = nullptr;

	double nearestTargetDistance = std::numeric_limits<double>::max();
	int minEnemyHealth = std::numeric_limits<int>::max();
	int maxEnemyHealth = std::numeric_limits<int>::min();

	for (const LivingUnit& target : targets)
	{
		if (target.getFaction() == Faction::FACTION_NEUTRAL || target.getFaction() == self.getFaction())
		{
			continue;
		}

		minEnemyHealth = std::min(minEnemyHealth, target.getLife());
		maxEnemyHealth = std::max(maxEnemyHealth, target.getLife());
	}

	for (const LivingUnit& target : targets) 
	{
		if (target.getFaction() == Faction::FACTION_NEUTRAL || target.getFaction() == self.getFaction()) 
		{
			continue;
		}

		double distance =  self.getDistanceTo(target);
		if (target.getLife() == minEnemyHealth)
		{
			distance /= 2;  // this is priority hack
		}
		if (target.getLife() == maxEnemyHealth)
		{
			distance *= 1.5;  // this is priority hack
		}

		if (distance < nearestTargetDistance) 
		{
			nearestTarget = &target;
			nearestTargetDistance = distance;
		}
	}

	static LivingUnit dirtyHack = m_state->m_self;   // TODO: remove this hack !!!
	if(nearestTarget != nullptr)
	{ 
		dirtyHack = *nearestTarget;
		return &dirtyHack;
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
void MyStrategy::goTo(const Point2D& point, model::Move& move)
{
	double angle = m_state->m_self.getAngleTo(point.m_x, point.m_y);

	move.setTurn(angle);

	if (abs(angle) < m_state->m_game.getStaffSector() / 4.0)
		move.setSpeed(m_state->m_game.getWizardForwardSpeed());
}

void MyStrategy::retreatTo(const Point2D& point, model::Move& move)
{
	double forwardAngle = m_state->m_self.getAngleTo(point.m_x, point.m_y);
	double angle = (forwardAngle < 0 ? 2*PI - forwardAngle : forwardAngle) - PI;

	move.setTurn(angle);  // may be overridden when shooting 

// 	if (abs(angle) < m_state->m_game.getStaffSector() / 4.0)    // TODO: factor is ok? 
		move.setSpeed(- m_state->m_game.getWizardBackwardSpeed());
}

MyStrategy::MyStrategy()
	: m_lastStrafeChangeTick(0)
	, m_lastStrafe(0.0)
{ }

State::State(const model::Wizard& self, const model::World& world, const model::Game& game, model::Move& move) 
	: m_self(self), m_world(world), m_game(game), m_move(move)
	, m_isEnemyAround(false)
	, m_attackingProjectile(nullptr)
	, m_isLowHP(false)
	, m_cooldownTicks()
{
	const auto& wizards = m_world.getWizards();
	const Point2D selfPoint = self;

	m_isLowHP = self.getLife() < (self.getMaxLife() * State::LOW_HP_FACTOR);

	// set cooldown ticks, taking into account both specific and common cooldown
	std::copy(self.getRemainingCooldownTicksByAction().begin(), self.getRemainingCooldownTicksByAction().end(), m_cooldownTicks.begin());
	std::transform(m_cooldownTicks.begin(), m_cooldownTicks.end(), m_cooldownTicks.begin(),
		[&self](int cooldown) { return std::max(cooldown, self.getRemainingActionCooldownTicks()); });

	m_isEnemyAround = std::end(wizards) != std::find_if(wizards.begin(), wizards.end(), 
		[&self, &selfPoint](const Wizard& w) 
	{
		double visionRange = std::max(self.getVisionRange(), w.getVisionRange());
		return w.getFaction() != self.getFaction() && selfPoint.getDistanceTo(w) <= visionRange;
	});

	const auto& projectiles = m_world.getProjectiles();
	auto itProjectile = std::find_if(std::begin(projectiles), std::end(projectiles),
		[&selfPoint, &self, &game](const model::Projectile& projectile)
	{
		Vec2d projectileSpeed   = Vec2d(projectile.getSpeedX(), projectile.getSpeedY());
		LineEquation firingLine = LineEquation::fromDirectionVector(projectile, projectileSpeed);

// 		if (firingLine.isContains(selfPoint))
// 		{
// 			return true;   // hit just into center
// 		}

		double collitionRadius = self.getRadius() + projectile.getRadius();

		// http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
		// compute the direction vector D from A to B
		Vec2d D = projectileSpeed;
		D.normalize();
		Point2D projectilePoint = projectile;
		double projectileDistance = selfPoint.getDistanceTo(projectilePoint);
		bool isIncoming = projectileDistance > selfPoint.getDistanceTo((D + Vec2d::fromPoint(projectilePoint)).toPoint());
		if (!isIncoming)
			return false;

		// TODO - improve max distance calculation
		const double maxProjectileDistance = projectile.getRadius() + self.getRadius() + 
			(projectile.getType() == model::PROJECTILE_DART ? game.getFetishBlowdartAttackRange() : game.getWizardCastRange());

		if (projectileDistance > maxProjectileDistance)
			return false;

		// TODO - it looks like this code doesn't care on projectile flying direction and distance
		// however, this may be somehow used to prevent staying on firing line like: me -> teammate's back -> enemy

		// Now the fire line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
		// where D is destination vector, A is a starting point, C is a circle center

		// compute the value t of the closest point to the circle center (Cx, Cy)
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

				// compute second intersection point
				// Gx = (t + dt)*Dx + Ax
				// Gy = (t + dt)*Dy + Ay
		}

		return LEC <= collitionRadius;
	});

	if (itProjectile != std::end(projectiles))
	{
		// fire in the hall!
		m_attackingProjectile = &(*itProjectile);
	}


}
