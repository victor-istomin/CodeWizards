#include "WorldMap.h"
#include "model/Game.h"
#include "model/Wizard.h"

#include <cmath>
#include <algorithm>
#include <iterator>

WorldMap::WorldMap(size_t tileSize, const model::Game& game, const model::World& world, const model::Wizard& self)
	: Map(tileSize, game, world, self)
{
}


WorldMap::~WorldMap()
{
}

void WorldMap::initTiles(const model::Game& game, const model::World& world, const model::Wizard& self)
{
	Timer timer(__FUNCTION__);

	calculateVisibility(game, self, world);
	calculateObstacles(game, world, self);
}

void WorldMap::calculateObstacles(const model::Game &game, const model::World &world, const model::Wizard &self)
{
	int worldSizeTiles = static_cast<size_t>(game.getMapSize()) / getTileSize();
	int roadRadius = worldSizeTiles / 10;

	auto fillRoads = [roadRadius, worldSizeTiles](int x, int y, TileState& tile)
	{
// 		if (tile.m_isVisible)
// 			return; // this rough map is needed under Fog of War only

		bool isVerticalRoad = x < roadRadius || x >(worldSizeTiles - roadRadius);
		bool isHorizontalRoad = y < roadRadius || y >(worldSizeTiles - roadRadius);
		bool isRaisingDiagonal = std::abs(worldSizeTiles - x - y) < roadRadius;
		bool isFallingDiagonal = std::abs(x - y) < roadRadius;

		if (!isVerticalRoad && !isHorizontalRoad && !isRaisingDiagonal && !isFallingDiagonal)
			tile.m_state = TileState::OCCUPIED;
	};

	fillWithEquations(fillRoads);

	const auto& buildings = world.getBuildings();
	fillWith(buildings);
	fillWith(world.getTrees());

	// mirror building for enemy's coordinates
	Point2D worldSize = Point2D(world.getWidth(), world.getHeight());
	for (const auto& building : buildings)
	{
		if (building.getFaction() != self.getFaction())
			continue;

		fillCircle(worldSize - building, building.getRadius());
	}
}

void WorldMap::calculateVisibility(const model::Game &game, const model::Wizard &self, const model::World &world)
{
	const int halfTile = getTileSize() / 2;
	auto fillUnitVisibility = [this, halfTile, self](double unitVisionRange, const model::CircularUnit& unit)
	{
// 		if (unit.getFaction() == self.getFaction())
// 			this->fillCircle(unit, unitVisionRange - halfTile, [](TileState& tile) { tile.m_isVisible = true; });
	};

	auto partialFunction = [](auto func, auto par1) { return [par1, func](auto par2) { return func(par1, par2); }; };

	double buildingVisionRange = std::min(game.getGuardianTowerVisionRange(), game.getFactionBaseVisionRange());
	std::for_each(world.getBuildings().begin(), world.getBuildings().end(), partialFunction(fillUnitVisibility, buildingVisionRange));
	std::for_each(world.getWizards().begin(),   world.getWizards().end(),   partialFunction(fillUnitVisibility, game.getWizardVisionRange()));
	std::for_each(world.getMinions().begin(),   world.getMinions().end(),   partialFunction(fillUnitVisibility, game.getMinionVisionRange()));
}

