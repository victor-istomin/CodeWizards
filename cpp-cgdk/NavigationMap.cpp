#include "NavigationMap.h"



NavigationMap::NavigationMap(size_t tileSize, const model::Game& game, const model::World& world, const model::Wizard& self)
	: Map(tileSize, game, world, self)
{
}


NavigationMap::~NavigationMap()
{
}

void NavigationMap::initTiles(const model::Game& game, const model::World& world, const model::Wizard& self)
{
	fillWith(world.getWizards(), [&self](const model::Wizard& w) { return w.getId() != self.getId(); });  // should not impede itself
	fillWith(world.getMinions());
	fillWith(world.getBuildings());
	fillWith(world.getProjectiles());
	fillWith(world.getTrees());

	// todo - forest
}
