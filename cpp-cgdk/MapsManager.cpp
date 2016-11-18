#include "MapsManager.h"
#include "Map.h"
#include "WorldMap.h"
#include "DebugVisualizer.h"

MapsManager::MapsManager(const model::Game& game, const model::World& world, const model::Wizard& self)
{
	// initialize maps
	update(game, world, self);
}

void MapsManager::update(const model::Game& game, const model::World& world, const model::Wizard& self)
{
	// TODO - cache?
	const size_t worldTileSize = static_cast<size_t>(self.getRadius() * 2);
	m_maps[MT_WORLD_MAP] = std::make_unique<WorldMap>(worldTileSize, game, world, self);
	m_maps[MT_WORLD_MAP]->initTiles(game, world, self);
}

MapsManager::~MapsManager()
{
}

void MapsManager::visualize(DebugMessage& message, MapType mapType) const
{
	message.visualizeMap(getMap(mapType));
}

const Map* MapsManager::getMap(MapType type) const
{
	auto mapIt = m_maps.find(type);
	assert(mapIt != m_maps.end());
	return mapIt->second.get();
}
