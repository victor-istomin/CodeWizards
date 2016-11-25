#pragma once
#include "Utils.h"
#include "model/World.h"
#include "model/Wizard.h"
#include "model/Game.h"

#include <map>
#include <memory>

class Map;
class DebugMessage;
class PathFinder;

class MapsManager
	: NonCopyable
{
public:
 	enum MapType
	{
		MT_INVALID = 0,
		MT_WORLD_MAP,          // world map with forest and buildings
		MT_NAVIGATION_DETAILS, // navigation details high-res map
	};
	
	typedef std::unique_ptr<Map>      MapPtr;
	typedef std::map<MapType, MapPtr> Maps;

	explicit MapsManager(const model::Game& game, const model::World& world, const model::Wizard& self, PathFinder& pathFinder);
	void update(const model::Game& game, const model::World& world, const model::Wizard& self, PathFinder& pathFinder);
	~MapsManager();

	void visualize(DebugMessage& message, MapType mapType) const;
	const Map* getMap(MapType type) const;

private:
	Maps m_maps;
};

