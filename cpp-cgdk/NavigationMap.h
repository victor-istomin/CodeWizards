#pragma once
#include "Map.h"
class NavigationMap 
	: public Map
{
public:
	NavigationMap(size_t tileSize, const model::Game& game, const model::World& world, const model::Wizard& self);
	~NavigationMap();

private:
	virtual void initTiles(const model::Game& game, const model::World& world, const model::Wizard& self) override;

};

