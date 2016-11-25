#pragma once
#include "Map.h"

class PathFinder;
class WorldMap 
	: public Map
{
public:
	WorldMap(size_t tileSize, const model::Game& game, const model::World& world, const model::Wizard& self);
	~WorldMap();

	void updatePathFinder(PathFinder& pathFinder) const;

private:
	virtual void initTiles(const model::Game& game, const model::World& world, const model::Wizard& self) override;

	void calculateObstacles(const model::Game &game, const model::World &world, const model::Wizard &self);

	void calculateVisibility(const model::Game &game, const model::Wizard &self, const model::World &world);

	template <typename TileVisitor>
	void fillWithEquations(TileVisitor f)  // 'f' is void(x, y, TileState& state)
	{
		if (getTilesYX().empty())
			return;

		int height = getTilesYX().size();
		int width  = getTilesYX().front().size();

		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				f(x, y, getTilesYX()[y][x]);
			}
		}
	}

	double getUnitVisionRange(const model::CircularUnit& unit) const;

	size_t hashRowObstacles(const TilesRow& row) const;
};

