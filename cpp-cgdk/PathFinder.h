#pragma once
#include "Vec.h"
#include "Map.h"
#include <list>

#include <unordered_map>
#include "sparsepp.h"

class PathFinder
{
public:
	PathFinder();
	~PathFinder();

	typedef std::list<Map::TileIndex> TilesPath;

	TilesPath getPath(const Point2D& start, const Point2D& finish, const Map& map);

private:
	PathFinder& operator=(const PathFinder&); // neither accessible nor implemented
	PathFinder(const PathFinder&); // neither accessible nor implemented

	struct Transition
	{
		double         m_cost;
		Map::TileIndex m_parent;

		Transition(double cost = -1, Map::TileIndex parent = Map::TileIndex()) : m_cost(cost), m_parent(parent) {}
	};

	// incredibly slow on MSVC on debug
	//typedef std::unordered_map<Map::TileIndex/*to*/, Transition, Map::TileIndex::Hasher> Transitions;
	typedef spp::sparse_hash_map<Map::TileIndex/*to*/, Transition, Map::TileIndex::Hasher> Transitions;

	double getHeuristics(const Map::TileIndex& from, const Map::TileIndex& to);  // should be 'admissible heuristics'
	double getTransitionCost(const Map::TileIndex& from, const Map::TileIndex& to);
	TilesPath reconstructPath(const Map::TileIndex& start, const Map::TileIndex& finish, const Transitions& transitions);

};

