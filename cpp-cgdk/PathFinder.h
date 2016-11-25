#pragma once
#include "Vec.h"
#include "Map.h"
#include <list>

#include <deque>
#include "sparsepp.h"

class PathFinder
{
public:
	PathFinder();
	~PathFinder();

	typedef std::list<Map::TileIndex> TilesPath;

	TilesPath getPath(const Point2D& start, const Point2D& finish, const Map& map);

	typedef std::vector<size_t> TileStateHashes;
	void updateTileStates(const TileStateHashes& hashes);

private:
	PathFinder& operator=(const PathFinder&); // neither accessible nor implemented
	PathFinder(const PathFinder&); // neither accessible nor implemented

	struct Transition
	{
		double         m_cost;
		Map::TileIndex m_parent;

		Transition(double cost = -1, Map::TileIndex parent = Map::TileIndex()) : m_cost(cost), m_parent(parent) {}
	};

	struct CacheKey
	{
		Map::TileIndex m_from;
		Map::TileIndex m_to;

// 		struct Hasher
// 		{
// 			size_t operator()(const CacheKey& key) const
// 			{
// 				size_t seed = Map::TileIndex::Hasher()(key.m_from);
// 				hash_combine<Map::TileIndex, Map::TileIndex::Hasher>(seed, key.m_to);
// 				return seed;
// 			}
// 		};

		CacheKey(const Map::TileIndex& from, const Map::TileIndex& to) : m_from(from), m_to(to) {}
		CacheKey() : m_from(), m_to() {}

		bool operator==(const CacheKey& right) const 
		{ 
			return m_from == right.m_from && m_to == right.m_to;
		}
	};

	struct CacheItem
	{
		CacheKey  m_key;
		TilesPath m_path;

		CacheItem(const CacheKey& key, const TilesPath& path) : m_key(key), m_path(path) {}

		CacheItem(const CacheItem&) = delete;
		CacheItem& operator=(const CacheItem&) = delete;
	};

		// this one is much faster than VS 2015 unordered_map in Debug
	typedef spp::sparse_hash_map<Map::TileIndex/*to*/, Transition, Map::TileIndex::Hasher> Transitions;
	typedef std::deque<CacheItem> PathCache;

	PathCache        m_cache;
	TileStateHashes  m_tilesHashe;
	
	double getHeuristics(const Map::TileIndex& from, const Map::TileIndex& to);  // should be 'admissible heuristics'
	double getTransitionCost(const Map::TileIndex& from, const Map::TileIndex& to);
	TilesPath reconstructPath(const Map::TileIndex& start, const Map::TileIndex& finish, const Transitions& transitions);

};

