#define _SECURE_SCL_DEPRECATE 0
#include "PathFinder.h"
#include <map>
#include <unordered_set>
#include "Fringe.h"

PathFinder::PathFinder()
{
}


double PathFinder::getHeuristics(const Map::TileIndex& from, const Map::TileIndex& to)
{
	return from.manhattanDistance(to);
}

double PathFinder::getTransitionCost(const Map::TileIndex& from, const Map::TileIndex& to)
{
	return from.manhattanDistance(to);
}

PathFinder::TilesPath PathFinder::reconstructPath(const Map::TileIndex& start, const Map::TileIndex& finish, const Transitions& transitions)
{
	TilesPath path;

	Map::TileIndex current = finish;
	while (current != start)
	{
		path.push_front(current);
		
		Transitions::const_iterator parentIt = transitions.find(current);
		current = parentIt->second.m_parent;
	}

	CacheKey key{ start, finish };
	TilesPath pathToCache = path;

	const int STEPS    = 3;
	const int MIN_SIZE = 6;
	for (int i = 0; i < STEPS && pathToCache.size() > MIN_SIZE; ++i)
	{
		m_cache.emplace_back(key, pathToCache);
		if (m_cache.size() > 10)
			m_cache.pop_front();

		pathToCache.pop_front();
		key.m_from = pathToCache.front();
	}

	const int MAX_CACHE_SIZE = 10 * STEPS;
	if (m_cache.size() > MAX_CACHE_SIZE)
		m_cache.pop_front();

	return path;
}

PathFinder::~PathFinder()
{
}

PathFinder::TilesPath PathFinder::getPath(const Point2D& start, const Point2D& finish, const Map& map)
{
	Timer timer(__FUNCTION__);

	using TileIndex = Map::TileIndex;
	using spp::sparse_hash_map;
	using spp::sparse_hash_set;

	typedef sparse_hash_set<TileIndex, TileIndex::Hasher> IndexSet;
	typedef Fringe<double, TileIndex> CostMap;

	TilesPath path;
	const TileIndex startIdx = map.getTileIndex(start);
	const TileIndex finishIdx = map.getTileIndex(finish);

	static size_t dbg_cacheHit = 0;
	static size_t dbg_cacheMiss = 0;

	const CacheKey key{ startIdx, finishIdx };
	auto cacheIt = std::find_if(m_cache.begin(), m_cache.end(), [&key](const CacheItem& item) {return item.m_key == key; });
	if (cacheIt != m_cache.end())
	{
		dbg_cacheHit++;
		return cacheIt->m_path;
	}

	dbg_cacheMiss++;
	
 	if (startIdx == finishIdx)
		return TilesPath{ startIdx };

	IndexSet    closedSet;
	IndexSet    fringeSet;
	CostMap     fringe(1024 * 32);
	Transitions transitions;

	closedSet.reserve(1024 * 32);
	fringeSet.reserve(1024 * 32);
	transitions.reserve(1024 * 32);

	fringe.insert(std::make_pair(getHeuristics(startIdx, finishIdx), startIdx));
	fringeSet.insert(startIdx);
	while (!fringe.empty())
	{
		auto thisCostAndIndex = fringe.pop();
		double    thisCost = thisCostAndIndex.first;
		TileIndex current  = thisCostAndIndex.second;

		double    thisCostGx = thisCost - getHeuristics(current, finishIdx);

		fringeSet.erase(current);
		closedSet.insert(current);

		if (current == finishIdx)
		{
			path = reconstructPath(startIdx, current, transitions);
			break;
		}

		// add neighbors
		TileIndex neighbors[] =
		{
			TileIndex(current.m_x + 1, current.m_y),
			TileIndex(current.m_x - 1, current.m_y),
			TileIndex(current.m_x, current.m_y + 1),
			TileIndex(current.m_x, current.m_y - 1),
		};

		const Transition& currentTransition = transitions[current];
		for (const TileIndex& next : neighbors)
		{
			if (!next.isValid(map))
				continue;

			// HACK - fix me: start and finish always 'not occupied'
			bool isOccupied = next != finishIdx && next != startIdx && map.getTileState(next).isOccupied();
			if(isOccupied || next == currentTransition.m_parent)
				continue;

			double nextCost        = thisCostGx + getTransitionCost(current, next) + getHeuristics(next, finishIdx);
			bool   isAlreadySeen   = closedSet.find(next) != closedSet.end() || fringeSet.find(next) != fringeSet.end();
			auto   oldTransitionIt = isAlreadySeen ? transitions.find(next) : transitions.end();

			if (!isAlreadySeen || nextCost < oldTransitionIt->second.m_cost)  // if current node unseen or current path is better than old			
			{				
				fringe.insert(std::make_pair(nextCost, next));
				fringeSet.insert(next);
				
				Transition& t = transitions[next];
				t.m_cost   = nextCost;
				t.m_parent = current;
			}
		}
	}

	return path;
}

void PathFinder::updateTileStates(const TileStateHashes& hashes)
{
	if (hashes != m_tilesHashe)
	{
		m_cache.clear(); // flush
		m_tilesHashe = hashes;
	}
}
