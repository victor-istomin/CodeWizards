#pragma once
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>
#include <string>
#include "Point2D.h"

class NonCopyable
{
	const NonCopyable& operator=(const NonCopyable&);
	NonCopyable(const NonCopyable&);

protected:
	NonCopyable() {}
};

#if _DEBUG || USE_TIMER
#	include <iostream>
#	include <ctime>
#	include <map>
#   undef max 
class Timer
{
	const char*  m_function;
	clock_t      m_start, m_finish;
	std::string  m_player;
	long long    m_playerId;

	struct TotalHolder
	{
		struct Data
		{
			clock_t totalTime, maxTime;
			size_t  totalCalls;
			Data() : totalTime(0), maxTime(0), totalCalls(0) {}
		};

		std::map<const char* /*function*/, Data> m_totals;
		std::string m_player;
		long long   m_playerId;

		TotalHolder() : m_playerId(0) {}

		~TotalHolder()
		{
			std::cout << "Total timings for " << m_player << " (" << m_playerId << "): " << std::endl;
			for (const auto& function : m_totals)
			{
				const Data& totals = function.second;
				std::cout << "  " << function.first << ": total " << totals.totalTime << " clocks, " << totals.totalCalls << " calls; "
					<< "average: " << (totals.totalCalls != 0 ? totals.totalTime / totals.totalCalls : 0) << " clocks per call; "
					<< "max: " << totals.maxTime << std::endl;
			}
			std::cout << "Press enter..."; std::cin.get();
		}
	};

public:

	explicit Timer(const char* function, const std::string& player = std::string(), long long id = -1)
		: m_function(function), m_start(clock()), m_finish(0), m_player(player), m_playerId(id) { }

	~Timer()
	{
		m_finish = clock();
		static TotalHolder totalHolder;

		if (!m_player.empty())
			totalHolder.m_player = m_player;
		if (m_playerId != -1)
			totalHolder.m_playerId = m_playerId;

		TotalHolder::Data& totals = totalHolder.m_totals[m_function];

		totals.totalCalls++;
		clock_t elapledTime = m_finish - m_start;
		totals.totalTime += elapledTime;
		totals.maxTime = std::max(totals.maxTime, elapledTime);
	}
};
#else
struct Timer
{
	Timer(...) {}
};

#endif
#include "model\LivingUnit.h"

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

template <typename OutputIterator, typename Functor, typename UnitCollection>
void filterIf(OutputIterator destination, const Functor& predicate, const UnitCollection& units)
{
	for (const auto& unit : units)
	{
		if (predicate(unit))
		{
			*destination++ = &unit;
		}
	}
}

template <typename OutputIterator, typename Functor, typename UnitCollection, typename ...Parameters>
void filterIf(OutputIterator destination, const Functor& predicate, const UnitCollection& units, const Parameters& ... params)
{
	filterIf(destination, predicate, units);

	if (sizeof...(Parameters) != 0)
	{
		filterIf(destination, predicate, params...);
	}
}

template <typename PointerType, typename Functor, typename ...Parameters>
std::vector<PointerType> filterPointers(const Functor& predicate, const Parameters& ... params)
{
	std::vector<PointerType> results;
	results.reserve(128);
	filterIf(std::back_inserter(results), predicate, params...);
	return results;
}

template <typename T, typename Hasher = std::hash<T>>
inline void hash_combine(std::size_t& seed, const T& v)
{
	Hasher hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}


class PredictedUnit : public model::LivingUnit
{
	static const int UNIT_HEALTH = 100;
	static const long long DEFAULT_ID = -1;
	static long long s_lastId;

	static const std::vector<model::Status> s_emptyStatus;

	double m_predictedDamage;
	int    m_expectedTick;
	double m_safeDistance;

public:
	PredictedUnit(const Point2D& location, double radius, model::Faction faction, int expectedTick, double damage, double safeDistance)
		: LivingUnit(s_lastId--, location.m_x, location.m_y, 0, 0, 0, faction, radius, UNIT_HEALTH, UNIT_HEALTH, s_emptyStatus)
		, m_predictedDamage(damage)
		, m_expectedTick(expectedTick)
		, m_safeDistance(safeDistance)
	{

	}

	double predictedDamage() const { return m_predictedDamage; }
	int    expectedTick()    const { return m_expectedTick; }
	double safeDistance()    const { return m_safeDistance; }
};

