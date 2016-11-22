#pragma once
#include <algorithm>
#include <vector>
#include <iterator>

class NonCopyable
{
	const NonCopyable& operator=(const NonCopyable&);
	NonCopyable(const NonCopyable&);

protected:
	NonCopyable() {}
};

#if _DEBUG
#	include <iostream>
#	include <ctime>
#	include <map>
#   undef max 
class Timer
{
	const char* m_function;
	clock_t m_start, m_finish;

	struct TotalHolder
	{
		struct Data
		{
			clock_t totalTime, maxTime;
			size_t  totalCalls;
			Data() : totalTime(0), maxTime(0), totalCalls(0) {}
		};

		std::map<const char* /*function*/, Data> m_totals;

		~TotalHolder()
		{
			std::cout << "Total timings: " << std::endl;
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

	explicit Timer(const char* function) : m_function(function), m_start(clock()), m_finish(0) {}

	~Timer()
	{
		m_finish = clock();
		static TotalHolder totalHolder;
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




