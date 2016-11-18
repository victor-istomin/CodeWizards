#pragma once
#include <algorithm>

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



