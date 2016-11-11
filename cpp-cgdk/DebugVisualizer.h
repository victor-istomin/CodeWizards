#pragma once

#if _DEBUG
#	include "../russian-ai-cup-visual/clients/cpp/Debug.h"

	class DebugVisualizer
	{
		Debug m_debug;

	public:
		DebugVisualizer() {}
		void test(Point2D point)
		{
			m_debug.beginPre();
			m_debug.fillRect(200, 200, 3000, 3000);
			m_debug.endPre();

			m_debug.beginPost();
			m_debug.text(point.m_x, point.m_y, "Test");
			m_debug.endPost();
		}
	};

#else

	class DebugVisualizer
	{

	};

#endif

