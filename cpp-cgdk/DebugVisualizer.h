#pragma once

#if _DEBUG
#	include "../russian-ai-cup-visual/clients/cpp/Debug.h"
#	include "model/Wizard.h"
#	include <memory>

	class DebugVisualizer
	{
		Debug m_debug;

	public:
		DebugVisualizer() {}
		void beginPre() { m_debug.beginPre();  }
		void endPre() { m_debug.endPre(); }

		void drawWaypoint(const model::Wizard& me, const Point2D& waypoint)
		{
			Point2D selfPoint = me;

			const int32_t color = 0x44CC99;

			m_debug.fillCircle(waypoint.m_x, waypoint.m_y, me.getRadius(), color);
			m_debug.line(selfPoint.m_x, selfPoint.m_y, waypoint.m_x, waypoint.m_y, color);
		}
	};

	class DebugMessage
	{
		DebugVisualizer& m_render;
		const model::Wizard& m_self;

		std::unique_ptr<Point2D> m_nextWaypoint;

	public:
		DebugMessage(DebugVisualizer& render, const model::Wizard& self)
			: m_render(render), m_self(self)
		{}

		void setNextWaypoint(const Point2D& waypoint) { m_nextWaypoint = std::make_unique<Point2D>(waypoint); }

		~DebugMessage()
		{
			// commit
			m_render.beginPre();
			if (m_nextWaypoint)
				m_render.drawWaypoint(m_self, *m_nextWaypoint);
			m_render.endPre();
		}

	};

#else

	class DebugVisualizer
	{

	};

	class DebugMessage
	{
	public:

		DebugMessage(DebugVisualizer& render, const model::Wizard& self) {}
		~DebugMessage() { }

		void setNextWaypoint(const Point2D& waypoint) { }
	};

#endif

