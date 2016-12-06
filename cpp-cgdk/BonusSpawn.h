#pragma once
#include "Point2D.h"
#include "Map.h"   // for TilePath

struct BonusSpawn
{
	enum BonusState
	{
		UNKNOWN = 0,   // don't know whether there is a bonus
		HAS_BONUS,
		NO_BONUS,
	};

	struct WizardsHealth
	{
		double enemies;
		double teammates;

		WizardsHealth() : enemies(0), teammates(0) {}
	};

	Point2D    m_point;
	BonusState m_state;          // true if someone has collected bonus
	int        m_lastCheckTick;
	int        m_teamateCompetitors;
	double     m_dangerHandicap;

	Map::PointPath m_smoothPathCache;
	Map::TilesPath m_tilesPathCache;
// 	WizardsHealth  m_wizardsHp;   // maybe, bad idea

	static const size_t  COUNT = 2;
	static const Point2D RESPAWN_POINTS[];
	static const double  DANGER_HANDICAP;

	BonusSpawn(const Point2D& point, BonusState state, double handicap = 0) 
		: m_point(point), m_state(state), m_lastCheckTick(0), m_dangerHandicap(handicap), m_teamateCompetitors(0) {}
};


