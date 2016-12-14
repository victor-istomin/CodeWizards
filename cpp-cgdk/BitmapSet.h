#pragma once
#include <vector>

class BitmapSet
{
	std::vector<bool> m_bitmap;

public:
	explicit BitmapSet(size_t size)
		: m_bitmap(size, false)
	{}

	bool contains(int index) const { return m_bitmap[index]; }
	void insert(int index)         { m_bitmap[index] = true; }
	void erase(int index)          { m_bitmap[index] = false; }
};
