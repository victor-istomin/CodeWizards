#pragma once
#include <algorithm>
#include <iterator>

template <typename Key, typename Value>
class Fringe
{
public:
	typedef std::pair<Key, Value> value_type;
	typedef std::vector<value_type> TBuffer;

	typedef typename TBuffer::iterator       iterator;
	typedef typename TBuffer::const_iterator const_iterator;

	explicit Fringe(size_t desiredSize) { m_sorted.reserve(desiredSize); m_aux.reserve(desiredSize); };

	const_iterator begin() const { return m_sorted.begin(); }
	const_iterator end() const   { return m_sorted.end(); }
	iterator       begin()       { return m_sorted.begin(); }
	iterator       end()         { return m_sorted.end(); }

	size_t size() const { return m_sorted.size(); }
	bool empty()  const { return m_sorted.empty();  }

	iterator find(const Key& k)
	{
		auto equalRange = std::equal_range(m_sorted.begin(), m_sorted.end(), SearchHelper(k), [](const SearchHelper& a, const SearchHelper& b) { return a > b; });
		auto start = equalRange.first;
		if (start != end() && start->first == k)
			return start;

		return end();
	}

	void insert(const value_type& v) 
	{ 
		value_type arr[] = { v };
		m_aux.resize(m_sorted.size() + 1);

		std::merge(m_sorted.begin(), m_sorted.end(), std::begin(arr), std::end(arr), m_aux.begin(), [](const SearchHelper& a, const SearchHelper& b) {return a > b; });
		m_sorted.swap(m_aux);
	}

	void erase(const_iterator where)
	{
		if (where == end())
			return;

		m_aux.resize(m_sorted.size() - 1);

		size_t index = where - begin();
		std::copy_n(begin(), index, m_aux.begin());
		std::copy(begin() + index + 1, end(), m_aux.begin() + index);

		m_sorted.swap(m_aux);
	}	

	value_type pop()
	{
		assert(!empty() && "attempt to pop empty fringe");

		auto value = std::move(m_sorted.back());
		m_sorted.pop_back();
		return std::move(value);
	}

private:
	struct SearchHelper 
	{ 
		const Key& m_key;

		SearchHelper(const Key& key) : m_key(key) {}
		SearchHelper(const typename Fringe::value_type& fringeValue) : m_key(fringeValue.first) {}

		bool operator==(const SearchHelper& other) const { return m_key == other.m_key; }
		bool operator> (const SearchHelper& other) const { return m_key > other.m_key; }

	};

	Fringe(const Fringe&)            = delete;
	Fringe& operator=(const Fringe&) = delete;

	TBuffer m_sorted;
	TBuffer m_aux;
};

#include <cassert>
inline void testFringe()
{
	Fringe<int, double> f(10);

	auto i = f.find(1);
	assert(i == f.end());

	f.insert(std::make_pair(4, 4.0));
	i = f.find(4);
	assert(i == f.begin());
	assert(i != f.end());

	f.insert(std::make_pair(1, 1.0));
	i = f.find(1);
	assert(i != f.begin());
	assert(i != f.end());

	assert(f.find(5) == f.end());
	assert(f.find(2) == f.end());
	assert(f.find(0) == f.end());

	i = f.find(1);
	f.erase(i);
	i = f.find(1);
	assert(i == f.end());

	i = f.find(4);
	f.erase(i);
	i = f.find(4);
	assert(i == f.end());

	f.insert(std::make_pair(5, 5.0));
	f.insert(std::make_pair(4, 4.0));
	f.insert(std::make_pair(6, 6.0));
	f.insert(std::make_pair(5, 5.0));
	f.insert(std::make_pair(4, 4.0));
	f.insert(std::make_pair(6, 6.0));

	assert(f.pop().second == 4.0);
	assert(f.pop().second == 4.0);
	assert(f.pop().second == 5.0);
	assert(f.pop().second == 5.0);
	assert(f.pop().second == 6.0);
	assert(f.pop().second == 6.0);

	assert(f.empty());
} 