#pragma once

#include "math.hpp"

/**
 * @file grid.hpp
 *
 * @brief Contains a grid that is used for spatial indexing
 */

T2D_NAMESPACE_BEGIN

template<typename Type, typename Int>
class FreeList {
public:
	static constexpr Int endOfList = std::numeric_limits<Int>::max();
	static constexpr size_t elementSize = sizeof(Type);

public:
	FreeList()
		: firstFree(endOfList) {}

	Int insert() {
		if (m_elements.size() + 1 >= endOfList)
			return endOfList;

		if (firstFree == endOfList) {
			m_elements.emplace_back();
			firstFree = m_elements.size() - 1;
			m_elements[firstFree].next = endOfList;
		}

		Int idx = firstFree;
		Node& node = m_elements[idx]; 
		firstFree = node.next;
		node.emplace();

		return idx;
	}

	Int insert(Type&& obj) {
		if (firstFree == endOfList) {
			m_elements.emplace_back();
			firstFree = m_elements.size() - 1;
			m_elements[firstFree].next = endOfList;
		}

		Int idx = firstFree;
		Node& node = m_elements[idx];
		firstFree = node.next;
		node.emplace(std::forward<Type&&>(obj));

		return idx;
	}

	void erase(Int idx) {
		Node& node = m_elements[idx];

		node.destroy();

		if (firstFree == endOfList) {
			node.next = endOfList;
		} else {
			node.next = firstFree;
		}

		firstFree = idx;
	}

	Type& get(Int idx) {
		return *(Type*)m_elements[idx].element.data.data();
	}

	const Type& get(Int idx) const {
		return *(Type*)m_elements[idx].element.data.data();
	}

	Type& operator[](Int idx) {
		return get(idx);
	}

	const Type& operator[](Int idx) const {
		return get(idx);
	}

private:
	union Node {
		Int next;

		struct {
			std::array<uint8_t, elementSize> data;
		} element;

		void emplace(Type&& obj) {
			new(element.data.data())Type(obj);
		}

		void emplace() {
			new(element.data.data())Type();
		}

		void destroy() noexcept {
			Type* obj = (Type*)element.data.data();
			obj->~Type();
		}
	};

	Int firstFree;
	std::vector<Node> m_elements;
};

inline const boost::thread::id mainThreadId = boost::this_thread::get_id();

/**
 * @class Grid
 * 
 * @brief Used for spatial indexing
 */
template<class T, typename Int = int32_t>
class Grid {
	using ivec2 = typename glm::vec<2, Int, glm::packed_lowp>;
protected:
	struct GridElement {
		AABB<Float> aabb;
		T data;
	};
public:
	struct ParentCell {
		void insert(uint16_t idx) {
			elements.push_back(idx);
		}

		void erase(uint16_t idx) {
			elements.erase(std::find(elements.begin(), elements.end(), idx));
		}

		std::vector<uint16_t> elements;
	};

	using GridMap = boost::unordered_flat_map<ivec2, ParentCell>;
public:
	/**
	 * @brief Constructs a new Grid
	 * 
	 * @parram cellSize the side length of a cell within this grid
	 */
	Grid(Float cellSize = (Float)tcW)
		: m_cellSize(cellSize), m_invCellSize(1.0f / cellSize) {
	}

	/**
	 * @brief Inserts a new element into the grid
	 * 
	 * @param aabb The AABB of the element
	 * @param data The user data of the element
	 * 
	 * @return The id of the element
	 */
	inline uint16_t insert(const AABB<Float>& aabb, const T& data) {
		uint16_t idx = m_elementList.insert();
		GridElement& element = m_elementList.get(idx);
		element.aabb = aabb;
		element.data = data;

		const ivec2& min = getCellCoords(aabb.min());
		const ivec2& max = getCellCoords(aabb.max());

		for (auto y = min.y; y <= max.y; y++) {
			for (auto x = min.x; x <= max.x; x++) {
				m_cells[{x, y}].insert(idx);
			}
		}

		return idx;
	}


	/**
	 * @brief Relocates an already existing element within the grid.
	 *
	 * @param idx The id of the element
	 * @param newAABB The new AABB of the element
	 */
	inline void relocate(uint16_t idx, const AABB<Float>& newAABB) {
		GridElement& element = m_elementList.get(idx);

		const ivec2& oldMin = getCellCoords(element.aabb.min());
		const ivec2& oldMax = getCellCoords(element.aabb.max());
		ivec2 newMin = getCellCoords(newAABB.min());
		ivec2 newMax = getCellCoords(newAABB.max());

		for (auto y = oldMin.y; y <= oldMax.y; y++) {
			for (auto x = oldMin.x; x <= oldMax.x; x++) {
				if (contains(newMin, newMax, { x, y }))
					continue;

				auto iter = m_cells.find({ x, y });
				iter->second.erase(idx);
				if (iter->second.elements.size() == 0)
					m_possiblyEmptyCells.push_back({ x, y });
			}
		}

		for (auto y = newMin.y; y <= newMax.y; y++) {
			for (auto x = newMin.x; x <= newMax.x; x++) {
				if (contains(oldMin, oldMax, { x, y }))
					continue;

				m_cells[{x, y}].insert(idx);
			}
		}

		element.aabb = newAABB;
	}

	/**
	 * @brief Erases an element
	 *
	 * @param idx The id of the element to remove
	 */
	inline void erase(uint16_t idx) {
		GridElement& element = m_elementList.get(idx);
		const ivec2& min = getCellCoords(element.aabb.min());
		const ivec2& max = getCellCoords(element.aabb.max());

		for (auto y = min.y; y <= max.y; y++) {
			for (auto x = min.x; x <= max.x; x++) {
				m_cells[{x, y}].erase(idx);
			}
		}

		m_elementList.erase(idx);
	}

	template<typename Callback>
	void queryIntersects(const AABB<Float>& area, Callback&& clbk) const {
		const ivec2 min = getCellCoords(area.min());
		const ivec2 max = getCellCoords(area.max());
		
		for (auto y = min.y; y <= max.y; y++) {
			for (auto x = min.x; x <= max.x; x++) {
				ivec2 coord(x, y);

				auto iter = m_cells.find(coord);
				if (iter == m_cells.end())
					continue;

				for (uint16_t idx : iter->second.elements) {
					const GridElement& element = m_elementList.get(idx);

					if (element.aabb.intersects(area)) {
						clbk(element.data);
					}
				}
			}
		}
	}

	/**
	 * @brief Removes empty cells, if any.
	 */
	void cleanup() {
		for (const ivec2& coord : m_possiblyEmptyCells) {
			if (m_cells[coord].elements.size() == 0) {
				m_cells.erase(coord);
			}
		}

		m_possiblyEmptyCells.clear();
	}

	/**
	 * @brief Returns the map of all cells
	 * 
	 * @return the map of all cells
	 */
	GridMap& gridMap() {
		return m_cells;
	}

protected:
	inline ivec2 getCellCoords(const vec2& coords) const {
		ivec2 icoords;
		icoords.x = floor<Int>(coords.x * m_invCellSize);
		icoords.y = floor<Int>(coords.y * m_invCellSize);

		return  icoords;
	}

	bool contains(const Int& min, const Int& max, const Int& point) {
		return min <= point && point <= max;
	}

	bool contains(const ivec2& min, const ivec2& max, const ivec2& point) {
		return contains(min.x, max.x, point.x) && contains(min.y, max.y, point.y);
	}

protected:
	Float m_invCellSize;
	Float m_cellSize;
	FreeList<GridElement, uint16_t> m_elementList;

	GridMap m_cells;
	std::vector<ivec2> m_possiblyEmptyCells;
};

T2D_NAMESPACE_END
