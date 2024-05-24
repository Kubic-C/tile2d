#pragma once

#include "collide.hpp"
#include "spatial.hpp"

/**
 * @file world.hpp
 *
 * @brief Head include file
 */

T2D_NAMESPACE_BEGIN

template<typename Int>
struct LinkedListElement {
	static constexpr Int invalid = std::numeric_limits<Int>::max();
public:
	LinkedListElement()
		: prev(invalid), next(invalid) {}

	virtual ~LinkedListElement() = default;

	Int prev;
	Int next;
};

template<typename T, typename Access, typename Int>
class LinkedListHeader {
	static constexpr Int invalid = std::numeric_limits<Int>::max();
public:
	LinkedListHeader(Access& access)
		: first(invalid), last(invalid), m_access(access), m_size(0) {}

	bool is_empty() { return first == invalid; }

	void push_front(Int element);
	void push_back(Int element);
	void remove_element(Int element);
	void pop_front();
	void pop_back();

	size_t size() const {
		return m_size;
	}

public:
	struct Iterator {
	private:
		Access& m_access;
		Int m_idx;
	public:
		Iterator(Access& access, Int it)
			: m_access(access), m_idx(it) {}

		T& operator*() {
			return m_access[m_idx];
		}

		T* operator->() {
			return &m_access[m_idx];
		}

		Iterator& operator++() {
			m_idx = m_access[m_idx].next;
			return *this;
		}

		Iterator& operator--() {
			m_idx = m_access[m_idx].prev;
			return *this;
		}

		bool operator!=(const Iterator& other) const {
			return m_idx != other.m_idx;
		}
	};

	Iterator begin() { return Iterator(m_access, first); }
	Iterator end() { return Iterator(m_access, invalid); }

protected:
	void assign_first(Int element);
	
	size_t m_size;
	Int first = invalid;
	Int last = invalid;
	Access& m_access;
};

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::assign_first(Int element) {
	first = element;
	last = element;
}

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::push_front(Int element) {
	LinkedListElement<Int>* list_element = dynamic_cast<LinkedListElement<Int>*>(&m_access[element]);

	if (is_empty()) {
		assign_first(element);
	}
	else {
		m_access[first].prev = element;
		list_element->next = first;
		first = element;
	}

	m_size++;
}

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::push_back(Int element) {
	LinkedListElement<Int>* list_element = dynamic_cast<LinkedListElement<Int>*>(&m_access[element]);

	if (is_empty()) {
		assign_first(element);
	}
	else {
		m_access[last].next = element;
		list_element->prev = last;
		last = element;
	}

	m_size++;
}

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::remove_element(Int element) {
	LinkedListElement<Int>* list_element = dynamic_cast<LinkedListElement<Int>*>(&m_access[element]);

	if (element == first) {
		first = list_element->next;
	}
	if (element == last) {
		last = list_element->prev;
	}

	if (list_element->prev != invalid) {
		m_access[list_element->prev].next = list_element->next;
	}
	if (list_element->next != invalid) {
		m_access[list_element->next].prev = list_element->prev;
	}

	list_element->prev = invalid;
	list_element->next = invalid;

	m_size--;
}

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::pop_front() {
	if (is_empty()) {
		return;
	}
	else {
		first = access[first].next;

		access[first].prev = invalid;
		access[first].next = invalid;
	}

	m_size--;
}

template<typename T, typename Access, typename Int>
void LinkedListHeader<T, Access, Int>::pop_back() {
	if (is_empty()) {
		return;
	}
	else {
		last = access[last].prev;

		access[last].prev = invalid;
		access[last].next = invalid;
	}

	m_size--;
}

/**
 * @class World
 * 
 * @brief Allows collisions between different bodies to occur. 
 */
template<class TileData>
class World {
protected:
	struct SpatialIndex {
		uint32_t id = 0;

		bool operator==(const SpatialIndex& other) const {
			return id == other.id;
		}
	};

	struct Cache {
		Cache() {}

		std::vector<SpatialIndex> results;
		::t2d::TileBodyCollisionCache<TileData> detectionCache;

		std::vector<std::pair<uint16_t, AABB<Float>>> shouldMove;
		std::vector<std::pair<uint32_t, uint32_t>> couldCollide;
	};

public:
	struct BodyElement : public LinkedListElement<uint32_t> {
		uint16_t element = std::numeric_limits<uint16_t>::max();
		WorldBody* body = nullptr;
	};

	using TileMapT = TileMap<TileData>;
	using TileBodyT = TileBody<TileData>;

	using BodyList = LinkedListHeader<BodyElement, FreeList<BodyElement, uint32_t>, uint32_t>;
public:
	/**
	 * @brief Constructs a physics world
	 * 
	 * @param threadCount The amount of threads to create to allow for additional speed up, may be 0.
	 */
	World(size_t threadCount)
		: m_bodyList(m_bodies), m_grid(tileWidth * 20), ioServiceWork(ioService) {
		threadCount = std::min(threadCount, (size_t)boost::thread::hardware_concurrency());

		std::pair<boost::thread::id, Cache> pair;
		pair.first = boost::this_thread::get_id();
		m_threadCache.insert(pair);

		for (size_t i = 0; i < threadCount; i++) {
			boost::thread* thread = threadPool.create_thread(boost::bind(&boost::asio::io_service::run, &ioService));

			std::pair<boost::thread::id, Cache> pair;
			pair.first = thread->get_id();
			m_threadCache.insert(pair);
		}
	}

	~World() {
		ioService.stop();
		threadPool.join_all();
	}

	/**
	 * @brief Do not use. See createTileMap()
	 */
	WorldBody* createTileBody(Transform& srcTransform, TileMapT& tileMap) {
		uint32_t newId = m_bodies.insert();

		BodyElement& bodyElement = m_bodies[newId];
		bodyElement.body = m_tileBodyPool.template construct<Transform&, TileMapT&, uint32_t>(srcTransform, tileMap, newId);
		if (!bodyElement.body)
			return nullptr;

		m_bodyList.push_back(bodyElement.body->m_id);

		insertIntoTree(bodyElement.body);

		return bodyElement.body;
	}

	/**
	 * @brief NOT YET IMPLEMENTED. Creates a bullet body.
	 * 
	 * @param srcTransform A transform of some external object. Will be manipulated as the simulation runs
	 * @param radius The radius of the bullet
	 * @param initialLinearVelocity The initial linear velocity of the body
	 * 
	 * @return The new bullet body
	 */
	WorldBody* createBulletBody(Transform& srcTransform, Float radius, const vec2& initialLinearVelocity) {
		return nullptr; // to be implemented

		uint32_t newId = m_bodies.insert();

		BodyElement& bodyElement = m_bodies[newId];
		bodyElement.body = m_bulletBodyPool.template construct<uint32_t, Float>(srcTransform, newId, radius);
		if (!bodyElement.body)
			return nullptr;

		m_bodyList.push_back(bodyElement.body->m_id);

		bodyElement.body->addLinearVelocity(initialLinearVelocity);

		insertIntoTree(bodyElement.body);

		return bodyElement.body;
	}

	/**
	 * @brief Destroys a body, must be a type of a bullet or a tile body.
	 * 
	 * @param body The body to destroy
	 */
	void destroyBody(WorldBody* body) {
		switch (body->m_bodyType) {
		case BodyType::Tile:
			m_tileBodyPool.destroy(dynamic_cast<TileBody<TileData>*>(body));
			break;

		case BodyType::Bullet:
			m_bulletBodyPool.destroy(dynamic_cast<BulletBody*>(body)); 
			break;

		default:
			assert(false);
		}

		eraseFromTree(body);
		m_bodyList.remove_element(body->m_id);
		m_bodies.erase(body->m_id);
	}

	/**
	 * @brief Updates the body by @p dt
	 *
	 * @param dt The amount of time to simulate
	 * @param steps Essentialy, the quality of the simulation. Higher steps means higher quality but is slower to process. Recommended amount is 6 if its called 60 times a second
	 */
	void update(const Float dt, size_t steps) {
		auto start = std::chrono::steady_clock::now();

		Float timePerStep = dt / (Float)steps;

		for (size_t i = 0; i < steps; i++) {
			/* Update bodies and relocate into grid if necessary*/
			executeOnBodies(timePerStep, &World::updateBodies);

			m_grid.cleanup();

			/* Perform grid queries */
			executeOnBodies(timePerStep, &World::batchQuery);

			std::vector<CollisionManifold> manifolds;
			Cache& mainThreadCache = m_threadCache[boost::this_thread::get_id()];
			for (const std::pair<uint32_t, uint32_t>& collidingPair : m_possibleCollisions) {
				WorldBody* body = m_bodies[collidingPair.first].body;
				WorldBody* otherBody = m_bodies[collidingPair.second].body;

				vec2 body1Offset(0.0);
				vec2 body2Offset(0.0);

				manifolds.clear();

				switch (body->m_bodyType) {
				case BodyType::Tile: {
					switch (otherBody->m_bodyType) {
					case BodyType::Tile:
						detectTileBodyCollision<TileData>(dynamic_cast<TileBodyT&>(*body), dynamic_cast<TileBodyT&>(*otherBody), mainThreadCache.detectionCache, manifolds, body1Offset, body2Offset);
						break;

					default:
						assert(false);
						break;
					}
				} break;

				default:
					assert(false);
				}

				for (CollisionManifold& manifold : manifolds) {
					ImpulseMethod()(*(Body*)body, *(Body*)otherBody, manifold);
				}

				body->moveBy(body1Offset);
				otherBody->moveBy(body2Offset);
			}
			m_possibleCollisions.clear();
		
		}

		auto end = std::chrono::steady_clock::now();

		m_executionTimeAvgTotal += std::chrono::duration_cast<std::chrono::duration<Float, std::milli>>(end - start).count();
		m_executionTimeAvgCount++;
	}

	/**
	 * @brief Returns the average speed of update. The stats will reset after call.
	 * 
	 * @return The average time it takes for update() to proccess
	 */
	Float getExecutionTimeAvg() {
		Float avg = m_executionTimeAvgTotal / m_executionTimeAvgCount;

		m_executionTimeAvgTotal = 0;
		m_executionTimeAvgCount = 0;
		return avg;
	}

	/**
	 * @brief Returns the gravity of the world
	 *
	 * @return The gravity of the world
	 */
	vec2 gravity() const {
		return m_worldForces.gravity;
	}

	/**
	 * @brief Set the gravity of the world
	 *
	 * @param gravity The new gravity
	 */
	void setGravity(const vec2& gravity) {
		m_worldForces.gravity = gravity;
	}

	/**
	 * @brief Returns a grid refence, may be used for queries or insertions.
	 * 
	 * @return The world's grid
	 */
	Grid<SpatialIndex>& grid() {
		return m_grid;
	}

public:
	using Iterator = typename BodyList::Iterator;
	
	Iterator begin() { return m_bodyList.begin(); }
	Iterator end() { return m_bodyList.end(); }

protected:
	template<class Func>
	void executeOnBodies(Float timePerStep, Func&& func) {
		jobsDone = 0;
		size_t threadPoolSize = threadPool.size() + 1; // +1 includes main thread
		size_t bodiesPerThread = m_bodyList.size() / threadPoolSize;
		for (size_t i = 0; i < threadPoolSize; i++) {
			size_t bodyStart = i * bodiesPerThread;
			size_t bodyEnd = bodyStart + bodiesPerThread;
			/* Once all threads have been assigned the task,
			   give the main thread the task as well */
			if (i + 1 == threadPoolSize) {
				bodyEnd = m_bodyList.size();
				(this->*func)(timePerStep, bodyStart, bodyEnd);
			} else {
				ioService.post(boost::bind(func, this, timePerStep, bodyStart, bodyEnd));
			}
		}
		while (jobsDone < threadPoolSize) {}
	}

	void updateBodies(Float timePerStep, size_t start, size_t end) {
		Cache& cache = m_threadCache.find(boost::this_thread::get_id())->second;
		cache.shouldMove.clear();

		typename BodyList::Iterator i = m_bodyList.begin();
		for (size_t index = 0; index < end; index++, ++i) {
			if (index < start)
				continue;

			WorldBody* body = i->body;

			body->addLinearVel(m_worldForces.gravity * timePerStep);
			body->integrate(timePerStep);

			if (body->m_flags[Body::NEEDS_REINSERT]) {
				cache.shouldMove.emplace_back(std::pair<uint16_t, AABB<Float>>(i->element, body->getAABB()));
				body->m_flags[Body::NEEDS_REINSERT] = false;
			}
		}

		m_gridLock.lock();
		for (auto& movePair : cache.shouldMove) {
			m_grid.relocate(movePair.first, movePair.second);
		}
		m_gridLock.unlock();

		jobsDone++;
	}

	void batchQuery(Float timePerStep, size_t start, size_t end) {
		Cache& cache = m_threadCache.find(boost::this_thread::get_id())->second;
		cache.couldCollide.clear();

		typename BodyList::Iterator i = m_bodyList.begin();
		for (size_t index = 0; index < end; index++, ++i) {
			if (index < start)
				continue;

			WorldBody* body = i->body;
			AABB<Float> bodyAABB = body->getAABB();
			m_grid.queryIntersects(bodyAABB, [&](const SpatialIndex& spatialIndex) {
				if (spatialIndex.id == body->id())
					return;
				if (body->isStatic() && m_bodies[spatialIndex.id].body->isStatic())
					return;

				cache.couldCollide.push_back(std::pair(body->id(), spatialIndex.id));
			});
		}

		m_collisionListLock.lock();
		m_possibleCollisions.insert(m_possibleCollisions.end(), cache.couldCollide.begin(), cache.couldCollide.end());
		m_collisionListLock.unlock();

		jobsDone++;
	}

	void insertIntoTree(WorldBody* body) {
		BodyElement& bodyElement = m_bodies[body->id()];
		bodyElement.element = m_grid.insert(body->getAABB(), SpatialIndex{ body->id() });
	}

	void eraseFromTree(WorldBody* body) {
		BodyElement& bodyElement = m_bodies[body->id()];
		m_grid.erase(bodyElement.element);
	}

private:
	FlatMap<boost::thread::id, Cache> m_threadCache;

	BodyList m_bodyList;
	FreeList<BodyElement, uint32_t> m_bodies;
	boost::object_pool<TileBodyT> m_tileBodyPool;
	boost::object_pool<BulletBody> m_bulletBodyPool;

	boost::mutex m_gridLock;
	Grid<SpatialIndex> m_grid;
	boost::mutex m_collisionListLock;
	std::vector<std::pair<uint32_t, uint32_t>> m_possibleCollisions;

	Float m_executionTimeAvgCount = 0;
	Float m_executionTimeAvgTotal = 0;

	boost::asio::io_service ioService;
	boost::asio::io_service::work ioServiceWork;
	boost::thread_group threadPool;
	std::atomic<size_t> jobsDone;

	struct {
		vec2 gravity = { 0.0f, 0.0f };
	} m_worldForces;
};

T2D_NAMESPACE_END