#pragma once

#include "collide.hpp"

T2D_NAMESPACE_BEGIN

template<class TileData>
class World {
public:
	using PhConverter = phtree::ConverterBoxFloatIEEE<2>;
	using TileMapT = ::t2d::TileMap<TileData>;
	using TileBodyT = ::t2d::TileBody<TileData>;

	World(size_t threadCount)
		: ioServiceWork(ioService)  {
		threadCount = std::min(threadCount, (size_t)boost::thread::hardware_concurrency());

		for (size_t i = 0; i < threadCount; i++) {
			threadPool.create_thread(boost::bind(&boost::asio::io_service::run, &ioService));
		}
	}

	~World() {
		ioService.stop();
		threadPool.join_all();
	}

	Body* createTileBody(TileMapT& tileMap) {
		uint32_t newId = m_lastId++;

		BodyElement bodyElement;
		bodyElement.body = m_tileBodyPool.construct<TileMapT&, uint32_t>(tileMap, newId);
		if (!bodyElement.body)
			return nullptr;
		bodyElement.currentKey = bodyElement.body->getAABB().phbox();

		m_bodies.insert({ newId, bodyElement });

		return bodyElement.body;
	}

	void insertIntoTree(Body* body) {
		BodyElement& bodyElement = m_bodies[body->id()];
		bodyElement.currentKey = body->getAABB().phbox();
		m_phtree.insert(bodyElement.currentKey, SpatialIndex{ body->id() });
	}

	void eraseFromTree(Body* body) {
		BodyElement& bodyElement = m_bodies[body->id()];
		m_phtree.erase(bodyElement.currentKey);
	}

	void destroyBody(Body* body) {
		switch (body->m_type) {
		case BodyType::Tile: {
			m_tileBodyPool.destroy(dynamic_cast<TileBody*>(body));
		} break;

		default:
			assert(false);
		}

		m_bodies.erase(body->m_id);
	}

	void update(const Float dt, size_t steps, Float minThreshold) {
		auto start = std::chrono::steady_clock::now();

		if (dt > minThreshold) {
			steps = 2;
		}

		Float timePerStep = dt / (Float)steps;

		for (size_t i = 0; i < steps; i++) {
			jobsDone = 0;
			size_t threadPoolSize = threadPool.size(); // .size() uses a lock_gaurd, so best to save it in a var
			size_t bodiesPerThread = m_bodies.size() / threadPoolSize;
			for (size_t i = 0; i < threadPoolSize; i++) {
				size_t bodyStart = i * bodiesPerThread;
				size_t bodyEnd = bodyStart + bodiesPerThread;
				if (i + 1 == threadPool.size()) {
					bodyEnd = m_bodies.size();
				}

				ioService.post(boost::bind(&World::updateBodies, this, timePerStep, bodyStart, bodyEnd));
			}
			while (jobsDone < threadPoolSize) {}

			m_writeNeedsReinsertLock.lock();
			for (auto& i : m_needsReinsert) {
				BodyElement& bodyElement = m_bodies[i.first];
				
				bool res = m_phtree.relocate(bodyElement.currentKey, i.second);
				bodyElement.currentKey = i.second;
			}
			m_needsReinsert.clear();
			m_writeNeedsReinsertLock.unlock();

			for (auto& pair : m_bodies) {
				Body* body = pair.second.body;
				phtree::PhBoxF<2>& box = pair.second.currentKey;

				m_phtree.for_each([&](phtree::PhBoxF<2> key, SpatialIndex& spatialIndex){
					if (spatialIndex.id == body->id())
						return;

					Body* otherBody = m_bodies[spatialIndex.id].body;

					switch (body->m_bodyType) {
					case BodyType::Tile: {
						switch (otherBody->m_bodyType) {
						case BodyType::Tile:
							detectAndResolveTileBodyCollision<TileData>(dynamic_cast<TileBodyT&>(*body), dynamic_cast<TileBodyT&>(*otherBody), m_cache.detectionCache);
							break;

						default:
							assert(false);
							break;
						}
					} break;

					default:
						assert(false);
					}
				}, phtree::FilterBoxAABB(box.min(), box.max(), m_phtree.converter()));
			}
		}

		auto end = std::chrono::steady_clock::now();

		m_executionTimeAvgTotal += std::chrono::duration_cast<std::chrono::duration<Float, std::milli>>(end - start).count();
		m_executionTimeAvgCount++;
	}

	boost::container::flat_map<uint32_t, Body*> bodies() {
		return m_bodies;
	}

	Float getExecutionTimeAvg() {
		Float avg = m_executionTimeAvgTotal / m_executionTimeAvgCount;

		m_executionTimeAvgTotal = 0;
		m_executionTimeAvgCount = 0;
		return avg;
	}

public:
	void updateBodies(Float timePerStep, uint32_t start, uint32_t end) {
		auto itStart = m_bodies.begin() + start;
		auto itEnd = m_bodies.begin() + end;
		std::vector<std::pair<uint32_t, phtree::PhBoxF<2>>> needsReinsert;

		needsReinsert.reserve(end - start);
		for (auto i = itStart; i < itEnd; i++) {
			Body* body = i->second.body;

			body->integrate(timePerStep);

			auto phbox = body->getAABB().phbox();
			if (phbox != i->second.currentKey) {
				needsReinsert.emplace_back(std::pair(body->id(), phbox));
			}
		}

		m_writeNeedsReinsertLock.lock();
		m_needsReinsert.insert(m_needsReinsert.begin(), needsReinsert.begin(), needsReinsert.end());
		m_writeNeedsReinsertLock.unlock();

		jobsDone++;
	}

private:
	struct SpatialIndex {
		uint32_t id = 0;

		bool operator==(const SpatialIndex& other) const {
			return id == other.id;
		}
	};

	struct SpatialIndexGetter {
		const Float* min(const SpatialIndex& value) const { return value.aabb.getMinArr(); }
		const Float* max(const SpatialIndex& value) const { return value.aabb.getMaxArr(); }
	};

	struct Cache {
		std::vector<SpatialIndex> results;
		::t2d::TileBodyCache<TileData> detectionCache;
	} m_cache;

	struct BodyElement {
		phtree::PhBoxF<2> currentKey;
		Body* body;
	};

	uint32_t m_lastId = 0;
	boost::container::flat_map<uint32_t, BodyElement> m_bodies;
	boost::object_pool<TileBodyT> m_tileBodyPool;
	boost::mutex m_writeNeedsReinsertLock;
	std::vector<std::pair<uint32_t, phtree::PhBoxF<2>>> m_needsReinsert;
	phtree::PhTreeBoxF<2, SpatialIndex> m_phtree;

	Float m_executionTimeAvgCount = 0;
	Float m_executionTimeAvgTotal = 0;

	boost::asio::io_service ioService;
	boost::asio::io_service::work ioServiceWork;
	boost::thread_group threadPool;
	std::atomic<size_t> jobsDone;
};

T2D_NAMESPACE_END