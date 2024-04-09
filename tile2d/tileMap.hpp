#pragma once

#include "body.hpp"

#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/function.hpp>

T2D_NAMESPACE_BEGIN

static constexpr Float tileWidth = 5.0f;
static constexpr Float tileHeight = 5.0f;
static constexpr vec2 tileSize = vec2(tileWidth, tileHeight);
static constexpr Float tileRadi = sqrt<Float>(tileWidth * tileWidth + tileHeight * tileHeight) * 0.5f;

template<typename T>
struct TileProperties {

	bool isMultiTile = false;
	bool isMainTile = false;
	union {

		// will be accessed when isMainTile is true
		struct {
			uint16_t width;
			uint16_t height;
		} mutliTile;

		// will be accessed when isMainTile is false but isMultiTile is true
		glm::i32vec2 mainTilePos;
	};

	T userData;
};

struct PhysicsTileProperties {
	// returns moment of inertia as a scalar
	template<typename Integer>
	Float getMomentOfInertia(const glm::vec<2, Integer>& tileDimensions) const {
		// MOI scalar of a rectangle
		// I = (1 / 12) * m(w * w + h * h)

		constexpr Float one_twelfth = 1.0f / 12.0f;

		return one_twelfth * getMass(tileDimensions) * (sqaure(tileWidth * tileDimensions.x) + sqaure(tileHeight * tileDimensions.y));
	}

	// returns the mass of this tile
	template<typename Integer>
	Float getMass(const glm::vec<2, Integer>& tileDimensions) const {
		// area = width * height
		// mass = area * density
		return (tileWidth * (Float)tileDimensions.x) * (tileHeight * (Float)tileDimensions.y) * density;
	}

	// Must be set
	Float density = 1.0f;
	uint8_t staticFriction = 255; // [0 - 255] -> [0.0 = 1.0]
	uint8_t dynamicFriction = 255; // [0 - 255] -> [0.0 = 1.0]
	uint8_t restitution = 0;// [0 - 255] -> [0.0 = 1.0]
};

struct _CreateTileMap {};

template<typename TileType>
class TileBody;

template<typename TileType>
class TileMap {
	friend class TileBody<TileType>;
	template<class TileType, class TileMapAllocator, class PhysicsWorld>
	friend class CreateTileMap;

protected:
	TileMap();

public:
	TileMap(_CreateTileMap) {}
	virtual ~TileMap() {}

	inline const PhysicsTileProperties& getPhysicsTileProperties(const glm::i32vec2& tilePos) const {
		return m_physicsTiles.find(tilePos)->second;
	}

	inline TileProperties<TileType>& getTileProperties(const glm::i32vec2& tilePos) {
		return m_tiles.find(tilePos)->second;
	}

	void beginBulkInsert();
	void endBulkInsert();
	void beginBulkErase();
	void endBulkErase();

	bool addTile(const glm::i32vec2& pos, const TileProperties<TileType>& props, const PhysicsTileProperties& physicsProperties = {});
	void removeTile(const glm::i32vec2& pos) noexcept;

	inline glm::u16vec2 getTileDimensions(const glm::i32vec2& pos) const {
		auto& props = m_tiles.find(pos)->second;

		if (props.isMultiTile) {
			return { props.mutliTile.width, props.mutliTile.height };
		}

		return { 1, 1 };
	}

	inline vec2 getRealTileDimensions(const glm::i32vec2& pos) const {
		return (vec2)getTileDimensions(pos) * tileSize;
	}

	size_t count() const {
		return m_tiles.size();
	}

	TileBody<TileType>& body();

public:
	struct Iterator {
		using ContainerIterator = typename boost::container::flat_map<glm::i32vec2, TileProperties<TileType>>::iterator;
	private:
		ContainerIterator m_it;
	public:
		Iterator(const ContainerIterator& it)
			: m_it(it) {}

		const glm::i32vec2& operator*() const {
			return m_it->first;
		}

		Iterator& operator++() {
			m_it = ++m_it;
			return *this;
		}

		Iterator& operator--() {
			m_it = --m_it;
			return *this;
		}

		bool operator!=(const Iterator& other) const {
			return m_it != other.m_it;
		}
	};

	Iterator begin() { return Iterator(m_tiles.begin()); }
	Iterator end() { return Iterator(m_tiles.end()); }

protected:
	void setTileBody(TileBody<TileType>* body) {
		m_body = body;
	}

protected:
	boost::container::flat_map<glm::i32vec2, TileProperties<TileType>> m_tiles;
	boost::container::flat_map<glm::i32vec2, PhysicsTileProperties> m_physicsTiles;

private:
	TileBody<TileType>* m_body;
};

template<typename TileType>
class TileBody : public Body {
	friend class TileMap<TileType>;

	static constexpr size_t chunkWidth = 4;
	static constexpr size_t chunkHeight = 4;
	static constexpr Float tcW = (Float)tileWidth * (Float)chunkWidth;
	static constexpr Float tcH = (Float)tileHeight * (Float)chunkHeight;
public:
	struct SpatialIndex {
		SpatialIndex() = default;
		SpatialIndex(const glm::i32vec2& pos)
			: aabb(), pos(pos) {

		}
		SpatialIndex(const AABB<Float>& aabb, const glm::i32vec2& pos)
			: aabb(aabb), pos(pos) {

		}

		AABB<Float> aabb;
		glm::i32vec2 pos;

		inline bool operator==(const SpatialIndex& other) const {
			return pos == other.pos;
		}
	};

public:
	TileBody(TileMap<TileType>& tileMap, uint32_t id)
		: Body(id, BodyType::Tile), m_tileMap(tileMap) {
		clear();

		tileMap.setTileBody(this);
	}

	inline TileMap<TileType>& tileMap() const {
		return m_tileMap;
	}

	inline virtual void integrate(Float dt) override {
		Body::integrate(dt);
	}

	inline virtual AABB<Float> getAABB() override {
		return getAABB(Transform());
	}

	inline std::array<vec2, 2> normals() const {
		const SinCos& sincos = m_transform.sincos;

		// _1_2, 2_3
		return { vec2(sincos.cos, sincos.sin), vec2(-sincos.sin, sincos.cos) };
	}

	// get the local AABB of this object
	inline AABB<Float> getLocalAABB() const {
		const vec2 halfDim = {
			(Float)corners.width() * tileWidth * Float(0.5f) + tileWidth * Float(0.5f),
			(Float)corners.height() * tileHeight * Float(0.5f) + tileHeight * Float(0.5f)
		};
		return AABB(halfDim) + getTileLocalPointNoDim(corners.min()) + halfDim;
	}

	// get the local AABB (of the entire object) in the localSpace of spaceTransform
	inline AABB<Float> getAABB(const Transform& spaceTransform, const vec2& localOffset = { Float(0.5f), Float(0.5f) }) const {
		return getAABB(getLocalAABB(), spaceTransform, localOffset);
	}

	// get the some local AABB in the localSpace of spaceTransform
	inline AABB<Float> getAABB(const AABB<Float>& localAABB, const Transform& spaceTransform, const vec2& localOffset = { Float(0.5f), Float(0.5f) }) const {
		vec2 relPos = spaceTransform.getLocalPoint(getWorldPoint(localAABB.midpoint()), localOffset);
		return AABB(relPos, localAABB.width() * 0.5f, localAABB.height() * 0.5f).rotate(spaceTransform.sincos - m_transform.sincos);
	}

public: /* Tiles! */
	// get the local position of a tile
	inline vec2 getTileLocalPoint(const glm::i32vec2& iPos) const {
		return (vec2)iPos * vec2(tileWidth, tileHeight) + m_tileMap.getRealTileDimensions(iPos) * Float(0.5f);
	}

	// get the world position of a tile
	inline vec2 getTileWorldPoint(const glm::i32vec2& iPos) const {
		return getWorldPoint(getTileLocalPoint(iPos));
	}

	inline TOBB getTileWorldOBB(const glm::i32vec2& iPos) const {
		TOBB tileOBB;

		tileOBB.extent = m_tileMap.getRealTileDimensions(iPos) * Float(0.5);
		tileOBB.transform.pos = getTileWorldPoint(iPos);
		tileOBB.transform.rot = m_transform.rot;
		tileOBB.transform.sincos = m_transform.sincos;

		return tileOBB;
	}

	inline AABB<Float> getTileWorldAABB(const glm::i32vec2& iPos) const {
		vec2 tileDim = m_tileMap.getRealTileDimensions(iPos);

		return AABB(getTileWorldPoint(iPos), tileDim.x * Float(0.5), tileDim.y * Float(0.5)).rotate(m_transform.sincos);
	}

	inline AABB<Float> getTileLocalAABB(const glm::i32vec2& iPos) const {
		vec2 tileDim = m_tileMap.getRealTileDimensions(iPos);

		return { getTileLocalPoint(iPos), tileDim.x * Float(0.5), tileDim.y * Float(0.5) };
	}

public:
	template<typename OutIt>
	inline void queryChunks(const AABB<Float>& intersectBox, OutIt chunks) const {
		glm::i32vec2 minChunk = getChunk((glm::i32vec2)(glm::floor(intersectBox.min()) / tileSize));
		glm::i32vec2 maxChunk = getChunk((glm::i32vec2)(glm::ceil(intersectBox.max()) / tileSize));

		assert(minChunk.x <= maxChunk.x);
		assert(minChunk.y <= maxChunk.y);

		for (int32_t y = minChunk.y; y <= maxChunk.y; y++) {
			for (int32_t x = minChunk.x; x <= maxChunk.x; x++) {
				glm::i32vec2 coord(x, y);

				if (m_chunks.contains(coord)) {
					chunks++;
					*chunks = SpatialIndex(getChunkAABB(coord), coord);
				}
			}
		}
	}

	template<typename OutIt>
	inline void queryTiles(const AABB<Float>& intersectBox, OutIt tiles) const {
		glm::i32vec2 minTiles = (glm::i32vec2)(glm::floor(intersectBox.min()) / tileSize);
		glm::i32vec2 maxTiles = (glm::i32vec2)(glm::ceil(intersectBox.max()) / tileSize);

		assert(minTiles.x <= maxTiles.x);
		assert(minTiles.y <= maxTiles.y);

		for (int32_t y = minTiles.y; y <= maxTiles.y; y++) {
			for (int32_t x = minTiles.x; x <= maxTiles.x; x++) {
				glm::i32vec2 coord(x, y);

				if (m_tileMap.m_tiles.contains(coord)) {
					tiles++;
					if (m_tileMap.m_tiles[coord].isMultiTile && !m_tileMap.m_tiles[coord].isMainTile)
						*tiles = SpatialIndex(m_tileMap.m_tiles[coord].mainTilePos);
					else
						*tiles = SpatialIndex(coord);
				}
			}
		}
	}

	inline void insertIntoChunk(const glm::i32vec2& tile) {
		glm::i32vec2 chunk = getChunk(tile);

		if (m_chunks.find(chunk) == m_chunks.end()) {
			m_chunks.insert(chunk);
		}
	}

	inline AABB<Float> getChunkAABB(const glm::i32vec2& chunk) const {
		constexpr vec2 chunkDim((Float)(tileWidth * chunkWidth), (Float)(tileHeight * chunkHeight));

		return AABB((vec2)chunk * chunkDim + chunkDim * Float(0.5f) - tileSize * Float(0.5), chunkDim.x * Float(0.5), chunkDim.y * Float(0.5));
	}

	inline glm::i32vec2 getChunk(const glm::i32vec2& tile) const {
		glm::i32vec2 chunk = {
			tile.x / chunkWidth,
			tile.y / chunkHeight
		};

		return chunk;
	}

protected:
	// get the local position of a tile without accounting for the tiles actual dimensions
	inline vec2 getTileLocalPointNoDim(const glm::i32vec2& iPos) const {
		return (vec2)iPos * vec2(tileWidth, tileHeight);
	}

	inline void calculateRotationalInertia() {
		m_I = 0.0f;
		for (const auto& pair : m_tileMap.m_physicsTiles) {
			auto tileDimensions = m_tileMap.getTileDimensions(pair.first);
			m_I += parallelAxisTheorem(pair.second.getMomentOfInertia<uint16_t>(tileDimensions), pair.second.getMass(tileDimensions), glm::length2(getTileLocalPoint(pair.first) - m_centroid));
		}
		if (m_I != 0.0f)
			m_inverseI = 1.0f / m_I;
		else
			m_inverseI = 0.0f;
	}

	inline void calculateRotationalInertiaAndAABB() {
		m_I = 0.0f;
		clear();
		auto& min = corners.min();
		auto& max = corners.max();
		for (const auto& pair : m_tileMap.m_physicsTiles) {
			const glm::i32vec2& tilePos = pair.first;
			const glm::u16vec2 tileDimensions = m_tileMap.getTileDimensions(tilePos);

			if (tilePos.x < min.x) {
				min.x = tilePos.x;
			}
			if (tilePos.y < min.y) {
				min.y = tilePos.y;
			}
			if (tilePos.x + (tileDimensions.x - 1) > max.x) {
				max.x = tilePos.x + (tileDimensions.x - 1);
			}
			if (tilePos.y + (tileDimensions.y - 1) > max.y) {
				max.y = tilePos.y + (tileDimensions.y - 1);
			}

			m_I += parallelAxisTheorem(pair.second.getMomentOfInertia<uint16_t>(tileDimensions), pair.second.getMass(tileDimensions), glm::length2(getTileLocalPoint(tilePos) - m_centroid));
		}

		if (m_I != 0.0f)
			m_inverseI = 1.0f / m_I;
		else
			m_inverseI = 0.0f;
	}

protected: /* For use within the Tile Map Class*/
	void addTile(const glm::i32vec2& iPos) {
		const vec2 pos = getTileLocalPoint(iPos);
		const glm::u16vec2 tileDimensions = m_tileMap.getTileDimensions(iPos);
		const PhysicsTileProperties& physicsProperties = m_tileMap.m_physicsTiles.at(iPos);
		const Float tileMass = physicsProperties.getMass<uint16_t>(tileDimensions);

		m_unweightedCentroid += pos;
		m_centroid = m_unweightedCentroid / (Float)m_tileMap.m_physicsTiles.size();
		if (!isBulkAdd) {
			calculateRotationalInertia();
		}

		m_mass += tileMass;
		m_inverseMass = 1.0f / m_mass;
		m_unweightedCom += tileMass * pos;
		m_com = m_unweightedCom * m_inverseMass;

		for (auto y = iPos.y; y < (iPos.y + (int32_t)tileDimensions.y); y++) {
			for (auto x = iPos.x; x < (iPos.x + (int32_t)tileDimensions.x); x++) {
				insertIntoChunk({ x, y });
			}
		}

		auto& min = corners.min();
		auto& max = corners.max();
		if (iPos.x < min.x) {
			min.x = iPos.x;
		}
		if (iPos.y < min.y) {
			min.y = iPos.y;
		}
		if (iPos.x + (tileDimensions.x - 1) > max.x) {
			max.x = iPos.x + (tileDimensions.x - 1);
		}
		if (iPos.y + (tileDimensions.y - 1) > max.y) {
			max.y = iPos.y + (tileDimensions.y - 1);
		}

		assert(isValid());
	}

	void removeTile(const glm::i32vec2& iPos) {
		const vec2 pos = getTileLocalPoint(iPos);
		const glm::u16vec2 tileDimensions = m_tileMap.getTileDimensions(iPos);
		const PhysicsTileProperties& physicsProperties = m_tileMap.m_physicsTiles.at(iPos);
		const Float tileMass = physicsProperties.getMass<uint16_t>(tileDimensions);

		m_mass -= tileMass;
		if (m_mass != 0.0f)
			m_inverseMass = 1.0f / m_mass;
		else
			m_inverseMass = 0.0f;
		m_unweightedCom -= tileMass * pos;
		m_com = m_unweightedCom * m_inverseMass;

		m_unweightedCentroid -= pos;
		m_centroid = m_unweightedCentroid / (Float)(m_tileMap.m_physicsTiles.size() - 1);
		if (!isBulkErase) {
			calculateRotationalInertiaAndAABB();
		}

		assert(isValid());
	}

	void beginBulkInsert() {
		isBulkAdd = true;
	}

	void endBulkInsert() {
		isBulkAdd = false;

		calculateRotationalInertia();
	}

	void beginBulkErase() {
		isBulkErase = true;
	}

	void endBulkErase() {
		isBulkErase = false;

		calculateRotationalInertiaAndAABB();
	}

private:
	void clear() {
		corners.setMin(glm::i32vec2(std::numeric_limits<int32_t>::max()));
		corners.setMax(glm::i32vec2(-std::numeric_limits<int32_t>::max()));
	}

	bool isBulkErase = false;
	bool isBulkAdd = false;
	TileMap<TileType>& m_tileMap;
	AABB<int32_t> corners;
	boost::container::flat_set<glm::i32vec2> m_chunks;
};

template<typename TileType>
bool TileMap<TileType>::addTile(const glm::i32vec2& pos, const TileProperties<TileType>& props, const PhysicsTileProperties& physicsProperties) {
	assert(m_body);

	if (m_tiles.contains(pos))
		return false;

	if (props.isMultiTile) {
		glm::i32vec2 endPos = {
			pos.x + props.mutliTile.width,
			pos.y + props.mutliTile.height
		};

		TileProperties bufferTileProperties = props;
		bufferTileProperties.isMainTile = false;
		bufferTileProperties.mainTilePos = pos;

		// ensure the area is clear
		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				if (m_tiles.contains({ x, y })) {
					return false;
				}
			}
		}

		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				m_tiles[{x, y}] = bufferTileProperties;
			}
		}
	}

	m_tiles[pos] = props;
	m_physicsTiles.insert(std::pair(pos, physicsProperties));
	m_body->addTile(pos);

	return true;
}

template<typename TileType>
void TileMap<TileType>::removeTile(const glm::i32vec2& pos) noexcept {
	assert(m_body);

	if (!m_tiles.contains(pos))
		throw "Tile does not exist";

	m_body->removeTile(pos);
	m_physicsTiles.erase(pos);

	TileProperties& tileProperties = m_tiles.at(pos);
	if (tileProperties.isMultiTile) {
		if (!tileProperties.isMainTile)
			throw "Attempt to remove multi tile that isn't the main tile\n";

		glm::i32vec2 endPos = {
			pos.x + tileProperties.mutliTile.width,
			pos.y + tileProperties.mutliTile.height
		};

		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				m_tiles.erase({ x, y });
			}
		}

		return;
	}

	m_tiles.erase(pos);
}

template<typename TileType>
void TileMap<TileType>::beginBulkInsert() {
	assert(m_body);
	m_body->beginBulkInsert();
}

template<typename TileType>
void TileMap<TileType>::endBulkInsert() {
	assert(m_body);
	m_body->endBulkInsert();
}

template<typename TileType>
void TileMap<TileType>::beginBulkErase() {
	assert(m_body);
	m_body->beginBulkErase();
}

template<typename TileType>
void TileMap<TileType>::endBulkErase() {
	assert(m_body);
	m_body->endBulkErase();
}

template<typename TileType>
TileBody<TileType>& TileMap<TileType>::body() {
	return *m_body;
}

template<class TileType, class TileMapAllocator, class PhysicsWorld>
struct CreateTileMap {
	std::pair<TileMap<TileType>*, Body*> operator()(TileMapAllocator& tileMapAllocator, PhysicsWorld& physicsWorld) {
		TileMap<TileType>* tileMap = tileMapAllocator.construct<_CreateTileMap>(_CreateTileMap());
		Body* tileBody = physicsWorld.createTileBody(*tileMap);

		return { tileMap, tileBody };
	}
};

template<class TileType, class TileMapAllocator, class PhysicsWorld>
std::pair<TileMap<TileType>*, Body*> createTileMap(TileMapAllocator& tileMapAllocator, PhysicsWorld& physicsWorld) {
	return CreateTileMap<TileType, TileMapAllocator, PhysicsWorld>()(tileMapAllocator, physicsWorld);
}

T2D_NAMESPACE_END