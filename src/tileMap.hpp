#pragma once

#include "body.hpp"

/**
 * @file tileMap.hpp
 *
 * @brief Defines TileMap<> and TileBody<>
 */

T2D_NAMESPACE_BEGIN

/**
 * @class TileProperties
 * 
 * @brief Contains the data of a tile or multi tile
 * 
 * @tparam T the user data type of the tile
 */
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

/**
 * @class PhysicsTileProperties
 *
 * @brief Contains the physical properties of a tile, such as elasticity
 */
struct PhysicsTileProperties {
	/**
	 * @brief Returns the rotational inertia of this tile
	 *
	 * @param tileDimensions The tileDimensions from which the area of this tile will be calculated
	 */
	template<typename Integer>
	Float getMomentOfInertia(const glm::vec<2, Integer>& tileDimensions) const {
		// MOI scalar of a rectangle
		// I = (1 / 12) * m(w * w + h * h)

		constexpr Float one_twelfth = 1.0f / 12.0f;

		return one_twelfth * getMass(tileDimensions) * (sqaure(tileWidth * tileDimensions.x) + sqaure(tileHeight * tileDimensions.y));
	}

	/**
	 * @brief Returns the mass of this tile with using the area calculated from @p tileDimensions
	 * 
	 * @param tileDimensions The tileDimensions from which the area of this tile will be calculated
	 */
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

/* Forward Declaration */
template<typename Access, typename TileType>
class TileBody;

/* Used for construction of a TileMap */
struct _CreateTileMap {};

template<typename Access, typename TileType>
class TileMap {
	friend class TileBody<Access, TileType>;
	template<class Access, class ATileType, class TileMapAllocator, class PhysicsWorld>
	friend class CreateTileMap;

public:
	/**
	 * @brief Do not call directly, to construct a tileMap use createTileMap()
	 */
	TileMap(_CreateTileMap) {};

	virtual ~TileMap() {}

	/**
	 * @brief Returns the physical properties of a tile located at @p tilePos
	 *
	 * @param tilePos The position of some tile to get the physical properties of 
	 * 
	 * @return The physical properties of a tile located at @p tilePos
	 */
	inline const PhysicsTileProperties& getPhysicsTileProperties(const glm::i32vec2& tilePos) const {
		return this->m_physicsTiles.find(tilePos)->second;
	}

	/**
	 * @brief Returns the properties of a tile located at @p tilePos
	 *
	 * @param tilePos The position of some tile to get the properties of
	 *
	 * @return The reference properties of a tile located at @p tilePos
	 */
	inline TileProperties<TileType>& getTileProperties(const glm::i32vec2& tilePos) {
		return this->m_tiles.find(tilePos)->second;
	}

	/**
	 * @brief Used to speed up the insert of a large amount of tiles. To use for speed up, call before inserting a large amount of tiles. Once the wanted amount of tiles has been inserted, endBulkInsert() MUST be called
	 */
	void beginBulkInsert();

	/**
	 * @brief Used to speed up the insert of a large amount of tiles. To use for speed up, call after inserting a large amount of tiles. Before the wanted amount of tiles has been inserted, beginBulkInsert() MUST be called
	 */
	void endBulkInsert();

	/**
	 * @brief Used to speed up the erasure of a large amount of tiles. To use for speed up, call before erasing a large amount of tiles. After the wanted amount of tiles has been inserted, endBulkErase() MUST be called
	 */
	void beginBulkErase();

	/**
	 * @brief Used to speed up the erasure of a large amount of tiles. To use for speed up, call after erasing a large amount of tiles. Before the wanted amount of tiles has been inserted, beginBulkErase() MUST be called
	 */
	void endBulkErase();

	/**
	 * @brief Adds a tile at @p pos with the properties of @p props and the physics properties set to @p physicsProperties.
	 * If a tile already exists in that position, no change will be made.
	 * 
	 * @param pos The position of the tile to add
	 * @pararm props The tile properties of the tile
	 * @param physicsProperties The physicsProperties of the tile. 
	 * 
	 * @return True if the tile could be added.
	 */
	bool addTile(const glm::i32vec2& pos, const TileProperties<TileType>& props, const PhysicsTileProperties& physicsProperties = {});

	/**
	 * @brief Removes a tile at @p pos.
	 *
	 * @param pos The position of the tile to remove
	 *
	 * @return True if the tile could be removed.
	 */
	bool removeTile(const glm::i32vec2& pos);

	/**
	 * @brief Returns the tile dimensions of the tile located at @p pos
	 *
	 * @details The tile must exist before calling this function
	 * 
	 * @param pos The position of the tile to get the dimensions of
	 *
	 * @return The tile dimensions
	 */
	inline glm::u16vec2 getTileDimensions(const glm::i32vec2& pos) const {
		auto& props = this->m_tiles.find(pos)->second;

		if (props.isMultiTile) {
			assert(props.isMainTile);
			return { props.mutliTile.width, props.mutliTile.height };
		}

		return { 1, 1 };
	}

	/**
	 * @brief Returns the REAL tile dimensions, meaning world dimensions, of the tile located at @p pos
	 *
	 * @details The tile must exist before calling this function
	 *
	 * @param pos The position of the tile to get the dimensions of
	 *
	 * @return The real tile dimensions
	 */
	inline vec2 getRealTileDimensions(const glm::i32vec2& pos) const {
		return (vec2)getTileDimensions(pos) * tileSize;
	}

	/**
	 * @brief Returns the amount of tiles, including the sub-tiles of multi-tiles.
	 *
	 * @return The amount of tiles
	 */
	size_t count() const {
		return this->m_tiles.size();
	}

	/**
	 * @brief Returns the rigid body attached to this tile map
	 *
	 * @return The rigid body attached to this tile map
	 */
	TileBody<Access, TileType>& body();

public:
	/**
	 * @class Iterator 
	 * 
	 * @brief Allows iteration of tiles with for loops/
	 */
	struct Iterator {
		using ContainerIterator = typename FlatMap<glm::i32vec2, TileProperties<TileType>>::iterator;
	private:
		ContainerIterator m_it;
	public:
		Iterator(const ContainerIterator& it)
			: m_it(it) {}

		const glm::i32vec2& operator*() const {
			return this->m_it->first;
		}

		Iterator& operator++() {
			this->m_it = ++this->m_it;
			return *this;
		}

		Iterator& operator--() {
			this->m_it = --this->m_it;
			return *this;
		}

		bool operator!=(const Iterator& other) const {
			return this->m_it != other.m_it;
		}
	};

	Iterator begin() { return Iterator(this->m_tiles.begin()); }
	Iterator end() { return Iterator(this->m_tiles.end()); }

protected:
	/**
	 * SHOULD only be called while the tile map is being created.
	 */
	void setTileBody(TileBody<Access, TileType>* body) {
		this->m_body = body;
	}

protected:
	FlatMap<glm::i32vec2, TileProperties<TileType>> m_tiles;
	FlatMap<glm::i32vec2, PhysicsTileProperties> m_physicsTiles;

private:
	TileBody<Access, TileType>* m_body;
};

/**
 * @class TileBody
 * 
 * @brief For use with a TileMap<>, allows collisions with other rigid bodies inside World<>
 */
template<typename Access, typename TileType>
class TileBody : public Body<Access> {
	friend class TileMap<Access, TileType>;
public:
	/**
	 * @class SpatialIndex
	 * 
	 * @brief A spatial index where pos may point towards a chunk or tile
	 */
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
	/**
	 * @brief Do not call directly, to construct a TileBody use createTileMap()
	 */
	TileBody(const Access& access, TileMap<Access, TileType>& tileMap, uint32_t id)
		: Body<Access>(access, id, BodyType::Tile), m_tileMap(tileMap) {
		clear();

		tileMap.setTileBody(this);
	}

	inline virtual void integrate(Float dt) override {
		Body<Access>::integrate(dt);
	}

	inline virtual AABB<Float> getAABB() const {
		return Body<Access>::getAABB(getLocalAABB(), Transform());
	}

	/**
	 * @brief Returns the tile map attached to this rigid body
	 *
	 * @return The tile map attached to this rigid body
	 */
	inline TileMap<Access, TileType>& tileMap() const {
		return this->m_tileMap;
	}

	/**
	 * @brief Returns the normals of this body. Can be used for all tiles of this body
	 *
	 * @return The normals of this body
	 */
	inline std::array<vec2, 2> normals() const {
		const SinCos& sincos = this->m_transform->sincos;

		// _1_2, 2_3
		return { vec2(sincos.cos, sincos.sin), vec2(-sincos.sin, sincos.cos) };
	}

public: /* Tiles! */
	/**
	 * @brief Returns the local center of the tile located at @p iPos. Tile must be already created
 	 *
	 * @param iPos The tile position of the tile to get the local center of
	 *
	 * @return The local center of the tile located at @p iPos.
	 */
	inline vec2 getTileLocalPoint(const glm::i32vec2& iPos) const {
		return (vec2)iPos * vec2(tileWidth, tileHeight) + this->m_tileMap.getRealTileDimensions(iPos) * Float(0.5f);
	}

	/**
	 * @brief Returns the world center of the tile located at @p iPos. Tile must be already created
	 *
	 * @param iPos The tile position of the tile to get the world center of
	 *
	 * @return The world center of the tile located at @p iPos.
	 */
	inline vec2 getTileWorldPoint(const glm::i32vec2& iPos) const {
		return this->getWorldPoint(getTileLocalPoint(iPos));
	}

	/**
	 * @brief Returns the world oriented bounding box of the tile located at @p iPos. Tile must be already created
	 *
	 * @param iPos The tile position of the tile to get the world oriented bounding box of
	 *
	 * @return The world oriented bounding box of the tile located at @p iPos.
	 */
	inline TOBB getTileWorldOBB(const glm::i32vec2& iPos) const {
		return getTileWorldOBBOffset(iPos, vec2(0.0f));
	}

	/**
	 * @brief Returns the world oriented bounding box of the tile located at @p iPos. with an added offset Tile must be already created
	 *
	 * @param iPos The tile position of the tile to get the world oriented bounding box of
	 * @param worldOffset The world offset to add to the position of TOBB
	 * 
	 * @return The world oriented bounding box of the tile located at @p iPos.
	 */
	inline TOBB getTileWorldOBBOffset(const glm::i32vec2& iPos, const vec2& worldOffset) const {
		TOBB tileOBB;

		tileOBB.extent = this->m_tileMap.getRealTileDimensions(iPos) * Float(0.5);
		tileOBB.transform.pos = getTileWorldPoint(iPos) + worldOffset;
		tileOBB.transform.rot = this->m_transform->rot;
		tileOBB.transform.sincos = this->m_transform->sincos;

		return tileOBB;
	}

	/**
	 * @brief Returns the world vertices of the tile located at @p iPos. with an added offset
	 *
	 * @param iPos The tile position of the tile to get the world vertices of
	 * @param worldOffset The world offset to add to the position of the world vertices
	 *
	 * @return The world vertices of the tile located at @p iPos.
	 */
	inline std::array<vec2, 4> getTileWorldOBBOffsetVertices(const glm::i32vec2& iPos, const vec2& worldOffset) const {
		vec2 extent = this->m_tileMap.getRealTileDimensions(iPos) * Float(0.5);
		vec2 worldPos = getTileWorldPoint(iPos) + worldOffset;
		const vec2 wExt = rotate({ extent.x, extent.y }, this->m_transform->sincos);
		const vec2 blExt = rotate({ extent.x, -extent.y }, this->m_transform->sincos);
		const vec2 trExt = -blExt;

		return {
			 worldPos - wExt,
			{worldPos.x + blExt.x, worldPos.y + blExt.y},
			 worldPos + wExt,
			{worldPos.x + trExt.x, worldPos.y + trExt.y}
		};
	}

	/**
	 * @brief Returns the world AABB of a tile located at @p iPos
	 *
	 * @param iPos The tile position of the tile to get the world AABB of
	 *
	 * @return The world AABB of a tile located at @p iPos
	 */
	inline AABB<Float> getTileWorldAABB(const glm::i32vec2& iPos) const {
		vec2 tileDim = this->m_tileMap.getRealTileDimensions(iPos);

		return AABB(getTileWorldPoint(iPos), tileDim.x * Float(0.5), tileDim.y * Float(0.5)).rotate(this->m_transform->sincos);
	}

	/**
	 * @brief Returns the local AABB of a tile located at @p iPos
	 *
	 * @param iPos The tile position of the tile to get the local AABB of
	 *
	 * @return The local AABB of a tile located at @p iPos
	 */
	inline AABB<Float> getTileLocalAABB(const glm::i32vec2& iPos) const {
		vec2 tileDim = this->m_tileMap.getRealTileDimensions(iPos);

		return { getTileLocalPoint(iPos), tileDim.x * Float(0.5), tileDim.y * Float(0.5) };
	}

public:
	/**
	 * @brief Will get the list of chunks that intersect @p intersectBox, along with their AABBS
	 * 
	 * @param intersectBox An AABB that will be used to recieve a list of chunks that intersect it
	 * @param chunks An inserter object used to recieve the list of chunks intersectBox
	 * 
	 * @tparam OutIt The type of the inserter object, See @p chunks
	 */
	template<typename OutIt>
	inline void queryChunks(const AABB<Float>& intersectBox, OutIt chunks) const {
		const glm::i32vec2 minChunk = getChunkCoords(intersectBox.min());
		const glm::i32vec2 maxChunk = getChunkCoords(intersectBox.max());

		assert(minChunk.x <= maxChunk.x);
		assert(minChunk.y <= maxChunk.y);

		for (int32_t y = minChunk.y; y <= maxChunk.y; y++) {
			for (int32_t x = minChunk.x; x <= maxChunk.x; x++) {
				glm::i32vec2 coord(x, y);

				if (this->m_chunks.contains(coord)) {
					chunks++;
					*chunks = SpatialIndex(getChunkAABB(coord), coord);
				}
			}
		}
	}

	/**
	 * @brief Will get the list of tiles that intersect @p intersectBox
	 *
	 * @param intersectBox An AABB that will be used to recieve a list of tiles that intersect it
	 * @param tiles An inserter object used to recieve the list of tiles that intersect intersectBox
	 *
	 * @tparam OutIt The type of the inserter object, See @p tiles
	 */
	template<typename OutIt>
	inline void queryTiles(const AABB<Float>& intersectBox, OutIt tiles) const {
		glm::i32vec2 minTiles = getTileCoords(intersectBox.min());
		glm::i32vec2 maxTiles = getTileCoords(intersectBox.max());

		assert(minTiles.x <= maxTiles.x);
		assert(minTiles.y <= maxTiles.y);

		for (int32_t y = minTiles.y; y <= maxTiles.y; y++) {
			for (int32_t x = minTiles.x; x <= maxTiles.x; x++) {
				glm::i32vec2 coord(x, y);

				if (this->m_tileMap.m_tiles.contains(coord)) {
					tiles++;
					if (this->m_tileMap.m_tiles[coord].isMultiTile && !this->m_tileMap.m_tiles[coord].isMainTile)
						*tiles = SpatialIndex(this->m_tileMap.m_tiles[coord].mainTilePos);
					else
						*tiles = SpatialIndex(coord);
				}
			}
		}
	}

	/**
	 * @brief Get the chunk that contains localCoords. Chunk may or may not exist
	 *
	 * @param localCoords A local position
	 *
	 * @return The chunk position
	 */
	glm::i32vec2 getChunkCoords(const vec2& localCoords) const {
		return (glm::i32vec2)glm::floor(localCoords / (tileSize * vec2(chunkWidth, chunkHeight)));
	}

	/**
	 * @brief Get the chunk that contains tileCoords. Chunk may or may not exist
	 *
	 * @param tileCoords A tile position
	 *
	 * @return The chunk position
	 */
	inline glm::i32vec2 getChunkCoords(const glm::i32vec2& tileCoords) const {
		glm::i32vec2 chunk = {
			tileCoords.x / chunkWidth,
			tileCoords.y / chunkHeight
		};

		return chunk;
	}

	/**
	 * @brief Get the tile that contains localCoords. Tile may or may not exist
	 *
	 * @param localCoords A local position
	 *
	 * @return The tile position
	 */
	glm::i32vec2 getTileCoords(const vec2& localCoords) const {
		return (glm::i32vec2)glm::floor(localCoords / (tileSize));
	}

	/**
	 * @brief Get the AABB of the chunk located at @p chunk
	 *
	 * @param chunk The chunk position
	 *
	 * @return The chunk AABB
	 */
	inline AABB<Float> getChunkAABB(const glm::i32vec2& chunk) const {
		constexpr vec2 chunkDim((Float)(tileWidth * chunkWidth), (Float)(tileHeight * chunkHeight));

		return AABB((vec2)chunk * chunkDim + chunkDim * Float(0.5f), chunkDim.x * Float(0.5), chunkDim.y * Float(0.5));
	}

	// gets the local AABB of this object
	inline AABB<Float> getLocalAABB() const {
		const vec2 halfDim = {
			(Float)corners.width() * tileWidth * Float(0.5f) + tileWidth * Float(0.5f),
			(Float)corners.height() * tileHeight * Float(0.5f) + tileHeight * Float(0.5f)
		};
		return AABB(halfDim) + getTileLocalPointNoDim(corners.min()) + halfDim;
	}

protected:
	inline void insertIntoChunk(const glm::i32vec2& tile) {
		glm::i32vec2 chunk = getChunkCoords(tile);

		if (this->m_chunks.find(chunk) == this->m_chunks.end()) {
			this->m_chunks.insert(chunk);
		}
	}


	// get the local position of a tile without accounting for the tiles actual dimensions
	inline vec2 getTileLocalPointNoDim(const glm::i32vec2& iPos) const {
		return (vec2)iPos * vec2(tileWidth, tileHeight);
	}

	inline void calculateRotationalInertia() {
		this->m_I = 0.0f;
		for (const auto& pair : this->m_tileMap.m_physicsTiles) {
			auto tileDimensions = this->m_tileMap.getTileDimensions(pair.first);
			this->m_I += parallelAxisTheorem<Float>(
				pair.second.template getMomentOfInertia<uint16_t>(tileDimensions), 
				pair.second.template getMass<uint16_t>(tileDimensions), 
				glm::length2(getTileLocalPoint(pair.first) - this->m_centroid));
		}
		if (this->m_I != 0.0f && !this->m_flags[IS_STATIC])
			this->m_inverseI = 1.0f / this->m_I;
		else
			this->m_inverseI = 0.0f;
	}

	inline void calculateRotationalInertiaAndAABB() {
		this->m_I = 0.0f;
		clear();
		auto& min = corners.min();
		auto& max = corners.max();
		for (const auto& pair : this->m_tileMap.m_physicsTiles) {
			const glm::i32vec2& tilePos = pair.first;
			const glm::u16vec2 tileDimensions = this->m_tileMap.getTileDimensions(tilePos);

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

			this->m_I += parallelAxisTheorem<Float>(
				pair.second.template getMomentOfInertia<uint16_t>(tileDimensions), 
				pair.second.template getMass<uint16_t>(tileDimensions), 
				glm::length2(getTileLocalPoint(tilePos) - this->m_centroid));
		}

		if (this->m_I != 0.0f && !this->m_flags[IS_STATIC])
			this->m_inverseI = 1.0f / this->m_I;
		else
			this->m_inverseI = 0.0f;
	}

protected: /* For use within the Tile Map Class*/
	void addTile(const glm::i32vec2& iPos) {
		const vec2 pos = getTileLocalPoint(iPos);
		const glm::u16vec2 tileDimensions = this->m_tileMap.getTileDimensions(iPos);
		const PhysicsTileProperties& physicsProperties = this->m_tileMap.m_physicsTiles.at(iPos);
		const Float tileMass = physicsProperties.getMass<uint16_t>(tileDimensions);

		this->m_unweightedCentroid += pos;
		this->m_centroid = this->m_unweightedCentroid / (Float)this->m_tileMap.m_physicsTiles.size();
		if (!isBulkAdd) {
			calculateRotationalInertia();
		}

		this->m_mass += tileMass;
		if(!this->m_flags[IS_STATIC])
			this->m_inverseMass = 1.0f / this->m_mass;
		this->m_unweightedCom += tileMass * pos;
		this->m_com = this->m_unweightedCom * this->m_inverseMass;

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
	}

	void removeTile(const glm::i32vec2& iPos) {
		const vec2 pos = getTileLocalPoint(iPos);
		const glm::u16vec2 tileDimensions = this->m_tileMap.getTileDimensions(iPos);
		const PhysicsTileProperties& physicsProperties = this->m_tileMap.m_physicsTiles.at(iPos);
		const Float tileMass = physicsProperties.getMass<uint16_t>(tileDimensions);

		this->m_mass -= tileMass;
		if (this->m_mass != 0.0f && !this->m_flags[IS_STATIC])
			this->m_inverseMass = 1.0f / this->m_mass;
		else
			this->m_inverseMass = 0.0f;
		this->m_unweightedCom -= tileMass * pos;
		this->m_com = this->m_unweightedCom * this->m_inverseMass;

		this->m_unweightedCentroid -= pos;
		this->m_centroid = this->m_unweightedCentroid / (Float)(this->m_tileMap.m_physicsTiles.size() - 1);
		if (!isBulkErase) {
			calculateRotationalInertiaAndAABB();
		}
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
	TileMap<Access, TileType>& m_tileMap;
	AABB<int32_t> corners;
	boost::unordered_flat_set<glm::i32vec2> m_chunks;
};

template<typename Access, typename TileType>
bool TileMap<Access, TileType>::addTile(const glm::i32vec2& pos, const TileProperties<TileType>& props, const PhysicsTileProperties& physicsProperties) {
	assert(this->m_body);

	if (this->m_tiles.contains(pos))
		return false;

	if (props.isMultiTile) {
		glm::i32vec2 endPos = {
			pos.x + props.mutliTile.width,
			pos.y + props.mutliTile.height
		};

		TileProperties<TileType> bufferTileProperties = props;
		bufferTileProperties.isMainTile = false;
		bufferTileProperties.mainTilePos = pos;

		// ensure the area is clear
		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				if (this->m_tiles.contains({ x, y })) {
					return false;
				}
			}
		}

		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				this->m_tiles[{x, y}] = bufferTileProperties;
			}
		}
	}

	this->m_tiles[pos] = props;
	this->m_physicsTiles.insert(std::pair(pos, physicsProperties));
	this->m_body->addTile(pos);

	return true;
}

template<typename Access, typename TileType>
bool TileMap<Access, TileType>::removeTile(const glm::i32vec2& pos) {
	assert(this->m_body);

	if (!this->m_tiles.contains(pos))
		return false;

	this->m_body->removeTile(pos);
	this->m_physicsTiles.erase(pos);

	TileProperties<TileType>& tileProperties = this->m_tiles.at(pos);
	if (tileProperties.isMultiTile) {
		if (!tileProperties.isMainTile)
			throw "Attempt to remove multi tile that isn't the main tile\n";

		glm::i32vec2 endPos = {
			pos.x + tileProperties.mutliTile.width,
			pos.y + tileProperties.mutliTile.height
		};

		// ensure the area is clear
		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				if (!this->m_tiles.contains({ x, y })) {
					return false;
				}
			}
		}

		for (auto y = pos.y; y < endPos.y; y++) {
			for (auto x = pos.x; x < endPos.x; x++) {
				this->m_tiles.erase({ x, y });
			}
		}

		return true;
	}

	this->m_tiles.erase(pos);

	return true;
}

template<typename Access, typename TileType>
void TileMap<Access, TileType>::beginBulkInsert() {
	assert(this->m_body);
	this->m_body->beginBulkInsert();
}

template<typename Access, typename TileType>
void TileMap<Access, TileType>::endBulkInsert() {
	assert(this->m_body);
	this->m_body->endBulkInsert();
}

template<typename Access, typename TileType>
void TileMap<Access, TileType>::beginBulkErase() {
	assert(this->m_body);
	this->m_body->beginBulkErase();
}

template<typename Access, typename TileType>
void TileMap<Access, TileType>::endBulkErase() {
	assert(this->m_body);
	this->m_body->endBulkErase();
}

template<typename Access, typename TileType>
TileBody<Access, TileType>& TileMap<Access, TileType>::body() {
	return *this->m_body;
}

template<class Access, class TileType, class TileMapAllocator, class PhysicsWorld>
struct CreateTileMap {
	std::pair<TileMap<Access, TileType>*, TileBody<Access, TileType>*> operator()(const Access& ref, TileMapAllocator& tileMapAllocator, PhysicsWorld& physicsWorld) {
		TileMap<Access, TileType>* tileMap = tileMapAllocator.construct<_CreateTileMap>(_CreateTileMap());
		TileBody<Access, TileType>* tileBody = (TileBody<Access, TileType>*)physicsWorld.createTileBody(ref, *tileMap);

		return { tileMap, tileBody };
	}
};

/**
 * @brief Creates a tile map and a tile body using @p tileMapAllocator for the tileMap and the @p physicsWorld for the tile body
 * 
 * @param ref A transform of some external object. Will be manipulated as the simulation runs
 * @param tileMapAllocator An allocator used to create the tile map, MUST have a construct<>() method defined
 * @param physicsWorld A physicsWorld used to create the tile body, MUST be t2d::World<>
 * 
 * @return A pair pointing towards the tile map and tile body. Check if nullptr.
 */
template<class Access, class TileType, class TileMapAllocator, class PhysicsWorld>
std::pair<TileMap<Access, TileType>*, TileBody<Access, TileType>*> createTileMap(const Access& ref, TileMapAllocator& tileMapAllocator, PhysicsWorld& physicsWorld) {
	return CreateTileMap<Access, TileType, TileMapAllocator, PhysicsWorld>()(ref, tileMapAllocator, physicsWorld);
}

T2D_NAMESPACE_END