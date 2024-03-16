#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <SFML/Graphics.hpp>
#include <functional>
#include <chrono>
#include "physicsAlgorithims.hpp"

#include <boost/container/flat_map.hpp>
#include <boost/container/map.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/thread/latch.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

using namespace std::chrono;

template<>
struct std::less<glm::i32vec2> {
	static constexpr int32_t width = 46341; // sqrt(INT32_MAX)

	bool operator()(const glm::i32vec2& _1, const glm::i32vec2& _2) const {
		float _1i = (_1.y * width) + _1.x;
		float _2i = (_2.y * width) + _2.x;

		return _1i < _2i;
	}
};

template<>
struct std::hash<glm::i32vec2> {
	bool operator()(const glm::i32vec2& vec) const {
		const int32_t p1 = 73856093;
		const int32_t p2 = 19349663;

		return vec.x * p1 ^ vec.y * p2 ;
	}
};

template<typename K, typename T>
using Map = std::unordered_map<K, T>;

namespace bgi = boost::geometry::index;
static constexpr size_t dimension = 2;
using CST = boost::geometry::cs::cartesian;
using Point = boost::geometry::model::point<float, dimension, CST>;
using Box = boost::geometry::model::box<Point>;

class Stopwatch {
public:
	void start() {
		m_start = steady_clock::now();
	}
	
	float getTime() {
		return duration_cast<microseconds>(duration<float>(steady_clock::now() - m_start)).count();
	}

private:
	steady_clock::time_point m_start;
};

struct {
	size_t collisionDetCalled = 1;
	float collisionDetTotalTime = 0.0f;
	size_t collisionResCalled = 1;
	float collisionResTotalTime = 0.0f;
} stats;

struct SinCos {
	SinCos()
		: sin(0.0f), cos(1.0f) {}

	SinCos(float rot) {
		setRot(rot);
	}

	SinCos(float psin, float pcos)
		: sin(psin), cos(pcos) {}

	void setRot(float rot) {
		sin = glm::sin(rot);
		cos = glm::cos(rot);
	}

	SinCos getNegate() const {
		return SinCos(-sin, cos);
	}

	SinCos operator-(const SinCos& other) const {
		//sin(a−c)=sin(a)cos(c)−cos(a)sin(c)
		//cos(a−c)=cos(a)cos(c)+sin(a)sin(c)

		SinCos dif;
		dif.sin = sin * other.cos - cos * other.sin;
		dif.cos = cos * other.cos + sin * other.sin;
		
		return dif;
	}

	SinCos operator+(const SinCos& other) const {
		//sin(a+c)=sin(a)cos(c)+cos(a)sin(c)
		//cos(a+c)=cos(a)cos(c)−sin(a)sin(c)

		SinCos dif;
		dif.sin = sin * other.cos + cos * other.sin;
		dif.cos = cos * other.cos - sin * other.sin;

		return dif;
	}

	float sin;
	float cos;
};

inline glm::vec2 rotate(const glm::vec2& v, float sin, float cos) {
	glm::vec2 result;

	result.x = v.x * cos - v.y * sin;
	result.y = v.x * sin + v.y * cos;
	return result;
}

inline glm::vec2 rotate(const glm::vec2& v, const SinCos& sincos) {
	return rotate(v, sincos.sin, sincos.cos);
}

sf::Vector2f convert(const glm::vec2& vec) {
	return sf::Vector2f(vec.x, vec.y);
}

struct Transform {
	using vec2 = glm::vec<2, float>;

	Transform()
		: pos(0.0f, 0.0f), rot(0.0f) {}

	Transform(const vec2& pos, float rot)
		: pos(pos), rot(rot) {}

	Transform getInverse() const {
		return Transform(-pos, -rot);
	}

	vec2 getWorldPoint(const vec2& localPoint) {
		return rotate(localPoint, sincos) + pos;
	}

	vec2 getWorldPoint(const vec2& localPoint, const glm::vec2& localOffset) {
		return rotate(localPoint - localOffset, sincos) + localOffset + pos;
	}

	vec2 getLocalPoint(const vec2& worldPoint, const glm::vec2& localOffset) {
		return rotate(worldPoint - pos - localOffset, sincos.getNegate()) + localOffset;
	}

	// updates cache of sincos
	void update() {
		sincos.setRot(rot);
	}

	vec2  pos; // In Meters
	float rot; // In radians
	SinCos sincos;
};

template<typename T = float>
struct AABB {
public:
	using vec2 = glm::vec<2, T>;

	AABB()
		: m_min(0, 0), m_max(0, 0) {}

	AABB(vec2 min, vec2 max)
		: m_min(min), m_max(max) {}

	AABB(vec2 pos, float hW, float hH)
		: AABB({hW, hH}) {
		*this += pos;
	}

	AABB(vec2 halfDim)
		: m_min(-halfDim.x, -halfDim.y), m_max(halfDim.x, halfDim.y) {}


	AABB(vec2 halfDim, const SinCos& sincos) {
		*this = AABB<T>(halfDim).rotate(sincos);
	}

	AABB& operator+=(const vec2& off) {
		m_min += off;
		m_max += off;
	
		return *this;
	}

	AABB operator+(const vec2& off) const {
		AABB newAABB = *this;
		newAABB.m_min += off;
		newAABB.m_max += off;

		return newAABB;
	}

	AABB operator-(const vec2& off) const {
		AABB newAABB = *this;
		newAABB.m_min -= off;
		newAABB.m_max -= off;

		return newAABB;
	}

	vec2& min() { return m_min; }
	vec2& max() { return m_max; }
	const vec2& min() const { return m_min; }
	const vec2& max() const { return m_max; }

	void setMin(vec2 newMin) {
		m_min = newMin;
	}

	void setMax(vec2 newMax) {
		m_max = newMax;
	}

	vec2 bl() const {
		return m_min;
	}

	vec2 br() const {
		return {m_max.x, m_min.y};
	}

	vec2 tr() const {
		return m_max;
	}

	vec2 tl() const {
		return {m_min.x, m_max.y};
	}

	void forEach(std::function<void(vec2)> callback) const {
		callback(bl());
		callback(br());
		callback(tr());
		callback(tl());
	}

	vec2 midpoint() const {
		return (m_max + m_min) / (T)2;
	}

	T width() const {
		return glm::abs(m_max.x - m_min.x);
	}

	T height() const {
		return glm::abs(m_max.y - m_min.y);
	}

	bool intersects(const AABB& other) const {
		vec2 d1, d2;
		d1 = vec2(other.m_min.x, other.m_min.y) - vec2(m_max.x, m_max.y);
		d2 = vec2(m_min.x, m_min.y) - vec2(other.m_max.x, other.m_max.y);

		if (d1.x > 0.0f || d1.y > 0.0f)
			return false;

		if (d2.x > 0.0f || d2.y > 0.0f)
			return false;

		return true;
	}

	Box getBoostBox() const {
		return Box(Point(m_min.x, m_min.y), Point(m_max.x, m_max.y));
	}

	// When T is an integer type, rotation will be unstable.
	AABB<T> rotate(T sin, T cos) const {
		sin = glm::abs(sin);
		cos = glm::abs(cos); 

		T hWidth = width() / (T)2;
		T hHeight = height() / (T)2;

		return {
			midpoint(), 
			hHeight * sin + hWidth * cos, 
			hWidth * sin + hHeight * cos
		};
	}

	AABB<T> rotate(const SinCos& sincos) const {
		return rotate(sincos.sin, sincos.cos);
	}

private:
	vec2 m_min;
	vec2 m_max;
};

struct Tile {
	static constexpr float width = 10.0f;
	static constexpr float height = 10.0f;

	// returns moment of inertia as a scalar
	float getMomentOfInertia() {
		// MOI scalar of a rectangle
		// I = (1 / 12) * m(w * w + h * h)
	
		float one_twelth = 1.0f / 12.0f;
		float mass = getMass();
		
		return one_twelth * mass * (width * width + height * height);
	}

	// returns the mass of this tile
	float getMass() {
		// area = width * height
		// mass = area * density
		return width * height * density;
	}

	// Must be set
	float density;
	float staticFriction;
	float dynamicFriction;
	float E; // Conservation of energy after impact: [0.0 - 1.0]
};

struct SpatialTileIndex {
	AABB<float> aabb;
	glm::i32vec2 tile;
};

struct SpatialTileIndexGetter {
	using result_type = Box const;

	result_type operator()(SpatialTileIndex const& index) const  {
		return index.aabb.getBoostBox();
	}
};

struct SpatialTileIndexEqualTo {
	bool operator()(SpatialTileIndex const& l, SpatialTileIndex const& r) const {
		return l.tile == r.tile;
	}
};

using Rtree = bgi::rtree<SpatialTileIndex, bgi::linear<16, 8>,
	SpatialTileIndexGetter, SpatialTileIndexEqualTo>;

struct TileBody {
	//float minQuadTree[2] = {-100000.0f, -100000.0f};
	//float maxQuadTree[2] = { 100000.0f, 100000.0f };
public:

	static constexpr std::array<glm::vec2, 2> defaultNormals = {
		glm::vec2(1.0f, 0.0f),
		glm::vec2(0.0f, 1.0f)
	};

	TileBody() 
		: inverseI(0.0f), inverseMass(0.0f),
		mass(0.0f), I(0.0f), com(0.0f, 0.0f), unweightedCom(0.0f, 0.0f), 
		linearVelocity(0.0f, 0.0f), force(0.0f, 0.0f), angularVelocity(0.0f), 
		torque(0.0f), normals(defaultNormals) {
		cache.clear();
	}

	void integrate(float dt) {
		// Using the semi implicit euler method

		// A(v) = force / mass
		// V = v + A(v) * dt
		// P = p + V * dt;

		linearVelocity = linearVelocity + force * inverseMass * dt;
		m_transform.pos = m_transform.pos + linearVelocity * dt;
		force = {0.0f, 0.0f};

		angularVelocity = angularVelocity + torque * inverseI * dt;
		m_transform.rot = m_transform.rot + angularVelocity * dt;
		torque = 0.0f;

		m_transform.update();

		normals[0] = rotate(defaultNormals[0], m_transform.sincos); 
		normals[1] = rotate(defaultNormals[1], m_transform.sincos);

		glm::vec2 dim = {
			(float)cache.corners.width() * Tile::width * 0.5f,
			(float)cache.corners.height() * Tile::height * 0.5f,
		};

		aabb = AABB(dim).rotate(m_transform.sincos) + m_transform.pos + com;
	}

	void addTile(glm::i32vec2 iPos, float density = 1.0f, float staticFriction = 1.0f, float dynamicFriction = 1.0f, float E = 0.5f) {
		//assert(!tiles.contains(iPos));

		glm::vec2 pos = getTileLocalPoint(iPos);
		tiles.insert(std::pair(iPos, Tile()));
		Tile& tile = tiles[iPos];

		tile.density = density;
		tile.staticFriction = staticFriction;
		tile.dynamicFriction = dynamicFriction;
		tile.E = E;

		float tileMass = tile.getMass();
		I += tile.getMomentOfInertia();
		mass += tileMass;
		inverseI = 1.0f / I;
		inverseMass = 1.0f / mass;
		unweightedCom += tileMass * pos;
		com = unweightedCom * inverseMass;
	
		AABB<float> tileAABB = getTileLocalAABB(iPos);
		tilePartition.insert({tileAABB, iPos});
		
		auto& min = cache.corners.min();
		auto& max = cache.corners.max();

		if(iPos.x < min.x) {
			min.x = iPos.x;
		}
		if (iPos.y < min.y) {
			min.y = iPos.y;
		}
		if (iPos.x > max.x) {
			max.x = iPos.x;
		}
		if (iPos.y > max.y) {
			max.y = iPos.y;
		}
	}

	void removeTile(glm::i32vec2 iPos) {
		assert(tiles.find(iPos) != tiles.end());
		
		glm::vec2 pos = getTileLocalPoint(iPos);
		Tile& tile = tiles.at(iPos);

		float tileMass = tile.getMass();

		I -= tile.getMomentOfInertia();
		mass -= tileMass;

		if(I != 0.0f)
			inverseI = 1.0f / I;
		else
			inverseI = 0.0f;
		if(mass != 0.0f)
			inverseMass = 1.0f / mass;
		else
			inverseMass = 0.0f;

		unweightedCom -= tileMass * pos;
		com = unweightedCom * inverseMass;

		tilePartition.remove({getTileLocalAABB(iPos), iPos});
		cache.clear();
		auto& min = cache.corners.min();
		auto& max = cache.corners.max();
		for(auto& pair : tiles) {
			glm::i32vec2 tilePos = pair.first;

			if (tilePos.x < min.x) {
				min.x = tilePos.x;
			}
			if (tilePos.y < min.y) {
				min.y = tilePos.y;
			}
			if (tilePos.x > max.x) {
				max.x = tilePos.x;
			}
			if (tilePos.y > max.y) {
				max.y = tilePos.y;
			}
		}
	}

	Transform& transform() {
		return m_transform;
	}

	void moveBy(const glm::vec2& addPos) {
		m_transform.pos += addPos;
	}

	void rotateBy(float amount) {
		m_transform.rot += amount;
		m_transform.update();
	}

	void setPos(const glm::vec2& pos) {
		m_transform.pos = pos;
	}

	void setRot(float rot) {
		m_transform.rot = rot;
		m_transform.update();
	}

	glm::vec2 getWorldPoint(const glm::vec2& localPoint) {
		return m_transform.getWorldPoint(localPoint, com);
	} 

	glm::vec2 getLocalPoint(const glm::vec2& worldPoint) {
		return m_transform.getLocalPoint(worldPoint, com);
	}

	// converts a world aabb to a local aabb for use with colliding in the local spatial index tree
	// this means its not entirely a true local conversion as it does not take into account
	// Center of Mass.
	AABB<float> getLocalAABB(const AABB<float>& worldAABB) {
		AABB localAABB(getLocalPoint(worldAABB.midpoint()), worldAABB.width() * 0.5f, worldAABB.height() * 0.5f);

		return localAABB.rotate(m_transform.sincos.getNegate());
	}

	// get the AABB in the localSpace of spaceTransform
	AABB<float> getAABB(const Transform& spaceTransform, const glm::vec2& localOffset = {0.0f, 0.0f}) {
		glm::vec2 dim = {
			(float)cache.corners.width() * Tile::width * 0.5f,
			(float)cache.corners.height() * Tile::height * 0.5f,
		};

		glm::vec2 relPos = (spaceTransform.pos + localOffset) - (m_transform.pos + com);
		AABB aabb = AABB(dim, spaceTransform.sincos - m_transform.sincos) + relPos;

		return aabb;
	}

	// get the local position of a tile
	glm::vec2 getTileLocalPoint(glm::i32vec2 iPos) {
		return (glm::vec2)iPos * glm::vec2(Tile::width, Tile::height);
	}

	// get the world position of a tile
	glm::vec2 getTileWorldPoint(glm::i32vec2 iPos) {
		return getWorldPoint(getTileLocalPoint(iPos));
	}
	
	std::array<glm::vec2, 4> getTileWorldOBB(glm::i32vec2 iPos) {
		glm::vec2 tilePos = getTileLocalPoint(iPos);
		glm::vec2 halfTile = glm::vec2(Tile::width, Tile::height) * 0.5f;

		std::array<glm::vec2, 4> obb = {
			tilePos - halfTile,
			{tilePos.x + halfTile.x, tilePos.y - halfTile.y},
			tilePos + halfTile,
			{tilePos.x - halfTile.x, tilePos.y + halfTile.y}
		};

		for(glm::vec2& point : obb) {
			point = getWorldPoint(point);
		}

		return obb;
	}

	AABB<float> getTileWorldAABB(glm::i32vec2 iPos) {
		glm::vec2 tilePos = (glm::vec2)iPos * glm::vec2(Tile::width, Tile::height);
		
		return AABB(getWorldPoint(tilePos), Tile::width * 0.5f, Tile::height * 0.5f).rotate(m_transform.sincos);
	}

	// Gets aabb in local space
	AABB<float> getTileLocalAABB(glm::i32vec2 iPos) {
		glm::vec2 pos = (glm::vec2)iPos * glm::vec2(Tile::width, Tile::height);
		glm::vec2 halfTile = glm::vec2(Tile::width, Tile::height) * 0.5f;

		return {pos, Tile::width * 0.5f, Tile::height * 0.5};
	}

	Map<glm::i32vec2, Tile> tiles;

private: 
	float inverseI;
	float inverseMass;

private:
	glm::vec2 com;
	float mass;
	float I;
	glm::vec2 unweightedCom;

	Transform m_transform;
public: // integrating
	glm::vec2 linearVelocity;
	glm::vec2 force;
	float angularVelocity;
	float torque;

	struct Cache {
		void clear() {
			corners.setMin(glm::i32vec2(std::numeric_limits<int32_t>::max()));
			corners.setMax(glm::i32vec2(-std::numeric_limits<int32_t>::max()));
		}

		AABB<int32_t> corners;
	} cache;

	AABB<float> aabb;
	std::array<glm::vec2, 2> normals;
	Rtree tilePartition;
};

using MinMax = std::pair<float, float>;

template<typename T>
bool computeApproxCollisionArea(const AABB<T>& _1, const AABB<T>& _2, AABB<T>& area) {
	if(!_1.intersects(_2))
		return false;

	std::array<glm::vec<2, T>, 4> _1vertices = {
		_1.bl(), _1.br(), _1.tr(), _1.tl()
	};

	std::array<glm::vec<2, T>, 4> _2vertices = {
		_2.bl(), _2.br(), _2.tr(), _2.tl()
	};

	std::vector<glm::vec<2, T>> res = sutherlandHodgmanClip(_2vertices, _1vertices);
	assert(!res.empty() && res.size() == 4); // based on our previous test, this should never be empty and always be 4

	// cww order of a rect, BL, BR, TR, TL
	//                      0   1   2   3
	area.setMin(res[0]);
	area.setMax(res[2]);

	// may need to swap around coords 
	if(area.min().x > area.max().x) {
		std::swap(area.min().x, area.max().x);
	}
	if (area.min().y > area.max().y) {
		std::swap(area.min().y, area.max().y);
	}

	return true;
}

struct CollisionManifold {
	glm::vec2 normal = { 0.0f, 0.0f };
	float depth = std::numeric_limits<float>::max();
};

void collisionResolve(TileBody& body1, TileBody& body2, const CollisionManifold& manifold) {
	body1.moveBy(-manifold.normal * manifold.depth * 0.5f);
	body2.moveBy(manifold.normal * manifold.depth * 0.5f);
}

bool fineCollisionDetect(TileBody& body1, glm::i32vec2 tilePos1, TileBody& body2, glm::i32vec2 tilePos2, CollisionManifold& manifold) {
	auto tileObb1 = body1.getTileWorldOBB(tilePos1);
	auto tileObb2 = body2.getTileWorldOBB(tilePos2);

	if (!satHalfTest(tileObb1, tileObb2, body1.normals, manifold.normal, manifold.depth)) {
		return false;
	}

	if (!satHalfTest(tileObb1, tileObb2, body2.normals, manifold.normal, manifold.depth)) {
		return false;
	}

	return true;
}

// Test Collision 01 - O(n^2) with tile AABB
// The best collision detection algorithim:
// for every tile in body1 check it against all of body2's tiles using
// their world AABB's, if they collide then use the fine collision detection (SAT)
void testCollision_01(TileBody& body1, TileBody& body2) {
	Stopwatch timer;
	timer.start();

	for (auto& tile1 : body1.tiles) {
		auto tile1AABB = body1.getTileWorldAABB(tile1.first);

		for (auto& tile2 : body2.tiles) {
			auto tile2AABB = body2.getTileWorldAABB(tile2.first);
			if(!tile1AABB.intersects(tile2AABB))
				continue;

			CollisionManifold manifold;
			if (fineCollisionDetect(body1, tile1.first, body2, tile2.first, manifold))
				collisionResolve(body1, body2, manifold);
		}
	}

leave:
	stats.collisionDetTotalTime += timer.getTime();
	stats.collisionDetCalled++;
}

// Test Collision 02 - O(n^2) AABB skim
// Using the the AABB from body1 and body2, it tests if there could be a collision.
// If there is a collision form a new AABB from body1's and body2's AABB 
// using computeApproxCollisionArea() function.
// With this new AABB, for every tile in the first body check if its AABB collides with this
// approximate AABB, if it does then for every tile in the second body check it against
// this tile's AABB, if it does check these two tiles against eachother using the finer collision detection
// algorithim (SAT)
void testCollision_02(TileBody& body1, TileBody& body2) {
	Stopwatch timer;
	timer.start();

	AABB<float> approxArea;
	if (!computeApproxCollisionArea(body1.aabb, body2.aabb, approxArea))
		goto leave;

	for (auto& tile1 : body1.tiles) {
		auto tile1AABB = body1.getTileWorldAABB(tile1.first);
		if(!tile1AABB.intersects(approxArea))
			continue;

		for (auto& tile2 : body2.tiles) {
			auto tile2AABB = body2.getTileWorldAABB(tile2.first);
			if (!tile1AABB.intersects(tile2AABB))
				continue;

			CollisionManifold manifold;
			if (fineCollisionDetect(body1, tile1.first, body2, tile2.first, manifold))
				collisionResolve(body1, body2, manifold);
		}
	}

leave:
	stats.collisionDetTotalTime += timer.getTime();
	stats.collisionDetCalled++;
}

// Test Collision 03 - O(n^2) Local R-Tree AABB skim
// Using the the AABB from body1 and body2, it tests if there could be a collision.
// If there is a collision form a new AABB from body1's and body2's AABB 
// using computeApproxCollisionArea() function, then convert this aabb into the local 
// space of body1 and body2. With this new local aabb query into the local spatial 
// index trees for any possible colliding tiles, do this for both body1 and body2. 
// With tiles recieved from body1's and body2's tree query, use a O(n^2) loop
// to test each tile from opposite lists against eachother using a finer collision detection test (SAT).
void testCollision_03(TileBody& body1, TileBody& body2) {
	Stopwatch timer;
	timer.start();

	std::vector<SpatialTileIndex> tiles1;
	std::vector<SpatialTileIndex> tiles2;

	AABB<float> approxArea;
	if (!computeApproxCollisionArea(body1.aabb, body2.aabb, approxArea))
		goto leave;

	AABB approxAreaBody1 = body1.getLocalAABB(approxArea);
	body1.tilePartition.query(bgi::intersects(approxAreaBody1.getBoostBox()), std::back_inserter(tiles1));
	AABB approxAreaBody2 = body2.getLocalAABB(approxArea);
	body2.tilePartition.query(bgi::intersects(approxAreaBody2.getBoostBox()), std::back_inserter(tiles2));

	for (auto& tile1 : tiles1) {
		for (auto& tile2 : tiles2) {
			CollisionManifold manifold;
			if (fineCollisionDetect(body1, tile1.tile, body2, tile2.tile, manifold))
				collisionResolve(body1, body2, manifold);
		}
	}

leave:
	stats.collisionDetTotalTime += timer.getTime();
	stats.collisionDetCalled++;
}

// Test Collision 04 - O(n^2) Local R-Tree AABB skim + tile AABB - (Porter Tile Collision Detection):
// Using the the AABB from body1 and body2, it tests if there could be a collision.
// If there is a collision form a new AABB from body1's and body2's AABB 
// using computeApproxCollisionArea() function, then convert this aabb into the local 
// space of body1 and body2. With this new local aabb query into the local spatial 
// index trees for any possible colliding tiles, do this for both body1 and body2. 
// With tiles recieved from body1's and body2's tree query, use a O(n^2) loop
// to test each tile from opposite lists against eachother using their WorldAABB before continuing
// to a finer collision detection test (SAT).
void testCollision_04(TileBody& body1, TileBody& body2, AABB<float>& oApproxArea) {
	Stopwatch timer;
	timer.start();

	std::vector<SpatialTileIndex> tiles1;
	std::vector<SpatialTileIndex> tiles2;

	AABB<float> approxArea;
	if(!computeApproxCollisionArea(body1.aabb, body2.aabb, approxArea))
		goto leave;

	oApproxArea = approxArea;

	AABB approxAreaBody1 = body1.getLocalAABB(approxArea);
	body1.tilePartition.query(bgi::intersects(approxAreaBody1.getBoostBox()), std::back_inserter(tiles1));
	AABB approxAreaBody2 = body2.getLocalAABB(approxArea);
	body2.tilePartition.query(bgi::intersects(approxAreaBody2.getBoostBox()), std::back_inserter(tiles2));

	for(auto& tile1 : tiles1) {
		auto tile1AABB = body1.getTileWorldAABB(tile1.tile);

		for(auto& tile2 : tiles2) {
			auto tile2AABB = body2.getTileWorldAABB(tile2.tile);
			if(!tile1AABB.intersects(tile2AABB))
				continue;

			CollisionManifold manifold;
			if(fineCollisionDetect(body1, tile1.tile, body2, tile2.tile, manifold))
				collisionResolve(body1, body2, manifold);
		}
	}

leave:
	stats.collisionDetTotalTime += timer.getTime();
	stats.collisionDetCalled++;
}

// Test Collision 05 - O(n^2) MT Local R-Tree AABB skim + tile AABB - (Porter Tile Collision Detection):
// Using the the AABB from body1 and body2, it tests if there could be a collision.
// If there is a collision form a new AABB from body1's and body2's AABB 
// using computeApproxCollisionArea() function, then convert this aabb into the local 
// space of body1 and body2. With this new local aabb query into the local spatial 
// index trees for any possible colliding tiles, do this for both body1 and body2. 
// With tiles recieved from body1's and body2's tree query, use a O(n^2) loop
// to test each tile from opposite lists against eachother using their WorldAABB before continuing
// to a finer collision detection test (SAT).
void testCollision_05(boost::asio::thread_pool& tp, TileBody& body1, TileBody& body2) {
	Stopwatch timer;
	timer.start();

	std::vector<SpatialTileIndex> tiles1;
	std::vector<SpatialTileIndex> tiles2;
	boost::latch latch(2);
	
	AABB<float> approxArea;
	if (!computeApproxCollisionArea(body1.aabb, body2.aabb, approxArea))
		goto leave;

	auto queryTree =
		[&latch](const Box& AABB, Rtree& rtree, std::vector<SpatialTileIndex>& tiles) {
			rtree.query(bgi::intersects(AABB), std::back_inserter(tiles));
			latch.count_down();
		};

	queryTree(body1.getLocalAABB(approxArea).getBoostBox(), body1.tilePartition, tiles1);
	queryTree(body2.getLocalAABB(approxArea).getBoostBox(), body2.tilePartition, tiles2);
	latch.wait();

	for (auto& tile1 : tiles1) {
		auto tile1AABB = body1.getTileWorldAABB(tile1.tile);

		for (auto& tile2 : tiles2) {
			auto tile2AABB = body2.getTileWorldAABB(tile2.tile);
			if (!tile1AABB.intersects(tile2AABB))
				continue;

			CollisionManifold manifold;
			if (fineCollisionDetect(body1, tile1.tile, body2, tile2.tile, manifold))
				collisionResolve(body1, body2, manifold);
		}
	}

leave:
	stats.collisionDetTotalTime += timer.getTime();
	stats.collisionDetCalled++;
}

void drawAABB(sf::RenderWindow& window, AABB<float>& aabb, sf::Color outlineColor, float thickness) {
	sf::ConvexShape shape(4);
	shape.setPoint(0, convert(aabb.bl()));
	shape.setPoint(1, convert(aabb.br()));
	shape.setPoint(2, convert(aabb.tr()));
	shape.setPoint(3, convert(aabb.tl()));
	shape.setOutlineColor(outlineColor);
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineThickness(-thickness);
	window.draw(shape);
}

int main() {
	boost::asio::thread_pool tp(4);

	float a = 0.5f;
	std::cout << "sin(a)" << glm::sin(a) << '\n';
	std::cout << "sin(-a)" << glm::sin(-a) << '\n';
	std::cout << "-sin(a)" << -glm::sin(a) << '\n';


	int testSizes[] = {
		5,
		10,
		25,
		50,
		100,
		200
	};
	int currentTest = 0;
	int maxTests = 6;
	const float testLength = 500.0; // 5 seconds

	sf::RenderWindow window(sf::VideoMode({ 800U, 600U }), "Test Field", sf::State::Windowed);
	for(; currentTest < maxTests; currentTest++) {
		TileBody body;
		TileBody body2;

		for(int y = 0; y <= testSizes[currentTest]; y++) {
			for(int x = 0; x <= testSizes[currentTest]; x++) {
				body.addTile({x, y});
			}
		}

		for (int y = 0; y <= 25; y++) {
			for (int x = 0; x <= 25; x++) {
				body2.addTile({ x, y });
			}
		}

		body.setPos({ 300, 300 });
		body.setRot(glm::two_over_pi<float>() * 0.1f);
		body2.setPos({ 400, 300 });
		body2.setRot(glm::two_over_pi<float>());

		steady_clock::time_point start = steady_clock::now();
		while(window.isOpen() && duration_cast<seconds>(steady_clock::now() - start).count() < testLength) {
			sf::Event event;
			while(window.pollEvent(event)) {
				switch(event.type) {
				case sf::Event::Closed:
					window.close();
					break;

				default:
					break;
				}
			}

			float moveSpeed = 0.1f;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::D)) {
				body.moveBy({moveSpeed, 0.0f});
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::A)) {
				body.moveBy({-moveSpeed, 0.0f});
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::W)) {
				body.moveBy({0.0f, -moveSpeed});
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::S)) {
				body.moveBy({0.0f, moveSpeed});
			}

			body.integrate(0.16f);
			body2.integrate(0.16f);
			
			body.rotateBy(glm::two_over_pi<float>() * 0.02f);

			window.clear();

			sf::ConvexShape tileShape(4);
			for (auto& tile : body.tiles) {
				auto obb = body.getTileWorldOBB(tile.first);
				tileShape.setFillColor(sf::Color::Magenta);

				for (int i = 0; i < 4; i++) {
					tileShape.setPoint(i, convert(obb[i]));
				}

				window.draw(tileShape);
			}
			for (auto& tile : body2.tiles) {
				tileShape.setFillColor(sf::Color(255 - abs(tile.first.x * 5), 255 - abs(tile.first.y * 5), abs(255 - tile.first.y * 5)));
				auto obb = body2.getTileWorldOBB(tile.first);

				for (int i = 0; i < 4; i++) {
					tileShape.setPoint(i, convert(obb[i]));
				}

				window.draw(tileShape);
			}

			drawAABB(window, body.aabb, sf::Color::Blue, 2.0f);
			drawAABB(window, body2.aabb, sf::Color::Blue, 2.0f);

			AABB approxArea;
			testCollision_04(body, body2, approxArea);

			drawAABB(window, approxArea, sf::Color(238, 210, 2), -2.5f);
			

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::Q)) {
				{
					AABB approxAreaBody1 = body.getLocalAABB(approxArea);
					std::vector<SpatialTileIndex> tiles;
					body.tilePartition.query(bgi::intersects(approxAreaBody1.getBoostBox()), std::back_inserter(tiles));
					for(auto& tile : tiles) {
						drawAABB(window, body.getTileWorldAABB(tile.tile), sf::Color::Red, -1.0f);
					}
					sf::ConvexShape shape(4);
					shape.setPoint(0, convert(body.getWorldPoint(approxAreaBody1.bl())));
					shape.setPoint(1, convert(body.getWorldPoint(approxAreaBody1.br())));
					shape.setPoint(2, convert(body.getWorldPoint(approxAreaBody1.tr())));
					shape.setPoint(3, convert(body.getWorldPoint(approxAreaBody1.tl())));
					shape.setOutlineColor(sf::Color::Green);
					shape.setFillColor(sf::Color::Transparent);
					shape.setOutlineThickness(-2.0f);
					window.draw(shape);
				}
				{
					AABB approxAreaBody2 = body2.getLocalAABB(approxArea);
					std::vector<SpatialTileIndex> tiles;
					body2.tilePartition.query(bgi::intersects(approxAreaBody2.getBoostBox()), std::back_inserter(tiles));
					for (auto& tile : tiles) {
						drawAABB(window, body2.getTileWorldAABB(tile.tile), sf::Color::Red, -1.0f);
					}
					sf::ConvexShape shape(4);
					shape.setPoint(0, convert(body2.getWorldPoint(approxAreaBody2.bl())));
					shape.setPoint(1, convert(body2.getWorldPoint(approxAreaBody2.br())));
					shape.setPoint(2, convert(body2.getWorldPoint(approxAreaBody2.tr())));
					shape.setPoint(3, convert(body2.getWorldPoint(approxAreaBody2.tl())));
					shape.setOutlineColor(sf::Color::Green);
					shape.setFillColor(sf::Color::Transparent);
					shape.setOutlineThickness(-2.0f);
					window.draw(shape);
				}
			}

			window.display();
		}

		std::cout << "Tilebody A size: " << testSizes[currentTest] * testSizes[currentTest] << '\n';
		std::cout << "Collision Detection Average(us): " << stats.collisionDetTotalTime / (float)stats.collisionDetCalled << '\n';
		std::cout << "Collision Resolution Average(us): " << stats.collisionResTotalTime / (float)stats.collisionResCalled << '\n';
	
		stats.collisionDetCalled = 1;
		stats.collisionDetTotalTime = 0.0f;
		stats.collisionResCalled = 1;
		stats.collisionResTotalTime = 0.0f;
	}

	return 0;
}