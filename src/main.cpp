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
		float _1i = (_1.y * width) + _1.x;
		float _2i = (_2.y * width) + _2.x;

		return _1i < _2i;
	}
};

template<typename K, typename T>
using Map = std::map<K, T>;

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

inline glm::vec2 rotate(const glm::vec2& v, float sin, float cos) {
	glm::vec2 result;

	result.x = v.x * cos - v.y * sin;
	result.y = v.x * sin + v.y * cos;
	return result;
}

sf::Vector2f convert(const glm::vec2& vec) {
	return sf::Vector2f(vec.x, vec.y);
}

template<typename T = float>
struct AABB {
public:
	using vec2 = glm::vec<2, T>;

	AABB()
		: m_min(0, 0), m_max(0, 0) {}

	AABB(vec2 pos, vec2 halfDim)
		: m_min(-halfDim.x, -halfDim.y), m_max(halfDim.x, halfDim.y) {
		
		*this += pos;
	}

	AABB& operator+=(const vec2& off) {
		m_min += off;
		m_max += off;
	
		return *this;
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

	T width() const {
		return glm::abs(m_max.x - m_min.x);
	}

	T height() const {
		return glm::abs(m_max.y - m_min.y);
	}

	bool intersects(const AABB& other) const {
		glm::vec2 d1, d2;
		d1 = glm::vec2(other.m_min.x, other.m_min.y) - glm::vec2(m_max.x, m_max.y);
		d2 = glm::vec2(m_min.x, m_min.y) - glm::vec2(other.m_max.x, other.m_max.y);

		if (d1.x > 0.0f || d1.y > 0.0f)
			return false;

		if (d2.x > 0.0f || d2.y > 0.0f)
			return false;

		return true;
	}

	Box getBoostBox() const {
		return Box(Point(m_min.x, m_min.y), Point(m_max.x, m_max.y));
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

struct TileBody {
	//float minQuadTree[2] = {-100000.0f, -100000.0f};
	//float maxQuadTree[2] = { 100000.0f, 100000.0f };
public:

	static constexpr std::array<glm::vec2, 2> defaultNormals = {
		glm::vec2(1.0f, 0.0f),
		glm::vec2(0.0f, 1.0f)
	};

	TileBody() 
		: inverseI(0.0f), inverseMass(0.0f), rotSin(0.0f), rotCos(0.0f),
		mass(0.0f), I(0.0f), com(0.0f, 0.0f), unweightedCom(0.0f, 0.0f),
		pos(0.0f, 0.0f), linearVelocity(0.0f, 0.0f), force(0.0f, 0.0f),
		rot(0.0f), angularVelocity(0.0f), torque(0.0f), normals(defaultNormals) {
		cache.clear();
	}

	void integrate(float dt) {
		// Using the semi implicit euler method

		// A(v) = force / mass
		// V = v + A(v) * dt
		// P = p + V * dt;

		linearVelocity = linearVelocity + force * inverseMass * dt;
		pos = pos + linearVelocity * dt;
		force = {0.0f, 0.0f};

		angularVelocity = angularVelocity + torque * inverseI * dt;
		rot = rot + angularVelocity * dt;
		torque = 0.0f;

		rotSin = glm::sin(rot);
		rotCos = glm::cos(rot);
		normals[0] = rotate(defaultNormals[0], rotSin, rotCos); 
		normals[1] = rotate(defaultNormals[1], rotSin, rotCos);

		// https://i.stack.imgur.com/0SH6d.png
		float hWidth  = (float)cache.corners.width() * 0.5f * Tile::width + Tile::width * 0.5f;
		float hHeight = (float)cache.corners.height() * 0.5f * Tile::height + Tile::height * 0.5f;
		glm::vec2 dim = {
			hHeight * abs(rotSin) + hWidth * abs(rotCos),
			hWidth  * abs(rotSin) + hHeight * abs(rotCos)
		};
		aabb = AABB(pos + com, dim);
	}

	void addTile(glm::i32vec2 iPos, float density = 1.0f, float staticFriction = 1.0f, float dynamicFriction = 1.0f, float E = 0.5f) {
		assert(!tiles.contains(iPos));

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

	glm::vec2 getWorldPoint(const glm::vec2& localPoint) const {
		glm::vec2 rot_point = rotate(localPoint - com, rotSin, rotCos);

		return (rot_point + com) + pos;
	}

	glm::vec2 getLocalPoint(const glm::vec2& worldPoint) const {
		glm::vec2 worldPos = pos + com;

		return worldPoint - worldPos;
	}

	// converts a world aabb to a local aabb for use with colliding in the local spatial index tree
	// this means its not entirely a true local conversion as it does not take into account
	// Center of Mass.
	AABB<float> getLocalAABB(const AABB<float>& worldAABB) {
		return worldAABB - (pos);
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

		float hWidth = (float)0.5f * Tile::width;
		float hHeight = (float)0.5f * Tile::height;
		glm::vec2 dim = {
			hHeight * abs(rotSin) + hWidth * abs(rotCos),
			hWidth * abs(rotSin) + hHeight * abs(rotCos)
		};
		
		return {getWorldPoint(tilePos), dim};
	}

	// Gets aabb in local space
	AABB<float> getTileLocalAABB(glm::i32vec2 iPos) {
		glm::vec2 pos = (glm::vec2)iPos * glm::vec2(Tile::width, Tile::height);
		glm::vec2 halfTile = glm::vec2(Tile::width, Tile::height) * 0.5f;

		return {pos, halfTile};
	}

	Map<glm::i32vec2, Tile> tiles;

	glm::vec2 com;
	float rotSin;
	float rotCos;
private: 
	float inverseI;
	float inverseMass;

private:
	float mass;
	float I;
	glm::vec2 unweightedCom;

public: // integrating
	glm::vec2 pos;
	glm::vec2 linearVelocity;
	glm::vec2 force;
	float rot;
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
	bgi::rtree<SpatialTileIndex, bgi::quadratic<16, 8>, 
		SpatialTileIndexGetter, SpatialTileIndexEqualTo> tilePartition;
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
	//body1.pos -= manifold.normal * manifold.depth * 0.5f;
	//body2.pos += manifold.normal * manifold.depth * 0.5f;
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
void testCollision_04(TileBody& body1, TileBody& body2) {
	Stopwatch timer;
	timer.start();

	std::vector<SpatialTileIndex> tiles1;
	std::vector<SpatialTileIndex> tiles2;

	AABB<float> approxArea;
	if(!computeApproxCollisionArea(body1.aabb, body2.aabb, approxArea))
		goto leave;

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
	const float testLength = 5.0; // 5 seconds

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

		body.pos = { 400, 300 };
		body2.pos = { 400, 300 };

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

			float moveSpeed = 5.0f;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::D)) {
				body.pos.x += moveSpeed;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::A)) {
				body.pos.x -= moveSpeed;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::W)) {
				body.pos.y -= moveSpeed;
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::S)) {
				body.pos.y += moveSpeed;
			}

			body.integrate(0.16f);
			body2.integrate(0.16f);

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

			testCollision_04(body, body2);

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