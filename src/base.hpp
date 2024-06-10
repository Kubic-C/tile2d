#pragma once

/**
 * @file base.hpp
 * 
 * @brief A primary include file for all other files. Its purpose is to 
 * contain "#include"s that are not within this project, i.e. ' <external> ' and 
 * not ' "internal" ', and define 'Float', 'vec2', and tile sizes to allow quickly
 * switching between different floating type representations, packing, and of course tile sizes.
 */

/** @mainpage Tile2d Manual

tile2d is still a pretty small library, but no one will magically know how to use it, so instead
of spending a 20 minutes looking through source code and other documentation, look at Basics and Internals.

Related pages:
 - [Basics](_basics.html)
 - [Internals](_internals.html)
 */

 /** @page Basics
 # Basics

 tile2d's interface so far is pretty simple. Define some class/struct that represents what a tile
 contains, create a World object, create a TileMap object using createTileMap() and you have your first tile body and tile map

```cpp

#include <iostream>
#include <src/world.hpp>

struct TileData {
    std::string name;
};

int main() {
    boost::object_pool<t2d::TileMap<TileData>> tileMapPool;
    t2d::World<TileData> physicsWorld(4); // 4 is the amount of threads to use.

    std::pair<t2d::TileMap<TileData>*, t2d::WorldBody*> tileMap = t2d::createTileMap<TileData>(tileMapPool, physicsWorld);

    t2d::TileProperties<TileData> tileProperty;
    tileProperty.userData.name = "My cool tile";
    // You can also create multi tiles
    tileProperty.isMultiTile = true; // Very important that both isMultiTile and isMainTile is set to true when creating a multi tile
    tileProperty.isMainTile = true;
    tileProperty.mutliTile.width = 4;
    tileProperty.mutliTile.height = 4;

    t2d::PhysicsTileProperties physicsTileProperty;
    physicsTileProperty.density = 10.0f;
    // Range from [0, 255] will be normalized into [0.0 -> 1.0]
    physicsTileProperty.staticFriction = 100;
    physicsTileProperty.dynamicFriction = 100;
    physicsTileProperty.restitution = 100;

    // Create a 4x4 tile at position (0, 0)
    tileMap.first->addTile({ 0, 0 }, tileProperty, physicsTileProperty);
    // Theres a default parameter for the physics properties of a tile
    // tileMap.first->addTile({0, 0}, tileProperty);

    // Attempting to create a tile or a multi tile when its space is alread occupied by another will result in addTile() returning false
    bool res = tileMap.first->addTile({ 0, 0 }, tileProperty, physicsTileProperty);
    if (res) {
        std::cout << "Tile could not be created!\n";
    }

    // Removing a tile is just that easy
    tileMap.first->removeTile({ 0, 0 });

    // Tile properties can be reused and modified after creating a tile with them
    tileProperty.isMultiTile = false;

    // Using beginBulkInsert() and endBulkInsert() speed up the process
    // of inserting a large amount of tiles.
    //
    // Note: there is no upper or lower bound limit on how big a tile body can be
    // so tileMap.first.addTile({100000, 100000}, tileProperty) is perfectly fine here.
    // This does come at a moderate performance cost for lots of these large bodies
    tileMap.first->beginBulkInsert();
    for (uint32_t i = 0; i < 100; i++)
        for (uint32_t j = 0; j < 100; j++)
            tileMap.first->addTile({ i, j }, tileProperty);
    tileMap.first->endBulkInsert();

    // Add some linear velocity to the attached rigid body of the tile map
    tileMap.second->addLinearVel({ 0.0f, 100.0f });


    while (true) {
        // simulate 0.016 seconds with 6 sub steps.
        physicsWorld.update(0.016, 6);
    }

    // There also exists a beginBulkErase() and endBulkErase()
     tileMap.first->beginBulkErase();
     for(uint32_t i = 0; i < 100; i++)
      for(uint32_t j = 0; j < 100; j++)
          tileMap.first->removeTile({i, j});
     tileMap.first->endBulkErase();

    physicsWorld.destroyBody(tileMap.second);
    tileMapPool.destroy(tileMap.first);

    return 0;
}

```
 */


 /** @page Internals
 # Internals

 For developers looking to improve tile2d and need to understand how tile2d works
 this page exists. I'll only be explainig the major details such as the multi threading aspect, not a 
 line by line breakdown.

 ## The Physics World and its threads

 t2d::World<> employs a set number of threads based on the value passed through its constructor.
 Each threads has its own cache structure it uses to speed up processing of rigid bodies by reducing the calls
 to malloc() and free(). It should be noted that after the main thread has assigned work to all other threads, it will
 then assign itself a task to execute, ensuring that if the passed thread count is 0 we can still execute some work.
 The work executed is rigid body integration, reinserting rigid bodies, and doing spatial queries for a set of rigid bodies.

## Tile maps

Because of how heavily bound a TileMap and a TileBody are together, they must be essentaily created together, thus the
reason for the user having to call createTileMap() instead of somePool.constructTileMap() and world.createTileBody(someTileMap).
If the latter were to be used, it would be both a more boiler plate and bug prone approach.

*/


#include <vector>
#include <iterator>
#include <cassert>
#include <array>
#include <set>
#include <bitset>
#include <stack>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>

#ifdef WIN32
#define BOOST_MSVC 1
#endif

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/function.hpp>

// Multithreading
#include <boost/pool/object_pool.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#define T2D_NAMESPACE_BEGIN namespace t2d {
#define T2D_NAMESPACE_END }

T2D_NAMESPACE_BEGIN

using Float = float;
using vec2 = glm::vec<2, Float, glm::packed_lowp>;

namespace debug {

}

namespace impl {
	template<typename T>
	T constexpr sqrtNewtonRaphson(T x, T curr, T prev)
	{
		return curr == prev
			? curr
			: sqrtNewtonRaphson<T>(x, (T)0.5 * (curr + x / curr), curr);
	}
}

/**
 * @brief Returns the sqaure root of some constant @p x
 * @param x The number to get the constant of
 * @return The sqaure root of @p x
 */
template<typename T>
T constexpr sqrt(T x)
{
	return x >= 0 && x < std::numeric_limits<T>::infinity()
		? impl::sqrtNewtonRaphson<T>(x, x, 0)
		: std::numeric_limits<T>::quiet_NaN();
}

static constexpr Float tileWidth = 10.0f;
static constexpr Float tileHeight = 10.0f;
static constexpr vec2 tileSize = vec2(tileWidth, tileHeight);
static constexpr Float tileRadi = sqrt<Float>(tileWidth * tileWidth + tileHeight * tileHeight) * 0.5f;

static constexpr size_t chunkSideLength = 4;
static constexpr size_t chunkWidth = chunkSideLength;
static constexpr size_t chunkHeight = chunkSideLength;
static constexpr Float tcW = (Float)tileWidth * (Float)chunkWidth;
static constexpr Float tcH = (Float)tileHeight * (Float)chunkHeight;

template<typename K, typename T>
using FlatMap = boost::unordered_flat_map<K, T>;

template<typename K>
using FlatSet = boost::unordered_flat_set<K>;

template<typename T>
bool isNaN(const T& v) {
	return glm::isnan(v);
}

template<>
inline bool isNaN<vec2>(const vec2& v) {
	return glm::isnan(v.x) || glm::isnan(v.y);
}

T2D_NAMESPACE_END