#pragma once

#include <vector>
#include <iterator>
#include <cassert>
#include <array>
#include <set>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/pool/object_pool.hpp>
#include <glm/glm.hpp>

#include <boost/asio/io_service.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#include <bitset>

#include <stack>

#define T2D_NAMESPACE_BEGIN namespace t2d {
#define T2D_NAMESPACE_END }

using Float = float;
using vec2 = glm::vec<2, Float, glm::packed_lowp>;

T2D_NAMESPACE_BEGIN

namespace debug {

}

template<typename T>
T constexpr sqrtNewtonRaphson(T x, T curr, T prev)
{
	return curr == prev
		? curr
		: sqrtNewtonRaphson<T>(x, (T)0.5 * (curr + x / curr), curr);
}

template<typename T>
T constexpr sqrt(T x)
{
	return x >= 0 && x < std::numeric_limits<T>::infinity()
		? sqrtNewtonRaphson<T>(x, x, 0)
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

T2D_NAMESPACE_END