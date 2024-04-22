#pragma once

/**
 * @file base.hpp
 * 
 * @brief A primary include file for all other files. Its purpose is to 
 * contain "#include"s that are not within this project, i.e. ' <external> ' and 
 * not ' "internal" ', and define 'Float', 'vec2', and tile sizes to allow quickly
 * switching between different floating type representations, packing, and of course tile sizes.
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

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

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
	return std::isnan(v);
}

template<>
bool isNaN<vec2>(const vec2& v) {
	return std::isnan(v.x) || std::isnan(v.y);
}

T2D_NAMESPACE_END