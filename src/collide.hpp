#pragma once

#include "tileMap.hpp"

/**
 * @file collide.hpp
 *
 * @brief Contains different types of collision detection and resolve functions
 */

T2D_NAMESPACE_BEGIN

/**
 * @class CollisionManifold
 * 
 * @brief Defines the relevant collision information generated
 * by detectNarrowColllision() to both resolve the collision through its
 * normals and impulse.
 */
struct CollisionManifold {
	CollisionManifold() {}

	CollisionManifold(const CollisionManifold& other) {
		normal = other.normal;
		depth = other.depth;

		count = other.count;
		for (size_t i = 0; i < other.count; i++) {
			points[i] = other.points[i];
		}

		staticFriction = other.staticFriction;
		dynamicFriction = other.dynamicFriction;
		restitution = other.restitution;

	}

	vec2 normal = { Float(0.0), Float(0.0) };
	Float depth = std::numeric_limits<Float>::max();

	size_t count = 0;
	std::array<vec2, 2> points;

	Float staticFriction = Float(1.0);
	Float dynamicFriction = Float(1.0);
	Float restitution = Float(0.0);

	/**
	 * @brief If any members are NaN, normal is not normalized, or the contact count is more then 2, the collision manifold is not valid.
	 * 
	 * @return True if valid
	 */
	bool isValid() const {
		return !isNaN(normal) && 
			   !isNaN(depth) && 
			   count <= 2 && 
			   !isNaN(staticFriction) &&
			   !isNaN(dynamicFriction) &&
			   !isNaN(restitution);
	}
};

namespace debug {
	struct CollideInfo {
		std::vector<CollisionManifold> manifolds;
		std::vector<std::array<vec2, 4>> tileAABBs;
		std::vector<std::array<vec2, 4>> chunkAABBs;
	};

	std::vector<CollideInfo> collideInfos;
}

/**
 * @brief Returns the normal and depth of two polygons defined by the vertices
 * @p vertices1 and @p vertices2 by using @p normals inside of the @p manifold
 * 
 * @details The size of vertices1 and vertices2 must be 4. The size of normals
 * must be 2.
 * 
 * @param[in] vertices1 The first set of vertices that define a convex polygon
 * @param[in] vertices2 The second set of vertices that define a convex polygon
 * @param[out] manifold If satBoxTest() returns true, it will contain the normal and depth
 * of penetration between vertices1 and vertices2 inside of its members, 'normal' and 'depth'
 * 
 * @return Will return true if there was a collision detected, false otherwise
 */
template<class BoxVertexContainer, class NormalContainer>
bool satBoxTest(const BoxVertexContainer& vertices1, const BoxVertexContainer& vertices2, const NormalContainer& normals, CollisionManifold& manifold) {
	assert(normals.size() == 2);
	assert(vertices1.size() == 4);
	assert(vertices2.size() == 4);

	for (size_t i = 0; i < normals.size(); i++) {

		const vec2& normal = normals[i];

		MinMax proj1 = projectBoxOnNormal(vertices1, normal);
		MinMax proj2 = projectBoxOnNormal(vertices2, normal);

		if (!(proj1.second >= proj2.first && proj2.second >= proj1.first)) {
			// they are not colliding
			return false;
		}
		else {
			Float newDepth = std::max(Float(0.0), std::min(proj1.second, proj2.second) - std::max(proj1.first, proj2.first));
			if (newDepth <= manifold.depth) {
				manifold.normal = normal;
				manifold.depth = newDepth;

				// we check to see if this value is on the left or right side of proj1
				Float direction = proj1.second - proj2.second;
				if (direction > 0) {
					manifold.normal *= -1.0f;
				}
			}
		}

	}

	return true;
};


/**
 * @brief Finds the closest point between points1 and the faces defined by points2
 *
 * @details The size of vertices1 and vertices2 must be 4. The size of normals
 * must be 2.
 *
 * @param[in, out] cp1 The first closest point between points1 and points2
 * @param[in, out] cp2 The second closest point between points1 and points2, may not exist
 * @param[in, out] count The count of closest points between points1 and points2
 * @param[in, out] distance1 Contains the minimum distance between points1 and faces defined by points2
 * @param[in] points1 The set of points used to find the closest point in the faces defined by points2
 * @param[in] points2 The set of points that define faces that points1 will check against
 */
template<typename Container, typename Integer>
inline void findClosestPoint(vec2& cp1, vec2& cp2, Integer& count, Float& distance1,
	const Container& points1, const Container& points2) {
	for (Integer i = 0; i < points1.size(); i++) {
		vec2 p = points1[i];

		for (Integer j = 0; j < points2.size(); j++) {
			vec2 cur = points2[j];
			vec2 next = points2[(j + 1) % points2.size()];

			vec2 cp;
			Float distance = pointSegmentDistance(p, cur, next, cp);

			if (nearlyEqual(distance, distance1, 0.1f)) {
				if (!nearlyEqual(cp, cp1, 0.005f)) {
					count = 2;
					cp2 = cp;
				}
			}
			else if (distance <= distance1) {
				count = 1;
				cp1 = cp;
				distance1 = distance;
			}
		}
	}
}

/**
 * @brief Finds the closest point between points1 and the faces defined by points2
 *
 * @details The size of vertices1 and vertices2 must be 4. The size of normals
 * must be 2.
 *
 * @param[in] points1 The set of points used to find the closest point in the faces defined by points2
 * @param[in] points2 The set of points that define faces that points1 will check against
 * @param[in, out] manifold The normal and depth must be already calculated within manifold. The contact points betweeen the shapes defined by points1 and points2 will be written inside of manifold
 */
template<typename Container>
inline void computeBoxManifold(const Container& points1, const Container& points2, CollisionManifold& manifold) {
	std::array<vec2, 2> edge1;
	std::array<vec2, 2> edge2;

	// Face 1, finding the edge, whom's normals are most parallel to the manifold normal
	Float max = -std::numeric_limits<Float>::max();
	for (size_t i = 0; i < 4; i++) {
		const vec2 p1 = points1[i];
		const vec2 p2 = points1[(i + 1) % 4];
		const vec2 edge = p2 - p1;
		const vec2 normal = { -edge.y, edge.x };

		const Float dot = glm::dot(normal, -manifold.normal);
		if (dot > max) {
			max = dot;
			edge1[0] = p1;
			edge1[1] = p2;
		}
	}
	
	// Face 2, finding the edge, whom's normals are most parallel but against the manifold normal
	max = -std::numeric_limits<Float>::max();
	for (size_t i = 0; i < 4; i++) {
		const vec2 p1 = points2[i];
		const vec2 p2 = points2[(i + 1) % 4];
		const vec2 edge = p2 - p1;
		const vec2 normal = { -edge.y, edge.x };

		const Float dot = glm::dot(normal, manifold.normal);
		if (dot > max) {
			max = dot;
			edge2[0] = p1;
			edge2[1] = p2;
		}
	}

	Float minDist = std::numeric_limits<Float>::max();
	findClosestPoint(manifold.points[0], manifold.points[1], manifold.count, minDist, edge1, edge2);

	assert(manifold.count >= 1);
}

/**
 * @brief Detects the collision between the tile in @p tilePos1 of @p body1 and the tile @p tilePos2 of body2
 *
 * @details In order to not modify body1 and body2, detectTileBodyCollision() uses offsets that are applied to body1 and body2 that would correctly resolve the collision between the two bodies. The offsets after each call to detectNarrowCollision() will have "normal * depth" added to them.
 * 
 * @param[in] body1 The first body to grab the tile data defined by @p tilePos1
 * @param[in] tilePos1 The tile position of @p body1 to check against @p tilePos2
 * @param[in] body1Offset A world offset to apply to the OBB of the tile defined by @p tilePos1 of @p body1
 * @param[in] body2 The second body to grab the tile data defined by @p tilePos2
 * @param[in] tilePos2 The tile position of @p body2 to check against @p tilePos1
 * @param[in] body2Offset A world offset to apply to the OBB of the tile defined by @p tilePos2 of @p body2
 * @param[out] manifold If detectNarrowCollision() returns true, Will contain a set of contact points and the normal and depth that may be
 * applied to body1 and body2 to resolve the collision of the tiles defined by tilePos1 and tilePos2
 * 
 * @return If true, the collision between the tile of tilePos1 of body1 and the tile of tilePos2 of body2 is occuring. 
 */
template<class TileData>
bool detectNarrowCollision(
	const TileBody<TileData>& body1, 
	const glm::i32vec2& tilePos1, 
	const vec2& body1Offset, 
	const TileBody<TileData>& body2, 
	const glm::i32vec2& tilePos2, 
	const vec2& body2Offset, 
	CollisionManifold& manifold) {
	std::array<vec2, 4> tileObb1 = body1.getTileWorldOBBOffsetVertices(tilePos1, body1Offset);
	std::array<vec2, 4> tileObb2 = body2.getTileWorldOBBOffsetVertices(tilePos2, body2Offset);

	{
		auto normals1 = body1.normals();
		if (!satBoxTest(tileObb1, tileObb2, normals1, manifold))
			return false;

		auto normals2 = body2.normals();
		if (!satBoxTest(tileObb1, tileObb2, normals2, manifold))
			return false;
	}

	if (manifold.depth == 0.0f)
		return false;

	computeBoxManifold(tileObb1, tileObb2, manifold);

	assert(manifold.isValid());

	const PhysicsTileProperties& tile1 = body1.tileMap().getPhysicsTileProperties(tilePos1);
	const PhysicsTileProperties& tile2 = body2.tileMap().getPhysicsTileProperties(tilePos2);
	manifold.dynamicFriction = convertToFloat(tile1.dynamicFriction) * convertToFloat(tile2.dynamicFriction);
	manifold.staticFriction = convertToFloat(tile1.staticFriction) * convertToFloat(tile2.staticFriction);
	manifold.restitution = convertToFloat(std::max(tile1.restitution, tile2.restitution));

	return true;
}

/**
 * @class TileBodyCollisionCache
 * @brief Used to speed up the collision detection between two tile bodies
 */
template<class TileData>
struct TileBodyCollisionCache {
	using SpatialIndex = typename ::t2d::TileBody<TileData>::SpatialIndex;

	std::vector<SpatialIndex> results1;
	std::vector<SpatialIndex> results2;
	std::vector<std::pair<AABB<Float>, AABB<Float>>> approxAreas;
};

/**
 * @brief Detects the collision between the tiles bodies, body1 and body2.
 *
 * @details Uses chunking as a broad phase to speed up the collision of extremely large tile bodies
 *
 * @param[in] body1 The first tile body
 * @param[in] body2 The second tile body
 * @param[in, out] cache In order to speed up collisions, a cache is used to lessen calls to malloc() and free()
 * @param[out] manifolds the contact manifolds between the tiles of body1 and body2
 * @param[out] body1Offset A world offset to apply to body1 to resolve the collision
 * @param[out] body2Offset A world offset to apply to body2 to resolve the collision
 */
template<class TileData>
void detectTileBodyCollision(
	const TileBody<TileData>& body1, 
	const TileBody<TileData>& body2, 
	TileBodyCollisionCache<TileData>& cache, 
	std::vector<CollisionManifold>& manifolds,
	vec2& body1Offset,
	vec2& body2Offset) {
	AABB<Float> approxAreaOfBody1Local = computeAABBCollisionArea(body1.getLocalAABB(), body2.Body::getAABB(body1.transform(), body1.com()));
	if (std::isnan(approxAreaOfBody1Local.min().x))
		return;

	AABB<Float> approxAreaOfBody2Local = computeAABBCollisionArea(body2.getLocalAABB(), body1.Body::getAABB(body2.transform(), body2.com()));
	if (std::isnan(approxAreaOfBody2Local.min().x))
		return;

	cache.results1.clear();
	cache.results2.clear();
	cache.approxAreas.clear();

	body1.queryChunks(approxAreaOfBody1Local, std::back_inserter(cache.results1));
	body2.queryChunks(approxAreaOfBody2Local, std::back_inserter(cache.results2));

	for (auto& chunk1 : cache.results1) {
		for (auto& chunk2 : cache.results2) {
			AABB<Float> approxAreaOfChunk1Local = computeAABBCollisionArea(chunk1.aabb, body2.Body::getAABB(chunk2.aabb, body1.transform(), body1.com()));
			if (std::isnan(approxAreaOfChunk1Local.min().x))
				continue;

			AABB<Float> approxAreaOfChunk2Local = computeAABBCollisionArea(chunk2.aabb, body1.Body::getAABB(chunk1.aabb, body2.transform(), body2.com()));
			if (std::isnan(approxAreaOfChunk2Local.min().x))
				continue;

			cache.approxAreas.push_back(std::pair(approxAreaOfChunk1Local, approxAreaOfChunk2Local));
		}
	}

	for (const auto& approxArea : cache.approxAreas) {
		cache.results1.clear();
		cache.results2.clear();

		body1.queryTiles(approxArea.first, std::back_inserter(cache.results1));
		body2.queryTiles(approxArea.second, std::back_inserter(cache.results2));


		for (auto& tile1 : cache.results1) {
			for (auto& tile2 : cache.results2) {
				CollisionManifold manifold;

				if (detectNarrowCollision(body1, tile1.pos, body1Offset, body2, tile2.pos, body2Offset, manifold)) {
					Float body1Mass = body1.mass() * (Float)!body1.isStatic();
					Float body2Mass = body2.mass() * (Float)!body2.isStatic();
					Float totalMass = body1Mass + body2Mass;
					Float body1Depth = (body1Mass / totalMass) * manifold.depth;
					Float body2Depth = (body2Mass / totalMass) * manifold.depth;

					body1Offset -= manifold.normal * body1Depth * Float(0.5);
					body2Offset += manifold.normal * body2Depth * Float(0.5);

					manifolds.emplace_back(manifold);
				}
			}
		}
	}
}

/**
 * @class ImpulseMethod
 *
 * @brief Resolves the collision between two bodies using the impulse method
 *
 * @param[out] body1 The first tbody to resolve
 * @param[out] body2 The second body to resolve
 * @param[in] manifolds Must contain the depth, normal, and collision points between body1 and body2.
 */
struct ImpulseMethod {
	struct Impulse {
		vec2 r1 = {Float(0.0), Float(0.0)};
		vec2 r2 = {Float(0.0), Float(0.0)};
		Float j = 0.0f;
		vec2 friction_impulse = { 0.0f, 0.0f };
	};

	void operator()(Body& body1, Body& body2, const CollisionManifold& manifold) {
		Impulse impulse;
		vec2 average = manifold.points[0];

		if (manifold.count == 2) {
			average += manifold.points[1];
			average /= 2.0f;
		}

		{ // normal calculations
			impulse.r1 = body1.getWorldPos() - average;
			impulse.r2 = body2.getWorldPos() - average;

			vec2 r1_perp = { impulse.r1.y, -impulse.r1.x };
			vec2 r2_perp = { impulse.r2.y, -impulse.r2.x };

			vec2 angular_linear1 = r1_perp * body1.m_angularVelocity;
			vec2 angular_linear2 = r2_perp * body2.m_angularVelocity;

			vec2 rel_vel =
				(body2.m_linearVelocity + angular_linear2) -
				(body1.m_linearVelocity + angular_linear1);

			Float rel_vel_dot_n = glm::dot(rel_vel, manifold.normal);
			if (rel_vel_dot_n > 0.0f)
				return;

			Float r1_perp_dot_n = glm::dot(r1_perp, manifold.normal);
			Float r2_perp_dot_n = glm::dot(r2_perp, manifold.normal);

			Float denom =
				body1.m_inverseMass + body2.m_inverseMass +
				(sqaure(r1_perp_dot_n) * body1.m_inverseI) +
				(sqaure(r2_perp_dot_n) * body2.m_inverseI);

			impulse.j = -(1.0f + manifold.restitution) * rel_vel_dot_n;
			impulse.j /= denom;
			impulse.j /= (Float)manifold.count;

			// apply impulse
			vec2 impulsej = impulse.j * manifold.normal;

			body1.m_linearVelocity -= impulsej * body1.m_inverseMass;
			body1.m_angularVelocity -= cross(impulsej, impulse.r1) * body1.m_inverseI;
			body2.m_linearVelocity += impulsej * body2.m_inverseMass;
			body2.m_angularVelocity += cross(impulsej, impulse.r2) * body2.m_inverseI;
		}

		{ // tangent calculations
			vec2 r1_perp = { impulse.r1.y, -impulse.r1.x };
			vec2 r2_perp = { impulse.r2.y, -impulse.r2.x };

			vec2 angular_linear1 = r1_perp * body1.m_angularVelocity;
			vec2 angular_linear2 = r2_perp * body2.m_angularVelocity;

			vec2 rel_vel =
				(body2.m_linearVelocity + angular_linear2) - (body1.m_linearVelocity + angular_linear1);

			vec2 tangent = rel_vel - glm::dot(rel_vel, manifold.normal) * manifold.normal;
			if (nearlyEqual(tangent, { 0.0f, 0.0f }))
				return;
			else
				tangent = glm::normalize(tangent);

			Float r1_perp_dot_t = glm::dot(r1_perp, tangent);
			Float r2_perp_dot_t = glm::dot(r2_perp, tangent);

			Float denom =
				body1.m_inverseMass + body2.m_inverseMass +
				(sqaure(r1_perp_dot_t) * body1.m_inverseI) +
				(sqaure(r2_perp_dot_t) * body2.m_inverseI);

			Float jt = -glm::dot(rel_vel, tangent);
			jt /= denom;
			jt /= manifold.count;

			if (glm::abs(jt) <= impulse.j * manifold.staticFriction) {
				impulse.friction_impulse = jt * tangent;
			}
			else {
				impulse.friction_impulse = -impulse.j * tangent * manifold.dynamicFriction;
			}

			body1.m_linearVelocity -= impulse.friction_impulse * body1.m_inverseMass;
			body1.m_angularVelocity -= cross(impulse.friction_impulse, impulse.r1) * body1.m_inverseI;

			body2.m_linearVelocity += impulse.friction_impulse * body2.m_inverseMass;
			body2.m_angularVelocity += cross(impulse.friction_impulse, impulse.r2) * body2.m_inverseI;
		}
	}
};

T2D_NAMESPACE_END