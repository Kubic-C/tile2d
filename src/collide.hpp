#pragma once

#include "tileMap.hpp"

T2D_NAMESPACE_BEGIN

enum NormalFaces : uint8_t {
	_0_1,
	_1_2,
	_2_3,
	_3_0,
	invalid,
};

constexpr std::array<NormalFaces, 4> faces = {
	_1_2,  //  |
	_2_3, //  -
	_3_0, // |
	_0_1, //  _
};

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

	bool isValid() const {
		return !glm::isnan(normal).x && !std::isnan(depth) && count <= 2;
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

// _1_2, 2_3

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

// finds the closest point on points1 to one of points2's edges
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

// computes the collision manifold between two OBBs
template<typename Container>
inline void computeBoxManifold(const Container& points1, const Container& points2, CollisionManifold& manifold) {
	/**
	 * Finding contact points happens to be extremely slow, but I have a
	 * gut feeling there is a better way to do this; so note to self
	 * optimize finding contact points
	 */

	//Float min_distance = std::numeric_limits<Float>::max();

	//findClosestPoint(manifold.points[0], manifold.points[1], manifold.count, min_distance, points1, points2);
	//findClosestPoint(manifold.points[0], manifold.points[1], manifold.count, min_distance, points2, points1);

	//assert(manifold.count != 0);

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

inline std::array<vec2, 4> tileObb1 = {};
inline std::array<vec2, 4> tileObb2 = {};

template<class TileData>
bool detectNarrowCollision(const TileBody<TileData>& body1, glm::i32vec2 tilePos1, vec2 body1Offset, const TileBody<TileData>& body2, glm::i32vec2 tilePos2, vec2 body2Offset, CollisionManifold& manifold) {
	body1.getTileWorldOBBOffsetVertices(tilePos1, body1Offset, tileObb1);
	body2.getTileWorldOBBOffsetVertices(tilePos2, body2Offset, tileObb2);

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

template<class TileData>
struct TileBodyCache {
	using SpatialIndex = typename ::t2d::TileBody<TileData>::SpatialIndex;

	std::vector<SpatialIndex> results1;
	std::vector<SpatialIndex> results2;
	std::vector<std::pair<AABB<Float>, AABB<Float>>> approxAreas;
};

// Detect and resolve a tilebody collision
template<class TileData>
void detectTileBodyCollision(
	const TileBody<TileData>& body1, 
	const TileBody<TileData>& body2, 
	TileBodyCache<TileData>& cache, 
	std::vector<CollisionManifold>& manifolds,
	vec2& body1Offset,
	vec2& body2Offset) {
	AABB<Float> approxAreaOfBody1Local = computeAABBCollisionArea(body1.getLocalAABB(), body2.getAABB(body1.transform(), body1.com()));
	if (std::isnan(approxAreaOfBody1Local.min().x))
		return;

	AABB<Float> approxAreaOfBody2Local = computeAABBCollisionArea(body2.getLocalAABB(), body1.getAABB(body2.transform(), body2.com()));
	if (std::isnan(approxAreaOfBody2Local.min().x))
		return;

	cache.results1.clear();
	cache.results2.clear();
	cache.approxAreas.clear();

	body1.queryChunks(approxAreaOfBody1Local, std::back_inserter(cache.results1));
	body2.queryChunks(approxAreaOfBody2Local, std::back_inserter(cache.results2));

#ifndef NDEBUG
	debug::CollideInfo collideInfo;
	for (auto& r1 : cache.results1)
		collideInfo.chunkAABBs.push_back(body1.getLocalAABBWorldVertices(r1.aabb));

	for (auto& r2 : cache.results2)
		collideInfo.chunkAABBs.push_back(body2.getLocalAABBWorldVertices(r2.aabb));
#endif

	for (auto& chunk1 : cache.results1) {
		for (auto& chunk2 : cache.results2) {
			AABB<Float> approxAreaOfChunk1Local = computeAABBCollisionArea(chunk1.aabb, body2.getAABB(chunk2.aabb, body1.transform(), body1.com()));
			if (std::isnan(approxAreaOfChunk1Local.min().x))
				continue;

			AABB<Float> approxAreaOfChunk2Local = computeAABBCollisionArea(chunk2.aabb, body1.getAABB(chunk1.aabb, body2.transform(), body2.com()));
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

#ifndef NDEBUG
		for (auto& r1 : cache.results1)
			collideInfo.tileAABBs.push_back(body1.getLocalAABBWorldVertices(body1.getTileLocalAABB(r1.pos)));

		for (auto& r2 : cache.results2)
			collideInfo.tileAABBs.push_back(body2.getLocalAABBWorldVertices(body2.getTileLocalAABB(r2.pos)));
#endif


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
#ifndef NDEBUG
					collideInfo.manifolds.emplace_back(manifold);
#endif
				}
			}
		}
	}

#ifndef NDEBUG
	debug::collideInfos.emplace_back(std::move(collideInfo));
#endif
}

struct ResolveMethods {
	struct Impulse {
		vec2 r1 = {Float(0.0), Float(0.0)};
		vec2 r2 = {Float(0.0), Float(0.0)};
		Float j = 0.0f;
		vec2 friction_impulse = { 0.0f, 0.0f };
	};

	// solve for new velocties for two bodies given a collision manifold
	static inline void impulseMethod(Body& body1, Body& body2, const CollisionManifold& manifold) {
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