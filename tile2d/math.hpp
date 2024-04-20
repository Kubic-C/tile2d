#pragma once

#include "base.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <iostream>

template<>
struct std::less<glm::i32vec2> {
	bool operator()(const glm::i32vec2& lhs, const glm::i32vec2& rhs) const {
		return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
	}
};

template<>
struct std::less<glm::vec<2, int32_t, glm::packed_lowp>> {
	bool operator()(const glm::vec<2, int32_t, glm::packed_lowp>& lhs, const glm::vec<2, int32_t, glm::packed_lowp>& rhs) const {
		return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
	}
};

template<>
struct std::hash<glm::i32vec2> {
	int32_t operator()(const glm::i32vec2& vec) const {
		const int32_t p1 = 73856093;
		const int32_t p2 = 19349663;

		return vec.x * p1 ^ vec.y * p2;
	}
};

template<>
struct boost::hash<glm::vec<2, int32_t, glm::packed_lowp>> {
	int32_t operator()(const glm::vec<2, int32_t, glm::packed_lowp>& vec) const {
		const int32_t p1 = 73856093;
		const int32_t p2 = 19349663;

		return vec.x * p1 ^ vec.y * p2;
	}
};

template<>
struct boost::hash<glm::i32vec2> {
	int32_t operator()(const glm::i32vec2& vec) const {
		const int32_t p1 = 73856093;
		const int32_t p2 = 19349663;

		return vec.x * p1 ^ vec.y * p2;
	}
};

template<>
struct std::equal_to<glm::i32vec2> {
	inline bool operator()(const glm::i32vec2& _1, const glm::i32vec2& _2) const {
		return _1 == _2;
	}
};


T2D_NAMESPACE_BEGIN

template<typename T>
T constexpr square(T x) {
	return x * x;
}

struct SinCos {
	SinCos()
		: sin(0.0f), cos(1.0f) {}

	SinCos(Float rot) {
		setRot(rot);
	}

	SinCos(Float psin, Float pcos)
		: sin(psin), cos(pcos) {}

	void setRot(Float rot) {
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

	SinCos abs() const {
		return { glm::abs(sin), glm::abs(cos) };
	}

	Float sin;
	Float cos;
};

inline vec2 rotate(const vec2& v, Float sin, Float cos) {
	vec2 result;

	result.x = v.x * cos - v.y * sin;
	result.y = v.x * sin + v.y * cos;
	return result;
}

inline vec2 rotate(const vec2& v, const SinCos& sincos) {
	return rotate(v, sincos.sin, sincos.cos);
}

struct Transform {
	Transform()
		: pos(0.0f, 0.0f), rot(0.0f) {}

	Transform(const vec2& pos, Float rot)
		: pos(pos), rot(rot) {}

	Transform getInverse() const {
		return Transform(-pos, -rot);
	}

	vec2 getWorldPoint(const vec2& localPoint) const {
		return rotate(localPoint, sincos) + pos;
	}

	vec2 getWorldPoint(const vec2& localPoint, const vec2& localOffset) const {
		return rotate(localPoint - localOffset, sincos) + localOffset + pos;
	}

	vec2 getLocalPoint(const vec2& worldPoint, const vec2& localOffset = { 0.0f, 0.0f }) const {
		vec2 localPoint = worldPoint - pos - localOffset;
		localPoint = rotate(localPoint, sincos.getNegate()) + localOffset;

		return localPoint;
	}

	// updates cache of sincos
	void update() {
		sincos.setRot(rot);
	}

	vec2  pos; // In Meters
	Float rot; // In radians
	SinCos sincos;
};

Float cross(const vec2& a, const vec2& b) {
	// a.x * b.y - b.x * a.y
	return (a.x * b.y) - (a.y * b.x);
}

inline bool overlap(Float min1, Float max1, Float min2, Float max2) {
	return (min1 <= max2 && max1 >= min2);
}

inline Float absdot(const vec2& vec1, const vec2& vec2) {
	return glm::abs(glm::dot(vec1, vec2));
}

inline vec2 reverse(const vec2& vec) {
	return { vec.y, vec.x };
}

// 0 on the line
// >0 on one side
// <0 on the other side
Float sideOf(const vec2& p, const vec2& a, const vec2& b) {
	return cross(a - b, p) + cross(b, a);
}

template<typename T = Float, glm::qualifier Prec = glm::packed_lowp>
struct AABB {
	using TAABB = AABB<T, Prec>;
public:
	using vec2 = glm::vec<2, T, Prec>;

	AABB()
		: m_min(0, 0), m_max(0, 0) {}

	AABB(vec2 min, vec2 max)
		: m_min(min), m_max(max) {}

	AABB(vec2 pos, T hW, T hH)
		: AABB({ hW, hH }) {
		*this += pos;
	}

	AABB(vec2 halfDim)
		: m_min(-halfDim.x, -halfDim.y), m_max(halfDim.x, halfDim.y) {}

	TAABB& operator+=(const vec2& off) {
		m_min += off;
		m_max += off;

		return *this;
	}

	TAABB operator+(const vec2& off) const {
		TAABB newAABB = *this;
		newAABB.m_min += off;
		newAABB.m_max += off;

		return newAABB;
	}

	TAABB operator-(const vec2& off) const {
		TAABB newAABB = *this;
		newAABB.m_min -= off;
		newAABB.m_max -= off;

		return newAABB;
	}

	TAABB operator*(T off) const {
		TAABB newAABB = *this;
		newAABB.m_min *= off;
		newAABB.m_max *= off;

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
		return { m_max.x, m_min.y };
	}

	vec2 tr() const {
		return m_max;
	}

	vec2 tl() const {
		return { m_min.x, m_max.y };
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

	AABB<T> intersectingArea(const AABB& other) const {
		AABB<T> area;

		area.setMin({
			std::max(m_min.x, other.m_min.x),
			std::max(m_min.y, other.m_min.y)
			});
		area.setMax({
			std::min(m_max.x, other.m_max.x),
			std::min(m_max.y, other.m_max.y)
			});

		return area;
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

	bool valid() {
		return m_min.x < m_max.x && m_min.y < m_max.y;
	}

private:
	vec2 m_min;
	vec2 m_max;
};

// describes a linear line (straight line) in general form i.e. Ax + By + C = 0
// can handle vertical lines, horizontal lines, etc. for intersection calculations
struct LinearGeneralForm {
	LinearGeneralForm(vec2 p1, vec2 p2) {
		a = p2.y - p1.y;
		b = -(p2.x - p1.x);
		c = p1.y * p2.x - p1.x * p2.y;
	}

	bool intersects(const LinearGeneralForm& other, vec2& intersectPoint) {
		float denom = a * other.b - other.a * b;
		if (denom == 0.0f) { // will only happen if the lines are parallel
			intersectPoint = { 0.0f, 0.0f };
			return false;
		}

		intersectPoint.x = (b * other.c - other.b * c) / denom;
		intersectPoint.y = (c * other.a - other.c * a) / denom;

		return true;
	}

	float a;
	float b;
	float c;
};

template<typename T>
T parallelAxisTheorem(T Icm, T mass, T sqDistance) {
	return Icm + mass * sqDistance;
}

// Tile OBB, designed specifically for collision detection
struct TOBB {
	Transform transform;
	vec2 extent;

	inline std::array<vec2, 4> getVertices() const {
		const vec2 wExt = rotate({ extent.x, extent.y }, transform.sincos);
		const vec2 blExt = rotate({ extent.x, -extent.y }, transform.sincos);
		const vec2 trExt = -blExt;


		return {
			 transform.pos - wExt,
			{transform.pos.x + blExt.x, transform.pos.y + blExt.y},
			 transform.pos + wExt,
			{transform.pos.x + trExt.x, transform.pos.y + trExt.y}
		};
	}
};


template<typename T>
AABB<T> computeAABBCollisionArea(const AABB<T>& _1, const AABB<T>& _2) {
	AABB<T> area = _1.intersectingArea(_2);
	if (!area.valid())
		return AABB<T>(glm::vec<2, T>(NAN), glm::vec<2, T>(NAN));

	return area;
}

inline bool nearlyEqual(Float a, Float b, Float max = 0.0001f) {
	return glm::abs(a - b) < max;
}

inline bool nearlyEqual(vec2 a, vec2 b, Float max = 0.0001f) {
	return glm::abs(a.x - b.x) < max && glm::abs(a.y - b.y) < max;
}

inline Float sqaure(Float value) {
	return value * value;
}

inline Float pointSegmentDistance(vec2 p, vec2 v1, vec2 v2, vec2& cp) {
	// credit goes to https://www.youtube.com/watch?v=egmZJU-1zPU&ab_channel=Two-BitCoding
	// for this function, incredible channel and resource

	vec2 p_to_v1 = p - v1;
	vec2 v1_to_v2 = v2 - v1;
	Float proj = glm::dot(p_to_v1, v1_to_v2);
	Float length = glm::length2(v1_to_v2);

	Float d = proj / length;

	if (d <= 0.0f) {
		cp = v1;
	}
	else if (d >= 1.0f) {
		cp = v2;
	}
	else {
		cp = v1 + v1_to_v2 * d;
	}

	return glm::distance(p, cp);
}

using MinMax = std::pair<Float, Float>;

template<class Container>
MinMax projectBoxOnNormal(const Container& vertices, const vec2& normal) {
	using float_t_prop = std::numeric_limits<Float>;
	Float firstDot = glm::dot(vertices[0], normal);
	MinMax minMax = { firstDot, firstDot };

	for (size_t i = 1; i < vertices.size(); i++) {
		Float dot = glm::dot(vertices[i], normal);

		if (dot < minMax.first) {
			minMax.first = dot;
		}
		else if (dot > minMax.second) {
			minMax.second = dot;
		}
	}

	return minMax;
}

// Vertices are expected to be in CCW order
template<class Container, class OutputContainer = std::vector<vec2>>
OutputContainer sutherlandHodgmanClip(const Container& subjectVertices, const Container& clipVertices) {
	OutputContainer outputVertices(subjectVertices.begin(), subjectVertices.end());
	OutputContainer inputVertices;

	vec2 prevClipVertex = clipVertices.back();
	for (vec2 curClipVertex : clipVertices) {
		LinearGeneralForm clipLine(prevClipVertex, curClipVertex);

		inputVertices = outputVertices;
		outputVertices.clear();

		if (inputVertices.empty())
			break;

		vec2 prevVertex = inputVertices.back();
		for (vec2 curVertex : inputVertices) {
			LinearGeneralForm subLine(prevVertex, curVertex);

			bool curVertexInside = sideOf(curVertex, prevClipVertex, curClipVertex) <= 0.0f;
			bool prevVertexInside = sideOf(prevVertex, prevClipVertex, curClipVertex) <= 0.0f;

			if (curVertexInside) {
				if (!prevVertexInside) {
					clipLine.intersects(subLine, outputVertices.emplace_back());
				}

				outputVertices.push_back(curVertex);
			}
			else if (prevVertexInside) {
				clipLine.intersects(subLine, outputVertices.emplace_back());
			}


			prevVertex = curVertex;
		}

		prevClipVertex = curClipVertex;
	}

	return outputVertices;
}

inline bool circleCollide(Float radi1, const vec2& pos1, Float radi2, const vec2& pos2) {
	Float difOfX = (pos2.x - pos1.x);
	Float difOfY = (pos2.y - pos1.y);
	Float sumOfRadi = radi1 + radi2;

	return square(difOfX) + square(difOfY) <= square(sumOfRadi);
}

// puts num in the range of [0.0, 1.0]
inline Float convertToFloat(uint8_t num) {
	constexpr Float inv255 = 1.0f / 255.0f;
	return (Float)num * inv255;
}

template<typename Tint>
Tint constexpr getSqrtMagicNumber() {
	static_assert(!(std::is_same_v<Tint, std::int64_t> || std::is_same_v<Tint, std::int32_t>));
	return Tint(0);
}

template<>
constexpr std::int32_t getSqrtMagicNumber() {
	return 0x5f3759df;
}

template<>
constexpr std::int64_t getSqrtMagicNumber() {
	return 0x5fe6eb50c7b537a9;
}

// Q_rsqrt
template<typename T>
inline T fastInverseSqrt(T x) {
	static_assert(std::is_floating_point<T>::value, "T must be floating point");
	using Tint = typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type;
	constexpr Tint magicNumber = getSqrtMagicNumber<Tint>();

	T y = x;
	T x2 = y * 0.5;
	Tint i = *(Tint*)&y; // *(Tint *)&y has strict-aliasing UB. Use memcpy, or C++20 std::bit_cast ???
	i = magicNumber - (i >> 1);
	y = *(T*)&i;
	y = y * (1.5 - (x2 * y * y));
	y = y * (1.5 - (x2 * y * y)); // 2nd iteration, not *needed* but helps with precision

	return y;
}

//inline vec2 normalize(const vec2& vec) {
//	vec2 normal = vec;
//	Float invSqrtMag = fastInverseSqrt<Float>(glm::dot(vec, vec));
//	normal.x *= invSqrtMag;
//	normal.y *= invSqrtMag;
//	return normal;
//}

template<typename Int, typename Float>
inline Int floor(Float x) {
	Int i = (Int)x;
	return i - (i > x);
}

T2D_NAMESPACE_END