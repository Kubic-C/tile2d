#pragma once

#include "math.hpp"

T2D_NAMESPACE_BEGIN

template<class TileData>
class World;

enum class BodyType : uint8_t {
	Tile,
	Bullet,
	Invalid
};

class WorldBody {
	template<class TileData>
	friend class World;
	friend class ResolveMethods;
public:
	WorldBody(uint32_t id, BodyType bodyType) 
		: m_id(id), m_bodyType(bodyType) {}

	uint32_t id() const {
		return m_id;
	}

	BodyType bodyType() const {
		return m_bodyType;
	}

	const Transform& transform() const {
		return m_transform;
	}

	void moveBy(const vec2& addPos) {
		m_transform.pos += addPos;
	}

	void rotateBy(Float amount) {
		m_transform.rot += amount;
		m_transform.update();
	}

	void setPos(const vec2& pos) {
		m_transform.pos = pos;
	}

	void setRot(Float rot) {
		m_transform.rot = rot;
		m_transform.update();
	}

	virtual void integrate(Float dt) = 0;

protected:
	Transform m_transform;
	BodyType m_bodyType = BodyType::Invalid;
private:
	uint32_t m_id = 0;
};

class Body : public WorldBody {
	friend class ResolveMethods;

public:
	Body(uint32_t id, BodyType type)
		: WorldBody(id, type) {}

	virtual void integrate(Float dt) override {
		assert(isValid());

		// Using the semi implicit euler method

		// A(v) = force / mass
		// V = v + A(v) * dt
		// P = p + V * dt;
		m_linearVelocity = m_linearVelocity + m_force * m_inverseMass * dt;
		m_transform.pos = m_transform.pos + m_linearVelocity * dt;
		m_force = { 0.0f, 0.0f };

		m_angularVelocity = m_angularVelocity + m_torque * m_inverseI * dt;
		m_transform.rot = m_transform.rot + m_angularVelocity * dt;
		m_torque = 0.0f;

		m_transform.update();
	}

	void addLinearVel(const vec2& vel) {
		m_linearVelocity += vel;
	}

	void addAngularVel(const Float& vel) {
		m_angularVelocity = vel;
	}

	vec2 getWorldPoint(const vec2& localPoint) const {
		return m_transform.getWorldPoint(localPoint, m_com);
	}

	vec2 getLocalPoint(const vec2& worldPoint) {
		return m_transform.getLocalPoint(worldPoint, m_com);
	}

	const vec2& com() const {
		return m_com;
	}

	const vec2& centroid() const {
		return m_centroid;
	}

	Transform& transform() {
		return m_transform;
	}

	virtual AABB<Float> getAABB() = 0;

	vec2 getWorldPos() {
		return m_transform.pos + m_com;
	}


	// for debug
	bool isValid() const {
		return
			!glm::isnan(m_linearVelocity).x &&
			!glm::isnan(m_transform.pos).x &&
			!glm::isnan(m_force).x &&
			!std::isnan(m_angularVelocity) &&
			!std::isnan(m_torque) &&
			!std::isnan(m_transform.rot);
	}

	Float mass() const {
		return m_mass;
	}

	Float rotationalInertia() const {
		return m_I;
	}

protected:
	Float m_inverseI = 0;
	Float m_inverseMass = 0;

	vec2 m_centroid = { 0.0f, 0.0f };
	vec2 m_com = { 0.0f, 0.0f };
	Float m_mass = 0.0f;
	Float m_I = 0.0f;
	vec2 m_unweightedCom = { 0.0f, 0.0f };
	vec2 m_unweightedCentroid = { 0.0f, 0.0f };

	vec2 m_linearVelocity = { 0.0f, 0.0f };
	vec2 m_force = { 0.0f, 0.0f };
	Float m_angularVelocity = 0.0f;
	Float m_torque = 0.0f;
};

class BulletBody : public WorldBody {
public:
	virtual void integrate(Float dt) override {
		m_transform.pos = m_transform.pos + m_linearVelocity * dt;
	}

private:
	uint32_t m_id;
	Float m_mass;
	vec2 m_linearVelocity;
};

T2D_NAMESPACE_END