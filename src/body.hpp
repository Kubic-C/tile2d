#pragma once

#include "math.hpp"

/**
 * @file body.hpp
 *
 * @brief Contains different types of rigid bodies aside from TileBody. See tileMap.hpp
 */

T2D_NAMESPACE_BEGIN

struct CollisionManifold;

template<class Access, class TileData>
class World;

/**
 * @enum BodyType
 */
enum class BodyType : uint8_t {
	Tile,
	Bullet,
	Invalid
};

enum FlagIndexes {
	NEEDS_REINSERT = 0, // needs to be reinserted into the spatial tree or grid.
	IS_STATIC = 1 // the body is static
};

/**
 * @class WorldBody
 * 
 * @brief The abstract base class of all Rigid Bodies that defines essential methods and members to be used in t2d::World<>
 */
template<typename Access>
class WorldBody {
	template<class Access, class TileData>
	friend class World;
	friend class ResolveMethods;
public:
	using CollisionCallback = boost::function<void(WorldBody* self, WorldBody* other, CollisionManifold* manifold)>;

	/**
	 * @brief constructs a new WorldBody
	 * 
	 * @param id The id assigned by physics world
	 * @param bodyType The type of body, see enum BodyType. Does not define whether or not the body is static, see setStatic()
	 */
	WorldBody(const Access& access, uint32_t id, BodyType bodyType) 
		: m_transform(access), m_id(id), m_bodyType(bodyType) {}

	/**
	 * @brief Returns the id assigned to this body the physics world
	 * @returns The id assigned to this body the physics world
	 */
	uint32_t id() const {
		return m_id;
	}

	/**
	 * @brief Returns the bodyType of this body, see enum BodyType
	 * @returns The bodyType of this body, see enum BodyType
	 */
	BodyType bodyType() const {
		return m_bodyType;
	}

	/**
	 * @brief Returns the REFERENCE transform of this body
	 * @return The REFERENCE tranfsorm of this body
	 */
	Transform& transform() {
		return *m_transform;
	}

	/**
	 * @brief Returns the CONST REFERENCE transform of this body
	 * @return The CONST REFERENCE tranfsorm of this body
	 */
	const Transform& transform() const {
		return *m_transform;
	}

	/**
	 * @brief Moves the body by @p amount
	 * 
	 * @param amount The amount to move this body by
	 */
	void moveBy(const vec2& amount) {
		m_flags[NEEDS_REINSERT] = true;
		m_transform->pos += amount;
	}

	/**
	 * @brief Rotates the body by @p amount
	 *
	 * @param amount The amount to rotate this body by
	 */
	void rotateBy(Float amount) {
		m_flags[NEEDS_REINSERT] = true;
		m_transform->rot += amount;
		m_transform->update();
	}

	/**
	 * @brief Sets the position of this body to @p pos
	 *
	 * @param pos The position to set this body to
	 */
	void setPos(const vec2& pos) {
		m_flags[NEEDS_REINSERT] = true;
		m_transform->pos = pos;
	}

	/**
	 * @brief Sets the rotation of this body to @p rot
	 *
	 * @param rot The rotation to set this body to
	 */
	void setRot(Float rot) {
		m_flags[NEEDS_REINSERT] = true;
		m_transform->rot = rot;
		m_transform->update();
	}

	/**
	 * @brief Integerates this body, meaning the linear velocity and angular velocity will be integrated into the position and rotation of this body
	 *
	 * @param dt The amount to integrate by; delta time
	 */
	virtual void integrate(Float dt) = 0;

	/**
	 * @brief Returns if the body is static, meaning it cannot be moved by forces or other bodies
	 * 
	 * @return True, if static
	 */
	bool isStatic() const {
		return m_flags[IS_STATIC];
	}

	/**
	 * @brief Either make the body static or dynamic.
	 * 
	 * @param static_ If true the body will be made static. See isStatic(). If false the body will be made dynamic.
	 */
	virtual void setStatic(bool static_) {
		m_flags[IS_STATIC] = static_;
	}

	/**
	 * @brief Gets a local position thats within the local space of this body and transforms into the world space.
	 *
	 * @param localPoint Position in the local space of this body
	 */
	virtual vec2 getWorldPoint(const vec2& localPoint) const {
		return m_transform->getWorldPoint(localPoint);
	}

	/**
	 * @brief Gets a world position thats space of this body and transforms into the local space of this body.
	 *
	 * @param worldPoint Position in world pos
	 */
	virtual vec2 getLocalPoint(const vec2& worldPoint) const {
		return m_transform->getLocalPoint(worldPoint);
	}

	/**
	 * @brief Returns the world position of this body
	 *
	 * @return the world position of this body
	 */
	virtual vec2 getWorldPos() {
		return m_transform->pos;
	}

	/**
	 * @brief applies a linear velocity to this body
	 * 
	 * @param vel The linear velocity to apply to this body
	 */
	void addLinearVel(const vec2& vel) {
		m_linearVelocity += vel * (Float)!m_flags[IS_STATIC];
	}

	/**
	 * @brief Returns the mass of this body
	 * 
	 * @return The mass of this body
	 */
	Float mass() const {
		return m_mass;
	}

	/**
	 * @brief Returns the inverse mass of this body
	 *
	 * @return The inverse mass of this body
	 */
	Float inverseMass() const {
		return m_inverseMass;
	}

	/**
	 * @brief Returns the AABB of this body
	 *
	 * @return The AABB of this body
	 */
	virtual AABB<Float> getLocalAABB() const = 0;

	/**
	 * @brief Returns the AABB of this body
	 * 
	 * @return The AABB of this body
	 */
	virtual AABB<Float> getAABB() const = 0;

	// get the local AABB (of the entire object) in the localSpace of spaceTransform
	inline virtual AABB<Float> getAABB(const Transform& spaceTransform, const vec2& localOffset = { Float(0.0f), Float(0.0f) }) const {
		return getAABB(getLocalAABB(), spaceTransform, localOffset);
	}

	// get the some local AABB in the localSpace of spaceTransform
	inline virtual AABB<Float> getAABB(const AABB<Float>& localAABB, const Transform& spaceTransform, const vec2& localOffset = { Float(0.0f), Float(0.0f) }) const {
		vec2 relPos = spaceTransform.getLocalPoint(getWorldPoint(localAABB.midpoint()), localOffset);
		return AABB<Float>(relPos, localAABB.width() * 0.5f, localAABB.height() * 0.5f).rotate(spaceTransform.sincos - m_transform->sincos);
	}

	/**
	 * @brief Sets the Collision group of this body.
	 * Bodies with the same bits set in their collision group mask, will not collide.
	 *
	 * @param mask The new Collision group mask of body
	 */
	void setCollisionGroup(uint32_t mask) {
		m_collideGroup = mask;
	}

	/**
	 * @brief OR's with the current Collision group of this body.
	 * Bodies with the same bits set in their collision group mask, will not collide.
	 *
	 * @param mask The new Collision group mask of body to perform an OR ( | ) operation with
	 */
	void orCollisionGroup(uint32_t orMask) {
		m_collideGroup |= orMask;
	}

	/**
	 * @brief Returns the collision group of this body
	 * Bodies with the same bits set in their collision group mask, will not collide.
	 *
	 * @returns The collision group
	 */
	uint32_t collisionGroup() const {
		return m_collideGroup;
	}

	/**
	 * @brief Sets the collision callback for this body
	 */
	void setCollisionCallback(CollisionCallback&& callback) {
		m_collisionCallback = callback;
	}

	/**
	 * @brief Returns the collision callback for this body
	 * 
	 * @returns the collision callback for this body
	 */
	const CollisionCallback& collisionCallback() {
		return m_collisionCallback;
	}

protected:
	Access m_transform;
	Float m_mass = 0.0f;
	Float m_inverseMass = 0;
	vec2 m_linearVelocity = { 0.0f, 0.0f };
	CollisionCallback m_collisionCallback;

protected:
	uint32_t m_collideGroup = 0;
	std::bitset<4> m_flags;
	BodyType m_bodyType = BodyType::Invalid;
private:
	uint32_t m_id = 0;
};

/**
 * @class Body
 * 
 * @brief Extends the functionality of WorldBody to allow for things such as angular velocity
 */
template<typename Access>
class Body : public WorldBody<Access> {
	friend class ImpulseMethod;
	template<class Access, class TileData>
	friend class World;

public:
	/**
	 * @brief constructs a new WorldBody
	 *
	 * @param id The id assigned by physics world
	 * @param bodyType The type of body, see enum BodyType. Does not define whether or not the body is static, see setStatic()
	 */
	Body(const Access& access, uint32_t id, BodyType type)
		: WorldBody<Access>(access, id, type) {}

	virtual void integrate(Float dt) override {
		// Using the semi implicit euler method

		// A(v) = force / mass
		// V = v + A(v) * dt
		// P = p + V * dt;
		this->m_linearVelocity = this->m_linearVelocity + m_force * this->m_inverseMass * dt;
		vec2 newPos = this->m_transform->pos + this->m_linearVelocity * dt;
		this->m_force = { 0.0f, 0.0f };

		this->m_angularVelocity = this->m_angularVelocity + this->m_torque * this->m_inverseI * dt;
		Float rot = this->m_transform->rot + this->m_angularVelocity * dt;
		this->m_torque = 0.0f;

		this->m_transform->update();

		if(!nearlyEqual(newPos, this->m_transform->pos, 0.1f) || !nearlyEqual(rot, this->m_transform->rot, 0.05f))
			this->m_flags[NEEDS_REINSERT] = true;

		this->m_transform->pos = newPos;
		this->m_transform->rot = rot;
	}

	virtual vec2 getWorldPoint(const vec2& localPoint) const override {
		return this->m_transform->getWorldPoint(localPoint, m_com);
	}

	virtual vec2 getLocalPoint(const vec2& worldPoint) const override {
		return this->m_transform->getLocalPoint(worldPoint, m_com);
	}

	virtual vec2 getWorldPos() override {
		return this->m_transform->pos + m_com;
	}

	virtual void setStatic(bool static_) override {
		WorldBody<Access>::setStatic(static_);

		if (static_) {
			this->m_inverseI = 0;
			this->m_inverseMass = 0;
			this->m_linearVelocity = vec2(0);
			this->m_angularVelocity = 0;
			this->m_com = { 0, 0 };
		}
		else {
			this->m_inverseI = (Float)1 / this->m_I;
			this->m_inverseMass = (Float)1 / this->m_mass;
			this->m_com = this->m_unweightedCom * this->m_inverseMass;
		}
	}

	/**
	 * @brief Applies an angular velocity to this body
	 *
	 * @param vel The amount of angular velocity to apply
	 */
	void addAngularVel(const Float& vel) {
		this->m_angularVelocity = vel * (Float)!this->m_flags[IS_STATIC];
	}

	/**
	 * @brief Returns the center of mass of this body
	 * 
	 * @return The center of mass of this body
	 */
	const vec2& com() const {
		return this->m_com;
	}

	/**
	 * @brief Returns the centroid of this body
	 *
	 * @return The centroid of this body
	 */
	const vec2& centroid() const {
		return this->m_centroid;
	}

	/**
	 * @brief Returns rotational inertia scalar of this body
	 *
	 * @return The rotational inertia scalar of this body
	 */
	Float rotationalInertia() const {
		return this->m_I;
	}

protected:
	Float m_inverseI = 0;

	vec2 m_centroid = { 0.0f, 0.0f };
	vec2 m_com = { 0.0f, 0.0f };
	Float m_I = 0.0f;
	vec2 m_unweightedCom = { 0.0f, 0.0f };
	vec2 m_unweightedCentroid = { 0.0f, 0.0f };

	vec2 m_force = { 0.0f, 0.0f };
	Float m_angularVelocity = 0.0f;
	Float m_torque = 0.0f;
};

template<typename Access>
class BulletBody : public WorldBody<Access> {
public:
	BulletBody(const Access& access, uint32_t id, Float radius)
		: WorldBody<Access>(access, id, BodyType::Bullet), m_radius(radius) {}

	virtual void integrate(Float dt) override {
		this->m_transform->pos = this->m_transform->pos + this->m_linearVelocity * dt;
	}

	virtual AABB<Float> getAABB() const {
		const Float m_aabbRadius = this->m_radius * sqrt<Float>(2.0f);

		return AABB<Float>(this->m_transform->pos, m_aabbRadius, m_aabbRadius);
	}

private:
	Float m_radius;
	Float m_mass = 0.0f;
};

T2D_NAMESPACE_END