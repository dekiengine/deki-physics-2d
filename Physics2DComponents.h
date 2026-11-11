#pragma once

#include <deki/Component.h>
#include <deki/Vector.h>
#include <deki/reflection/ObjectRef.h>
#include <deki/reflection/Property.h>

#include <functional>

#include "DekiPhysics2DAPI.h"
#include "Physics2DWorld.h"

namespace Deki
{
class Object;
}

namespace DekiPhysics2D
{

/// The one world every component in the game shares.
DEKI_PHYSICS2D_API World& GetWorld();

/// World gravity and step rate. Without one, the defaults hold. Put one
/// anywhere in the scene.
DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Physics 2D Settings")
DEKI_DESCRIPTION("Gravity and step rate for the 2D physics world.")
class DEKI_PHYSICS2D_API Physics2DSettings : public Deki::Component
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Sideways gravity, in metres per second squared.")
    float gravityX = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Gravity in metres per second squared. Negative pulls down.")
    float gravityY = -9.81f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Physics steps per second. The game plays the same at any frame rate. Fewer steps cost "
                 "less, which matters on a board.")
    DEKI_RANGE(10, 240)
    int32_t stepsPerSecond = 60;

    DEKI_EXPORT
    DEKI_TOOLTIP("Bodies that have come to rest stop being simulated until something touches them. Saves time "
                 "in a big scene, and keeps stacks still.")
    bool allowSleep = true;

    void Awake() override;
};

/// Makes an object move and turn under physics. Without one, a collider is a
/// static obstacle, which follows its object if the object is moved.
DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Rigidbody 2D")
DEKI_DESCRIPTION("Moves an object with gravity, forces and collisions.")
class DEKI_PHYSICS2D_API Rigidbody2D : public Deki::Component
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Dynamic: moved by physics. Kinematic: moved by code through its velocity, pushes dynamic "
                 "bodies and is not pushed. Static: never moves.")
    BodyType type = BodyType::Dynamic;

    DEKI_EXPORT
    DEKI_TOOLTIP("How hard it is to push, in kilograms.")
    float mass = 1.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Multiplies gravity for this body: 0 floats, 2 falls twice as hard.")
    float gravityScale = 1.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("The share of its speed it loses each second, as if moving through air or water.")
    float linearDamping = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("The share of its spin it loses each second.")
    float angularDamping = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Never turns, whatever hits it. Keeps a player upright. Costs a little less on a board.")
    bool freezeRotation = false;

    DEKI_EXPORT
    DEKI_TOOLTIP("May stop being simulated once it has come to rest. Turn off for something a script needs "
                 "kept moving with no forces on it.")
    bool canSleep = true;

    DEKI_EXPORT
    DEKI_TOOLTIP("Stops at what it would otherwise jump over in one step. Costs time only while it moves fast.")
    bool continuousCollision = true;

    ~Rigidbody2D() override;
    void Start() override;
    void Update() override;

    Vec2 GetVelocity() const;
    void SetVelocity(Vec2 velocity);
    float GetAngularVelocity() const;
    void SetAngularVelocity(float radiansPerSecond);
    void AddForce(Vec2 force);
    void AddTorque(float torque);
    void AddImpulse(Vec2 impulse);
    /// At a point in the world, so it turns the body unless aimed through its centre.
    void AddImpulseAt(Vec2 impulse, Vec2 worldPoint);
    bool IsAwake() const;
    /// Wakes it and everything resting with it.
    void WakeUp();

    /// A collider of this body started or stopped hitting another object's.
    /// A body can be destroyed from inside any of these.
    std::function<void(Deki::Object* other)> onCollisionBegin;
    std::function<void(Deki::Object* other)> onCollisionEnd;

    /// The same, when either collider is a trigger.
    std::function<void(Deki::Object* other)> onTriggerBegin;
    std::function<void(Deki::Object* other)> onTriggerEnd;

    /// The body in the world, made on first use, as colliders may start first.
    BodyHandle EnsureBody();

private:
    void FollowObject();
    void MoveObject();

    BodyHandle m_Body;
    float m_ObjectX = 0.0f;  // where the object was when physics last looked
    float m_ObjectY = 0.0f;
    float m_ObjectRotation = 0.0f;
};

/// What every collider shares.
class DEKI_PHYSICS2D_API Collider2D : public Deki::Component
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Offset from the object, sideways, in metres.")
    float offsetX = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Offset from the object, upwards, in metres.")
    float offsetY = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("0 slides like ice; 1 grips like rubber.")
    DEKI_RANGE(0, 1)
    float friction = 0.3f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Bounciness: 0 stops dead, 1 keeps all its speed.")
    DEKI_RANGE(0, 1)
    float bounciness = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Reports contacts without blocking anything: a trigger zone or a pickup.")
    bool isTrigger = false;

    DEKI_EXPORT
    DEKI_TOOLTIP("The layer bits this collider is on.")
    int32_t layer = 1;

    DEKI_EXPORT
    DEKI_TOOLTIP("The layer bits it collides with. Two colliders touch only if each one's mask has the "
                 "other's layer.")
    int32_t mask = 0xFFFF;

    ~Collider2D() override;
    void Start() override;
    void Update() override;

protected:
    virtual void FillShape(ShapeDef& shape) const = 0;

private:
    ShapeHandle AddToWorld(BodyHandle body);

    ShapeHandle m_Shape;
    BodyHandle m_OwnStaticBody;  // only without a Rigidbody2D
    float m_ObjectX = 0.0f;      // what it last saw of its object, to follow it
    float m_ObjectY = 0.0f;
    float m_ObjectRotation = 0.0f;
    float m_ObjectScaleX = 1.0f;
    float m_ObjectScaleY = 1.0f;
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Box Collider 2D")
DEKI_DESCRIPTION("A box-shaped collider.")
class DEKI_PHYSICS2D_API BoxCollider2D : public Collider2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Width in metres, before the object's scale.")
    float width = 1.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Height in metres, before the object's scale.")
    float height = 1.0f;

protected:
    void FillShape(ShapeDef& shape) const override;
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Circle Collider 2D")
DEKI_DESCRIPTION("A circular collider.")
class DEKI_PHYSICS2D_API CircleCollider2D : public Collider2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Radius in metres, before the object's scale.")
    float radius = 0.5f;

protected:
    void FillShape(ShapeDef& shape) const override;
};

enum class CapsuleDirection : uint8_t
{
    Vertical = 0,
    Horizontal = 1,
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Capsule Collider 2D")
DEKI_DESCRIPTION("A collider with straight sides and round ends. Slides over small steps, which suits characters.")
class DEKI_PHYSICS2D_API CapsuleCollider2D : public Collider2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Radius of its round ends in metres, before the object's scale.")
    float radius = 0.25f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Length from end to end in metres, before the object's scale.")
    float height = 1.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("The way it lies.")
    CapsuleDirection direction = CapsuleDirection::Vertical;

protected:
    void FillShape(ShapeDef& shape) const override;
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Polygon Collider 2D")
DEKI_DESCRIPTION("A collider with up to 8 corners. A dented outline is filled in: the shape is always convex.")
class DEKI_PHYSICS2D_API PolygonCollider2D : public Collider2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("How many corners it has.")
    DEKI_RANGE(3, 8)
    int32_t pointCount = 3;

    DEKI_EXPORT Deki::Vector2 point0 = { -0.5f, -0.5f };
    DEKI_EXPORT Deki::Vector2 point1 = { 0.5f, -0.5f };
    DEKI_EXPORT Deki::Vector2 point2 = { 0.0f, 0.5f };
    DEKI_EXPORT DEKI_VISIBLE_WHEN(pointCount, 4) Deki::Vector2 point3 = { 0.0f, 0.0f };
    DEKI_EXPORT DEKI_VISIBLE_WHEN(pointCount, 5) Deki::Vector2 point4 = { 0.0f, 0.0f };
    DEKI_EXPORT DEKI_VISIBLE_WHEN(pointCount, 6) Deki::Vector2 point5 = { 0.0f, 0.0f };
    DEKI_EXPORT DEKI_VISIBLE_WHEN(pointCount, 7) Deki::Vector2 point6 = { 0.0f, 0.0f };
    DEKI_EXPORT DEKI_VISIBLE_WHEN(pointCount, 8) Deki::Vector2 point7 = { 0.0f, 0.0f };

    /// Corner `index`, 0 to 7.
    Deki::Vector2& Point(int index)
    {
        Deki::Vector2* const points[] = { &point0, &point1, &point2, &point3, &point4, &point5, &point6, &point7 };
        return *points[index];
    }
    const Deki::Vector2& Point(int index) const { return const_cast<PolygonCollider2D*>(this)->Point(index); }

protected:
    void FillShape(ShapeDef& shape) const override;
};

/// What every joint shares. It joins the Rigidbody 2D on its own object to
/// another, or to the world.
class DEKI_PHYSICS2D_API Joint2D : public Deki::Component
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("The body it joins this one to. Empty: pinned to the world, where the anchor starts.")
    Deki::ObjectRef<Rigidbody2D> connectedBody;

    DEKI_EXPORT
    DEKI_TOOLTIP("Where on this object it joins, sideways from its centre, in metres.")
    float anchorX = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Where on this object it joins, upwards from its centre, in metres.")
    float anchorY = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Lets the two joined bodies still hit each other.")
    bool collideConnected = false;

    ~Joint2D() override;
    void Start() override;

protected:
    /// Makes the joint. `other` is invalid when it is pinned to the world.
    virtual JointHandle Create(BodyHandle own, BodyHandle other, Rigidbody2D* otherBody, Vec2 worldAnchor) = 0;

private:
    JointHandle m_Joint;
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Hinge Joint 2D")
DEKI_DESCRIPTION("Pins this body to another, or to the world, at one point, free to turn: doors, chains, wheels.")
class DEKI_PHYSICS2D_API HingeJoint2D : public Joint2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Stops it turning past the two angles below.")
    bool useLimits = false;

    DEKI_EXPORT
    DEKI_VISIBLE_WHEN(useLimits, 1)
    DEKI_TOOLTIP("How far it may turn clockwise from where it starts, in degrees, as a negative number.")
    float lowerAngle = -45.0f;

    DEKI_EXPORT
    DEKI_VISIBLE_WHEN(useLimits, 1)
    DEKI_TOOLTIP("How far it may turn counter-clockwise from where it starts, in degrees.")
    float upperAngle = 45.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Drives the turn at a set speed, as far as its torque allows.")
    bool useMotor = false;

    DEKI_EXPORT
    DEKI_VISIBLE_WHEN(useMotor, 1)
    DEKI_TOOLTIP("Degrees per second, counter-clockwise.")
    float motorSpeed = 90.0f;

    DEKI_EXPORT
    DEKI_VISIBLE_WHEN(useMotor, 1)
    DEKI_TOOLTIP("The most torque the motor has, in newton metres.")
    float maxMotorTorque = 10.0f;

protected:
    JointHandle Create(BodyHandle own, BodyHandle other, Rigidbody2D* otherBody, Vec2 worldAnchor) override;
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Distance Joint 2D")
DEKI_DESCRIPTION("Keeps a point on this body a set distance from another, or from the world: rods and ropes.")
class DEKI_PHYSICS2D_API DistanceJoint2D : public Joint2D
{
public:
    DEKI_EXPORT
    DEKI_TOOLTIP("Where on the connected object it joins, sideways from its centre, in metres. With no connected "
                 "body, a point in the world.")
    float connectedAnchorX = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("Where on the connected object it joins, upwards from its centre, in metres. With no connected "
                 "body, a point in the world.")
    float connectedAnchorY = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("The distance to keep, in metres. 0: how far apart they start.")
    float length = 0.0f;

    DEKI_EXPORT
    DEKI_TOOLTIP("A rope stops them moving further apart, but lets them come closer.")
    bool rope = false;

protected:
    JointHandle Create(BodyHandle own, BodyHandle other, Rigidbody2D* otherBody, Vec2 worldAnchor) override;
};

/// The nearest collider along a segment, in world metres, with its object.
DEKI_PHYSICS2D_API bool Raycast(Vec2 from, Vec2 to, RayHit& hit, Deki::Object** object = nullptr, uint16_t mask = 0xFFFF);

}  // namespace DekiPhysics2D
