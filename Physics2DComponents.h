#pragma once

#include <deki/Component.h>
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

    void Awake() override;
};

/// Makes an object move under physics. Without one, a collider is a static
/// obstacle. Bodies do not rotate.
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

    ~Rigidbody2D() override;
    void Start() override;
    void Update() override;

    Vec2 GetVelocity() const;
    void SetVelocity(Vec2 velocity);
    void AddForce(Vec2 force);
    void AddImpulse(Vec2 impulse);

    /// Called when a collider of this body starts or stops touching another
    /// object's. A body can be destroyed from inside these.
    std::function<void(Deki::Object* other)> onContactBegin;
    std::function<void(Deki::Object* other)> onContactEnd;

    /// The body in the world, made on first use, as colliders may start first.
    BodyHandle EnsureBody();

private:
    BodyHandle m_Body;
};

/// What the box and circle colliders share.
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
    float restitution = 0.0f;

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

protected:
    virtual void FillShape(ShapeDef& shape) const = 0;

private:
    ShapeHandle m_Shape;
    BodyHandle m_OwnStaticBody;  // only without a Rigidbody2D
};

DEKI_CATEGORY("Physics")
DEKI_DISPLAY_NAME("Box Collider 2D")
DEKI_DESCRIPTION("A box-shaped collider that stays axis-aligned.")
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

/// The nearest collider along a segment, in world metres, with its object.
DEKI_PHYSICS2D_API bool Raycast(Vec2 from, Vec2 to, RayHit& hit, Deki::Object** object = nullptr, uint16_t mask = 0xFFFF);

}  // namespace DekiPhysics2D
