#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "DekiPhysics2DAPI.h"

namespace DekiPhysics2D
{

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;
};

enum class BodyType : uint8_t
{
    Static = 0,     // never moves
    Kinematic = 1,  // moved by code; pushes dynamic bodies, is not pushed
    Dynamic = 2,    // moved by forces and collisions
};

/// A handle: a slot and the generation it was issued in, so one that outlives
/// its body is invalid rather than naming whatever took the slot.
struct Handle
{
    uint32_t index = 0xFFFFFFFFu;
    uint32_t generation = 0;
    bool operator==(const Handle& other) const { return index == other.index && generation == other.generation; }
    bool operator!=(const Handle& other) const { return !(*this == other); }
};
using BodyHandle = Handle;
using ShapeHandle = Handle;

struct BodyDef
{
    BodyType type = BodyType::Dynamic;
    Vec2 position;
    Vec2 velocity;
    float mass = 1.0f;
    float gravityScale = 1.0f;
    float linearDamping = 0.0f;  // fraction of speed lost per second
    void* user = nullptr;
};

struct ShapeDef
{
    bool isCircle = false;
    Vec2 offset;               // from the body's position
    float halfWidth = 0.5f;    // a box
    float halfHeight = 0.5f;   // a box
    float radius = 0.5f;       // a circle
    float friction = 0.3f;     // 0: ice, 1: rubber
    float restitution = 0.0f;  // 0: no bounce, 1: keeps all its speed
    bool sensor = false;       // reports contacts, pushes nothing
    uint16_t layer = 1;        // the layer bit this shape is on
    uint16_t mask = 0xFFFF;    // the layers it collides with; both sides must agree
    void* user = nullptr;
};

struct Contact
{
    ShapeHandle shapeA;
    ShapeHandle shapeB;
    BodyHandle bodyA;
    BodyHandle bodyB;
};

struct RayHit
{
    ShapeHandle shape;
    BodyHandle body;
    Vec2 point;
    Vec2 normal;
    float fraction = 0.0f;  // along the ray, 0 to 1
};

/// A small 2D physics world, sized for microcontrollers: axis-aligned boxes
/// and circles, no rotation, sort-and-sweep along X, a few solver iterations.
/// It knows nothing about the engine, so its behaviour is unit tested alone.
///
/// Steps are fixed: Step() banks the frame's time and runs whole steps of
/// `fixedStep`, so a game behaves the same at 20 FPS as at 60. The number of
/// steps a frame may run is bounded, so a slow frame slows the game down
/// instead of making the next frame slower still.
class DEKI_PHYSICS2D_API World
{
public:
    Vec2 gravity{ 0.0f, -9.81f };  // the engine's world: metres, +Y up
    float fixedStep = 1.0f / 60.0f;
    int maxStepsPerFrame = 4;
    int velocityIterations = 4;

    BodyHandle CreateBody(const BodyDef& def);
    void DestroyBody(BodyHandle body);  // and its shapes
    bool IsValid(BodyHandle body) const;
    ShapeHandle AddShape(BodyHandle body, const ShapeDef& def);
    void RemoveShape(ShapeHandle shape);

    Vec2 GetPosition(BodyHandle body) const;
    void SetPosition(BodyHandle body, Vec2 position);
    Vec2 GetVelocity(BodyHandle body) const;
    void SetVelocity(BodyHandle body, Vec2 velocity);
    void ApplyForce(BodyHandle body, Vec2 force);      // until the next step
    void ApplyImpulse(BodyHandle body, Vec2 impulse);  // now
    void* GetBodyUser(BodyHandle body) const;
    void* GetShapeUser(ShapeHandle shape) const;

    /// Runs the whole fixed steps `deltaSeconds` has banked; returns how many.
    int Step(float deltaSeconds);

    /// The nearest shape along the segment that `mask` selects.
    bool Raycast(Vec2 from, Vec2 to, RayHit& hit, uint16_t mask = 0xFFFF) const;

    /// Called from Step, after the step that started or ended the contact.
    std::function<void(const Contact&)> onContactBegin;
    std::function<void(const Contact&)> onContactEnd;

    size_t BodyCount() const { return m_LiveBodies; }
    size_t ShapeCount() const { return m_LiveShapes; }

private:
    struct Body
    {
        Vec2 position;
        Vec2 velocity;
        Vec2 force;
        float invMass = 0.0f;
        float gravityScale = 1.0f;
        float linearDamping = 0.0f;
        BodyType type = BodyType::Dynamic;
        void* user = nullptr;
        uint32_t generation = 1;
        bool alive = false;
    };
    struct Shape
    {
        ShapeDef def;
        uint32_t body = 0;
        uint32_t generation = 1;
        bool alive = false;
    };
    struct Aabb
    {
        float minX, minY, maxX, maxY;
    };
    struct Manifold
    {
        uint32_t shapeA, shapeB;
        Vec2 normal;  // from A to B
        float penetration;
    };

    Body* Find(BodyHandle body);
    const Body* Find(BodyHandle body) const;
    Aabb BoundsOf(const Shape& shape) const;
    bool Collide(uint32_t a, uint32_t b, Manifold& out) const;
    void FixedStep();
    void Solve(const Manifold& contact);
    void ReportContacts();

    std::vector<Body> m_Bodies;
    std::vector<Shape> m_Shapes;
    std::vector<uint32_t> m_FreeBodies;
    std::vector<uint32_t> m_FreeShapes;
    std::vector<uint32_t> m_Sweep;  // live shapes by min X, kept nearly sorted between steps
    std::vector<Manifold> m_Manifolds;
    std::vector<uint64_t> m_Touching;  // this step's pairs, sorted
    std::vector<uint64_t> m_WasTouching;
    float m_Banked = 0.0f;
    size_t m_LiveBodies = 0;
    size_t m_LiveShapes = 0;
};

}  // namespace DekiPhysics2D
