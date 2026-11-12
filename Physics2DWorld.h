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
    Static = 0,     // never moves on its own; code may place it
    Kinematic = 1,  // moved by code; pushes dynamic bodies, is not pushed
    Dynamic = 2,    // moved by forces and collisions
};

enum class ShapeType : uint8_t
{
    Box = 0,
    Circle = 1,
    Capsule = 2,  // a line with a radius: two round ends joined by straight sides
    Polygon = 3,  // convex
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
using JointHandle = Handle;

constexpr int kMaxPolygonPoints = 8;

struct BodyDef
{
    BodyType type = BodyType::Dynamic;
    Vec2 position;                 // of the object's origin; a dynamic body turns about the centre of its shapes
    float angle = 0.0f;            // radians, counter-clockwise, as the engine's rotation
    Vec2 velocity;                 // of the centre of its shapes
    float angularVelocity = 0.0f;  // radians per second
    float mass = 1.0f;
    float gravityScale = 1.0f;
    float linearDamping = 0.0f;   // fraction of speed lost per second
    float angularDamping = 0.0f;  // fraction of spin lost per second
    bool freezeRotation = false;  // never turns, whatever hits it
    bool canSleep = true;         // may stop being simulated while nothing happens to it
    bool continuous = true;       // stops at what it would otherwise jump over in one step
    void* user = nullptr;
};

struct ShapeDef
{
    ShapeType type = ShapeType::Box;
    Vec2 offset;                     // from the body's position, turning with it
    float halfWidth = 0.5f;          // a box
    float halfHeight = 0.5f;         // a box
    float radius = 0.5f;             // a circle or a capsule
    float halfLength = 0.5f;         // a capsule: from its middle to the centre of either end
    float angle = 0.0f;              // a capsule: the way it points; 0 lies along X
    Vec2 points[kMaxPolygonPoints];  // a polygon, any order; the convex outline of them is used
    int pointCount = 0;
    float friction = 0.3f;    // 0: ice, 1: rubber
    float bounciness = 0.0f;  // 0: no bounce, 1: keeps all its speed
    bool trigger = false;     // reports contacts, pushes nothing
    uint16_t layer = 1;       // the layer bit this shape is on
    uint16_t mask = 0xFFFF;   // the layers it collides with; both sides must agree
    void* user = nullptr;
};

/// Pins two bodies together at a point, leaving them free to turn. With no
/// bodyB, the point is pinned to the world.
struct HingeDef
{
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec2 anchor;  // in the world, when the joint is made
    bool collideConnected = false;
    bool limited = false;
    float lowerAngle = 0.0f;  // radians B may turn from where it started, relative to A
    float upperAngle = 0.0f;
    bool motor = false;
    float motorSpeed = 0.0f;  // radians per second of B relative to A
    float maxMotorTorque = 0.0f;
};

/// Keeps two points a set distance apart. A rope only stops them moving
/// further apart. With no bodyB, anchorB is a point in the world.
struct DistanceDef
{
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec2 anchorA;  // in the world, when the joint is made
    Vec2 anchorB;
    float length = -1.0f;  // below 0: how far apart the anchors are now
    bool rope = false;
    bool collideConnected = false;
};

struct Contact
{
    ShapeHandle shapeA;
    ShapeHandle shapeB;
    BodyHandle bodyA;
    BodyHandle bodyB;
    bool trigger = false;  // either shape is a trigger
};

struct RayHit
{
    ShapeHandle shape;
    BodyHandle body;
    Vec2 point;
    Vec2 normal;
    float fraction = 0.0f;  // along the ray, 0 to 1
};

/// The outline of `points`, counter-clockwise and without corners in a straight
/// line, as a polygon shape uses it. `out` holds up to `count`; returns how many.
DEKI_PHYSICS2D_API int ConvexOutline(const Vec2* points, int count, Vec2* out);

/// A small 2D physics world, sized for microcontrollers: convex shapes and
/// joints, sort-and-sweep along X, a few solver iterations, and no allocation
/// once the scene has settled. It knows nothing about the engine, so its
/// behaviour is unit tested alone.
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
    int velocityIterations = 6;
    bool allowSleep = true;

    BodyHandle CreateBody(const BodyDef& def);
    void DestroyBody(BodyHandle body);  // and its shapes and joints
    bool IsValid(BodyHandle body) const;
    /// An invalid handle when `def` has no area: a size of 0, or a polygon of
    /// fewer than 3 distinct corners in a line.
    ShapeHandle AddShape(BodyHandle body, const ShapeDef& def);
    void RemoveShape(ShapeHandle shape);

    JointHandle CreateHinge(const HingeDef& def);
    JointHandle CreateDistance(const DistanceDef& def);
    void DestroyJoint(JointHandle joint);
    bool IsValidJoint(JointHandle joint) const;

    Vec2 GetPosition(BodyHandle body) const;           // of the object's origin
    void SetPosition(BodyHandle body, Vec2 position);  // of the object's origin
    float GetAngle(BodyHandle body) const;
    void SetAngle(BodyHandle body, float radians);  // about the object's origin
    Vec2 GetVelocity(BodyHandle body) const;        // of the centre of its shapes
    void SetVelocity(BodyHandle body, Vec2 velocity);
    float GetAngularVelocity(BodyHandle body) const;
    void SetAngularVelocity(BodyHandle body, float radiansPerSecond);
    void AddForce(BodyHandle body, Vec2 force);                         // until the next step
    void AddTorque(BodyHandle body, float torque);                      // until the next step
    void AddImpulse(BodyHandle body, Vec2 impulse);                     // now, through the centre
    void AddImpulseAt(BodyHandle body, Vec2 impulse, Vec2 worldPoint);  // now; turns it unless at the centre
    void* GetBodyUser(BodyHandle body) const;
    void* GetShapeUser(ShapeHandle shape) const;

    /// False while a dynamic body sleeps, and always for a static one.
    bool IsAwake(BodyHandle body) const;
    /// Wakes the body and everything resting with it.
    void WakeUp(BodyHandle body);

    /// Runs the whole fixed steps `deltaSeconds` has banked; returns how many.
    int Step(float deltaSeconds);

    /// The nearest shape along the segment that `mask` selects.
    bool Raycast(Vec2 from, Vec2 to, RayHit& hit, uint16_t mask = 0xFFFF) const;

    /// Called from Step, after the step that started or ended the contact.
    std::function<void(const Contact&)> onContactBegin;
    std::function<void(const Contact&)> onContactEnd;

    size_t BodyCount() const { return m_LiveBodies; }
    size_t ShapeCount() const { return m_LiveShapes; }
    size_t JointCount() const { return m_LiveJoints; }

private:
    struct Hull;
    struct Body
    {
        Vec2 position;  // of its centre of mass, which is what the solver moves
        Vec2 centre;    // that centre, from the object's origin in the body's own axes
        float angle = 0.0f;
        float cosAngle = 1.0f;  // of `angle`, kept so a step computes each once
        float sinAngle = 0.0f;
        Vec2 velocity;
        float angularVelocity = 0.0f;
        // What it had as the step began, before gravity and forces: the speed a
        // contact bounces from.
        Vec2 arriving;
        float arrivingSpin = 0.0f;
        Vec2 force;
        float torque = 0.0f;
        float mass = 0.0f;  // as asked for; invMass is 0 unless dynamic
        float invMass = 0.0f;
        float invInertia = 0.0f;
        float gravityScale = 1.0f;
        float linearDamping = 0.0f;
        float angularDamping = 0.0f;
        float thinnest = 0.0f;   // of its solid shapes, from the middle to the nearest side
        float sleepTime = 0.0f;  // how long it has been nearly still
        float islandTime = 0.0f;
        uint32_t island = 0;  // who it sleeps with
        uint32_t jointCount = 0;
        BodyType type = BodyType::Dynamic;
        bool freezeRotation = false;
        bool canSleep = true;
        bool continuous = true;
        bool awake = true;
        bool moved = false;  // placed by code since the last step
        void* user = nullptr;
        uint32_t generation = 1;
        bool alive = false;
    };
    struct Aabb
    {
        float minX, minY, maxX, maxY;
    };
    struct Shape
    {
        Vec2 points[kMaxPolygonPoints];  // in the body's own axes
        Vec2 normals[kMaxPolygonPoints];
        Aabb bounds{};
        float radius = 0.0f;
        float friction = 0.3f;
        float bounciness = 0.0f;
        void* user = nullptr;
        uint32_t body = 0;
        uint32_t generation = 1;
        uint16_t layer = 1;
        uint16_t mask = 0xFFFF;
        uint8_t count = 0;  // 1: a circle, 2: a capsule, 3 or more: a polygon
        bool trigger = false;
        bool boundsValid = false;
        bool alive = false;
    };
    struct ContactPoint
    {
        Vec2 position;
        float penetration = 0.0f;
        // Prepared once per step, used by every solver pass.
        Vec2 toA, toB;
        float normalMass = 0.0f;
        float tangentMass = 0.0f;
        float bounce = 0.0f;
        float normalImpulse = 0.0f;
        float tangentImpulse = 0.0f;
    };
    struct Manifold
    {
        uint32_t shapeA, shapeB;
        Vec2 normal;  // from A to B
        int pointCount = 0;
        bool resting = false;  // both sides at rest: kept for when they wake
        // Prepared once per step, for a contact that is not resting: the
        // shapes' bodies and the friction between them.
        uint32_t bodyA = 0, bodyB = 0;
        float friction = 0.0f;
        // Its two points solved together (SolvePair): how a push at one
        // moves each, and one over the determinant of that.
        bool paired = false;
        float k11 = 0.0f, k12 = 0.0f, k22 = 0.0f, invDet = 0.0f;
        ContactPoint points[2];
    };
    struct Joint
    {
        uint32_t bodyA = 0;
        uint32_t bodyB = 0;  // kNoBody: the world
        Vec2 localA, localB;
        float referenceAngle = 0.0f;
        float length = 0.0f;
        float lowerAngle = 0.0f;
        float upperAngle = 0.0f;
        float motorSpeed = 0.0f;
        float maxMotorTorque = 0.0f;
        bool hinge = true;
        bool rope = false;
        bool collideConnected = false;
        bool limited = false;
        bool motor = false;
        // Prepared once per step, used by every solver pass.
        Vec2 toA, toB, axis, pointBias;
        Vec2 pointImpulse;
        float k11 = 0.0f, k12 = 0.0f, k22 = 0.0f;
        float axialMass = 0.0f;
        float axisMass = 0.0f;
        float axisBias = 0.0f;
        float axisImpulse = 0.0f;
        float limitBias = 0.0f;
        float limitImpulse = 0.0f;
        float motorImpulse = 0.0f;
        int limitState = 0;  // 0: free, 1: at the lower limit, 2: at the upper
        bool slack = false;
        uint32_t generation = 1;
        bool alive = false;
    };

    Body* Find(BodyHandle body);
    const Body* Find(BodyHandle body) const;
    const Body& BodyAt(uint32_t index) const;
    Body& BodyAt(uint32_t index);
    static Vec2 Turned(const Body& body, Vec2 v);
    static Vec2 OriginOf(const Body& body);
    void RefreshMassData(uint32_t bodyIndex);
    void SetPose(Body& body, Vec2 position, float angle);
    Aabb BoundsOf(const Shape& shape) const;
    void MakeHull(const Shape& shape, Hull& out) const;
    bool Collide(uint32_t a, uint32_t b, Manifold& out) const;
    static bool CollideHulls(const Hull& a, const Hull& b, Manifold& out);
    static bool ClipFace(const Hull& reference, const Hull& incident, int face, bool referenceIsA, Manifold& out);
    bool DoNotCollide(const Shape& a, const Shape& b) const;
    const Manifold* FindLastManifold(uint64_t key) const;
    // Every shape's box where its body is now, and the shapes in order along X.
    void RefreshSweep();
    void FindContacts();
    void PrepareContact(Manifold& contact);
    void WarmStart(Manifold& contact);
    void SolveVelocity(Manifold& contact, bool reverse);
    void SolvePair(Manifold& contact, Body& a, Body& b);
    void CorrectPositions(float dt);
    void PrepareJoint(Joint& joint, float dt);
    void SolveJoint(Joint& joint);
    bool IsFast(const Body& body, float dt) const;
    void SweepBody(uint32_t bodyIndex, float dt);
    void IntegratePositions(float dt);
    uint32_t FindIsland(uint32_t bodyIndex);
    void JoinIslands(uint32_t a, uint32_t b);
    void UpdateSleep(float dt);
    void Wake(uint32_t bodyIndex);
    void WakeTouching(uint32_t bodyIndex);
    void WakeAll();
    void FixedStep();
    void ReportContacts();
    Contact MakeContact(uint32_t shapeA, uint32_t shapeB) const;
    // No body is awake, moving or was moved: a step would change nothing.
    bool AtRest() const;
    // Something was changed from outside since the last step (a body made,
    // moved, pushed or destroyed, a shape or a joint added or removed), so
    // the next step runs even with everything at rest.
    bool m_Changed = true;
    JointHandle AddJoint(const Joint& joint);

    std::vector<Body> m_Bodies;
    std::vector<Shape> m_Shapes;
    std::vector<Joint> m_Joints;
    std::vector<uint32_t> m_FreeBodies;
    std::vector<uint32_t> m_FreeShapes;
    std::vector<uint32_t> m_FreeJoints;
    std::vector<uint32_t> m_Sweep;  // live shapes by min X, kept nearly sorted between steps
    std::vector<Manifold> m_Manifolds;
    std::vector<Manifold> m_LastManifolds;
    std::vector<uint64_t> m_Touching;  // this step's pairs, sorted
    std::vector<uint64_t> m_WasTouching;
    // ReportContacts' copies: a callback may destroy bodies, which edits the
    // lists above. Kept, so reporting allocates nothing.
    std::vector<uint64_t> m_ReportNow;
    std::vector<uint64_t> m_ReportBefore;
    Body m_Ground;  // what a joint with no second body pins to
    Vec2 m_LastGravity{ 0.0f, -9.81f };
    float m_Banked = 0.0f;
    size_t m_LiveBodies = 0;
    size_t m_LiveShapes = 0;
    size_t m_LiveJoints = 0;
};

}  // namespace DekiPhysics2D
