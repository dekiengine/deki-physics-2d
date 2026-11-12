#include "Physics2DWorld.h"

#include <algorithm>
#include <bit>
#include <cmath>

namespace DekiPhysics2D
{

namespace
{
// Overlap this small is left alone, so a resting body keeps touching what it
// rests on instead of jittering between touching and not.
constexpr float kSlop = 0.005f;
// Two shapes this near already have a contact, which lets them close the gap
// and no more. Without it boxes set down exactly touching have none until
// they have sunk into each other, one box a step up a stack.
constexpr float kContactMargin = kSlop;
// A contact lets its two close until they overlap by this: touching for
// certain, and within kSlop.
constexpr float kRestDepth = 0.5f * kSlop;
// An overlap past kSlop is pushed out as a spring this stiff and this damped
// would, and no faster than kMaxPushSpeed: all of it at once makes stacks
// bounce. Against something that cannot move the spring is twice as stiff.
constexpr float kContactHertz = 60.0f;
constexpr float kContactDamping = 10.0f;
constexpr float kMaxPushSpeed = 3.0f;
// Slower than this does not bounce, or a resting body would never settle.
constexpr float kBounceThreshold = 0.5f;
// Two contact points are solved together unless they are so close that the
// sums behind it lose their meaning in the rounding.
constexpr float kMaxPairCondition = 1000.0f;
// A body this slow for this long, with everything touching it, goes to sleep.
constexpr float kSleepSpeed = 0.03f;
constexpr float kSleepSpin = 0.06f;
constexpr float kTimeToSleep = 0.5f;
// The share of a joint's error a step removes, and the most it acts on at
// once: a joint pulled far apart must not fling its bodies.
constexpr float kJointCorrection = 0.2f;
constexpr float kMaxJointError = 0.5f;
// A fast body is moved in steps no longer than its own thinnest half, so it
// cannot step over anything; then the first hit is narrowed down this often.
constexpr int kMaxSweepSteps = 32;
constexpr int kSweepRefinements = 5;
constexpr uint32_t kNoBody = 0xFFFFFFFFu;
constexpr float kInfinity = 1e30f;
// One over a hull's number of points, without a division.
constexpr float kOneOver[kMaxPolygonPoints + 1] = { 0.0f,        1.0f,        1.0f / 2.0f, 1.0f / 3.0f, 1.0f / 4.0f,
                                                    1.0f / 5.0f, 1.0f / 6.0f, 1.0f / 7.0f, 1.0f / 8.0f };

// One over `x`, for an x above zero, by multiplying. A processor with no
// divide instruction for floats (an ESP32-S3) calls a routine for every
// division, many times slower than this: a first guess from the number's
// bits, which each of three rounds brings twice as many digits closer, to
// the last few bits of a float.
inline float Inverse(float x)
{
    float guess = std::bit_cast<float>(0x7EF311C7u - std::bit_cast<uint32_t>(x));
    guess = guess * (2.0f - x * guess);
    guess = guess * (2.0f - x * guess);
    return guess * (2.0f - x * guess);
}

float Dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

float Cross(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

Vec2 Sub(Vec2 a, Vec2 b)
{
    return { a.x - b.x, a.y - b.y };
}

Vec2 Add(Vec2 a, Vec2 b)
{
    return { a.x + b.x, a.y + b.y };
}

Vec2 Scale(Vec2 v, float s)
{
    return { v.x * s, v.y * s };
}

// `v` a quarter turn counter-clockwise, times `s`: the velocity a point has at
// offset `v` on a body spinning at `s`.
Vec2 Perp(float s, Vec2 v)
{
    return { -s * v.y, s * v.x };
}

float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

uint64_t PairKey(uint32_t a, uint32_t b)
{
    return a < b ? (uint64_t(a) << 32) | b : (uint64_t(b) << 32) | a;
}

// The nearest point to `p` on the segment from `a` to `b`.
Vec2 NearestOnSegment(Vec2 p, Vec2 a, Vec2 b)
{
    const Vec2 edge = Sub(b, a);
    const float lengthSq = Dot(edge, edge);
    if (lengthSq <= 0.0f)
    {
        return a;
    }
    return Add(a, Scale(edge, Clamp(Dot(Sub(p, a), edge) / lengthSq, 0.0f, 1.0f)));
}

bool LinesCross(const Vec2* a, const Vec2* b)
{
    auto side = [](Vec2 from, Vec2 to, Vec2 p) { return Cross(Sub(to, from), Sub(p, from)); };
    return side(a[0], a[1], b[0]) * side(a[0], a[1], b[1]) < 0.0f &&
           side(b[0], b[1], a[0]) * side(b[0], b[1], a[1]) < 0.0f;
}

}  // namespace

int ConvexOutline(const Vec2* points, int count, Vec2* out)
{
    count = std::min(count, kMaxPolygonPoints);
    Vec2 sorted[kMaxPolygonPoints];
    for (int i = 0; i < count; ++i)
    {
        const Vec2 p = points[i];
        int at = i;
        while (at > 0 && (p.x < sorted[at - 1].x || (p.x == sorted[at - 1].x && p.y < sorted[at - 1].y)))
        {
            sorted[at] = sorted[at - 1];
            --at;
        }
        sorted[at] = p;
    }

    Vec2 chain[kMaxPolygonPoints * 2];
    int size = 0;
    auto add = [&](Vec2 p, int floor)
    {
        while (size >= floor + 2 && Cross(Sub(chain[size - 1], chain[size - 2]), Sub(p, chain[size - 2])) <= 1e-8f)
        {
            --size;
        }
        chain[size++] = p;
    };
    for (int i = 0; i < count; ++i)
    {
        add(sorted[i], 0);
    }
    const int lower = size - 1;
    for (int i = count - 2; i >= 0; --i)
    {
        add(sorted[i], lower);
    }
    --size;  // the last repeats the first
    std::copy(chain, chain + size, out);
    return size;
}

// A shape in the world's axes, for one test: every kind is a few points with a
// radius around them, so one routine collides them all.
struct World::Hull
{
    Vec2 points[kMaxPolygonPoints];
    Vec2 normals[kMaxPolygonPoints];
    Vec2 middle;  // the mean of its points: somewhere inside it
    int count = 0;
    float radius = 0.0f;
};

BodyHandle World::CreateBody(const BodyDef& def)
{
    m_Changed = true;
    uint32_t index;
    if (!m_FreeBodies.empty())
    {
        index = m_FreeBodies.back();
        m_FreeBodies.pop_back();
    }
    else
    {
        index = static_cast<uint32_t>(m_Bodies.size());
        m_Bodies.emplace_back();
    }

    Body& body = m_Bodies[index];
    body.position = def.position;  // its shapes, as they are added, move the centre off the origin
    body.centre = {};
    body.angle = def.angle;
    body.cosAngle = std::cos(def.angle);
    body.sinAngle = std::sin(def.angle);
    body.velocity = def.type == BodyType::Static ? Vec2{} : def.velocity;
    body.angularVelocity = def.type == BodyType::Static ? 0.0f : def.angularVelocity;
    body.force = {};
    body.torque = 0.0f;
    body.mass = def.mass;
    body.invMass = (def.type == BodyType::Dynamic && def.mass > 0.0f) ? 1.0f / def.mass : 0.0f;
    body.invInertia = 0.0f;  // until it has shapes
    body.gravityScale = def.gravityScale;
    body.linearDamping = def.linearDamping;
    body.angularDamping = def.angularDamping;
    body.thinnest = 0.0f;
    body.sleepTime = 0.0f;
    body.island = index;
    body.jointCount = 0;
    body.freezeRotation = def.freezeRotation;
    body.canSleep = def.canSleep;
    body.continuous = def.continuous;
    body.awake = def.type == BodyType::Dynamic;
    body.moved = true;  // what it appeared on top of has to notice
    body.type = def.type;
    body.user = def.user;
    body.alive = true;
    ++m_LiveBodies;
    return { index, body.generation };
}

void World::DestroyBody(BodyHandle handle)
{
    m_Changed = true;
    Body* body = Find(handle);
    if (!body)
    {
        return;
    }
    for (uint32_t i = 0; i < m_Joints.size(); ++i)
    {
        if (m_Joints[i].alive && (m_Joints[i].bodyA == handle.index || m_Joints[i].bodyB == handle.index))
        {
            DestroyJoint({ i, m_Joints[i].generation });
        }
    }
    for (uint32_t i = 0; i < m_Shapes.size(); ++i)
    {
        if (m_Shapes[i].alive && m_Shapes[i].body == handle.index)
        {
            RemoveShape({ i, m_Shapes[i].generation });
        }
    }
    body->alive = false;
    ++body->generation;
    m_FreeBodies.push_back(handle.index);
    --m_LiveBodies;
}

bool World::IsValid(BodyHandle body) const
{
    return Find(body) != nullptr;
}

World::Body* World::Find(BodyHandle handle)
{
    if (handle.index >= m_Bodies.size())
    {
        return nullptr;
    }
    Body& body = m_Bodies[handle.index];
    return body.alive && body.generation == handle.generation ? &body : nullptr;
}

const World::Body* World::Find(BodyHandle handle) const
{
    return const_cast<World*>(this)->Find(handle);
}

World::Body& World::BodyAt(uint32_t index)
{
    return index == kNoBody ? m_Ground : m_Bodies[index];
}

const World::Body& World::BodyAt(uint32_t index) const
{
    return index == kNoBody ? m_Ground : m_Bodies[index];
}

Vec2 World::Turned(const Body& body, Vec2 v)
{
    return { v.x * body.cosAngle - v.y * body.sinAngle, v.x * body.sinAngle + v.y * body.cosAngle };
}

Vec2 World::OriginOf(const Body& body)
{
    return Sub(body.position, Turned(body, body.centre));
}

ShapeHandle World::AddShape(BodyHandle body, const ShapeDef& def)
{
    m_Changed = true;
    if (!Find(body))
    {
        return {};
    }

    // Every kind becomes points with a radius, before a slot is taken: a shape
    // with no area has no normals and cannot collide.
    Vec2 points[kMaxPolygonPoints];
    int count = 0;
    float radius = 0.0f;
    switch (def.type)
    {
        case ShapeType::Circle:
            if (def.radius <= 0.0f)
            {
                return {};
            }
            points[0] = def.offset;
            count = 1;
            radius = def.radius;
            break;
        case ShapeType::Capsule:
        {
            if (def.radius <= 0.0f || def.halfLength <= 0.0f)
            {
                return {};
            }
            const Vec2 along = Scale({ std::cos(def.angle), std::sin(def.angle) }, def.halfLength);
            points[0] = Sub(def.offset, along);
            points[1] = Add(def.offset, along);
            count = 2;
            radius = def.radius;
            break;
        }
        case ShapeType::Box:
            if (def.halfWidth <= 0.0f || def.halfHeight <= 0.0f)
            {
                return {};
            }
            points[0] = Add(def.offset, { -def.halfWidth, -def.halfHeight });
            points[1] = Add(def.offset, { def.halfWidth, -def.halfHeight });
            points[2] = Add(def.offset, { def.halfWidth, def.halfHeight });
            points[3] = Add(def.offset, { -def.halfWidth, def.halfHeight });
            count = 4;
            break;
        case ShapeType::Polygon:
        {
            if (def.pointCount < 3 || def.pointCount > kMaxPolygonPoints)
            {
                return {};
            }
            Vec2 moved[kMaxPolygonPoints];
            for (int i = 0; i < def.pointCount; ++i)
            {
                moved[i] = Add(def.points[i], def.offset);
            }
            count = ConvexOutline(moved, def.pointCount, points);
            if (count < 3)
            {
                return {};
            }
            break;
        }
    }

    uint32_t index;
    if (!m_FreeShapes.empty())
    {
        index = m_FreeShapes.back();
        m_FreeShapes.pop_back();
    }
    else
    {
        index = static_cast<uint32_t>(m_Shapes.size());
        m_Shapes.emplace_back();
    }
    Shape& shape = m_Shapes[index];
    std::copy(points, points + count, shape.points);
    shape.count = static_cast<uint8_t>(count);
    shape.radius = radius;
    if (count >= 2)
    {
        for (int i = 0; i < count; ++i)
        {
            const Vec2 edge = Sub(shape.points[(i + 1) % count], shape.points[i]);
            shape.normals[i] = Scale({ edge.y, -edge.x }, 1.0f / std::sqrt(Dot(edge, edge)));
        }
    }
    shape.friction = def.friction;
    shape.bounciness = def.bounciness;
    shape.trigger = def.trigger;
    shape.layer = def.layer;
    shape.mask = def.mask;
    shape.user = def.user;
    shape.body = body.index;
    shape.boundsValid = false;
    shape.alive = true;
    m_Sweep.push_back(index);
    ++m_LiveShapes;
    RefreshMassData(body.index);
    m_Bodies[body.index].moved = true;
    Wake(body.index);
    return { index, shape.generation };
}

void World::RemoveShape(ShapeHandle handle)
{
    m_Changed = true;
    if (handle.index >= m_Shapes.size())
    {
        return;
    }
    Shape& shape = m_Shapes[handle.index];
    if (!shape.alive || shape.generation != handle.generation)
    {
        return;
    }

    // Anything touching it stops touching, wakes in case it was resting there,
    // and is told so now: nothing else would ever report the pair again.
    auto involves = [&](uint64_t key)
    { return static_cast<uint32_t>(key >> 32) == handle.index || static_cast<uint32_t>(key) == handle.index; };
    for (uint64_t key : m_WasTouching)
    {
        if (involves(key))
        {
            const uint32_t a = static_cast<uint32_t>(key >> 32);
            const uint32_t b = static_cast<uint32_t>(key);
            Wake(m_Shapes[a].body);
            Wake(m_Shapes[b].body);
        }
    }
    if (onContactEnd)
    {
        for (uint64_t key : m_WasTouching)
        {
            if (involves(key))
            {
                onContactEnd(MakeContact(static_cast<uint32_t>(key >> 32), static_cast<uint32_t>(key)));
            }
        }
    }
    m_WasTouching.erase(std::remove_if(m_WasTouching.begin(), m_WasTouching.end(), involves), m_WasTouching.end());
    // And its contacts go with it: a shape made later takes its place in the
    // list, and would start from the pushes this one was held with.
    m_Manifolds.erase(std::remove_if(m_Manifolds.begin(), m_Manifolds.end(), [&](const Manifold& contact)
                                     { return contact.shapeA == handle.index || contact.shapeB == handle.index; }),
                      m_Manifolds.end());

    const uint32_t bodyIndex = shape.body;
    shape.alive = false;
    ++shape.generation;
    m_Sweep.erase(std::find(m_Sweep.begin(), m_Sweep.end(), handle.index));
    m_FreeShapes.push_back(handle.index);
    --m_LiveShapes;
    if (m_Bodies[bodyIndex].alive)
    {
        RefreshMassData(bodyIndex);
    }
}

// Where the body's weight is and how hard it is to turn, from its shapes. The
// mass is shared between them by area, and a shape off the centre counts for
// more. The object stays where it is: only the point the solver moves shifts.
void World::RefreshMassData(uint32_t bodyIndex)
{
    Body& body = m_Bodies[bodyIndex];
    body.invInertia = 0.0f;
    body.thinnest = 0.0f;
    if (body.type != BodyType::Dynamic)
    {
        return;
    }

    constexpr float kPi = 3.14159265f;
    float totalArea = 0.0f;
    float inertiaPerArea = 0.0f;  // about the origin until the centre is known
    Vec2 areaMoment{};
    float thinnest = kInfinity;
    for (const Shape& shape : m_Shapes)
    {
        if (!shape.alive || shape.body != bodyIndex || shape.trigger)
        {
            continue;
        }
        if (shape.count == 1)
        {
            const float rr = shape.radius * shape.radius;
            const float area = kPi * rr;
            totalArea += area;
            areaMoment = Add(areaMoment, Scale(shape.points[0], area));
            inertiaPerArea += area * (0.5f * rr + Dot(shape.points[0], shape.points[0]));
            thinnest = std::min(thinnest, shape.radius);
        }
        else if (shape.count == 2)
        {
            // Treated as a box and a whole circle at its ends.
            const Vec2 middle = Scale(Add(shape.points[0], shape.points[1]), 0.5f);
            const Vec2 edge = Sub(shape.points[1], shape.points[0]);
            const float lengthSq = Dot(edge, edge);
            const float rr = shape.radius * shape.radius;
            const float boxArea = std::sqrt(lengthSq) * 2.0f * shape.radius;
            const float roundArea = kPi * rr;
            totalArea += boxArea + roundArea;
            areaMoment = Add(areaMoment, Scale(middle, boxArea + roundArea));
            inertiaPerArea += boxArea * ((lengthSq + 4.0f * rr) / 12.0f) + roundArea * (0.5f * rr + lengthSq * 0.25f) +
                              (boxArea + roundArea) * Dot(middle, middle);
            thinnest = std::min(thinnest, shape.radius);
        }
        else
        {
            // Triangles from the body's origin, so an offset shape counts for
            // more by itself.
            Vec2 centre{};
            for (int i = 0; i < shape.count; ++i)
            {
                const Vec2 p = shape.points[i];
                const Vec2 q = shape.points[(i + 1) % shape.count];
                const float twice = Cross(p, q);
                totalArea += 0.5f * twice;
                areaMoment = Add(areaMoment, Scale(Add(p, q), twice / 6.0f));
                inertiaPerArea += twice * (Dot(p, p) + Dot(p, q) + Dot(q, q)) / 12.0f;
                centre = Add(centre, p);
            }
            centre = Scale(centre, 1.0f / static_cast<float>(shape.count));
            for (int i = 0; i < shape.count; ++i)
            {
                thinnest = std::min(thinnest, Dot(shape.normals[i], Sub(shape.points[i], centre)));
            }
        }
    }
    body.thinnest = thinnest < kInfinity ? thinnest : 0.0f;

    const Vec2 massCentre = totalArea > 0.0f ? Scale(areaMoment, 1.0f / totalArea) : Vec2{};
    const Vec2 shift = Sub(massCentre, body.centre);
    if (shift.x != 0.0f || shift.y != 0.0f)
    {
        body.position = Add(body.position, Turned(body, shift));
        body.centre = massCentre;
        body.moved = true;
    }
    inertiaPerArea -= totalArea * Dot(massCentre, massCentre);

    if (!body.freezeRotation && body.invMass > 0.0f && totalArea > 0.0f && inertiaPerArea > 0.0f)
    {
        body.invInertia = totalArea / (body.mass * inertiaPerArea);
    }
}

JointHandle World::AddJoint(const Joint& joint)
{
    m_Changed = true;
    uint32_t index;
    if (!m_FreeJoints.empty())
    {
        index = m_FreeJoints.back();
        m_FreeJoints.pop_back();
    }
    else
    {
        index = static_cast<uint32_t>(m_Joints.size());
        m_Joints.emplace_back();
    }
    const uint32_t generation = m_Joints[index].generation;
    m_Joints[index] = joint;
    m_Joints[index].generation = generation;
    m_Joints[index].alive = true;
    ++m_LiveJoints;
    ++m_Bodies[joint.bodyA].jointCount;
    Wake(joint.bodyA);
    if (joint.bodyB != kNoBody)
    {
        ++m_Bodies[joint.bodyB].jointCount;
        Wake(joint.bodyB);
    }
    return { index, generation };
}

JointHandle World::CreateHinge(const HingeDef& def)
{
    const Body* a = Find(def.bodyA);
    const Body* b = Find(def.bodyB);
    if (!a || (!b && def.bodyB != BodyHandle{}))
    {
        return {};
    }
    auto toLocal = [](const Body& body, Vec2 world)
    {
        const Vec2 d = Sub(world, OriginOf(body));
        return Vec2{ d.x * body.cosAngle + d.y * body.sinAngle, -d.x * body.sinAngle + d.y * body.cosAngle };
    };
    Joint joint;
    joint.hinge = true;
    joint.bodyA = def.bodyA.index;
    joint.bodyB = b ? def.bodyB.index : kNoBody;
    joint.localA = toLocal(*a, def.anchor);
    joint.localB = b ? toLocal(*b, def.anchor) : def.anchor;
    joint.referenceAngle = (b ? b->angle : 0.0f) - a->angle;
    joint.collideConnected = def.collideConnected;
    joint.limited = def.limited;
    joint.lowerAngle = def.lowerAngle;
    joint.upperAngle = def.upperAngle;
    joint.motor = def.motor;
    joint.motorSpeed = def.motorSpeed;
    joint.maxMotorTorque = def.maxMotorTorque;
    return AddJoint(joint);
}

JointHandle World::CreateDistance(const DistanceDef& def)
{
    const Body* a = Find(def.bodyA);
    const Body* b = Find(def.bodyB);
    if (!a || (!b && def.bodyB != BodyHandle{}))
    {
        return {};
    }
    auto toLocal = [](const Body& body, Vec2 world)
    {
        const Vec2 d = Sub(world, OriginOf(body));
        return Vec2{ d.x * body.cosAngle + d.y * body.sinAngle, -d.x * body.sinAngle + d.y * body.cosAngle };
    };
    Joint joint;
    joint.hinge = false;
    joint.bodyA = def.bodyA.index;
    joint.bodyB = b ? def.bodyB.index : kNoBody;
    joint.localA = toLocal(*a, def.anchorA);
    joint.localB = b ? toLocal(*b, def.anchorB) : def.anchorB;
    const Vec2 between = Sub(def.anchorB, def.anchorA);
    joint.length = def.length >= 0.0f ? def.length : std::sqrt(Dot(between, between));
    joint.rope = def.rope;
    joint.collideConnected = def.collideConnected;
    return AddJoint(joint);
}

void World::DestroyJoint(JointHandle handle)
{
    m_Changed = true;
    if (!IsValidJoint(handle))
    {
        return;
    }
    Joint& joint = m_Joints[handle.index];
    joint.alive = false;
    ++joint.generation;
    --m_LiveJoints;
    m_FreeJoints.push_back(handle.index);
    --m_Bodies[joint.bodyA].jointCount;
    Wake(joint.bodyA);
    if (joint.bodyB != kNoBody)
    {
        --m_Bodies[joint.bodyB].jointCount;
        Wake(joint.bodyB);
    }
}

bool World::IsValidJoint(JointHandle handle) const
{
    return handle.index < m_Joints.size() && m_Joints[handle.index].alive &&
           m_Joints[handle.index].generation == handle.generation;
}

void World::SetPose(Body& body, Vec2 position, float angle)
{
    body.position = position;
    body.angle = angle;
    body.cosAngle = std::cos(angle);
    body.sinAngle = std::sin(angle);
}

Vec2 World::GetPosition(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? OriginOf(*body) : Vec2{};
}

void World::SetPosition(BodyHandle handle, Vec2 position)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        body->position = Add(position, Turned(*body, body->centre));
        body->moved = true;
        Wake(handle.index);
        WakeTouching(handle.index);
    }
}

float World::GetAngle(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->angle : 0.0f;
}

void World::SetAngle(BodyHandle handle, float radians)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        const Vec2 origin = OriginOf(*body);
        SetPose(*body, body->position, radians);
        body->position = Add(origin, Turned(*body, body->centre));
        body->moved = true;
        Wake(handle.index);
        WakeTouching(handle.index);
    }
}

float World::GetAngularVelocity(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->angularVelocity : 0.0f;
}

void World::SetAngularVelocity(BodyHandle handle, float radiansPerSecond)
{
    m_Changed = true;
    Body* body = Find(handle);
    if (body && body->type != BodyType::Static && !body->freezeRotation)
    {
        body->angularVelocity = radiansPerSecond;
        if (radiansPerSecond != 0.0f)
        {
            Wake(handle.index);
        }
        if (body->type == BodyType::Kinematic)
        {
            WakeTouching(handle.index);
        }
    }
}

void World::AddTorque(BodyHandle handle, float torque)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        body->torque += torque;
        Wake(handle.index);
    }
}

void World::AddImpulseAt(BodyHandle handle, Vec2 impulse, Vec2 worldPoint)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        Wake(handle.index);
        body->velocity = Add(body->velocity, Scale(impulse, body->invMass));
        body->angularVelocity += body->invInertia * Cross(Sub(worldPoint, body->position), impulse);
    }
}

Vec2 World::GetVelocity(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->velocity : Vec2{};
}

void World::SetVelocity(BodyHandle handle, Vec2 velocity)
{
    m_Changed = true;
    Body* body = Find(handle);
    if (body && body->type != BodyType::Static)
    {
        body->velocity = velocity;
        if (velocity.x != 0.0f || velocity.y != 0.0f)
        {
            Wake(handle.index);
        }
        if (body->type == BodyType::Kinematic)
        {
            WakeTouching(handle.index);  // what rests on it has to notice it leave
        }
    }
}

void World::AddForce(BodyHandle handle, Vec2 force)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        body->force.x += force.x;
        body->force.y += force.y;
        Wake(handle.index);
    }
}

void World::AddImpulse(BodyHandle handle, Vec2 impulse)
{
    m_Changed = true;
    if (Body* body = Find(handle))
    {
        Wake(handle.index);
        body->velocity.x += impulse.x * body->invMass;
        body->velocity.y += impulse.y * body->invMass;
    }
}

void* World::GetBodyUser(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->user : nullptr;
}

void* World::GetShapeUser(ShapeHandle handle) const
{
    if (handle.index >= m_Shapes.size())
    {
        return nullptr;
    }
    const Shape& shape = m_Shapes[handle.index];
    return shape.alive && shape.generation == handle.generation ? shape.user : nullptr;
}

bool World::IsAwake(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body && (body->type == BodyType::Kinematic || body->awake);
}

void World::WakeUp(BodyHandle handle)
{
    m_Changed = true;
    if (Find(handle))
    {
        Wake(handle.index);
    }
}

void World::Wake(uint32_t bodyIndex)
{
    Body& body = m_Bodies[bodyIndex];
    if (body.type != BodyType::Dynamic)
    {
        return;
    }
    body.sleepTime = 0.0f;
    if (body.awake)
    {
        return;
    }
    const uint32_t island = body.island;
    for (Body& other : m_Bodies)
    {
        if (other.alive && other.type == BodyType::Dynamic && !other.awake && other.island == island)
        {
            other.awake = true;
            other.sleepTime = 0.0f;
        }
    }
}

void World::WakeTouching(uint32_t bodyIndex)
{
    for (uint64_t key : m_WasTouching)
    {
        const uint32_t a = m_Shapes[static_cast<uint32_t>(key >> 32)].body;
        const uint32_t b = m_Shapes[static_cast<uint32_t>(key)].body;
        if (a == bodyIndex)
        {
            Wake(b);
        }
        else if (b == bodyIndex)
        {
            Wake(a);
        }
    }
}

void World::WakeAll()
{
    for (Body& body : m_Bodies)
    {
        if (body.alive && body.type == BodyType::Dynamic)
        {
            body.awake = true;
            body.sleepTime = 0.0f;
        }
    }
}

World::Aabb World::BoundsOf(const Shape& shape) const
{
    const Body& body = m_Bodies[shape.body];
    Aabb box{ kInfinity, kInfinity, -kInfinity, -kInfinity };
    for (int i = 0; i < shape.count; ++i)
    {
        const Vec2 local = Sub(shape.points[i], body.centre);
        const float x = body.position.x + local.x * body.cosAngle - local.y * body.sinAngle;
        const float y = body.position.y + local.x * body.sinAngle + local.y * body.cosAngle;
        box.minX = std::min(box.minX, x);
        box.minY = std::min(box.minY, y);
        box.maxX = std::max(box.maxX, x);
        box.maxY = std::max(box.maxY, y);
    }
    box.minX -= shape.radius;
    box.minY -= shape.radius;
    box.maxX += shape.radius;
    box.maxY += shape.radius;
    return box;
}

void World::MakeHull(const Shape& shape, Hull& out) const
{
    const Body& body = m_Bodies[shape.body];
    const float c = body.cosAngle;
    const float s = body.sinAngle;
    out.count = shape.count;
    out.radius = shape.radius;
    Vec2 sum{};
    for (int i = 0; i < shape.count; ++i)
    {
        const Vec2 p = Sub(shape.points[i], body.centre);
        const Vec2 n = shape.normals[i];
        out.points[i] = { body.position.x + p.x * c - p.y * s, body.position.y + p.x * s + p.y * c };
        out.normals[i] = { n.x * c - n.y * s, n.x * s + n.y * c };
        sum = Add(sum, out.points[i]);
    }
    out.middle = Scale(sum, kOneOver[shape.count]);
}

// The incident side of one hull clipped against a face of the other. Two
// points on a flat contact, so a box resting on a face stays flat instead of
// rocking on a single corner. False when nothing lands on the face.
bool World::ClipFace(const Hull& reference, const Hull& incident, int face, bool referenceIsA, Manifold& out)
{
    const Vec2 n = reference.normals[face];
    const Vec2 v0 = reference.points[face];
    const Vec2 v1 = reference.points[(face + 1) % reference.count];
    const float totalRadius = reference.radius + incident.radius;

    // The other hull's side that faces this one most directly.
    Vec2 p0 = incident.points[0];
    Vec2 p1 = incident.points[incident.count > 1 ? 1 : 0];
    if (incident.count > 2)
    {
        int best = 0;
        float least = kInfinity;
        for (int i = 0; i < incident.count; ++i)
        {
            const float along = Dot(incident.normals[i], n);
            if (along < least)
            {
                least = along;
                best = i;
            }
        }
        p0 = incident.points[best];
        p1 = incident.points[(best + 1) % incident.count];
    }

    // Clip that side to the face's width.
    const Vec2 edge = Sub(v1, v0);
    const float width = std::sqrt(Dot(edge, edge));
    const Vec2 tangent = Scale(edge, Inverse(width));
    auto clip = [&](float sign, float limit)
    {
        const float d0 = sign * Dot(Sub(p0, v0), tangent) - limit;
        const float d1 = sign * Dot(Sub(p1, v0), tangent) - limit;
        if (d0 > 0.0f && d1 > 0.0f)
        {
            return false;
        }
        if (d0 > 0.0f)
        {
            p0 = Add(p0, Scale(Sub(p1, p0), d0 * Inverse(d0 - d1)));
        }
        else if (d1 > 0.0f)
        {
            p1 = Add(p1, Scale(Sub(p0, p1), d1 * Inverse(d1 - d0)));
        }
        return true;
    };
    if (!clip(1.0f, width) || !clip(-1.0f, 0.0f))
    {
        return false;
    }

    out.normal = referenceIsA ? n : Scale(n, -1.0f);
    out.pointCount = 0;
    const int points = incident.count > 1 ? 2 : 1;
    for (int i = 0; i < points; ++i)
    {
        const Vec2 p = i == 0 ? p0 : p1;
        const float separation = Dot(Sub(p, v0), n);
        if (separation < totalRadius + kContactMargin)
        {
            out.points[out.pointCount].position = Sub(p, Scale(n, incident.radius));
            out.points[out.pointCount].penetration = totalRadius - separation;
            ++out.pointCount;
        }
    }
    return out.pointCount > 0;
}

// Separating axes over the faces of both hulls, then either the touching faces
// clipped together or, past a corner, the two nearest points. Hulls within
// kContactMargin of each other have a contact too, its penetration below zero.
bool World::CollideHulls(const Hull& a, const Hull& b, Manifold& out)
{
    const float totalRadius = a.radius + b.radius;
    float best = -kInfinity;
    int bestFace = -1;
    bool referenceIsA = true;

    auto scan = [&](const Hull& reference, const Hull& other, bool isA)
    {
        if (reference.count < 2)
        {
            return true;
        }
        // A small preference for the first hull's faces keeps the choice from
        // flickering between two equal axes from one step to the next.
        const float bias = isA ? 0.0f : -1e-4f;
        for (int face = 0; face < reference.count; ++face)
        {
            // No point of the other hull is further out through this face
            // than its middle is: a face the middle is too far behind cannot
            // be the one, and is not measured.
            const float middle = Dot(reference.normals[face], Sub(other.middle, reference.points[face]));
            if (middle <= std::min(best - bias, totalRadius + kContactMargin))
            {
                continue;
            }
            float separation = kInfinity;
            for (int i = 0; i < other.count; ++i)
            {
                separation =
                    std::min(separation, Dot(reference.normals[face], Sub(other.points[i], reference.points[face])));
            }
            if (separation > totalRadius + kContactMargin)
            {
                return false;
            }
            if (separation + bias > best)
            {
                best = separation + bias;
                bestFace = face;
                referenceIsA = isA;
            }
        }
        return true;
    };
    if (!scan(a, b, true) || !scan(b, a, false))
    {
        return false;
    }

    // Two lines or a line and a point have no ends among their faces, so being
    // level with every face says nothing unless the lines really cross.
    bool overlapping = bestFace >= 0 && best <= 0.0f;
    if (a.count < 3 && b.count < 3)
    {
        overlapping = overlapping && a.count == 2 && b.count == 2 && LinesCross(a.points, b.points);
    }
    if (overlapping)
    {
        // The hulls overlap: out through the nearest face.
        if (ClipFace(referenceIsA ? a : b, referenceIsA ? b : a, bestFace, referenceIsA, out))
        {
            return true;
        }
        const Hull& reference = referenceIsA ? a : b;
        const Hull& other = referenceIsA ? b : a;
        const Vec2 n = reference.normals[bestFace];
        Vec2 deepest = other.points[0];
        for (int i = 1; i < other.count; ++i)
        {
            if (Dot(n, other.points[i]) < Dot(n, deepest))
            {
                deepest = other.points[i];
            }
        }
        out.normal = referenceIsA ? n : Scale(n, -1.0f);
        out.pointCount = 1;
        out.points[0].position = deepest;
        out.points[0].penetration = totalRadius - Dot(Sub(deepest, reference.points[bestFace]), n);
        return true;
    }

    // Apart, with only their radii reaching across: the nearest points.
    float nearestSq = kInfinity;
    Vec2 onA{}, onB{};
    auto closest = [&](const Hull& points, const Hull& edges, bool pointsAreA)
    {
        const int edgeCount = edges.count == 2 ? 1 : edges.count;
        for (int i = 0; i < points.count; ++i)
        {
            for (int e = 0; e < edgeCount; ++e)
            {
                const Vec2 q = NearestOnSegment(points.points[i], edges.points[e], edges.points[(e + 1) % edges.count]);
                const Vec2 d = Sub(q, points.points[i]);
                const float distSq = Dot(d, d);
                if (distSq < nearestSq)
                {
                    nearestSq = distSq;
                    onA = pointsAreA ? points.points[i] : q;
                    onB = pointsAreA ? q : points.points[i];
                }
            }
        }
    };
    closest(a, b, true);
    closest(b, a, false);
    const float reach = totalRadius + kContactMargin;
    if (nearestSq >= reach * reach)
    {
        return false;
    }
    const float distance = std::sqrt(nearestSq);

    // The face itself is the nearest thing, side by side: a flat contact.
    if (bestFace >= 0 && best >= distance - 1e-3f &&
        ClipFace(referenceIsA ? a : b, referenceIsA ? b : a, bestFace, referenceIsA, out))
    {
        return true;
    }

    out.normal = distance > 1e-6f ? Scale(Sub(onB, onA), Inverse(distance)) : Vec2{ 0.0f, 1.0f };
    out.pointCount = 1;
    out.points[0].position = Add(onA, Scale(out.normal, a.radius));
    out.points[0].penetration = totalRadius - distance;
    return true;
}

bool World::Collide(uint32_t a, uint32_t b, Manifold& out) const
{
    Hull ha, hb;
    MakeHull(m_Shapes[a], ha);
    MakeHull(m_Shapes[b], hb);
    out.shapeA = a;
    out.shapeB = b;
    return CollideHulls(ha, hb, out);
}

bool World::Touches(const Manifold& contact)
{
    for (int i = 0; i < contact.pointCount; ++i)
    {
        if (contact.points[i].penetration > 0.0f)
        {
            return true;
        }
    }
    return false;
}

void World::PrepareContact(Manifold& contact)
{
    const Shape& sa = m_Shapes[contact.shapeA];
    const Shape& sb = m_Shapes[contact.shapeB];
    const Body& a = m_Bodies[sa.body];
    const Body& b = m_Bodies[sb.body];
    const Vec2 normal = contact.normal;
    const Vec2 tangent = { -normal.y, normal.x };
    const float bounciness = std::max(sa.bounciness, sb.bounciness);
    // Kept for the passes that follow: each would find the two shapes again
    // for their bodies, and take the root for the friction every time.
    contact.bodyA = sa.body;
    contact.bodyB = sb.body;
    // Where the two are as the overlap is measured: each sub-step works out
    // what is left of it from where they have got to.
    contact.apart = Sub(b.position, a.position);
    contact.angleA = a.angle;
    contact.angleB = b.angle;
    contact.friction = std::sqrt(sa.friction * sb.friction);
    contact.againstFixed = a.invMass == 0.0f || b.invMass == 0.0f;

    const float invMass = a.invMass + b.invMass;
    for (int i = 0; i < contact.pointCount; ++i)
    {
        ContactPoint& point = contact.points[i];
        // A pass needs the point only as these four numbers: how a turn of
        // either body moves it along the normal, and along the tangent.
        const Vec2 toA = Sub(point.position, a.position);
        const Vec2 toB = Sub(point.position, b.position);
        point.leverA = Cross(toA, normal);
        point.leverB = Cross(toB, normal);
        point.slideA = Cross(toA, tangent);
        point.slideB = Cross(toB, tangent);
        const float invN =
            invMass + a.invInertia * point.leverA * point.leverA + b.invInertia * point.leverB * point.leverB;
        const float invT =
            invMass + a.invInertia * point.slideA * point.slideA + b.invInertia * point.slideB * point.slideB;
        point.normalMass = invN > 0.0f ? Inverse(invN) : 0.0f;
        point.tangentMass = invT > 0.0f ? Inverse(invT) : 0.0f;

        // A bounce gives back the speed the two met with: the speed they
        // arrived at, before this step's gravity and forces were added. With
        // those in it, every bounce returned a step of gravity more than it
        // took, and a ball that keeps all its speed rose higher each time.
        const float along =
            Dot(Sub(b.arriving, a.arriving), normal) + b.arrivingSpin * point.leverB - a.arrivingSpin * point.leverA;
        point.bounce = -along > kBounceThreshold ? -bounciness * along : 0.0f;
        point.most = 0.0f;
    }

    // Two points are solved together (SolvePair) when they are far enough
    // apart to tell from one: how each one's push moves the other.
    contact.paired = false;
    if (contact.pointCount == 2)
    {
        const ContactPoint& p1 = contact.points[0];
        const ContactPoint& p2 = contact.points[1];
        const float k11 = invMass + a.invInertia * p1.leverA * p1.leverA + b.invInertia * p1.leverB * p1.leverB;
        const float k22 = invMass + a.invInertia * p2.leverA * p2.leverA + b.invInertia * p2.leverB * p2.leverB;
        const float k12 = invMass + a.invInertia * p1.leverA * p2.leverA + b.invInertia * p1.leverB * p2.leverB;
        const float det = k11 * k22 - k12 * k12;
        if (k11 * k11 < kMaxPairCondition * det)
        {
            contact.paired = true;
            contact.k11 = k11;
            contact.k12 = k12;
            contact.k22 = k22;
            // Worked out here for the two usual cases, neither point a
            // spring or both: a division a pass otherwise.
            const float stiff = 1.0f + SoftOf(contact).spring;
            contact.invDet = Inverse(det);
            contact.invDetSprung = Inverse(k11 * k22 * stiff * stiff - k12 * k12);
        }
    }
}

const World::Manifold* World::FindLastManifold(uint64_t key) const
{
    const auto last = std::lower_bound(m_LastOrder.begin(), m_LastOrder.end(), key,
                                       [](const LastKey& entry, uint64_t k) { return entry.key < k; });
    return last != m_LastOrder.end() && last->key == key ? &m_LastManifolds[last->index] : nullptr;
}

// A contact starts from the pushes it ended the last step with, and its two
// bodies are given what those add to their speeds, as every push of the
// step then is (Body::held): each sub-step starts from it, without going
// over the contacts again.
void World::TakeLastPushes(Manifold& contact)
{
    const Manifold* last = FindLastManifold(PairKey(contact.shapeA, contact.shapeB));
    if (!last || last->pointCount != contact.pointCount || Dot(last->normal, contact.normal) < 0.95f)
    {
        return;
    }
    Body& a = m_Bodies[contact.bodyA];
    Body& b = m_Bodies[contact.bodyB];
    const Vec2 normal = contact.normal;
    const Vec2 tangent = { -normal.y, normal.x };
    for (int i = 0; i < contact.pointCount; ++i)
    {
        ContactPoint& point = contact.points[i];
        point.normalImpulse = last->points[i].normalImpulse;
        point.tangentImpulse = last->points[i].tangentImpulse;
        const Vec2 impulse = Add(Scale(normal, point.normalImpulse), Scale(tangent, point.tangentImpulse));
        const float turnA = point.leverA * point.normalImpulse + point.slideA * point.tangentImpulse;
        const float turnB = point.leverB * point.normalImpulse + point.slideB * point.tangentImpulse;
        a.held = Sub(a.held, Scale(impulse, a.invMass));
        a.heldSpin -= a.invInertia * turnA;
        b.held = Add(b.held, Scale(impulse, b.invMass));
        b.heldSpin += b.invInertia * turnB;
    }
}

// One pass over a contact in a sub-step of `1 / perSecond`. With `pushOut`
// an overlap past kSlop is given the speed that takes it out, as a spring
// would; the pass after the bodies have moved runs without, and takes that
// speed away again, so the push moves the bodies and adds no energy.
void World::SolveContact(Manifold& contact, float perSecond, bool pushOut, bool reverse)
{
    Body& a = m_Bodies[contact.bodyA];
    Body& b = m_Bodies[contact.bodyB];
    const Vec2 normal = contact.normal;
    const Vec2 tangent = { -normal.y, normal.x };
    const Soft& soft = SoftOf(contact);
    const float massA = a.invMass, massB = b.invMass, inertiaA = a.invInertia, inertiaB = b.invInertia;

    // The two bodies' speeds, worked on here and put back at the end.
    Vec2 speedA = a.velocity, speedB = b.velocity;
    float spinA = a.angularVelocity, spinB = b.angularVelocity;

    // How fast a point's two sides part along the normal, and slide along
    // the tangent; and a push along each.
    auto parting = [&](const ContactPoint& point)
    {
        return (speedB.x - speedA.x) * normal.x + (speedB.y - speedA.y) * normal.y + spinB * point.leverB -
               spinA * point.leverA;
    };
    auto sliding = [&](const ContactPoint& point)
    {
        return (speedB.x - speedA.x) * tangent.x + (speedB.y - speedA.y) * tangent.y + spinB * point.slideB -
               spinA * point.slideA;
    };
    auto push = [&](Vec2 along, float leverA, float leverB, float impulse)
    {
        speedA.x -= along.x * impulse * massA;
        speedA.y -= along.y * impulse * massA;
        spinA -= inertiaA * leverA * impulse;
        speedB.x += along.x * impulse * massB;
        speedB.y += along.y * impulse * massB;
        spinB += inertiaB * leverB * impulse;
    };

    // How far the two have moved and turned since the overlap was measured.
    const float moved = Dot(Sub(Sub(b.position, a.position), contact.apart), normal);
    const float turnedA = a.angle - contact.angleA;
    const float turnedB = b.angle - contact.angleB;

    // What a point is to do in this pass, from the gap between the two where
    // they are now (below zero, an overlap).
    auto aimOf = [&](const ContactPoint& point)
    {
        const float gap = moved + turnedB * point.leverB - turnedA * point.leverA - point.penetration;
        Aim aim;
        if (gap > -kRestDepth)
        {
            // Not yet resting on the other: it may close what is left in this
            // sub-step, and is let down instead of held in the air.
            aim.speed = -(gap + kRestDepth) * perSecond;
        }
        else if (gap < -kSlop && pushOut)
        {
            aim.speed = std::min(-soft.rate * (gap + kSlop), kMaxPushSpeed);
            aim.spring = soft.spring;
            aim.share = soft.share;
        }
        return aim;
    };
    // Friction is solved once a sub-step, in the pass after the bodies have
    // moved: that is the pass whose pushes are the ones kept, and friction
    // for the larger pushes of the pass before made a stack sway.
    auto friction = [&](ContactPoint& point)
    {
        if (pushOut)
        {
            return;
        }
        const float limit = contact.friction * point.normalImpulse;
        const float total = Clamp(point.tangentImpulse - point.tangentMass * sliding(point), -limit, limit);
        push(tangent, point.slideA, point.slideB, total - point.tangentImpulse);
        point.tangentImpulse = total;
    };
    auto setPush = [&](ContactPoint& point, float total)
    {
        push(normal, point.leverA, point.leverB, total - point.normalImpulse);
        point.normalImpulse = total;
        point.most = std::max(point.most, total);
    };

    if (contact.paired)
    {
        // Friction at each point for the push it carries, then both pushes at
        // once.
        friction(contact.points[reverse ? 1 : 0]);
        friction(contact.points[reverse ? 0 : 1]);
        ContactPoint& p1 = contact.points[0];
        ContactPoint& p2 = contact.points[1];
        const Aim aim1 = aimOf(p1);
        const Aim aim2 = aimOf(p2);
        float push1 = p1.normalImpulse, push2 = p2.normalImpulse;
        SolvePair(contact, parting(p1), parting(p2), aim1, aim2, push1, push2);
        setPush(p1, push1);
        setPush(p2, push2);
    }
    else
    {
        // Solved in turns from either end: always starting at the same corner
        // leaves a little spin behind each step, and a stack walks off
        // sideways.
        for (int k = 0; k < contact.pointCount; ++k)
        {
            ContactPoint& point = contact.points[reverse ? contact.pointCount - 1 - k : k];
            friction(point);

            // The running total is what is limited, so a later pass can take
            // back what an earlier one overdid. A spring makes a share of the
            // push and gives back a share of the push so far.
            const Aim aim = aimOf(point);
            const float change =
                (point.normalMass * (aim.speed - parting(point)) - aim.spring * point.normalImpulse) * aim.share;
            setPush(point, std::max(point.normalImpulse + change, 0.0f));
        }
    }

    // What the pass added to each body's speed is added to what its contacts
    // hold it with.
    a.held.x += speedA.x - a.velocity.x;
    a.held.y += speedA.y - a.velocity.y;
    a.heldSpin += spinA - a.angularVelocity;
    b.held.x += speedB.x - b.velocity.x;
    b.held.y += speedB.y - b.velocity.y;
    b.heldSpin += spinB - b.angularVelocity;
    a.velocity = speedA;
    a.angularVelocity = spinA;
    b.velocity = speedB;
    b.angularVelocity = spinB;
}

// The pushes of a contact's two points, found together. Each push must be
// zero or more and leave its point not closing; solved one after the other,
// each undoes part of what the other did, and a box set down flat rocks from
// corner to corner: a tall stack then leans and falls. Together there are
// four cases (both push, either alone, neither), tried in that order; the
// first that fits is the answer. `parting1` and `parting2` are how fast the
// points part now, `aim1` and `aim2` what each is to do; `push1` and `push2`
// come in as the pushes so far and go out as the new ones.
void World::SolvePair(const Manifold& contact, float parting1, float parting2, const Aim& aim1, const Aim& aim2,
                      float& push1, float& push2)
{
    // What the two would close at, past what they are to part at, with no
    // push at all.
    const float b1 = parting1 - aim1.speed - (contact.k11 * push1 + contact.k12 * push2);
    const float b2 = parting2 - aim2.speed - (contact.k12 * push1 + contact.k22 * push2);
    // A spring gives under its own push: more on the point's own term.
    const float k11 = contact.k11 * (1.0f + aim1.spring);
    const float k22 = contact.k22 * (1.0f + aim2.spring);
    const float k12 = contact.k12;
    // What one point alone would push: one over its own term.
    const float alone1 = -b1 * contact.points[0].normalMass * aim1.share;
    const float alone2 = -b2 * contact.points[1].normalMass * aim2.share;

    // Both push.
    const bool sprung = aim1.spring != 0.0f && aim2.spring != 0.0f;
    const float invDet = sprung                                       ? contact.invDetSprung
                         : aim1.spring == 0.0f && aim2.spring == 0.0f ? contact.invDet
                                                                      : Inverse(k11 * k22 - k12 * k12);
    const float both1 = -(k22 * b1 - k12 * b2) * invDet;
    const float both2 = -(k11 * b2 - k12 * b1) * invDet;
    if (both1 >= 0.0f && both2 >= 0.0f)
    {
        push1 = both1;
        push2 = both2;
    }
    // Only the first pushes, and the second is not closing.
    else if (alone1 >= 0.0f && k12 * alone1 + b2 >= 0.0f)
    {
        push1 = alone1;
        push2 = 0.0f;
    }
    // Only the second.
    else if (alone2 >= 0.0f && k12 * alone2 + b1 >= 0.0f)
    {
        push1 = 0.0f;
        push2 = alone2;
    }
    // Neither, when both are parting by themselves.
    else if (b1 >= 0.0f && b2 >= 0.0f)
    {
        push1 = 0.0f;
        push2 = 0.0f;
    }
    // (When no case fits, which rounding allows, the pushes stay as they are.)
}

// A bounce gives back the speed the two met with, once the step has stopped
// them: only where they did push on each other.
void World::Bounce(Manifold& contact)
{
    Body& a = m_Bodies[contact.bodyA];
    Body& b = m_Bodies[contact.bodyB];
    const Vec2 normal = contact.normal;
    for (int i = 0; i < contact.pointCount; ++i)
    {
        ContactPoint& point = contact.points[i];
        if (point.bounce <= 0.0f || point.most <= 0.0f)
        {
            continue;
        }
        const float along = Dot(Sub(b.velocity, a.velocity), normal) + b.angularVelocity * point.leverB -
                            a.angularVelocity * point.leverA;
        const float total = std::max(point.normalImpulse + point.normalMass * (point.bounce - along), 0.0f);
        const float change = total - point.normalImpulse;
        point.normalImpulse = total;
        a.velocity = Sub(a.velocity, Scale(normal, change * a.invMass));
        a.angularVelocity -= a.invInertia * point.leverA * change;
        b.velocity = Add(b.velocity, Scale(normal, change * b.invMass));
        b.angularVelocity += b.invInertia * point.leverB * change;
    }
}

void World::PrepareJoint(Joint& joint, float stepsPerSecond)
{
    Body& a = m_Bodies[joint.bodyA];
    Body& b = BodyAt(joint.bodyB);
    joint.toA = Turned(a, Sub(joint.localA, a.centre));
    joint.toB = Turned(b, Sub(joint.localB, b.centre));
    const float mA = a.invMass, mB = b.invMass, iA = a.invInertia, iB = b.invInertia;
    const Vec2 error = Sub(Add(b.position, joint.toB), Add(a.position, joint.toA));

    if (!joint.hinge)
    {
        const float length = std::sqrt(Dot(error, error));
        joint.slack = length < 1e-6f || (joint.rope && length <= joint.length);
        if (joint.slack)
        {
            joint.axisImpulse = 0.0f;
            return;
        }
        joint.axis = Scale(error, Inverse(length));
        const float crA = Cross(joint.toA, joint.axis);
        const float crB = Cross(joint.toB, joint.axis);
        const float inverse = mA + mB + iA * crA * crA + iB * crB * crB;
        joint.axisMass = inverse > 0.0f ? Inverse(inverse) : 0.0f;
        joint.axisBias =
            Clamp(length - joint.length, -kMaxJointError, kMaxJointError) * kJointCorrection * stepsPerSecond;
        return;
    }

    joint.k11 = mA + mB + iA * joint.toA.y * joint.toA.y + iB * joint.toB.y * joint.toB.y;
    joint.k12 = -iA * joint.toA.x * joint.toA.y - iB * joint.toB.x * joint.toB.y;
    joint.k22 = mA + mB + iA * joint.toA.x * joint.toA.x + iB * joint.toB.x * joint.toB.x;
    const float distance = std::sqrt(Dot(error, error));
    const float keep = distance > kMaxJointError ? kMaxJointError * Inverse(distance) : 1.0f;
    joint.pointBias = Scale(error, keep * kJointCorrection * stepsPerSecond);
    joint.axialMass = iA + iB > 0.0f ? Inverse(iA + iB) : 0.0f;
    const float det = joint.k11 * joint.k22 - joint.k12 * joint.k12;
    joint.invDet = det > 0.0f ? Inverse(det) : 0.0f;

    int state = 0;
    joint.limitBias = 0.0f;
    if (joint.limited)
    {
        const float angle = b.angle - a.angle - joint.referenceAngle;
        if (angle <= joint.lowerAngle)
        {
            state = 1;
            joint.limitBias = (angle - joint.lowerAngle) * kJointCorrection * stepsPerSecond;
        }
        else if (angle >= joint.upperAngle)
        {
            state = 2;
            joint.limitBias = (angle - joint.upperAngle) * kJointCorrection * stepsPerSecond;
        }
    }
    if (state != joint.limitState)
    {
        joint.limitImpulse = 0.0f;
        joint.limitState = state;
    }
}

void World::WarmStartJoint(const Joint& joint)
{
    Body& a = m_Bodies[joint.bodyA];
    Body& b = BodyAt(joint.bodyB);
    float axial = 0.0f;
    Vec2 impulse = joint.pointImpulse;
    if (joint.hinge)
    {
        axial = joint.limitImpulse + joint.motorImpulse;
    }
    else if (joint.slack)
    {
        return;
    }
    else
    {
        impulse = Scale(joint.axis, joint.axisImpulse);
    }
    a.velocity = Sub(a.velocity, Scale(impulse, a.invMass));
    a.angularVelocity -= a.invInertia * (Cross(joint.toA, impulse) + axial);
    b.velocity = Add(b.velocity, Scale(impulse, b.invMass));
    b.angularVelocity += b.invInertia * (Cross(joint.toB, impulse) + axial);
}

// A joint's pass in a sub-step of `seconds`.
void World::SolveJoint(Joint& joint, float seconds)
{
    Body& a = m_Bodies[joint.bodyA];
    Body& b = BodyAt(joint.bodyB);
    const float mA = a.invMass, mB = b.invMass, iA = a.invInertia, iB = b.invInertia;
    auto applyAxial = [&](float impulse)
    {
        a.angularVelocity -= iA * impulse;
        b.angularVelocity += iB * impulse;
    };
    auto applyLinear = [&](Vec2 impulse)
    {
        a.velocity = Sub(a.velocity, Scale(impulse, mA));
        a.angularVelocity -= iA * Cross(joint.toA, impulse);
        b.velocity = Add(b.velocity, Scale(impulse, mB));
        b.angularVelocity += iB * Cross(joint.toB, impulse);
    };
    auto pointSpeed = [&]()
    {
        return Sub(Add(b.velocity, Perp(b.angularVelocity, joint.toB)),
                   Add(a.velocity, Perp(a.angularVelocity, joint.toA)));
    };

    if (!joint.hinge)
    {
        if (joint.slack)
        {
            return;
        }
        const float change = -joint.axisMass * (Dot(pointSpeed(), joint.axis) + joint.axisBias);
        float total = joint.axisImpulse + change;
        if (joint.rope)
        {
            total = std::min(total, 0.0f);  // a rope pulls and never pushes
        }
        applyLinear(Scale(joint.axis, total - joint.axisImpulse));
        joint.axisImpulse = total;
        return;
    }

    const float spin = b.angularVelocity - a.angularVelocity;
    if (joint.motor)
    {
        const float limit = joint.maxMotorTorque * seconds;
        const float total = Clamp(joint.motorImpulse - joint.axialMass * (spin - joint.motorSpeed), -limit, limit);
        applyAxial(total - joint.motorImpulse);
        joint.motorImpulse = total;
    }
    if (joint.limitState != 0)
    {
        const float current = b.angularVelocity - a.angularVelocity;
        float total = joint.limitImpulse - joint.axialMass * (current + joint.limitBias);
        total = joint.limitState == 1 ? std::max(total, 0.0f) : std::min(total, 0.0f);
        applyAxial(total - joint.limitImpulse);
        joint.limitImpulse = total;
    }

    const Vec2 speed = Add(pointSpeed(), joint.pointBias);
    const Vec2 impulse{ -(joint.k22 * speed.x - joint.k12 * speed.y) * joint.invDet,
                        -(joint.k11 * speed.y - joint.k12 * speed.x) * joint.invDet };
    joint.pointImpulse = Add(joint.pointImpulse, impulse);
    applyLinear(impulse);
}

bool World::DoNotCollide(const Shape& a, const Shape& b) const
{
    if (a.body == b.body || !(a.layer & b.mask) || !(b.layer & a.mask))
    {
        return true;
    }
    if (m_Bodies[a.body].jointCount == 0 || m_Bodies[b.body].jointCount == 0)
    {
        return false;
    }
    for (const Joint& joint : m_Joints)
    {
        if (joint.alive && !joint.collideConnected &&
            ((joint.bodyA == a.body && joint.bodyB == b.body) || (joint.bodyA == b.body && joint.bodyB == a.body)))
        {
            return true;
        }
    }
    return false;
}

void World::RefreshSweep()
{
    for (uint32_t index : m_Sweep)
    {
        Shape& shape = m_Shapes[index];
        const Body& body = m_Bodies[shape.body];
        if (!shape.boundsValid || body.moved || body.type == BodyType::Kinematic || body.awake)
        {
            shape.bounds = BoundsOf(shape);
            shape.boundsValid = true;
        }
    }

    // Sorted along X. Insertion sort, because the order barely changes
    // between steps and it is then close to linear.
    for (size_t i = 1; i < m_Sweep.size(); ++i)
    {
        const uint32_t value = m_Sweep[i];
        const float minX = m_Shapes[value].bounds.minX;
        size_t j = i;
        while (j > 0 && m_Shapes[m_Sweep[j - 1]].bounds.minX > minX)
        {
            m_Sweep[j] = m_Sweep[j - 1];
            --j;
        }
        m_Sweep[j] = value;
    }
}

void World::FindContacts()
{
    RefreshSweep();

    // Kept, sorted by pair, so this step's contacts can start from last step's
    // pushes: a stack is then already held up before the solver begins.
    // Their order by pair is sorted, not the contacts: those are a good deal
    // larger to move about.
    m_LastManifolds.swap(m_Manifolds);
    m_LastOrder.clear();
    for (uint32_t i = 0; i < m_LastManifolds.size(); ++i)
    {
        m_LastOrder.push_back({ PairKey(m_LastManifolds[i].shapeA, m_LastManifolds[i].shapeB), i });
    }
    std::sort(m_LastOrder.begin(), m_LastOrder.end(), [](const LastKey& x, const LastKey& y) { return x.key < y.key; });

    auto active = [](const Body& body)
    {
        return body.moved || body.awake ||
               (body.type == BodyType::Kinematic &&
                (body.velocity.x != 0.0f || body.velocity.y != 0.0f || body.angularVelocity != 0.0f));
    };

    // Waking a body brings its contacts into play, so the pass runs again;
    // that only happens on the steps something wakes.
    bool woke;
    do
    {
        woke = false;
        m_Manifolds.clear();
        m_Touching.clear();
        for (size_t i = 0; i < m_Sweep.size(); ++i)
        {
            const uint32_t a = m_Sweep[i];
            const Shape& sa = m_Shapes[a];
            for (size_t k = i + 1; k < m_Sweep.size(); ++k)
            {
                const uint32_t b = m_Sweep[k];
                const Shape& sb = m_Shapes[b];
                if (sb.bounds.minX >= sa.bounds.maxX + kContactMargin)
                {
                    break;  // sorted: nothing later can be near on X
                }
                if (sb.bounds.minY >= sa.bounds.maxY + kContactMargin ||
                    sa.bounds.minY >= sb.bounds.maxY + kContactMargin || DoNotCollide(sa, sb))
                {
                    continue;
                }
                Body& bodyA = m_Bodies[sa.body];
                Body& bodyB = m_Bodies[sb.body];
                const bool solid = !sa.trigger && !sb.trigger;
                if (solid && bodyA.type != BodyType::Dynamic && bodyB.type != BodyType::Dynamic)
                {
                    continue;
                }

                const uint64_t key = PairKey(a, b);
                if (!active(bodyA) && !active(bodyB))
                {
                    // Nothing here can have changed: still touching if it was,
                    // and its pushes are kept for when it wakes.
                    if (std::binary_search(m_WasTouching.begin(), m_WasTouching.end(), key))
                    {
                        m_Touching.push_back(key);
                        if (const Manifold* last = FindLastManifold(key))
                        {
                            m_Manifolds.push_back(*last);
                            m_Manifolds.back().resting = true;
                        }
                    }
                    continue;
                }

                // Worked out where it will stay, and taken off again when
                // the two turn out apart or one is a trigger.
                m_Manifolds.emplace_back();
                const bool near = Collide(a, b, m_Manifolds.back());
                // Near enough for a contact is not yet touching: nothing is
                // reported, and a trigger is not entered.
                if (near && Touches(m_Manifolds.back()))
                {
                    m_Touching.push_back(key);
                }
                if (!near || !solid)
                {
                    m_Manifolds.pop_back();
                }
                if (near && solid)
                {
                    for (const uint32_t index : { sa.body, sb.body })
                    {
                        Body& body = m_Bodies[index];
                        if (body.type == BodyType::Dynamic && !body.awake)
                        {
                            Wake(index);
                            woke = true;
                        }
                    }
                }
            }
        }
    } while (woke);
    std::sort(m_Touching.begin(), m_Touching.end());
}

bool World::IsFast(const Body& body, float dt) const
{
    if (body.type != BodyType::Dynamic || !body.awake || !body.continuous || body.thinnest <= 0.0f)
    {
        return false;
    }
    return Dot(body.velocity, body.velocity) * dt * dt > body.thinnest * body.thinnest;
}

// Takes a fast body along the path it went this step, in steps too short to
// pass over anything, and stops it just inside the first thing it meets, so
// the next step's contact takes it from there.
void World::SweepBody(uint32_t bodyIndex)
{
    Body& body = m_Bodies[bodyIndex];
    const Vec2 start = body.from;
    const float startAngle = body.fromAngle;
    const Vec2 motion = Sub(body.position, start);
    const float turn = body.angle - startAngle;

    const float distance = std::sqrt(Dot(motion, motion));
    const int steps = std::min(static_cast<int>(std::ceil(distance / body.thinnest)), kMaxSweepSteps);
    auto place = [&](float t) { SetPose(body, Add(start, Scale(motion, t)), startAngle + turn * t); };

    auto hitsSomething = [&]()
    {
        for (uint32_t own : m_Sweep)
        {
            const Shape& mine = m_Shapes[own];
            if (mine.body != bodyIndex || mine.trigger)
            {
                continue;
            }
            const Aabb bounds = BoundsOf(mine);
            for (uint32_t other : m_Sweep)
            {
                const Shape& theirs = m_Shapes[other];
                if (theirs.bounds.minX >= bounds.maxX)
                {
                    break;  // sorted: nothing later can reach it
                }
                if (theirs.trigger || bounds.minX >= theirs.bounds.maxX || theirs.bounds.minY >= bounds.maxY ||
                    bounds.minY >= theirs.bounds.maxY || DoNotCollide(mine, theirs))
                {
                    continue;
                }
                // What it already touches is the contact solver's.
                if (std::binary_search(m_Touching.begin(), m_Touching.end(), PairKey(own, other)))
                {
                    continue;
                }
                Manifold manifold{};
                if (Collide(own, other, manifold) && Touches(manifold))
                {
                    return true;
                }
            }
        }
        return false;
    };

    for (int s = 1; s <= steps; ++s)
    {
        const float t = static_cast<float>(s) / static_cast<float>(steps);
        place(t);
        if (!hitsSomething())
        {
            continue;
        }
        float clear = static_cast<float>(s - 1) / static_cast<float>(steps);
        float hit = t;
        for (int i = 0; i < kSweepRefinements; ++i)
        {
            const float middle = (clear + hit) * 0.5f;
            place(middle);
            if (hitsSomething())
            {
                hit = middle;
            }
            else
            {
                clear = middle;
            }
        }
        place(hit);
        return;
    }
    place(1.0f);
}

// Where a fast body was before it moved is kept: it is taken along its path
// again once the sub-steps are done (SweepFastBodies), to find what it went
// through.
void World::IntegratePositions(float seconds)
{
    for (Body& body : m_Bodies)
    {
        if (!body.alive || body.type == BodyType::Static || (body.type == BodyType::Dynamic && !body.awake))
        {
            continue;
        }
        if (!body.swept && IsFast(body, fixedStep))
        {
            body.swept = true;
            body.from = body.position;
            body.fromAngle = body.angle;
        }
        body.position.x += body.velocity.x * seconds;
        body.position.y += body.velocity.y * seconds;
        if (body.angularVelocity != 0.0f)
        {
            // Its sine and cosine follow, for the joints of the next
            // sub-step: turned by the little it turned, and brought back to
            // length one. The step ends by working them out afresh.
            const float turn = body.angularVelocity * seconds;
            const float c = body.cosAngle - body.sinAngle * turn;
            const float s = body.sinAngle + body.cosAngle * turn;
            const float keep = 1.5f - 0.5f * (c * c + s * s);
            body.cosAngle = c * keep;
            body.sinAngle = s * keep;
            body.angle += turn;
        }
    }
}

// After the rest, so each fast body meets everything where it ended up: the
// boxes it is tested against are brought to where the others are now, once,
// before the first of them. (One fast body is still tested against where
// another began the step.)
void World::SweepFastBodies()
{
    bool refreshed = false;
    for (uint32_t i = 0; i < m_Bodies.size(); ++i)
    {
        Body& body = m_Bodies[i];
        if (!body.alive || !body.swept)
        {
            continue;
        }
        if (!refreshed)
        {
            RefreshSweep();
            refreshed = true;
        }
        body.swept = false;
        SweepBody(i);
    }
}

uint32_t World::FindIsland(uint32_t bodyIndex)
{
    while (m_Bodies[bodyIndex].island != bodyIndex)
    {
        m_Bodies[bodyIndex].island = m_Bodies[m_Bodies[bodyIndex].island].island;
        bodyIndex = m_Bodies[bodyIndex].island;
    }
    return bodyIndex;
}

void World::JoinIslands(uint32_t a, uint32_t b)
{
    if (m_Bodies[a].type == BodyType::Dynamic && m_Bodies[b].type == BodyType::Dynamic)
    {
        m_Bodies[FindIsland(a)].island = FindIsland(b);
    }
}

// A body sleeps only with everything it touches, or the bottom of a settled
// stack would freeze under a top that is still moving.
void World::UpdateSleep(float dt)
{
    if (!allowSleep)
    {
        return;
    }
    bool anyReady = false;
    for (Body& body : m_Bodies)
    {
        if (!body.alive || body.type != BodyType::Dynamic || !body.awake)
        {
            continue;
        }
        const bool still = body.canSleep && Dot(body.velocity, body.velocity) < kSleepSpeed * kSleepSpeed &&
                           body.angularVelocity * body.angularVelocity < kSleepSpin * kSleepSpin;
        body.sleepTime = still ? body.sleepTime + dt : 0.0f;
        anyReady = anyReady || body.sleepTime >= kTimeToSleep;
    }
    if (!anyReady)
    {
        return;
    }

    for (uint32_t i = 0; i < m_Bodies.size(); ++i)
    {
        Body& body = m_Bodies[i];
        if (body.alive && body.type == BodyType::Dynamic && body.awake)
        {
            body.island = i;
            body.islandTime = kInfinity;
        }
    }
    for (const Manifold& contact : m_Manifolds)
    {
        if (!contact.resting)
        {
            JoinIslands(contact.bodyA, contact.bodyB);
        }
    }
    for (const Joint& joint : m_Joints)
    {
        if (joint.alive && joint.bodyB != kNoBody)
        {
            JoinIslands(joint.bodyA, joint.bodyB);
        }
    }
    for (uint32_t i = 0; i < m_Bodies.size(); ++i)
    {
        const Body& body = m_Bodies[i];
        if (body.alive && body.type == BodyType::Dynamic && body.awake)
        {
            Body& root = m_Bodies[FindIsland(i)];
            root.islandTime = std::min(root.islandTime, body.sleepTime);
        }
    }
    for (uint32_t i = 0; i < m_Bodies.size(); ++i)
    {
        Body& body = m_Bodies[i];
        if (body.alive && body.type == BodyType::Dynamic && body.awake)
        {
            const uint32_t root = FindIsland(i);
            if (m_Bodies[root].islandTime >= kTimeToSleep)
            {
                body.island = root;
                body.velocity = {};
                body.angularVelocity = 0.0f;
                body.awake = false;
            }
        }
    }
}

bool World::AtRest() const
{
    for (const Body& body : m_Bodies)
    {
        if (!body.alive)
        {
            continue;
        }
        const bool moving = body.velocity.x != 0.0f || body.velocity.y != 0.0f || body.angularVelocity != 0.0f;
        if (body.moved || (body.type == BodyType::Dynamic && body.awake) ||
            (body.type == BodyType::Kinematic && moving))
        {
            return false;
        }
    }
    return true;
}

// A spring of `hertz` and kContactDamping, as numbers a pass over a sub-step
// of `seconds` can use.
World::Soft World::MakeSoft(float hertz, float seconds)
{
    const float omega = 2.0f * 3.14159265f * hertz;
    const float damped = 2.0f * kContactDamping + seconds * omega;
    const float spring = 1.0f / (seconds * omega * damped);
    return { omega / damped, spring, 1.0f / (1.0f + spring) };
}

void World::FixedStep()
{
    const float dt = fixedStep;

    if (gravity.x != m_LastGravity.x || gravity.y != m_LastGravity.y)
    {
        m_LastGravity = gravity;
        WakeAll();
    }

    // Nothing awake, nothing moving and nothing changed since the last step:
    // this one would find the same contacts and leave everything where it is.
    if (!m_Changed && AtRest())
    {
        return;
    }
    m_Changed = false;

    for (Body& body : m_Bodies)
    {
        body.arriving = body.velocity;
        body.arrivingSpin = body.angularVelocity;
        body.held = {};
        body.heldSpin = 0.0f;
    }

    FindContacts();

    // The step is solved in sub-steps: each moves the bodies a little and
    // solves from where they then are, so a push reaches the far end of a
    // tall stack or a long chain within the step. Several passes over one
    // whole step carry it a body a pass.
    const int count = std::max(subSteps, 1);
    const float h = dt / static_cast<float>(count);
    const float perSecond = 1.0f / h;
    const float stepsPerSecond = 1.0f / dt;
    m_Soft = MakeSoft(std::min(kContactHertz, 0.25f * perSecond), h);
    m_SoftAgainstFixed = MakeSoft(std::min(2.0f * kContactHertz, 0.5f * perSecond), h);

    auto jointIsAwake = [this](const Joint& joint)
    { return joint.alive && (m_Bodies[joint.bodyA].awake || (joint.bodyB != kNoBody && m_Bodies[joint.bodyB].awake)); };
    // Every contact starts from the pushes it ended the last step with: a
    // stack is then already held up before the solver begins. Each sub-step
    // gives a body its forces and what its contacts hold it with.
    for (Manifold& contact : m_Manifolds)
    {
        if (!contact.resting)
        {
            PrepareContact(contact);
            TakeLastPushes(contact);
        }
    }

    auto solveContacts = [&](bool correcting, bool reverse)
    {
        for (Manifold& contact : m_Manifolds)
        {
            if (!contact.resting)
            {
                SolveContact(contact, perSecond, correcting, reverse);
            }
        }
    };
    for (int step = 0; step < count; ++step)
    {
        for (Body& body : m_Bodies)
        {
            if (!body.alive || body.type != BodyType::Dynamic || !body.awake)
            {
                continue;
            }
            body.velocity.x += (gravity.x * body.gravityScale + body.force.x * body.invMass) * h;
            body.velocity.y += (gravity.y * body.gravityScale + body.force.y * body.invMass) * h;
            body.angularVelocity += body.torque * body.invInertia * h;
            const float keep = std::max(0.0f, 1.0f - body.linearDamping * h);
            body.velocity.x = body.velocity.x * keep + body.held.x;
            body.velocity.y = body.velocity.y * keep + body.held.y;
            body.angularVelocity =
                body.angularVelocity * std::max(0.0f, 1.0f - body.angularDamping * h) + body.heldSpin;
        }
        // A joint is set up from where its bodies have got to and solved
        // once, each sub-step: set up once a step it held them to where they
        // began it, and a pendulum lost its swing.
        for (Joint& joint : m_Joints)
        {
            if (jointIsAwake(joint))
            {
                PrepareJoint(joint, stepsPerSecond);
                WarmStartJoint(joint);
                SolveJoint(joint, h);
            }
        }
        solveContacts(true, step % 2 == 1);
        IntegratePositions(h);
        solveContacts(false, step % 2 == 0);
    }
    for (Manifold& contact : m_Manifolds)
    {
        if (!contact.resting)
        {
            Bounce(contact);
        }
    }
    SweepFastBodies();

    for (Body& body : m_Bodies)
    {
        body.force = {};
        body.torque = 0.0f;
    }
    for (Body& body : m_Bodies)
    {
        // Only a body the step could have turned: one asleep, or moved by
        // code and not spinning, has the angle it was last given (SetPose).
        const bool turned = body.type == BodyType::Dynamic ? body.awake : body.angularVelocity != 0.0f;
        if (body.alive && body.type != BodyType::Static && turned)
        {
            body.cosAngle = std::cos(body.angle);
            body.sinAngle = std::sin(body.angle);
        }
        body.moved = false;
    }

    UpdateSleep(dt);
    ReportContacts();
}

Contact World::MakeContact(uint32_t shapeA, uint32_t shapeB) const
{
    const Shape& sa = m_Shapes[shapeA];
    const Shape& sb = m_Shapes[shapeB];
    return { { shapeA, sa.generation },
             { shapeB, sb.generation },
             { sa.body, m_Bodies[sa.body].generation },
             { sb.body, m_Bodies[sb.body].generation },
             sa.trigger || sb.trigger };
}

void World::ReportContacts()
{
    auto report = [this](uint64_t key, const std::function<void(const Contact&)>& callback)
    {
        const uint32_t a = static_cast<uint32_t>(key >> 32);
        const uint32_t b = static_cast<uint32_t>(key);
        if (m_Shapes[a].alive && m_Shapes[b].alive && callback)
        {
            callback(MakeContact(a, b));
        }
    };

    // The lists change hands, with one copy made: last step's pairs to
    // report against, this step's to report, and to remember. A callback may
    // remove a shape, which takes its pairs out of the remembered list, so
    // the two reported from are the step's own.
    m_ReportBefore.swap(m_WasTouching);
    m_ReportNow.swap(m_Touching);
    m_Touching.clear();
    m_WasTouching = m_ReportNow;
    size_t i = 0, k = 0;
    while (i < m_ReportNow.size() || k < m_ReportBefore.size())
    {
        if (k == m_ReportBefore.size() || (i < m_ReportNow.size() && m_ReportNow[i] < m_ReportBefore[k]))
        {
            report(m_ReportNow[i++], onContactBegin);
        }
        else if (i == m_ReportNow.size() || m_ReportBefore[k] < m_ReportNow[i])
        {
            report(m_ReportBefore[k++], onContactEnd);
        }
        else
        {
            ++i;
            ++k;
        }
    }
}

int World::Step(float deltaSeconds)
{
    if (fixedStep <= 0.0f || deltaSeconds <= 0.0f)
    {
        return 0;
    }
    m_Banked += deltaSeconds;
    int steps = 0;
    // The tolerance is for rounding: 0.05 s holds three steps of 1/60 s, and
    // summing them in floats must not leave one short.
    while (m_Banked >= fixedStep - 1e-5f && steps < maxStepsPerFrame)
    {
        FixedStep();
        m_Banked -= fixedStep;
        ++steps;
    }
    if (m_Banked >= fixedStep - 1e-5f)
    {
        // Still a whole step behind after the most it may run: the game slows
        // down rather than spiralling. Less than a step is kept for the next
        // frame, as on any other.
        m_Banked = 0.0f;
    }
    return steps;
}

bool World::Raycast(Vec2 from, Vec2 to, RayHit& hit, uint16_t mask) const
{
    const Vec2 d = Sub(to, from);
    const float lengthSq = Dot(d, d);
    if (lengthSq <= 0.0f)
    {
        return false;
    }
    bool found = false;
    float best = 1.0f;

    // Where the ray enters the circle, if it does before `best`.
    auto throughCircle = [&](Vec2 centre, float radius, float& t, Vec2& normal)
    {
        const Vec2 f = Sub(from, centre);
        const float b = 2.0f * Dot(f, d);
        const float c = Dot(f, f) - radius * radius;
        const float disc = b * b - 4.0f * lengthSq * c;
        if (disc < 0.0f)
        {
            return false;
        }
        const float at = (-b - std::sqrt(disc)) / (2.0f * lengthSq);
        if (at < 0.0f || at > best)
        {
            return false;
        }
        t = at;
        normal = Scale(Sub(Add(from, Scale(d, at)), centre), 1.0f / radius);
        return true;
    };
    // Where the ray enters the convex outline, by its faces.
    auto throughFaces = [&](const Vec2* points, const Vec2* normals, int count, float& t, Vec2& normal)
    {
        float enter = 0.0f;
        float leave = best;
        int face = -1;
        for (int i = 0; i < count; ++i)
        {
            const float speed = Dot(normals[i], d);
            const float gap = Dot(normals[i], Sub(points[i], from));
            if (std::fabs(speed) < 1e-9f)
            {
                if (gap < 0.0f)
                {
                    return false;
                }
                continue;
            }
            const float at = gap / speed;
            if (speed < 0.0f)
            {
                if (at > enter)
                {
                    enter = at;
                    face = i;
                }
            }
            else
            {
                leave = std::min(leave, at);
            }
            if (enter > leave)
            {
                return false;
            }
        }
        t = enter;
        normal = face >= 0 ? normals[face] : Scale(d, -1.0f / std::sqrt(lengthSq));  // it starts inside
        return true;
    };

    for (uint32_t index : m_Sweep)
    {
        const Shape& shape = m_Shapes[index];
        if (!(shape.layer & mask) || shape.trigger)
        {
            continue;
        }
        Hull hull;
        MakeHull(shape, hull);
        float t = -1.0f;
        Vec2 normal;
        bool crossed = false;

        if (hull.count == 1)
        {
            crossed = throughCircle(hull.points[0], hull.radius, t, normal);
        }
        else if (hull.count == 2)
        {
            // The straight sides, then each round end.
            const Vec2 reach = Scale(hull.normals[0], hull.radius);
            const Vec2 edge = Sub(hull.points[1], hull.points[0]);
            const Vec2 along = Scale(edge, 1.0f / std::sqrt(Dot(edge, edge)));
            const Vec2 onFace[4] = { Add(hull.points[0], reach), Sub(hull.points[0], reach), hull.points[1],
                                     hull.points[0] };
            const Vec2 normals[4] = { hull.normals[0], hull.normals[1], along, Scale(along, -1.0f) };
            float side = -1.0f;
            Vec2 sideNormal;
            if (throughFaces(onFace, normals, 4, side, sideNormal))
            {
                t = side;
                normal = sideNormal;
                crossed = true;
            }
            for (const Vec2& end : { hull.points[0], hull.points[1] })
            {
                float round = -1.0f;
                Vec2 roundNormal;
                if (throughCircle(end, hull.radius, round, roundNormal) && (!crossed || round < t))
                {
                    t = round;
                    normal = roundNormal;
                    crossed = true;
                }
            }
        }
        else
        {
            crossed = throughFaces(hull.points, hull.normals, hull.count, t, normal);
        }
        if (!crossed)
        {
            continue;
        }

        found = true;
        best = t;
        hit.shape = { index, shape.generation };
        hit.body = { shape.body, m_Bodies[shape.body].generation };
        hit.point = Add(from, Scale(d, t));
        hit.normal = normal;
        hit.fraction = t;
    }
    return found;
}

}  // namespace DekiPhysics2D
