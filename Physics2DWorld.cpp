#include "Physics2DWorld.h"

#include <algorithm>
#include <cmath>

namespace DekiPhysics2D
{

namespace
{
// Overlap this small is left alone, so a resting body keeps touching what it
// rests on instead of jittering between touching and not.
constexpr float kSlop = 0.005f;
// The share of the remaining overlap a step removes: all of it at once makes
// stacks bounce.
constexpr float kCorrection = 0.5f;
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
    for (uint64_t key : m_Touching)
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
        for (uint64_t key : m_Touching)
        {
            if (involves(key))
            {
                onContactEnd(MakeContact(static_cast<uint32_t>(key >> 32), static_cast<uint32_t>(key)));
            }
        }
    }
    m_Touching.erase(std::remove_if(m_Touching.begin(), m_Touching.end(), involves), m_Touching.end());
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
    for (uint64_t key : m_Touching)
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
    for (int i = 0; i < shape.count; ++i)
    {
        const Vec2 p = Sub(shape.points[i], body.centre);
        const Vec2 n = shape.normals[i];
        out.points[i] = { body.position.x + p.x * c - p.y * s, body.position.y + p.x * s + p.y * c };
        out.normals[i] = { n.x * c - n.y * s, n.x * s + n.y * c };
    }
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
    const Vec2 tangent = Scale(edge, 1.0f / width);
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
            p0 = Add(p0, Scale(Sub(p1, p0), d0 / (d0 - d1)));
        }
        else if (d1 > 0.0f)
        {
            p1 = Add(p1, Scale(Sub(p0, p1), d1 / (d1 - d0)));
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
        if (separation < totalRadius)
        {
            out.points[out.pointCount].position = Sub(p, Scale(n, incident.radius));
            out.points[out.pointCount].penetration = totalRadius - separation;
            ++out.pointCount;
        }
    }
    return out.pointCount > 0;
}

// Separating axes over the faces of both hulls, then either the touching faces
// clipped together or, past a corner, the two nearest points.
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
            float separation = kInfinity;
            for (int i = 0; i < other.count; ++i)
            {
                separation =
                    std::min(separation, Dot(reference.normals[face], Sub(other.points[i], reference.points[face])));
            }
            if (separation > totalRadius)
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
    if (nearestSq >= totalRadius * totalRadius)
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

    out.normal = distance > 1e-6f ? Scale(Sub(onB, onA), 1.0f / distance) : Vec2{ 0.0f, 1.0f };
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

void World::PrepareContact(Manifold& contact)
{
    const Shape& sa = m_Shapes[contact.shapeA];
    const Shape& sb = m_Shapes[contact.shapeB];
    const Body& a = m_Bodies[sa.body];
    const Body& b = m_Bodies[sb.body];
    const Vec2 tangent = { -contact.normal.y, contact.normal.x };
    const float bounciness = std::max(sa.bounciness, sb.bounciness);
    // Kept for the passes that follow: each would find the two shapes again
    // for their bodies, and take the root for the friction every time.
    contact.bodyA = sa.body;
    contact.bodyB = sb.body;
    contact.friction = std::sqrt(sa.friction * sb.friction);

    for (int i = 0; i < contact.pointCount; ++i)
    {
        ContactPoint& point = contact.points[i];
        point.toA = Sub(point.position, a.position);
        point.toB = Sub(point.position, b.position);
        const float rnA = Cross(point.toA, contact.normal);
        const float rnB = Cross(point.toB, contact.normal);
        const float rtA = Cross(point.toA, tangent);
        const float rtB = Cross(point.toB, tangent);
        const float invN = a.invMass + b.invMass + a.invInertia * rnA * rnA + b.invInertia * rnB * rnB;
        const float invT = a.invMass + b.invMass + a.invInertia * rtA * rtA + b.invInertia * rtB * rtB;
        point.normalMass = invN > 0.0f ? 1.0f / invN : 0.0f;
        point.tangentMass = invT > 0.0f ? 1.0f / invT : 0.0f;

        // A bounce gives back the speed the two met with: the speed they
        // arrived at, before this step's gravity and forces were added. With
        // those in it, every bounce returned a step of gravity more than it
        // took, and a ball that keeps all its speed rose higher each time.
        const Vec2 va = Add(a.arriving, Perp(a.arrivingSpin, point.toA));
        const Vec2 vb = Add(b.arriving, Perp(b.arrivingSpin, point.toB));
        const float along = Dot(Sub(vb, va), contact.normal);
        point.bounce = -along > kBounceThreshold ? -bounciness * along : 0.0f;
    }

    // Two points are solved together (SolvePair) when they are far enough
    // apart to tell from one: how each one's push moves the other.
    contact.paired = false;
    if (contact.pointCount == 2)
    {
        const float rn1A = Cross(contact.points[0].toA, contact.normal);
        const float rn1B = Cross(contact.points[0].toB, contact.normal);
        const float rn2A = Cross(contact.points[1].toA, contact.normal);
        const float rn2B = Cross(contact.points[1].toB, contact.normal);
        const float k11 = a.invMass + b.invMass + a.invInertia * rn1A * rn1A + b.invInertia * rn1B * rn1B;
        const float k22 = a.invMass + b.invMass + a.invInertia * rn2A * rn2A + b.invInertia * rn2B * rn2B;
        const float k12 = a.invMass + b.invMass + a.invInertia * rn1A * rn2A + b.invInertia * rn1B * rn2B;
        const float det = k11 * k22 - k12 * k12;
        if (k11 * k11 < kMaxPairCondition * det)
        {
            contact.paired = true;
            contact.k11 = k11;
            contact.k12 = k12;
            contact.k22 = k22;
            contact.invDet = 1.0f / det;
        }
    }
}

const World::Manifold* World::FindLastManifold(uint64_t key) const
{
    const auto last = std::lower_bound(m_LastManifolds.begin(), m_LastManifolds.end(), key,
                                       [](const Manifold& m, uint64_t k) { return PairKey(m.shapeA, m.shapeB) < k; });
    return last != m_LastManifolds.end() && PairKey(last->shapeA, last->shapeB) == key ? &*last : nullptr;
}

void World::WarmStart(Manifold& contact)
{
    const Manifold* last = FindLastManifold(PairKey(contact.shapeA, contact.shapeB));
    if (!last || last->pointCount != contact.pointCount || Dot(last->normal, contact.normal) < 0.95f)
    {
        return;
    }

    Body& a = m_Bodies[contact.bodyA];
    Body& b = m_Bodies[contact.bodyB];
    const Vec2 tangent = { -contact.normal.y, contact.normal.x };
    for (int i = 0; i < contact.pointCount; ++i)
    {
        ContactPoint& point = contact.points[i];
        point.normalImpulse = last->points[i].normalImpulse;
        point.tangentImpulse = last->points[i].tangentImpulse;
        const Vec2 impulse = Add(Scale(contact.normal, point.normalImpulse), Scale(tangent, point.tangentImpulse));
        a.velocity = Sub(a.velocity, Scale(impulse, a.invMass));
        a.angularVelocity -= a.invInertia * Cross(point.toA, impulse);
        b.velocity = Add(b.velocity, Scale(impulse, b.invMass));
        b.angularVelocity += b.invInertia * Cross(point.toB, impulse);
    }
}

void World::SolveVelocity(Manifold& contact, bool reverse)
{
    Body& a = m_Bodies[contact.bodyA];
    Body& b = m_Bodies[contact.bodyB];
    const Vec2 normal = contact.normal;
    const Vec2 tangent = { -normal.y, normal.x };
    const float mu = contact.friction;

    auto apply = [&](const ContactPoint& point, Vec2 impulse)
    {
        a.velocity = Sub(a.velocity, Scale(impulse, a.invMass));
        a.angularVelocity -= a.invInertia * Cross(point.toA, impulse);
        b.velocity = Add(b.velocity, Scale(impulse, b.invMass));
        b.angularVelocity += b.invInertia * Cross(point.toB, impulse);
    };
    auto relative = [&](const ContactPoint& point)
    {
        return Sub(Add(b.velocity, Perp(b.angularVelocity, point.toB)),
                   Add(a.velocity, Perp(a.angularVelocity, point.toA)));
    };

    if (contact.paired)
    {
        // Friction at each point for the push it carries, then both pushes at
        // once.
        for (int k = 0; k < 2; ++k)
        {
            ContactPoint& point = contact.points[reverse ? 1 - k : k];
            const float slide = Dot(relative(point), tangent);
            const float limit = mu * point.normalImpulse;
            const float friction = Clamp(point.tangentImpulse - point.tangentMass * slide, -limit, limit);
            const float frictionChange = friction - point.tangentImpulse;
            point.tangentImpulse = friction;
            apply(point, Scale(tangent, frictionChange));
        }
        SolvePair(contact, a, b);
        return;
    }

    // Solved in turns from either end: always starting at the same corner
    // leaves a little spin behind each step, and a stack walks off sideways.
    for (int k = 0; k < contact.pointCount; ++k)
    {
        ContactPoint& point = contact.points[reverse ? contact.pointCount - 1 - k : k];

        // The running total is what is limited, so a later pass can take back
        // what an earlier one overdid.
        const float along = Dot(relative(point), normal);
        const float total = std::max(point.normalImpulse + point.normalMass * (point.bounce - along), 0.0f);
        const float change = total - point.normalImpulse;
        point.normalImpulse = total;
        apply(point, Scale(normal, change));

        const float slide = Dot(relative(point), tangent);
        const float limit = mu * point.normalImpulse;
        const float friction = Clamp(point.tangentImpulse - point.tangentMass * slide, -limit, limit);
        const float frictionChange = friction - point.tangentImpulse;
        point.tangentImpulse = friction;
        apply(point, Scale(tangent, frictionChange));
    }
}

// The pushes of a contact's two points, found together. Each push must be
// zero or more and leave its point not closing; solved one after the other,
// each undoes part of what the other did, and a box set down flat rocks from
// corner to corner: a tall stack then leans and falls. Together there are
// four cases (both push, either alone, neither), tried in that order; the
// first that fits is the answer.
void World::SolvePair(Manifold& contact, Body& a, Body& b)
{
    ContactPoint& p1 = contact.points[0];
    ContactPoint& p2 = contact.points[1];
    const Vec2 normal = contact.normal;
    auto closing = [&](const ContactPoint& point)
    {
        const Vec2 v = Sub(Add(b.velocity, Perp(b.angularVelocity, point.toB)),
                           Add(a.velocity, Perp(a.angularVelocity, point.toA)));
        return Dot(v, normal) - point.bounce;
    };
    // What the two would close at with no push at all.
    const float had1 = p1.normalImpulse, had2 = p2.normalImpulse;
    const float b1 = closing(p1) - (contact.k11 * had1 + contact.k12 * had2);
    const float b2 = closing(p2) - (contact.k12 * had1 + contact.k22 * had2);

    float x1 = 0.0f, x2 = 0.0f;
    // Both push.
    const float both1 = -(contact.k22 * b1 - contact.k12 * b2) * contact.invDet;
    const float both2 = -(contact.k11 * b2 - contact.k12 * b1) * contact.invDet;
    if (both1 >= 0.0f && both2 >= 0.0f)
    {
        x1 = both1;
        x2 = both2;
    }
    // Only the first pushes, and the second is not closing.
    else if (-b1 * p1.normalMass >= 0.0f && contact.k12 * (-b1 * p1.normalMass) + b2 >= 0.0f)
    {
        x1 = -b1 * p1.normalMass;
    }
    // Only the second.
    else if (-b2 * p2.normalMass >= 0.0f && contact.k12 * (-b2 * p2.normalMass) + b1 >= 0.0f)
    {
        x2 = -b2 * p2.normalMass;
    }
    // Neither, when both are parting by themselves. (When no case fits, which
    // rounding allows, neither pushes this pass either.)

    const Vec2 push1 = Scale(normal, x1 - had1);
    const Vec2 push2 = Scale(normal, x2 - had2);
    a.velocity = Sub(a.velocity, Scale(Add(push1, push2), a.invMass));
    a.angularVelocity -= a.invInertia * (Cross(p1.toA, push1) + Cross(p2.toA, push2));
    b.velocity = Add(b.velocity, Scale(Add(push1, push2), b.invMass));
    b.angularVelocity += b.invInertia * (Cross(p1.toB, push1) + Cross(p2.toB, push2));
    p1.normalImpulse = x1;
    p2.normalImpulse = x2;
}

void World::CorrectPositions(float dt)
{
    for (const Manifold& contact : m_Manifolds)
    {
        if (contact.resting)
        {
            continue;
        }
        Body& a = m_Bodies[contact.bodyA];
        Body& b = m_Bodies[contact.bodyB];
        // A point's share of the push: a contact has one point or two, and
        // halving is what dividing by two gives.
        const float share = contact.pointCount == 2 ? 0.5f : 1.0f;
        for (int i = 0; i < contact.pointCount; ++i)
        {
            const ContactPoint& point = contact.points[i];
            // What is left of the overlap once the step has moved the two:
            // the overlap was measured before they moved, and a pair flying
            // apart has already taken itself out of some of it. Pushing that
            // apart as well lifted a bouncing ball a little higher each time.
            const Vec2 apart = Sub(Add(b.velocity, Perp(b.angularVelocity, point.toB)),
                                   Add(a.velocity, Perp(a.angularVelocity, point.toA)));
            const float separated = std::max(Dot(apart, contact.normal), 0.0f) * dt;
            const float depth = point.penetration - kSlop - separated;
            if (depth <= 0.0f || point.normalMass <= 0.0f)
            {
                continue;
            }
            const float move = depth * kCorrection * point.normalMass * share;
            const Vec2 push = Scale(contact.normal, move);
            a.position = Sub(a.position, Scale(push, a.invMass));
            a.angle -= a.invInertia * Cross(point.toA, push);
            b.position = Add(b.position, Scale(push, b.invMass));
            b.angle += b.invInertia * Cross(point.toB, push);
        }
    }
}

void World::PrepareJoint(Joint& joint, float dt)
{
    Body& a = m_Bodies[joint.bodyA];
    Body& b = BodyAt(joint.bodyB);
    joint.toA = Turned(a, Sub(joint.localA, a.centre));
    joint.toB = Turned(b, Sub(joint.localB, b.centre));
    const float mA = a.invMass, mB = b.invMass, iA = a.invInertia, iB = b.invInertia;
    const Vec2 error = Sub(Add(b.position, joint.toB), Add(a.position, joint.toA));

    auto applyLinear = [&](Vec2 impulse)
    {
        a.velocity = Sub(a.velocity, Scale(impulse, mA));
        a.angularVelocity -= iA * Cross(joint.toA, impulse);
        b.velocity = Add(b.velocity, Scale(impulse, mB));
        b.angularVelocity += iB * Cross(joint.toB, impulse);
    };

    if (!joint.hinge)
    {
        const float length = std::sqrt(Dot(error, error));
        joint.slack = length < 1e-6f || (joint.rope && length <= joint.length);
        if (joint.slack)
        {
            joint.axisImpulse = 0.0f;
            return;
        }
        joint.axis = Scale(error, 1.0f / length);
        const float crA = Cross(joint.toA, joint.axis);
        const float crB = Cross(joint.toB, joint.axis);
        const float inverse = mA + mB + iA * crA * crA + iB * crB * crB;
        joint.axisMass = inverse > 0.0f ? 1.0f / inverse : 0.0f;
        joint.axisBias = Clamp(length - joint.length, -kMaxJointError, kMaxJointError) * kJointCorrection / dt;
        applyLinear(Scale(joint.axis, joint.axisImpulse));
        return;
    }

    joint.k11 = mA + mB + iA * joint.toA.y * joint.toA.y + iB * joint.toB.y * joint.toB.y;
    joint.k12 = -iA * joint.toA.x * joint.toA.y - iB * joint.toB.x * joint.toB.y;
    joint.k22 = mA + mB + iA * joint.toA.x * joint.toA.x + iB * joint.toB.x * joint.toB.x;
    const float distance = std::sqrt(Dot(error, error));
    const float keep = distance > kMaxJointError ? kMaxJointError / distance : 1.0f;
    joint.pointBias = Scale(error, keep * kJointCorrection / dt);
    joint.axialMass = iA + iB > 0.0f ? 1.0f / (iA + iB) : 0.0f;

    int state = 0;
    joint.limitBias = 0.0f;
    if (joint.limited)
    {
        const float angle = b.angle - a.angle - joint.referenceAngle;
        if (angle <= joint.lowerAngle)
        {
            state = 1;
            joint.limitBias = (angle - joint.lowerAngle) * kJointCorrection / dt;
        }
        else if (angle >= joint.upperAngle)
        {
            state = 2;
            joint.limitBias = (angle - joint.upperAngle) * kJointCorrection / dt;
        }
    }
    if (state != joint.limitState)
    {
        joint.limitImpulse = 0.0f;
        joint.limitState = state;
    }

    applyLinear(joint.pointImpulse);
    const float axial = joint.limitImpulse + joint.motorImpulse;
    a.angularVelocity -= iA * axial;
    b.angularVelocity += iB * axial;
}

void World::SolveJoint(Joint& joint)
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
        const float limit = joint.maxMotorTorque * fixedStep;
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

    const float det = joint.k11 * joint.k22 - joint.k12 * joint.k12;
    if (det == 0.0f)
    {
        return;
    }
    const Vec2 speed = Add(pointSpeed(), joint.pointBias);
    const Vec2 impulse{ -(joint.k22 * speed.x - joint.k12 * speed.y) / det,
                        -(joint.k11 * speed.y - joint.k12 * speed.x) / det };
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
    m_LastManifolds.swap(m_Manifolds);
    std::sort(m_LastManifolds.begin(), m_LastManifolds.end(), [](const Manifold& x, const Manifold& y)
              { return PairKey(x.shapeA, x.shapeB) < PairKey(y.shapeA, y.shapeB); });

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
                if (sb.bounds.minX >= sa.bounds.maxX)
                {
                    break;  // sorted: nothing later can overlap on X
                }
                if (sb.bounds.minY >= sa.bounds.maxY || sa.bounds.minY >= sb.bounds.maxY || DoNotCollide(sa, sb))
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

                Manifold manifold{};
                if (!Collide(a, b, manifold))
                {
                    continue;
                }
                m_Touching.push_back(key);
                if (solid)
                {
                    m_Manifolds.push_back(manifold);
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
    const float reach = body.thinnest / dt;
    return Dot(body.velocity, body.velocity) > reach * reach;
}

// Moves a fast body along its path in steps too short to pass over anything,
// and stops it just inside the first thing it meets, so the next step's
// contact takes it from there.
void World::SweepBody(uint32_t bodyIndex, float dt)
{
    Body& body = m_Bodies[bodyIndex];
    const Vec2 start = body.position;
    const float startAngle = body.angle;
    const Vec2 motion = Scale(body.velocity, dt);
    const float turn = body.angularVelocity * dt;

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
                if (Collide(own, other, manifold))
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

void World::IntegratePositions(float dt)
{
    for (Body& body : m_Bodies)
    {
        if (body.alive && body.type != BodyType::Static && (body.type == BodyType::Kinematic || body.awake) &&
            !IsFast(body, dt))
        {
            body.position.x += body.velocity.x * dt;
            body.position.y += body.velocity.y * dt;
            body.angle += body.angularVelocity * dt;
        }
    }
    // After the rest, so each fast body meets everything where it ended up:
    // the boxes it is tested against are brought to where the others are
    // now, once, before the first of them. (One fast body is still tested
    // against where another began the step.)
    bool refreshed = false;
    for (uint32_t i = 0; i < m_Bodies.size(); ++i)
    {
        if (m_Bodies[i].alive && IsFast(m_Bodies[i], dt))
        {
            if (!refreshed)
            {
                RefreshSweep();
                refreshed = true;
            }
            SweepBody(i, dt);
        }
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
        if (!body.alive || body.type != BodyType::Dynamic || !body.awake)
        {
            continue;
        }
        body.velocity.x += (gravity.x * body.gravityScale + body.force.x * body.invMass) * dt;
        body.velocity.y += (gravity.y * body.gravityScale + body.force.y * body.invMass) * dt;
        body.angularVelocity += body.torque * body.invInertia * dt;
        const float keep = std::max(0.0f, 1.0f - body.linearDamping * dt);
        body.velocity.x *= keep;
        body.velocity.y *= keep;
        body.angularVelocity *= std::max(0.0f, 1.0f - body.angularDamping * dt);
        body.force = {};
        body.torque = 0.0f;
    }

    FindContacts();

    auto jointIsAwake = [this](const Joint& joint)
    { return joint.alive && (m_Bodies[joint.bodyA].awake || (joint.bodyB != kNoBody && m_Bodies[joint.bodyB].awake)); };
    // Every contact is prepared before any of last step's pushes go back on: a
    // bounce is decided by how fast a contact is closing, and those pushes would
    // look like an impact. A joint puts its own back as it is prepared.
    for (Manifold& contact : m_Manifolds)
    {
        if (!contact.resting)
        {
            PrepareContact(contact);
        }
    }
    for (Joint& joint : m_Joints)
    {
        if (jointIsAwake(joint))
        {
            PrepareJoint(joint, dt);
        }
    }
    for (Manifold& contact : m_Manifolds)
    {
        if (!contact.resting)
        {
            WarmStart(contact);
        }
    }

    for (int i = 0; i < velocityIterations; ++i)
    {
        for (Joint& joint : m_Joints)
        {
            if (jointIsAwake(joint))
            {
                SolveJoint(joint);
            }
        }
        for (Manifold& contact : m_Manifolds)
        {
            if (!contact.resting)
            {
                SolveVelocity(contact, i % 2 == 1);
            }
        }
    }

    IntegratePositions(dt);
    CorrectPositions(dt);

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

    m_ReportNow = m_Touching;
    m_ReportBefore = m_WasTouching;
    m_WasTouching = m_Touching;
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
