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

}  // namespace

BodyHandle World::CreateBody(const BodyDef& def)
{
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
    body.position = def.position;
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
    body.freezeRotation = def.freezeRotation;
    body.type = def.type;
    body.user = def.user;
    body.alive = true;
    ++m_LiveBodies;
    return { index, body.generation };
}

void World::DestroyBody(BodyHandle handle)
{
    Body* body = Find(handle);
    if (!body)
    {
        return;
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

ShapeHandle World::AddShape(BodyHandle body, const ShapeDef& def)
{
    if (!Find(body))
    {
        return {};
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
    shape.def = def;
    shape.body = body.index;
    shape.alive = true;
    m_Sweep.push_back(index);
    ++m_LiveShapes;
    RefreshMassData(body.index);
    return { index, shape.generation };
}

void World::RemoveShape(ShapeHandle handle)
{
    if (handle.index >= m_Shapes.size())
    {
        return;
    }
    Shape& shape = m_Shapes[handle.index];
    if (!shape.alive || shape.generation != handle.generation)
    {
        return;
    }

    // Anything touching it stops touching, and is told so now: nothing else
    // would ever report the pair again.
    if (onContactEnd)
    {
        for (uint64_t key : m_Touching)
        {
            const uint32_t a = static_cast<uint32_t>(key >> 32);
            const uint32_t b = static_cast<uint32_t>(key);
            if (a == handle.index || b == handle.index)
            {
                onContactEnd(MakeContact(a, b));
            }
        }
    }
    auto involves = [&](uint64_t key)
    { return static_cast<uint32_t>(key >> 32) == handle.index || static_cast<uint32_t>(key) == handle.index; };
    m_Touching.erase(std::remove_if(m_Touching.begin(), m_Touching.end(), involves), m_Touching.end());

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

// How hard the body is to turn, from its shapes. The mass is shared between
// them by area, and a shape off the centre counts for more.
void World::RefreshMassData(uint32_t bodyIndex)
{
    Body& body = m_Bodies[bodyIndex];
    body.invInertia = 0.0f;
    if (body.type != BodyType::Dynamic || body.freezeRotation || body.invMass <= 0.0f)
    {
        return;
    }

    constexpr float kPi = 3.14159265f;
    float totalArea = 0.0f;
    float inertiaPerArea = 0.0f;
    for (const Shape& shape : m_Shapes)
    {
        if (!shape.alive || shape.body != bodyIndex || shape.def.trigger)
        {
            continue;
        }
        const float distSq = Dot(shape.def.offset, shape.def.offset);
        if (shape.def.isCircle)
        {
            const float rr = shape.def.radius * shape.def.radius;
            const float area = kPi * rr;
            totalArea += area;
            inertiaPerArea += area * (0.5f * rr + distSq);
        }
        else
        {
            const float w = shape.def.halfWidth * 2.0f;
            const float h = shape.def.halfHeight * 2.0f;
            const float area = w * h;
            totalArea += area;
            inertiaPerArea += area * ((w * w + h * h) / 12.0f + distSq);
        }
    }
    if (totalArea > 0.0f && inertiaPerArea > 0.0f)
    {
        body.invInertia = totalArea / (body.mass * inertiaPerArea);
    }
}

Vec2 World::GetPosition(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->position : Vec2{};
}

void World::SetPosition(BodyHandle handle, Vec2 position)
{
    if (Body* body = Find(handle))
    {
        body->position = position;
    }
}

float World::GetAngle(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->angle : 0.0f;
}

void World::SetAngle(BodyHandle handle, float radians)
{
    if (Body* body = Find(handle))
    {
        body->angle = radians;
        body->cosAngle = std::cos(radians);
        body->sinAngle = std::sin(radians);
    }
}

float World::GetAngularVelocity(BodyHandle handle) const
{
    const Body* body = Find(handle);
    return body ? body->angularVelocity : 0.0f;
}

void World::SetAngularVelocity(BodyHandle handle, float radiansPerSecond)
{
    Body* body = Find(handle);
    if (body && body->type != BodyType::Static && !body->freezeRotation)
    {
        body->angularVelocity = radiansPerSecond;
    }
}

void World::AddTorque(BodyHandle handle, float torque)
{
    if (Body* body = Find(handle))
    {
        body->torque += torque;
    }
}

void World::AddImpulseAt(BodyHandle handle, Vec2 impulse, Vec2 worldPoint)
{
    if (Body* body = Find(handle))
    {
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
    Body* body = Find(handle);
    if (body && body->type != BodyType::Static)
    {
        body->velocity = velocity;
    }
}

void World::AddForce(BodyHandle handle, Vec2 force)
{
    if (Body* body = Find(handle))
    {
        body->force.x += force.x;
        body->force.y += force.y;
    }
}

void World::AddImpulse(BodyHandle handle, Vec2 impulse)
{
    if (Body* body = Find(handle))
    {
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
    return shape.alive && shape.generation == handle.generation ? shape.def.user : nullptr;
}

Vec2 World::CentreOf(const Shape& shape) const
{
    const Body& body = m_Bodies[shape.body];
    const Vec2 o = shape.def.offset;
    return { body.position.x + o.x * body.cosAngle - o.y * body.sinAngle,
             body.position.y + o.x * body.sinAngle + o.y * body.cosAngle };
}

World::Aabb World::BoundsOf(const Shape& shape) const
{
    const Vec2 c = CentreOf(shape);
    if (shape.def.isCircle)
    {
        const float r = shape.def.radius;
        return { c.x - r, c.y - r, c.x + r, c.y + r };
    }
    const Body& body = m_Bodies[shape.body];
    const float ac = std::fabs(body.cosAngle);
    const float as = std::fabs(body.sinAngle);
    const float hw = shape.def.halfWidth * ac + shape.def.halfHeight * as;
    const float hh = shape.def.halfWidth * as + shape.def.halfHeight * ac;
    return { c.x - hw, c.y - hh, c.x + hw, c.y + hh };
}

bool World::CollideCircles(const Shape& a, const Shape& b, Manifold& out) const
{
    const Vec2 ca = CentreOf(a);
    const Vec2 cb = CentreOf(b);
    const Vec2 d = Sub(cb, ca);
    const float radii = a.def.radius + b.def.radius;
    const float distSq = Dot(d, d);
    if (distSq >= radii * radii)
    {
        return false;
    }
    const float dist = std::sqrt(distSq);
    out.normal = dist > 1e-6f ? Scale(d, 1.0f / dist) : Vec2{ 0.0f, 1.0f };
    out.pointCount = 1;
    out.points[0].position = Add(ca, Scale(out.normal, a.def.radius));
    out.points[0].penetration = radii - dist;
    return true;
}

bool World::CollideCircleBox(const Shape& circle, const Shape& box, bool circleIsA, Manifold& out) const
{
    const Body& boxBody = m_Bodies[box.body];
    const Vec2 boxCentre = CentreOf(box);
    const Vec2 circleCentre = CentreOf(circle);
    const float c = boxBody.cosAngle;
    const float s = boxBody.sinAngle;

    // The circle's centre in the box's own axes, where the box is axis-aligned.
    const Vec2 rel = Sub(circleCentre, boxCentre);
    const Vec2 local{ rel.x * c + rel.y * s, -rel.x * s + rel.y * c };
    const float hw = box.def.halfWidth;
    const float hh = box.def.halfHeight;
    const float radius = circle.def.radius;

    const Vec2 nearest{ Clamp(local.x, -hw, hw), Clamp(local.y, -hh, hh) };
    Vec2 d = Sub(local, nearest);
    const float distSq = Dot(d, d);
    Vec2 boxToCircle;  // in the box's axes
    float penetration;
    Vec2 surface = nearest;
    if (distSq > 1e-12f)
    {
        if (distSq >= radius * radius)
        {
            return false;
        }
        const float dist = std::sqrt(distSq);
        boxToCircle = Scale(d, 1.0f / dist);
        penetration = radius - dist;
    }
    else
    {
        // The centre is inside the box: out through the nearest side.
        const float toRight = hw - local.x;
        const float toLeft = local.x + hw;
        const float toTop = hh - local.y;
        const float toBottom = local.y + hh;
        const float least = std::min({ toRight, toLeft, toTop, toBottom });
        if (least == toRight)
        {
            boxToCircle = { 1.0f, 0.0f };
            surface.x = hw;
        }
        else if (least == toLeft)
        {
            boxToCircle = { -1.0f, 0.0f };
            surface.x = -hw;
        }
        else if (least == toTop)
        {
            boxToCircle = { 0.0f, 1.0f };
            surface.y = hh;
        }
        else
        {
            boxToCircle = { 0.0f, -1.0f };
            surface.y = -hh;
        }
        penetration = radius + least;
    }

    const Vec2 worldNormal{ boxToCircle.x * c - boxToCircle.y * s, boxToCircle.x * s + boxToCircle.y * c };
    out.normal = circleIsA ? Scale(worldNormal, -1.0f) : worldNormal;
    out.pointCount = 1;
    out.points[0].position = Add(boxCentre, Vec2{ surface.x * c - surface.y * s, surface.x * s + surface.y * c });
    out.points[0].penetration = penetration;
    return true;
}

// Separating axes, then the touching edge of one box clipped against the
// other. Two points, so a box resting on a face stays flat instead of rocking
// on a single corner.
bool World::CollideBoxes(const Shape& a, const Shape& b, Manifold& out) const
{
    struct Box
    {
        Vec2 centre, axisX, axisY;
        float hw, hh;
    };
    auto make = [this](const Shape& shape)
    {
        const Body& body = m_Bodies[shape.body];
        return Box{ CentreOf(shape),
                    { body.cosAngle, body.sinAngle },
                    { -body.sinAngle, body.cosAngle },
                    shape.def.halfWidth,
                    shape.def.halfHeight };
    };
    const Box A = make(a);
    const Box B = make(b);
    const Vec2 d = Sub(B.centre, A.centre);

    auto reach = [](const Box& box, Vec2 axis)
    { return std::fabs(Dot(box.axisX, axis)) * box.hw + std::fabs(Dot(box.axisY, axis)) * box.hh; };

    float best = 1e30f;
    Vec2 normal{};
    bool referenceIsA = true;
    int referenceAxis = 0;  // 0: its X, 1: its Y
    const Vec2 axes[4] = { A.axisX, A.axisY, B.axisX, B.axisY };
    for (int i = 0; i < 4; ++i)
    {
        const float distance = Dot(d, axes[i]);
        const float overlap = reach(A, axes[i]) + reach(B, axes[i]) - std::fabs(distance);
        if (overlap <= 0.0f)
        {
            return false;
        }
        // A small preference for the first box's faces keeps the choice from
        // flickering between two equal axes from one step to the next.
        const float score = i < 2 ? overlap : overlap + 1e-4f;
        if (score < best)
        {
            best = score;
            normal = distance < 0.0f ? Scale(axes[i], -1.0f) : axes[i];
            referenceIsA = i < 2;
            referenceAxis = i % 2;
        }
    }

    const Box& ref = referenceIsA ? A : B;
    const Box& inc = referenceIsA ? B : A;
    const Vec2 outward = referenceIsA ? normal : Scale(normal, -1.0f);  // out of the reference face

    // The other box's edge that faces the reference face most directly.
    const float alongX = Dot(inc.axisX, outward);
    const float alongY = Dot(inc.axisY, outward);
    Vec2 edgeCentre, edgeDir;
    float edgeHalf;
    if (std::fabs(alongX) > std::fabs(alongY))
    {
        edgeCentre = Add(inc.centre, Scale(inc.axisX, alongX > 0.0f ? -inc.hw : inc.hw));
        edgeDir = inc.axisY;
        edgeHalf = inc.hh;
    }
    else
    {
        edgeCentre = Add(inc.centre, Scale(inc.axisY, alongY > 0.0f ? -inc.hh : inc.hh));
        edgeDir = inc.axisX;
        edgeHalf = inc.hw;
    }
    Vec2 p0 = Sub(edgeCentre, Scale(edgeDir, edgeHalf));
    Vec2 p1 = Add(edgeCentre, Scale(edgeDir, edgeHalf));

    // Clip that edge to the reference face's width.
    const Vec2 side = referenceAxis == 0 ? ref.axisY : ref.axisX;
    const float sideHalf = referenceAxis == 0 ? ref.hh : ref.hw;
    const float faceHalf = referenceAxis == 0 ? ref.hw : ref.hh;
    const float s0 = Dot(Sub(p0, ref.centre), side);
    const float s1 = Dot(Sub(p1, ref.centre), side);
    auto clip = [&](float limit, float sign)
    {
        const float v0 = sign * Dot(Sub(p0, ref.centre), side) - limit;
        const float v1 = sign * Dot(Sub(p1, ref.centre), side) - limit;
        if (v0 > 0.0f && v1 < 0.0f)
        {
            p0 = Add(p0, Scale(Sub(p1, p0), v0 / (v0 - v1)));
        }
        else if (v1 > 0.0f && v0 < 0.0f)
        {
            p1 = Add(p1, Scale(Sub(p0, p1), v1 / (v1 - v0)));
        }
    };
    (void)s0;
    (void)s1;
    clip(sideHalf, 1.0f);
    clip(sideHalf, -1.0f);

    out.normal = normal;
    out.pointCount = 0;
    for (const Vec2& p : { p0, p1 })
    {
        const float depth = faceHalf - Dot(Sub(p, ref.centre), outward);
        if (depth > 0.0f && out.pointCount < 2)
        {
            out.points[out.pointCount].position = p;
            out.points[out.pointCount].penetration = depth;
            ++out.pointCount;
        }
    }
    return out.pointCount > 0;
}

bool World::Collide(uint32_t a, uint32_t b, Manifold& out) const
{
    const Shape& sa = m_Shapes[a];
    const Shape& sb = m_Shapes[b];
    out.shapeA = a;
    out.shapeB = b;
    if (sa.def.isCircle && sb.def.isCircle)
    {
        return CollideCircles(sa, sb, out);
    }
    if (!sa.def.isCircle && !sb.def.isCircle)
    {
        return CollideBoxes(sa, sb, out);
    }
    return sa.def.isCircle ? CollideCircleBox(sa, sb, true, out) : CollideCircleBox(sb, sa, false, out);
}

void World::PrepareContact(Manifold& contact)
{
    const Shape& sa = m_Shapes[contact.shapeA];
    const Shape& sb = m_Shapes[contact.shapeB];
    const Body& a = m_Bodies[sa.body];
    const Body& b = m_Bodies[sb.body];
    const Vec2 tangent = { -contact.normal.y, contact.normal.x };
    const float bounciness = std::max(sa.def.bounciness, sb.def.bounciness);

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

        const Vec2 va = Add(a.velocity, Perp(a.angularVelocity, point.toA));
        const Vec2 vb = Add(b.velocity, Perp(b.angularVelocity, point.toB));
        const float along = Dot(Sub(vb, va), contact.normal);
        point.bounce = -along > kBounceThreshold ? -bounciness * along : 0.0f;
    }
}

void World::WarmStart(Manifold& contact)
{
    const uint64_t key = PairKey(contact.shapeA, contact.shapeB);
    const auto last = std::lower_bound(m_LastManifolds.begin(), m_LastManifolds.end(), key,
                                       [](const Manifold& m, uint64_t k) { return PairKey(m.shapeA, m.shapeB) < k; });
    if (last == m_LastManifolds.end() || PairKey(last->shapeA, last->shapeB) != key ||
        last->pointCount != contact.pointCount || Dot(last->normal, contact.normal) < 0.95f)
    {
        return;
    }

    Body& a = m_Bodies[m_Shapes[contact.shapeA].body];
    Body& b = m_Bodies[m_Shapes[contact.shapeB].body];
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
    const Shape& sa = m_Shapes[contact.shapeA];
    const Shape& sb = m_Shapes[contact.shapeB];
    Body& a = m_Bodies[sa.body];
    Body& b = m_Bodies[sb.body];
    const Vec2 normal = contact.normal;
    const Vec2 tangent = { -normal.y, normal.x };
    const float mu = std::sqrt(sa.def.friction * sb.def.friction);

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

void World::CorrectPositions()
{
    for (const Manifold& contact : m_Manifolds)
    {
        Body& a = m_Bodies[m_Shapes[contact.shapeA].body];
        Body& b = m_Bodies[m_Shapes[contact.shapeB].body];
        for (int i = 0; i < contact.pointCount; ++i)
        {
            const ContactPoint& point = contact.points[i];
            const float depth = point.penetration - kSlop;
            if (depth <= 0.0f || point.normalMass <= 0.0f)
            {
                continue;
            }
            const float move = depth * kCorrection * point.normalMass / static_cast<float>(contact.pointCount);
            const Vec2 push = Scale(contact.normal, move);
            a.position = Sub(a.position, Scale(push, a.invMass));
            a.angle -= a.invInertia * Cross(point.toA, push);
            b.position = Add(b.position, Scale(push, b.invMass));
            b.angle += b.invInertia * Cross(point.toB, push);
        }
    }
}

void World::FixedStep()
{
    const float dt = fixedStep;

    for (Body& body : m_Bodies)
    {
        if (!body.alive || body.type != BodyType::Dynamic)
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

    // Sort and sweep along X. Insertion sort, because the order barely changes
    // between steps and it is then close to linear.
    std::vector<float> minX(m_Shapes.size());
    for (uint32_t index : m_Sweep)
    {
        minX[index] = BoundsOf(m_Shapes[index]).minX;
    }
    for (size_t i = 1; i < m_Sweep.size(); ++i)
    {
        const uint32_t value = m_Sweep[i];
        size_t j = i;
        while (j > 0 && minX[m_Sweep[j - 1]] > minX[value])
        {
            m_Sweep[j] = m_Sweep[j - 1];
            --j;
        }
        m_Sweep[j] = value;
    }

    // Kept, sorted by pair, so this step's contacts can start from last step's
    // pushes: a stack is then already held up before the solver begins.
    m_LastManifolds.swap(m_Manifolds);
    std::sort(m_LastManifolds.begin(), m_LastManifolds.end(), [](const Manifold& x, const Manifold& y)
              { return PairKey(x.shapeA, x.shapeB) < PairKey(y.shapeA, y.shapeB); });
    m_Manifolds.clear();
    m_Touching.clear();
    for (size_t i = 0; i < m_Sweep.size(); ++i)
    {
        const uint32_t a = m_Sweep[i];
        const Shape& sa = m_Shapes[a];
        const Aabb ba = BoundsOf(sa);
        for (size_t k = i + 1; k < m_Sweep.size(); ++k)
        {
            const uint32_t b = m_Sweep[k];
            if (minX[b] >= ba.maxX)
            {
                break;  // sorted: nothing later can overlap on X
            }
            const Shape& sb = m_Shapes[b];
            if (sa.body == sb.body || !(sa.def.layer & sb.def.mask) || !(sb.def.layer & sa.def.mask))
            {
                continue;
            }
            const bool bothFixed = m_Bodies[sa.body].type != BodyType::Dynamic && m_Bodies[sb.body].type != BodyType::Dynamic;
            if (bothFixed && !sa.def.trigger && !sb.def.trigger)
            {
                continue;
            }
            const Aabb bb = BoundsOf(sb);
            if (bb.minY >= ba.maxY || ba.minY >= bb.maxY)
            {
                continue;
            }
            Manifold manifold{};
            if (!Collide(a, b, manifold))
            {
                continue;
            }
            m_Touching.push_back(PairKey(a, b));
            if (!sa.def.trigger && !sb.def.trigger)
            {
                PrepareContact(manifold);
                m_Manifolds.push_back(manifold);
            }
        }
    }

    for (Manifold& contact : m_Manifolds)
    {
        WarmStart(contact);
    }

    for (int i = 0; i < velocityIterations; ++i)
    {
        for (Manifold& contact : m_Manifolds)
        {
            SolveVelocity(contact, i % 2 == 1);
        }
    }

    for (Body& body : m_Bodies)
    {
        if (body.alive && body.type != BodyType::Static)
        {
            body.position.x += body.velocity.x * dt;
            body.position.y += body.velocity.y * dt;
            body.angle += body.angularVelocity * dt;
        }
    }

    CorrectPositions();

    for (Body& body : m_Bodies)
    {
        if (body.alive && body.type != BodyType::Static)
        {
            body.cosAngle = std::cos(body.angle);
            body.sinAngle = std::sin(body.angle);
        }
    }

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
             sa.def.trigger || sb.def.trigger };
}

void World::ReportContacts()
{
    std::sort(m_Touching.begin(), m_Touching.end());
    auto report = [this](uint64_t key, const std::function<void(const Contact&)>& callback)
    {
        const uint32_t a = static_cast<uint32_t>(key >> 32);
        const uint32_t b = static_cast<uint32_t>(key);
        if (m_Shapes[a].alive && m_Shapes[b].alive && callback)
        {
            callback(MakeContact(a, b));
        }
    };

    // Copies: a callback may destroy bodies, which edits m_Touching.
    const std::vector<uint64_t> now = m_Touching;
    const std::vector<uint64_t> before = m_WasTouching;
    m_WasTouching = now;
    size_t i = 0, k = 0;
    while (i < now.size() || k < before.size())
    {
        if (k == before.size() || (i < now.size() && now[i] < before[k]))
        {
            report(now[i++], onContactBegin);
        }
        else if (i == now.size() || before[k] < now[i])
        {
            report(before[k++], onContactEnd);
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
    if (steps == maxStepsPerFrame)
    {
        m_Banked = 0.0f;  // behind: the game slows down rather than spiralling
    }
    return steps;
}

bool World::Raycast(Vec2 from, Vec2 to, RayHit& hit, uint16_t mask) const
{
    const Vec2 d{ to.x - from.x, to.y - from.y };
    bool found = false;
    float best = 1.0f;

    for (uint32_t index : m_Sweep)
    {
        const Shape& shape = m_Shapes[index];
        if (!(shape.def.layer & mask) || shape.def.trigger)
        {
            continue;
        }
        const Vec2 centre = CentreOf(shape);
        const float cx = centre.x;
        const float cy = centre.y;
        float t = -1.0f;
        Vec2 normal;

        if (shape.def.isCircle)
        {
            const Vec2 f{ from.x - cx, from.y - cy };
            const float a = Dot(d, d);
            const float b = 2.0f * Dot(f, d);
            const float c = Dot(f, f) - shape.def.radius * shape.def.radius;
            const float disc = b * b - 4.0f * a * c;
            if (a <= 0.0f || disc < 0.0f)
            {
                continue;
            }
            t = (-b - std::sqrt(disc)) / (2.0f * a);
            if (t < 0.0f || t > best)
            {
                continue;
            }
            normal = { (from.x + d.x * t - cx) / shape.def.radius, (from.y + d.y * t - cy) / shape.def.radius };
        }
        else
        {
            float tMin = 0.0f;
            float tMax = best;
            bool hitAxisX = true;
            // In the box's own axes, where it is axis-aligned.
            const float c = m_Bodies[shape.body].cosAngle;
            const float sn = m_Bodies[shape.body].sinAngle;
            const Vec2 rel{ from.x - cx, from.y - cy };
            const float lo[2] = { -shape.def.halfWidth, -shape.def.halfHeight };
            const float hi[2] = { shape.def.halfWidth, shape.def.halfHeight };
            const float o[2] = { rel.x * c + rel.y * sn, -rel.x * sn + rel.y * c };
            const float v[2] = { d.x * c + d.y * sn, -d.x * sn + d.y * c };
            bool miss = false;
            for (int axis = 0; axis < 2 && !miss; ++axis)
            {
                if (std::fabs(v[axis]) < 1e-9f)
                {
                    miss = o[axis] < lo[axis] || o[axis] > hi[axis];
                    continue;
                }
                float t1 = (lo[axis] - o[axis]) / v[axis];
                float t2 = (hi[axis] - o[axis]) / v[axis];
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                }
                if (t1 > tMin)
                {
                    tMin = t1;
                    hitAxisX = axis == 0;
                }
                tMax = std::min(tMax, t2);
                miss = tMin > tMax;
            }
            if (miss)
            {
                continue;
            }
            t = tMin;
            const Vec2 local = hitAxisX ? Vec2{ v[0] > 0.0f ? -1.0f : 1.0f, 0.0f } : Vec2{ 0.0f, v[1] > 0.0f ? -1.0f : 1.0f };
            normal = { local.x * c - local.y * sn, local.x * sn + local.y * c };
        }

        found = true;
        best = t;
        hit.shape = { index, shape.generation };
        hit.body = { shape.body, m_Bodies[shape.body].generation };
        hit.point = { from.x + d.x * t, from.y + d.y * t };
        hit.normal = normal;
        hit.fraction = t;
    }
    return found;
}

}  // namespace DekiPhysics2D
