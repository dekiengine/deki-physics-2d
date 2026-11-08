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
constexpr float kCorrection = 0.7f;
// Slower than this does not bounce, or a resting body would never settle.
constexpr float kBounceThreshold = 0.5f;

float Dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
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
    body.velocity = def.type == BodyType::Static ? Vec2{} : def.velocity;
    body.force = {};
    body.invMass = (def.type == BodyType::Dynamic && def.mass > 0.0f) ? 1.0f / def.mass : 0.0f;
    body.gravityScale = def.gravityScale;
    body.linearDamping = def.linearDamping;
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
                const Shape& sa = m_Shapes[a];
                const Shape& sb = m_Shapes[b];
                onContactEnd({ { a, sa.generation },
                               { b, sb.generation },
                               { sa.body, m_Bodies[sa.body].generation },
                               { sb.body, m_Bodies[sb.body].generation } });
            }
        }
    }
    auto involves = [&](uint64_t key)
    { return static_cast<uint32_t>(key >> 32) == handle.index || static_cast<uint32_t>(key) == handle.index; };
    m_Touching.erase(std::remove_if(m_Touching.begin(), m_Touching.end(), involves), m_Touching.end());

    shape.alive = false;
    ++shape.generation;
    m_Sweep.erase(std::find(m_Sweep.begin(), m_Sweep.end(), handle.index));
    m_FreeShapes.push_back(handle.index);
    --m_LiveShapes;
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

void World::ApplyForce(BodyHandle handle, Vec2 force)
{
    if (Body* body = Find(handle))
    {
        body->force.x += force.x;
        body->force.y += force.y;
    }
}

void World::ApplyImpulse(BodyHandle handle, Vec2 impulse)
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

World::Aabb World::BoundsOf(const Shape& shape) const
{
    const Vec2 p = m_Bodies[shape.body].position;
    const float cx = p.x + shape.def.offset.x;
    const float cy = p.y + shape.def.offset.y;
    const float hw = shape.def.isCircle ? shape.def.radius : shape.def.halfWidth;
    const float hh = shape.def.isCircle ? shape.def.radius : shape.def.halfHeight;
    return { cx - hw, cy - hh, cx + hw, cy + hh };
}

bool World::Collide(uint32_t a, uint32_t b, Manifold& out) const
{
    const Shape& sa = m_Shapes[a];
    const Shape& sb = m_Shapes[b];
    const Vec2 pa = m_Bodies[sa.body].position;
    const Vec2 pb = m_Bodies[sb.body].position;
    const float ax = pa.x + sa.def.offset.x;
    const float ay = pa.y + sa.def.offset.y;
    const float bx = pb.x + sb.def.offset.x;
    const float by = pb.y + sb.def.offset.y;
    out.shapeA = a;
    out.shapeB = b;

    if (sa.def.isCircle && sb.def.isCircle)
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float radii = sa.def.radius + sb.def.radius;
        const float distSq = dx * dx + dy * dy;
        if (distSq >= radii * radii)
        {
            return false;
        }
        const float dist = std::sqrt(distSq);
        out.normal = dist > 1e-6f ? Vec2{ dx / dist, dy / dist } : Vec2{ 0.0f, -1.0f };
        out.penetration = radii - dist;
        return true;
    }

    if (!sa.def.isCircle && !sb.def.isCircle)
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float overlapX = sa.def.halfWidth + sb.def.halfWidth - std::fabs(dx);
        const float overlapY = sa.def.halfHeight + sb.def.halfHeight - std::fabs(dy);
        if (overlapX <= 0.0f || overlapY <= 0.0f)
        {
            return false;
        }
        if (overlapX < overlapY)
        {
            out.normal = { dx < 0.0f ? -1.0f : 1.0f, 0.0f };
            out.penetration = overlapX;
        }
        else
        {
            out.normal = { 0.0f, dy < 0.0f ? -1.0f : 1.0f };
            out.penetration = overlapY;
        }
        return true;
    }

    // A circle and a box, computed as circle against box and turned around if
    // A was the box.
    const bool aIsCircle = sa.def.isCircle;
    const Shape& box = aIsCircle ? sb : sa;
    const float cx = aIsCircle ? ax : bx;
    const float cy = aIsCircle ? ay : by;
    const float boxX = aIsCircle ? bx : ax;
    const float boxY = aIsCircle ? by : ay;
    const float radius = aIsCircle ? sa.def.radius : sb.def.radius;

    const float nearX = Clamp(cx, boxX - box.def.halfWidth, boxX + box.def.halfWidth);
    const float nearY = Clamp(cy, boxY - box.def.halfHeight, boxY + box.def.halfHeight);
    float dx = nearX - cx;
    float dy = nearY - cy;
    const float distSq = dx * dx + dy * dy;
    Vec2 toBox;  // from the circle's centre towards the box
    float penetration;
    if (distSq > 1e-12f)
    {
        if (distSq >= radius * radius)
        {
            return false;
        }
        const float dist = std::sqrt(distSq);
        toBox = { dx / dist, dy / dist };
        penetration = radius - dist;
    }
    else
    {
        // The centre is inside the box: out through the nearest side.
        const float left = cx - (boxX - box.def.halfWidth);
        const float right = (boxX + box.def.halfWidth) - cx;
        const float top = cy - (boxY - box.def.halfHeight);
        const float bottom = (boxY + box.def.halfHeight) - cy;
        const float least = std::min({ left, right, top, bottom });
        if (least == left)
        {
            toBox = { 1.0f, 0.0f };
        }
        else if (least == right)
        {
            toBox = { -1.0f, 0.0f };
        }
        else if (least == top)
        {
            toBox = { 0.0f, 1.0f };
        }
        else
        {
            toBox = { 0.0f, -1.0f };
        }
        penetration = radius + least;
    }
    out.normal = aIsCircle ? toBox : Vec2{ -toBox.x, -toBox.y };
    out.penetration = penetration;
    return true;
}

void World::Solve(const Manifold& contact)
{
    Shape& sa = m_Shapes[contact.shapeA];
    Shape& sb = m_Shapes[contact.shapeB];
    Body& a = m_Bodies[sa.body];
    Body& b = m_Bodies[sb.body];
    const float invSum = a.invMass + b.invMass;
    if (invSum <= 0.0f)
    {
        return;
    }

    const Vec2 relative{ b.velocity.x - a.velocity.x, b.velocity.y - a.velocity.y };
    const float along = Dot(relative, contact.normal);
    if (along > 0.0f)
    {
        return;  // already separating
    }

    const float restitution = -along > kBounceThreshold ? std::max(sa.def.restitution, sb.def.restitution) : 0.0f;
    const float j = -(1.0f + restitution) * along / invSum;
    a.velocity.x -= j * contact.normal.x * a.invMass;
    a.velocity.y -= j * contact.normal.y * a.invMass;
    b.velocity.x += j * contact.normal.x * b.invMass;
    b.velocity.y += j * contact.normal.y * b.invMass;

    // Friction, along the surface, no more than the push allows.
    const Vec2 after{ b.velocity.x - a.velocity.x, b.velocity.y - a.velocity.y };
    const Vec2 tangent{ -contact.normal.y, contact.normal.x };
    const float slide = Dot(after, tangent);
    const float mu = std::sqrt(sa.def.friction * sb.def.friction);
    const float jt = Clamp(-slide / invSum, -j * mu, j * mu);
    a.velocity.x -= jt * tangent.x * a.invMass;
    a.velocity.y -= jt * tangent.y * a.invMass;
    b.velocity.x += jt * tangent.x * b.invMass;
    b.velocity.y += jt * tangent.y * b.invMass;
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
        const float keep = std::max(0.0f, 1.0f - body.linearDamping * dt);
        body.velocity.x *= keep;
        body.velocity.y *= keep;
        body.force = {};
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
            if (bothFixed && !sa.def.sensor && !sb.def.sensor)
            {
                continue;
            }
            const Aabb bb = BoundsOf(sb);
            if (bb.minY >= ba.maxY || ba.minY >= bb.maxY)
            {
                continue;
            }
            Manifold manifold;
            if (!Collide(a, b, manifold))
            {
                continue;
            }
            m_Touching.push_back(PairKey(a, b));
            if (!sa.def.sensor && !sb.def.sensor)
            {
                m_Manifolds.push_back(manifold);
            }
        }
    }

    for (int i = 0; i < velocityIterations; ++i)
    {
        for (const Manifold& contact : m_Manifolds)
        {
            Solve(contact);
        }
    }

    for (Body& body : m_Bodies)
    {
        if (body.alive && body.type != BodyType::Static)
        {
            body.position.x += body.velocity.x * dt;
            body.position.y += body.velocity.y * dt;
        }
    }

    for (const Manifold& contact : m_Manifolds)
    {
        Body& a = m_Bodies[m_Shapes[contact.shapeA].body];
        Body& b = m_Bodies[m_Shapes[contact.shapeB].body];
        const float invSum = a.invMass + b.invMass;
        const float depth = contact.penetration - kSlop;
        if (invSum <= 0.0f || depth <= 0.0f)
        {
            continue;
        }
        const float move = depth * kCorrection / invSum;
        a.position.x -= contact.normal.x * move * a.invMass;
        a.position.y -= contact.normal.y * move * a.invMass;
        b.position.x += contact.normal.x * move * b.invMass;
        b.position.y += contact.normal.y * move * b.invMass;
    }

    ReportContacts();
}

void World::ReportContacts()
{
    std::sort(m_Touching.begin(), m_Touching.end());
    auto report = [this](uint64_t key, const std::function<void(const Contact&)>& callback)
    {
        const uint32_t a = static_cast<uint32_t>(key >> 32);
        const uint32_t b = static_cast<uint32_t>(key);
        const Shape& sa = m_Shapes[a];
        const Shape& sb = m_Shapes[b];
        if (sa.alive && sb.alive && callback)
        {
            callback({ { a, sa.generation },
                       { b, sb.generation },
                       { sa.body, m_Bodies[sa.body].generation },
                       { sb.body, m_Bodies[sb.body].generation } });
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
        if (!(shape.def.layer & mask) || shape.def.sensor)
        {
            continue;
        }
        const Vec2 p = m_Bodies[shape.body].position;
        const float cx = p.x + shape.def.offset.x;
        const float cy = p.y + shape.def.offset.y;
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
            const float lo[2] = { cx - shape.def.halfWidth, cy - shape.def.halfHeight };
            const float hi[2] = { cx + shape.def.halfWidth, cy + shape.def.halfHeight };
            const float o[2] = { from.x, from.y };
            const float v[2] = { d.x, d.y };
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
            normal = hitAxisX ? Vec2{ d.x > 0.0f ? -1.0f : 1.0f, 0.0f } : Vec2{ 0.0f, d.y > 0.0f ? -1.0f : 1.0f };
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
