#include "Physics2DComponents.h"

#include <deki/Object.h>
#include <deki/Time.h>

#include <algorithm>
#include <vector>

namespace DekiPhysics2D
{

namespace
{

World s_World;
std::vector<Rigidbody2D*> s_Bodies;  // the first steps the world for all of them

Deki::Object* OwnerOfShape(ShapeHandle shape)
{
    auto* collider = static_cast<Collider2D*>(s_World.GetShapeUser(shape));
    return collider ? collider->GetOwner() : nullptr;
}

template <auto Member>
void Tell(BodyHandle body, ShapeHandle otherShape)
{
    auto* rigidbody = static_cast<Rigidbody2D*>(s_World.GetBodyUser(body));
    if (rigidbody && rigidbody->*Member)
    {
        (rigidbody->*Member)(OwnerOfShape(otherShape));
    }
}

void InstallContactCallbacks()
{
    if (s_World.onContactBegin)
    {
        return;
    }
    s_World.onContactBegin = [](const Contact& contact)
    {
        if (contact.trigger)
        {
            Tell<&Rigidbody2D::onTriggerBegin>(contact.bodyA, contact.shapeB);
            Tell<&Rigidbody2D::onTriggerBegin>(contact.bodyB, contact.shapeA);
        }
        else
        {
            Tell<&Rigidbody2D::onCollisionBegin>(contact.bodyA, contact.shapeB);
            Tell<&Rigidbody2D::onCollisionBegin>(contact.bodyB, contact.shapeA);
        }
    };
    s_World.onContactEnd = [](const Contact& contact)
    {
        if (contact.trigger)
        {
            Tell<&Rigidbody2D::onTriggerEnd>(contact.bodyA, contact.shapeB);
            Tell<&Rigidbody2D::onTriggerEnd>(contact.bodyB, contact.shapeA);
        }
        else
        {
            Tell<&Rigidbody2D::onCollisionEnd>(contact.bodyA, contact.shapeB);
            Tell<&Rigidbody2D::onCollisionEnd>(contact.bodyB, contact.shapeA);
        }
    };
}

}  // namespace

World& GetWorld()
{
    return s_World;
}

void Physics2DSettings::Awake()
{
    s_World.gravity = { gravityX, gravityY };
    s_World.fixedStep = 1.0f / static_cast<float>(std::max<int32_t>(stepsPerSecond, 1));
}

BodyHandle Rigidbody2D::EnsureBody()
{
    if (s_World.IsValid(m_Body))
    {
        return m_Body;
    }
    InstallContactCallbacks();
    BodyDef def;
    def.type = type;
    def.position = { GetOwner()->GetWorldX(), GetOwner()->GetWorldY() };
    def.angle = GetOwner()->GetWorldRotation();
    def.mass = mass;
    def.gravityScale = gravityScale;
    def.linearDamping = linearDamping;
    def.angularDamping = angularDamping;
    def.freezeRotation = freezeRotation;
    def.user = this;
    m_Body = s_World.CreateBody(def);
    s_Bodies.push_back(this);
    return m_Body;
}

void Rigidbody2D::Start()
{
    EnsureBody();
}

Rigidbody2D::~Rigidbody2D()
{
    s_Bodies.erase(std::remove(s_Bodies.begin(), s_Bodies.end(), this), s_Bodies.end());
    s_World.DestroyBody(m_Body);
}

void Rigidbody2D::Update()
{
    // One body steps the world for everyone, then moves every object, so a
    // script that runs after it sees the same frame as one that runs before.
    if (s_Bodies.empty() || s_Bodies.front() != this)
    {
        return;
    }
    if (s_World.Step(Deki::Time::GetDeltaTimeF() / 1000.0f) == 0)
    {
        return;
    }
    // A copy: moving an object can run code that destroys a body.
    const std::vector<Rigidbody2D*> bodies = s_Bodies;
    for (Rigidbody2D* body : bodies)
    {
        if (body->type != BodyType::Static && std::find(s_Bodies.begin(), s_Bodies.end(), body) != s_Bodies.end())
        {
            const Vec2 position = s_World.GetPosition(body->m_Body);
            Deki::Object* owner = body->GetOwner();
            owner->SetWorldPosition(position.x, position.y);
            if (!body->freezeRotation)
            {
                const Deki::Object* parent = owner->GetParent();
                owner->SetRotation(s_World.GetAngle(body->m_Body) - (parent ? parent->GetWorldRotation() : 0.0f));
            }
        }
    }
}

Vec2 Rigidbody2D::GetVelocity() const
{
    return s_World.GetVelocity(m_Body);
}

void Rigidbody2D::SetVelocity(Vec2 velocity)
{
    s_World.SetVelocity(EnsureBody(), velocity);
}

float Rigidbody2D::GetAngularVelocity() const
{
    return s_World.GetAngularVelocity(m_Body);
}

void Rigidbody2D::SetAngularVelocity(float radiansPerSecond)
{
    s_World.SetAngularVelocity(EnsureBody(), radiansPerSecond);
}

void Rigidbody2D::AddTorque(float torque)
{
    s_World.AddTorque(EnsureBody(), torque);
}

void Rigidbody2D::AddImpulseAt(Vec2 impulse, Vec2 worldPoint)
{
    s_World.AddImpulseAt(EnsureBody(), impulse, worldPoint);
}

void Rigidbody2D::AddForce(Vec2 force)
{
    s_World.AddForce(EnsureBody(), force);
}

void Rigidbody2D::AddImpulse(Vec2 impulse)
{
    s_World.AddImpulse(EnsureBody(), impulse);
}

void Collider2D::Start()
{
    ShapeDef shape;
    FillShape(shape);
    shape.offset = { offsetX, offsetY };
    shape.friction = friction;
    shape.bounciness = bounciness;
    shape.trigger = isTrigger;
    shape.layer = static_cast<uint16_t>(layer);
    shape.mask = static_cast<uint16_t>(mask);
    shape.user = this;

    BodyHandle body;
    if (auto* rigidbody = GetOwner()->GetComponent<Rigidbody2D>())
    {
        body = rigidbody->EnsureBody();
    }
    else
    {
        BodyDef def;
        def.type = BodyType::Static;
        def.position = { GetOwner()->GetWorldX(), GetOwner()->GetWorldY() };
        def.angle = GetOwner()->GetWorldRotation();
        m_OwnStaticBody = s_World.CreateBody(def);
        body = m_OwnStaticBody;
    }
    m_Shape = s_World.AddShape(body, shape);
}

Collider2D::~Collider2D()
{
    s_World.RemoveShape(m_Shape);
    s_World.DestroyBody(m_OwnStaticBody);
}

void BoxCollider2D::FillShape(ShapeDef& shape) const
{
    shape.isCircle = false;
    shape.halfWidth = width * 0.5f * GetOwner()->GetWorldScaleX();
    shape.halfHeight = height * 0.5f * GetOwner()->GetWorldScaleY();
}

void CircleCollider2D::FillShape(ShapeDef& shape) const
{
    shape.isCircle = true;
    shape.radius = radius * std::max(GetOwner()->GetWorldScaleX(), GetOwner()->GetWorldScaleY());
}

bool Raycast(Vec2 from, Vec2 to, RayHit& hit, Deki::Object** object, uint16_t mask)
{
    if (!s_World.Raycast(from, to, hit, mask))
    {
        return false;
    }
    if (object)
    {
        *object = OwnerOfShape(hit.shape);
    }
    return true;
}

}  // namespace DekiPhysics2D
