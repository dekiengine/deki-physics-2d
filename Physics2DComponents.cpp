#include "Physics2DComponents.h"

#include <deki/Object.h>
#include <deki/LogSystem.h>
#include <deki/Time.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace DekiPhysics2D
{

namespace
{

World s_World;
std::vector<Rigidbody2D*> s_Bodies;  // the first steps the world for all of them

constexpr float kDegreesToRadians = 3.14159265f / 180.0f;

// A point given from an object's centre, turning and scaling with it.
Vec2 PointOn(const Deki::Object& object, float x, float y)
{
    const float scaledX = x * object.GetWorldScaleX();
    const float scaledY = y * object.GetWorldScaleY();
    const float rotation = object.GetWorldRotation();
    const float c = std::cos(rotation);
    const float s = std::sin(rotation);
    return { object.GetWorldX() + scaledX * c - scaledY * s, object.GetWorldY() + scaledX * s + scaledY * c };
}

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
    s_World.allowSleep = allowSleep;
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
    def.canSleep = canSleep;
    def.continuous = continuousCollision;
    def.user = this;
    m_Body = s_World.CreateBody(def);
    m_ObjectX = def.position.x;
    m_ObjectY = def.position.y;
    m_ObjectRotation = def.angle;
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
    for (Rigidbody2D* body : s_Bodies)
    {
        body->FollowObject();
    }
    if (s_World.Step(Deki::Time::GetDeltaTimeF() / 1000.0f) == 0)
    {
        return;
    }
    // By index: moving an object can run code that destroys a body.
    for (size_t i = 0; i < s_Bodies.size(); ++i)
    {
        s_Bodies[i]->MoveObject();
    }
}

// An object something else moved since physics last looked: the body goes
// with it.
void Rigidbody2D::FollowObject()
{
    const Deki::Object* owner = GetOwner();
    const float x = owner->GetWorldX();
    const float y = owner->GetWorldY();
    const float rotation = owner->GetWorldRotation();
    if (x != m_ObjectX || y != m_ObjectY)
    {
        s_World.SetPosition(m_Body, { x, y });
    }
    if (rotation != m_ObjectRotation)
    {
        s_World.SetAngle(m_Body, rotation);
    }
    m_ObjectX = x;
    m_ObjectY = y;
    m_ObjectRotation = rotation;
}

void Rigidbody2D::MoveObject()
{
    if (type == BodyType::Static || !s_World.IsAwake(m_Body))
    {
        return;
    }
    const Vec2 position = s_World.GetPosition(m_Body);
    Deki::Object* owner = GetOwner();
    owner->SetWorldPosition(position.x, position.y);
    if (!freezeRotation)
    {
        const Deki::Object* parent = owner->GetParent();
        owner->SetRotation(s_World.GetAngle(m_Body) - (parent ? parent->GetWorldRotation() : 0.0f));
    }
    // Read back rather than remembered: a parent's transform can round them.
    m_ObjectX = owner->GetWorldX();
    m_ObjectY = owner->GetWorldY();
    m_ObjectRotation = owner->GetWorldRotation();
}

bool Rigidbody2D::IsAwake() const
{
    return s_World.IsAwake(m_Body);
}

void Rigidbody2D::WakeUp()
{
    s_World.WakeUp(EnsureBody());
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

ShapeHandle Collider2D::AddToWorld(BodyHandle body)
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
    const ShapeHandle handle = s_World.AddShape(body, shape);
    if (handle == ShapeHandle{})
    {
        DEKI_LOG_ERROR("Physics 2D: the collider on '%s' has no area, so it collides with nothing. Give it a size, "
                       "or a polygon at least three corners that are not in a line.",
                       GetOwner()->GetName().c_str());
    }
    return handle;
}

void Collider2D::Start()
{
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
    m_Shape = AddToWorld(body);
    m_ObjectX = GetOwner()->GetWorldX();
    m_ObjectY = GetOwner()->GetWorldY();
    m_ObjectRotation = GetOwner()->GetWorldRotation();
    m_ObjectScaleX = GetOwner()->GetWorldScaleX();
    m_ObjectScaleY = GetOwner()->GetWorldScaleY();
}

// A collider with no Rigidbody 2D has nothing else to keep it with its object.
void Collider2D::Update()
{
    if (!s_World.IsValid(m_OwnStaticBody))
    {
        return;
    }
    const Deki::Object* owner = GetOwner();
    const float x = owner->GetWorldX();
    const float y = owner->GetWorldY();
    const float rotation = owner->GetWorldRotation();
    const float scaleX = owner->GetWorldScaleX();
    const float scaleY = owner->GetWorldScaleY();
    if (x != m_ObjectX || y != m_ObjectY)
    {
        s_World.SetPosition(m_OwnStaticBody, { x, y });
        m_ObjectX = x;
        m_ObjectY = y;
    }
    if (rotation != m_ObjectRotation)
    {
        s_World.SetAngle(m_OwnStaticBody, rotation);
        m_ObjectRotation = rotation;
    }
    if (scaleX != m_ObjectScaleX || scaleY != m_ObjectScaleY)
    {
        s_World.RemoveShape(m_Shape);
        m_Shape = AddToWorld(m_OwnStaticBody);
        m_ObjectScaleX = scaleX;
        m_ObjectScaleY = scaleY;
    }
}

Collider2D::~Collider2D()
{
    s_World.RemoveShape(m_Shape);
    s_World.DestroyBody(m_OwnStaticBody);
}

void BoxCollider2D::FillShape(ShapeDef& shape) const
{
    shape.type = ShapeType::Box;
    shape.halfWidth = width * 0.5f * GetOwner()->GetWorldScaleX();
    shape.halfHeight = height * 0.5f * GetOwner()->GetWorldScaleY();
}

void CircleCollider2D::FillShape(ShapeDef& shape) const
{
    shape.type = ShapeType::Circle;
    shape.radius = radius * std::max(GetOwner()->GetWorldScaleX(), GetOwner()->GetWorldScaleY());
}

void CapsuleCollider2D::FillShape(ShapeDef& shape) const
{
    const bool vertical = direction == CapsuleDirection::Vertical;
    const float along = vertical ? GetOwner()->GetWorldScaleY() : GetOwner()->GetWorldScaleX();
    const float across = vertical ? GetOwner()->GetWorldScaleX() : GetOwner()->GetWorldScaleY();
    shape.radius = radius * across;
    shape.halfLength = height * 0.5f * along - shape.radius;
    shape.angle = vertical ? 1.5707963f : 0.0f;
    // No longer than it is wide: its two round ends are one circle.
    shape.type = shape.halfLength > 0.0f ? ShapeType::Capsule : ShapeType::Circle;
}

void PolygonCollider2D::FillShape(ShapeDef& shape) const
{
    shape.type = ShapeType::Polygon;
    shape.pointCount = std::min<int32_t>(pointCount, kMaxPolygonPoints);
    for (int i = 0; i < shape.pointCount; ++i)
    {
        shape.points[i] = { Point(i).x * GetOwner()->GetWorldScaleX(), Point(i).y * GetOwner()->GetWorldScaleY() };
    }
}

Joint2D::~Joint2D()
{
    s_World.DestroyJoint(m_Joint);
}

void Joint2D::Start()
{
    auto* own = GetOwner()->GetComponent<Rigidbody2D>();
    if (!own)
    {
        DEKI_LOG_ERROR("Physics 2D: the joint on '%s' needs a Rigidbody 2D on the same object, so it joins nothing.",
                       GetOwner()->GetName().c_str());
        return;
    }
    Rigidbody2D* other = connectedBody.Get();
    m_Joint = Create(own->EnsureBody(), other ? other->EnsureBody() : BodyHandle{}, other,
                     PointOn(*GetOwner(), anchorX, anchorY));
}

JointHandle HingeJoint2D::Create(BodyHandle own, BodyHandle other, Rigidbody2D* otherBody, Vec2 worldAnchor)
{
    // The world measures body B against body A. This object is the one that
    // turns, so it is B, unless there is no other body to be A.
    const float direction = otherBody ? 1.0f : -1.0f;
    HingeDef def;
    def.bodyA = otherBody ? other : own;
    def.bodyB = otherBody ? own : BodyHandle{};
    def.anchor = worldAnchor;
    def.collideConnected = collideConnected;
    def.limited = useLimits;
    def.lowerAngle = (otherBody ? lowerAngle : -upperAngle) * kDegreesToRadians;
    def.upperAngle = (otherBody ? upperAngle : -lowerAngle) * kDegreesToRadians;
    def.motor = useMotor;
    def.motorSpeed = direction * motorSpeed * kDegreesToRadians;
    def.maxMotorTorque = maxMotorTorque;
    return GetWorld().CreateHinge(def);
}

JointHandle DistanceJoint2D::Create(BodyHandle own, BodyHandle other, Rigidbody2D* otherBody, Vec2 worldAnchor)
{
    DistanceDef def;
    def.bodyA = own;
    def.bodyB = other;
    def.anchorA = worldAnchor;
    def.anchorB = otherBody ? PointOn(*otherBody->GetOwner(), connectedAnchorX, connectedAnchorY)
                            : Vec2{ connectedAnchorX, connectedAnchorY };
    def.length = length > 0.0f ? length : -1.0f;
    def.rope = rope;
    def.collideConnected = collideConnected;
    return GetWorld().CreateDistance(def);
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
