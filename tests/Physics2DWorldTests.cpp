// The physics world on its own: what a game can rely on it to do.

#include <gtest/gtest.h>

#include <Physics2DWorld.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <new>
#include <vector>

using namespace DekiPhysics2D;

namespace
{

// The tests read as a screen does, with +Y down; the world does not care which
// way gravity points.
struct TestWorld : World
{
    TestWorld() { gravity = { 0.0f, 9.81f }; }
};

BodyHandle AddBox(World& world, BodyType type, Vec2 position, float halfWidth = 0.5f, float halfHeight = 0.5f,
                  float bounciness = 0.0f)
{
    BodyDef body;
    body.type = type;
    body.position = position;
    BodyHandle handle = world.CreateBody(body);
    ShapeDef shape;
    shape.halfWidth = halfWidth;
    shape.halfHeight = halfHeight;
    shape.bounciness = bounciness;
    world.AddShape(handle, shape);
    return handle;
}

BodyHandle AddCircle(World& world, BodyType type, Vec2 position, float radius = 0.5f)
{
    BodyDef body;
    body.type = type;
    body.position = position;
    BodyHandle handle = world.CreateBody(body);
    ShapeDef shape;
    shape.type = ShapeType::Circle;
    shape.radius = radius;
    world.AddShape(handle, shape);
    return handle;
}

void Simulate(World& world, float seconds)
{
    const int frames = static_cast<int>(seconds * 60.0f + 0.5f);
    for (int i = 0; i < frames; ++i)
    {
        world.Step(1.0f / 60.0f);
    }
}

}  // namespace

TEST(Physics2DWorld, AFreeBodyFallsAsGravitySays)
{
    TestWorld world;
    BodyHandle body = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    Simulate(world, 1.0f);
    // Semi-implicit Euler lands a little past s = g t^2 / 2 (4.905 m).
    EXPECT_NEAR(world.GetPosition(body).y, 4.905f, 0.1f);
    EXPECT_NEAR(world.GetVelocity(body).y, 9.81f, 0.05f);
}

TEST(Physics2DWorld, ABodyComesToRestOnTheGround)
{
    TestWorld world;
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(box).y, 4.0f, 0.05f);
    EXPECT_NEAR(world.GetVelocity(box).y, 0.0f, 0.3f);
}

TEST(Physics2DWorld, AStackStaysStacked)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    std::vector<BodyHandle> stack;
    for (int i = 0; i < 4; ++i)
    {
        stack.push_back(AddBox(world, BodyType::Dynamic, { 0.0f, 3.5f - static_cast<float>(i) * 1.05f }));
    }
    Simulate(world, 5.0f);
    for (size_t i = 0; i < stack.size(); ++i)
    {
        EXPECT_NEAR(world.GetPosition(stack[i]).x, 0.0f, 0.01f);
        EXPECT_NEAR(world.GetPosition(stack[i]).y, 4.0f - static_cast<float>(i), 0.1f) << "box " << i;
    }
}

TEST(Physics2DWorld, ABouncyStackStillComesToRest)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f, 0.1f);
    std::vector<BodyHandle> stack;
    for (int i = 0; i < 4; ++i)
    {
        stack.push_back(AddBox(world, BodyType::Dynamic, { 0.0f, 3.99f - static_cast<float>(i) }, 0.5f, 0.5f, 0.1f));
    }
    Simulate(world, 5.0f);
    for (const BodyHandle& box : stack)
    {
        EXPECT_FALSE(world.IsAwake(box));
        EXPECT_NEAR(world.GetPosition(box).x, 0.0f, 0.02f) << "a stack does not walk off sideways";
    }
}

TEST(Physics2DWorld, RestitutionBouncesAndZeroDoesNot)
{
    TestWorld bouncy;
    BodyHandle ball = AddBox(bouncy, BodyType::Dynamic, { 0.0f, 0.0f }, 0.5f, 0.5f, 0.8f);
    AddBox(bouncy, BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    float highestAfterHit = 100.0f;
    bool hit = false;
    for (int i = 0; i < 240; ++i)
    {
        bouncy.Step(1.0f / 60.0f);
        hit = hit || bouncy.GetVelocity(ball).y < -1.0f;
        if (hit)
        {
            highestAfterHit = std::min(highestAfterHit, bouncy.GetPosition(ball).y);
        }
    }
    ASSERT_TRUE(hit);
    EXPECT_LT(highestAfterHit, 3.0f) << "it bounced back up";
    EXPECT_GT(highestAfterHit, 0.0f) << "it cannot gain energy";
}

TEST(Physics2DWorld, CircleLandsOnBox)
{
    TestWorld world;
    BodyHandle ball = AddCircle(world, BodyType::Dynamic, { 1.0f, 0.0f }, 0.5f);
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(ball).y, 4.0f, 0.05f);
}

TEST(Physics2DWorld, CirclesCollide)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle left = AddCircle(world, BodyType::Dynamic, { 0.0f, 0.0f });
    BodyHandle right = AddCircle(world, BodyType::Dynamic, { 3.0f, 0.0f });
    world.SetVelocity(left, { 2.0f, 0.0f });
    Simulate(world, 2.0f);
    EXPECT_GT(world.GetVelocity(right).x, 0.5f) << "the moving circle pushed the other";
    EXPECT_GE(world.GetPosition(right).x - world.GetPosition(left).x, 0.99f) << "they do not pass through";
}

TEST(Physics2DWorld, FrictionStopsASlidingBody)
{
    TestWorld world;
    BodyDef def;
    def.position = { 0.0f, 4.0f };
    def.velocity = { 5.0f, 0.0f };
    BodyHandle box = world.CreateBody(def);
    ShapeDef shape;
    shape.friction = 1.0f;
    world.AddShape(box, shape);
    BodyHandle ground = AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 50.0f, 0.5f);
    (void)ground;
    Simulate(world, 3.0f);
    EXPECT_LT(std::fabs(world.GetVelocity(box).x), 0.5f);
    EXPECT_GT(world.GetPosition(box).x, 0.5f) << "it slid before it stopped";
}

TEST(Physics2DWorld, AFixedStepMakesTheFrameRateIrrelevant)
{
    TestWorld at60;
    TestWorld at20;
    BodyHandle a = AddBox(at60, BodyType::Dynamic, { 0.0f, 0.0f });
    BodyHandle b = AddBox(at20, BodyType::Dynamic, { 0.0f, 0.0f });
    for (int i = 0; i < 60; ++i)
    {
        at60.Step(1.0f / 60.0f);
    }
    for (int i = 0; i < 20; ++i)
    {
        at20.Step(1.0f / 20.0f);
    }
    EXPECT_NEAR(at60.GetPosition(a).y, at20.GetPosition(b).y, 0.02f);
}

TEST(Physics2DWorld, AVerySlowFrameRunsAtMostItsSteps)
{
    TestWorld world;
    AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    EXPECT_EQ(world.Step(10.0f), world.maxStepsPerFrame);
    EXPECT_EQ(world.Step(1.0f / 120.0f), 0) << "the time it could not run is dropped, not owed";
}

TEST(Physics2DWorld, KinematicPushesAndIsNotPushed)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle wall = AddBox(world, BodyType::Kinematic, { 0.0f, 0.0f });
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 2.0f, 0.0f });
    world.SetVelocity(wall, { 1.0f, 0.0f });
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(wall).x, 3.0f, 0.05f);
    EXPECT_GT(world.GetPosition(box).x, 3.5f);
}

TEST(Physics2DWorld, ATriggerReportsAndPushesNothing)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    world.SetVelocity(box, { 2.0f, 0.0f });

    BodyDef zoneDef;
    zoneDef.type = BodyType::Static;
    zoneDef.position = { 3.0f, 0.0f };
    BodyHandle zone = world.CreateBody(zoneDef);
    ShapeDef trigger;
    trigger.trigger = true;
    world.AddShape(zone, trigger);

    int began = 0;
    int ended = 0;
    world.onContactBegin = [&](const Contact&) { ++began; };
    world.onContactEnd = [&](const Contact&) { ++ended; };
    Simulate(world, 4.0f);

    EXPECT_NEAR(world.GetVelocity(box).x, 2.0f, 0.001f);
    EXPECT_EQ(began, 1);
    EXPECT_EQ(ended, 1);
}

TEST(Physics2DWorld, LayersMustAgreeBothWays)
{
    TestWorld world;
    BodyDef def;
    BodyHandle box = world.CreateBody(def);
    ShapeDef shape;
    shape.layer = 2;
    shape.mask = 2;
    world.AddShape(box, shape);
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);  // layer 1
    Simulate(world, 3.0f);
    EXPECT_GT(world.GetPosition(box).y, 8.0f) << "it fell through a layer it does not collide with";
}

TEST(Physics2DWorld, RaycastFindsTheNearestAndItsNormal)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle far = AddBox(world, BodyType::Static, { 10.0f, 0.0f });
    BodyHandle near = AddBox(world, BodyType::Static, { 5.0f, 0.0f });
    (void)far;

    RayHit hit;
    ASSERT_TRUE(world.Raycast({ 0.0f, 0.0f }, { 20.0f, 0.0f }, hit));
    EXPECT_EQ(hit.body, near);
    EXPECT_NEAR(hit.point.x, 4.5f, 0.001f);
    EXPECT_FLOAT_EQ(hit.normal.x, -1.0f);
    EXPECT_FALSE(world.Raycast({ 0.0f, 3.0f }, { 20.0f, 3.0f }, hit));
}

TEST(Physics2DWorld, RaycastHitsACircle)
{
    TestWorld world;
    world.gravity = {};
    AddCircle(world, BodyType::Static, { 0.0f, 5.0f }, 1.0f);
    RayHit hit;
    ASSERT_TRUE(world.Raycast({ 0.0f, 0.0f }, { 0.0f, 10.0f }, hit));
    EXPECT_NEAR(hit.point.y, 4.0f, 0.001f);
    EXPECT_NEAR(hit.normal.y, -1.0f, 0.001f);
}

TEST(Physics2DWorld, AStaleHandleNamesNothing)
{
    TestWorld world;
    BodyHandle first = AddBox(world, BodyType::Dynamic, { 1.0f, 1.0f });
    world.DestroyBody(first);
    EXPECT_FALSE(world.IsValid(first));
    BodyHandle second = AddBox(world, BodyType::Dynamic, { 2.0f, 2.0f });
    EXPECT_EQ(first.index, second.index) << "the slot is reused";
    EXPECT_FALSE(world.IsValid(first));
    world.SetPosition(first, { 9.0f, 9.0f });
    EXPECT_FLOAT_EQ(world.GetPosition(second).x, 2.0f) << "the old handle cannot move the new body";
    EXPECT_EQ(world.BodyCount(), 1u);
    EXPECT_EQ(world.ShapeCount(), 1u);
}

TEST(Physics2DWorld, DestroyingATouchingBodyEndsTheContact)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle a = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    BodyHandle b = AddBox(world, BodyType::Dynamic, { 0.5f, 0.0f });
    int ended = 0;
    world.onContactEnd = [&](const Contact&) { ++ended; };
    Simulate(world, 0.1f);
    (void)b;
    world.DestroyBody(a);
    EXPECT_GE(ended, 0);
    Simulate(world, 0.1f);  // and the step after does not touch the dead shape
    EXPECT_EQ(world.ShapeCount(), 1u);
}

TEST(Physics2DWorld, AContactCallbackMayDestroyABody)
{
    TestWorld world;
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    AddBox(world, BodyType::Static, { 0.0f, 3.0f }, 10.0f, 0.5f);
    world.onContactBegin = [&](const Contact&) { world.DestroyBody(box); };
    Simulate(world, 2.0f);
    EXPECT_FALSE(world.IsValid(box));
}

// ---- rotation ------------------------------------------------------------

TEST(Physics2DWorld, AnImpulseOffCentreSpinsABody)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    world.AddImpulseAt(box, { 0.0f, 1.0f }, { 0.5f, 0.0f });
    EXPECT_GT(world.GetAngularVelocity(box), 0.0f) << "a push up on the right turns it counter-clockwise";
    Simulate(world, 0.5f);
    EXPECT_GT(world.GetAngle(box), 0.1f);
}

TEST(Physics2DWorld, FixedRotationNeverTurns)
{
    TestWorld world;
    world.gravity = {};
    BodyDef def;
    def.freezeRotation = true;
    BodyHandle box = world.CreateBody(def);
    world.AddShape(box, ShapeDef{});
    world.AddImpulseAt(box, { 0.0f, 1.0f }, { 0.5f, 0.0f });
    world.AddTorque(box, 100.0f);
    Simulate(world, 0.5f);
    EXPECT_FLOAT_EQ(world.GetAngle(box), 0.0f);
}

TEST(Physics2DWorld, ABoxOffALedgeTipsOver)
{
    TestWorld world;
    // Its centre is past the ledge's right edge (+Y is down here).
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 1.3f, 3.9f });
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 1.0f, 0.5f);
    Simulate(world, 2.0f);
    EXPECT_GT(std::fabs(world.GetAngle(box)), 0.3f) << "unsupported on one side, it turned";
    EXPECT_GT(world.GetPosition(box).y, 5.0f) << "and fell past the ledge";
}

TEST(Physics2DWorld, ATiltedBoxSettlesFlatOnTheGround)
{
    TestWorld world;
    BodyDef def;
    def.position = { 0.0f, 2.0f };
    def.angle = 0.5f;
    BodyHandle box = world.CreateBody(def);
    world.AddShape(box, ShapeDef{});
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    Simulate(world, 5.0f);

    const float quarter = 3.14159265f / 2.0f;
    const float remainder = std::fabs(std::remainder(world.GetAngle(box), quarter));
    EXPECT_LT(remainder, 0.05f) << "a face is on the ground";
    EXPECT_NEAR(world.GetPosition(box).y, 4.0f, 0.05f);
    EXPECT_LT(std::fabs(world.GetAngularVelocity(box)), 0.2f) << "it has stopped rocking";
}

TEST(Physics2DWorld, ACircleRollsDownASlope)
{
    TestWorld world;
    BodyDef slope;
    slope.type = BodyType::Static;
    slope.position = { 0.0f, 5.0f };
    slope.angle = 0.3f;  // lower on the right with +Y down
    BodyHandle ramp = world.CreateBody(slope);
    ShapeDef surface;
    surface.halfWidth = 20.0f;
    surface.halfHeight = 0.5f;
    surface.friction = 1.0f;
    world.AddShape(ramp, surface);

    BodyDef def;
    def.position = { 0.0f, 3.5f };
    BodyHandle ball = world.CreateBody(def);
    ShapeDef shape;
    shape.type = ShapeType::Circle;
    shape.radius = 0.5f;
    shape.friction = 1.0f;
    world.AddShape(ball, shape);

    Simulate(world, 2.0f);
    EXPECT_GT(world.GetPosition(ball).x, 1.0f) << "down the slope";
    EXPECT_GT(std::fabs(world.GetAngularVelocity(ball)), 1.0f) << "and turning as it goes";
}

TEST(Physics2DWorld, RaycastHitsATurnedBox)
{
    TestWorld world;
    world.gravity = {};
    BodyDef def;
    def.type = BodyType::Static;
    def.position = { 5.0f, 0.0f };
    def.angle = 3.14159265f / 4.0f;
    BodyHandle box = world.CreateBody(def);
    world.AddShape(box, ShapeDef{});

    RayHit hit;
    ASSERT_TRUE(world.Raycast({ 0.0f, 0.0f }, { 10.0f, 0.0f }, hit));
    EXPECT_NEAR(hit.point.x, 5.0f - std::sqrt(0.5f), 0.01f) << "the corner points at the ray";
    EXPECT_FALSE(world.Raycast({ 0.0f, 0.75f }, { 10.0f, 0.75f }, hit)) << "beyond the corner";
}

TEST(Physics2DWorld, ATriggerContactSaysItIsATrigger)
{
    TestWorld world;
    world.gravity = {};
    BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    BodyDef zoneDef;
    zoneDef.type = BodyType::Static;
    BodyHandle zone = world.CreateBody(zoneDef);
    ShapeDef trigger;
    trigger.trigger = true;
    world.AddShape(zone, trigger);

    int began = 0;
    bool wasTrigger = false;
    world.onContactBegin = [&](const Contact& contact)
    {
        ++began;
        wasTrigger = contact.trigger;
    };
    (void)box;
    Simulate(world, 1.0f);
    EXPECT_EQ(began, 1);
    EXPECT_TRUE(wasTrigger);
}

// ---------------------------------------------------------------------------
// No allocation per step

namespace
{
size_t s_Allocations = 0;
}

void* operator new(std::size_t size)
{
    ++s_Allocations;
    void* memory = std::malloc(size);
    if (!memory)
    {
        std::abort();
    }
    return memory;
}

void operator delete(void* memory) noexcept
{
    std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
    std::free(memory);
}

namespace
{

BodyHandle AddBody(World& world, BodyType type, Vec2 position, const ShapeDef& shape)
{
    BodyDef body;
    body.type = type;
    body.position = position;
    BodyHandle handle = world.CreateBody(body);
    world.AddShape(handle, shape);
    return handle;
}

ShapeDef CapsuleShape(float radius, float halfLength, float angle = 0.0f)
{
    ShapeDef shape;
    shape.type = ShapeType::Capsule;
    shape.radius = radius;
    shape.halfLength = halfLength;
    shape.angle = angle;
    return shape;
}

ShapeDef TriangleShape()
{
    ShapeDef shape;
    shape.type = ShapeType::Polygon;
    shape.points[0] = { -1.0f, 0.5f };
    shape.points[1] = { 1.0f, 0.5f };
    shape.points[2] = { 0.0f, -0.5f };
    shape.pointCount = 3;
    return shape;
}

// Long side along X and short side up the screen, with the object's origin at
// the right angle: its centre of mass is at (2/3, -1/3), well away from it.
ShapeDef CornerTriangleShape()
{
    ShapeDef shape;
    shape.type = ShapeType::Polygon;
    shape.points[0] = { 0.0f, 0.0f };
    shape.points[1] = { 2.0f, 0.0f };
    shape.points[2] = { 0.0f, -1.0f };
    shape.pointCount = 3;
    return shape;
}

float Distance(Vec2 a, Vec2 b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

}  // namespace

TEST(Physics2DWorld, ARunningWorldAllocatesNothing)
{
    TestWorld world;
    world.allowSleep = false;
    AddBox(world, BodyType::Static, { 0.0f, 10.0f }, 20.0f, 0.5f);
    for (int i = 0; i < 12; ++i)
    {
        AddBox(world, BodyType::Dynamic, { -5.0f + i, 0.0f });
        AddCircle(world, BodyType::Dynamic, { -5.0f + i, -3.0f });
    }
    int contacts = 0;
    world.onContactBegin = [&](const Contact&) { ++contacts; };
    Simulate(world, 5.0f);  // long enough for the lists to reach their size
    ASSERT_GT(contacts, 0);

    const size_t before = s_Allocations;
    Simulate(world, 1.0f);
    EXPECT_EQ(s_Allocations, before);
}

// ---------------------------------------------------------------------------
// Sleeping

TEST(Physics2DWorld, ABodyAtRestGoesToSleep)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 3.0f });
    EXPECT_TRUE(world.IsAwake(box));
    Simulate(world, 4.0f);
    EXPECT_FALSE(world.IsAwake(box));
    const Vec2 at = world.GetPosition(box);
    Simulate(world, 1.0f);
    EXPECT_EQ(world.GetPosition(box).y, at.y) << "a sleeping body stays exactly where it is";
}

TEST(Physics2DWorld, CanSleepFalseKeepsItAwake)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    BodyDef def;
    def.position = { 0.0f, 3.0f };
    def.canSleep = false;
    const BodyHandle box = world.CreateBody(def);
    world.AddShape(box, ShapeDef{});
    Simulate(world, 4.0f);
    EXPECT_TRUE(world.IsAwake(box));
}

TEST(Physics2DWorld, SleepingDoesNotEndAContact)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 3.0f });
    int begins = 0, ends = 0;
    world.onContactBegin = [&](const Contact&) { ++begins; };
    world.onContactEnd = [&](const Contact&) { ++ends; };
    Simulate(world, 4.0f);
    ASSERT_FALSE(world.IsAwake(box));
    EXPECT_EQ(begins, 1);
    EXPECT_EQ(ends, 0);
}

TEST(Physics2DWorld, AStackSleepsAndWakesTogether)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    std::vector<BodyHandle> stack;
    for (int i = 0; i < 4; ++i)
    {
        stack.push_back(AddBox(world, BodyType::Dynamic, { 0.0f, 3.99f - i * 1.0f }));
    }
    Simulate(world, 5.0f);
    for (const BodyHandle& box : stack)
    {
        ASSERT_FALSE(world.IsAwake(box));
    }

    world.AddImpulse(stack.back(), { 0.1f, 0.0f });
    for (const BodyHandle& box : stack)
    {
        EXPECT_TRUE(world.IsAwake(box));
    }
    Simulate(world, 1.0f);
    EXPECT_NEAR(world.GetPosition(stack[0]).y, 4.0f, 0.1f) << "waking must not drop the stack";
}

TEST(Physics2DWorld, SomethingLandingOnASleeperWakesIt)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle sleeper = AddBox(world, BodyType::Dynamic, { 0.0f, 4.0f });
    Simulate(world, 3.0f);
    ASSERT_FALSE(world.IsAwake(sleeper));
    AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    Simulate(world, 0.9f);
    EXPECT_TRUE(world.IsAwake(sleeper));
}

TEST(Physics2DWorld, RemovingTheFloorWakesWhatRestedOnIt)
{
    TestWorld world;
    const BodyHandle floor = AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 3.0f });
    Simulate(world, 4.0f);
    ASSERT_FALSE(world.IsAwake(box));
    const float restedAt = world.GetPosition(box).y;
    world.DestroyBody(floor);
    Simulate(world, 0.5f);
    EXPECT_GT(world.GetPosition(box).y, restedAt + 0.5f);
}

TEST(Physics2DWorld, ChangingGravityWakesEverything)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 3.0f });
    Simulate(world, 4.0f);
    ASSERT_FALSE(world.IsAwake(box));
    const float restedAt = world.GetPosition(box).y;
    world.gravity = { 0.0f, -9.81f };
    Simulate(world, 0.5f);
    EXPECT_LT(world.GetPosition(box).y, restedAt - 0.5f);
}

// ---------------------------------------------------------------------------
// A static collider that moves

TEST(Physics2DWorld, MovingAStaticPlatformAwayLetsItsLoadFall)
{
    TestWorld world;
    const BodyHandle platform = AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 2.0f, 0.5f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 3.0f });
    Simulate(world, 4.0f);
    ASSERT_FALSE(world.IsAwake(box));
    const float restedAt = world.GetPosition(box).y;
    world.SetPosition(platform, { 50.0f, 5.0f });
    Simulate(world, 0.5f);
    EXPECT_GT(world.GetPosition(box).y, restedAt + 0.5f);
}

TEST(Physics2DWorld, AStaticPlacedIntoASleeperPushesItOut)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle wall = AddBox(world, BodyType::Static, { 30.0f, 3.0f }, 0.5f, 3.0f);
    const BodyHandle box = AddBox(world, BodyType::Dynamic, { 0.0f, 4.0f });
    Simulate(world, 3.0f);
    ASSERT_FALSE(world.IsAwake(box));
    world.SetPosition(wall, { 0.8f, 3.0f });
    Simulate(world, 1.0f);
    EXPECT_NEAR(world.GetPosition(box).x, -0.2f, 0.02f);
}

TEST(Physics2DWorld, RaycastSeesAStaticWhereItWasMoved)
{
    TestWorld world;
    const BodyHandle wall = AddBox(world, BodyType::Static, { 5.0f, 0.0f });
    world.SetPosition(wall, { 8.0f, 0.0f });
    RayHit hit;
    ASSERT_TRUE(world.Raycast({ 0.0f, 0.0f }, { 20.0f, 0.0f }, hit));
    EXPECT_NEAR(hit.point.x, 7.5f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Fast bodies

TEST(Physics2DWorld, AFastBodyStopsAtAThinWall)
{
    TestWorld world;
    world.gravity = {};
    AddBox(world, BodyType::Static, { 10.0f, 0.0f }, 0.02f, 5.0f);
    BodyDef def;
    def.velocity = { 200.0f, 0.0f };
    const BodyHandle ball = world.CreateBody(def);
    ShapeDef shape;
    shape.type = ShapeType::Circle;
    shape.radius = 0.1f;
    world.AddShape(ball, shape);
    Simulate(world, 0.5f);
    EXPECT_LT(world.GetPosition(ball).x, 10.0f) << "it must not be behind the wall";
    EXPECT_NEAR(world.GetVelocity(ball).x, 0.0f, 1.0f);
}

TEST(Physics2DWorld, AFastBodyBouncesOffAThinWall)
{
    TestWorld world;
    world.gravity = {};
    ShapeDef wall;
    wall.halfWidth = 0.02f;
    wall.halfHeight = 5.0f;
    wall.bounciness = 1.0f;
    AddBody(world, BodyType::Static, { 10.0f, 0.0f }, wall);
    BodyDef def;
    def.velocity = { 100.0f, 0.0f };
    const BodyHandle box = world.CreateBody(def);
    ShapeDef shape;
    shape.halfWidth = 0.1f;
    shape.halfHeight = 0.1f;
    world.AddShape(box, shape);
    Simulate(world, 0.3f);
    EXPECT_LT(world.GetVelocity(box).x, -50.0f);
    EXPECT_LT(world.GetPosition(box).x, 10.0f);
}

TEST(Physics2DWorld, ContinuousOffMayPassThrough)
{
    TestWorld world;
    world.gravity = {};
    AddBox(world, BodyType::Static, { 10.0f, 0.0f }, 0.02f, 5.0f);
    BodyDef def;
    def.velocity = { 200.0f, 0.0f };
    def.continuous = false;
    const BodyHandle ball = world.CreateBody(def);
    ShapeDef shape;
    shape.type = ShapeType::Circle;
    shape.radius = 0.1f;
    world.AddShape(ball, shape);
    Simulate(world, 0.5f);
    EXPECT_GT(world.GetPosition(ball).x, 10.0f);
}

TEST(Physics2DWorld, AFastBodyHitsAnotherBody)
{
    TestWorld world;
    world.gravity = {};
    const BodyHandle target = AddBox(world, BodyType::Dynamic, { 10.0f, 0.0f }, 0.05f, 2.0f);
    BodyDef def;
    def.velocity = { 150.0f, 0.0f };
    const BodyHandle ball = world.CreateBody(def);
    ShapeDef shape;
    shape.type = ShapeType::Circle;
    shape.radius = 0.1f;
    world.AddShape(ball, shape);
    Simulate(world, 0.2f);
    EXPECT_GT(world.GetVelocity(target).x, 10.0f);
}

TEST(Physics2DWorld, SlidingFastAlongTheFloorKeepsGoing)
{
    TestWorld world;
    ShapeDef floor;
    floor.halfWidth = 500.0f;
    floor.friction = 0.0f;
    AddBody(world, BodyType::Static, { 0.0f, 5.0f }, floor);
    ShapeDef shape;
    shape.halfWidth = 0.1f;
    shape.halfHeight = 0.1f;
    shape.friction = 0.0f;
    const BodyHandle box = AddBody(world, BodyType::Dynamic, { 0.0f, 4.4f }, shape);
    Simulate(world, 1.0f);
    world.SetVelocity(box, { 60.0f, 0.0f });
    Simulate(world, 0.5f);
    EXPECT_NEAR(world.GetPosition(box).x, 30.0f, 1.0f) << "what it already touches must not stop it";
}

// ---------------------------------------------------------------------------
// Polygons and capsules

TEST(Physics2DWorld, AShapeWithNoAreaIsRefused)
{
    TestWorld world;
    const BodyHandle body = world.CreateBody(BodyDef{});
    ShapeDef line;
    line.type = ShapeType::Polygon;
    line.points[0] = { 0.0f, 0.0f };
    line.points[1] = { 1.0f, 1.0f };
    line.points[2] = { 2.0f, 2.0f };
    line.pointCount = 3;
    EXPECT_EQ(world.AddShape(body, line), ShapeHandle{});
    ShapeDef none;
    none.halfWidth = 0.0f;
    EXPECT_EQ(world.AddShape(body, none), ShapeHandle{});
    EXPECT_EQ(world.ShapeCount(), 0u);
}

TEST(Physics2DWorld, APolygonInAnyOrderRestsOnItsFlatSide)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    // Given out of order, with one point inside the outline.
    ShapeDef slab;
    slab.type = ShapeType::Polygon;
    slab.points[0] = { 1.0f, 0.5f };
    slab.points[1] = { 0.0f, 0.1f };
    slab.points[2] = { -1.0f, 0.5f };
    slab.points[3] = { -1.0f, -0.5f };
    slab.points[4] = { 1.0f, -0.5f };
    slab.pointCount = 5;
    const BodyHandle body = AddBody(world, BodyType::Dynamic, { 0.0f, 3.5f }, slab);
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(body).y, 4.0f, 0.02f);
    EXPECT_NEAR(world.GetAngle(body), 0.0f, 0.02f);
}

TEST(Physics2DWorld, AnOffCentrePolygonFallsOverItsCentreOfMass)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 0.5f }, 20.0f, 0.5f);
    // Tilted up onto the corner at its origin, with its weight out to the right.
    BodyDef def;
    def.position = { 0.0f, -0.01f };
    def.angle = -0.5f;
    const BodyHandle body = world.CreateBody(def);
    world.AddShape(body, CornerTriangleShape());
    Simulate(world, 5.0f);
    EXPECT_NEAR(world.GetAngle(body), 0.0f, 0.05f) << "down onto its long side";
    EXPECT_NEAR(world.GetPosition(body).y, 0.0f, 0.03f);
}

TEST(Physics2DWorld, AnOffCentrePolygonSpinsAboutItsCentreOfMass)
{
    TestWorld world;
    world.gravity = {};
    const BodyHandle body = world.CreateBody(BodyDef{});
    world.AddShape(body, CornerTriangleShape());
    world.SetAngularVelocity(body, 3.14159265f);
    Simulate(world, 1.0f);
    // Half a turn about (2/3, -1/3) carries the origin to twice that.
    EXPECT_NEAR(world.GetPosition(body).x, 4.0f / 3.0f, 0.02f);
    EXPECT_NEAR(world.GetPosition(body).y, -2.0f / 3.0f, 0.02f);
}

TEST(Physics2DWorld, APositionIsTheOriginWhereverTheShapeIs)
{
    TestWorld world;
    world.gravity = {};
    BodyDef def;
    def.position = { 3.0f, 4.0f };
    def.angle = 0.7f;
    const BodyHandle body = world.CreateBody(def);
    world.AddShape(body, CornerTriangleShape());
    EXPECT_NEAR(world.GetPosition(body).x, 3.0f, 1e-4f) << "adding a shape does not move the object";
    EXPECT_NEAR(world.GetPosition(body).y, 4.0f, 1e-4f);

    world.SetPosition(body, { 1.0f, 2.0f });
    EXPECT_NEAR(world.GetPosition(body).x, 1.0f, 1e-4f);
    EXPECT_NEAR(world.GetPosition(body).y, 2.0f, 1e-4f);

    world.SetAngle(body, 1.5f);
    EXPECT_NEAR(world.GetPosition(body).x, 1.0f, 1e-4f) << "turning by code turns it about its origin";
    EXPECT_NEAR(world.GetPosition(body).y, 2.0f, 1e-4f);
    EXPECT_NEAR(world.GetAngle(body), 1.5f, 1e-4f);

    Simulate(world, 0.5f);
    EXPECT_NEAR(world.GetPosition(body).x, 1.0f, 1e-3f);
    EXPECT_NEAR(world.GetPosition(body).y, 2.0f, 1e-3f);
}

TEST(Physics2DWorld, AHingeAtTheOriginSwingsAnOffCentreBody)
{
    TestWorld world;
    BodyDef def;
    def.linearDamping = 2.0f;  // a pinned bar's swing is mostly its centre moving
    def.angularDamping = 2.0f;
    const BodyHandle body = world.CreateBody(def);
    ShapeDef bar;
    bar.type = ShapeType::Polygon;
    bar.points[0] = { 1.0f, -0.1f };
    bar.points[1] = { 2.0f, -0.1f };
    bar.points[2] = { 2.0f, 0.1f };
    bar.points[3] = { 1.0f, 0.1f };
    bar.pointCount = 4;
    world.AddShape(body, bar);

    HingeDef hinge;
    hinge.bodyA = body;
    hinge.anchor = { 0.0f, 0.0f };
    ASSERT_TRUE(world.IsValidJoint(world.CreateHinge(hinge)));
    Simulate(world, 15.0f);
    EXPECT_NEAR(world.GetPosition(body).x, 0.0f, 0.02f) << "the origin stays on the pin";
    EXPECT_NEAR(world.GetPosition(body).y, 0.0f, 0.02f);
    EXPECT_NEAR(world.GetAngle(body), 1.5707963f, 0.05f) << "the bar hangs straight down";
}

TEST(Physics2DWorld, ACircleLandsOnAPolygonFace)
{
    TestWorld world;
    AddBody(world, BodyType::Static, { 0.0f, 5.0f }, TriangleShape());
    const BodyHandle ball = AddCircle(world, BodyType::Dynamic, { 0.0f, 3.0f }, 0.25f);
    Simulate(world, 0.8f);
    EXPECT_NEAR(world.GetPosition(ball).y, 4.25f, 0.2f) << "on the point of the triangle, which is its top";
}

TEST(Physics2DWorld, ACapsuleLiesFlat)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle capsule = AddBody(world, BodyType::Dynamic, { 0.0f, 3.0f }, CapsuleShape(0.25f, 1.0f));
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(capsule).y, 4.25f, 0.02f);
    EXPECT_NEAR(world.GetAngle(capsule), 0.0f, 0.02f);
    EXPECT_FALSE(world.IsAwake(capsule));
}

TEST(Physics2DWorld, AStandingCapsuleTipsOver)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle capsule =
        AddBody(world, BodyType::Dynamic, { 0.05f, 3.5f }, CapsuleShape(0.25f, 0.8f, 1.5707963f));
    world.SetAngularVelocity(capsule, 0.2f);
    Simulate(world, 4.0f);
    EXPECT_NEAR(world.GetPosition(capsule).y, 4.25f, 0.05f) << "ends up on its side";
}

TEST(Physics2DWorld, CapsulesRestOnEachOther)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle lower = AddBody(world, BodyType::Dynamic, { 0.0f, 4.0f }, CapsuleShape(0.25f, 1.0f));
    const BodyHandle upper = AddBody(world, BodyType::Dynamic, { 0.0f, 3.0f }, CapsuleShape(0.25f, 1.0f));
    Simulate(world, 3.0f);
    EXPECT_NEAR(world.GetPosition(lower).y, 4.25f, 0.03f);
    EXPECT_NEAR(world.GetPosition(upper).y, 3.75f, 0.05f);
}

TEST(Physics2DWorld, ACircleHitsACapsulesSideAndEnd)
{
    TestWorld world;
    world.gravity = {};
    AddBody(world, BodyType::Static, { 0.0f, 0.0f }, CapsuleShape(0.5f, 2.0f));
    const BodyHandle fromAbove = AddCircle(world, BodyType::Dynamic, { 1.0f, -3.0f }, 0.5f);
    world.SetVelocity(fromAbove, { 0.0f, 4.0f });
    const BodyHandle fromEnd = AddCircle(world, BodyType::Dynamic, { 6.0f, 0.0f }, 0.5f);
    world.SetVelocity(fromEnd, { -4.0f, 0.0f });
    Simulate(world, 1.5f);
    EXPECT_NEAR(world.GetPosition(fromAbove).y, -1.0f, 0.02f);
    EXPECT_NEAR(world.GetPosition(fromEnd).x, 3.0f, 0.02f);
}

TEST(Physics2DWorld, RaycastHitsAPolygonAndACapsule)
{
    TestWorld world;
    AddBody(world, BodyType::Static, { 0.0f, 0.0f }, TriangleShape());
    AddBody(world, BodyType::Static, { 10.0f, 0.0f }, CapsuleShape(0.5f, 2.0f));

    RayHit hit;
    ASSERT_TRUE(world.Raycast({ 0.0f, -5.0f }, { 0.0f, 5.0f }, hit));
    EXPECT_NEAR(hit.point.y, -0.5f, 1e-3f);
    ASSERT_TRUE(world.Raycast({ 10.0f, -5.0f }, { 10.0f, 5.0f }, hit));
    EXPECT_NEAR(hit.point.y, -0.5f, 1e-3f);
    EXPECT_NEAR(hit.normal.y, -1.0f, 1e-3f);
    ASSERT_TRUE(world.Raycast({ 20.0f, 0.0f }, { 10.0f, 0.0f }, hit));
    EXPECT_NEAR(hit.point.x, 12.5f, 1e-3f) << "the round end";
    EXPECT_FALSE(world.Raycast({ 12.4f, -5.0f }, { 12.4f, -0.5f }, hit));
}

// ---------------------------------------------------------------------------
// Joints

TEST(Physics2DWorld, AHingeToTheWorldSwingsAndKeepsItsLength)
{
    TestWorld world;
    const BodyHandle bob = AddBox(world, BodyType::Dynamic, { 3.0f, 0.0f }, 0.25f, 0.25f);
    HingeDef hinge;
    hinge.bodyA = bob;
    hinge.anchor = { 0.0f, 0.0f };
    ASSERT_TRUE(world.IsValidJoint(world.CreateHinge(hinge)));
    float lowest = -100.0f;
    for (int i = 0; i < 240; ++i)
    {
        world.Step(1.0f / 60.0f);
        EXPECT_NEAR(Distance(world.GetPosition(bob), { 0.0f, 0.0f }), 3.0f, 0.08f);
        lowest = std::max(lowest, world.GetPosition(bob).y);
    }
    EXPECT_GT(lowest, 2.8f) << "it swung down";
}

TEST(Physics2DWorld, AChainHoldsTogether)
{
    TestWorld world;
    std::vector<BodyHandle> links;
    for (int i = 0; i < 8; ++i)
    {
        BodyDef def;
        def.position = { 0.5f + i * 1.0f, 0.0f };
        def.linearDamping = 0.5f;
        def.angularDamping = 0.5f;
        links.push_back(world.CreateBody(def));
        ShapeDef shape;
        shape.halfWidth = 0.45f;
        shape.halfHeight = 0.1f;
        world.AddShape(links[i], shape);
        HingeDef hinge;
        hinge.bodyA = links[i];
        if (i > 0)
        {
            hinge.bodyB = links[i - 1];
        }
        hinge.anchor = { i * 1.0f, 0.0f };
        world.CreateHinge(hinge);
    }
    auto end = [&](size_t i, float side)
    {
        const Vec2 at = world.GetPosition(links[i]);
        const float angle = world.GetAngle(links[i]);
        return Vec2{ at.x + std::cos(angle) * side * 0.5f, at.y + std::sin(angle) * side * 0.5f };
    };
    for (int step = 0; step < 60 * 20; ++step)
    {
        world.Step(1.0f / 60.0f);
        for (size_t i = 1; i < links.size(); ++i)
        {
            ASSERT_LT(Distance(end(i, -1.0f), end(i - 1, 1.0f)), 0.15f) << "the links stay joined, at step " << step;
        }
    }
    EXPECT_GT(world.GetPosition(links.back()).y, 6.5f) << "it hangs from the pin";
}

TEST(Physics2DWorld, AHingeLimitStopsTheTurn)
{
    TestWorld world;
    const BodyHandle door = AddBox(world, BodyType::Dynamic, { 1.0f, 0.0f }, 1.0f, 0.1f);
    const BodyHandle frame = AddBox(world, BodyType::Static, { -5.0f, 0.0f }, 0.1f, 0.1f);
    HingeDef hinge;
    hinge.bodyA = frame;
    hinge.bodyB = door;
    hinge.anchor = { 0.0f, 0.0f };
    hinge.limited = true;
    hinge.lowerAngle = -0.5f;
    hinge.upperAngle = 0.5f;
    world.CreateHinge(hinge);
    Simulate(world, 4.0f);
    EXPECT_NEAR(world.GetAngle(door), 0.5f, 0.05f) << "gravity is +Y here, so it swings to the upper limit";
}

TEST(Physics2DWorld, AHingeMotorSpinsAtItsSpeed)
{
    TestWorld world;
    world.gravity = {};
    const BodyHandle wheel = AddCircle(world, BodyType::Dynamic, { 0.0f, 0.0f }, 0.5f);
    const BodyHandle axle = AddBox(world, BodyType::Static, { 0.0f, 3.0f }, 0.1f, 0.1f);
    HingeDef hinge;
    hinge.bodyA = axle;
    hinge.bodyB = wheel;
    hinge.anchor = { 0.0f, 0.0f };
    hinge.motor = true;
    hinge.motorSpeed = 3.0f;
    hinge.maxMotorTorque = 50.0f;
    world.CreateHinge(hinge);
    Simulate(world, 1.0f);
    EXPECT_NEAR(world.GetAngularVelocity(wheel), 3.0f, 0.01f);
    EXPECT_NEAR(world.GetPosition(wheel).x, 0.0f, 0.01f);
}

TEST(Physics2DWorld, ADistanceJointKeepsItsLength)
{
    TestWorld world;
    const BodyHandle bob = AddCircle(world, BodyType::Dynamic, { 2.0f, 0.0f }, 0.2f);
    DistanceDef joint;
    joint.bodyA = bob;
    joint.anchorA = { 2.0f, 0.0f };
    joint.anchorB = { 0.0f, 0.0f };
    world.CreateDistance(joint);
    for (int i = 0; i < 180; ++i)
    {
        world.Step(1.0f / 60.0f);
        EXPECT_NEAR(Distance(world.GetPosition(bob), { 0.0f, 0.0f }), 2.0f, 0.08f);
    }
}

TEST(Physics2DWorld, ARopeOnlyPulls)
{
    TestWorld world;
    const BodyHandle bob = AddCircle(world, BodyType::Dynamic, { 0.0f, 0.0f }, 0.2f);
    DistanceDef rope;
    rope.bodyA = bob;
    rope.anchorA = { 0.0f, 0.0f };
    rope.anchorB = { 0.0f, -1.0f };
    rope.length = 3.0f;
    rope.rope = true;
    world.CreateDistance(rope);
    Simulate(world, 0.2f);
    EXPECT_GT(world.GetPosition(bob).y, 0.15f) << "slack: it falls freely";
    Simulate(world, 4.0f);
    EXPECT_LT(Distance(world.GetPosition(bob), { 0.0f, -1.0f }), 3.1f);
    EXPECT_GT(world.GetPosition(bob).y, 1.5f);
}

TEST(Physics2DWorld, JoinedBodiesDoNotCollideUnlessAsked)
{
    TestWorld world;
    world.gravity = {};
    const BodyHandle a = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    const BodyHandle b = AddBox(world, BodyType::Dynamic, { 0.5f, 0.0f });
    HingeDef hinge;
    hinge.bodyA = a;
    hinge.bodyB = b;
    hinge.anchor = { 0.25f, 0.0f };
    world.CreateHinge(hinge);
    int contacts = 0;
    world.onContactBegin = [&](const Contact&) { ++contacts; };
    Simulate(world, 0.5f);
    EXPECT_EQ(contacts, 0);
}

TEST(Physics2DWorld, DestroyingABodyDestroysItsJoints)
{
    TestWorld world;
    const BodyHandle a = AddBox(world, BodyType::Dynamic, { 0.0f, 0.0f });
    const BodyHandle b = AddBox(world, BodyType::Dynamic, { 3.0f, 0.0f });
    HingeDef hinge;
    hinge.bodyA = a;
    hinge.bodyB = b;
    hinge.anchor = { 1.5f, 0.0f };
    const JointHandle joint = world.CreateHinge(hinge);
    world.DestroyBody(b);
    EXPECT_FALSE(world.IsValidJoint(joint));
    EXPECT_EQ(world.JointCount(), 0u);
    const float y = world.GetPosition(a).y;
    Simulate(world, 0.5f);
    EXPECT_GT(world.GetPosition(a).y, y + 0.5f);
}

TEST(Physics2DWorld, JoinedBodiesSleepTogether)
{
    TestWorld world;
    AddBox(world, BodyType::Static, { 0.0f, 5.0f }, 20.0f, 0.5f);
    const BodyHandle a = AddBox(world, BodyType::Dynamic, { -2.0f, 4.0f });
    const BodyHandle b = AddBox(world, BodyType::Dynamic, { 2.0f, 4.0f });
    DistanceDef joint;
    joint.bodyA = a;
    joint.bodyB = b;
    joint.anchorA = { -2.0f, 4.0f };
    joint.anchorB = { 2.0f, 4.0f };
    world.CreateDistance(joint);
    Simulate(world, 4.0f);
    ASSERT_FALSE(world.IsAwake(a));
    ASSERT_FALSE(world.IsAwake(b));
    world.WakeUp(a);
    EXPECT_TRUE(world.IsAwake(b));
}

TEST(Physics2DWorld, AFastBodyStopsAtAWallThatMoves)
{
    // The wall is where it has moved to this step, not where the step began.
    World world;
    world.gravity = { 0.0f, 0.0f };
    BodyDef wallBody;
    wallBody.type = BodyType::Kinematic;
    wallBody.position = { 10.0f, 0.0f };
    wallBody.velocity = { 60.0f, 0.0f };  // a metre a step
    BodyHandle wall = world.CreateBody(wallBody);
    ShapeDef wallShape;
    wallShape.halfWidth = 0.05f;
    wallShape.halfHeight = 2.0f;
    world.AddShape(wall, wallShape);

    BodyDef bulletBody;
    bulletBody.position = { 0.0f, 0.0f };
    bulletBody.velocity = { 600.0f, 0.0f };  // ten metres a step
    BodyHandle bullet = world.CreateBody(bulletBody);
    ShapeDef bulletShape;
    bulletShape.type = ShapeType::Circle;
    bulletShape.radius = 0.1f;
    world.AddShape(bullet, bulletShape);

    for (int i = 0; i < 4; ++i)
    {
        world.Step(1.0f / 60.0f);
        EXPECT_LT(world.GetPosition(bullet).x, world.GetPosition(wall).x + 0.2f) << "step " << i;
    }
}

TEST(Physics2DWorld, TimeUnderAStepIsKeptWhenAFrameRunsTheMostSteps)
{
    // Frames of four and a half steps and three and a half: eight steps a
    // pair, the first of each keeping its half for the second.
    World world;
    world.maxStepsPerFrame = 4;
    int steps = 0;
    for (int frame = 0; frame < 20; ++frame)
    {
        steps += world.Step((frame % 2 == 0 ? 4.5f : 3.5f) / 60.0f);
    }
    EXPECT_EQ(steps, 80);
}

TEST(Physics2DWorld, AFrameFarBehindDropsWhatItCannotRun)
{
    World world;
    world.maxStepsPerFrame = 4;
    EXPECT_EQ(world.Step(1.0f), 4);          // sixty steps asked for
    EXPECT_EQ(world.Step(1.0f / 60.0f), 1);  // none of the rest comes back
}