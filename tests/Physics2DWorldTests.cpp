// The physics world on its own: what a game can rely on it to do.

#include <gtest/gtest.h>

#include <Physics2DWorld.h>

#include <cmath>
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
    shape.isCircle = true;
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
    shape.isCircle = true;
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
