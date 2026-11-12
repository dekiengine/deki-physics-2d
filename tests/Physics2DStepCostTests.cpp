// What a step costs, and that it is the same step every time: a few scenes
// stepped twice from the same start must end bit for bit where they ended
// before. Each also prints its time a step and a hash of where every body
// ended, so a change to the solver can be compared before and after:
//   DekiPhysics2DTests --gtest_filter=Physics2DStepCost.*

#include <gtest/gtest.h>

#include <Physics2DWorld.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace DekiPhysics2D;

namespace
{

struct Scene
{
    World world;
    std::vector<BodyHandle> bodies;

    Scene() { world.gravity = { 0.0f, 9.81f }; }  // +Y down, as a screen

    BodyHandle Box(BodyType type, Vec2 position, float halfWidth = 0.5f, float halfHeight = 0.5f)
    {
        BodyDef body;
        body.type = type;
        body.position = position;
        BodyHandle handle = world.CreateBody(body);
        ShapeDef shape;
        shape.halfWidth = halfWidth;
        shape.halfHeight = halfHeight;
        world.AddShape(handle, shape);
        bodies.push_back(handle);
        return handle;
    }

    BodyHandle Circle(Vec2 position, float radius, Vec2 velocity = {})
    {
        BodyDef body;
        body.position = position;
        body.velocity = velocity;
        BodyHandle handle = world.CreateBody(body);
        ShapeDef shape;
        shape.type = ShapeType::Circle;
        shape.radius = radius;
        world.AddShape(handle, shape);
        bodies.push_back(handle);
        return handle;
    }

    // The bits of where every body is and how it moves.
    uint64_t Hash() const
    {
        uint64_t hash = 1469598103934665603ull;
        auto add = [&hash](float value)
        {
            uint32_t bits;
            std::memcpy(&bits, &value, sizeof(bits));
            hash = (hash ^ bits) * 1099511628211ull;
        };
        for (BodyHandle body : bodies)
        {
            add(world.GetPosition(body).x);
            add(world.GetPosition(body).y);
            add(world.GetAngle(body));
            add(world.GetVelocity(body).x);
            add(world.GetVelocity(body).y);
            add(world.GetAngularVelocity(body));
        }
        return hash;
    }
};

// A heap of boxes dropped on the ground: contacts and the solver.
void Pile(Scene& scene, bool sleeps)
{
    scene.world.allowSleep = sleeps;
    scene.Box(BodyType::Static, { 0.0f, 20.0f }, 30.0f, 0.5f);
    scene.Box(BodyType::Static, { -8.0f, 10.0f }, 0.5f, 10.0f);
    scene.Box(BodyType::Static, { 8.0f, 10.0f }, 0.5f, 10.0f);
    for (int i = 0; i < 100; ++i)
    {
        const float x = -6.0f + static_cast<float>(i % 10) * 1.3f + static_cast<float>(i / 10 % 2) * 0.4f;
        scene.Box(BodyType::Dynamic, { x, 18.0f - static_cast<float>(i / 10) * 1.2f }, 0.5f, 0.5f);
    }
}

// Columns of ground under a few movers: the broad phase.
void Columns(Scene& scene)
{
    scene.world.allowSleep = false;
    for (int column = 0; column < 30; ++column)
    {
        for (int row = 0; row < 10; ++row)
        {
            scene.Box(BodyType::Static, { static_cast<float>(column), 10.0f + static_cast<float>(row) }, 0.5f, 0.5f);
        }
    }
    for (int i = 0; i < 10; ++i)
    {
        scene.Circle({ 2.0f + static_cast<float>(i) * 2.7f, 4.0f - static_cast<float>(i % 3) }, 0.4f,
                     { 1.0f - static_cast<float>(i % 2) * 2.0f, 0.0f });
    }
}

// A chain of boxes hung from the world: joints.
void Chain(Scene& scene)
{
    scene.world.allowSleep = false;
    BodyHandle last;
    for (int i = 0; i < 30; ++i)
    {
        BodyHandle link = scene.Box(BodyType::Dynamic, { static_cast<float>(i) * 0.5f, 0.0f }, 0.25f, 0.1f);
        HingeDef hinge;
        hinge.bodyA = link;
        if (i > 0)
        {
            hinge.bodyB = last;
        }
        hinge.anchor = { static_cast<float>(i) * 0.5f - 0.25f, 0.0f };
        scene.world.CreateHinge(hinge);
        last = link;
    }
}

// A fast ball among many shapes: stopping at what it would jump over.
void FastBall(Scene& scene)
{
    scene.world.allowSleep = false;
    scene.world.gravity = { 0.0f, 0.0f };
    for (int i = 0; i < 200; ++i)
    {
        scene.Box(BodyType::Static, { static_cast<float>(i % 20) * 3.0f, static_cast<float>(i / 20) * 3.0f }, 0.3f,
                  0.3f);
    }
    scene.Circle({ 1.5f, 1.5f }, 0.2f, { 90.0f, 70.0f });
}

struct Result
{
    uint64_t hash = 0;
    double microsecondsAStep = 0.0;
};

// Builds the scene, lets it settle, then steps it, timed.
Result Run(void (*build)(Scene&), int warmUpSteps, int steps)
{
    Scene scene;
    build(scene);
    for (int i = 0; i < warmUpSteps; ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    const auto end = std::chrono::steady_clock::now();
    Result result;
    result.hash = scene.Hash();
    result.microsecondsAStep = std::chrono::duration<double, std::micro>(end - start).count() / steps;
    return result;
}

// The best of a few runs, which must all end at the same place.
void Check(const char* name, void (*build)(Scene&), int warmUpSteps, int steps)
{
    const Result first = Run(build, warmUpSteps, steps);
    double best = first.microsecondsAStep;
    for (int i = 0; i < 4; ++i)
    {
        const Result again = Run(build, warmUpSteps, steps);
        EXPECT_EQ(again.hash, first.hash) << name << ": the same steps ended somewhere else";
        best = std::min(best, again.microsecondsAStep);
    }
    std::printf("STEPCOST %-22s %8.2f us a step, ends at %016llx\n", name, best,
                static_cast<unsigned long long>(first.hash));
}

}  // namespace

TEST(Physics2DStepCost, APileOfBoxesAwake)
{
    Check("pile awake", [](Scene& scene) { Pile(scene, false); }, 120, 600);
}

TEST(Physics2DStepCost, APileOfBoxesAsleep)
{
    Check("pile asleep", [](Scene& scene) { Pile(scene, true); }, 1200, 600);
}

TEST(Physics2DStepCost, MoversOverColumnsOfGround)
{
    Check("columns", Columns, 60, 600);
}

TEST(Physics2DStepCost, AChainOfHinges)
{
    Check("chain", Chain, 60, 600);
}

TEST(Physics2DStepCost, AFastBallAmongManyShapes)
{
    Check("fast ball", FastBall, 0, 600);
}

// A world at rest is not stepped; what code changes while it rests still is.
TEST(Physics2DStepCost, AChangeWhileEverythingSleepsIsNotMissed)
{
    Scene scene;
    BodyHandle ground = scene.Box(BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    BodyHandle box = scene.Box(BodyType::Dynamic, { 0.0f, 4.0f });
    for (int i = 0; i < 600 && scene.world.IsAwake(box); ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    ASSERT_FALSE(scene.world.IsAwake(box));
    const float restingAt = scene.world.GetPosition(box).y;

    // Resting: nothing moves, however long.
    for (int i = 0; i < 120; ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    EXPECT_EQ(scene.world.GetPosition(box).y, restingAt);

    // A push wakes it.
    scene.world.AddImpulse(box, { 0.0f, -3.0f });
    scene.world.Step(1.0f / 60.0f);
    EXPECT_LT(scene.world.GetPosition(box).y, restingAt);
    for (int i = 0; i < 600 && scene.world.IsAwake(box); ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    ASSERT_FALSE(scene.world.IsAwake(box));

    // The ground taken away under it: it falls.
    scene.world.DestroyBody(ground);
    for (int i = 0; i < 30; ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    EXPECT_GT(scene.world.GetPosition(box).y, restingAt + 0.5f);
}

TEST(Physics2DStepCost, ATriggerRemovedWhileEverythingSleepsEndsItsContact)
{
    Scene scene;
    scene.Box(BodyType::Static, { 0.0f, 5.0f }, 10.0f, 0.5f);
    BodyHandle box = scene.Box(BodyType::Dynamic, { 0.0f, 4.0f });
    BodyDef zoneBody;
    zoneBody.type = BodyType::Static;
    zoneBody.position = { 0.0f, 4.0f };
    BodyHandle zone = scene.world.CreateBody(zoneBody);
    ShapeDef zoneShape;
    zoneShape.halfWidth = 2.0f;
    zoneShape.halfHeight = 2.0f;
    zoneShape.trigger = true;
    ShapeHandle trigger = scene.world.AddShape(zone, zoneShape);

    int began = 0, ended = 0;
    // The box in the zone (the ground touches the zone too).
    auto boxInZone = [box](const Contact& contact)
    { return contact.trigger && (contact.bodyA == box || contact.bodyB == box); };
    scene.world.onContactBegin = [&](const Contact& contact) { began += boxInZone(contact) ? 1 : 0; };
    scene.world.onContactEnd = [&](const Contact& contact) { ended += boxInZone(contact) ? 1 : 0; };
    for (int i = 0; i < 600 && scene.world.IsAwake(box); ++i)
    {
        scene.world.Step(1.0f / 60.0f);
    }
    ASSERT_FALSE(scene.world.IsAwake(box));
    EXPECT_EQ(began, 1);
    EXPECT_EQ(ended, 0);

    scene.world.RemoveShape(trigger);
    scene.world.Step(1.0f / 60.0f);
    EXPECT_EQ(ended, 1);
}