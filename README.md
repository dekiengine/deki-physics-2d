# Deki Physics 2D

2D physics for the Deki Engine, sized for microcontrollers: boxes and circles
that move and turn, on a fixed time step.

Part of [Deki Engine](https://github.com/dekiengine/deki-engine).

## Using it

Add a **Rigidbody 2D** and a **Box Collider 2D** or **Circle Collider 2D** to an
object. A collider with no Rigidbody 2D is a static obstacle. Positions are
world metres and +Y is up. A **Physics 2D Settings** component, if
the scene has one, sets gravity and the step rate.

From code:

```cpp
using namespace DekiPhysics2D;

auto* body = GetOwner()->GetComponent<Rigidbody2D>();
body->AddImpulse({ 0.0f, 5.0f });
body->onCollisionBegin = [](Deki::Object* other) { /* landed on, or hit, other */ };
body->onTriggerBegin = [](Deki::Object* other) { /* entered a trigger area */ };

RayHit hit;
Deki::Object* what = nullptr;
if (Raycast({ 0.0f, 0.0f }, { 0.0f, 10.0f }, hit, &what))
{
    // hit.point, hit.normal
}
```

## Limits

- A static collider stays where it was when it started.
- A fast small body can pass through a thin wall in one step: there is no
  continuous collision. Raise the step rate or thicken the wall.
