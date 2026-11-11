# Deki Physics 2D

2D physics for the Deki Engine, sized for microcontrollers: boxes, circles,
capsules and convex polygons that move and turn, joints, and no allocation
once a scene is running, on a fixed time step.

Part of [Deki Engine](https://github.com/dekiengine/deki-engine).

## Using it

Add a **Rigidbody 2D** and a **Box**, **Circle**, **Capsule** or **Polygon
Collider 2D** to an object. A collider with no Rigidbody 2D is a static
obstacle, which follows its object if something moves it. Positions are world
metres and +Y is up. A **Physics 2D Settings** component, if the scene has one,
sets gravity, the step rate and whether bodies may sleep.

A **Hinge Joint 2D** pins a body to another or to the world at one point, with
optional limits and a motor. A **Distance Joint 2D** keeps two points a set
distance apart, or as a rope stops them moving further apart. Each goes on an
object that has a Rigidbody 2D.

Colliders and joints are drawn in the Scene View. Select an object to drag the
handles on its colliders.

A body that has come to rest, with everything touching it, goes to sleep and
costs nothing until something touches it, a force reaches it, or its object is
moved. A fast body stops at what it would otherwise jump over in one step.

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

- A polygon has at most 8 corners and is always convex: a dent is filled in.
- Moving an object with a collider places it there, without pushing through
  what is in the way: a moved static collider or teleported body overlaps
  whatever it lands on until the solver pushes that out.
- A fast body can still pass through a trigger without reporting it, and a
  static or kinematic body is never swept.
- Changing a collider's size at runtime takes effect only through the object's
  scale, and only on a collider with no Rigidbody 2D.
