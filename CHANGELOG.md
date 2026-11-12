# Changelog

Notable changes to `deki-physics-2d`. Engine and editor changes are in the
[engine changelog](https://github.com/dekiengine/deki-engine/blob/master/CHANGELOG.md).

A package's `minEngine` names the engine version it needs. Before 1.0 a
breaking change bumps the minor across the editor, the engine and every
package together, so a package with no changes of its own is still released
alongside one that has them.

## Unreleased

### Added
- The package: rigid bodies that move and turn, box, circle, capsule and convex
  polygon colliders, triggers, layers, raycasts and collision and trigger
  callbacks, on a fixed step so a game plays the same at any frame rate.
- A body falls, balances and spins about the centre of its colliders, wherever
  its object's origin is.
- Hinge joints with limits and a motor, and distance joints and ropes.
- Bodies at rest sleep, with everything touching them, until something wakes
  them.
- Fast bodies stop at what they would otherwise jump over in one step.
- A collider with no Rigidbody 2D follows its object, and a body follows an
  object that other code moves.
- Nothing is allocated while a scene runs, once its lists have reached their
  size.
- Colliders and joints are drawn in the Scene View, with handles to drag a
  selected collider's size and corners.
- A world at rest is not stepped. With no body awake or moving and nothing
  changed from outside since the last step, a step returns at once; it used
  to find every contact again and copy what it kept of each. A settled pile
  of 100 boxes went from 22.7 to 0.2 microseconds a step on a desktop, and
  ends where it did before, bit for bit. Whatever code changes while the
  world rests (a push, a body or a trigger removed) is still stepped.
- A contact keeps its two bodies and the friction between them for the step:
  each solver pass looked the shapes up again and took a square root.
- A body asleep, or moved by code and not spinning, is not given its sine
  and cosine again every step.
- A contact's two points are solved together. Solved one after the other, a
  box set down flat rocked from corner to corner, and a stack of 15 leant and
  fell; a stack of 10 now stands, set down exactly or a little to one side.
  A taller one stands or falls by the rounding of a sum: twenty boxes stand
  on a desktop and fall on an ESP32-S3.
- A bounce gives back the speed the two arrived with, before the step's
  gravity. A ball that keeps all its speed rose 4% higher with every bounce;
  it now comes back to where it fell from.
- Resting bodies settle to the overlap they are allowed, 5 mm. What is left
  of an overlap is worked out from where the two bodies are now, and removed
  in four passes from alternate ends of the contacts, and a body still being
  pushed apart does not go to sleep. A stack of 20 boxes went to sleep 16 mm
  into each box below; it now rests 6 mm in.
