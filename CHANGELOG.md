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
  box set down flat rocked from corner to corner, and a stack leant and fell.
- A bounce gives back the speed the two arrived with, before the step's
  gravity. A ball that keeps all its speed rose 4% higher with every bounce;
  it now comes back to where it fell from.
- A step is solved in sub-steps (`World::subSteps`, 4): each moves the
  bodies a little and solves from where they then are, so the weight of the
  top of a stack reaches the ground within the step. Several passes over one
  whole step carried it a box a pass, and a stack of more than ten stood or
  fell by the rounding of a sum: twenty boxes stood on a desktop and fell on
  an ESP32-S3. Now ten stand however they are set down, twenty with six
  sub-steps, the same on both. On a T-Deck a step of 30 boxes awake takes
  2.1 ms, where it took 1.7.
- An overlap is pushed out as a spring would, and the speed that takes is
  taken back in the same sub-step, so it moves the bodies and adds no
  energy. Bodies rest 2.5 to 5 mm into each other; a stack of 20 went to
  sleep 16 mm into each box below.
- Shapes within 5 mm of each other already have a contact, which lets them
  close the gap and no more. Boxes set down exactly touching had none until
  they had sunk into each other, a box a step: the top of a stack of 20 fell
  half a metre. Nothing is reported, and a trigger is not entered, until two
  shapes overlap.
- A joint is set up from where its bodies are in each sub-step. A pendulum
  keeps 97% of its swing over 20 seconds; it kept 90%.
