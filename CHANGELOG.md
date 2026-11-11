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
