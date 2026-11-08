# Changelog

Notable changes to `deki-physics-2d`. Engine and editor changes are in the
[engine changelog](https://github.com/dekiengine/deki-engine/blob/master/CHANGELOG.md).

A package's `minEngine` names the engine version it needs. Before 1.0 a
breaking change bumps the minor across the editor, the engine and every
package together, so a package with no changes of its own is still released
alongside one that has them.

## Unreleased

### Added
- The package: rigid bodies that move and turn, box and circle colliders,
  triggers, layers, raycasts and collision and trigger callbacks, on a fixed step so a game plays the same at
  any frame rate.
