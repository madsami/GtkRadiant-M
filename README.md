GtkRadiant (custom 1.6.71)
==========================

A customised build of [GtkRadiant](https://github.com/TTimo/GtkRadiant) 1.6.7 (the `1.6-release` branch), made for
level editing in Star Wars Jedi Knight: Jedi Academy and used on Windows. It keeps the classic GtkRadiant
interface and controls; it is not a rewrite.

What is different
-----------------

**Camera view and grid**, inspired by [NetRadiant Custom](https://github.com/garux/netradiant-custom) and
re-implemented in GtkRadiant's own GTK2 / OpenGL code:

- Camera: multisampled anti-aliasing, a headlight plus "sun" lighting rig with adjustable levels, a thin wireframe
  overlay on the textured view (also visible through transparent brushes), and smoother, frame-rate independent
  camera movement. WASD flying while in freelook.
- Grid: NetRadiant-style major / minor grid lines that follow the zoom level, a 0.125 grid size (and 512), a deeper
  maximum zoom, and numpad keys that mirror the number-row grid shortcuts.
- Texture panel: transparent shaders get a dotted outline.
- Camera options are in the Camera page of the preferences; the wireframe overlay can also be toggled from the View menu.

**Improved cylinders**, in the Curve menu: Improved Cylinder, End cap, Bevel and Cone. They build rounder patch
cylinders than the stock ones, with corner points snapped to the grid.

Also new: a magenta application icon.

Building
--------

The build is the same as upstream's. On Windows it builds with Visual Studio 2022 (`radiant.sln`); see the
[upstream README](https://github.com/TTimo/GtkRadiant#how-to-build) and its
[Windows build guide](https://icculus.org/gtkradiant/documentation/windows_compile_guide/) for the dependencies.

License
-------

GPL, like upstream; see [LICENSE](LICENSE). Thanks to TTimo and the GtkRadiant contributors, and to garux and the
NetRadiant Custom contributors for the ideas.
