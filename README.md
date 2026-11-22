GtkRadiant (custom 1.6.71)
==========================

A customised build of [GtkRadiant](https://github.com/TTimo/GtkRadiant) 1.6.7 (the `1.6-release` branch), made for
level editing in Star Wars Jedi Knight: Jedi Academy and used on Windows. It keeps the classic GtkRadiant
interface and controls; it is not a rewrite.

AI disclosure
-------------

All of the changes in this fork were written by Claude (Anthropic's AI model) under my direction. I chose what to
build and tested the results; the code itself was produced with AI assistance.

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

**Patchwork tools**, for maps built almost entirely from patches (most are in the Curve menu, many have a toolbar button):

- Segmented patches: a patch can have a chosen number of segments across and along it, so circles are exact and a
  tessellation is yours to set. Thicken Segmented makes a solid wall from a surface. Segment counts are kept in the
  map file as `//@seg` comment lines, so other compile tools that ignore comments still read the map.
- Caps and cones: Cap Segmented (top / bottom) closes a cylinder; a small panel sets the cone height and an inner
  radius, so the cap can be a hole (a donut) instead of a filled disc.
- Bevel Edge: Alt+click patch borders (double-click to follow a whole loop, Alt+drag to box-select), then set the
  width, the subdivisions and a snap-to-grid in the Bevel panel. A bevel stays adjustable: its panel comes back when you
  select the strip, it follows the model when it is moved or rotated, and it is recognised again from its shape after a
  load, an undo or an edit of a neighbour (Curve > Recognize Bevel does it by hand). Inner corners are mitred and capped.
- Bridge Edge Loops: pick two edge loops with the same number of points and fill the ring between them with patches.
  With the same two loops picked, the Bevel button makes a rounded strip between them instead.
- Make Cog: turns a cylinder into a cog (teeth inside or outside, taper, hole, sloped ends, bevelled rims) and makes a
  matching socket, all editable in a panel. Select Whole Cog picks every piece.
- Merge Selection (patch groups): tie patches into one piece that is picked and moved together; Ctrl+Alt+click picks a
  single patch of a group.
- Seat cut: a hole is cut into a segmented patch where a segmented cutter touches it, and follows the cutter when it moves.
- Rotation: rotating a patch object is permanent and never snaps its points; the object remembers how far it was turned.
  Straight Edit shows it straightened on the grid so you can keep editing it on the grid, then puts it back at its tilt.
  Entities inside a rotated object turn with it. F centres the views on the selection.
- Console: Ctrl+C copies the selected console text; crashes write a report instead of just closing.

Also new: a magenta application icon.

Building
--------

The build is the same as upstream's. On Windows it builds with Visual Studio 2022 (`radiant.sln`); see the
[upstream README](https://github.com/TTimo/GtkRadiant#how-to-build) and its
[Windows build guide](https://icculus.org/gtkradiant/documentation/windows_compile_guide/) for the dependencies.

License
-------

GPL, like upstream; see [LICENSE](LICENSE) (GtkRadiant mixes GPL, LGPL and BSD licensed parts, each file says which).
Forks are welcome. The same licenses apply to any fork, which means keeping the source available and the
copyright notices intact.

Thanks to TTimo and the GtkRadiant contributors, and to garux and the
NetRadiant Custom contributors for the ideas.
