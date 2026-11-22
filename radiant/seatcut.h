/*
   Seat cut: a hole in a segmented patch (the host) for a segmented cutter that sits in it, which follows the cutter.

   The selected segmented patches become a cutter (patchMesh_t::seatId, the same on every patch of one cutter). The
   hull of all its points is turned into planes, and every other segmented patch that the hull reaches gets those
   planes in patchMesh_t::seat. A patch with planes is drawn and saved without the cells inside them, and clipped
   along them (see SegPatch_Cut in segpatch.h); the cutter itself is never changed, and nothing closes the hole.

   The planes are derived data: whenever a cutter or a segmented patch has moved and has been left alone for a
   moment, all of them are worked out again from the cutters as they are (Seat_Tick). So the hole follows the
   cutter, moving it away gives the host back as it was, and undo needs no steps of its own for it. The
   planes are saved with the host (//@seat) so that the compilers get the cut pieces.
 */

#ifndef __SEATCUT_H__
#define __SEATCUT_H__

// Make the selected segmented patches a cutter (every patch of a cutter that is selected joins that cutter), and cut
bool Seat_MakeSelected();

// The selected patches stop being cutters; the holes they made close
bool Seat_ClearSelected();

// The panel's Cut box: the selected segmented patches become a cutter (or stop being one), each patch on its own
// unless several are selected at once. The state shown is that of the first selected segmented patch.
bool Seat_GetSelected( bool *isCutter, bool *canBeCut );
bool Seat_SetCutterSelected( bool on );

// The panel's Can be cut box: off, and no cutter cuts a hole into the selected patches
bool Seat_SetCanBeCutSelected( bool can );

// The outline of the active cutters, drawn in every view
void Seat_Draw();

// Select every patch of the cutter(s) that have a patch selected. Returns how many patches were added.
int Seat_SelectWhole();

// Work out all the holes again, now (the timer does this on its own after things have moved)
void Seat_Recut();

// Start the timer that does it (once, after the main window exists)
void Seat_Init();

// around a paste / clone: the copies of a cutter get an id of their own
void Seat_ImportBegin();
void Seat_ImportEnd();

#endif
