#ifndef __STRAIGHT_H__
#define __STRAIGHT_H__

/*
   Straight Edit.

   A rotated object (patches with a rotation frame, see rotframe.h, and the entities that were turned with them) is
   awkward to edit: nothing of it sits on the grid. Straight Edit lays it on the grid for as long as you work on it.
   The selected objects are turned round the middle of the selection by the smallest turn that makes them
   axis-aligned (the inverse of their frame, then whichever of the 24 grid orientations is nearest to how they lie
   now, so a cylinder that lies on its side stays on its side and only the rest of the map turns about the vertical
   axis). Every tool and the grid snapping work as usual, and the rest of the map is shown turned the same way,
   grayed out (wire only) and out of reach: it is only drawn that way, none of its numbers change.

   Leaving the mode turns the objects (and anything made in the meantime, which is selected on leaving) back and
   gives them their frame again, so they are exactly where they were, plus whatever was done to them.

   While the mode is on
     - undo history is cleared when it is entered and when it is left
     - saving leaves the mode for a moment (the file is written with the objects in place) and comes back to it
     - the seat cut does not follow the cutters, it is worked out again on leaving
 */

bool Straight_Active();

// the menu command: turn the mode on for the selection, or off
void Straight_Toggle();

// the map is going away (new map, open): forget the mode without turning anything
void Straight_Reset();

// around saving the map: back to the real positions, and into the mode again. Straight_Suspend returns true if the
// mode was on, only then call Straight_Resume.
bool Straight_Suspend();
void Straight_Resume();

// for drawing the rest of the map: a 4x4 matrix for glMultMatrixf (world position -> where it is shown), and the box
// that holds the box mins..maxs after that
void Straight_GhostMatrix( float m[16] );
void Straight_GhostBounds( const vec3_t mins, const vec3_t maxs, vec3_t outMins, vec3_t outMaxs );

// the rest of the map in a grid view (nDim1, nDim2: the world axes of the view; vmins, vmaxs: the part shown)
void Straight_DrawGhostsXY( int viewType, int nDim1, int nDim2, const float *vmins, const float *vmaxs );

#endif
