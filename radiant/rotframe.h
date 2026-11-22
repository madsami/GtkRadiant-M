#ifndef __ROTFRAME_H__
#define __ROTFRAME_H__

/*
   Rotation frames.

   A patch that has been turned off the grid by a rotation remembers by how much: patchMesh_t::hasFrame and ::frameR
   (a 3x3 rotation, row by row, "world = R * straight"). The rotation itself is nothing special, it moves the control
   points exactly as before and they are never snapped. The frame is what a later "straight edit" view needs: turning
   the world by the inverse of R puts the object back on the grid it was built on. It is saved in a //@frm comment
   line in front of the patch (the compilers ignore it), and the clipboard format carries it too.

   Moving an object does not change its frame, only turning it does. A mirror clears it, because a mirrored
   object has no rotation that takes it back.

   Things that belong to the object and are not control points are turned along with it:
     - the axis and the middle of a cog (cog.h), so a rotated cog can still be changed in its panel
     - the facing ( "angle" / "angles" ) and the origin of the entities that are selected with it; an origin is
       written with its decimals, not cut down to whole units
 */

// A whole patch has just been turned by M (3x3, new = M * (old - origin) + origin): update its frame and its cog numbers.
void RotFrame_PatchRotated( patchMesh_t *p, const vec3_t origin, const vec3_t M[3] );

// The patch has been mirrored or flipped: it has no frame any more.
void RotFrame_PatchMirrored( patchMesh_t *p );

// The entity (a point entity, whose origin key was just turned by M) turns to face the new way.
void RotFrame_EntityRotated( entity_t *e, const vec3_t M[3] );

// the pivot of the mouse rotation, drawn as three axis lines in the camera view
void RotFrame_DrawPivot();

// "x y z" with the decimals that matter (up to 3, trailing zeros dropped), for the origin key of an entity
void RotFrame_FormatVec( char *out, const vec3_t v );

#endif
