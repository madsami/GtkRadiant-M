#ifndef __ENTANGLE_H__
#define __ENTANGLE_H__

/*
   Entanglement: copies that follow the object they were made from.

   Make Source marks the selected patches as the source of a new link. Copies made of a source (or of one of its
   instances) with copy / paste / clone are entangled on their own: they become instances of the same link. Make
   Instanced does the same for copies that already exist (select the source and the copies).

   An instance has the shape of its source. Whenever the source has been edited and left alone for a moment, every
   instance is brought to the new shape (control points, and the grid size and segment counts when those changed), at
   the place and in the turn it already has. Moving or turning the source changes nothing at the instances, and moving,
   turning or scaling an instance as a whole is fine. Editing a vertex of an instance is undone (edit the source, or
   Unlink the instance first).

   The link is saved in a //@lnk <id> <role> <inst> <idx> comment line in front of each patch (patchMesh_t::lnkId,
   lnkRole, lnkInst, lnkIdx). The patches are saved as they are, so the compilers see ordinary patches.

   The source is outlined in a colour made from the link's number, strongly; its instances in the same colour, faintly.
   A number marks each source and instance in the views.
 */

// The selected patches become the source of a new link. Returns true when a link was made.
bool Entangle_MakeSource();

// The selection holds the source of a link (a patch of it) and other patches: the other patches become instances of it
// (grouped into objects by their group, or by touching bounds). Returns the number of instances made.
int Entangle_MakeInstanced();

// The selected instances are free again. When the selection holds a source, the whole link is dissolved.
int Entangle_Unlink();

// Select the source of the links that have a patch selected / all patches of those links
int Entangle_SelectSource();
int Entangle_SelectAll();

// Outlines and numbers on / off
void Entangle_ToggleOutlines();
bool Entangle_OutlinesOn();

// drawn in the grid views and the camera
void Entangle_Draw();

// Bring every instance to the shape of its source now (the timer does this on its own)
bool Entangle_Sync();

// The origin of an entangled patch goes along when the whole patch is moved, turned or scaled (called from Patch_Move,
// Patch_ApplyMatrix and Patch_Scale; not when single control points are edited).
void Entangle_FrameMoved( patchMesh_t *p, const vec3_t vMove );
void Entangle_FrameMatrix( patchMesh_t *p, const vec3_t vOrigin, const vec3_t vMatrix[3] );
void Entangle_FrameScaled( patchMesh_t *p, const vec3_t vOrigin, const vec3_t vAmt );

// A new map: forget what was remembered
void Entangle_Reset();

// Start the timer (once, after the main window exists)
void Entangle_Init();

// around a paste / clone: copies of a source or an instance become instances of the same link
void Entangle_ImportBegin();
void Entangle_ImportEnd();

#endif
