#ifndef __PATCHGROUP_H__
#define __PATCHGROUP_H__

/*
   Patch groups.

   A group ties a number of patches (segmented cylinders, bevels, caps, anything made of patches) together so
   they are picked, deselected and moved as one object. Nothing is merged: every patch keeps its own control
   points and shape, so patches that Merge Patches cannot join (rotated 90 degrees, other segment counts, other
   topology) can be grouped just the same. The group is the number patchMesh_t::groupId, saved in a //@grp
   comment line in front of each patch (the compilers ignore it). It adds nothing to the map output.

   Picking a patch of a group (camera, grid, or an Alt+drag rectangle that touches it) selects all of them;
   deselecting one deselects all of them. Merge Selection makes a group of the selected patches, Split Group
   dissolves the groups the selected patches are in.
 */

// Merge Selection: the selected patches become one group. Returns the number of patches grouped.
int PatchGroup_Merge();

// Split Group: every patch of the groups that have a patch selected leaves its group. Returns how many did.
int PatchGroup_Split();

// Select Whole Group: add the rest of the group of every selected patch to the selection. Returns how many
// patches were added.
int PatchGroup_SelectWhole( bool bPrint = false );

// A patch of a group was just deselected by a click: deselect the rest of its group too.
void PatchGroup_DeselectWhole( brush_t *brush );

// around a paste / clone: the copies of a group get an id of their own
void PatchGroup_ImportBegin();
void PatchGroup_ImportEnd();

#endif
