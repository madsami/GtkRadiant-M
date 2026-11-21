#ifndef __COG_H__
#define __COG_H__

/*
   Cogs.

   A cog is a profile (one tooth repeated "teeth" times round an axis) extruded along that axis and closed
   with a cap at each end. It is made of ordinary patches, one per face, so the faces keep their crisp corners,
   can have a shader each, and the patch tools (Alt+click edge picking, Bevel Edge) work on them:

     - a wall for every edge of the profile (tip, flank, root gap, flank), 3 x 3 control points
     - on each cap the top of every tooth (one quad), and a fan of triangles from the middle to the root polygon
       (the base of every tooth and every gap): 10 patches per tooth in all
     - with a hole through the middle: the fan becomes a ring of pieces out to the edge of the hole, and the
       bore gets a wall for every piece (12 per tooth)

   Every patch carries the cog's numbers (patchMesh_t::cogId and ::cog), saved as //@cog comment lines in the map
   file (they are ignored by the compilers), so a cog can be changed again after saving and loading.

   Extrude: the cylinder radius is the root and the teeth stand out of it. Notch: the cylinder radius is the tip
   and the gaps are cut into it.
 */

struct cogRec_t
{
	int teeth;
	int mode;               // 0 extrude, 1 notch
	float cylR;             // radius of the cylinder the cog was made from
	float depth;            // how far the tip is from the root
	float taper;            // how much the tooth narrows towards the tip, in percent of its base width (0 = parallel sides)
	float snap;             // every control point is rounded to a multiple of this (0 = not snapped)
	int tipArc;             // the tip is an arc of the circle instead of a flat chord
	int rootArc;            // the same for the root
	float hole;             // radius of a hole through the middle (0 = a solid cog)
	float height;           // length along the axis
	float C[3];             // middle of the cog
	float A[3];             // axis (unit)
	float E1[3];            // direction of the middle of tooth 0 (unit, across the axis)
	// kept for rounding and rim bevel, not used yet
	float tipRound, rootRound;
	int roundSeg;
	float rimBevel;
	int rimSub;
	// how much higher (+) or lower (-) the middle of each cap is than its edge: from the middle (or the edge of the
	// hole) out to the tips of the teeth the cap runs in a fixed slope, at both ends of the cog
	float cone;             // the end on the +axis side
	float cone2;            // the end on the other side
};

void Cog_Pack( const cogRec_t &r, float *out );             // out: COG_NPARAM floats
void Cog_Unpack( const float *in, cogRec_t &r );

// Make Cog: the selected cylinder (a ring of patch control points, segmented or not) becomes a cog, and the
// cog is selected. Returns false (with a message) if the selection is not a cylinder.
bool Cog_MakeFromSelected();

// The record of the selected cog (the first selected patch that belongs to one); false if there is none.
// The middle of the cog follows it if it was moved since it was made.
bool Cog_Selected( cogRec_t *out, int *id );

// Select every patch of the cog(s) that have a patch selected (and the bevel strips made on them), so a whole
// cog can be moved from a single picked face. Returns how many patches were added to the selection.
int Cog_SelectWhole();

// Make the selected cog again with other numbers (its patches are replaced, with one undo step).
bool Cog_Rebuild( const cogRec_t &r );

// the sizes the panel needs: the radii of tip and root, and the room left in a gap, for the label
void Cog_Describe( const cogRec_t &r, char *text, int len );

// around a paste / clone: the copies of a cog get an id of their own
void Cog_ImportBegin();
void Cog_ImportEnd();

#endif
