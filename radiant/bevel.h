#ifndef __BEVEL_H__
#define __BEVEL_H__

/*
   Bevel from edge.

   An "edge" is a patch border: one of the four outer rows / columns of control points of a patch.
   Another patch (b) may continue from it, with a border that has the same control points; b is NULL
   for a free edge. side numbers the border of a patch:
   0 = the column i = 0, 1 = the column i = width - 1, 2 = the row j = 0, 3 = the row j = height - 1.
 */
struct bevelEdge_t
{
	brush_t *a;
	int sideA;
	brush_t *b;
	int sideB;
	bool reversed;      // the border of b runs the other way round
	float ratio;        // 0 (or 1): both patches are cut by the same width; else the cut of b is ratio times the cut of a
};

#endif
