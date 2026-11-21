/*
   Bevel from edge: finding and picking the seams between patches (see bevel.h).
 */
#include "stdafx.h"
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <math.h>
#include "bevel.h"
#include "segpatch.h"

#define BEVEL_PICK_EPSILON 8        // pick radius in pixels, like the vertex / edge picking

void WINAPI Patch_Naturalize( patchMesh_t *p );

static std::vector<bevelEdge_t> g_bevelEdges;

struct bevelPt_t
{
	float v[3];
};

static bool Bevel_BrushLive( brush_t *b ){
	if ( b == NULL ) {
		return false;
	}
	brush_t *p;
	for ( p = active_brushes.next; p != &active_brushes; p = p->next )
		if ( p == b ) {
			return b->patchBrush && b->pPatch;
		}
	for ( p = selected_brushes.next; p != &selected_brushes; p = p->next )
		if ( p == b ) {
			return b->patchBrush && b->pPatch;
		}
	return false;
}

// the control points along one border, in order of increasing index
static int Bevel_Rim( const patchMesh_t *p, int side, bevelPt_t *out ){
	int n = ( side < 2 ) ? p->height : p->width;
	for ( int k = 0; k < n; k++ )
	{
		const float *c = side == 0 ? p->ctrl[0][k].xyz
						 : side == 1 ? p->ctrl[p->width - 1][k].xyz
						 : side == 2 ? p->ctrl[k][0].xyz
						 : p->ctrl[k][p->height - 1].xyz;
		VectorCopy( c, out[k].v );
	}
	return n;
}

static bool Bevel_RimDegenerate( const bevelPt_t *a, int n ){
	for ( int k = 1; k < n; k++ )
	{
		vec3_t d;
		VectorSubtract( a[k].v, a[0].v, d );
		if ( VectorLength( d ) > 0.5f ) {
			return false;
		}
	}
	return true;
}

static bool Bevel_RimsMatchTol( const bevelPt_t *a, const bevelPt_t *b, int n, float tol, bool *reversed ){
	for ( int dir = 0; dir < 2; dir++ )
	{
		bool ok = true;
		for ( int k = 0; k < n && ok; k++ )
		{
			vec3_t d;
			VectorSubtract( a[k].v, b[dir ? n - 1 - k : k].v, d );
			if ( VectorLength( d ) > tol ) {
				ok = false;
			}
		}
		if ( ok ) {
			*reversed = dir == 1;
			return true;
		}
	}
	return false;
}

static bool Bevel_RimsMatch( const bevelPt_t *a, const bevelPt_t *b, int n, bool *reversed ){
	return Bevel_RimsMatchTol( a, b, n, 0.5f, reversed );
}

// every patch that can take part: the selected ones and the rest of the map
static void Bevel_Patches( std::vector<brush_t*> &out ){
	brush_t *p;
	for ( p = selected_brushes.next; p != &selected_brushes; p = p->next )
		if ( p->patchBrush && p->pPatch ) {
			out.push_back( p );
		}
	for ( p = active_brushes.next; p != &active_brushes; p = p->next )
		if ( p->patchBrush && p->pPatch ) {
			out.push_back( p );
		}
}

// The box round the control points of a patch. The mins / maxs of the brush are not used for this: for a
// patch they come from the stand-in brush it is built on, which does not always enclose the control points.
static void Bevel_PatchBounds( const patchMesh_t *p, float *mn, float *mx ){
	mn[0] = mn[1] = mn[2] = 99999.0f;
	mx[0] = mx[1] = mx[2] = -99999.0f;
	for ( int w = 0; w < p->width; w++ )
		for ( int h = 0; h < p->height; h++ )
			for ( int j = 0; j < 3; j++ )
			{
				const float f = p->ctrl[w][h].xyz[j];
				if ( f < mn[j] ) {
					mn[j] = f;
				}
				if ( f > mx[j] ) {
					mx[j] = f;
				}
			}
}

// the other patch that has a border on top of this one, if any
static bool Bevel_FindPartnerTol( brush_t *A, int sa, brush_t **outB, int *outSb, bool *outRev, float tol ){
	std::vector<brush_t*> list;
	Bevel_Patches( list );
	bevelPt_t ra[MAX_PATCH_WIDTH], rb[MAX_PATCH_WIDTH];
	int na = Bevel_Rim( A->pPatch, sa, ra );
	const float margin = tol > 1.0f ? tol : 1.0f;
	float amin[3], amax[3];
	Bevel_PatchBounds( A->pPatch, amin, amax );
	for ( size_t j = 0; j < list.size(); j++ )
	{
		brush_t *B = list[j];
		if ( B == A ) {
			continue;
		}
		float bmin[3], bmax[3];
		Bevel_PatchBounds( B->pPatch, bmin, bmax );
		int k;
		for ( k = 0; k < 3; k++ )
			if ( amin[k] > bmax[k] + margin || bmin[k] > amax[k] + margin ) {
				break;
			}
		if ( k < 3 ) {
			continue; // too far apart to touch
		}
		for ( int sb = 0; sb < 4; sb++ )
		{
			int nb = Bevel_Rim( B->pPatch, sb, rb );
			bool rev;
			if ( na == nb && Bevel_RimsMatchTol( ra, rb, na, tol, &rev ) ) {
				*outB = B;
				*outSb = sb;
				*outRev = rev;
				return true;
			}
		}
	}
	return false;
}

static bool Bevel_FindPartner( brush_t *A, int sa, brush_t **outB, int *outSb, bool *outRev ){
	return Bevel_FindPartnerTol( A, sa, outB, outSb, outRev, 0.5f );
}

// the border as a polyline: every span of three control points is a quadratic Bezier curve
static void Bevel_Polyline( const bevelPt_t *r, int n, std::vector<bevelPt_t> &out ){
	const int steps = 8;
	for ( int k = 0; k + 2 < n; k += 2 )
	{
		for ( int s = ( k == 0 ? 0 : 1 ); s <= steps; s++ )
		{
			float t = (float)s / steps, u = 1.0f - t;
			bevelPt_t q;
			for ( int c = 0; c < 3; c++ )
				q.v[c] = u * u * r[k].v[c] + 2 * u * t * r[k + 1].v[c] + t * t * r[k + 2].v[c];
			out.push_back( q );
		}
	}
}

// the border as it is drawn: the sampled outline of a segmented patch, else the curve of its control points
static void Bevel_Border( patchMesh_t *p, int side, std::vector<bevelPt_t> &out ){
	int U, V;
	if ( Patch_SegGridSize( p, &U, &V ) ) {
		const int n = side < 2 ? V : U;
		for ( int k = 0; k < n; k++ )
		{
			const float *g = side == 0 ? Patch_SegGridXYZ( p, 0, k )
							 : side == 1 ? Patch_SegGridXYZ( p, U - 1, k )
							 : side == 2 ? Patch_SegGridXYZ( p, k, 0 )
							 : Patch_SegGridXYZ( p, k, V - 1 );
			if ( g ) {
				bevelPt_t q;
				VectorCopy( g, q.v );
				out.push_back( q );
			}
		}
		if ( out.size() > 1 ) {
			return;
		}
		out.clear();
	}
	bevelPt_t rim[MAX_PATCH_WIDTH];
	const int n = Bevel_Rim( p, side, rim );
	Bevel_Polyline( rim, n, out );
}

// distance of the ray from a segment; t is how far along the ray the closest point is
static bool Bevel_RaySegment( const vec3_t o, const vec3_t d, const float *p0, const float *p1, float *dist, float *t ){
	vec3_t e, w0;
	VectorSubtract( p1, p0, e );
	VectorSubtract( o, p0, w0 );
	const float a = DotProduct( d, d ), b = DotProduct( d, e ), c = DotProduct( e, e );
	const float dd = DotProduct( d, w0 ), ee = DotProduct( e, w0 );
	if ( a < 1e-12f ) {
		return false;
	}
	const float den = a * c - b * b;
	float s = den > 1e-12f ? ( a * ee - b * dd ) / den : 0.0f;
	s = s < 0 ? 0 : ( s > 1 ? 1 : s );
	const float tt = ( s * b - dd ) / a;
	if ( tt < 0 ) {
		return false; // behind the viewer
	}
	vec3_t onRay, onSeg, diff;
	VectorMA( o, tt, d, onRay );
	VectorMA( p0, s, e, onSeg );
	VectorSubtract( onRay, onSeg, diff );
	*dist = VectorLength( diff );
	*t = tt;
	return true;
}

static bool Bevel_SameEdge( const bevelEdge_t &x, const bevelEdge_t &y ){
	return x.a == y.a && x.sideA == y.sideA && x.b == y.b && x.sideB == y.sideB;
}

void BevelEdge_Clear(){
	if ( !g_bevelEdges.empty() ) {
		g_bevelEdges.clear();
		Sys_UpdateWindows( W_XY | W_CAMERA );
	}
}

int BevelEdge_Count(){
	return (int)g_bevelEdges.size();
}

// =============================================================================
// Edge loops
//
// A picked edge can be followed round to the rest of its loop. The rule is the one of a regular quad
// mesh: where an edge meets a vertex that has four edges (so four patches meet there), the loop goes
// straight on into the opposite edge, the one that shares no patch with the edge it came along. A free
// edge goes on into the one other free edge of the vertex, when that belongs to another patch (so the
// outline of a row of patches is one loop, but a patch's own corner is not followed round). Anywhere
// else (three or five edges, a T-junction in the middle of a border) the loop stops. A border that
// closes on itself, like the ring of a cylinder, is a loop already.
//
// This is only run when an edge is double clicked (Alt+click twice on the same edge), and only when the
// preference "Alt+click follows edge loops" is on. A single click picks the one edge, exactly as before.
// Patches that were made separately seldom meet exactly, so here points count as the same when they are
// within BEVEL_LOOP_TOL of each other, a little more than the pick itself allows.
// =============================================================================

#define BEVEL_LOOP_MAX 4096
#define BEVEL_LOOP_TOL 1.5f

static bool Bevel_SamePt( const float *a, const float *b ){
	vec3_t d;
	VectorSubtract( a, b, d );
	return VectorLength( d ) <= BEVEL_LOOP_TOL;
}

// the edge as it is stored: a patch border and the border it shares with another patch, always the same way round
static bevelEdge_t Bevel_MakeEdge( brush_t *A, int side ){
	bevelEdge_t e = { A, side, NULL, 0, false };
	if ( Bevel_FindPartnerTol( A, side, &e.b, &e.sideB, &e.reversed, BEVEL_LOOP_TOL ) && e.b < e.a ) {
		bevelEdge_t s = { e.b, e.sideB, e.a, e.sideA, e.reversed };
		e = s;
	}
	return e;
}

// the two ends of the border of the first patch of an edge
static void Bevel_Ends( const bevelEdge_t &e, float *p0, float *p1 ){
	bevelPt_t rim[MAX_PATCH_WIDTH];
	const int n = Bevel_Rim( e.a->pPatch, e.sideA, rim );
	VectorCopy( rim[0].v, p0 );
	VectorCopy( rim[n - 1].v, p1 );
}

// every edge that has an end at the point v
static void Bevel_EdgesAt( const std::vector<brush_t*> &patches, const float *v, std::vector<bevelEdge_t> &out ){
	bevelPt_t rim[MAX_PATCH_WIDTH], other[MAX_PATCH_WIDTH];
	for ( size_t i = 0; i < patches.size(); i++ )
	{
		brush_t *b = patches[i];
		float bmin[3], bmax[3];
		Bevel_PatchBounds( b->pPatch, bmin, bmax );
		int k;
		for ( k = 0; k < 3; k++ )
			if ( v[k] < bmin[k] - BEVEL_LOOP_TOL || v[k] > bmax[k] + BEVEL_LOOP_TOL ) {
				break;
			}
		if ( k < 3 ) {
			continue;
		}
		patchMesh_t *p = b->pPatch;
		for ( int side = 0; side < 4; side++ )
		{
			const int n = Bevel_Rim( p, side, rim );
			if ( Bevel_RimDegenerate( rim, n ) ) {
				continue;
			}
			if ( side == 1 || side == 3 ) {
				// a ring: the last column / row is the first one again
				bool rev;
				const int n0 = Bevel_Rim( p, side - 1, other );
				if ( n0 == n && Bevel_RimsMatch( rim, other, n, &rev ) && !rev ) {
					continue;
				}
			}
			if ( !Bevel_SamePt( rim[0].v, v ) && !Bevel_SamePt( rim[n - 1].v, v ) ) {
				continue;
			}
			bevelEdge_t e = Bevel_MakeEdge( b, side );
			bool known = false;
			for ( size_t j = 0; j < out.size() && !known; j++ )
				known = Bevel_SameEdge( out[j], e );
			if ( !known ) {
				out.push_back( e );
			}
		}
	}
}

static bool Bevel_EdgeHasPatch( const bevelEdge_t &e, const brush_t *p ){
	return e.a == p || e.b == p;
}

// do the two edges have a patch in common
static bool Bevel_EdgesShare( const bevelEdge_t &x, const bevelEdge_t &y ){
	return Bevel_EdgeHasPatch( y, x.a ) || ( x.b != NULL && Bevel_EdgeHasPatch( y, x.b ) );
}

// the direction an edge leaves the point v in, along its border; false if it has no length
static bool Bevel_LeaveDir( const bevelEdge_t &e, const float *v, float *out ){
	bevelPt_t rim[MAX_PATCH_WIDTH];
	const int n = Bevel_Rim( e.a->pPatch, e.sideA, rim );
	const bool atStart = Bevel_SamePt( rim[0].v, v );
	for ( int k = 1; k < n; k++ )
	{
		const float *q = rim[atStart ? k : n - 1 - k].v;
		vec3_t d;
		VectorSubtract( q, v, d );
		const float len = VectorLength( d );
		if ( len > 0.5f ) {
			VectorScale( d, 1.0f / len, out );
			return true;
		}
	}
	return false;
}

// ---- the rim of a flat face -------------------------------------------------------------------------
// Where the four edge rule cannot decide (a corner where three or five edges meet, as in a flat cap made of
// a fan of triangles and some quads), an edge between a flat face and something else carries on along the
// rest of the rim of that face. A flat face is the patches that lie in one plane and touch each other.

struct bevelPlane_t
{
	bool ok;                    // the patch is flat
	float n[3], p0[3];          // its plane
	float mn[3], mx[3];         // its box
	float area;
};

static void Bevel_PlaneOf( const patchMesh_t *p, bevelPlane_t &o ){
	o.ok = false;
	o.area = 0;
	Bevel_PatchBounds( p, o.mn, o.mx );
	const float *a = p->ctrl[0][0].xyz, *b = a, *c = a;
	float best = 0;
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			vec3_t d;
			VectorSubtract( p->ctrl[i][j].xyz, a, d );
			if ( VectorLength( d ) > best ) {
				best = VectorLength( d );
				b = p->ctrl[i][j].xyz;
			}
		}
	if ( best < 1.0f ) {
		return;
	}
	vec3_t ab, nrm = { 0, 0, 0 };
	VectorSubtract( b, a, ab );
	float bestCross = 0;
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			vec3_t d, cr;
			VectorSubtract( p->ctrl[i][j].xyz, a, d );
			CrossProduct( ab, d, cr );
			if ( VectorLength( cr ) > bestCross ) {
				bestCross = VectorLength( cr );
				c = p->ctrl[i][j].xyz;
				VectorCopy( cr, nrm );
			}
		}
	if ( bestCross < 0.5f * best ) {
		return; // all on a line
	}
	VectorScale( nrm, 1.0f / bestCross, nrm );
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			vec3_t d;
			VectorSubtract( p->ctrl[i][j].xyz, a, d );
			if ( fabsf( DotProduct( d, nrm ) ) > 0.5f ) {
				return; // not flat
			}
		}
	for ( int i = 0; i + 1 < p->width; i++ )
		for ( int j = 0; j + 1 < p->height; j++ )
		{
			vec3_t d1, d2, cr;
			VectorSubtract( p->ctrl[i + 1][j + 1].xyz, p->ctrl[i][j].xyz, d1 );
			VectorSubtract( p->ctrl[i][j + 1].xyz, p->ctrl[i + 1][j].xyz, d2 );
			CrossProduct( d1, d2, cr );
			o.area += 0.5f * VectorLength( cr );
		}
	VectorCopy( nrm, o.n );
	VectorCopy( a, o.p0 );
	o.ok = true;
}

struct bevelFace_t
{
	bool built, have;
	std::set<brush_t*> group;   // the patches of the flat face the loop runs along the rim of
};

static void Bevel_BuildFace( const std::vector<brush_t*> &patches, const bevelEdge_t &start, bevelFace_t &f ){
	f.built = true;
	f.have = false;
	f.group.clear();
	std::vector<bevelPlane_t> pl( patches.size() );
	for ( size_t i = 0; i < patches.size(); i++ )
		Bevel_PlaneOf( patches[i]->pPatch, pl[i] );

	float bestArea = 0;
	std::set<brush_t*> best;
	brush_t *sides[2] = { start.a, start.b };
	for ( int s = 0; s < 2; s++ )
	{
		if ( sides[s] == NULL ) {
			continue;
		}
		size_t si = patches.size();
		for ( size_t i = 0; i < patches.size(); i++ )
			if ( patches[i] == sides[s] ) {
				si = i;
			}
		if ( si == patches.size() || !pl[si].ok ) {
			continue;
		}
		std::set<brush_t*> grp;
		std::vector<size_t> todo;
		grp.insert( patches[si] );
		todo.push_back( si );
		float area = 0;
		while ( !todo.empty() && grp.size() < 4096 )
		{
			const size_t i = todo.back();
			todo.pop_back();
			area += pl[i].area;
			for ( size_t j = 0; j < patches.size(); j++ )
			{
				if ( !pl[j].ok || grp.count( patches[j] ) ) {
					continue;
				}
				vec3_t d;
				VectorSubtract( pl[j].p0, pl[i].p0, d );
				if ( fabsf( DotProduct( pl[i].n, pl[j].n ) ) < 0.999f || fabsf( DotProduct( pl[i].n, d ) ) > 0.5f ) {
					continue;
				}
				int k;
				for ( k = 0; k < 3; k++ )
					if ( pl[i].mn[k] > pl[j].mx[k] + 1.0f || pl[j].mn[k] > pl[i].mx[k] + 1.0f ) {
						break;
					}
				if ( k < 3 ) {
					continue;
				}
				grp.insert( patches[j] );
				todo.push_back( j );
			}
		}
		if ( area > bestArea ) {
			bestArea = area;
			best = grp;
		}
	}
	if ( best.empty() ) {
		return;
	}
	// the picked edge has to be on the rim of this face: one side in it, the other not
	const bool inA = best.count( start.a ) > 0, inB = start.b != NULL && best.count( start.b ) > 0;
	if ( inA == inB ) {
		return;
	}
	f.group = best;
	f.have = true;
}

// The edge the loop goes on into, coming along cur to the point v. nAt is how many edges meet at v and
// nCand how many of them could carry the loop on (for the message when it stops).
static bool Bevel_NextAround( const std::vector<brush_t*> &patches, const bevelEdge_t &start, bevelFace_t *face, const bevelEdge_t &cur, const float *v, bevelEdge_t *next, int *nAt, int *nCand ){
	std::vector<bevelEdge_t> at;
	Bevel_EdgesAt( patches, v, at );
	*nAt = (int)at.size();
	*nCand = 0;
	bool found = false;
	for ( size_t i = 0; i < at.size(); i++ )
		if ( Bevel_SameEdge( at[i], cur ) ) {
			found = true;
		}
	if ( !found ) {
		*nCand = -1; // the edge we came along is not among the edges found at its own end
		return false;
	}
	const bool shared = cur.b != NULL;
	std::vector<bevelEdge_t> cand;
	for ( size_t i = 0; i < at.size(); i++ )
	{
		if ( Bevel_SameEdge( at[i], cur ) || Bevel_EdgesShare( cur, at[i] ) ) {
			continue; // the opposite edge has no patch in common with this one
		}
		if ( !shared && at[i].b != NULL ) {
			continue; // from a free edge only into another free edge
		}
		cand.push_back( at[i] );
	}
	*nCand = (int)cand.size();
	if ( cand.size() == 1 ) {
		*next = cand[0];
		return true;
	}
	// not a corner of four edges: along the rim of the flat face the picked edge is on, if there is one
	if ( face ) {
		if ( !face->built ) {
			Bevel_BuildFace( patches, start, *face );
		}
		if ( face->have ) {
			std::vector<bevelEdge_t> rim;
			for ( size_t i = 0; i < at.size(); i++ )
			{
				if ( Bevel_SameEdge( at[i], cur ) ) {
					continue;
				}
				const bool inA = face->group.count( at[i].a ) > 0;
				const bool inB = at[i].b != NULL && face->group.count( at[i].b ) > 0;
				if ( inA != inB ) {
					rim.push_back( at[i] );
				}
			}
			if ( rim.size() == 1 ) {
				*nCand = 1;
				*next = rim[0];
				return true;
			}
		}
	}
	if ( shared || cand.empty() ) {
		return false;
	}
	// A free edge with several free edges to choose from (a seam whose two sides do not match counts as
	// two free edges): the one that carries straight on, if it clearly does.
	float dc[3];
	if ( !Bevel_LeaveDir( cur, v, dc ) ) {
		return false;
	}
	int best = -1;
	float bestS = -2.0f, second = -2.0f;
	for ( size_t i = 0; i < cand.size(); i++ )
	{
		float d[3];
		if ( !Bevel_LeaveDir( cand[i], v, d ) ) {
			continue;
		}
		const float s = -DotProduct( dc, d ); // 1 = straight on, 0 = a right angle
		if ( s > bestS ) {
			second = bestS;
			bestS = s;
			best = (int)i;
		}
		else if ( s > second ) {
			second = s;
		}
	}
	if ( best >= 0 && bestS >= 0.5f && bestS - second >= 0.15f ) {
		*next = cand[best];
		return true;
	}
	return false;
}

// for the message when a loop ends: how far away the nearest border end is that just misses the point v
static float Bevel_NearMiss( const std::vector<brush_t*> &patches, const float *v ){
	float best = -1.0f;
	bevelPt_t rim[MAX_PATCH_WIDTH];
	for ( size_t i = 0; i < patches.size(); i++ )
		for ( int side = 0; side < 4; side++ )
		{
			const int n = Bevel_Rim( patches[i]->pPatch, side, rim );
			for ( int e = 0; e < 2; e++ )
			{
				vec3_t d;
				VectorSubtract( rim[e ? n - 1 : 0].v, v, d );
				const float len = VectorLength( d );
				if ( len > BEVEL_LOOP_TOL && len < 16.0f && ( best < 0 || len < best ) ) {
					best = len;
				}
			}
		}
	return best;
}

// the edge and the rest of its loop. The first one of the loop is the edge as the loop sees it, which
// may have found a partner border a little further off than the plain pick does.
static void Bevel_FollowLoop( const bevelEdge_t &picked, std::vector<bevelEdge_t> &loop ){
	loop.clear();
	const bevelEdge_t start = Bevel_MakeEdge( picked.a, picked.sideA );
	loop.push_back( start );
	float s0[3], s1[3];
	Bevel_Ends( start, s0, s1 );
	if ( Bevel_SamePt( s0, s1 ) ) {
		return; // closes on itself
	}
	std::vector<brush_t*> patches;
	Bevel_Patches( patches );
	bool closed = false;
	bevelFace_t face;
	face.built = false;
	face.have = false;
	for ( int dir = 0; dir < 2 && !closed; dir++ )
	{
		bevelEdge_t cur = start;
		float v[3];
		VectorCopy( dir == 0 ? s1 : s0, v );
		for ( int guard = 0; guard < BEVEL_LOOP_MAX; guard++ )
		{
			bevelEdge_t next;
			int nAt, nCand;
			if ( !Bevel_NextAround( patches, start, &face, cur, v, &next, &nAt, &nCand ) ) {
				const float miss = Bevel_NearMiss( patches, v );
				if ( nCand < 0 ) {
					float c0[3], c1[3];
					Bevel_Ends( cur, c0, c1 );
					vec3_t d0, d1;
					VectorSubtract( c0, v, d0 );
					VectorSubtract( c1, v, d1 );
					bool listed = false;
					for ( size_t i = 0; i < patches.size() && !listed; i++ )
						listed = patches[i] == cur.a;
					Sys_Printf( "Edge loop ends at ( %.0f %.0f %.0f ): the edge itself was not found there (side %d, ends %.1f / %.1f away, patch %s, %d patches)\n",
								v[0], v[1], v[2], cur.sideA, VectorLength( d0 ), VectorLength( d1 ), listed ? "listed" : "NOT listed", (int)patches.size() );
				}
				else if ( miss > 0 ) {
					Sys_Printf( "Edge loop ends at ( %.0f %.0f %.0f ): %d edges meet there, %d could carry it on; a border end just misses it by %.1f\n",
								v[0], v[1], v[2], nAt, nCand, miss );
				}
				else
				{
					Sys_Printf( "Edge loop ends at ( %.0f %.0f %.0f ): %d edges meet there, %d could carry it on\n",
								v[0], v[1], v[2], nAt, nCand );
				}
				break;
			}
			if ( Bevel_SameEdge( next, start ) ) {
				closed = true; // all the way round
				break;
			}
			bool seen = false;
			for ( size_t i = 0; i < loop.size() && !seen; i++ )
				seen = Bevel_SameEdge( loop[i], next );
			if ( seen ) {
				break;
			}
			loop.push_back( next );
			float n0[3], n1[3];
			Bevel_Ends( next, n0, n1 );
			if ( Bevel_SamePt( n0, v ) && Bevel_SamePt( n1, v ) ) {
				break;
			}
			// pick the far end first: VectorCopy repeats its first argument for every coordinate, and v changes as it goes
			const float *farEnd = Bevel_SamePt( n0, v ) ? n1 : n0;
			float nv[3];
			VectorCopy( farEnd, nv );
			VectorCopy( nv, v );
			cur = next;
		}
	}
}

// the last pick, to tell a double click (a second pick of the same edge straight after the first)
static bevelEdge_t g_lastPickEdge;
static bool g_lastPickValid = false;
static double g_lastPickTime = 0;
static std::vector<bevelEdge_t> g_picksBeforeLast;     // the picked edges as they were before that pick

static double Bevel_NowMs(){
	GTimeVal tv;
	g_get_current_time( &tv );
	return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

static double Bevel_DoubleClickMs(){
	gint ms = 400;
	GtkSettings *s = gtk_settings_get_default();
	if ( s ) {
		g_object_get( s, "gtk-double-click-time", &ms, NULL );
	}
	return ms;
}

// Alt+click: pick the patch border the ray passes nearest to, whether another patch continues from
// it or not. additive (Ctrl+Alt) adds it to the picked ones, or takes it out again if it is already
// one. A second click on the same edge straight after the first (a double click) picks the whole loop
// it is on instead (see the edge loops above). False if no border is near the ray, so the click can do
// what it did before.
bool BevelEdge_Pick( vec3_t org, vec3_t dir, bool additive, bool camera ){
	float epsilon = 0, divergence = 0;
	if ( camera ) {
		divergence = BEVEL_PICK_EPSILON / ( g_pParentWnd->GetCamWnd()->Camera()->width * 0.5f );
	}
	else
	{
		epsilon = BEVEL_PICK_EPSILON / g_pParentWnd->ActiveXY()->Scale();
	}

	std::vector<brush_t*> patches;
	Bevel_Patches( patches );

	// Every border segment the ray passes close enough to. In the camera view the nearest ones win (below),
	// since a border far back in the scene passes just as close to the ray, and with a bigger tolerance.
	struct pickCand_t
	{
		brush_t *b;
		int side;
		float score, t;
	};
	std::vector<pickCand_t> cands;
	bevelPt_t rim[MAX_PATCH_WIDTH], other[MAX_PATCH_WIDTH];
	for ( size_t i = 0; i < patches.size(); i++ )
	{
		patchMesh_t *p = patches[i]->pPatch;
		for ( int side = 0; side < 4; side++ )
		{
			int n = Bevel_Rim( p, side, rim );
			if ( Bevel_RimDegenerate( rim, n ) ) {
				continue;
			}
			if ( side == 1 || side == 3 ) {
				// a ring: the last column / row is the first one again, one border is enough
				int n0 = Bevel_Rim( p, side - 1, other );
				bool rev;
				if ( n0 == n && Bevel_RimsMatch( rim, other, n, &rev ) && !rev ) {
					continue;
				}
			}
			std::vector<bevelPt_t> line;
			Bevel_Border( p, side, line );
			for ( size_t k = 0; k + 1 < line.size(); k++ )
			{
				float dist, t;
				if ( !Bevel_RaySegment( org, dir, line[k].v, line[k + 1].v, &dist, &t ) ) {
					continue;
				}
				const float tol = epsilon + divergence * t;
				if ( dist > tol ) {
					continue;
				}
				const pickCand_t c = { patches[i], side, dist / tol + t * 1e-6f, t }; // the closest to the ray, the nearer one on a tie
				cands.push_back( c );
			}
		}
	}
	if ( cands.empty() ) {
		return false;
	}
	float tmin = cands[0].t;
	for ( size_t i = 1; i < cands.size(); i++ )
		if ( cands[i].t < tmin ) {
			tmin = cands[i].t;
		}
	// In the camera view only what is about as near as the nearest hit counts (a tenth further, or a few
	// units): the border in front, not one behind it at the same spot on the screen. Among those the one
	// the ray passes closest to. In the XY view there is no depth to go by, so it is the closest overall.
	const float depthLimit = camera ? tmin + ( tmin * 0.1f > 8.0f ? tmin * 0.1f : 8.0f ) : 1e30f;
	brush_t *bestBrush = NULL;
	int bestSide = -1;
	float bestScore = 1e30f;
	for ( size_t i = 0; i < cands.size(); i++ )
		if ( cands[i].t <= depthLimit && cands[i].score < bestScore ) {
			bestScore = cands[i].score;
			bestBrush = cands[i].b;
			bestSide = cands[i].side;
		}

	bevelEdge_t e = { bestBrush, bestSide, NULL, 0, false };
	if ( Bevel_FindPartner( bestBrush, bestSide, &e.b, &e.sideB, &e.reversed ) && e.b < e.a ) {
		// the same edge is always stored the same way round, whichever patch it was picked from
		bevelEdge_t s = { e.b, e.sideB, e.a, e.sideA, e.reversed };
		e = s;
	}

	// A double click on an edge picks its whole loop. The second click starts from the picks as they were
	// before the first one, so Ctrl+Alt double click adds the loop (or takes it out if it was picked).
	// With the preference off none of this happens and every click picks one edge.
	bool dbl = false;
	if ( g_PrefsDlg.m_bBevelLoops ) {
		const double now = Bevel_NowMs();
		dbl = g_lastPickValid && Bevel_SameEdge( g_lastPickEdge, e ) && now - g_lastPickTime <= Bevel_DoubleClickMs();
		if ( dbl ) {
			g_bevelEdges = g_picksBeforeLast;
			g_lastPickValid = false;
		}
		else
		{
			g_picksBeforeLast = g_bevelEdges;
			g_lastPickEdge = e;
			g_lastPickTime = now;
			g_lastPickValid = true;
		}
	}

	std::vector<bevelEdge_t> loop;
	if ( dbl ) {
		Bevel_FollowLoop( e, loop );
	}
	else
	{
		loop.push_back( e );
	}
	const bevelEdge_t key = loop[0]; // the edge the click was on, as it is stored

	bool removed = false;
	if ( additive ) {
		for ( size_t i = 0; i < g_bevelEdges.size(); i++ )
			if ( Bevel_SameEdge( g_bevelEdges[i], key ) ) {
				removed = true; // already picked: it is taken out, with the rest of its loop
				break;
			}
		if ( removed ) {
			for ( size_t j = 0; j < loop.size(); j++ )
				for ( size_t i = 0; i < g_bevelEdges.size(); i++ )
					if ( Bevel_SameEdge( g_bevelEdges[i], loop[j] ) ) {
						g_bevelEdges.erase( g_bevelEdges.begin() + i );
						break;
					}
		}
	}
	else
	{
		g_bevelEdges.clear();
	}
	if ( !removed ) {
		for ( size_t j = 0; j < loop.size(); j++ )
		{
			bool known = false;
			for ( size_t i = 0; i < g_bevelEdges.size() && !known; i++ )
				known = Bevel_SameEdge( g_bevelEdges[i], loop[j] );
			if ( !known ) {
				g_bevelEdges.push_back( loop[j] );
			}
		}
	}
	if ( loop.size() > 1 ) {
		Sys_Printf( "Edge loop %s: %d edges (%d picked)\n", removed ? "taken out" : "picked", (int)loop.size(), (int)g_bevelEdges.size() );
	}
	else
	{
		Sys_Printf( "Edge %s: %s, %d control points (%d picked)\n", removed ? "taken out" : "picked",
					e.b ? "shared with another patch" : "free edge",
					( e.sideA < 2 ) ? e.a->pPatch->height : e.a->pPatch->width, (int)g_bevelEdges.size() );
	}
	Sys_UpdateWindows( W_XY | W_CAMERA );
	return true;
}

// the picked edges, drawn on top of everything in both views
void BevelEdge_Draw(){
	for ( size_t i = 0; i < g_bevelEdges.size(); )
	{
		if ( Bevel_BrushLive( g_bevelEdges[i].a ) && ( g_bevelEdges[i].b == NULL || Bevel_BrushLive( g_bevelEdges[i].b ) ) ) {
			i++;
		}
		else
		{
			g_bevelEdges.erase( g_bevelEdges.begin() + i ); // a patch it belonged to is gone or has changed
		}
	}
	if ( g_bevelEdges.empty() ) {
		return;
	}

	qglPushAttrib( GL_ALL_ATTRIB_BITS );
	qglDisable( GL_TEXTURE_2D );
	qglDisable( GL_LIGHTING );
	qglDisable( GL_BLEND );
	qglDisable( GL_CULL_FACE );
	qglDisable( GL_DEPTH_TEST );
	qglDisable( GL_LINE_STIPPLE );
	qglLineWidth( 3 );
	qglColor3f( 1.0f, 0.85f, 0.1f );

	for ( size_t i = 0; i < g_bevelEdges.size(); i++ )
	{
		std::vector<bevelPt_t> line;
		Bevel_Border( g_bevelEdges[i].a->pPatch, g_bevelEdges[i].sideA, line );
		qglBegin( GL_LINE_STRIP );
		for ( size_t k = 0; k < line.size(); k++ )
			qglVertex3fv( line[k].v );
		qglEnd();
	}
	qglLineWidth( 1 );
	qglPopAttrib();
}

// =============================================================================
// The bevel itself
//
// Both patches that meet at the edge are cut back by the bevel width, measured along their surface,
// and the gap is filled with a strip. Every column across the edge becomes a small curve: it leaves
// the first patch along its surface, comes back to the second one along its surface, and the
// control point in between is where those two tangent lines meet (the old corner, for straight
// surfaces). With the same width on both sides that is a circular arc.
// =============================================================================

// a patch seen from one of its borders: k runs along the border, r goes in from it
struct bevelFrame_t
{
	patchMesh_t *p;
	int side;
	int n;      // control points along the border
	int m;      // control points going inwards
};

static bevelFrame_t Bevel_Frame( patchMesh_t *p, int side ){
	bevelFrame_t f = { p, side, side < 2 ? p->height : p->width, side < 2 ? p->width : p->height };
	return f;
}

static drawVert_t& Bevel_At( const bevelFrame_t &f, int k, int r ){
	patchMesh_t *p = f.p;
	switch ( f.side )
	{
	case 0:  return p->ctrl[r][k];
	case 1:  return p->ctrl[p->width - 1 - r][k];
	case 2:  return p->ctrl[k][r];
	default: return p->ctrl[k][p->height - 1 - r];
	}
}

static void Bevel_Lerp( const float *a, const float *b, float t, float *o ){
	for ( int c = 0; c < 3; c++ )
		o[c] = a[c] + ( b[c] - a[c] ) * t;
}

// cumulative length of the first span (three control points) of one column, in 64 steps
static void Bevel_SpanTable( const float *p0, const float *p1, const float *p2, float *cum ){
	float prev[3] = { p0[0], p0[1], p0[2] };
	cum[0] = 0;
	for ( int i = 1; i <= 64; i++ )
	{
		float t = i / 64.0f, u = 1.0f - t, q[3], d[3];
		for ( int c = 0; c < 3; c++ )
			q[c] = u * u * p0[c] + 2 * u * t * p1[c] + t * t * p2[c];
		VectorSubtract( q, prev, d );
		cum[i] = cum[i - 1] + VectorLength( d );
		VectorCopy( q, prev );
	}
}

// how far in from the border a bevel can reach: the first span of the patch, nearly all of it
static bool Bevel_Reach( const bevelFrame_t &f, float *maxWidth ){
	if ( f.m < 3 || ( f.m & 1 ) == 0 || f.n < 3 ) {
		return false;
	}
	float best = 1e30f;
	for ( int k = 0; k < f.n; k += 2 )
	{
		float cum[65];
		Bevel_SpanTable( Bevel_At( f, k, 0 ).xyz, Bevel_At( f, k, 1 ).xyz, Bevel_At( f, k, 2 ).xyz, cum );
		if ( cum[64] < 0.01f ) {
			continue; // a collapsed column
		}
		if ( cum[64] < best ) {
			best = cum[64];
		}
	}
	if ( best > 1e29f ) {
		return false;
	}
	*maxWidth = best * 0.98f;
	return true;
}

// the curve parameter at which the first span has been cut by `width`, averaged over the columns
static float Bevel_TrimParam( const bevelFrame_t &f, float width ){
	float sum = 0;
	int cnt = 0;
	for ( int k = 0; k < f.n; k += 2 )
	{
		float cum[65];
		Bevel_SpanTable( Bevel_At( f, k, 0 ).xyz, Bevel_At( f, k, 1 ).xyz, Bevel_At( f, k, 2 ).xyz, cum );
		if ( cum[64] < 0.01f ) {
			continue;
		}
		int i = 1;
		while ( i < 64 && cum[i] < width )
			i++;
		const float seg = cum[i] - cum[i - 1];
		const float frac = seg > 1e-6f ? ( width - cum[i - 1] ) / seg : 0.0f;
		sum += ( i - 1 + ( frac < 0 ? 0 : ( frac > 1 ? 1 : frac ) ) ) / 64.0f;
		cnt++;
	}
	return cnt ? sum / cnt : 0.0f;
}

// cut the first span at t: the border moves to the cut, and the control point after it moves with it.
// Gives back where the border went (S) and where the curve was heading from there to the old border (L)
static void Bevel_Trim( const bevelFrame_t &f, float t, std::vector<bevelPt_t> &S, std::vector<bevelPt_t> &L ){
	S.resize( f.n );
	L.resize( f.n );
	for ( int k = 0; k < f.n; k++ )
	{
		drawVert_t &d0 = Bevel_At( f, k, 0 ), &d1 = Bevel_At( f, k, 1 ), &d2 = Bevel_At( f, k, 2 );
		float l[3], r[3], s[3];
		Bevel_Lerp( d0.xyz, d1.xyz, t, l );
		Bevel_Lerp( d1.xyz, d2.xyz, t, r );
		Bevel_Lerp( l, r, t, s );
		float ls[2], rs[2], ss[2];
		for ( int c = 0; c < 2; c++ )
		{
			ls[c] = d0.st[c] + ( d1.st[c] - d0.st[c] ) * t;
			rs[c] = d1.st[c] + ( d2.st[c] - d1.st[c] ) * t;
			ss[c] = ls[c] + ( rs[c] - ls[c] ) * t;
		}
		VectorCopy( s, S[k].v );
		VectorCopy( l, L[k].v );
		VectorCopy( s, d0.xyz );
		VectorCopy( r, d1.xyz );
		d0.st[0] = ss[0];
		d0.st[1] = ss[1];
		d1.st[0] = rs[0];
		d1.st[1] = rs[1];
	}
}

// corner of the tangent lines qa + s*ta (ta towards the corner) and qb + r*tb (tb away from it)
static void Bevel_Corner( const float *qa, const float *ta, const float *qb, const float *tb, float *out ){
	vec3_t w, dd;
	VectorAdd( qa, qb, out );
	VectorScale( out, 0.5f, out ); // fallback: the middle
	VectorSubtract( qa, qb, w );
	const float b = DotProduct( ta, tb ), d = DotProduct( ta, w ), e = DotProduct( tb, w );
	const float den = 1.0f - b * b;
	if ( den < 1e-4f ) {
		return;
	}
	const float s = ( b * e - d ) / den, r = ( e - b * d ) / den;
	VectorSubtract( qb, qa, dd );
	const float chord = VectorLength( dd );
	if ( s < -0.01f * chord || r > 0.01f * chord || s > 20.0f * chord || -r > 20.0f * chord ) {
		return;
	}
	vec3_t pa, pb;
	VectorMA( qa, s, ta, pa );
	VectorMA( qb, r, tb, pb );
	VectorAdd( pa, pb, out );
	VectorScale( out, 0.5f, out );
}

static bool Bevel_Unit( const float *v, float *out ){
	float l = sqrtf( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
	if ( l < 1e-4f ) {
		return false;
	}
	VectorScale( v, 1.0f / l, out );
	return true;
}

// the direction of the border at its column k (unit vector, zero if it has no length)
static void Bevel_AlongAt( const bevelFrame_t &f, int k, float *out ){
	const int k0 = k > 0 ? k - 1 : k, k1 = k < f.n - 1 ? k + 1 : k;
	vec3_t d;
	VectorSubtract( Bevel_At( f, k1, 0 ).xyz, Bevel_At( f, k0, 0 ).xyz, d );
	if ( !Bevel_Unit( d, out ) ) {
		VectorClear( out );
	}
}

// The direction t of the cut across a face is the line the face was trimmed along; on a face whose grid lines
// are not square to the border (a fan triangle) that line is slanted, and the corner of the tangent lines that
// gives the middle point of the strip would sit beside the edge. Only the part of t that is square to the border
// is kept, which puts that point back on the edge.
static void Bevel_Perp( float *t, const float *along ){
	vec3_t q;
	VectorMA( t, -DotProduct( t, along ), along, q );
	vec3_t u;
	if ( VectorLength( q ) > 0.3f && Bevel_Unit( q, u ) ) {
		VectorCopy( u, t );
	}
}

// the largest width this edge allows; false if it cannot be beveled
static bool Bevel_EdgeReach( const bevelEdge_t &e, float *maxWidth ){
	if ( e.b == NULL || e.a == e.b || !Bevel_BrushLive( e.a ) || !Bevel_BrushLive( e.b ) ) {
		return false;
	}
	bevelFrame_t fa = Bevel_Frame( e.a->pPatch, e.sideA ), fb = Bevel_Frame( e.b->pPatch, e.sideB );
	float ra, rb;
	if ( fa.n != fb.n || !Bevel_Reach( fa, &ra ) || !Bevel_Reach( fb, &rb ) ) {
		return false;
	}
	*maxWidth = ra < rb ? ra : rb;
	return true;
}

// The widest bevel that fits all of these edges at once, on the patches as they are now: the least room any of
// them has, and half of it where two of the edges cut the same patch from opposite sides (they must not run into
// each other). Every bevel of a batch gets this one width, so that where two of them meet they are equally wide.
// 0 if none of them can be beveled.
static float Bevel_BatchLimit( const std::vector<bevelEdge_t> &edges ){
	float best = 0;
	bool any = false;
	for ( size_t i = 0; i < edges.size(); i++ )
	{
		float w;
		if ( !Bevel_EdgeReach( edges[i], &w ) ) {
			continue;
		}
		bool shared = false;
		for ( size_t j = 0; j < edges.size() && !shared; j++ )
		{
			if ( j == i ) {
				continue;
			}
			for ( int x = 0; x < 2 && !shared; x++ )
			{
				brush_t *px = x ? edges[i].b : edges[i].a;
				const int sx = x ? edges[i].sideB : edges[i].sideA;
				if ( ( edges[j].a == px && edges[j].sideA == ( sx ^ 1 ) ) || ( edges[j].b == px && edges[j].sideB == ( sx ^ 1 ) ) ) {
					shared = true;
				}
			}
		}
		if ( shared ) {
			w *= 0.5f;
		}
		if ( !any || w < best ) {
			best = w;
			any = true;
		}
	}
	return any ? best : 0.0f;
}

// the largest width that works for every picked edge that can be beveled; 0 if none can
float BevelEdge_MaxWidth(){
	return Bevel_BatchLimit( g_bevelEdges );
}

// Every bevel made in this session is remembered: the control points of both patches as they were
// before, and a snapshot of everything as the bevel left it. While the snapshot still matches, the
// bevel can be redone with other numbers. If anything was moved or edited since (or an undo replaced
// the patches), the record is dropped and the bevel is just ordinary patches again.
// a face that is not part of the bevel but had a corner moved to meet its strips (see Bevel_Mitre), with how it was
struct bevelExtra_t
{
	brush_t *b;
	std::vector<drawVert_t> orig;
};

struct bevelRec_t
{
	bevelEdge_t e;
	brush_t *strip;
	std::vector<bevelExtra_t> extras;
	std::vector<drawVert_t> origA, origB;
	std::vector<float> postA, postB, postS;
	float width, maxW;
	int nsub;
	int undoId;
	brush_t *cap[2];        // the piece that closes the cap at an end of the strip that no other bevel meets (or NULL)
	float v0[2][3];         // the two corners of the edge before it was beveled (first and last column of the strip)
	int gen;                // which batch of bevels it was made with; a later one may be made on the strips of an earlier one
	bool inv;               // the rows of the strip run from the second patch to the first (the strip was turned over)
	bevelRec_t() : strip( NULL ), width( 0 ), maxW( 0 ), nsub( 0 ), undoId( 0 ), gen( 0 ), inv( false ){
		cap[0] = cap[1] = NULL;
		memset( v0, 0, sizeof( v0 ) );
	}
};

static std::vector<bevelRec_t> g_bevelRecs;
static int g_bevelGen = 0;

// do two records have an end at the same corner of the model
static bool Bevel_Meets( const bevelRec_t &a, const bevelRec_t &b ){
	for ( int i = 0; i < 2; i++ )
		for ( int j = 0; j < 2; j++ )
		{
			vec3_t d;
			VectorSubtract( a.v0[i], b.v0[j], d );
			if ( VectorLength( d ) <= 0.5f ) {
				return true;
			}
		}
	return false;
}

// do two records work on one another's faces: they share a patch, or one is made on the strip or cap piece of the other
static bool Bevel_Linked( const bevelRec_t &a, const bevelRec_t &b ){
	brush_t *pa[4] = { a.e.a, a.e.b, a.strip, NULL }, *pb[4] = { b.e.a, b.e.b, b.strip, NULL };
	for ( int i = 0; i < 2; i++ )
		for ( int j = 0; j < 2; j++ )
			if ( pa[i] == pb[j] ) {
				return true;
			}
	for ( int c = 0; c < 2; c++ )
	{
		for ( int i = 0; i < 2; i++ )
		{
			if ( a.cap[c] && ( b.e.a == a.cap[c] || b.e.b == a.cap[c] ) ) {
				return true;
			}
			if ( b.cap[c] && ( a.e.a == b.cap[c] || a.e.b == b.cap[c] ) ) {
				return true;
			}
		}
	}
	for ( int i = 0; i < 2; i++ )
		if ( pa[i] == b.strip || pb[i] == a.strip ) {
			return true;
		}
	return false;
}

static void Bevel_Save( const patchMesh_t *p, std::vector<drawVert_t> &out ){
	out.clear();
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
			out.push_back( p->ctrl[i][j] );
}

static void Bevel_Restore( patchMesh_t *p, const std::vector<drawVert_t> &in ){
	size_t n = 0;
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
			if ( n < in.size() ) {
				p->ctrl[i][j] = in[n++];
			}
	p->bDirty = true;
}

static void Bevel_Snap( const patchMesh_t *p, std::vector<float> &out ){
	out.clear();
	out.push_back( (float)p->width );
	out.push_back( (float)p->height );
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			out.push_back( p->ctrl[i][j].xyz[0] );
			out.push_back( p->ctrl[i][j].xyz[1] );
			out.push_back( p->ctrl[i][j].xyz[2] );
		}
}

static bool Bevel_SnapSame( const patchMesh_t *p, const std::vector<float> &snap ){
	std::vector<float> now;
	Bevel_Snap( p, now );
	return now == snap;
}

static bool Bevel_RecValid( const bevelRec_t &r ){
	return Bevel_BrushLive( r.e.a ) && Bevel_BrushLive( r.e.b ) && Bevel_BrushLive( r.strip )
		   && Bevel_SnapSame( r.e.a->pPatch, r.postA ) && Bevel_SnapSame( r.e.b->pPatch, r.postB )
		   && Bevel_SnapSame( r.strip->pPatch, r.postS );
}

// Brushes that are already saved in the undo step being made. A patch that two beveled edges share must be saved
// once, as it was before the first bevel: saving it again would leave a second, half beveled copy behind when the
// step is undone.
static std::set<brush_t*> g_bevelUndone;

static void Bevel_UndoAdd( brush_t *b ){
	if ( g_bevelUndone.insert( b ).second ) {
		Undo_AddBrush( b );
	}
}

static bool Bevel_IsSelected( brush_t *b ){
	for ( brush_t *p = selected_brushes.next; p != &selected_brushes; p = p->next )
		if ( p == b ) {
			return true;
		}
	return false;
}

// the bevel strips (made with Bevel Edge) that join two patches of the given set, or hang off one of them
void Bevel_StripsOf( const std::vector<brush_t*> &parts, std::vector<brush_t*> &out ){
	// A bevel can be made on the strip or cap piece of another bevel (an inner corner on a beveled rim), so the
	// strips and caps found count as parts of the set too, until nothing new turns up.
	std::set<brush_t*> in( parts.begin(), parts.end() ), done;
	for ( bool grew = true; grew; )
	{
		grew = false;
		for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		{
			const bevelRec_t &r = g_bevelRecs[i];
			if ( !Bevel_BrushLive( r.strip ) || done.count( r.strip ) ) {
				continue;
			}
			if ( in.count( r.e.a ) || in.count( r.e.b ) ) {
				done.insert( r.strip );
				in.insert( r.strip );
				out.push_back( r.strip );
				for ( int c = 0; c < 2; c++ )
					if ( Bevel_BrushLive( r.cap[c] ) ) {
						in.insert( r.cap[c] );
						out.push_back( r.cap[c] );
					}
				grew = true;
			}
		}
	}
}

// Cuts both patches back and fills the gap. With a record the patches are first put back as they were
// and the strip that is already there is rebuilt in place; without one a new strip is made (and
// remembered). useUndo: save the brushes for undo (the caller has started the undo).
static void Bevel_EdgeCorners( const bevelEdge_t &e, float v0[2][3] ){
	bevelFrame_t f = Bevel_Frame( e.a->pPatch, e.sideA );
	VectorCopy( Bevel_At( f, 0, 0 ).xyz, v0[0] );
	VectorCopy( Bevel_At( f, f.n - 1, 0 ).xyz, v0[1] );
}

static brush_t* Bevel_Build( const bevelEdge_t &e, float width, int nsub, bevelRec_t *rec, bool useUndo, float limit = 0.0f, const float *v0in = NULL ){
	if ( e.a == NULL || e.b == NULL || e.a == e.b ) {
		return NULL; // a free edge has no second patch to bevel towards
	}
	brush_t *A = e.a, *B = e.b;
	patchMesh_t *pa = A->pPatch, *pb = B->pPatch;

	// the patches as they are now; with a record the caller (BevelRec_Edit) has put them back to how they were
	// before the bevels first, so this is the state this bevel starts from
	std::vector<drawVert_t> oA, oB;
	Bevel_Save( pa, oA );
	Bevel_Save( pb, oB );
	float v0Now[2][3];
	Bevel_EdgeCorners( e, v0Now );
	if ( useUndo ) {
		Bevel_UndoAdd( A );
		Bevel_UndoAdd( B );
		if ( rec ) {
			Bevel_UndoAdd( rec->strip );
		}
	}

	float maxW;
	if ( !Bevel_EdgeReach( e, &maxW ) ) {
		return NULL;
	}
	if ( limit > 0.0f && limit < maxW ) {
		maxW = limit;
	}
	if ( width > maxW ) {
		width = maxW;
	}
	bevelFrame_t fa = Bevel_Frame( pa, e.sideA ), fb = Bevel_Frame( pb, e.sideB );
	const int n = fa.n;

	// which way the first patch shows its face (cross(dU,dV) of the patch), at the middle of the edge
	int km = ( n / 2 ) & ~1;
	vec3_t along, inward, faceA;
	if ( km == 0 ) {
		VectorSubtract( Bevel_At( fa, 1, 0 ).xyz, Bevel_At( fa, 0, 0 ).xyz, along );
	}
	else
	{
		VectorSubtract( Bevel_At( fa, km, 0 ).xyz, Bevel_At( fa, km - 1, 0 ).xyz, along );
	}
	VectorSubtract( Bevel_At( fa, km, 1 ).xyz, Bevel_At( fa, km, 0 ).xyz, inward );
	CrossProduct( inward, along, faceA );
	if ( e.sideA == 1 || e.sideA == 2 ) {
		VectorScale( faceA, -1.0f, faceA );
	}

	std::vector<bevelPt_t> SA, LA, SB, LB;
	const float tA = Bevel_TrimParam( fa, width ), tB = Bevel_TrimParam( fb, width );
	Bevel_Trim( fa, tA, SA, LA );
	Bevel_Trim( fb, tB, SB, LB );
	// the strip's columns are numbered like the first patch's; the second one may run the other way
	const int kmB = e.reversed ? n - 1 - km : km;

	patchMesh_t *s = rec ? rec->strip->pPatch : MakeNewPatch();
	if ( !rec ) {
		s->pShader = pa->pShader;
		s->d_texture = pa->d_texture;
	}
	s->width = n;
	s->height = 3;
	s->type = PATCH_GENERIC;
	for ( int k = 0; k < n; k++ )
	{
		const int kb = e.reversed ? n - 1 - k : k;
		drawVert_t &a = Bevel_At( fa, k, 0 ), &b = Bevel_At( fb, kb, 0 );
		VectorCopy( a.xyz, s->ctrl[k][0].xyz );
		VectorCopy( b.xyz, s->ctrl[k][2].xyz );
		vec3_t ta, tb, d;
		VectorSubtract( LA[k].v, SA[k].v, d );
		bool okA = Bevel_Unit( d, ta );
		VectorSubtract( SB[kb].v, LB[kb].v, d );
		bool okB = Bevel_Unit( d, tb );
		if ( okA ) {
			vec3_t al;
			Bevel_AlongAt( fa, k, al );
			Bevel_Perp( ta, al );
		}
		if ( okB ) {
			vec3_t al;
			Bevel_AlongAt( fb, kb, al );
			Bevel_Perp( tb, al );
		}
		if ( okA && okB ) {
			Bevel_Corner( s->ctrl[k][0].xyz, ta, s->ctrl[k][2].xyz, tb, s->ctrl[k][1].xyz );
		}
		else
		{
			VectorAdd( s->ctrl[k][0].xyz, s->ctrl[k][2].xyz, s->ctrl[k][1].xyz );
			VectorScale( s->ctrl[k][1].xyz, 0.5f, s->ctrl[k][1].xyz );
		}
		for ( int c = 0; c < 2; c++ )
		{
			s->ctrl[k][0].st[c] = a.st[c];
			s->ctrl[k][2].st[c] = b.st[c];
			s->ctrl[k][1].st[c] = 0.5f * ( a.st[c] + b.st[c] );
		}
	}

	// the strip faces the same way as the patches it joins
	vec3_t across, faceS;
	VectorSubtract( SB[kmB].v, SA[km].v, across );
	CrossProduct( along, across, faceS ); // cross(dU,dV) with U along the edge and V across it
	// (inverting a patch reverses its rows: the first row of the strip is then on the side of the second patch)
	bool inverted = false;
	if ( DotProduct( faceS, faceA ) < 0 ) {
		patchInvert( s );
		inverted = true;
	}

	// counts: the strip carries the along-the-edge count of the patch it came from
	int along_count = 0;
	if ( pa->segH > 0 ) {
		along_count = e.sideA < 2 ? pa->segV : pa->segH;
	}
	else if ( pb->segH > 0 ) {
		along_count = e.sideB < 2 ? pb->segV : pb->segH;
	}
	if ( along_count <= 0 ) {
		// Plain patches carry no count: take it from how much the edge bends. A straight edge needs none
		// (the popup's subdivisions are then the ones across the bevel), a curved one about one per 11 degrees.
		double turn = 0.0;
		for ( int k = 1; k + 1 < n; k++ )
		{
			vec3_t d0, d1, u0, u1;
			VectorSubtract( Bevel_At( fa, k, 0 ).xyz, Bevel_At( fa, k - 1, 0 ).xyz, d0 );
			VectorSubtract( Bevel_At( fa, k + 1, 0 ).xyz, Bevel_At( fa, k, 0 ).xyz, d1 );
			if ( Bevel_Unit( d0, u0 ) && Bevel_Unit( d1, u1 ) ) {
				float c = DotProduct( u0, u1 );
				c = c > 1.0f ? 1.0f : ( c < -1.0f ? -1.0f : c );
				turn += acos( c ) * 180.0 / 3.14159265358979;
			}
		}
		along_count = turn < 2.0 ? 1 : (int)ceil( turn / 11.25 );
	}
	s->segH = SegPatch_ClampH( along_count, SegPatch_MinH( s ) );
	s->segV = SegPatch_ClampV( nsub );

	brush_t *sb;
	if ( rec ) {
		sb = rec->strip;
		s->bDirty = true;
		Patch_Rebuild( s );
	}
	else
	{
		sb = AddBrushForPatch( s, false );
		Brush_AddToList( sb, &active_brushes );
		Entity_LinkBrush( A->owner, sb );
		Brush_Build( sb );
		Patch_Rebuild( s );
	}
	Patch_Rebuild( pa );
	Patch_Rebuild( pb );
	if ( useUndo ) {
		Undo_EndBrush( A );
		Undo_EndBrush( B );
		Undo_EndBrush( sb );
	}

	bevelRec_t r;
	if ( rec ) {
		r = *rec;
	}
	else
	{
		r.e = e;
		r.strip = sb;
		r.undoId = 0;
		if ( v0in ) {
			memcpy( r.v0, v0in, sizeof( r.v0 ) );
		}
		else
		{
			memcpy( r.v0, v0Now, sizeof( r.v0 ) );
		}
	}
	r.origA = oA;
	r.origB = oB;
	r.inv = inverted;
	r.width = width;
	r.maxW = maxW;
	r.nsub = s->segV;
	Bevel_Snap( pa, r.postA );
	Bevel_Snap( pb, r.postB );
	Bevel_Snap( s, r.postS );
	if ( rec ) {
		*rec = r;
	}
	else
	{
		g_bevelRecs.push_back( r );
	}
	return sb;
}

// ---------------------------------------------------------------------------
// Mitred corners.
//
// Where two beveled edges meet at a corner, each strip ends in a flat cut across its own edge, and the two cuts
// leave a wedge-shaped hole. Instead of filling it with a piece of its own, the strips are extended (or cut
// back) until they meet along the plane that halves the corner, the way two picture-frame mouldings do. A strip
// is a straight sweep of its cross-section, so its end is moved by sliding each of the three end control points
// along the strip onto that plane.
//
// The faces round the corner follow: every face that has a corner at the vertex, or a corner that the cut of
// a strip left behind on one of its edges, gets that corner moved to where the strips now end (the corner
// points of a face and the points between them on its edges move together, so the edges stay straight and the
// faces stay flat). On a cog that is what lets the three cap pieces at a root corner and the two walls meet the
// two strips without gaps or flaps.
//
// The plane depends only on the directions of the two edges, so doing it again changes nothing. A corner where
// three or more beveled edges meet is left alone: no single plane halves it.

struct mitreEnd_t
{
	size_t rec;
	int end;                // 0 = the first column of the strip, 1 = the last
	vec3_t v0;              // the corner of the edge before it was beveled
	vec3_t p;               // the middle control point of the end column (on the line of the edge)
	vec3_t u;               // unit vector along the edge, away from the end
	float width;
	bool rowStraight[3];    // the three rows of the strip run in a straight line (the edge is straight)
};

// closest approach of two lines (u1, u2 unit vectors); false if they run parallel
static bool Bevel_LineMeet( const float *p1, const float *u1, const float *p2, const float *u2, float *meet, float *gap ){
	vec3_t w0;
	VectorSubtract( p1, p2, w0 );
	const float b = DotProduct( u1, u2 );
	const float d = DotProduct( u1, w0 ), e = DotProduct( u2, w0 );
	const float den = 1.0f - b * b;
	if ( den < 1e-3f ) {
		return false;
	}
	const float s = ( b * e - d ) / den, t = ( e - b * d ) / den;
	vec3_t q1, q2, df;
	for ( int k = 0; k < 3; k++ )
	{
		q1[k] = p1[k] + s * u1[k];
		q2[k] = p2[k] + t * u2[k];
	}
	VectorSubtract( q1, q2, df );
	*gap = VectorLength( df );
	for ( int k = 0; k < 3; k++ )
		meet[k] = 0.5f * ( q1[k] + q2[k] );
	return true;
}

static float Bevel_MaxF( float a, float b ){
	return a > b ? a : b;
}

// the middle control point lies on the line between the two ends
static bool Bevel_Straight3( const float *a, const float *m, const float *b ){
	vec3_t mid, d, e;
	for ( int k = 0; k < 3; k++ )
		mid[k] = 0.5f * ( a[k] + b[k] );
	VectorSubtract( m, mid, d );
	VectorSubtract( b, a, e );
	return VectorLength( d ) <= 0.02f + 0.002f * VectorLength( e );
}

static void Bevel_Mid3( const float *a, const float *b, float *out ){
	for ( int k = 0; k < 3; k++ )
		out[k] = 0.5f * ( a[k] + b[k] );
}

// the control point of the patch that is the corner at the end of the border `side`, column k
static bool Bevel_CornerAt( patchMesh_t *p, int side, int k, int *ci, int *cj ){
	bevelFrame_t f = Bevel_Frame( p, side );
	const drawVert_t *d = &Bevel_At( f, k, 0 );
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
			if ( &p->ctrl[i][j] == d ) {
				*ci = i;
				*cj = j;
				return true;
			}
	return false;
}

// Moves a corner of a patch to `target`. The points between it and the next corners come along: on a
// 3 x 3 patch an edge that was straight stays straight (its middle point stays on the line between the ends),
// a patch that was a flat bilinear one stays that, anything else is pulled along in proportion.
static void Bevel_MoveCorner( patchMesh_t *p, int ci, int cj, const float *target ){
	const int W = p->width, H = p->height;
	vec3_t d;
	VectorSubtract( target, p->ctrl[ci][cj].xyz, d );
	if ( VectorLength( d ) < 1e-4f ) {
		return;
	}
	if ( W == 3 && H == 3 ) {
		#define BC( i, j ) p->ctrl[i][j].xyz
		const bool st[4] = {
			Bevel_Straight3( BC( 0, 0 ), BC( 1, 0 ), BC( 2, 0 ) ),
			Bevel_Straight3( BC( 0, 2 ), BC( 1, 2 ), BC( 2, 2 ) ),
			Bevel_Straight3( BC( 0, 0 ), BC( 0, 1 ), BC( 0, 2 ) ),
			Bevel_Straight3( BC( 2, 0 ), BC( 2, 1 ), BC( 2, 2 ) )
		};
		vec3_t avg;
		for ( int k = 0; k < 3; k++ )
			avg[k] = 0.25f * ( BC( 0, 0 )[k] + BC( 2, 0 )[k] + BC( 0, 2 )[k] + BC( 2, 2 )[k] );
		vec3_t dc;
		VectorSubtract( BC( 1, 1 ), avg, dc );
		const bool bilinear = st[0] && st[1] && st[2] && st[3] && VectorLength( dc ) <= 0.02f + 0.002f * VectorLength( d );
		VectorCopy( target, BC( ci, cj ) );
		const bool touch[4] = { cj == 0, cj == 2, ci == 0, ci == 2 };
		for ( int e = 0; e < 4; e++ )
		{
			float *mid = e == 0 ? BC( 1, 0 ) : ( e == 1 ? BC( 1, 2 ) : ( e == 2 ? BC( 0, 1 ) : BC( 2, 1 ) ) );
			if ( st[e] ) {
				float *a = e == 0 ? BC( 0, 0 ) : ( e == 1 ? BC( 0, 2 ) : ( e == 2 ? BC( 0, 0 ) : BC( 2, 0 ) ) );
				float *b = e == 0 ? BC( 2, 0 ) : ( e == 1 ? BC( 2, 2 ) : ( e == 2 ? BC( 0, 2 ) : BC( 2, 2 ) ) );
				Bevel_Mid3( a, b, mid );
			}
			else if ( touch[e] ) {
				for ( int k = 0; k < 3; k++ )
					mid[k] += 0.5f * d[k];
			}
		}
		if ( bilinear ) {
			for ( int k = 0; k < 3; k++ )
				BC( 1, 1 )[k] = 0.25f * ( BC( 0, 0 )[k] + BC( 2, 0 )[k] + BC( 0, 2 )[k] + BC( 2, 2 )[k] );
		}
		else
		{
			for ( int k = 0; k < 3; k++ )
				BC( 1, 1 )[k] += 0.25f * d[k];
		}
		#undef BC
	}
	else
	{
		for ( int i = 0; i < W; i++ )
			for ( int j = 0; j < H; j++ )
			{
				const float wi = 1.0f - (float)abs( i - ci ) / (float)( W - 1 );
				const float wj = 1.0f - (float)abs( j - cj ) / (float)( H - 1 );
				for ( int k = 0; k < 3; k++ )
					p->ctrl[i][j].xyz[k] += wi * wj * d[k];
			}
	}
	p->bDirty = true;
}

// puts the end column of a strip on the three points S[0..2]; the control points next to the end follow (on a
// straight 3-point row the middle one is simply put in the middle)
static void Bevel_SetStripEnd( patchMesh_t *s, int end, float S[3][3], const bool *rowStraight ){
	const int n = s->width;
	const int k = end ? n - 1 : 0, ka = end ? n - 2 : 1;
	for ( int j = 0; j < 3; j++ )
	{
		vec3_t d;
		VectorSubtract( S[j], s->ctrl[k][j].xyz, d );
		VectorCopy( S[j], s->ctrl[k][j].xyz );
		if ( n == 3 && rowStraight[j] ) {
			Bevel_Mid3( s->ctrl[0][j].xyz, s->ctrl[2][j].xyz, s->ctrl[1][j].xyz );
		}
		else
		{
			for ( int c = 0; c < 3; c++ )
				s->ctrl[ka][j].xyz[c] += 0.5f * d[c];
		}
	}
	s->bDirty = true;
}

static bool Bevel_IsRecPatch( brush_t *b ){
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( g_bevelRecs[i].e.a == b || g_bevelRecs[i].e.b == b ) {
			return true;
		}
	return false;
}

static bool Bevel_IsStrip( brush_t *b ){
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( g_bevelRecs[i].strip == b ) {
			return true;
		}
	return false;
}

static bool Bevel_IsExtra( brush_t *b ){
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		for ( size_t j = 0; j < g_bevelRecs[i].extras.size(); j++ )
			if ( g_bevelRecs[i].extras[j].b == b ) {
				return true;
			}
	return false;
}

struct mitreMove_t
{
	brush_t *b;
	int ci, cj;
	vec3_t target;
};

static bool Bevel_IsCap( brush_t *b ){
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( g_bevelRecs[i].cap[0] == b || g_bevelRecs[i].cap[1] == b ) {
			return true;
		}
	return false;
}

// a face is about to be changed: it goes into the undo step, and (unless it is one of the bevel's own) is
// remembered with the record so that it can be put back when the bevel is edited or removed
// how every face touched in the current run of Bevel_Mitre was before the run
static std::map<brush_t*, std::vector<drawVert_t> > g_touchOrig;

static void Bevel_TouchFace( brush_t *b, bevelRec_t &owner, std::vector<brush_t*> &modified, bool useUndo ){
	bool known = false;
	for ( size_t k = 0; k < modified.size() && !known; k++ )
		known = modified[k] == b;
	if ( !known ) {
		if ( useUndo ) {
			Bevel_UndoAdd( b );
		}
		Bevel_Save( b->pPatch, g_touchOrig[b] );
		modified.push_back( b );
	}
	// The record that moved the face keeps how it was, to put it back when the bevel is edited or removed. A face
	// that is one of the record's own is put back with those, and a strip or cap piece is made again anyway.
	if ( b == owner.e.a || b == owner.e.b || b == owner.strip || b == owner.cap[0] || b == owner.cap[1] ) {
		return;
	}
	for ( size_t k = 0; k < owner.extras.size(); k++ )
		if ( owner.extras[k].b == b ) {
			return;
		}
	bevelExtra_t ex;
	ex.b = b;
	ex.orig = g_touchOrig[b];
	owner.extras.push_back( ex );
}

// ---- closing the cap at the end of a strip nothing else meets ------------------------------------------
// A strip that ends at a corner where no other beveled edge ends leaves the faces round that corner cut open:
// the two patches it joins were cut back along their borders, but the faces that touch the corner from the
// other side (the cap) still run into the old vertex. The cap faces round the corner are taken in order from the
// one next to the first patch to the one next to the second; the first ones are pulled to where the strip's
// end curve starts, the rest to where it ends, and the hole that is left, a triangle with the end curve on one
// side, is filled with a patch of its own.

struct capFace_t
{
	brush_t *b;
	int nc;                 // the corners of the face that are at the vertex (two for the point of a triangle)
	int ci[2], cj[2];
	vec3_t r[2];            // unit vectors along the two edges that leave the corner
	vec3_t farEnd[2];       // the other end of those edges
	float len[2];
	vec3_t nrm;             // the way the front of the face looks at the corner
};

// Cap pieces made here (see Bevel_CapEnd) that are about to be made again (a bevel is being edited): until that
// has happened the faces round them are as they were before the bevels, so these must not be looked at
static std::set<brush_t*> g_capStale;

static bool Bevel_CapFaceAt( brush_t *b, const float *V, float tol, capFace_t &f ){
	patchMesh_t *p = b->pPatch;
	if ( p->width != 3 || p->height != 3 ) {
		return false;
	}
	f.b = b;
	if ( Bevel_IsCap( b ) ) {
		// a piece made by Bevel_CapEnd: a triangle whose point is a collapsed row (or column) of the patch
		for ( int type = 0; type < 2; type++ )
			for ( int fix = 0; fix < 3; fix += 2 )
			{
				const float *pt[3];
				for ( int i = 0; i < 3; i++ )
					pt[i] = type == 0 ? p->ctrl[i][fix].xyz : p->ctrl[fix][i].xyz;
				vec3_t d1, d2, dv;
				VectorSubtract( pt[1], pt[0], d1 );
				VectorSubtract( pt[2], pt[0], d2 );
				VectorSubtract( pt[0], V, dv );
				if ( VectorLength( d1 ) > 0.05f || VectorLength( d2 ) > 0.05f || VectorLength( dv ) > tol ) {
					continue;
				}
				const int other = 2 - fix;
				const float *far0 = type == 0 ? p->ctrl[0][other].xyz : p->ctrl[other][0].xyz;
				const float *far1 = type == 0 ? p->ctrl[2][other].xyz : p->ctrl[other][2].xyz;
				f.nc = 2;
				f.ci[0] = type == 0 ? 0 : fix;
				f.cj[0] = type == 0 ? fix : 0;
				f.ci[1] = type == 0 ? 2 : fix;
				f.cj[1] = type == 0 ? fix : 2;
				VectorCopy( far0, f.farEnd[0] );
				VectorCopy( far1, f.farEnd[1] );
				for ( int k = 0; k < 2; k++ )
				{
					vec3_t d;
					VectorSubtract( f.farEnd[k], pt[0], d );
					f.len[k] = VectorLength( d );
					if ( !Bevel_Unit( d, f.r[k] ) ) {
						return false;
					}
				}
				vec3_t dU, dV, n;
				if ( type == 0 ) {
					VectorSubtract( p->ctrl[2][other].xyz, p->ctrl[0][other].xyz, dU );
					VectorSubtract( p->ctrl[1][2].xyz, p->ctrl[1][0].xyz, dV );
				}
				else
				{
					VectorSubtract( p->ctrl[2][1].xyz, p->ctrl[0][1].xyz, dU );
					VectorSubtract( p->ctrl[other][2].xyz, p->ctrl[other][0].xyz, dV );
				}
				CrossProduct( dU, dV, n );
				return Bevel_Unit( n, f.nrm );
			}
		return false;
	}
	float bestD = tol;
	bool found = false;
	for ( int i = 0; i < 3; i += 2 )
		for ( int j = 0; j < 3; j += 2 )
		{
			vec3_t d;
			VectorSubtract( p->ctrl[i][j].xyz, V, d );
			if ( VectorLength( d ) <= bestD ) {
				bestD = VectorLength( d );
				f.ci[0] = i;
				f.cj[0] = j;
				found = true;
			}
		}
	if ( !found ) {
		return false;
	}
	f.nc = 1;
	const int fci = f.ci[0], fcj = f.cj[0];
	const float *c = p->ctrl[fci][fcj].xyz;
	vec3_t d0, d1;
	VectorSubtract( p->ctrl[1][fcj].xyz, c, d0 );
	VectorSubtract( p->ctrl[fci][1].xyz, c, d1 );
	if ( !Bevel_Unit( d0, f.r[0] ) || !Bevel_Unit( d1, f.r[1] ) ) {
		return false;
	}
	VectorCopy( p->ctrl[2 - fci][fcj].xyz, f.farEnd[0] );
	VectorCopy( p->ctrl[fci][2 - fcj].xyz, f.farEnd[1] );
	for ( int k = 0; k < 2; k++ )
	{
		vec3_t d;
		VectorSubtract( f.farEnd[k], c, d );
		f.len[k] = VectorLength( d );
	}
	// dU and dV at the corner: the control point next to it is in the direction of increasing index only at index 0
	vec3_t dU, dV, n;
	VectorScale( d0, fci == 0 ? 1.0f : -1.0f, dU );
	VectorScale( d1, fcj == 0 ? 1.0f : -1.0f, dV );
	CrossProduct( dU, dV, n );
	return Bevel_Unit( n, f.nrm );
}

static void Bevel_StandInBounds( const patchMesh_t *p, float *lo, float *hi ){
	lo[0] = lo[1] = lo[2] = 1e30f;
	hi[0] = hi[1] = hi[2] = -1e30f;
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
			for ( int k = 0; k < 3; k++ )
			{
				const float v = p->ctrl[i][j].xyz[k];
				if ( v < lo[k] ) {
					lo[k] = v;
				}
				if ( v > hi[k] ) {
					hi[k] = v;
				}
			}
}

// the brush a patch sits on is never thin (see Cog_AddBrushForPatch)
static brush_t* Bevel_AddStandIn( patchMesh_t *pm ){
	float lo[3], hi[3];
	Bevel_StandInBounds( pm, lo, hi );
	for ( int k = 0; k < 3; k++ )
		if ( hi[k] - lo[k] < 8.0f ) {
			const float mid = 0.5f * ( lo[k] + hi[k] );
			lo[k] = mid - 4.0f;
			hi[k] = mid + 4.0f;
		}
	vec3_t vMin, vMax;
	VectorCopy( lo, vMin );
	VectorCopy( hi, vMax );
	brush_t *b = Brush_Create( vMin, vMax, &g_qeglobals.d_texturewin.texdef );
	b->patchBrush = true;
	b->pPatch = pm;
	pm->pSymbiot = b;
	pm->bSelected = false;
	pm->bOverlay = false;
	pm->bDirty = true;
	pm->nListID = -1;
	return b;
}

// A plane mapping from position to texture coordinates, fitted to the points of a flat patch (three points that
// span the most, the others are checked against it)
struct stFit_t
{
	bool ok;
	vec3_t a, e1, e2;
	float g11, g12, g22, det;
	float sa[2], sb[2], sc[2];
	stFit_t() : ok( false ){
	}
};

static void Bevel_FitST( const std::vector<drawVert_t> &pts, bool strict, stFit_t &f ){
	f.ok = false;
	if ( pts.size() < 3 ) {
		return;
	}
	const drawVert_t *a = &pts[0], *b = a, *c = NULL;
	float best = 0, bestC = 0;
	for ( size_t i = 0; i < pts.size(); i++ )
	{
		vec3_t d;
		VectorSubtract( pts[i].xyz, a->xyz, d );
		if ( VectorLength( d ) > best ) {
			best = VectorLength( d );
			b = &pts[i];
		}
	}
	VectorSubtract( b->xyz, a->xyz, f.e1 );
	for ( size_t i = 0; i < pts.size(); i++ )
	{
		vec3_t d, cr;
		VectorSubtract( pts[i].xyz, a->xyz, d );
		CrossProduct( f.e1, d, cr );
		if ( VectorLength( cr ) > bestC ) {
			bestC = VectorLength( cr );
			c = &pts[i];
		}
	}
	if ( c == NULL || best < 0.5f || bestC < 0.5f * best ) {
		return;
	}
	VectorCopy( a->xyz, f.a );
	VectorSubtract( c->xyz, a->xyz, f.e2 );
	f.g11 = DotProduct( f.e1, f.e1 );
	f.g12 = DotProduct( f.e1, f.e2 );
	f.g22 = DotProduct( f.e2, f.e2 );
	f.det = f.g11 * f.g22 - f.g12 * f.g12;
	if ( fabsf( f.det ) < 1e-6f ) {
		return;
	}
	for ( int k = 0; k < 2; k++ )
	{
		f.sa[k] = a->st[k];
		f.sb[k] = b->st[k] - a->st[k];
		f.sc[k] = c->st[k] - a->st[k];
	}
	f.ok = true;
	if ( strict ) {
		// every point must be on the plane and carry the texture coordinates the plane mapping gives
		for ( size_t i = 0; i < pts.size() && f.ok; i++ )
		{
			vec3_t d;
			VectorSubtract( pts[i].xyz, f.a, d );
			const float r1 = DotProduct( d, f.e1 ), r2 = DotProduct( d, f.e2 );
			const float al = ( r1 * f.g22 - r2 * f.g12 ) / f.det, be = ( r2 * f.g11 - r1 * f.g12 ) / f.det;
			vec3_t off;
			for ( int k = 0; k < 3; k++ )
				off[k] = d[k] - al * f.e1[k] - be * f.e2[k];
			if ( VectorLength( off ) > 0.05f + 0.002f * best ) {
				f.ok = false;
			}
			for ( int k = 0; k < 2; k++ )
				if ( fabsf( f.sa[k] + al * f.sb[k] + be * f.sc[k] - pts[i].st[k] ) > 0.01f ) {
					f.ok = false;
				}
		}
	}
}

static void Bevel_ApplyST( const stFit_t &f, patchMesh_t *p ){
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			vec3_t d;
			VectorSubtract( p->ctrl[i][j].xyz, f.a, d );
			const float r1 = DotProduct( d, f.e1 ), r2 = DotProduct( d, f.e2 );
			const float al = ( r1 * f.g22 - r2 * f.g12 ) / f.det, be = ( r2 * f.g11 - r1 * f.g12 ) / f.det;
			for ( int k = 0; k < 2; k++ )
				p->ctrl[i][j].st[k] = f.sa[k] + al * f.sb[k] + be * f.sc[k];
		}
}

// the texture coordinates of a flat patch as it was (`like`) carried over to the flat patch p
static void Bevel_CopyPlanarST( const std::vector<drawVert_t> &like, patchMesh_t *p ){
	stFit_t f;
	Bevel_FitST( like, false, f );
	if ( !f.ok ) {
		Patch_Naturalize( p );
		return;
	}
	Bevel_ApplyST( f, p );
}

// A face whose corners were moved keeps the texture it had: the planar mapping of how it was (when it was flat and
// mapped that way) is applied to where its points are now
static void Bevel_RefitST( brush_t *b ){
	std::map<brush_t*, std::vector<drawVert_t> >::iterator it = g_touchOrig.find( b );
	if ( it == g_touchOrig.end() || !b->pPatch ) {
		return;
	}
	stFit_t f;
	Bevel_FitST( it->second, true, f );
	if ( f.ok ) {
		Bevel_ApplyST( f, b->pPatch );
	}
}

// Returns true if a cap was made (or updated). The strip end is column `end` of strip `r`.
static bool Bevel_CapEnd( bevelRec_t &r, int end, std::vector<brush_t*> &modified, bool useUndo ){
	patchMesh_t *s = r.strip->pPatch;
	const int n = s->width;
	const int k = end ? n - 1 : 0, ka = end ? n - 2 : 1;
	vec3_t P0, P1, P2, V, u, d;
	VectorCopy( s->ctrl[k][0].xyz, P0 );
	VectorCopy( s->ctrl[k][1].xyz, P1 );
	VectorCopy( s->ctrl[k][2].xyz, P2 );
	VectorCopy( r.v0[end], V );
	VectorSubtract( s->ctrl[ka][1].xyz, P1, d );
	if ( !Bevel_Unit( d, u ) ) {
		return false;
	}
	vec3_t d0, d2, dir0, dir2;
	VectorSubtract( P0, V, d0 );
	VectorSubtract( P2, V, d2 );
	if ( !Bevel_Unit( d0, dir0 ) || !Bevel_Unit( d2, dir2 ) ) {
		return false;
	}
	const float tol = 0.4f;
	vec3_t nc, ncRaw;
	CrossProduct( dir0, dir2, ncRaw );
	const bool haveNc = VectorLength( ncRaw ) > 0.2f && Bevel_Unit( ncRaw, nc );

	std::vector<capFace_t> cand;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &selected_brushes : &active_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			if ( !b->patchBrush || !b->pPatch || b == r.strip || b == r.e.a || b == r.e.b || b == r.cap[end] || Bevel_IsStrip( b ) ) {
				continue;
			}
			if ( g_capStale.count( b ) ) {
				continue; // a piece of another corner that is about to be made again
			}
			capFace_t f;
			if ( !Bevel_CapFaceAt( b, V, tol, f ) ) {
				continue;
			}
			// a face of the cap lies in the plane of the end curve: the one through the vertex, P0 and P2 (or, when
			// those are in a line, across the strip)
			if ( haveNc ? fabsf( DotProduct( f.nrm, nc ) ) < 0.97f : fabsf( DotProduct( f.nrm, u ) ) < 0.85f ) {
				continue;
			}
			cand.push_back( f );
		}
	}
	if ( cand.empty() ) {
		return false;
	}

	// the faces in order, from the ray towards P0 round to the ray towards P2
	std::vector<int> chain, enter;
	std::vector<bool> used( cand.size(), false );
	vec3_t cur;
	VectorCopy( dir0, cur );
	bool done = false;
	for ( size_t step = 0; step < cand.size() && !done; step++ )
	{
		int pick = -1, q = 0;
		float bestDot = 0.97f;
		for ( size_t i = 0; i < cand.size(); i++ )
		{
			if ( used[i] ) {
				continue;
			}
			for ( int w = 0; w < 2; w++ )
			{
				const float dp = DotProduct( cand[i].r[w], cur );
				if ( dp > bestDot ) {
					bestDot = dp;
					pick = (int)i;
					q = w;
				}
			}
		}
		if ( pick < 0 ) {
			return false;
		}
		used[pick] = true;
		chain.push_back( pick );
		enter.push_back( q );
		VectorCopy( cand[pick].r[1 - q], cur );
		done = DotProduct( cur, dir2 ) > 0.97f;
	}
	if ( !done ) {
		return false;
	}
	const int m = (int)chain.size();

	// Where the faces are split: along the longest of the edges between two of them (on a cog that is the one to
	// the middle, so the new piece never ends on another corner of the rim, which may be beveled too)
	int split = 1;            // the first `split` faces go to P0, the rest to P2
	vec3_t X;
	if ( m >= 2 ) {
		float bestLen = -1.0f;
		for ( int j = 1; j < m; j++ )
		{
			const capFace_t &f = cand[chain[j - 1]];
			const int ex = 1 - enter[j - 1];
			if ( f.len[ex] > bestLen ) {
				bestLen = f.len[ex];
				split = j;
				VectorCopy( f.farEnd[ex], X );
			}
		}
	}
	else
	{
		const capFace_t &f = cand[chain[0]];
		const int en = enter[0], ex = 1 - en;
		if ( f.len[ex] <= f.len[en] ) {
			split = 1;
			VectorCopy( f.farEnd[ex], X );
		}
		else
		{
			split = 0;
			VectorCopy( f.farEnd[en], X );
		}
	}
	vec3_t pp;
	VectorSubtract( P2, P0, pp );
	if ( VectorLength( pp ) < 0.1f ) {
		return false;
	}

	// the faces
	for ( int i = 0; i < m; i++ )
	{
		const capFace_t &f = cand[chain[i]];
		Bevel_TouchFace( f.b, r, modified, useUndo );
		for ( int c = 0; c < f.nc; c++ )
			Bevel_MoveCorner( f.b->pPatch, f.ci[c], f.cj[c], i < split ? P0 : P2 );
	}

	// the piece in the hole: P0 .. strip end .. P2 on one side, the point X where the two groups of faces part on the other
	const capFace_t &first = cand[chain[0]];
	brush_t *fb = Bevel_BrushLive( r.cap[end] ) ? r.cap[end] : NULL;
	patchMesh_t *fp = fb ? fb->pPatch : MakeNewPatch();
	if ( !fb ) {
		fp->pShader = first.b->pPatch->pShader;
		fp->d_texture = first.b->pPatch->d_texture;
	}
	else if ( useUndo ) {
		Bevel_UndoAdd( fb );
	}
	fp->width = 3;
	fp->height = 3;
	fp->type = PATCH_GENERIC;
	fp->segH = fp->segV = 0;
	const float *row0[3] = { P0, P1, P2 };
	for ( int i = 0; i < 3; i++ )
	{
		VectorCopy( row0[i], fp->ctrl[i][0].xyz );
		VectorCopy( X, fp->ctrl[i][2].xyz );
		for ( int c = 0; c < 3; c++ )
			fp->ctrl[i][1].xyz[c] = 0.5f * ( row0[i][c] + X[c] );
	}
	{
		// the texture of the cap as it was before its corners were moved
		std::vector<drawVert_t> was;
		std::map<brush_t*, std::vector<drawVert_t> >::iterator wi = g_touchOrig.find( first.b );
		if ( wi != g_touchOrig.end() ) {
			was = wi->second;
		}
		else
		{
			Bevel_Save( first.b->pPatch, was );
		}
		Bevel_CopyPlanarST( was, fp );
	}
	// the edge it shares with the strip is cut into as many pieces as the strip is across, so the points meet
	if ( s->segV > 0 ) {
		fp->segH = SegPatch_ClampH( s->segV, SegPatch_MinH( fp ) );
		fp->segV = 1;
	}
	// it faces the way the cap does
	vec3_t dU, dV, fn;
	VectorSubtract( fp->ctrl[2][0].xyz, fp->ctrl[0][0].xyz, dU );
	VectorSubtract( fp->ctrl[1][2].xyz, fp->ctrl[1][0].xyz, dV );
	CrossProduct( dU, dV, fn );
	if ( DotProduct( fn, first.nrm ) < 0 ) {
		patchInvert( fp );
	}
	fp->bDirty = true;
	if ( !fb ) {
		fb = Bevel_AddStandIn( fp );
		Brush_AddToList( fb, &active_brushes );
		Entity_LinkBrush( r.e.a->owner, fb );
		Brush_Build( fb );
		r.cap[end] = fb;
	}
	Patch_Rebuild( fp );
	if ( useUndo ) {
		Undo_EndBrush( fb );
	}
	g_capStale.erase( fb );
	return true;
}

static void Bevel_Mitre( const std::vector<brush_t*> &movable, bool useUndo ){
	g_touchOrig.clear();
	std::vector<mitreEnd_t> ends;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
	{
		const bevelRec_t &r = g_bevelRecs[i];
		if ( !Bevel_BrushLive( r.strip ) || !Bevel_BrushLive( r.e.a ) || !Bevel_BrushLive( r.e.b ) ) {
			continue;
		}
		const patchMesh_t *s = r.strip->pPatch;
		const int n = s->width;
		if ( n < 3 || s->height != 3 ) {
			continue;
		}
		for ( int end = 0; end < 2; end++ )
		{
			const int k = end ? n - 1 : 0, ka = end ? n - 2 : 1;
			mitreEnd_t m;
			m.rec = i;
			m.end = end;
			m.width = r.width;
			VectorCopy( s->ctrl[k][1].xyz, m.p );
			for ( int j = 0; j < 3; j++ )
				m.rowStraight[j] = n != 3 || Bevel_Straight3( s->ctrl[0][j].xyz, s->ctrl[1][j].xyz, s->ctrl[2][j].xyz );
			vec3_t d;
			VectorSubtract( s->ctrl[ka][1].xyz, s->ctrl[k][1].xyz, d );
			VectorCopy( r.v0[end], m.v0 );
			if ( Bevel_Unit( d, m.u ) ) {
				ends.push_back( m );
			}
		}
	}

	// which strip end meets which: the nearest corner of two edge lines, if exactly one other end is there
	const size_t E = ends.size();
	std::vector<int> best( E, -1 );
	std::vector<float> bestV( 3 * E, 0.0f );
	for ( size_t a = 0; a < E; a++ )
	{
		const mitreEnd_t &m = ends[a];
		// the ends of other bevels that were at the same corner of the model before any bevel was made
		std::vector<size_t> cand;
		for ( size_t c = 0; c < E; c++ )
		{
			const mitreEnd_t &q = ends[c];
			if ( q.rec == m.rec ) {
				continue;
			}
			vec3_t dv;
			VectorSubtract( q.v0, m.v0, dv );
			if ( VectorLength( dv ) <= 0.5f ) {
				cand.push_back( c );
			}
		}
		if ( cand.empty() ) {
			continue;
		}
		if ( cand.size() > 1 ) {
			best[a] = -2; // a third edge ends at the same corner
			continue;
		}
		best[a] = (int)cand[0];
		for ( int k = 0; k < 3; k++ )
			bestV[3 * a + k] = 0.5f * ( m.v0[k] + ends[cand[0]].v0[k] );
	}

	std::vector<brush_t*> modified;      // patches (not strips) that were changed
	std::set<size_t> touched;            // recs whose strip changed
	int joined = 0;
	for ( size_t a = 0; a < E; a++ )
	{
		const int c = best[a];
		if ( c < 0 || (size_t)c < a || best[c] != (int)a ) {
			continue;       // not a corner of two, or handled from the other end
		}
		const mitreEnd_t &m = ends[a], &q = ends[c];
		brush_t *sbm = g_bevelRecs[m.rec].strip, *sbq = g_bevelRecs[q.rec].strip;
		bool mov = false;
		for ( size_t i = 0; i < movable.size() && !mov; i++ )
			mov = movable[i] == sbm || movable[i] == sbq;
		if ( !mov ) {
			continue;
		}
		vec3_t V;
		VectorCopy( &bestV[3 * a], V );
		vec3_t nrm, diff;
		VectorSubtract( m.u, q.u, diff );
		if ( !Bevel_Unit( diff, nrm ) ) {
			continue;
		}
		const float denM = DotProduct( nrm, m.u ), denQ = DotProduct( nrm, q.u );
		if ( fabsf( denM ) < 0.05f || fabsf( denQ ) < 0.05f ) {
			continue;
		}
		bevelRec_t &rm = g_bevelRecs[m.rec], &rq = g_bevelRecs[q.rec];
		patchMesh_t *sm = sbm->pPatch, *sq = sbq->pPatch;
		const int kM = m.end ? sm->width - 1 : 0, kQ = q.end ? sq->width - 1 : 0;

		// where each end column goes: onto the plane, along its strip
		float Pm[3][3], Pq[3][3];
		float far_ = 0.0f;
		for ( int j = 0; j < 3; j++ )
		{
			vec3_t rel, mv;
			VectorSubtract( sm->ctrl[kM][j].xyz, V, rel );
			VectorScale( m.u, -DotProduct( nrm, rel ) / denM, mv );
			VectorAdd( sm->ctrl[kM][j].xyz, mv, Pm[j] );
			far_ = Bevel_MaxF( far_, VectorLength( mv ) );
			VectorSubtract( sq->ctrl[kQ][j].xyz, V, rel );
			VectorScale( q.u, -DotProduct( nrm, rel ) / denQ, mv );
			VectorAdd( sq->ctrl[kQ][j].xyz, mv, Pq[j] );
			far_ = Bevel_MaxF( far_, VectorLength( mv ) );
		}
		if ( far_ > 5.0f * Bevel_MaxF( m.width, q.width ) + 1.0f ) {
			continue; // not a corner after all
		}
		// the two strips share one end curve; the side that is the patch A of one may be the patch B of the other
		vec3_t t0, t1;
		VectorSubtract( Pm[0], Pq[0], t0 );
		VectorSubtract( Pm[2], Pq[2], t1 );
		const float same = VectorLength( t0 ) + VectorLength( t1 );
		VectorSubtract( Pm[0], Pq[2], t0 );
		VectorSubtract( Pm[2], Pq[0], t1 );
		const float cross = VectorLength( t0 ) + VectorLength( t1 );
		const int map[3] = { cross < same ? 2 : 0, 1, cross < same ? 0 : 2 };
		float S[3][3], Sq[3][3];
		for ( int j = 0; j < 3; j++ )
			for ( int k = 0; k < 3; k++ )
				S[j][k] = 0.5f * ( Pm[j][k] + Pq[map[j]][k] );
		// Each row of a strip is a straight line along its edge, and the end of the strip slides along it. Where the
		// matching rows of the two strips cross (the borders of the faces they cut), that is the point to end at:
		// the faces then keep their borders on the lines they had. If the rows do not meet (the two strips are not
		// the same width) the average of the two projections stays.
		for ( int j = 0; j < 3; j++ )
		{
			vec3_t meet;
			float gap;
			if ( Bevel_LineMeet( sm->ctrl[kM][j].xyz, m.u, sq->ctrl[kQ][map[j]].xyz, q.u, meet, &gap )
				 && gap <= 0.25f * Bevel_MaxF( m.width, q.width ) + 0.3f ) {
				vec3_t dm;
				VectorSubtract( meet, S[j], dm );
				if ( VectorLength( dm ) <= 5.0f * Bevel_MaxF( m.width, q.width ) + 1.0f ) {
					VectorCopy( meet, S[j] );
				}
			}
		}
		for ( int j = 0; j < 3; j++ )
			for ( int k = 0; k < 3; k++ )
				Sq[map[j]][k] = S[j][k];
		Bevel_SetStripEnd( sm, m.end, S, m.rowStraight );
		Bevel_SetStripEnd( sq, q.end, Sq, q.rowStraight );
		touched.insert( m.rec );
		touched.insert( q.rec );

		// the faces next to the strips: their corner at the end of the strip goes to where the strip ends now
		std::vector<mitreMove_t> moves;
		for ( int who = 0; who < 2; who++ )
		{
			const bevelRec_t &r = who ? rq : rm;
			const int kcol = who ? kQ : kM;
			const int nn = ( who ? sq : sm )->width;
			for ( int row = 0; row < 3; row += 2 )
			{
				// "row" 0 is patch A, 2 is patch B; the row of the strip next to it depends on whether the strip was turned over
				const int strow = ( ( row == 2 ) != r.inv ) ? 2 : 0;
				brush_t *pb = row == 0 ? r.e.a : r.e.b;
				const int side = row == 0 ? r.e.sideA : r.e.sideB;
				const int k = row == 0 ? kcol : ( r.e.reversed ? nn - 1 - kcol : kcol );
				mitreMove_t mv;
				mv.b = pb;
				if ( !Bevel_CornerAt( pb->pPatch, side, k, &mv.ci, &mv.cj ) ) {
					continue;
				}
				const float *tg = who ? Sq[strow] : S[strow];
				VectorCopy( tg, mv.target );
				bool dup = false;
				for ( size_t i = 0; i < moves.size(); i++ )
					dup = dup || ( moves[i].b == mv.b && moves[i].ci == mv.ci && moves[i].cj == mv.cj );
				if ( !dup ) {
					moves.push_back( mv );
				}
			}
		}
		// faces that have a corner at the vertex itself and were not trimmed by any strip (the middle piece of a
		// fan, say): the corner goes to whichever of the two strip sides lies in the plane of the face
		for ( int list = 0; list < 2; list++ )
		{
			brush_t *head = list ? &selected_brushes : &active_brushes;
			for ( brush_t *b = head->next; b != head; b = b->next )
			{
				if ( !b->patchBrush || !b->pPatch || Bevel_IsStrip( b ) ) {
					continue;
				}
				patchMesh_t *p = b->pPatch;
				const int W = p->width, H = p->height;
				for ( int ci = 0; ci < W; ci += W - 1 )
					for ( int cj = 0; cj < H; cj += H - 1 )
					{
						vec3_t dv;
						VectorSubtract( p->ctrl[ci][cj].xyz, V, dv );
						if ( VectorLength( dv ) > 0.3f + 0.1f * Bevel_MaxF( m.width, q.width ) ) {
							continue;
						}
						bool dup = false;
						for ( size_t i = 0; i < moves.size(); i++ )
							dup = dup || ( moves[i].b == b && moves[i].ci == ci && moves[i].cj == cj );
						if ( dup ) {
							continue;
						}
						// the plane of the face at this corner
						const int ni = ci == 0 ? 1 : W - 2, nj = cj == 0 ? 1 : H - 2;
						vec3_t e1, e2, nn;
						VectorSubtract( p->ctrl[ni][cj].xyz, p->ctrl[ci][cj].xyz, e1 );
						VectorSubtract( p->ctrl[ci][nj].xyz, p->ctrl[ci][cj].xyz, e2 );
						CrossProduct( e1, e2, nn );
						vec3_t nu;
						if ( !Bevel_Unit( nn, nu ) ) {
							continue;
						}
						int pick = -1;
						float bestOff = 1e30f;
						for ( int row = 0; row < 3; row += 2 )
						{
							vec3_t sv;
							VectorSubtract( S[row], V, sv );
							const float off = fabsf( DotProduct( sv, nu ) );
							if ( off < bestOff && off <= 0.2f * VectorLength( sv ) + 0.05f ) {
								bestOff = off;
								pick = row;
							}
						}
						if ( pick < 0 ) {
							continue;
						}
						mitreMove_t mv;
						mv.b = b;
						mv.ci = ci;
						mv.cj = cj;
						VectorCopy( S[pick], mv.target );
						moves.push_back( mv );
					}
			}
		}
		for ( size_t i = 0; i < moves.size(); i++ )
		{
			brush_t *b = moves[i].b;
			// a face that is not one of the bevel's own is put back from here when the bevel is edited
			Bevel_TouchFace( b, rm, modified, useUndo );
			Bevel_MoveCorner( b->pPatch, moves[i].ci, moves[i].cj, moves[i].target );
		}
		joined++;
	}

	// strip ends that no other bevel meets: close the cap there
	int capped = 0, open = 0, crowded = 0;
	for ( size_t a = 0; a < E; a++ )
	{
		const mitreEnd_t &m = ends[a];
		if ( best[a] == -2 ) {
			crowded++;
			continue;
		}
		if ( best[a] >= 0 && best[best[a]] == (int)a ) {
			continue; // joined with another strip
		}
		bool mov = false;
		for ( size_t i = 0; i < movable.size() && !mov; i++ )
			mov = movable[i] == g_bevelRecs[m.rec].strip;
		if ( !mov ) {
			continue;
		}
		if ( Bevel_CapEnd( g_bevelRecs[m.rec], m.end, modified, useUndo ) ) {
			capped++;
		}
		else
		{
			open++;
		}
	}

	for ( size_t i = 0; i < modified.size(); i++ )
	{
		if ( !Bevel_IsStrip( modified[i] ) ) {
			Bevel_RefitST( modified[i] ); // the faces keep the texture they had, on their new outline
		}
		Patch_Rebuild( modified[i]->pPatch );
		if ( useUndo ) {
			Undo_EndBrush( modified[i] );
		}
	}
	for ( std::set<size_t>::iterator it = touched.begin(); it != touched.end(); ++it )
	{
		brush_t *sb = g_bevelRecs[*it].strip;
		sb->pPatch->bDirty = true;
		Patch_Rebuild( sb->pPatch );
		if ( useUndo ) {
			Undo_EndBrush( sb );
		}
	}
	// the records remember what the patches look like so that a change made by hand is noticed: they must
	// remember the corners as they are now
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
	{
		bevelRec_t &r = g_bevelRecs[i];
		if ( !Bevel_BrushLive( r.strip ) || !Bevel_BrushLive( r.e.a ) || !Bevel_BrushLive( r.e.b ) ) {
			continue;
		}
		for ( size_t k = 0; k < modified.size(); k++ )
		{
			if ( modified[k] == r.e.a ) {
				Bevel_Snap( r.e.a->pPatch, r.postA );
			}
			if ( modified[k] == r.e.b ) {
				Bevel_Snap( r.e.b->pPatch, r.postB );
			}
			if ( modified[k] == r.strip ) {
				Bevel_Snap( r.strip->pPatch, r.postS );
			}
		}
		if ( touched.count( i ) ) {
			Bevel_Snap( r.strip->pPatch, r.postS );
		}
	}
	if ( joined > 0 || capped > 0 || open > 0 || crowded > 0 ) {
		Sys_Printf( "Bevel Edge: %d corners mitred, %d strip ends capped, %d ends left open", joined, capped, open );
		if ( crowded > 0 ) {
			Sys_Printf( ", %d ends where three or more bevels meet (not joined)", crowded );
		}
		Sys_Printf( ".\n" );
	}
}

// the records remember what their patches and strip look like, so that a change made by hand is noticed. After
// bevels were made or changed they must remember the new state, of every record that shares a patch with them too
static void Bevel_Resnap( const std::vector<brush_t*> &strips ){
	std::set<brush_t*> patches, mine;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
	{
		const bevelRec_t &r = g_bevelRecs[i];
		if ( std::find( strips.begin(), strips.end(), r.strip ) != strips.end() && Bevel_BrushLive( r.strip ) && Bevel_BrushLive( r.e.a ) && Bevel_BrushLive( r.e.b ) ) {
			patches.insert( r.e.a );
			patches.insert( r.e.b );
			mine.insert( r.strip );
		}
	}
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
	{
		bevelRec_t &r = g_bevelRecs[i];
		if ( !Bevel_BrushLive( r.strip ) || !Bevel_BrushLive( r.e.a ) || !Bevel_BrushLive( r.e.b ) ) {
			continue;
		}
		if ( patches.count( r.e.a ) ) {
			Bevel_Snap( r.e.a->pPatch, r.postA );
		}
		if ( patches.count( r.e.b ) ) {
			Bevel_Snap( r.e.b->pPatch, r.postB );
		}
		if ( mine.count( r.strip ) || patches.count( r.strip ) ) {
			Bevel_Snap( r.strip->pPatch, r.postS );
		}
	}
}

// the way the front of a flat patch looks (cross(dU,dV), summed over the control cells) and a point of it; false if
// the patch is not flat
static bool Bevel_FlatFront( const patchMesh_t *p, float *n, float *c ){
	vec3_t sum = { 0, 0, 0 };
	for ( int i = 0; i + 1 < p->width; i++ )
		for ( int j = 0; j + 1 < p->height; j++ )
		{
			vec3_t du, dv, cr;
			VectorSubtract( p->ctrl[i + 1][j].xyz, p->ctrl[i][j].xyz, du );
			VectorSubtract( p->ctrl[i][j + 1].xyz, p->ctrl[i][j].xyz, dv );
			CrossProduct( du, dv, cr );
			VectorAdd( sum, cr, sum );
		}
	if ( !Bevel_Unit( sum, n ) ) {
		return false;
	}
	VectorCopy( p->ctrl[0][0].xyz, c );
	for ( int i = 0; i < p->width; i++ )
		for ( int j = 0; j < p->height; j++ )
		{
			vec3_t d;
			VectorSubtract( p->ctrl[i][j].xyz, c, d );
			if ( fabsf( DotProduct( d, n ) ) > 0.1f ) {
				return false;
			}
		}
	return true;
}

// A check for the console after bevels were made: patch borders near the bevels that have no border of another
// patch on top of them. In a closed model (a cog) each of those is a gap; on a surface with a real edge it is
// just that edge.
static void Bevel_ReportOpen( const std::vector<brush_t*> &strips ){
	float lo[3] = { 1e30f, 1e30f, 1e30f }, hi[3] = { -1e30f, -1e30f, -1e30f };
	int any = 0;
	for ( size_t i = 0; i < strips.size(); i++ )
	{
		if ( !Bevel_BrushLive( strips[i] ) ) {
			continue;
		}
		float mn[3], mx[3];
		Bevel_PatchBounds( strips[i]->pPatch, mn, mx );
		for ( int k = 0; k < 3; k++ )
		{
			lo[k] = mn[k] < lo[k] ? mn[k] : lo[k];
			hi[k] = mx[k] > hi[k] ? mx[k] : hi[k];
		}
		any++;
	}
	if ( !any ) {
		return;
	}
	std::vector<brush_t*> patches;
	Bevel_Patches( patches );
	bevelPt_t rim[MAX_PATCH_WIDTH], other[MAX_PATCH_WIDTH];
	int open = 0, flipped = 0;
	for ( size_t i = 0; i < patches.size(); i++ )
	{
		brush_t *b = patches[i];
		float mn[3], mx[3];
		Bevel_PatchBounds( b->pPatch, mn, mx );
		int k;
		for ( k = 0; k < 3; k++ )
			if ( mn[k] > hi[k] + 1.0f || mx[k] < lo[k] - 1.0f ) {
				break;
			}
		if ( k < 3 ) {
			continue;
		}
		for ( int side = 0; side < 4; side++ )
		{
			const int n = Bevel_Rim( b->pPatch, side, rim );
			if ( Bevel_RimDegenerate( rim, n ) ) {
				continue;
			}
			if ( side == 1 || side == 3 ) {
				bool rev;
				const int n0 = Bevel_Rim( b->pPatch, side - 1, other );
				if ( n0 == n && Bevel_RimsMatch( rim, other, n, &rev ) && !rev ) {
					continue; // a ring
				}
			}
			brush_t *pb;
			int sb;
			bool rev;
			if ( Bevel_FindPartnerTol( b, side, &pb, &sb, &rev, 0.3f ) ) {
				// two flat patches in one plane that face opposite ways are shaded differently
				vec3_t n1, n2, c1, c2;
				if ( b < pb && Bevel_FlatFront( b->pPatch, n1, c1 ) && Bevel_FlatFront( pb->pPatch, n2, c2 ) ) {
					vec3_t dc;
					VectorSubtract( c2, c1, dc );
					if ( fabsf( DotProduct( n1, n2 ) ) > 0.999f && fabsf( DotProduct( dc, n1 ) ) < 0.2f && DotProduct( n1, n2 ) < 0 ) {
						if ( flipped < 6 ) {
							Sys_Printf( "  facing differs: two flat patches in one plane at ( %.1f %.1f %.1f ) face opposite ways\n", c1[0], c1[1], c1[2] );
						}
						flipped++;
					}
				}
				continue;
			}
			// a border that sits on the border of a patch whose control points are laid out differently is not a gap
			// either, but there is no cheap test for it: the console shows where
			if ( open < 6 ) {
				Sys_Printf( "  open edge: ( %.1f %.1f %.1f ) - ( %.1f %.1f %.1f ), side %d of a patch with %dx%d points\n",
							rim[0].v[0], rim[0].v[1], rim[0].v[2], rim[n - 1].v[0], rim[n - 1].v[1], rim[n - 1].v[2],
							side, b->pPatch->width, b->pPatch->height );
			}
			open++;
		}
	}
	if ( open > 0 ) {
		Sys_Printf( "Bevel Edge: %d patch borders near the bevels have nothing joined to them (gaps, or a real edge of the model).\n", open );
	}
	if ( flipped > 0 ) {
		Sys_Printf( "Bevel Edge: %d pairs of flat patches in one plane face opposite ways.\n", flipped );
	}
}

// An edge that ends where it meets the seam between two bevel strips (the line along which two bevels of a
// corner join, on a model that is beveled already) goes on over that seam: the seam is part of the edge, as it
// would have been before the first bevel was made. Only where four edges meet, so there is no doubt which one.
static void Bevel_ExtendOverSeams( std::vector<bevelEdge_t> &edges ){
	std::vector<brush_t*> patches;
	Bevel_Patches( patches );
	for ( size_t i = 0; i < edges.size(); i++ )
	{
		if ( edges[i].a == NULL || edges[i].b == NULL ) {
			continue;
		}
		float ends[2][3];
		Bevel_Ends( edges[i], ends[0], ends[1] );
		for ( int end = 0; end < 2; end++ )
		{
			std::vector<bevelEdge_t> at;
			Bevel_EdgesAt( patches, ends[end], at );
			if ( at.size() != 4 ) {
				continue;
			}
			std::vector<bevelEdge_t> across;
			for ( size_t k = 0; k < at.size(); k++ )
				if ( !Bevel_SameEdge( at[k], edges[i] ) && !Bevel_EdgesShare( edges[i], at[k] ) ) {
					across.push_back( at[k] );
				}
			if ( across.size() != 1 || across[0].b == NULL || !Bevel_IsStrip( across[0].a ) || !Bevel_IsStrip( across[0].b ) ) {
				continue;
			}
			bool known = false;
			for ( size_t k = 0; k < edges.size() && !known; k++ )
				known = Bevel_SameEdge( edges[k], across[0] );
			if ( !known ) {
				edges.push_back( across[0] );
				Sys_Printf( "Bevel Edge: the edge goes on over the seam between two bevels.\n" );
			}
		}
	}
}

// bevel every picked edge; returns how many worked (the caller has started the undo step)
int BevelEdges_Apply( float width, int nsub ){
	std::vector<bevelEdge_t> edges = g_bevelEdges;
	Bevel_ExtendOverSeams( edges );
	std::vector<brush_t*> made;
	g_bevelUndone.clear();
	g_capStale.clear();
	g_bevelGen++;
	const float lim = Bevel_BatchLimit( edges );
	// the corners of every edge as they are before any of them is cut (a patch two edges share moves its corners)
	std::vector<float> corners( 6 * edges.size() + 1, 0.0f );
	for ( size_t i = 0; i < edges.size(); i++ )
		if ( edges[i].a && Bevel_BrushLive( edges[i].a ) ) {
			float c[2][3];
			Bevel_EdgeCorners( edges[i], c );
			memcpy( &corners[6 * i], c, sizeof( c ) );
		}
	for ( size_t i = 0; i < edges.size(); i++ )
	{
		brush_t *b = Bevel_Build( edges[i], width, nsub, NULL, true, lim, &corners[6 * i] );
		if ( b ) {
			made.push_back( b );
			g_bevelRecs.back().undoId = Undo_GetUndoId();
			g_bevelRecs.back().gen = g_bevelGen;
		}
		else
		{
			Sys_Printf( "Bevel Edge: an edge was skipped (it needs a patch on both sides with the same number of points, and room to cut back).\n" );
		}
	}
	Bevel_Mitre( made, true );
	Bevel_Resnap( made );
	Bevel_ReportOpen( made );
	BevelEdge_Clear();
	for ( size_t i = 0; i < made.size(); i++ )
		Select_Brush( made[i] );
	UpdatePatchInspector();
	return (int)made.size();
}

// the selected strip of a bevel that can still be edited, with its numbers; NULL if there is none
brush_t* BevelRec_Selected( float *width, float *maxW, int *nsub ){
	for ( size_t i = 0; i < g_bevelRecs.size(); )
	{
		if ( !Bevel_IsSelected( g_bevelRecs[i].strip ) ) {
			i++; // not looked at (and not checked) until it is selected
			continue;
		}
		if ( !Bevel_RecValid( g_bevelRecs[i] ) ) {
			const bevelRec_t &r = g_bevelRecs[i];
			Sys_Printf( "Bevel Edge: a selected bevel can no longer be adjusted (patches live %d/%d/%d, unchanged since it was made: %d/%d/%d).\n",
						Bevel_BrushLive( r.e.a ), Bevel_BrushLive( r.e.b ), Bevel_BrushLive( r.strip ),
						Bevel_BrushLive( r.e.a ) && Bevel_SnapSame( r.e.a->pPatch, r.postA ),
						Bevel_BrushLive( r.e.b ) && Bevel_SnapSame( r.e.b->pPatch, r.postB ),
						Bevel_BrushLive( r.strip ) && Bevel_SnapSame( r.strip->pPatch, r.postS ) );
			g_bevelRecs.erase( g_bevelRecs.begin() + i );
			continue;
		}
		{
			*width = g_bevelRecs[i].width;
			*maxW = g_bevelRecs[i].maxW;
			*nsub = g_bevelRecs[i].nsub;
			return g_bevelRecs[i].strip;
		}
	}
	return NULL;
}

static bool Bevel_RecLive( const bevelRec_t &r ){
	return Bevel_BrushLive( r.strip ) && Bevel_BrushLive( r.e.a ) && Bevel_BrushLive( r.e.b );
}

// Put the faces of some records (given in the order they were made) back to how they were before the first of them
// that touched each face: its patches, and the faces whose corners were moved to meet its strips.
static void Bevel_PutBackAll( const std::vector<bevelRec_t*> &recs, std::vector<brush_t*> &touchedFaces ){
	std::map<brush_t*, const std::vector<drawVert_t>*> first;
	std::vector<brush_t*> order;
	for ( size_t i = 0; i < recs.size(); i++ )
	{
		const bevelRec_t &r = *recs[i];
		if ( Bevel_BrushLive( r.e.a ) && !first.count( r.e.a ) ) {
			first[r.e.a] = &r.origA;
			order.push_back( r.e.a );
		}
		if ( Bevel_BrushLive( r.e.b ) && !first.count( r.e.b ) ) {
			first[r.e.b] = &r.origB;
			order.push_back( r.e.b );
		}
		for ( size_t k = 0; k < r.extras.size(); k++ )
			if ( Bevel_BrushLive( r.extras[k].b ) && !first.count( r.extras[k].b ) ) {
				first[r.extras[k].b] = &r.extras[k].orig;
				order.push_back( r.extras[k].b );
			}
	}
	for ( size_t i = 0; i < order.size(); i++ )
	{
		Bevel_Restore( order[i]->pPatch, *first[order[i]] );
		touchedFaces.push_back( order[i] );
	}
}

// redo the selected bevels with other numbers. Edits that follow each other right after the bevel
// (or each other) share one undo step. Bevels that share a patch with the edited ones are made again with their
// own numbers, since the patch is put back first.
int BevelRec_Edit( float width, int nsub ){
	std::vector<size_t> sel;
	std::vector<bool> own;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( Bevel_RecValid( g_bevelRecs[i] ) && Bevel_IsSelected( g_bevelRecs[i].strip ) ) {
			sel.push_back( i );
			own.push_back( true );
		}
	if ( sel.empty() ) {
		UpdatePatchInspector();
		Sys_UpdateWindows( W_ALL );
		return 0;
	}
	for ( bool grew = true; grew; )
	{
		grew = false;
		for ( size_t j = 0; j < g_bevelRecs.size(); j++ )
		{
			if ( !Bevel_RecLive( g_bevelRecs[j] ) || std::find( sel.begin(), sel.end(), j ) != sel.end() ) {
				continue;
			}
			for ( size_t k = 0; k < sel.size(); k++ )
			{
				const bevelRec_t &a = g_bevelRecs[j], &b = g_bevelRecs[sel[k]];
				// only bevels made after a selected one are pulled in: they were made on its result. The ones made
				// before it stay as they are (the selected one started from what they left).
				const bool sameBatch = a.gen == b.gen;
				if ( ( ( j > sel[k] || sameBatch ) && Bevel_Linked( a, b ) ) || ( sameBatch && Bevel_Meets( a, b ) ) ) {
					sel.push_back( j );
					own.push_back( sameBatch && own[k] ); // the rest of the batch is edited with it
					grew = true;
					break;
				}
			}
		}
	}
	// in the order they were made
	for ( size_t i = 0; i < sel.size(); i++ )
		for ( size_t j = i + 1; j < sel.size(); j++ )
			if ( sel[j] < sel[i] ) {
				std::swap( sel[i], sel[j] );
				const bool t = own[i];
				own[i] = own[j];
				own[j] = t;
			}

	const int top = Undo_GetUndoId();
	bool continuing = top != 0;
	for ( size_t i = 0; i < sel.size(); i++ )
		if ( g_bevelRecs[sel[i]].undoId != top ) {
			continuing = false;
		}
	const bool useUndo = !continuing;
	if ( useUndo ) {
		Undo_Start( "edit bevel" );
		g_bevelUndone.clear();
		for ( size_t i = 0; i < sel.size(); i++ )
		{
			bevelRec_t &r = g_bevelRecs[sel[i]];
			Bevel_UndoAdd( r.e.a );
			Bevel_UndoAdd( r.e.b );
			Bevel_UndoAdd( r.strip );
			for ( int c = 0; c < 2; c++ )
				if ( Bevel_BrushLive( r.cap[c] ) ) {
					Bevel_UndoAdd( r.cap[c] );
				}
			for ( size_t k = 0; k < r.extras.size(); k++ )
				if ( Bevel_BrushLive( r.extras[k].b ) ) {
					Bevel_UndoAdd( r.extras[k].b );
				}
		}
	}
	// Only the number of subdivisions changes (the width is the same): nothing but the strips and the cap pieces
	// that follow their count has to change, the rest of the model is left alone.
	{
		bool fast = true;
		for ( size_t i = 0; i < sel.size() && fast; i++ )
		{
			const bevelRec_t &r = g_bevelRecs[sel[i]];
			const float eff = width < r.maxW ? width : r.maxW;
			if ( !own[i] || fabsf( eff - r.width ) > 0.005f ) {
				fast = false;
			}
		}
		if ( fast ) {
			for ( size_t i = 0; i < sel.size(); i++ )
			{
				bevelRec_t &r = g_bevelRecs[sel[i]];
				patchMesh_t *s = r.strip->pPatch;
				s->segV = SegPatch_ClampV( nsub );
				r.nsub = s->segV;
				s->bDirty = true;
				Patch_Rebuild( s );
				if ( useUndo ) {
					Undo_EndBrush( r.strip );
				}
				for ( int c = 0; c < 2; c++ )
					if ( Bevel_BrushLive( r.cap[c] ) ) {
						patchMesh_t *fp = r.cap[c]->pPatch;
						fp->segH = SegPatch_ClampH( s->segV, SegPatch_MinH( fp ) );
						fp->segV = 1;
						fp->bDirty = true;
						Patch_Rebuild( fp );
						if ( useUndo ) {
							Undo_EndBrush( r.cap[c] );
						}
					}
				Bevel_Snap( s, r.postS );
			}
			if ( useUndo ) {
				const int id = Undo_GetUndoId();
				for ( size_t i = 0; i < sel.size(); i++ )
					g_bevelRecs[sel[i]].undoId = id;
				Undo_End();
			}
			Sys_Printf( "Bevel Edge: %d bevels changed to %d subdivisions.\n", (int)sel.size(), nsub );
			UpdatePatchInspector();
			Sys_UpdateWindows( W_ALL );
			return (int)sel.size();
		}
	}
	// everything back to before the bevels (each patch to how it was before the first of them that cut it)
	std::vector<brush_t*> faces;
	{
		std::vector<bevelRec_t*> recs;
		for ( size_t i = 0; i < sel.size(); i++ )
			recs.push_back( &g_bevelRecs[sel[i]] );
		Bevel_PutBackAll( recs, faces );
	}
	for ( size_t i = 0; i < sel.size(); i++ )
		g_bevelRecs[sel[i]].extras.clear();
	for ( size_t i = 0; i < faces.size(); i++ )
	{
		Patch_Rebuild( faces[i]->pPatch );
		if ( useUndo ) {
			Undo_EndBrush( faces[i] );
		}
	}

	int done = 0, limited = 0, failed = 0;
	std::vector<brush_t*> edited;
	std::vector<bevelEdge_t> ownEdges;
	for ( size_t i = 0; i < sel.size(); i++ )
		if ( own[i] ) {
			ownEdges.push_back( g_bevelRecs[sel[i]].e );
		}
	const float lim = Bevel_BatchLimit( ownEdges );
	g_capStale.clear();
	for ( size_t i = 0; i < sel.size(); i++ )
		for ( int c = 0; c < 2; c++ )
			if ( Bevel_BrushLive( g_bevelRecs[sel[i]].cap[c] ) ) {
				g_capStale.insert( g_bevelRecs[sel[i]].cap[c] );
			}
	// batch by batch, the way they were made: a later batch is made on the strips and cap pieces of an earlier one,
	// which have to be finished (corners and caps included) first
	std::set<int> gens;
	for ( size_t i = 0; i < sel.size(); i++ )
		gens.insert( g_bevelRecs[sel[i]].gen );
	for ( std::set<int>::iterator gi = gens.begin(); gi != gens.end(); ++gi )
	{
		std::vector<brush_t*> editedNow;
		for ( size_t i = 0; i < sel.size(); i++ )
		{
			bevelRec_t &r = g_bevelRecs[sel[i]];
			if ( r.gen != *gi ) {
				continue;
			}
			bevelEdge_t e = r.e;
			const float want = own[i] ? width : r.width;
			if ( Bevel_Build( e, want, own[i] ? nsub : r.nsub, &r, useUndo, own[i] ? lim : 0.0f ) ) {
				done++;
				editedNow.push_back( r.strip );
				if ( own[i] && r.width < want - 0.01f ) {
					limited++; // a short edge or a narrow patch: it has no room for the width asked
				}
			}
			else
			{
				failed++;
			}
		}
		Bevel_Mitre( editedNow, useUndo );
		for ( size_t i = 0; i < editedNow.size(); i++ )
			edited.push_back( editedNow[i] );
	}
	g_capStale.clear();
	{
		int selected = 0;
		for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
			if ( Bevel_BrushLive( g_bevelRecs[i].strip ) && Bevel_IsSelected( g_bevelRecs[i].strip ) ) {
				selected++;
			}
		Sys_Printf( "Bevel Edge: %d bevels edited (%d selected)", done, selected );
		if ( limited > 0 ) {
			Sys_Printf( ", %d of them narrower than asked because the edge has no room for more", limited );
		}
		if ( failed > 0 ) {
			Sys_Printf( ", %d could not be rebuilt", failed );
		}
		Sys_Printf( ".\n" );
	}
	Bevel_Resnap( edited );
	Bevel_ReportOpen( edited );
	if ( useUndo ) {
		const int id = Undo_GetUndoId();
		for ( size_t i = 0; i < sel.size(); i++ )
			g_bevelRecs[sel[i]].undoId = id;
		Undo_End();
	}
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	return done;
}

// Takes the selected bevels away: both patches go back to how they were before the bevel, the strip is
// deleted, and the edge is picked again, so the bevel can be made again from the panel. One undo step.
int BevelRec_Remove(){
	std::vector<bevelRec_t> todo;
	std::vector<size_t> idx;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( Bevel_RecValid( g_bevelRecs[i] ) && Bevel_IsSelected( g_bevelRecs[i].strip ) ) {
			idx.push_back( i );
		}
	// bevels made later on the strips or cap pieces of these go with them
	for ( bool grew = true; grew && !idx.empty(); )
	{
		grew = false;
		for ( size_t j = 0; j < g_bevelRecs.size(); j++ )
		{
			if ( !Bevel_RecLive( g_bevelRecs[j] ) || std::find( idx.begin(), idx.end(), j ) != idx.end() ) {
				continue;
			}
			for ( size_t k = 0; k < idx.size(); k++ )
				if ( j > idx[k] && Bevel_Linked( g_bevelRecs[j], g_bevelRecs[idx[k]] ) ) {
					idx.push_back( j );
					grew = true;
					break;
				}
		}
	}
	std::sort( idx.begin(), idx.end() );
	for ( size_t i = 0; i < idx.size(); i++ )
		todo.push_back( g_bevelRecs[idx[i]] );
	if ( todo.empty() ) {
		return 0;
	}
	for ( size_t i = 0; i < g_bevelRecs.size(); )
	{
		bool gone = false;
		for ( size_t j = 0; j < todo.size() && !gone; j++ )
			gone = g_bevelRecs[i].strip == todo[j].strip;
		if ( gone ) {
			g_bevelRecs.erase( g_bevelRecs.begin() + i );
		}
		else
		{
			i++;
		}
	}

	Undo_Start( "remove bevel" );
	g_bevelUndone.clear();
	for ( size_t i = 0; i < todo.size(); i++ )
	{
		Bevel_UndoAdd( todo[i].e.a );
		Bevel_UndoAdd( todo[i].e.b );
		for ( size_t k = 0; k < todo[i].extras.size(); k++ )
			if ( Bevel_BrushLive( todo[i].extras[k].b ) ) {
				Bevel_UndoAdd( todo[i].extras[k].b );
			}
	}
	std::vector<brush_t*> faces;
	{
		std::vector<bevelRec_t*> recs;
		for ( size_t i = 0; i < todo.size(); i++ )
			recs.push_back( &todo[i] ); // each patch goes back to how it was before the first of them that cut it
		Bevel_PutBackAll( recs, faces );
	}
	for ( size_t i = 0; i < faces.size(); i++ )
	{
		Patch_Rebuild( faces[i]->pPatch );
		Undo_EndBrush( faces[i] );
	}

	// delete the strips the way Delete does it
	Select_Deselect( true );
	for ( size_t i = 0; i < todo.size(); i++ )
	{
		Select_Brush( todo[i].strip );
		for ( int c = 0; c < 2; c++ )
			if ( Bevel_BrushLive( todo[i].cap[c] ) && !Bevel_IsSelected( todo[i].cap[c] ) ) {
				Select_Brush( todo[i].cap[c] );
			}
	}
	Undo_AddBrushList( &selected_brushes );
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		Undo_AddEntity( b->owner );
	Select_Delete();
	Undo_EndBrushList( &selected_brushes );
	Undo_End();

	g_bevelEdges.clear();
	for ( size_t i = 0; i < todo.size(); i++ )
		g_bevelEdges.push_back( todo[i].e );
	g_lastPickValid = false;
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	return (int)todo.size();
}
