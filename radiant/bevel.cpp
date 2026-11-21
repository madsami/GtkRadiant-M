/*
   Bevel from edge: finding and picking the seams between patches (see bevel.h).
 */
#include "stdafx.h"
#include <vector>
#include <math.h>
#include "bevel.h"
#include "segpatch.h"

#define BEVEL_PICK_EPSILON 8        // pick radius in pixels, like the vertex / edge picking

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

// The edge the loop goes on into, coming along cur to the point v. nAt is how many edges meet at v and
// nCand how many of them could carry the loop on (for the message when it stops).
static bool Bevel_NextAround( const std::vector<brush_t*> &patches, const bevelEdge_t &cur, const float *v, bevelEdge_t *next, int *nAt, int *nCand ){
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
	for ( int dir = 0; dir < 2 && !closed; dir++ )
	{
		bevelEdge_t cur = start;
		float v[3];
		VectorCopy( dir == 0 ? s1 : s0, v );
		for ( int guard = 0; guard < BEVEL_LOOP_MAX; guard++ )
		{
			bevelEdge_t next;
			int nAt, nCand;
			if ( !Bevel_NextAround( patches, cur, v, &next, &nAt, &nCand ) ) {
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

// the largest width that works for every picked edge that can be beveled; 0 if none can
float BevelEdge_MaxWidth(){
	float best = 0;
	bool any = false;
	for ( size_t i = 0; i < g_bevelEdges.size(); i++ )
	{
		float w;
		if ( Bevel_EdgeReach( g_bevelEdges[i], &w ) && ( !any || w < best ) ) {
			best = w;
			any = true;
		}
	}
	return any ? best : 0.0f;
}

// Every bevel made in this session is remembered: the control points of both patches as they were
// before, and a snapshot of everything as the bevel left it. While the snapshot still matches, the
// bevel can be redone with other numbers. If anything was moved or edited since (or an undo replaced
// the patches), the record is dropped and the bevel is just ordinary patches again.
struct bevelRec_t
{
	bevelEdge_t e;
	brush_t *strip;
	std::vector<drawVert_t> origA, origB;
	std::vector<float> postA, postB, postS;
	float width, maxW;
	int nsub;
	int undoId;
};

static std::vector<bevelRec_t> g_bevelRecs;

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

static bool Bevel_IsSelected( brush_t *b ){
	for ( brush_t *p = selected_brushes.next; p != &selected_brushes; p = p->next )
		if ( p == b ) {
			return true;
		}
	return false;
}

// Cuts both patches back and fills the gap. With a record the patches are first put back as they were
// and the strip that is already there is rebuilt in place; without one a new strip is made (and
// remembered). useUndo: save the brushes for undo (the caller has started the undo).
static brush_t* Bevel_Build( const bevelEdge_t &e, float width, int nsub, bevelRec_t *rec, bool useUndo ){
	if ( e.a == NULL || e.b == NULL || e.a == e.b ) {
		return NULL; // a free edge has no second patch to bevel towards
	}
	brush_t *A = e.a, *B = e.b;
	patchMesh_t *pa = A->pPatch, *pb = B->pPatch;

	std::vector<drawVert_t> oA, oB;
	if ( rec ) {
		oA = rec->origA;
		oB = rec->origB;
	}
	else
	{
		Bevel_Save( pa, oA );
		Bevel_Save( pb, oB );
	}
	if ( useUndo ) {
		Undo_AddBrush( A );
		Undo_AddBrush( B );
		if ( rec ) {
			Undo_AddBrush( rec->strip );
		}
	}
	if ( rec ) {
		Bevel_Restore( pa, oA );
		Bevel_Restore( pb, oB );
	}

	float maxW;
	if ( !Bevel_EdgeReach( e, &maxW ) ) {
		return NULL;
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
	if ( DotProduct( faceS, faceA ) < 0 ) {
		patchInvert( s );
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
		along_count = SEGPATCH_DEFAULT_H;
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
		r.origA = oA;
		r.origB = oB;
		r.undoId = 0;
	}
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

// bevel every picked edge; returns how many worked
int BevelEdges_Apply( float width, int nsub ){
	std::vector<bevelEdge_t> edges = g_bevelEdges;
	std::vector<brush_t*> made;
	for ( size_t i = 0; i < edges.size(); i++ )
	{
		brush_t *b = Bevel_Build( edges[i], width, nsub, NULL, true );
		if ( b ) {
			made.push_back( b );
			g_bevelRecs.back().undoId = Undo_GetUndoId();
		}
		else
		{
			Sys_Printf( "Bevel Edge: an edge was skipped (it needs a patch on both sides with the same number of points, and room to cut back).\n" );
		}
	}
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

// redo the selected bevels with other numbers. Edits that follow each other right after the bevel
// (or each other) share one undo step.
int BevelRec_Edit( float width, int nsub ){
	int done = 0;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
	{
		bevelRec_t &r = g_bevelRecs[i];
		if ( !Bevel_RecValid( r ) || !Bevel_IsSelected( r.strip ) ) {
			continue;
		}
		const int top = Undo_GetUndoId();
		const bool fresh = !( top != 0 && r.undoId == top );
		if ( fresh ) {
			Undo_Start( "edit bevel" );
		}
		bevelEdge_t e = r.e;
		if ( Bevel_Build( e, width, nsub, &r, fresh ) ) {
			done++;
		}
		if ( fresh ) {
			r.undoId = Undo_GetUndoId();
			Undo_End();
		}
	}
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	return done;
}

// Takes the selected bevels away: both patches go back to how they were before the bevel, the strip is
// deleted, and the edge is picked again, so the bevel can be made again from the panel. One undo step.
int BevelRec_Remove(){
	std::vector<bevelRec_t> todo;
	for ( size_t i = 0; i < g_bevelRecs.size(); i++ )
		if ( Bevel_RecValid( g_bevelRecs[i] ) && Bevel_IsSelected( g_bevelRecs[i].strip ) ) {
			todo.push_back( g_bevelRecs[i] );
		}
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
	for ( size_t i = 0; i < todo.size(); i++ )
	{
		const bevelRec_t &r = todo[i];
		Undo_AddBrush( r.e.a );
		Undo_AddBrush( r.e.b );
		Bevel_Restore( r.e.a->pPatch, r.origA );
		Bevel_Restore( r.e.b->pPatch, r.origB );
		Patch_Rebuild( r.e.a->pPatch );
		Patch_Rebuild( r.e.b->pPatch );
		Undo_EndBrush( r.e.a );
		Undo_EndBrush( r.e.b );
	}

	// delete the strips the way Delete does it
	Select_Deselect( true );
	for ( size_t i = 0; i < todo.size(); i++ )
		Select_Brush( todo[i].strip );
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
