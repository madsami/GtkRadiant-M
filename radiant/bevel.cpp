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

static bool Bevel_RimsMatch( const bevelPt_t *a, const bevelPt_t *b, int n, bool *reversed ){
	for ( int dir = 0; dir < 2; dir++ )
	{
		bool ok = true;
		for ( int k = 0; k < n && ok; k++ )
		{
			vec3_t d;
			VectorSubtract( a[k].v, b[dir ? n - 1 - k : k].v, d );
			if ( VectorLength( d ) > 0.5f ) {
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

// the other patch that has a border on top of this one, if any
static bool Bevel_FindPartner( brush_t *A, int sa, brush_t **outB, int *outSb, bool *outRev ){
	std::vector<brush_t*> list;
	Bevel_Patches( list );
	bevelPt_t ra[MAX_PATCH_WIDTH], rb[MAX_PATCH_WIDTH];
	int na = Bevel_Rim( A->pPatch, sa, ra );
	for ( size_t j = 0; j < list.size(); j++ )
	{
		brush_t *B = list[j];
		if ( B == A ) {
			continue;
		}
		int k;
		for ( k = 0; k < 3; k++ )
			if ( A->mins[k] > B->maxs[k] + 1 || B->mins[k] > A->maxs[k] + 1 ) {
				break;
			}
		if ( k < 3 ) {
			continue; // too far apart to touch
		}
		for ( int sb = 0; sb < 4; sb++ )
		{
			int nb = Bevel_Rim( B->pPatch, sb, rb );
			bool rev;
			if ( na == nb && Bevel_RimsMatch( ra, rb, na, &rev ) ) {
				*outB = B;
				*outSb = sb;
				*outRev = rev;
				return true;
			}
		}
	}
	return false;
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

// Alt+click: pick the patch border the ray passes nearest to, whether another patch continues from
// it or not. additive (Ctrl+Alt) adds it to the picked ones, or takes it out again if it is already
// one. False if no border is near the ray, so the click can do what it did before.
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

	brush_t *bestBrush = NULL;
	int bestSide = -1;
	float bestScore = 1e30f;
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
				const float score = dist / tol + t * 1e-6f; // the closest to the ray, the nearer one on a tie
				if ( score < bestScore ) {
					bestScore = score;
					bestBrush = patches[i];
					bestSide = side;
				}
			}
		}
	}
	if ( bestBrush == NULL ) {
		return false;
	}

	bevelEdge_t e = { bestBrush, bestSide, NULL, 0, false };
	if ( Bevel_FindPartner( bestBrush, bestSide, &e.b, &e.sideB, &e.reversed ) && e.b < e.a ) {
		// the same edge is always stored the same way round, whichever patch it was picked from
		bevelEdge_t s = { e.b, e.sideB, e.a, e.sideA, e.reversed };
		e = s;
	}

	bool removed = false;
	if ( additive ) {
		for ( size_t i = 0; i < g_bevelEdges.size(); i++ )
			if ( Bevel_SameEdge( g_bevelEdges[i], e ) ) {
				g_bevelEdges.erase( g_bevelEdges.begin() + i );
				removed = true;
				break;
			}
	}
	else
	{
		g_bevelEdges.clear();
	}
	if ( !removed ) {
		g_bevelEdges.push_back( e );
	}
	Sys_Printf( "Edge %s: %s, %d control points (%d picked)\n", removed ? "taken out" : "picked",
				e.b ? "shared with another patch" : "free edge",
				( e.sideA < 2 ) ? e.a->pPatch->height : e.a->pPatch->width, (int)g_bevelEdges.size() );
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
