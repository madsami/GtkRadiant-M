/*
   Seat cut: holes in segmented patches that follow a segmented cutter (see seatcut.h).
 */
#include "stdafx.h"
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <math.h>
#include <string.h>
#include "segpatch.h"
#include "seatcut.h"
#include "patchgroup.h"
#include "straight.h"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

const segGrid_t* Patch_GetSegGrid( patchMesh_t *p ); // pmesh.cpp

// The hole is a little smaller than the hull of the cutter, so that what is left of the host lies inside the
// cutter, hidden, and no hairline gap can open between the two.
#define SEAT_INSET 0.05f

// =============================================================================
// convex hull

struct SeatV3
{
	float v[3];
};

struct SeatFace
{
	int v[3];
	float n[3], d;
	bool dead;
};

static float Seat_Dot( const float *a, const float *b ){
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void Seat_Sub( const float *a, const float *b, float *o ){
	o[0] = a[0] - b[0];
	o[1] = a[1] - b[1];
	o[2] = a[2] - b[2];
}

static void Seat_Cross( const float *a, const float *b, float *o ){
	o[0] = a[1] * b[2] - a[2] * b[1];
	o[1] = a[2] * b[0] - a[0] * b[2];
	o[2] = a[0] * b[1] - a[1] * b[0];
}

static void Seat_FacePlane( const std::vector<SeatV3> &P, SeatFace &f ){
	float e1[3], e2[3];
	Seat_Sub( P[f.v[1]].v, P[f.v[0]].v, e1 );
	Seat_Sub( P[f.v[2]].v, P[f.v[0]].v, e2 );
	Seat_Cross( e1, e2, f.n );
	float l = sqrtf( Seat_Dot( f.n, f.n ) );
	if ( l < 1e-7f ) {
		f.n[0] = f.n[1] = f.n[2] = 0;
		f.d = 0;
		return;
	}
	f.n[0] /= l;
	f.n[1] /= l;
	f.n[2] /= l;
	f.d = -Seat_Dot( f.n, P[f.v[0]].v );
}

// the faces of the convex hull of the points (outward normals, triangles); false if the points are flat
static bool Seat_Hull( const std::vector<SeatV3> &P, std::vector<SeatFace> &faces ){
	const int N = (int)P.size();
	if ( N < 4 ) {
		return false;
	}
	const float eps = 0.005f;
	int i0 = 0;
	for ( int i = 1; i < N; i++ )
		if ( P[i].v[0] < P[i0].v[0] ) {
			i0 = i;
		}
	int i1 = i0;
	float best = 0;
	for ( int i = 0; i < N; i++ )
	{
		float d[3];
		Seat_Sub( P[i].v, P[i0].v, d );
		float l = Seat_Dot( d, d );
		if ( l > best ) {
			best = l;
			i1 = i;
		}
	}
	if ( best < 1e-2f ) {
		return false;
	}
	int i2 = -1;
	best = 0;
	float ax[3];
	Seat_Sub( P[i1].v, P[i0].v, ax );
	for ( int i = 0; i < N; i++ )
	{
		float d[3], c[3];
		Seat_Sub( P[i].v, P[i0].v, d );
		Seat_Cross( ax, d, c );
		float l = Seat_Dot( c, c );
		if ( l > best ) {
			best = l;
			i2 = i;
		}
	}
	if ( i2 < 0 || best < 1e-2f ) {
		return false;
	}
	SeatFace base;
	base.v[0] = i0;
	base.v[1] = i1;
	base.v[2] = i2;
	base.dead = false;
	Seat_FacePlane( P, base );
	int i3 = -1;
	best = 0;
	for ( int i = 0; i < N; i++ )
	{
		float d = fabsf( Seat_Dot( base.n, P[i].v ) + base.d );
		if ( d > best ) {
			best = d;
			i3 = i;
		}
	}
	if ( i3 < 0 || best < 0.05f ) {
		return false;
	}

	faces.clear();
	if ( Seat_Dot( base.n, P[i3].v ) + base.d > 0 ) {
		std::swap( base.v[1], base.v[2] ); // the fourth point has to be behind the first face
	}
	SeatFace f;
	f.dead = false;
	int tet[4][3] = { { base.v[0], base.v[1], base.v[2] }, { base.v[0], i3, base.v[1] }, { base.v[1], i3, base.v[2] }, { base.v[2], i3, base.v[0] } };
	for ( int k = 0; k < 4; k++ )
	{
		f.v[0] = tet[k][0];
		f.v[1] = tet[k][1];
		f.v[2] = tet[k][2];
		Seat_FacePlane( P, f );
		faces.push_back( f );
	}

	for ( int q = 0; q < N; q++ )
	{
		if ( q == i0 || q == i1 || q == i2 || q == i3 ) {
			continue;
		}
		std::vector<int> vis;
		for ( size_t k = 0; k < faces.size(); k++ )
			if ( Seat_Dot( faces[k].n, P[q].v ) + faces[k].d > eps ) {
				vis.push_back( (int)k );
			}
		if ( vis.empty() ) {
			continue;
		}
		std::set<std::pair<int, int> > edges;
		for ( size_t k = 0; k < vis.size(); k++ )
		{
			const SeatFace &vf = faces[vis[k]];
			for ( int e = 0; e < 3; e++ )
				edges.insert( std::make_pair( vf.v[e], vf.v[( e + 1 ) % 3] ) );
		}
		std::vector<SeatFace> next;
		next.reserve( faces.size() + 8 );
		std::vector<char> gone( faces.size(), 0 );
		for ( size_t k = 0; k < vis.size(); k++ )
			gone[vis[k]] = 1;
		for ( size_t k = 0; k < faces.size(); k++ )
			if ( !gone[k] ) {
				next.push_back( faces[k] );
			}
		for ( size_t k = 0; k < vis.size(); k++ )
		{
			const SeatFace &vf = faces[vis[k]];
			for ( int e = 0; e < 3; e++ )
			{
				const int a = vf.v[e], b = vf.v[( e + 1 ) % 3];
				if ( edges.count( std::make_pair( b, a ) ) ) {
					continue; // an edge inside the part that is covered
				}
				SeatFace nf;
				nf.v[0] = a;
				nf.v[1] = b;
				nf.v[2] = q;
				nf.dead = false;
				Seat_FacePlane( P, nf );
				next.push_back( nf );
			}
		}
		faces.swap( next );
	}
	return true;
}

// the hull as planes, coplanar faces made one; false if there are more than SEAT_MAX_PLANES
static bool Seat_HullPlanes( const std::vector<SeatV3> &P, seatHull_t &h, std::vector<float> *edges = NULL ){
	std::vector<SeatFace> faces;
	if ( !Seat_Hull( P, faces ) ) {
		return false;
	}
	if ( edges ) {
		// the edges of the hull where its surface really bends (not the diagonals inside one flat face), as line pairs
		edges->clear();
		std::map<std::pair<int, int>, int> dir;
		for ( size_t k = 0; k < faces.size(); k++ )
			for ( int e = 0; e < 3; e++ )
				dir[std::make_pair( faces[k].v[e], faces[k].v[( e + 1 ) % 3] )] = (int)k;
		for ( std::map<std::pair<int, int>, int>::iterator it = dir.begin(); it != dir.end(); ++it )
		{
			const int a = it->first.first, b = it->first.second;
			if ( a > b ) {
				continue;
			}
			std::map<std::pair<int, int>, int>::iterator ot = dir.find( std::make_pair( b, a ) );
			if ( ot == dir.end() ) {
				continue;
			}
			if ( Seat_Dot( faces[it->second].n, faces[ot->second].n ) < 0.9999f ) {
				for ( int k = 0; k < 3; k++ )
					edges->push_back( P[a].v[k] );
				for ( int k = 0; k < 3; k++ )
					edges->push_back( P[b].v[k] );
			}
		}
	}
	const float dots[3] = { 0.99995f, 0.9995f, 0.995f };
	for ( int pass = 0; pass < 3; pass++ )
	{
		std::vector<SeatFace> kept;
		for ( size_t k = 0; k < faces.size(); k++ )
		{
			const SeatFace &f = faces[k];
			if ( Seat_Dot( f.n, f.n ) < 0.5f ) {
				continue;
			}
			bool same = false;
			for ( size_t j = 0; j < kept.size() && !same; j++ )
				if ( Seat_Dot( f.n, kept[j].n ) > dots[pass] && fabsf( f.d - kept[j].d ) < ( pass ? 0.5f : 0.02f ) ) {
					same = true;
				}
			if ( !same ) {
				kept.push_back( f );
			}
		}
		if ( (int)kept.size() <= SEAT_MAX_PLANES ) {
			h.n = (int)kept.size();
			for ( int k = 0; k < h.n; k++ )
			{
				h.pl[k][0] = kept[k].n[0];
				h.pl[k][1] = kept[k].n[1];
				h.pl[k][2] = kept[k].n[2];
				h.pl[k][3] = kept[k].d + SEAT_INSET; // a little inside
			}
			return true;
		}
	}
	return false;
}

// =============================================================================
// the cutters and their hosts

static std::vector<patchMesh_t*> Seat_AllPatches(){
	std::vector<patchMesh_t*> v;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( b->patchBrush && b->pPatch ) {
				v.push_back( b->pPatch );
			}
	}
	return v;
}

static bool Seat_SameHulls( const patchMesh_t *p, const std::vector<seatHull_t> &hs ){
	if ( p->seatCount != (int)hs.size() ) {
		return false;
	}
	for ( int i = 0; i < p->seatCount; i++ )
	{
		if ( p->seat[i].id != hs[i].id || p->seat[i].n != hs[i].n ) {
			return false;
		}
		if ( memcmp( p->seat[i].pl, hs[i].pl, sizeof( float ) * 4 * hs[i].n ) != 0 ) {
			return false;
		}
	}
	return true;
}

// the edges of the hull of every cutter (by cutter id), for the outline that shows where the cut is
static std::map<int, std::vector<float> > g_seatCue;

static bool Seat_Apply( bool report ){
	g_seatCue.clear();
	std::vector<patchMesh_t*> all = Seat_AllPatches();
	std::map<int, std::vector<patchMesh_t*> > cutters;
	for ( size_t i = 0; i < all.size(); i++ )
		if ( all[i]->seatId > 0 && all[i]->segH > 0 ) {
			cutters[all[i]->seatId].push_back( all[i] );
		}

	// the grids and bounds of the possible hosts
	struct Host
	{
		patchMesh_t *p;
		const segGrid_t *g;
		float mins[3], maxs[3];
		std::vector<seatHull_t> hulls;
	};
	std::vector<Host> hosts;
	for ( size_t i = 0; i < all.size(); i++ )
	{
		Host h;
		h.p = all[i];
		h.g = NULL;
		// every segmented patch can be cut, a cutter too (by the cutters that come before it), unless it was
		// switched off ("Can be cut")
		if ( all[i]->segH > 0 && all[i]->seatCanCut ) {
			h.g = Patch_GetSegGrid( all[i] );
		}
		if ( h.g ) {
			for ( int k = 0; k < 3; k++ )
			{
				h.mins[k] = 1e30f;
				h.maxs[k] = -1e30f;
			}
			for ( size_t n = 0; n < h.g->p.size(); n++ )
				for ( int k = 0; k < 3; k++ )
				{
					h.mins[k] = std::min( h.mins[k], h.g->p[n].xyz[k] );
					h.maxs[k] = std::max( h.maxs[k], h.g->p[n].xyz[k] );
				}
		}
		hosts.push_back( h );
	}

	for ( std::map<int, std::vector<patchMesh_t*> >::iterator it = cutters.begin(); it != cutters.end(); ++it )
	{
		std::vector<SeatV3> pts;
		float mins[3] = { 1e30f, 1e30f, 1e30f }, maxs[3] = { -1e30f, -1e30f, -1e30f };
		for ( size_t i = 0; i < it->second.size(); i++ )
		{
			const segGrid_t *g = Patch_GetSegGrid( it->second[i] );
			if ( !g ) {
				continue;
			}
			for ( size_t n = 0; n < g->p.size(); n++ )
			{
				SeatV3 v;
				for ( int k = 0; k < 3; k++ )
				{
					v.v[k] = g->p[n].xyz[k];
					mins[k] = std::min( mins[k], v.v[k] );
					maxs[k] = std::max( maxs[k], v.v[k] );
				}
				pts.push_back( v );
			}
		}
		seatHull_t hull;
		memset( &hull, 0, sizeof( hull ) );
		hull.id = it->first;
		if ( !Seat_HullPlanes( pts, hull, &g_seatCue[it->first] ) ) {
			g_seatCue.erase( it->first );
			if ( report ) {
				Sys_Printf( "Seat cut: the cutter %d has no usable hull (flat, or more than %d faces); lower its segments.\n", it->first, SEAT_MAX_PLANES );
			}
			continue;
		}
		int cuts = 0;
		for ( size_t i = 0; i < hosts.size(); i++ )
		{
			Host &h = hosts[i];
			if ( !h.g ) {
				continue;
			}
			// A cutter cuts the patches that are not cutters and those of the cutters that come after it. A cutter
			// never cuts its own patches, and two cutters never cut each other (that would leave a hole in both).
			if ( h.p->seatId > 0 && h.p->seatId <= it->first ) {
				continue;
			}
			bool apart = false;
			for ( int k = 0; k < 3; k++ )
				if ( h.mins[k] > maxs[k] || h.maxs[k] < mins[k] ) {
					apart = true;
				}
			if ( apart ) {
				continue;
			}
			segCut_t r;
			SegPatch_Cut( *h.g, &hull, 1, r );
			if ( r.removed == 0 && r.frags.empty() ) {
				continue;
			}
			if ( (int)h.hulls.size() < SEAT_MAX_HULLS ) {
				h.hulls.push_back( hull );
				cuts++;
			}
			else if ( report ) {
				Sys_Printf( "Seat cut: a patch is reached by more than %d cutters, the cutter %d is left out there.\n", SEAT_MAX_HULLS, it->first );
			}
		}
	}

	bool changed = false;
	for ( size_t i = 0; i < hosts.size(); i++ )
	{
		patchMesh_t *p = hosts[i].p;
		if ( Seat_SameHulls( p, hosts[i].hulls ) ) {
			continue;
		}
		p->seatCount = (int)hosts[i].hulls.size();
		for ( int k = 0; k < p->seatCount; k++ )
			p->seat[k] = hosts[i].hulls[k];
		p->bDirty = true;
		changed = true;
	}
	return changed;
}

void Seat_Recut(){
	if ( Seat_Apply( true ) ) {
		Sys_UpdateWindows( W_ALL );
	}
}

// =============================================================================
// following the cutters

static void Seat_Mix( unsigned long long &h, unsigned int v ){
	h ^= v;
	h *= 1099511628211ULL;
}

static void Seat_MixFloat( unsigned long long &h, float f ){
	unsigned int u;
	memcpy( &u, &f, 4 );
	Seat_Mix( h, u );
}

// everything that decides where the holes are: the control points and counts of every segmented patch
static unsigned long long Seat_StateHash( bool *any ){
	unsigned long long h = 14695981039346656037ULL;
	*any = false;
	std::vector<patchMesh_t*> all = Seat_AllPatches();
	for ( size_t i = 0; i < all.size(); i++ )
	{
		const patchMesh_t *p = all[i];
		if ( p->seatId > 0 || p->seatCount > 0 ) {
			*any = true;
		}
		if ( p->segH <= 0 ) {
			continue;
		}
		Seat_Mix( h, (unsigned int)( p->seatId * 2 + ( p->seatCanCut ? 1 : 0 ) ) );
		Seat_Mix( h, (unsigned int)( p->segH * 256 + p->segV ) );
		Seat_Mix( h, (unsigned int)( p->width * 256 + p->height ) );
		for ( int x = 0; x < p->width; x++ )
			for ( int y = 0; y < p->height; y++ )
				for ( int k = 0; k < 3; k++ )
					Seat_MixFloat( h, p->ctrl[x][y].xyz[k] );
	}
	return h;
}

static unsigned long long g_seatLastHash = 0;
static int g_seatStable = 0;
static bool g_seatPending = false;

// Every 100 ms. When something that matters has changed, the holes are worked out again once it has been
// left alone for three ticks (0.3 s), so dragging a cutter around does not redo them at every step.
static gboolean Seat_Tick( gpointer ){
	if ( Straight_Active() ) {
		return TRUE; // the cutters are in their straight pose: the holes are worked out again on leaving
	}
	bool any;
	unsigned long long h = Seat_StateHash( &any );
	if ( !any ) {
		g_seatLastHash = h;
		g_seatPending = false;
		return TRUE;
	}
	if ( h != g_seatLastHash ) {
		g_seatLastHash = h;
		g_seatStable = 0;
		g_seatPending = true;
		return TRUE;
	}
	if ( g_seatPending && ++g_seatStable >= 3 ) {
		g_seatPending = false;
		MapModifiedKeep keep; // the holes follow from the cutters (after a load, say): working them out again is not an edit
		Seat_Recut();
	}
	return TRUE;
}

void Seat_Init(){
	g_timeout_add( 100, Seat_Tick, NULL );
}

// =============================================================================
// commands

static int Seat_TopId(){
	int top = 0;
	std::vector<patchMesh_t*> all = Seat_AllPatches();
	for ( size_t i = 0; i < all.size(); i++ )
		top = std::max( top, all[i]->seatId );
	return top;
}

// =============================================================================
// Extract Cut Piece: what the cutter takes out of the host, kept as patches of their own

// The part of the grid that lies inside the hull: the cells that are inside as a whole (put together in rectangles, the
// grid points are the surface) and the polygons of the cells that the hull crosses. The complement of SegPatch_Cut.
static void Seat_Inside( const segGrid_t &g, const seatHull_t &h, std::vector<segRect_t> &rects, std::vector<segFrag_t> &frags ){
	const int CU = g.U - 1, CV = g.V - 1;
	if ( CU < 1 || CV < 1 || h.n < 1 ) {
		return;
	}
	const float eps = 0.001f;
	const float cuv[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
	std::vector<char> in( CU * CV, 0 ), used( CU * CV, 0 );
	for ( int u = 0; u < CU; u++ )
	{
		for ( int v = 0; v < CV; v++ )
		{
			const segPoint_t *c[4] = { &g.p[u * g.V + v], &g.p[( u + 1 ) * g.V + v], &g.p[( u + 1 ) * g.V + v + 1], &g.p[u * g.V + v + 1] };
			bool apart = false, inside = true;
			for ( int k = 0; k < h.n; k++ )
			{
				int outCount = 0, inCount = 0;
				for ( int q = 0; q < 4; q++ )
				{
					if ( SegPatch_PlaneDist( h.pl[k], c[q]->xyz ) > eps ) {
						outCount++;
					}
					else{
						inCount++;
					}
				}
				if ( outCount == 4 ) {
					apart = true;
					break;
				}
				if ( inCount < 4 ) {
					inside = false;
				}
			}
			if ( apart ) {
				continue;
			}
			if ( inside ) {
				in[u * CV + v] = 1;
				continue;
			}
			segPoly_t rest( 4 );
			for ( int q = 0; q < 4; q++ )
			{
				for ( int k = 0; k < 3; k++ )
					rest[q].xyz[k] = c[q]->xyz[k];
				rest[q].st[0] = c[q]->st[0];
				rest[q].st[1] = c[q]->st[1];
				rest[q].cu = u + cuv[q][0];
				rest[q].cv = v + cuv[q][1];
			}
			for ( int k = 0; k < h.n && rest.size() >= 3; k++ )
			{
				segPoly_t o, i;
				SegPatch_SplitPoly( rest, h.pl[k], o, i );
				rest.swap( i );
			}
			if ( rest.size() >= 3 && SegPatch_PolyArea( rest ) > 0.01f ) {
				segFrag_t f;
				f.cu = u;
				f.cv = v;
				f.poly = rest;
				frags.push_back( f );
			}
		}
	}
	for ( int v = 0; v < CV; v++ )
	{
		for ( int u = 0; u < CU; u++ )
		{
			if ( !in[u * CV + v] || used[u * CV + v] ) {
				continue;
			}
			int u2 = u;
			while ( u2 < CU && in[u2 * CV + v] && !used[u2 * CV + v] )
				u2++;
			int v2 = v + 1;
			for ( ; v2 < CV; v2++ )
			{
				bool full = true;
				for ( int x = u; x < u2 && full; x++ )
					if ( !in[x * CV + v2] || used[x * CV + v2] ) {
						full = false;
					}
				if ( !full ) {
					break;
				}
			}
			for ( int x = u; x < u2; x++ )
				for ( int y = v; y < v2; y++ )
					used[x * CV + y] = 1;
			segRect_t rc = { u, u2, v, v2 };
			rects.push_back( rc );
		}
	}
}

// the grid as a surface: the point (and texture coordinate) at the grid coordinates cu (along the width) and cv, as
// SegVert_t carries them (cell index + fraction); a ring wraps round
static void Seat_GridEval( const segGrid_t &g, bool ring, float cu, float cv, float *xyz, float *st ){
	const int CU = g.U - 1, CV = g.V - 1;
	if ( ring ) {
		cu = fmodf( cu, (float)CU );
		if ( cu < 0 ) {
			cu += CU;
		}
	}
	cu = cu < 0 ? 0 : ( cu > CU ? (float)CU : cu );
	cv = cv < 0 ? 0 : ( cv > CV ? (float)CV : cv );
	int iu = (int)cu, iv = (int)cv;
	if ( iu > CU - 1 ) {
		iu = CU - 1;
	}
	if ( iv > CV - 1 ) {
		iv = CV - 1;
	}
	const float fu = cu - iu, fv = cv - iv;
	const segPoint_t &a = g.p[iu * g.V + iv], &b = g.p[( iu + 1 ) * g.V + iv], &c = g.p[( iu + 1 ) * g.V + iv + 1], &d = g.p[iu * g.V + iv + 1];
	const float wa = ( 1 - fu ) * ( 1 - fv ), wb = fu * ( 1 - fv ), wc = fu * fv, wd = ( 1 - fu ) * fv;
	for ( int k = 0; k < 3; k++ )
		xyz[k] = wa * a.xyz[k] + wb * b.xyz[k] + wc * c.xyz[k] + wd * d.xyz[k];
	for ( int k = 0; k < 2; k++ )
		st[k] = wa * a.st[k] + wb * b.st[k] + wc * c.st[k] + wd * d.st[k];
}

static bool Seat_GridRing( const segGrid_t &g ){
	if ( g.U < 4 ) {
		return false;
	}
	for ( int v = 0; v < g.V; v++ )
	{
		const float *a = g.p[v].xyz, *b = g.p[( g.U - 1 ) * g.V + v].xyz;
		if ( fabsf( a[0] - b[0] ) + fabsf( a[1] - b[1] ) + fabsf( a[2] - b[2] ) > 0.05f ) {
			return false;
		}
	}
	return true;
}

// The piece with the topology of the hole: the outline of the hole is the line where the hull cuts the host, and it bends
// only where the cutter has a wireframe edge (the corners; between two of them the cut runs straight, which is how the hole
// itself is cut). The corners are joined to a middle point, so a round cutter gives a fan of equal wedges, one for every
// side of the cutter, the way a cap is made. Every wedge is a patch whose outer edge is the straight line between two
// corners, exactly the edge of the hole, and whose inner rows follow the surface of the host. False if the outline is not
// a closed line with corners (the hole runs off the host, say).
static bool Seat_Cookie( const segGrid_t &g, const seatHull_t &h, const std::vector<segFrag_t> &frags, std::vector<segPiece_t> &out ){
	struct Edge
	{
		segVert_t a, b;
		bool used;
	};
	std::vector<Edge> edges;
	for ( size_t f = 0; f < frags.size(); f++ )
	{
		const segPoly_t &p = frags[f].poly;
		for ( size_t i = 0; i < p.size(); i++ )
		{
			const segVert_t &a = p[i], &b = p[( i + 1 ) % p.size()];
			for ( int k = 0; k < h.n; k++ )
				if ( fabsf( SegPatch_PlaneDist( h.pl[k], a.xyz ) ) < 0.02f && fabsf( SegPatch_PlaneDist( h.pl[k], b.xyz ) ) < 0.02f ) {
					Edge e;
					e.a = a;
					e.b = b;
					e.used = false;
					edges.push_back( e );
					break;
				}
		}
	}
	if ( edges.empty() ) {
		return false;
	}
	const bool ring = Seat_GridRing( g );
	const int CU = g.U - 1;
	std::vector<std::vector<segVert_t> > loops;
	for ( size_t s = 0; s < edges.size(); s++ )
	{
		if ( edges[s].used ) {
			continue;
		}
		std::vector<segVert_t> loop;
		loop.push_back( edges[s].a );
		edges[s].used = true;
		size_t cur = s;
		for ( ;; )
		{
			const segVert_t end = edges[cur].b;
			if ( SegPatch_SamePoint( end.xyz, loop[0].xyz ) || fabsf( end.xyz[0] - loop[0].xyz[0] ) + fabsf( end.xyz[1] - loop[0].xyz[1] ) + fabsf( end.xyz[2] - loop[0].xyz[2] ) < 0.02f ) {
				break;
			}
			loop.push_back( end );
			int next = -1;
			for ( size_t j = 0; j < edges.size() && next < 0; j++ )
				if ( !edges[j].used && fabsf( edges[j].a.xyz[0] - end.xyz[0] ) + fabsf( edges[j].a.xyz[1] - end.xyz[1] ) + fabsf( edges[j].a.xyz[2] - end.xyz[2] ) < 0.02f ) {
					next = (int)j;
				}
			if ( next < 0 ) {
				return false; // not closed
			}
			edges[next].used = true;
			cur = next;
		}
		loops.push_back( loop );
	}
	std::vector<segPiece_t> made;
	for ( size_t l = 0; l < loops.size(); l++ )
	{
		std::vector<segVert_t> corner;
		for ( size_t i = 0; i < loops[l].size(); i++ )
			if ( SegPatch_OnPlanes( h, loops[l][i].xyz ) >= 2 ) {
				corner.push_back( loops[l][i] );
			}
		if ( corner.size() < 3 ) {
			return false;
		}
		if ( ring ) {
			// a hole across the seam of a ring: the grid coordinates run on past the end
			float lo = 1e30f, hi = -1e30f;
			for ( size_t i = 0; i < corner.size(); i++ )
			{
				lo = corner[i].cu < lo ? corner[i].cu : lo;
				hi = corner[i].cu > hi ? corner[i].cu : hi;
			}
			if ( hi - lo > 0.5f * CU ) {
				for ( size_t i = 0; i < corner.size(); i++ )
					if ( corner[i].cu < 0.5f * CU ) {
						corner[i].cu += CU;
					}
			}
		}
		const int m = (int)corner.size();
		float ccu = 0, ccv = 0;
		for ( int i = 0; i < m; i++ )
		{
			ccu += corner[i].cu;
			ccv += corner[i].cv;
		}
		ccu /= m;
		ccv /= m;
		float C[3], Cst[2];
		Seat_GridEval( g, ring, ccu, ccv, C, Cst );
		for ( int i = 0; i < m; i++ )
		{
			const segVert_t &va = corner[i], &vb = corner[( i + 1 ) % m];
			float B[3][3], Bst[3][2], Gcu[3], Gcv[3];
			for ( int k = 0; k < 3; k++ )
			{
				B[0][k] = va.xyz[k];
				B[2][k] = vb.xyz[k];
				B[1][k] = 0.5f * ( va.xyz[k] + vb.xyz[k] );
			}
			for ( int k = 0; k < 2; k++ )
			{
				Bst[0][k] = va.st[k];
				Bst[2][k] = vb.st[k];
				Bst[1][k] = 0.5f * ( va.st[k] + vb.st[k] );
			}
			Gcu[0] = va.cu;
			Gcu[2] = vb.cu;
			Gcu[1] = 0.5f * ( va.cu + vb.cu );
			Gcv[0] = va.cv;
			Gcv[2] = vb.cv;
			Gcv[1] = 0.5f * ( va.cv + vb.cv );
			segPiece_t pc;
			pc.width = pc.height = 3;
			pc.c.resize( 9 );
			for ( int x = 0; x < 3; x++ )
			{
				// the middle row: the curve from the edge to the middle point passes through the surface half way
				float S[3], Sst[2];
				Seat_GridEval( g, ring, 0.5f * ( Gcu[x] + ccu ), 0.5f * ( Gcv[x] + ccv ), S, Sst );
				segPoint_t &p0 = pc.c[x * 3 + 0], &p1 = pc.c[x * 3 + 1], &p2 = pc.c[x * 3 + 2];
				for ( int k = 0; k < 3; k++ )
				{
					p0.xyz[k] = B[x][k];
					p2.xyz[k] = C[k];
					p1.xyz[k] = 2.0f * S[k] - 0.5f * ( B[x][k] + C[k] );
				}
				for ( int k = 0; k < 2; k++ )
				{
					p0.st[k] = Bst[x][k];
					p2.st[k] = Cst[k];
					p1.st[k] = 2.0f * Sst[k] - 0.5f * ( Bst[x][k] + Cst[k] );
				}
			}
			made.push_back( pc );
		}
	}
	out.insert( out.end(), made.begin(), made.end() );
	return true;
}

// Curve > Extract Cut Piece: the selected patches are the cutter. Where its hull cuts a hole in a host, what was taken
// out is made again as ordinary patches (one group, facing and textured like the host). They are copies: the hole stays
// as it is, because it follows the cutter, so the piece can be moved, turned or used as a hatch without touching it.
int Seat_ExtractPiece(){
	std::set<int> ids;
	std::set<patchMesh_t*> selP;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch ) {
			selP.insert( b->pPatch );
			if ( b->pPatch->seatId > 0 ) {
				ids.insert( b->pPatch->seatId );
			}
		}
	if ( ids.empty() ) {
		Sys_Printf( "Extract Cut Piece: select the cutter (a patch of it) first. A patch that is selected as well as the cutter limits the piece to that host.\n" );
		return 0;
	}
	Seat_Apply( false ); // the holes as the cutter is now
	std::vector<patchMesh_t*> all = Seat_AllPatches();
	std::vector<patchMesh_t*> hosts;
	std::vector<int> hullOf;
	bool limited = false;
	for ( size_t i = 0; i < all.size(); i++ )
	{
		patchMesh_t *p = all[i];
		if ( p->segH <= 0 || !p->pSymbiot ) {
			continue;
		}
		for ( int k = 0; k < p->seatCount; k++ )
			if ( ids.count( p->seat[k].id ) ) {
				hosts.push_back( p );
				hullOf.push_back( k );
				if ( selP.count( p ) ) {
					limited = true;
				}
			}
	}
	std::vector<segPiece_t> pieces;
	std::vector<patchMesh_t*> pieceHost;
	int nHosts = 0;
	bool simple = false;
	for ( size_t i = 0; i < hosts.size(); i++ )
	{
		patchMesh_t *host = hosts[i];
		if ( limited && !selP.count( host ) ) {
			continue;
		}
		const segGrid_t *g = Patch_GetSegGrid( host );
		if ( !g ) {
			continue;
		}
		std::vector<segRect_t> rects;
		std::vector<segFrag_t> frags;
		Seat_Inside( *g, host->seat[hullOf[i]], rects, frags );
		const size_t before = pieces.size();
		if ( !Seat_Cookie( *g, host->seat[hullOf[i]], frags, pieces ) ) {
			// the outline of the hole could not be followed: the cells inside the hull are copied as they are
			simple = true;
			for ( size_t r = 0; r < rects.size(); r++ )
				SegPatch_RectPieces( *g, rects[r].u0, rects[r].u1, rects[r].v0, rects[r].v1, pieces );
			for ( size_t f = 0; f < frags.size(); f++ )
				SegPatch_FragPieces( frags[f], pieces );
		}
		if ( pieces.size() > before ) {
			nHosts++;
		}
		pieceHost.resize( pieces.size(), host );
	}
	if ( pieces.empty() ) {
		Sys_Printf( "Extract Cut Piece: the cutter has not cut a hole in anything (a host has to be segmented, with Can be cut on).\n" );
		return 0;
	}

	const int gid = pieces.size() > 1 ? PatchGroup_NextId() : 0;
	std::vector<brush_t*> made;
	for ( size_t i = 0; i < pieces.size(); i++ )
	{
		const segPiece_t &pc = pieces[i];
		patchMesh_t *host = pieceHost[i];
		if ( pc.width > MAX_PATCH_WIDTH || pc.height > MAX_PATCH_HEIGHT ) {
			continue;
		}
		patchMesh_t *s = MakeNewPatch();
		s->pShader = host->pShader;
		s->d_texture = host->d_texture;
		s->contents = host->contents;
		s->flags = host->flags;
		s->value = host->value;
		s->width = pc.width;
		s->height = pc.height;
		s->type = PATCH_GENERIC;
		for ( int x = 0; x < pc.width; x++ )
			for ( int y = 0; y < pc.height; y++ )
			{
				const segPoint_t &c = pc.c[x * pc.height + y];
				VectorCopy( c.xyz, s->ctrl[x][y].xyz );
				s->ctrl[x][y].st[0] = c.st[0];
				s->ctrl[x][y].st[1] = c.st[1];
			}
		s->groupId = gid;
		s->hasFrame = host->hasFrame;
		memcpy( s->frameR, host->frameR, sizeof( s->frameR ) );
		brush_t *sb = AddBrushForPatch( s, false );
		Brush_AddToList( sb, &active_brushes );
		Entity_LinkBrush( host->pSymbiot->owner ? host->pSymbiot->owner : world_entity, sb );
		Brush_Build( sb );
		s->bDirty = true;
		Patch_Rebuild( s );
		Undo_EndBrush( sb );
		made.push_back( sb );
	}
	Select_Deselect();
	for ( size_t i = 0; i < made.size(); i++ )
		Select_Brush( made[i] );
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	Sys_Printf( "Extract Cut Piece: %d patch(es) from %d host patch(es), %s%s. The hole stays where the cutter is.\n", (int)made.size(), nHosts,
				gid ? "one group (move it with the selection)" : "selected",
				simple ? "; the outline of a hole could not be followed there, so the cells inside it were copied as they are" : "" );
	return (int)made.size();
}

bool Seat_MakeSelected(){
	std::vector<patchMesh_t*> sel;
	int id = 0, skipped = 0;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
	{
		if ( !b->patchBrush || !b->pPatch ) {
			continue;
		}
		if ( b->pPatch->segH <= 0 ) {
			skipped++;
			continue;
		}
		sel.push_back( b->pPatch );
		if ( id == 0 && b->pPatch->seatId > 0 ) {
			id = b->pPatch->seatId; // patches added to a cutter that is already there join it
		}
	}
	if ( sel.empty() ) {
		Sys_Printf( "Seat Cut: select the segmented patches of the cutter first.\n" );
		return false;
	}
	if ( id == 0 ) {
		id = Seat_TopId() + 1;
	}
	for ( size_t i = 0; i < sel.size(); i++ )
		sel[i]->seatId = id;
	if ( skipped ) {
		Sys_Printf( "Seat Cut: %d patch(es) that are not segmented were left out.\n", skipped );
	}
	Seat_Apply( true );
	Sys_UpdateWindows( W_ALL );
	return true;
}

// what the panel shows: the first selected segmented patch
bool Seat_GetSelected( bool *isCutter, bool *canBeCut ){
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->segH > 0 ) {
			*isCutter = b->pPatch->seatId > 0;
			*canBeCut = b->pPatch->seatCanCut != 0;
			return true;
		}
	return false;
}

// The Cut box of the panel. On: the selected patches that are no cutter yet become one (a single patch its own
// cutter; several selected at once one cutter together). Nothing about a patch changes but that. Off: they stop.
bool Seat_SetCutterSelected( bool on ){
	std::vector<patchMesh_t*> sel;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->segH > 0 ) {
			sel.push_back( b->pPatch );
		}
	if ( sel.empty() ) {
		return false;
	}
	int id = 0;
	for ( size_t i = 0; i < sel.size(); i++ )
	{
		if ( !on ) {
			sel[i]->seatId = 0;
		}
		else if ( sel[i]->seatId == 0 ) {
			if ( id == 0 ) {
				id = Seat_TopId() + 1;
			}
			sel[i]->seatId = id;
		}
	}
	Seat_Apply( true );
	Sys_UpdateWindows( W_ALL );
	return true;
}

// The Can be cut box: off keeps every cutter from cutting a hole into the selected patches
bool Seat_SetCanBeCutSelected( bool can ){
	bool any = false;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->segH > 0 ) {
			b->pPatch->seatCanCut = can ? 1 : 0;
			any = true;
		}
	if ( any ) {
		Seat_Apply( true );
		Sys_UpdateWindows( W_ALL );
	}
	return any;
}

bool Seat_ClearSelected(){
	int n = 0;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->seatId > 0 ) {
			b->pPatch->seatId = 0;
			n++;
		}
	if ( !n ) {
		Sys_Printf( "Clear Seat Cut: select patches of a cutter first.\n" );
		return false;
	}
	Seat_Apply( true );
	Sys_UpdateWindows( W_ALL );
	Sys_Printf( "Clear Seat Cut: %d patch(es) are no cutter any more.\n", n );
	return true;
}

int Seat_SelectWhole(){
	if ( Straight_Active() ) {
		return 0; // the rest of the map is out of reach while Straight Edit is on
	}
	std::vector<int> ids;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->seatId > 0 && std::find( ids.begin(), ids.end(), b->pPatch->seatId ) == ids.end() ) {
			ids.push_back( b->pPatch->seatId );
		}
	if ( ids.empty() ) {
		Sys_Printf( "Select Whole Cutter: select a patch of a cutter first.\n" );
		return 0;
	}
	std::vector<brush_t*> add;
	for ( brush_t *b = active_brushes.next; b != &active_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && std::find( ids.begin(), ids.end(), b->pPatch->seatId ) != ids.end() ) {
			add.push_back( b );
		}
	for ( size_t i = 0; i < add.size(); i++ )
		Select_Brush( add[i] );
	Sys_Printf( "Select Whole Cutter: %d patch(es) added to the selection.\n", (int)add.size() );
	Sys_UpdateWindows( W_ALL );
	return (int)add.size();
}

// =============================================================================
// copies: a copy of a cutter is a cutter of its own

static std::vector<brush_t*> g_seatBefore;

void Seat_ImportBegin(){
	g_seatBefore.clear();
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( b->patchBrush && b->pPatch && b->pPatch->seatId > 0 ) {
				g_seatBefore.push_back( b );
			}
	}
}

void Seat_ImportEnd(){
	std::vector<int> oldIds;
	for ( size_t i = 0; i < g_seatBefore.size(); i++ )
		oldIds.push_back( g_seatBefore[i]->pPatch->seatId );
	int top = Seat_TopId();
	std::vector<int> from, to;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			if ( !b->patchBrush || !b->pPatch || b->pPatch->seatId <= 0 ) {
				continue;
			}
			if ( std::find( g_seatBefore.begin(), g_seatBefore.end(), b ) != g_seatBefore.end() ) {
				continue;
			}
			const int id = b->pPatch->seatId;
			if ( std::find( oldIds.begin(), oldIds.end(), id ) == oldIds.end() ) {
				continue; // not a copy of a cutter that is here
			}
			size_t k = 0;
			while ( k < from.size() && from[k] != id )
				k++;
			if ( k == from.size() ) {
				from.push_back( id );
				to.push_back( ++top );
			}
			b->pPatch->seatId = to[k];
		}
	}
	g_seatBefore.clear();
}

// =============================================================================
// the outline of an active cutter

static void Seat_StripLine( const segGrid_t *g, int u0, int v0, int du, int dv, int count ){
	qglBegin( GL_LINE_STRIP );
	for ( int k = 0; k < count; k++ )
		qglVertex3fv( g->p[( u0 + du * k ) * g->V + v0 + dv * k].xyz );
	qglEnd();
}

static bool Seat_SamePoint( const float *a, const float *b ){
	return fabsf( a[0] - b[0] ) + fabsf( a[1] - b[1] ) + fabsf( a[2] - b[2] ) < 0.5f;
}

// Drawn in the grid views and the camera: the border of every active cutter patch in a bright colour (the seam of a
// ring is left out), and, once the holes are up to date, the edges of the hull that cuts them.
void Seat_Draw(){
	if ( Straight_Active() ) {
		return; // the outlines would sit where the cutters really are, not where Straight Edit shows them
	}
	std::vector<patchMesh_t*> all = Seat_AllPatches();
	std::vector<patchMesh_t*> cutters;
	for ( size_t i = 0; i < all.size(); i++ )
		if ( all[i]->seatId > 0 && all[i]->segH > 0 ) {
			cutters.push_back( all[i] );
		}
	if ( cutters.empty() ) {
		return;
	}

	qglPushAttrib( GL_ALL_ATTRIB_BITS );
	qglDisable( GL_TEXTURE_2D );
	qglDisable( GL_LIGHTING );
	qglDisable( GL_CULL_FACE );
	qglDisable( GL_DEPTH_TEST );
	qglDisable( GL_LINE_STIPPLE );

	if ( !g_seatPending ) {
		qglEnable( GL_BLEND );
		qglBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		qglLineWidth( 1 );
		qglColor4f( 1.0f, 0.0f, 0.35f, 0.07f ); // hull edges: hundreds of them overlap, so each one is faint
		for ( std::map<int, std::vector<float> >::iterator it = g_seatCue.begin(); it != g_seatCue.end(); ++it )
		{
			qglBegin( GL_LINES );
			for ( size_t k = 0; k + 2 < it->second.size(); k += 3 )
				qglVertex3fv( &it->second[k] );
			qglEnd();
		}
		qglDisable( GL_BLEND );
	}

	qglEnable( GL_BLEND );
	qglBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	qglLineWidth( 3 );
	qglColor4f( 1.0f, 0.0f, 0.35f, 0.2f );
	for ( size_t i = 0; i < cutters.size(); i++ )
	{
		const segGrid_t *g = Patch_GetSegGrid( cutters[i] );
		if ( !g || g->U < 2 || g->V < 2 ) {
			continue;
		}
		const int U = g->U, V = g->V;
		bool ringU = Seat_SamePoint( g->p[0].xyz, g->p[( U - 1 ) * V].xyz ) && Seat_SamePoint( g->p[V - 1].xyz, g->p[( U - 1 ) * V + V - 1].xyz );
		bool ringV = Seat_SamePoint( g->p[0].xyz, g->p[V - 1].xyz ) && Seat_SamePoint( g->p[( U - 1 ) * V].xyz, g->p[( U - 1 ) * V + V - 1].xyz );
		if ( !ringU ) {
			Seat_StripLine( g, 0, 0, 0, 1, V );
			Seat_StripLine( g, U - 1, 0, 0, 1, V );
		}
		if ( !ringV ) {
			Seat_StripLine( g, 0, 0, 1, 0, U );
			Seat_StripLine( g, 0, V - 1, 1, 0, U );
		}
	}
	qglLineWidth( 1 );
	qglPopAttrib();
}
