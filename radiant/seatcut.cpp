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
		if ( report ) {
			Sys_Printf( "Seat cut: cutter %d (%d patches) cuts %d patch(es).\n", it->first, (int)it->second.size(), cuts );
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
		qglColor4f( 0.1f, 0.9f, 1.0f, 0.55f );
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
	qglColor4f( 0.1f, 0.9f, 1.0f, 0.25f );
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
