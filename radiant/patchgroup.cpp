// Patch groups, see patchgroup.h

#include "stdafx.h"
#include <vector>
#include <algorithm>
#include <map>
#include "patchgroup.h"
#include "straight.h"

static bool PG_IsGrouped( brush_t *b ){
	return b->patchBrush && b->pPatch && b->pPatch->groupId != 0;
}

// the highest id in use anywhere in the map
static int PG_TopId(){
	int top = 0;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( PG_IsGrouped( b ) && b->pPatch->groupId > top ) {
				top = b->pPatch->groupId;
			}
	}
	return top;
}

// the ids of the groups that have a patch selected
static void PG_SelectedIds( std::vector<int> &ids ){
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( PG_IsGrouped( b ) && std::find( ids.begin(), ids.end(), b->pPatch->groupId ) == ids.end() ) {
			ids.push_back( b->pPatch->groupId );
		}
}

int PatchGroup_NextId(){
	return PG_TopId() + 1;
}

int PatchGroup_Merge(){
	std::vector<brush_t*> patches;
	int others = 0;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
	{
		if ( b->patchBrush && b->pPatch ) {
			patches.push_back( b );
		}
		else{
			others++;
		}
	}
	if ( patches.size() < 2 ) {
		Sys_Printf( "Merge Selection: select two or more patches.\n" );
		return 0;
	}
	const int id = PG_TopId() + 1;
	Undo_Start( "merge selection" );
	Undo_AddBrushList( &selected_brushes );
	for ( size_t i = 0; i < patches.size(); i++ )
		patches[i]->pPatch->groupId = id;
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_MarkMapModified();
	Sys_Printf( "Merge Selection: %d patches are one group now (id %d)%s.\n", (int)patches.size(), id,
				others ? ", brushes and entities in the selection were left out" : "" );
	Sys_UpdateWindows( W_ALL );
	return (int)patches.size();
}

int PatchGroup_Split(){
	std::vector<int> ids;
	PG_SelectedIds( ids );
	if ( ids.empty() ) {
		Sys_Printf( "Split Group: select a patch of a group first.\n" );
		return 0;
	}
	Undo_Start( "split group" );
	Undo_AddBrushList( &selected_brushes );
	int n = 0;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( PG_IsGrouped( b ) && std::find( ids.begin(), ids.end(), b->pPatch->groupId ) != ids.end() ) {
				b->pPatch->groupId = 0;
				n++;
			}
	}
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_MarkMapModified();
	Sys_Printf( "Split Group: %d patches left their group.\n", n );
	Sys_UpdateWindows( W_ALL );
	return n;
}

int PatchGroup_SelectWhole( bool bPrint ){
	if ( Straight_Active() ) {
		return 0; // the rest of the map is out of reach while Straight Edit is on
	}
	std::vector<int> ids;
	PG_SelectedIds( ids );
	if ( ids.empty() ) {
		if ( bPrint ) {
			Sys_Printf( "Select Whole Group: select a patch of a group first.\n" );
		}
		return 0;
	}
	std::vector<brush_t*> add;
	for ( brush_t *b = active_brushes.next; b != &active_brushes; b = b->next )
		if ( PG_IsGrouped( b ) && std::find( ids.begin(), ids.end(), b->pPatch->groupId ) != ids.end() ) {
			add.push_back( b );
		}
	for ( size_t i = 0; i < add.size(); i++ )
		Select_Brush( add[i], false, false );
	if ( !add.empty() ) {
		UpdatePatchInspector();
		UpdateSurfaceDialog();
	}
	if ( bPrint ) {
		Sys_Printf( "Select Whole Group: %d patches added.\n", (int)add.size() );
		Sys_UpdateWindows( W_ALL );
	}
	return (int)add.size();
}

void PatchGroup_DeselectWhole( brush_t *brush ){
	if ( !brush || !PG_IsGrouped( brush ) ) {
		return;
	}
	const int id = brush->pPatch->groupId;
	std::vector<brush_t*> drop;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( PG_IsGrouped( b ) && b->pPatch->groupId == id ) {
			drop.push_back( b );
		}
	for ( size_t i = 0; i < drop.size(); i++ )
	{
		Brush_RemoveFromList( drop[i] );
		Brush_AddToList( drop[i], &active_brushes );
	}
	UpdatePatchInspector();
}

// =============================================================================
// double click: the whole connected object

namespace {
struct ConnPt {
	float p[3];
	int patch;
};

typedef unsigned long long ConnKey;

const float CONN_CELL = 0.5f; // size of a hash cell
const float CONN_TOL = 0.2f;  // two points closer than this are the same vertex

ConnKey Conn_Key( int x, int y, int z ){
	return ( (ConnKey)( x + 1048576 ) & 0x1FFFFF ) | ( ( (ConnKey)( y + 1048576 ) & 0x1FFFFF ) << 21 ) | ( ( (ConnKey)( z + 1048576 ) & 0x1FFFFF ) << 42 );
}

int Conn_Find( std::vector<int> &up, int i ){
	while ( up[i] != i )
	{
		up[i] = up[up[i]];
		i = up[i];
	}
	return i;
}
}

int PatchGroup_SelectConnected( vec3_t origin, vec3_t dir, bool camera, bool add ){
	if ( Straight_Active() ) {
		return 0;
	}
	trace_t t = Test_Ray( origin, dir, camera ? SF_CAMERA : SF_ENTITIES_FIRST );
	if ( !t.brush || !t.brush->patchBrush || !t.brush->pPatch ) {
		return 0;
	}

	// the patches of the same entity that are there to be picked
	std::vector<brush_t*> patches;
	int seed = -1;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			if ( !b->patchBrush || !b->pPatch || b->owner != t.brush->owner || b->bFiltered ) {
				continue;
			}
			if ( b == t.brush ) {
				seed = (int)patches.size();
			}
			patches.push_back( b );
		}
	}
	if ( seed < 0 ) {
		return 0;
	}

	// the points on the edges of every patch, in a hash of cells
	std::vector<ConnPt> pts;
	std::map<ConnKey, std::vector<int> > cells;
	for ( size_t i = 0; i < patches.size(); i++ )
	{
		const patchMesh_t *p = patches[i]->pPatch;
		for ( int c = 0; c < p->width; c++ )
			for ( int r = 0; r < p->height; r++ )
			{
				if ( c != 0 && c != p->width - 1 && r != 0 && r != p->height - 1 ) {
					continue; // inside the patch, it cannot touch another one
				}
				ConnPt q;
				VectorCopy( p->ctrl[c][r].xyz, q.p );
				q.patch = (int)i;
				cells[Conn_Key( (int)floor( q.p[0] / CONN_CELL ), (int)floor( q.p[1] / CONN_CELL ), (int)floor( q.p[2] / CONN_CELL ) )].push_back( (int)pts.size() );
				pts.push_back( q );
			}
	}

	// patches that have a point in common are one object
	std::vector<int> up( patches.size() );
	for ( size_t i = 0; i < up.size(); i++ )
		up[i] = (int)i;
	for ( size_t k = 0; k < pts.size(); k++ )
	{
		const int cx = (int)floor( pts[k].p[0] / CONN_CELL );
		const int cy = (int)floor( pts[k].p[1] / CONN_CELL );
		const int cz = (int)floor( pts[k].p[2] / CONN_CELL );
		for ( int dx = -1; dx <= 1; dx++ )
			for ( int dy = -1; dy <= 1; dy++ )
				for ( int dz = -1; dz <= 1; dz++ )
				{
					std::map<ConnKey, std::vector<int> >::const_iterator it = cells.find( Conn_Key( cx + dx, cy + dy, cz + dz ) );
					if ( it == cells.end() ) {
						continue;
					}
					for ( size_t m = 0; m < it->second.size(); m++ )
					{
						const ConnPt &o = pts[it->second[m]];
						if ( o.patch == pts[k].patch ) {
							continue;
						}
						if ( fabsf( o.p[0] - pts[k].p[0] ) <= CONN_TOL && fabsf( o.p[1] - pts[k].p[1] ) <= CONN_TOL && fabsf( o.p[2] - pts[k].p[2] ) <= CONN_TOL ) {
							const int a = Conn_Find( up, o.patch );
							const int b = Conn_Find( up, pts[k].patch );
							if ( a != b ) {
								up[a] = b;
							}
						}
					}
				}
	}

	const int root = Conn_Find( up, seed );
	std::vector<brush_t*> object;
	for ( size_t i = 0; i < patches.size(); i++ )
		if ( Conn_Find( up, (int)i ) == root ) {
			object.push_back( patches[i] );
		}

	// select them
	std::vector<brush_t*> already;
	if ( add ) {
		for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
			already.push_back( b );
	}
	else{
		Select_Deselect();
	}
	for ( size_t i = 0; i < object.size(); i++ )
		if ( std::find( already.begin(), already.end(), object[i] ) == already.end() ) {
			Select_Brush( object[i], false, false );
		}
	const int n = (int)object.size();
	PatchGroup_SelectWhole(); // anything that was grouped on purpose comes along as well
	UpdatePatchInspector();
	UpdateSurfaceDialog();
	Sys_Printf( "Select Connected: %d patches are one object.\n", n );
	Sys_UpdateWindows( W_ALL );
	return n;
}

// =============================================================================
// copies

// Copy / paste / clone bring patches that carry the id of the group they were copied from. A copy must be a group
// of its own (otherwise picking the copy would also pick the original), so the patches that came in with an id that
// a patch that was there before already has get a new one (the same new one for all patches of one old id).
static std::vector<brush_t*> g_groupBefore;

void PatchGroup_ImportBegin(){
	g_groupBefore.clear();
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( PG_IsGrouped( b ) ) {
				g_groupBefore.push_back( b );
			}
	}
}

void PatchGroup_ImportEnd(){
	std::vector<int> oldIds;
	for ( size_t i = 0; i < g_groupBefore.size(); i++ )
		oldIds.push_back( g_groupBefore[i]->pPatch->groupId );
	const int top = PG_TopId();
	int next = top;
	std::vector<int> from, to;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			if ( !PG_IsGrouped( b ) || std::find( g_groupBefore.begin(), g_groupBefore.end(), b ) != g_groupBefore.end() ) {
				continue;
			}
			patchMesh_t *p = b->pPatch;
			if ( std::find( oldIds.begin(), oldIds.end(), p->groupId ) == oldIds.end() ) {
				continue; // not a copy of a group that is here
			}
			size_t k = 0;
			while ( k < from.size() && from[k] != p->groupId )
				k++;
			if ( k == from.size() ) {
				from.push_back( p->groupId );
				to.push_back( ++next );
			}
			p->groupId = to[k];
		}
	}
	g_groupBefore.clear();
}
