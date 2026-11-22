// Patch groups, see patchgroup.h

#include "stdafx.h"
#include <vector>
#include <algorithm>
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
