// Straight Edit, see straight.h

#include "stdafx.h"
#include <math.h>
#include <vector>
#include "straight.h"
#include "rotframe.h"
#include "seatcut.h"
#include "filters.h"

extern vec3_t select_matrix[3];
extern vec3_t select_origin;
extern qboolean select_fliporder;
void Select_ApplyMatrix( bool bSnap, bool bRotation, int nAxis, float fDeg );
void Brush_DrawXY( brush_t *b, int nViewType );
void Brush_RebuildBrush( brush_t *b, vec3_t vMins, vec3_t vMaxs );

static bool g_stOn = false;
static float g_stR[9];          // the frame of the objects: world = R * straight
static vec3_t g_stP;            // the middle of the selection, the pivot of the turn
static float g_stM[16];         // for drawing the rest of the map: world -> shown (the inverse of R about P)

// what Suspend / Resume need to bring the straight numbers back exactly
struct patchSnap_t
{
	patchMesh_t *p;
	int width, height;
	std::vector<float> xyz;
	int hasFrame;
	float frameR[9];
	float cog[COG_NPARAM];
	int lnkHasF;
	float lnkF[12];
};
static std::vector<patchSnap_t> g_stSnap;
static std::vector<brush_t*> g_stMovedIn;       // brushes Suspend selected, Resume puts them back

bool Straight_Active(){
	return g_stOn;
}

static void ST_SetMatrix( bool toStraight ){
	// select_matrix: new = M * ( old - origin ) + origin. To straighten M is the inverse (the transpose) of R.
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			select_matrix[i][j] = toStraight ? g_stR[j * 3 + i] : g_stR[i * 3 + j];
	VectorCopy( g_stP, select_origin );
	select_fliporder = false;
}

// turn the selected objects: to the straight pose, or back
static void ST_Transform( bool toStraight ){
	const select_t mode = g_qeglobals.d_select_mode;
	const int nmp = g_qeglobals.d_num_move_points;
	const bool bend = g_bPatchBendMode;
	// whole objects, not just the points that happen to be selected in a patch
	g_qeglobals.d_select_mode = sel_brush;
	g_qeglobals.d_num_move_points = 0;
	g_bPatchBendMode = false;
	ST_SetMatrix( toStraight );
	Select_ApplyMatrix( false, true, 0, 0 );
	g_qeglobals.d_select_mode = mode;
	g_qeglobals.d_num_move_points = nmp;
	g_bPatchBendMode = bend;
}

static void ST_UpdateGhostMatrix(){
	// shown = Rt * ( p - P ) + P, with Rt = transpose( R ): column major for OpenGL
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			g_stM[j * 4 + i] = g_stR[j * 3 + i]; // element row i, column j of Rt = R[j][i]
	for ( int i = 0; i < 3; i++ )
	{
		g_stM[i * 4 + 3] = 0.0f;
		g_stM[12 + i] = g_stP[i];
		for ( int j = 0; j < 3; j++ )
			g_stM[12 + i] -= g_stR[j * 3 + i] * g_stP[j];
	}
	g_stM[15] = 1.0f;
}

void Straight_GhostMatrix( float m[16] ){
	memcpy( m, g_stM, sizeof( g_stM ) );
}

void Straight_GhostBounds( const vec3_t mins, const vec3_t maxs, vec3_t outMins, vec3_t outMaxs ){
	for ( int k = 0; k < 3; k++ )
	{
		outMins[k] = 1e30f;
		outMaxs[k] = -1e30f;
	}
	for ( int c = 0; c < 8; c++ )
	{
		float v[3] = { ( c & 1 ) ? maxs[0] : mins[0], ( c & 2 ) ? maxs[1] : mins[1], ( c & 4 ) ? maxs[2] : mins[2] };
		for ( int i = 0; i < 3; i++ )
		{
			// Rt * ( v - P ) + P, Rt[i][j] = R[j][i]
			float s = g_stP[i];
			for ( int j = 0; j < 3; j++ )
				s += g_stR[j * 3 + i] * ( v[j] - g_stP[j] );
			if ( s < outMins[i] ) {
				outMins[i] = s;
			}
			if ( s > outMaxs[i] ) {
				outMaxs[i] = s;
			}
		}
	}
}

static bool ST_SameFrame( const float *a, const float *b ){
	for ( int i = 0; i < 9; i++ )
		if ( fabs( a[i] - b[i] ) > 1e-3 ) {
			return false;
		}
	return true;
}

static void ST_Message( const char *text ){
	Sys_Printf( "Straight Edit: %s\n", text );
	gtk_MessageBox( g_pParentWnd->m_pWidget, text, "Straight Edit" );
}

// The frame the view is turned by. The objects are axis-aligned after any frame of the form R * S, where S is one of the
// 24 ways to lay a box onto the grid (a rotation that only permutes the axes and flips their signs). Taking the S that
// leaves R * S the smallest turn keeps the objects lying the way they lie and turns the rest of the map as little as
// possible; S = identity (the pose the objects were built in) is only chosen when that is the smallest.
static void ST_SmallestTurn( const float R[9], float out[9] ){
	static const int perm[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
	static const int parity[6] = { 1, -1, -1, 1, 1, -1 };
	float best = -1e30f;
	memcpy( out, R, 9 * sizeof( float ) );
	for ( int p = 0; p < 6; p++ )
		for ( int sg = 0; sg < 8; sg++ )
		{
			const float s[3] = { ( sg & 1 ) ? -1.0f : 1.0f, ( sg & 2 ) ? -1.0f : 1.0f, ( sg & 4 ) ? -1.0f : 1.0f };
			if ( parity[p] * s[0] * s[1] * s[2] < 0 ) {
				continue; // a mirror, not a turn
			}
			float S[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			for ( int i = 0; i < 3; i++ )
				S[i * 3 + perm[p][i]] = s[i];
			float M[9];
			for ( int i = 0; i < 3; i++ )
				for ( int j = 0; j < 3; j++ )
				{
					M[i * 3 + j] = 0;
					for ( int k = 0; k < 3; k++ )
						M[i * 3 + j] += R[i * 3 + k] * S[k * 3 + j];
				}
			// the smaller the turn, the larger the trace
			const float tr = M[0] + M[4] + M[8];
			if ( tr > best + 1e-4f ) {
				best = tr;
				memcpy( out, M, sizeof( M ) );
			}
		}
}

static bool ST_Enter(){
	if ( selected_brushes.next == &selected_brushes ) {
		ST_Message( "Select the rotated object first." );
		return false;
	}
	bool have = false;
	int unframed = 0;
	float R[9];
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
	{
		if ( b->patchBrush && b->pPatch ) {
			if ( b->pPatch->hasFrame ) {
				if ( !have ) {
					memcpy( R, b->pPatch->frameR, sizeof( R ) );
					have = true;
				}
				else if ( !ST_SameFrame( R, b->pPatch->frameR ) ) {
					ST_Message( "The selection has parts that were turned differently. Select the parts of one rotated object." );
					return false;
				}
			}
			else{
				unframed++;
			}
		}
		else if ( !( b->owner && b->owner->eclass && b->owner->eclass->fixedsize ) ) {
			ST_Message( "Straight Edit works on patches and point entities. The selection has a brush." );
			return false;
		}
	}
	if ( !have ) {
		ST_Message( "Nothing in the selection has been rotated. Rotate it first (R), then Straight Edit can put it back on the grid." );
		return false;
	}

	ST_SmallestTurn( R, g_stR );
	Select_GetTrueMid( g_stP );

	// the rest of the map stays where it is and is drawn turned: nothing in it can be picked or edited
	for ( brush_t *b = active_brushes.next; b != &active_brushes; b = b->next )
	{
		b->ghost = true;
		b->bFiltered = true;
	}
	ST_Transform( true );
	ST_UpdateGhostMatrix();
	g_stOn = true;
	Undo_Clear();
	if ( unframed > 0 ) {
		Sys_Printf( "Straight Edit: %d selected patches had no rotation of their own and were turned with the rest\n", unframed );
	}
	Sys_Printf( "Straight Edit on: the selection is straight, the rest of the map is shown turned and grayed out. Undo history cleared. Esc or the same command leaves it\n" );
	Sys_Status( "STRAIGHT EDIT", 0 );
	Sys_UpdateWindows( W_ALL );
	return true;
}

// every brush that is part of the working set: not a ghost, selected or not
static void ST_Working( std::vector<brush_t*> &out ){
	for ( int pass = 0; pass < 2; pass++ )
	{
		brush_t *head = pass ? &selected_brushes : &active_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( !b->ghost ) {
				out.push_back( b );
			}
	}
}

static void ST_Leave(){
	// everything made or deselected in the meantime goes back with the rest: select it
	std::vector<brush_t*> work;
	ST_Working( work );
	for ( size_t i = 0; i < work.size(); i++ )
		if ( work[i]->next && work[i]->prev && !IsBrushSelected( work[i] ) ) {
			Brush_RemoveFromList( work[i] );
			Brush_AddToList( work[i], &selected_brushes );
		}
	ST_Transform( false );

	g_stOn = false;
	for ( int pass = 0; pass < 2; pass++ )
	{
		brush_t *head = pass ? &selected_brushes : &active_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			b->ghost = false;
			b->bFiltered = FilterBrush( b );
		}
	}
	Undo_Clear();
	Seat_Recut(); // the cutters are back in place: the holes follow
	Sys_Printf( "Straight Edit off: the selection is back where it belongs. Undo history cleared\n" );
	Sys_Status( "", 0 );
	Sys_UpdateWindows( W_ALL );
}

void Straight_Toggle(){
	if ( g_stOn ) {
		ST_Leave();
	}
	else{
		ST_Enter();
	}
}

void Straight_Reset(){
	g_stOn = false;
	g_stSnap.clear();
	g_stMovedIn.clear();
}

bool Straight_Suspend(){
	if ( !g_stOn ) {
		return false;
	}
	std::vector<brush_t*> work;
	ST_Working( work );
	g_stSnap.clear();
	g_stMovedIn.clear();
	for ( size_t i = 0; i < work.size(); i++ )
	{
		brush_t *b = work[i];
		if ( b->patchBrush && b->pPatch ) {
			patchSnap_t s;
			s.p = b->pPatch;
			s.width = b->pPatch->width;
			s.height = b->pPatch->height;
			for ( int w = 0; w < s.width; w++ )
				for ( int h = 0; h < s.height; h++ )
					for ( int k = 0; k < 3; k++ )
						s.xyz.push_back( b->pPatch->ctrl[w][h].xyz[k] );
			s.hasFrame = b->pPatch->hasFrame;
			memcpy( s.frameR, b->pPatch->frameR, sizeof( s.frameR ) );
			memcpy( s.cog, b->pPatch->cog, sizeof( s.cog ) );
			s.lnkHasF = b->pPatch->lnkHasF;
			memcpy( s.lnkF, b->pPatch->lnkF, sizeof( s.lnkF ) );
			g_stSnap.push_back( s );
		}
		if ( !IsBrushSelected( b ) ) {
			Brush_RemoveFromList( b );
			Brush_AddToList( b, &selected_brushes );
			g_stMovedIn.push_back( b );
		}
	}
	ST_Transform( false );
	return true;
}

void Straight_Resume(){
	ST_Transform( true );
	// the numbers as they were: no rounding from the way there and back
	for ( size_t i = 0; i < g_stSnap.size(); i++ )
	{
		patchSnap_t &s = g_stSnap[i];
		if ( s.p->width != s.width || s.p->height != s.height ) {
			continue;
		}
		size_t n = 0;
		for ( int w = 0; w < s.width; w++ )
			for ( int h = 0; h < s.height; h++ )
				for ( int k = 0; k < 3; k++ )
					s.p->ctrl[w][h].xyz[k] = s.xyz[n++];
		s.p->hasFrame = s.hasFrame;
		memcpy( s.p->frameR, s.frameR, sizeof( s.frameR ) );
		memcpy( s.p->cog, s.cog, sizeof( s.cog ) );
		s.p->lnkHasF = s.lnkHasF;
		memcpy( s.p->lnkF, s.lnkF, sizeof( s.lnkF ) );
		s.p->bDirty = true;
		vec3_t vMin, vMax;
		Patch_CalcBounds( s.p, vMin, vMax );
		Brush_RebuildBrush( s.p->pSymbiot, vMin, vMax );
	}
	for ( size_t i = 0; i < g_stMovedIn.size(); i++ )
	{
		brush_t *b = g_stMovedIn[i];
		if ( b->next && b->prev && IsBrushSelected( b ) ) {
			Brush_RemoveFromList( b );
			Brush_AddToList( b, &active_brushes );
		}
	}
	g_stSnap.clear();
	g_stMovedIn.clear();
}

void Straight_DrawGhostsXY( int viewType, int nDim1, int nDim2, const float *vmins, const float *vmaxs ){
	if ( !g_stOn ) {
		return;
	}
	qglPushMatrix();
	qglMultMatrixf( g_stM );
	qglColor3f( 0.58f, 0.58f, 0.64f );
	for ( brush_t *b = active_brushes.next; b != &active_brushes; b = b->next )
	{
		if ( !b->ghost ) {
			continue;
		}
		vec3_t gm, gx;
		Straight_GhostBounds( b->mins, b->maxs, gm, gx );
		if ( gm[nDim1] > vmaxs[0] || gm[nDim2] > vmaxs[1] || gx[nDim1] < vmins[0] || gx[nDim2] < vmins[1] ) {
			continue;
		}
		Brush_DrawXY( b, viewType );
	}
	qglPopMatrix();
}
