/*
   Cogs: generating the patches of a cog from its numbers (see cog.h).
 */
#include "stdafx.h"
#include <vector>
#include <algorithm>
#include <math.h>
#include "cog.h"
#include "bevel.h"

void WINAPI Patch_Naturalize( patchMesh_t *p );

#define COG_PI 3.14159265358979f
#define COG_MIN_TEETH 3
#define COG_MAX_TEETH 64
// the last of the cog numbers every patch carries is not part of the record: it says which part of the cog the patch
// is (0 a wall, 10 + end a piece of a cap, 1000 + end * 100 + k the top of tooth k), so the cog can find where it is
// turned to even after it was rotated
#define COG_PARTSLOT ( COG_NPARAM - 1 )

// what the last cog was set to, for the next one: teeth inside to begin with
static int g_cogLastMode = 1;
static float g_cogLastSnap = 0.0f;

// =============================================================================
// the numbers, as they are stored in every patch of the cog

void Cog_Pack( const cogRec_t &r, float *o ){
	for ( int i = 0; i < COG_NPARAM; i++ )
		o[i] = 0;
	int n = 0;
	o[n++] = (float)r.teeth;
	o[n++] = (float)r.mode;
	o[n++] = r.cylR;
	o[n++] = r.depth;
	o[n++] = r.taper;
	o[n++] = r.snap;
	o[n++] = (float)r.tipArc;
	o[n++] = (float)r.rootArc;
	o[n++] = r.hole;
	o[n++] = r.height;
	for ( int k = 0; k < 3; k++ )
		o[n++] = r.C[k];
	for ( int k = 0; k < 3; k++ )
		o[n++] = r.A[k];
	for ( int k = 0; k < 3; k++ )
		o[n++] = r.E1[k];
	o[n++] = r.tipRound;
	o[n++] = r.rootRound;
	o[n++] = (float)r.roundSeg;
	o[n++] = r.rimBevel;
	o[n++] = (float)r.rimSub;
	o[n++] = r.cone;
	o[n++] = r.cone2;
}

void Cog_Unpack( const float *in, cogRec_t &r ){
	int n = 0;
	r.teeth = (int)( in[n++] + 0.5f );
	r.mode = (int)( in[n++] + 0.5f );
	r.cylR = in[n++];
	r.depth = in[n++];
	r.taper = in[n++];
	r.snap = in[n++];
	r.tipArc = (int)( in[n++] + 0.5f );
	r.rootArc = (int)( in[n++] + 0.5f );
	r.hole = in[n++];
	r.height = in[n++];
	for ( int k = 0; k < 3; k++ )
		r.C[k] = in[n++];
	for ( int k = 0; k < 3; k++ )
		r.A[k] = in[n++];
	for ( int k = 0; k < 3; k++ )
		r.E1[k] = in[n++];
	r.tipRound = in[n++];
	r.rootRound = in[n++];
	r.roundSeg = (int)( in[n++] + 0.5f );
	r.rimBevel = in[n++];
	r.rimSub = (int)( in[n++] + 0.5f );
	r.cone = in[n++]; // cogs saved before it existed have 0 here
	if ( !( r.cone > -1e6f && r.cone < 1e6f ) ) {
		r.cone = 0;
	}
	r.cone2 = in[n++];
	if ( !( r.cone2 > -1e6f && r.cone2 < 1e6f ) ) {
		r.cone2 = 0;
	}
	if ( !( r.snap >= 0.0f && r.snap <= 16.0f ) ) {
		r.snap = 0;
	}
	if ( r.hole < 0 || r.hole > 1e6f ) {
		r.hole = 0; // maps saved by an earlier build had a rim band in this place
	}
}

// =============================================================================
// the profile, in the plane across the axis (x along E1, y along E2, the middle at 0,0)

struct cogShape_t
{
	int n;                                  // vertices: four to a tooth
	float rTip, rRoot, at, ar;              // radii, and half angles of the tip and of the base of the tooth
	float maxHole;                          // the widest hole that leaves material everywhere
	std::vector<float> vx, vy;              // vertices: tip start, tip end, root start, root end, tooth after tooth
	std::vector<float> cx, cy;              // the middle control point of every edge from vertex i to i + 1
};

static float Cog_Min( float a, float b ){
	return a < b ? a : b;
}

static float Cog_Max( float a, float b ){
	return a > b ? a : b;
}

static bool Cog_Shape( const cogRec_t &r, cogShape_t &s ){
	const int N = r.teeth;
	if ( N < COG_MIN_TEETH || N > COG_MAX_TEETH || r.cylR < 1.0f || r.depth < 0.25f || r.taper < 0.0f || r.taper > 99.0f ) {
		return false;
	}
	const float P = 2.0f * COG_PI / N;
	if ( r.mode == 0 ) {
		s.rRoot = r.cylR;
		s.rTip = r.cylR + r.depth;
	}
	else
	{
		s.rTip = r.cylR;
		s.rRoot = r.cylR - r.depth;
	}
	if ( s.rRoot < 1.0f ) {
		return false;
	}
	// The root is a regular polygon with two edges to a tooth: the base of the tooth and the gap are the same
	// width, so the cog keeps the roundness of the cylinder it came from. The tip is the base width narrowed
	// by the taper (0 = the sides of the tooth are parallel, towards 100 = it comes to a point).
	const float ar = 0.25f * P;
	const float baseHalf = s.rRoot * sinf( ar );     // half the width of the base of the tooth
	if ( baseHalf < 0.25f ) {
		return false; // too many teeth for this radius
	}
	const float tipHalf = Cog_Max( baseHalf * ( 1.0f - 0.01f * r.taper ), 0.05f );
	const float at = asinf( Cog_Min( 1.0f, tipHalf / s.rTip ) );
	s.at = at;
	s.ar = ar;

	const int n = 4 * N;
	s.n = n;
	s.vx.resize( n );
	s.vy.resize( n );
	for ( int k = 0; k < N; k++ )
	{
		const float th = k * P;
		const float ang[4] = { th - at, th + at, th + ar, th + P - ar };
		const float rad[4] = { s.rTip, s.rTip, s.rRoot, s.rRoot };
		for ( int m = 0; m < 4; m++ )
		{
			s.vx[4 * k + m] = rad[m] * cosf( ang[m] );
			s.vy[4 * k + m] = rad[m] * sinf( ang[m] );
		}
	}

	s.cx.resize( n );
	s.cy.resize( n );
	for ( int i = 0; i < n; i++ )
	{
		const int j = ( i + 1 ) % n;
		const int type = i % 4;
		const bool arc = ( type == 0 && r.tipArc ) || ( type == 2 && r.rootArc );
		if ( arc ) {
			const float ta = atan2f( s.vy[i], s.vx[i] ), tb = atan2f( s.vy[j], s.vx[j] );
			float span = tb - ta;
			while ( span > COG_PI )
				span -= 2.0f * COG_PI;
			while ( span < -COG_PI )
				span += 2.0f * COG_PI;
			const float tm = ta + 0.5f * span;
			const float rad = sqrtf( s.vx[i] * s.vx[i] + s.vy[i] * s.vy[i] );
			const float k = rad / cosf( 0.5f * span );
			s.cx[i] = k * cosf( tm );
			s.cy[i] = k * sinf( tm );
		}
		else
		{
			s.cx[i] = 0.5f * ( s.vx[i] + s.vx[j] );
			s.cy[i] = 0.5f * ( s.vy[i] + s.vy[j] );
		}
	}

	// A hole must stay inside the root polygon, whose nearest edges are the flat chords at the base of a tooth
	// and across a gap (both the same distance from the middle).
	s.maxHole = s.rRoot * cosf( ar ) - 1.0f;
	if ( r.hole > 0 && ( r.hole < 0.25f || r.hole > s.maxHole ) ) {
		return false;
	}
	// a cap that dips must not reach the middle plane, so the walls between the caps keep a length
	for ( int cap = 0; cap < 2; cap++ )
	{
		const float c = cap == 0 ? r.cone : r.cone2;
		if ( c < 0.0f && 0.5f * r.height + c * ( 1.0f - r.hole / s.rTip ) < 0.25f ) {
			return false;
		}
	}
	return true;
}

// The height of a cap at a point of the profile, along the axis from the middle plane: the slope runs in a straight
// line (in the distance from the axis) from `cone` above the edge height in the middle to the edge height at the
// tips of the teeth. Used for the points the surface passes through; the control point between two of them is put
// half way between their heights, so it lies on the straight line between them.
static float Cog_CapZ( const cogRec_t &r, const cogShape_t &s, int cap, float x, float y ){
	const float d = sqrtf( x * x + y * y );
	return 0.5f * r.height + ( cap == 0 ? r.cone : r.cone2 ) * ( 1.0f - d / s.rTip );
}

void Cog_Describe( const cogRec_t &r, char *text, int len ){
	cogShape_t s;
	if ( !Cog_Shape( r, s ) ) {
		cogRec_t solid = r;
		solid.hole = 0;
		if ( r.hole > 0 && Cog_Shape( solid, s ) ) {
			_snprintf( text, len, "The hole can be at most %.1f in radius with these teeth", s.maxHole );
		}
		else
		{
			_snprintf( text, len, "These numbers do not make a cog (too many teeth, too deep or too narrow, or the dip of the ends is too deep for the length)" );
		}
		text[len - 1] = 0;
		return;
	}
	const float P = 2.0f * COG_PI / r.teeth;
	const float gap = 2.0f * s.rRoot * sinf( 0.5f * P - s.ar );
	_snprintf( text, len, "%d teeth, %d patches. Tip radius %.1f, root %.1f, gap %.1f wide, hole up to %.1f",
			   r.teeth, ( r.hole > 0 ? 12 : 10 ) * r.teeth, s.rTip, s.rRoot, gap, s.maxHole );
	text[len - 1] = 0;
}

// =============================================================================
// the patches

struct cogGen_t
{
	const cogRec_t *r;
	float E2[3];
	int id;
	float packed[COG_NPARAM];
	IShader *shader;
	entity_t *owner;
	float texW, texH;
	std::vector<brush_t*> made;
};

static void Cog_Lift( const cogGen_t &g, float x, float y, float z, float *o ){
	for ( int k = 0; k < 3; k++ )
		o[k] = g.r->C[k] + x * g.r->E1[k] + y * g.E2[k] + z * g.r->A[k];
}

static void Cog_Dir( const cogGen_t &g, float x, float y, float *o ){
	for ( int k = 0; k < 3; k++ )
		o[k] = x * g.r->E1[k] + y * g.E2[k];
}

static patchMesh_t* Cog_NewPatch( cogGen_t &g, int w, int h ){
	patchMesh_t *p = MakeNewPatch();
	p->pShader = g.shader;
	p->d_texture = g.shader->getTexture();
	p->width = w;
	p->height = h;
	p->type = PATCH_GENERIC;
	p->cogId = g.id;
	memcpy( p->cog, g.packed, sizeof( p->cog ) );
	return p;
}

// Like AddBrushForPatch, but the stand-in brush is never thin. Every patch sits on a brush made from its bounds,
// and building a brush snaps its planes to the grid: one that is only a little thicker than nothing (a wall that
// is nearly, but not quite, in a plane across an axis) collapses and gives "brush plane with no normal". Only a
// patch that is exactly flat was padded. Here any direction shorter than a minimum is widened round its middle.
#define COG_MIN_BRUSH 8.0f

static void Cog_Bounds( const patchMesh_t *p, float *lo, float *hi ){
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

static brush_t* Cog_AddBrushForPatch( patchMesh_t *pm ){
	float lo[3], hi[3];
	Cog_Bounds( pm, lo, hi );
	for ( int k = 0; k < 3; k++ )
		if ( hi[k] - lo[k] < COG_MIN_BRUSH ) {
			const float mid = 0.5f * ( lo[k] + hi[k] );
			lo[k] = mid - 0.5f * COG_MIN_BRUSH;
			hi[k] = mid + 0.5f * COG_MIN_BRUSH;
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

// Texture, facing, and into the map. want is the way the front of the patch has to face (out of the material).
static void Cog_Finish( cogGen_t &g, patchMesh_t *p, const float *want, bool planar ){
	if ( planar ) {
		// a cap: the texture is laid flat across it, so it carries on from one piece to the next
		for ( int i = 0; i < p->width; i++ )
			for ( int j = 0; j < p->height; j++ )
			{
				float d[3];
				VectorSubtract( p->ctrl[i][j].xyz, g.r->C, d );
				p->ctrl[i][j].st[0] = DotProduct( d, g.r->E1 ) / g.texW;
				p->ctrl[i][j].st[1] = DotProduct( d, g.E2 ) / g.texH;
			}
		p->bDirty = true;
	}
	else
	{
		Patch_Naturalize( p );
	}
	// The side a patch shows as its front is cross(dU,dV) (as Thicken Segmented found out on screen).
	const int mi = p->width / 2, last = p->height - 1;
	float dU[3], dV[3], n[3];
	VectorSubtract( p->ctrl[p->width - 1][last].xyz, p->ctrl[0][last].xyz, dU );
	VectorSubtract( p->ctrl[mi][last].xyz, p->ctrl[mi][0].xyz, dV );
	CrossProduct( dU, dV, n );
	if ( DotProduct( n, want ) < 0 ) {
		patchInvert( p );
	}
	// onto the grid: every control point is rounded to a multiple of the snap step (in the world, not from the
	// middle of the cog). A point shared by two patches comes out the same in both.
	if ( g.r->snap > 0.0f ) {
		// Only the points the surface passes through (even column and row) are rounded. A point between them
		// (the middle control point of an edge or of the face) moves by the average of the moves of its
		// neighbours on the curve, so a straight edge stays straight with its middle point exactly on the line,
		// and an arc keeps its bulge. Neighbours are shared between patches, so shared edges stay identical.
		const float st = g.r->snap;
		const int W = p->width, H = p->height;
		static float disp[MAX_PATCH_WIDTH][MAX_PATCH_HEIGHT][3];
		for ( int i = 0; i < W; i += 2 )
			for ( int j = 0; j < H; j += 2 )
				for ( int k = 0; k < 3; k++ )
					disp[i][j][k] = floorf( p->ctrl[i][j].xyz[k] / st + 0.5f ) * st - p->ctrl[i][j].xyz[k];
		for ( int i = 0; i < W; i++ )
			for ( int j = 0; j < H; j++ )
			{
				if ( ( i & 1 ) == 0 && ( j & 1 ) == 0 ) {
					continue;
				}
				const int i0 = ( i & 1 ) ? i - 1 : i, i1 = ( i & 1 ) ? i + 1 : i;
				const int j0 = ( j & 1 ) ? j - 1 : j, j1 = ( j & 1 ) ? j + 1 : j;
				for ( int k = 0; k < 3; k++ )
					disp[i][j][k] = 0.25f * ( disp[i0][j0][k] + disp[i1][j0][k] + disp[i0][j1][k] + disp[i1][j1][k] );
			}
		for ( int i = 0; i < W; i++ )
			for ( int j = 0; j < H; j++ )
				for ( int k = 0; k < 3; k++ )
					p->ctrl[i][j].xyz[k] += disp[i][j][k];
	}
	// A face that lies in a plane across an axis, give or take rounding, must lie exactly in it: the brush a patch
	// sits on is as thin as the patch is, and a paper thin one makes "brush plane with no normal" in the compilers.
	for ( int k = 0; k < 3; k++ )
	{
		float lo = 1e30f, hi = -1e30f;
		for ( int i = 0; i < p->width; i++ )
			for ( int j = 0; j < p->height; j++ )
			{
				const float v = p->ctrl[i][j].xyz[k];
				lo = Cog_Min( lo, v );
				hi = Cog_Max( hi, v );
			}
		if ( hi > lo && hi - lo < 0.01f ) {
			const float mid = 0.5f * ( lo + hi );
			for ( int i = 0; i < p->width; i++ )
				for ( int j = 0; j < p->height; j++ )
					p->ctrl[i][j].xyz[k] = mid;
		}
	}
	brush_t *b = Cog_AddBrushForPatch( p );
	Brush_AddToList( b, &active_brushes );
	Entity_LinkBrush( g.owner, b );
	Brush_Build( b );
	Patch_Rebuild( p );
	Undo_EndBrush( b );
	g.made.push_back( b );
}

// a point of the hole, on the circle of radius rh at the angle of the point (x, y)
static void Cog_OnHole( float rh, float x, float y, float *ox, float *oy ){
	const float l = sqrtf( x * x + y * y );
	*ox = l > 1e-6f ? rh * x / l : rh;
	*oy = l > 1e-6f ? rh * y / l : 0.0f;
}

// The control point between two points of the hole. The hole is a polygon with a flat piece for every piece of the
// root polygon (like the segments of the cylinder), so the point lies exactly on the line between the two.
static void Cog_HoleMid( float ax, float ay, float bx, float by, float *ox, float *oy ){
	*ox = 0.5f * ( ax + bx );
	*oy = 0.5f * ( ay + by );
}

// The patches of the cog, all linked into owner. Needs an open undo step. Returns how many.
static int Cog_Generate( const cogRec_t &r, int id, IShader *shader, entity_t *owner, std::vector<brush_t*> &made ){
	cogShape_t s;
	if ( !Cog_Shape( r, s ) ) {
		return 0;
	}
	cogGen_t g;
	g.r = &r;
	CrossProduct( r.A, r.E1, g.E2 );
	g.id = id;
	Cog_Pack( r, g.packed );
	g.shader = shader;
	g.owner = owner ? owner : world_entity;
	g.texW = Cog_Max( 1.0f, shader->getTexture()->width * g_PrefsDlg.m_fDefTextureScale );
	g.texH = Cog_Max( 1.0f, shader->getTexture()->height * g_PrefsDlg.m_fDefTextureScale );

	const int n = s.n;
	const int N = r.teeth;
	const bool hollow = r.hole > 0;
	float want[3];

	// the root polygon: the base of every tooth and every gap, with the control point of each edge
	struct rootEdge_t
	{
		float ax, ay, cx, cy, bx, by;
	};
	std::vector<rootEdge_t> root;
	for ( int k = 0; k < N; k++ )
	{
		const int bl = ( 4 * k + n - 1 ) % n, br = 4 * k + 2, gl = 4 * k + 2, gr = 4 * k + 3;
		rootEdge_t base = { s.vx[bl], s.vy[bl], 0.5f * ( s.vx[bl] + s.vx[br] ), 0.5f * ( s.vy[bl] + s.vy[br] ), s.vx[br], s.vy[br] };
		rootEdge_t gap = { s.vx[gl], s.vy[gl], s.cx[gl], s.cy[gl], s.vx[gr], s.vy[gr] };
		root.push_back( base );
		root.push_back( gap );
	}

	// walls: along the edge across 3, down the axis across 3
	for ( int i = 0; i < n; i++ )
	{
		const int j = ( i + 1 ) % n;
		patchMesh_t *p = Cog_NewPatch( g, 3, 3 );
		const float px[3] = { s.vx[i], s.cx[i], s.vx[j] };
		const float py[3] = { s.vy[i], s.cy[i], s.vy[j] };
		// the top of the wall is where the cap is (it slopes with the cap); the middle point is half way
		const float zt0 = Cog_CapZ( r, s, 0, px[0], py[0] ), zt2 = Cog_CapZ( r, s, 0, px[2], py[2] );
		const float zb0 = Cog_CapZ( r, s, 1, px[0], py[0] ), zb2 = Cog_CapZ( r, s, 1, px[2], py[2] );
		const float zt[3] = { zt0, 0.5f * ( zt0 + zt2 ), zt2 }, zb[3] = { zb0, 0.5f * ( zb0 + zb2 ), zb2 };
		for ( int a = 0; a < 3; a++ )
		{
			Cog_Lift( g, px[a], py[a], zt[a], p->ctrl[a][0].xyz );
			Cog_Lift( g, px[a], py[a], 0.0f, p->ctrl[a][1].xyz );
			Cog_Lift( g, px[a], py[a], -zb[a], p->ctrl[a][2].xyz );
		}
		Cog_Dir( g, s.vy[j] - s.vy[i], -( s.vx[j] - s.vx[i] ), want ); // out of the material
		Cog_Finish( g, p, want, false );
	}

	// the bore: a wall for every piece of the root polygon, facing the axis
	if ( hollow ) {
		for ( size_t e = 0; e < root.size(); e++ )
		{
			const rootEdge_t &re = root[e];
			float hax, hay, hbx, hby, hcx, hcy;
			Cog_OnHole( r.hole, re.ax, re.ay, &hax, &hay );
			Cog_OnHole( r.hole, re.bx, re.by, &hbx, &hby );
			Cog_HoleMid( hax, hay, hbx, hby, &hcx, &hcy );
			patchMesh_t *p = Cog_NewPatch( g, 3, 3 );
			const float px[3] = { hax, hcx, hbx };
			const float py[3] = { hay, hcy, hby };
			const float zt0 = Cog_CapZ( r, s, 0, px[0], py[0] ), zt2 = Cog_CapZ( r, s, 0, px[2], py[2] );
			const float zb0 = Cog_CapZ( r, s, 1, px[0], py[0] ), zb2 = Cog_CapZ( r, s, 1, px[2], py[2] );
			const float zt[3] = { zt0, 0.5f * ( zt0 + zt2 ), zt2 }, zb[3] = { zb0, 0.5f * ( zb0 + zb2 ), zb2 };
			for ( int a = 0; a < 3; a++ )
			{
				Cog_Lift( g, px[a], py[a], zt[a], p->ctrl[a][0].xyz );
				Cog_Lift( g, px[a], py[a], 0.0f, p->ctrl[a][1].xyz );
				Cog_Lift( g, px[a], py[a], -zb[a], p->ctrl[a][2].xyz );
			}
			Cog_Dir( g, -( hby - hay ), hbx - hax, want ); // towards the axis
			Cog_Finish( g, p, want, false );
		}
	}

	// the two caps
	for ( int cap = 0; cap < 2; cap++ )
	{
		const float sg = cap == 0 ? 1.0f : -1.0f;
		for ( int k = 0; k < 3; k++ )
			want[k] = cap == 0 ? r.A[k] : -r.A[k];

		// the top of every tooth: one quad, from the base (row 0) to the tip (row 2), from its left to its right
		for ( int k = 0; k < N; k++ )
		{
			const int bl = ( 4 * k + n - 1 ) % n, tl = 4 * k, tr = 4 * k + 1, br = 4 * k + 2;
			patchMesh_t *p = Cog_NewPatch( g, 3, 3 );
			p->cog[COG_PARTSLOT] = (float)( 1000 + cap * 100 + k ); // the top of tooth k of this end
			const float zbl = Cog_CapZ( r, s, cap, s.vx[bl], s.vy[bl] ), zbr = Cog_CapZ( r, s, cap, s.vx[br], s.vy[br] );
			const float ztl = Cog_CapZ( r, s, cap, s.vx[tl], s.vy[tl] ), ztr = Cog_CapZ( r, s, cap, s.vx[tr], s.vy[tr] );
			Cog_Lift( g, s.vx[bl], s.vy[bl], sg * zbl, p->ctrl[0][0].xyz );
			Cog_Lift( g, s.vx[br], s.vy[br], sg * zbr, p->ctrl[2][0].xyz );
			Cog_Lift( g, s.vx[tl], s.vy[tl], sg * ztl, p->ctrl[0][2].xyz );
			Cog_Lift( g, s.vx[tr], s.vy[tr], sg * ztr, p->ctrl[2][2].xyz );
			Cog_Lift( g, 0.5f * ( s.vx[bl] + s.vx[br] ), 0.5f * ( s.vy[bl] + s.vy[br] ), sg * 0.5f * ( zbl + zbr ), p->ctrl[1][0].xyz );
			Cog_Lift( g, s.cx[tl], s.cy[tl], sg * 0.5f * ( ztl + ztr ), p->ctrl[1][2].xyz );
			Cog_Lift( g, 0.5f * ( s.vx[bl] + s.vx[tl] ), 0.5f * ( s.vy[bl] + s.vy[tl] ), sg * 0.5f * ( zbl + ztl ), p->ctrl[0][1].xyz );
			Cog_Lift( g, 0.5f * ( s.vx[br] + s.vx[tr] ), 0.5f * ( s.vy[br] + s.vy[tr] ), sg * 0.5f * ( zbr + ztr ), p->ctrl[2][1].xyz );
			for ( int k3 = 0; k3 < 3; k3++ )
				p->ctrl[1][1].xyz[k3] = 0.5f * ( p->ctrl[1][0].xyz[k3] + p->ctrl[1][2].xyz[k3] );
			Cog_Finish( g, p, want, true );
		}

		// the rest of the cap: a piece to every edge of the root polygon, from the middle (or the edge of the
		// hole) in row 0 out to the edge in row 2
		for ( size_t e = 0; e < root.size(); e++ )
		{
			const rootEdge_t &re = root[e];
			float ix[3], iy[3];
			if ( hollow ) {
				Cog_OnHole( r.hole, re.ax, re.ay, &ix[0], &iy[0] );
				Cog_OnHole( r.hole, re.bx, re.by, &ix[2], &iy[2] );
				Cog_HoleMid( ix[0], iy[0], ix[2], iy[2], &ix[1], &iy[1] );
			}
			else
			{
				ix[0] = ix[1] = ix[2] = 0.0f;
				iy[0] = iy[1] = iy[2] = 0.0f;
			}
			const float ox[3] = { re.ax, re.cx, re.bx };
			const float oy[3] = { re.ay, re.cy, re.by };
			patchMesh_t *p = Cog_NewPatch( g, 3, 3 );
			p->cog[COG_PARTSLOT] = (float)( 10 + cap ); // a piece of this end
			const float zi0 = Cog_CapZ( r, s, cap, ix[0], iy[0] ), zi2 = Cog_CapZ( r, s, cap, ix[2], iy[2] );
			const float zo0 = Cog_CapZ( r, s, cap, ox[0], oy[0] ), zo2 = Cog_CapZ( r, s, cap, ox[2], oy[2] );
			const float zi[3] = { zi0, 0.5f * ( zi0 + zi2 ), zi2 };
			const float zo[3] = { zo0, 0.5f * ( zo0 + zo2 ), zo2 };
			for ( int a = 0; a < 3; a++ )
			{
				Cog_Lift( g, ix[a], iy[a], sg * zi[a], p->ctrl[a][0].xyz );
				Cog_Lift( g, ox[a], oy[a], sg * zo[a], p->ctrl[a][2].xyz );
				for ( int k3 = 0; k3 < 3; k3++ )
					p->ctrl[a][1].xyz[k3] = 0.5f * ( p->ctrl[a][0].xyz[k3] + p->ctrl[a][2].xyz[k3] );
			}
			Cog_Finish( g, p, want, true );
		}
	}
	made = g.made;
	return (int)made.size();
}

// =============================================================================
// finding cogs

static bool Cog_IsCogPatch( brush_t *b ){
	return b->patchBrush && b->pPatch && b->pPatch->cogId > 0;
}

// every patch of the cog with this id
static void Cog_Collect( int id, std::vector<brush_t*> &out ){
	brush_t *b;
	for ( b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( Cog_IsCogPatch( b ) && b->pPatch->cogId == id ) {
			out.push_back( b );
		}
	for ( b = active_brushes.next; b != &active_brushes; b = b->next )
		if ( Cog_IsCogPatch( b ) && b->pPatch->cogId == id ) {
			out.push_back( b );
		}
}

// The pieces of bevels made on a cog (strips and corner caps) carry the cog's id as a negative number, so that
// they stay part of it when the map is saved and loaded again (the bevel records do not). They are not cog
// patches in the sense of Cog_IsCogPatch: they have none of the cog's own numbers to edit.
static void Cog_CollectBevels( int id, std::vector<brush_t*> &out ){
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( b->patchBrush && b->pPatch && b->pPatch->cogId == -id ) {
				out.push_back( b );
			}
	}
}

static int Cog_NextId(){
	int top = 0;
	brush_t *b;
	for ( b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( Cog_IsCogPatch( b ) && b->pPatch->cogId > top ) {
			top = b->pPatch->cogId;
		}
	for ( b = active_brushes.next; b != &active_brushes; b = b->next )
		if ( Cog_IsCogPatch( b ) && b->pPatch->cogId > top ) {
			top = b->pPatch->cogId;
		}
	return top + 1;
}

bool Cog_Selected( cogRec_t *out, int *id ){
	brush_t *first = NULL;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( Cog_IsCogPatch( b ) ) {
			first = b;
			break;
		}
	if ( first == NULL ) {
		return false;
	}
	cogRec_t r;
	Cog_Unpack( first->pPatch->cog, r );
	const int cid = first->pPatch->cogId;

	std::vector<brush_t*> all;
	Cog_Collect( cid, all );

	bool framed = false;
	// Where the cog points now, if it was rotated after it was made. The pieces of a cap are the same all the way
	// round the axis, so the middle of all the points of one cap is on the axis: the axis runs from the bottom cap to
	// the top one. The direction of tooth 0 is the one from the axis to the middle of the top of tooth 0 (the patches
	// say which part they are, see COG_PARTSLOT; cogs saved before that keep the direction they had).
	{
		double cs[2][3] = { { 0, 0, 0 }, { 0, 0, 0 } };
		long cn[2] = { 0, 0 };
		const patchMesh_t *tooth0 = NULL;
		for ( size_t i = 0; i < all.size(); i++ )
		{
			const patchMesh_t *p = all[i]->pPatch;
			const int code = (int)( p->cog[COG_PARTSLOT] + 0.5f );
			int cap = -1;
			if ( code == 10 || code == 11 ) {
				cap = code - 10;
			}
			else if ( code >= 1000 && code < 1200 ) {
				cap = ( code - 1000 ) / 100;
				if ( code == 1000 ) {
					tooth0 = p;
				}
			}
			if ( cap < 0 ) {
				continue;
			}
			for ( int a = 0; a < p->width; a++ )
				for ( int b = 0; b < p->height; b++ )
				{
					for ( int k = 0; k < 3; k++ )
						cs[cap][k] += p->ctrl[a][b].xyz[k];
					cn[cap]++;
				}
		}
		if ( cn[0] > 0 && cn[1] > 0 ) {
			float d[3], u[3];
			for ( int k = 0; k < 3; k++ )
				d[k] = (float)( cs[0][k] / cn[0] - cs[1][k] / cn[1] );
			if ( VectorLength( d ) > 0.5f ) {
				VectorScale( d, 1.0f / VectorLength( d ), u );
				// the direction of tooth 0, across the new axis
				float e[3] = { 0, 0, 0 };
				bool haveE = false;
				if ( tooth0 ) {
					double ms[3] = { 0, 0, 0 };
					for ( int a = 0; a < 3; a++ )
						for ( int b = 0; b < 3; b++ )
							for ( int k = 0; k < 3; k++ )
								ms[k] += tooth0->ctrl[a][b].xyz[k] / 9.0;
					float axisPt[3];
					for ( int k = 0; k < 3; k++ )
						axisPt[k] = (float)( cs[0][k] / cn[0] );
					for ( int k = 0; k < 3; k++ )
						e[k] = (float)ms[k] - axisPt[k];
					const float al = DotProduct( e, u );
					for ( int k = 0; k < 3; k++ )
						e[k] -= al * u[k];
					haveE = VectorLength( e ) > 0.1f;
				}
				if ( !haveE ) {
					const float al = DotProduct( r.E1, u );
					for ( int k = 0; k < 3; k++ )
						e[k] = r.E1[k] - al * u[k];
					haveE = VectorLength( e ) > 0.1f;
				}
				if ( haveE ) {
					VectorScale( e, 1.0f / VectorLength( e ), e );
					VectorCopy( u, r.A );
					VectorCopy( e, r.E1 );
					framed = tooth0 != NULL;
				}
			}
		}
	}
	if ( !framed ) {
		// A cog without the part codes: the axis is the direction the columns of most of the 3 x 3 patches run in (the
		// walls: three columns, all along the axis), the sign the one nearest to what it was. The cog looks the same
		// turned by the pitch of its teeth, so the direction of any tooth will do: the one nearest to what it was.
		struct colDir_t
		{
			const patchMesh_t *p;
			float u[3];
		};
		std::vector<colDir_t> cand;
		for ( size_t i = 0; i < all.size(); i++ )
		{
			const patchMesh_t *p = all[i]->pPatch;
			if ( p->width != 3 || p->height != 3 ) {
				continue;
			}
			colDir_t c;
			c.p = p;
			bool ok = true;
			for ( int a = 0; a < 3 && ok; a++ )
			{
				float d[3];
				VectorSubtract( p->ctrl[a][2].xyz, p->ctrl[a][0].xyz, d );
				const float l = VectorLength( d );
				if ( l < 0.5f ) {
					ok = false;
					break;
				}
				VectorScale( d, 1.0f / l, d );
				if ( a == 0 ) {
					VectorCopy( d, c.u );
				}
				else if ( fabsf( DotProduct( d, c.u ) ) < 0.999f ) {
					ok = false;
				}
			}
			if ( ok ) {
				cand.push_back( c );
			}
		}
		size_t bestAt = 0;
		int bestN = 0;
		for ( size_t i = 0; i < cand.size(); i++ )
		{
			int n = 0;
			for ( size_t j = 0; j < cand.size(); j++ )
				if ( fabsf( DotProduct( cand[i].u, cand[j].u ) ) > 0.999f ) {
					n++;
				}
			if ( n > bestN ) {
				bestN = n;
				bestAt = i;
			}
		}
		if ( bestN >= 6 ) {
			float u[3];
			VectorCopy( cand[bestAt].u, u );
			if ( DotProduct( u, r.A ) < 0 ) {
				VectorScale( u, -1.0f, u );
			}
			std::vector<const patchMesh_t*> walls;
			double ws[3] = { 0, 0, 0 };
			for ( size_t i = 0; i < cand.size(); i++ )
				if ( fabsf( DotProduct( cand[i].u, u ) ) > 0.999f ) {
					walls.push_back( cand[i].p );
					for ( int a = 0; a < 3; a++ )
						for ( int k = 0; k < 3; k++ )
							ws[k] += cand[i].p->ctrl[a][1].xyz[k] / ( 3.0 * bestN );
				}
			float cw[3] = { (float)ws[0], (float)ws[1], (float)ws[2] };
			// the walls whose middle is furthest from the axis are the tips of the teeth
			std::vector<std::vector<float> > dirs;
			std::vector<float> dist;
			float maxD = 0;
			for ( size_t i = 0; i < walls.size(); i++ )
			{
				float m[3];
				VectorSubtract( walls[i]->ctrl[1][1].xyz, cw, m );
				const float al = DotProduct( m, u );
				for ( int k = 0; k < 3; k++ )
					m[k] -= al * u[k];
				const float l = VectorLength( m );
				std::vector<float> v( m, m + 3 );
				dirs.push_back( v );
				dist.push_back( l );
				maxD = Cog_Max( maxD, l );
			}
			float want[3];
			const float al = DotProduct( r.E1, u );
			for ( int k = 0; k < 3; k++ )
				want[k] = r.E1[k] - al * u[k];
			float bestDot = -2.0f;
			float pick[3] = { 0, 0, 0 };
			for ( size_t i = 0; i < walls.size(); i++ )
			{
				if ( dist[i] < 0.98f * maxD || dist[i] < 0.1f ) {
					continue;
				}
				float dp = 0;
				for ( int k = 0; k < 3; k++ )
					dp += dirs[i][k] / dist[i] * want[k];
				if ( dp > bestDot ) {
					bestDot = dp;
					for ( int k = 0; k < 3; k++ )
						pick[k] = dirs[i][k] / dist[i];
				}
			}
			if ( bestDot > -2.0f ) {
				VectorCopy( u, r.A );
				VectorCopy( pick, r.E1 );
			}
		}
	}

	// The cog is the same all the way round the axis, so the middle of the middle row of control points of its walls
	// (the points half way along the axis, whatever the slope of the two ends) is the middle of the cog: that is
	// where it is now, if it was moved after it was made. If no wall is found (turned by hand) all points are used.
	double sum[3] = { 0, 0, 0 };
	long count = 0;
	for ( int pass = 0; pass < 2 && count == 0; pass++ )
		for ( size_t i = 0; i < all.size(); i++ )
		{
			const patchMesh_t *p = all[i]->pPatch;
			bool wall = pass == 1;
			if ( !wall && p->width == 3 && p->height == 3 ) {
				wall = true;
				for ( int a = 0; a < 3 && wall; a++ )
				{
					float d[3];
					VectorSubtract( p->ctrl[a][2].xyz, p->ctrl[a][0].xyz, d );
					const float l = VectorLength( d );
					wall = l > 0.5f && fabsf( DotProduct( d, r.A ) ) > 0.99f * l;
				}
			}
			if ( !wall ) {
				continue;
			}
			for ( int a = 0; a < p->width; a++ )
				for ( int b = 0; b < p->height; b++ )
				{
					if ( pass == 0 && b != 1 ) {
						continue;
					}
					for ( int k = 0; k < 3; k++ )
						sum[k] += p->ctrl[a][b].xyz[k];
					count++;
				}
		}
	if ( count > 0 ) {
		float mid[3] = { (float)( sum[0] / count ), (float)( sum[1] / count ), (float)( sum[2] / count ) };
		float d[3];
		VectorSubtract( mid, r.C, d );
		if ( VectorLength( d ) > 0.75f ) {
			VectorCopy( mid, r.C );
		}
	}
	*out = r;
	if ( id ) {
		*id = cid;
	}
	return true;
}

void Bevel_StripsOf( const std::vector<brush_t*> &parts, std::vector<brush_t*> &out ); // bevel.cpp

static bool Cog_InSelection( brush_t *b ){
	for ( brush_t *p = selected_brushes.next; p != &selected_brushes; p = p->next )
		if ( p == b ) {
			return true;
		}
	return false;
}

int Cog_SelectWhole(){
	std::vector<int> ids;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->cogId != 0 ) { // a patch of a cog, or of a bevel made on one
			const int id = abs( b->pPatch->cogId );
			bool have = false;
			for ( size_t i = 0; i < ids.size(); i++ )
				have = have || ids[i] == id;
			if ( !have ) {
				ids.push_back( id );
			}
		}
	if ( ids.empty() ) {
		Sys_Printf( "Select Whole Cog: select a patch of a cog first.\n" );
		return 0;
	}
	std::vector<brush_t*> parts;
	for ( size_t i = 0; i < ids.size(); i++ )
	{
		Cog_Collect( ids[i], parts );
		Cog_CollectBevels( ids[i], parts );
	}
	// bevels made on the cog with Bevel Edge are part of it too (also those not marked, from the records)
	std::vector<brush_t*> strips;
	Bevel_StripsOf( parts, strips );
	for ( size_t i = 0; i < strips.size(); i++ )
		if ( std::find( parts.begin(), parts.end(), strips[i] ) == parts.end() ) {
			parts.push_back( strips[i] );
		}
	int added = 0, total = 0;
	for ( size_t i = 0; i < parts.size(); i++ )
	{
		if ( !Cog_InSelection( parts[i] ) ) {
			Select_Brush( parts[i] );
			added++;
		}
		total++;
	}
	Sys_Printf( "Select Whole Cog: %d patches selected (%d added).\n", total, added );
	Sys_UpdateWindows( W_ALL );
	return added;
}

// =============================================================================
// making and changing

static void Cog_DeleteBrushes( const std::vector<brush_t*> &v ){
	Select_Deselect( true );
	for ( size_t i = 0; i < v.size(); i++ )
		Select_Brush( v[i] );
	Undo_AddBrushList( &selected_brushes );
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		Undo_AddEntity( b->owner );
	Select_Delete();
	Undo_EndBrushList( &selected_brushes );
}

static float Cog_Len( const float *v ){
	return sqrtf( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
}

// the cylinder in a ring of control points: the middle, the axis, the radius, a first direction
static bool Cog_ReadCylinder( patchMesh_t *p, cogRec_t *r ){
	const int W = p->width, H = p->height;
	bool alongW = false, alongH = false;
	if ( W >= 5 && H >= 3 ) {
		alongW = true;
		for ( int j = 0; j < H && alongW; j++ )
		{
			float d[3];
			VectorSubtract( p->ctrl[0][j].xyz, p->ctrl[W - 1][j].xyz, d );
			if ( Cog_Len( d ) > 0.5f ) {
				alongW = false;
			}
		}
	}
	if ( !alongW && H >= 5 && W >= 3 ) {
		alongH = true;
		for ( int i = 0; i < W && alongH; i++ )
		{
			float d[3];
			VectorSubtract( p->ctrl[i][0].xyz, p->ctrl[i][H - 1].xyz, d );
			if ( Cog_Len( d ) > 0.5f ) {
				alongH = false;
			}
		}
	}
	if ( !alongW && !alongH ) {
		return false;
	}
	const int ringN = alongW ? W : H;       // control points round the ring (the last one is the first again)
	const int axisN = alongW ? H : W;       // rows from one end to the other
	#define COG_PT( a, b ) ( alongW ? p->ctrl[a][b].xyz : p->ctrl[b][a].xyz )

	float c0[3] = { 0, 0, 0 }, c1[3] = { 0, 0, 0 };
	const int cnt = ( ringN - 1 ) / 2;
	for ( int a = 0; a < ringN - 1; a += 2 )
		for ( int k = 0; k < 3; k++ )
		{
			c0[k] += COG_PT( a, 0 )[k] / cnt;
			c1[k] += COG_PT( a, axisN - 1 )[k] / cnt;
		}
	float axis[3];
	VectorSubtract( c1, c0, axis );
	const float height = Cog_Len( axis );
	if ( height < 1.0f ) {
		return false; // a disc, not a cylinder
	}
	VectorScale( axis, 1.0f / height, axis );

	float rad = 0;
	for ( int a = 0; a < ringN - 1; a += 2 )
		for ( int e = 0; e < 2; e++ )
		{
			float d[3];
			VectorSubtract( COG_PT( a, e ? axisN - 1 : 0 ), e ? c1 : c0, d );
			const float along = DotProduct( d, axis );
			VectorMA( d, -along, axis, d );
			rad += Cog_Len( d ) / ( 2.0f * cnt );
		}
	float e1[3];
	VectorSubtract( COG_PT( 0, 0 ), c0, e1 );
	VectorMA( e1, -DotProduct( e1, axis ), axis, e1 );
	const float l1 = Cog_Len( e1 );
	#undef COG_PT
	if ( l1 < 0.5f || rad < 1.0f ) {
		return false;
	}
	VectorScale( e1, 1.0f / l1, e1 );

	r->cylR = rad;
	r->height = height;
	for ( int k = 0; k < 3; k++ )
	{
		r->C[k] = 0.5f * ( c0[k] + c1[k] );
		r->A[k] = axis[k];
		r->E1[k] = e1[k];
	}
	return true;
}

bool Cog_MakeFromSelected(){
	brush_t *src = NULL;
	cogRec_t r;
	memset( &r, 0, sizeof( r ) );
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && b->pPatch->cogId == 0 && Cog_ReadCylinder( b->pPatch, &r ) ) {
			src = b;
			break;
		}
	if ( src == NULL ) {
		Sys_Printf( "Make Cog: select a cylinder patch first (a segmented cylinder gives the teeth their count).\n" );
		return false;
	}
	patchMesh_t *sp = src->pPatch;

	// first guesses, in proportion to the cylinder
	// every tooth takes two segments of the cylinder (the base of the tooth and the gap), so a cylinder of 16
	// segments gives 8 teeth
	r.teeth = sp->segH >= 2 * COG_MIN_TEETH ? sp->segH / 2 : 12;
	if ( r.teeth > COG_MAX_TEETH ) {
		r.teeth = COG_MAX_TEETH;
	}
	r.mode = g_cogLastMode;
	r.snap = g_cogLastSnap;
	r.depth = 0.2f * r.cylR;
	r.taper = 0.0f;
	r.tipArc = r.rootArc = 0;
	r.hole = 0;
	r.roundSeg = 4;
	cogRec_t chk = r;
	cogShape_t s;
	if ( !Cog_Shape( chk, s ) ) {
		Sys_Printf( "Make Cog: the cylinder is too small for %d teeth.\n", r.teeth );
		return false;
	}

	IShader *shader = sp->pShader;
	entity_t *owner = src->owner;
	const int id = Cog_NextId();
	std::vector<brush_t*> made;

	BevelEdge_Clear();
	Undo_Start( "make cog" );
	std::vector<brush_t*> old;
	old.push_back( src );
	Cog_DeleteBrushes( old );
	const int count = Cog_Generate( r, id, shader, owner, made );
	Undo_End();

	Select_Deselect( true );
	for ( size_t i = 0; i < made.size(); i++ )
		Select_Brush( made[i] );
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	Sys_Printf( "Make Cog: %d teeth, %d patches.\n", r.teeth, count );
	return count > 0;
}

bool Cog_Rebuild( const cogRec_t &in ){
	cogRec_t cur;
	int id = 0;
	if ( !Cog_Selected( &cur, &id ) ) {
		return false;
	}
	cogRec_t r = in;
	g_cogLastMode = r.mode; // the next cog is made the way this one was last set
	g_cogLastSnap = r.snap;
	// where the cog is now is not something the panel edits
	for ( int k = 0; k < 3; k++ )
	{
		r.C[k] = cur.C[k];
		r.A[k] = cur.A[k];
		r.E1[k] = cur.E1[k];
	}
	r.height = cur.height;
	cogRec_t chk = r;
	cogShape_t s;
	if ( !Cog_Shape( chk, s ) ) {
		return false; // not a cog with these numbers: leave it as it is
	}

	std::vector<brush_t*> old;
	Cog_Collect( id, old );
	if ( old.empty() ) {
		return false;
	}
	IShader *shader = old[0]->pPatch->pShader;
	entity_t *owner = old[0]->owner;

	// bevels made on the cog go with it (they would be left hanging in the air otherwise)
	{
		Cog_CollectBevels( id, old );
		std::vector<brush_t*> strips;
		Bevel_StripsOf( old, strips );
		for ( size_t i = 0; i < strips.size(); i++ )
			if ( std::find( old.begin(), old.end(), strips[i] ) == old.end() ) {
				old.push_back( strips[i] );
			}
	}
	BevelEdge_Clear();
	std::vector<brush_t*> made;
	Undo_Start( "edit cog" );
	Cog_DeleteBrushes( old );
	const int count = Cog_Generate( r, id, shader, owner, made );
	Undo_End();

	Select_Deselect( true );
	for ( size_t i = 0; i < made.size(); i++ )
		Select_Brush( made[i] );
	UpdatePatchInspector();
	Sys_UpdateWindows( W_ALL );
	return count > 0;
}

// =============================================================================
// copies

// Copy / paste / clone bring patches that carry the id of the cog they were copied from. A copy must be a cog of
// its own (the panel and Select Whole Cog work on one id), so the patches that came in with an id that is already
// used by a patch that was there before get a new one (the same new one for all patches of one old id).
static std::vector<brush_t*> g_cogBefore;

void Cog_ImportBegin(){
	g_cogBefore.clear();
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( b->patchBrush && b->pPatch && b->pPatch->cogId != 0 ) {
				g_cogBefore.push_back( b );
			}
	}
}

void Cog_ImportEnd(){
	int top = 0;
	std::vector<int> oldIds;
	for ( size_t i = 0; i < g_cogBefore.size(); i++ )
		oldIds.push_back( abs( g_cogBefore[i]->pPatch->cogId ) );
	std::vector<brush_t*> fresh;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
		{
			if ( !b->patchBrush || !b->pPatch || b->pPatch->cogId == 0 ) {
				continue;
			}
			if ( abs( b->pPatch->cogId ) > top ) {
				top = abs( b->pPatch->cogId );
			}
			if ( std::find( g_cogBefore.begin(), g_cogBefore.end(), b ) == g_cogBefore.end() ) {
				fresh.push_back( b );
			}
		}
	}
	std::vector<int> from, to;
	for ( size_t i = 0; i < fresh.size(); i++ )
	{
		patchMesh_t *p = fresh[i]->pPatch;
		const int id = abs( p->cogId );
		if ( std::find( oldIds.begin(), oldIds.end(), id ) == oldIds.end() ) {
			continue; // not a copy of a cog that is here
		}
		size_t k = 0;
		while ( k < from.size() && from[k] != id )
			k++;
		if ( k == from.size() ) {
			from.push_back( id );
			to.push_back( ++top );
		}
		p->cogId = p->cogId < 0 ? -to[k] : to[k];
	}
	g_cogBefore.clear();
}
