/*
   Segmented patches

   A segmented patch is an ordinary patch (the "cage": its control points are edited as usual) plus
   two segment counts, patchMesh_t::segH along the width and patchMesh_t::segV along the height.
   The surface it stands for is a regular grid of segH x segV quads that is generated from the cage:

   - along every row of the cage each arc is read as a conic section with the weight cos( theta / 2 ),
     where theta is the angle the arc turns through. With the off-curve point on the corner (the
     regular cylinder layout) that is an exact circle.
   - the points are placed at equal length along the whole row (and then along the whole column of
     the resulting points), so the segments are all the same size no matter how the arcs are set up.
   - the grid is cut into pieces of at most SEGPATCH_PIECE_SEGS segments per direction. Each piece is
     an ordinary patch whose control points are the grid points with the points between them halfway
     in between, which makes it exactly the flat quads. q3map2 and the game see only these.

   Header only, so the editor (preview) and the map plugin (save) use the very same code.
   patchMesh_t must be declared before this is included.
 */

#ifndef __SEGPATCH_H__
#define __SEGPATCH_H__

#include <math.h>
#include <vector>
#include <algorithm>
#include <utility>

#define SEGPATCH_MIN_H          3
#define SEGPATCH_MIN_V          1
#define SEGPATCH_MAX_SEGS       64
#define SEGPATCH_DEFAULT_H      16
#define SEGPATCH_DEFAULT_V      1
#define SEGPATCH_PIECE_SEGS     15      // 2 * 15 + 1 control points fit in MAX_PATCH_WIDTH / HEIGHT

struct segPoint_t
{
	float xyz[3];
	float st[2];
};

struct segGrid_t
{
	int U, V;                       // points along the width / height: segH + 1, segV + 1
	std::vector<segPoint_t> p;      // p[ u * V + v ]
};

struct segPiece_t
{
	int width, height;              // control points
	std::vector<segPoint_t> c;      // c[ x * height + y ]
};

inline int SegPatch_ClampH( int h, int minH = SEGPATCH_MIN_H ){
	return h < minH ? minH : ( h > SEGPATCH_MAX_SEGS ? SEGPATCH_MAX_SEGS : h );
}

// a patch whose first and last column coincide is a ring along its width and needs at least 3
// horizontal segments to be one; any other patch (a strip, a wall) may go down to 1
inline int SegPatch_MinH( const patchMesh_t *p ){
	if ( p->width < 5 ) {
		return 1;
	}
	for ( int j = 0; j < p->height; j++ )
	{
		const float *a = p->ctrl[0][j].xyz, *b = p->ctrl[p->width - 1][j].xyz;
		float d[3] = { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
		if ( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] > 0.25f ) {
			return 1;
		}
	}
	return SEGPATCH_MIN_H;
}

inline int SegPatch_ClampV( int v ){
	return v < SEGPATCH_MIN_V ? SEGPATCH_MIN_V : ( v > SEGPATCH_MAX_SEGS ? SEGPATCH_MAX_SEGS : v );
}

// ---- one line of control points -> equal length samples -------------------------------------------

struct segSamp_t
{
	int arc;        // which arc of the line
	float f;        // fraction along that arc, by length (0..1)
	float xyz[3];
};

// weight of the off-curve point C of the arc A-C-B: cos( turning angle / 2 )
inline float SegPatch_Weight( const float *A, const float *C, const float *B ){
	float a[3], b[3];
	for ( int n = 0; n < 3; n++ )
	{
		a[n] = A[n] - C[n];
		b[n] = B[n] - C[n];
	}
	float la = sqrtf( a[0] * a[0] + a[1] * a[1] + a[2] * a[2] );
	float lb = sqrtf( b[0] * b[0] + b[1] * b[1] + b[2] * b[2] );
	if ( la < 0.001f || lb < 0.001f ) {
		return 1.0f;
	}
	float cosPhi = ( a[0] * b[0] + a[1] * b[1] + a[2] * b[2] ) / ( la * lb );
	if ( cosPhi > 1.0f ) {
		cosPhi = 1.0f;
	}
	if ( cosPhi < -1.0f ) {
		cosPhi = -1.0f;
	}
	float theta = 3.14159265f - acosf( cosPhi );
	if ( theta < 0.001f || theta > 2.4f ) {
		return 1.0f; // straight, or too wide to read as a circle
	}
	return cosf( theta * 0.5f );
}

inline void SegPatch_EvalArc( const float *A, const float *C, const float *B, float w, float t, float *out ){
	float b0 = ( 1 - t ) * ( 1 - t );
	float b1 = 2 * t * ( 1 - t ) * w;
	float b2 = t * t;
	float inv = 1.0f / ( b0 + b1 + b2 );
	for ( int n = 0; n < 3; n++ )
		out[n] = ( b0 * A[n] + b1 * C[n] + b2 * B[n] ) * inv;
}

// pts: n control points (n odd, at least 3) as n * 3 floats; count + 1 samples at equal length
inline bool SegPatch_SampleLine( const float *pts, int n, int count, std::vector<segSamp_t> &out ){
	if ( n < 3 || ( n & 1 ) == 0 || count < 1 ) {
		return false;
	}
	const int S = 48;
	const int nArc = ( n - 1 ) / 2;
	std::vector<float> cum( nArc * ( S + 1 ) );   // length along each arc, per table step
	std::vector<float> w( nArc ), arcLen( nArc );
	float total = 0;
	int k, i;

	for ( k = 0; k < nArc; k++ )
	{
		const float *A = pts + ( 2 * k ) * 3;
		const float *C = pts + ( 2 * k + 1 ) * 3;
		const float *B = pts + ( 2 * k + 2 ) * 3;
		w[k] = SegPatch_Weight( A, C, B );
		float prev[3], cur[3];
		float *c = &cum[k * ( S + 1 )];
		c[0] = 0;
		SegPatch_EvalArc( A, C, B, w[k], 0.0f, prev );
		for ( i = 1; i <= S; i++ )
		{
			SegPatch_EvalArc( A, C, B, w[k], (float)i / S, cur );
			float d0 = cur[0] - prev[0], d1 = cur[1] - prev[1], d2 = cur[2] - prev[2];
			c[i] = c[i - 1] + sqrtf( d0 * d0 + d1 * d1 + d2 * d2 );
			prev[0] = cur[0];
			prev[1] = cur[1];
			prev[2] = cur[2];
		}
		arcLen[k] = c[S];
		total += arcLen[k];
	}

	out.resize( count + 1 );
	for ( i = 0; i <= count; i++ )
	{
		segSamp_t &s = out[i];
		if ( total < 1e-4f ) { // collapsed line (the tip of a cone)
			s.arc = 0;
			s.f = (float)i / count;
			for ( int a = 0; a < 3; a++ )
				s.xyz[a] = pts[a];
			continue;
		}
		if ( i == 0 || i == count ) {
			s.arc = ( i == 0 ) ? 0 : nArc - 1;
			s.f = ( i == 0 ) ? 0.0f : 1.0f;
			const float *e = pts + ( ( i == 0 ) ? 0 : ( n - 1 ) ) * 3;
			for ( int a = 0; a < 3; a++ )
				s.xyz[a] = e[a];
			continue;
		}
		float target = total * i / count;
		k = 0;
		while ( k < nArc - 1 && target > arcLen[k] )
		{
			target -= arcLen[k];
			k++;
		}
		const float *c = &cum[k * ( S + 1 )];
		int st = 0;
		while ( st < S - 1 && c[st + 1] < target )
			st++;
		float span = c[st + 1] - c[st];
		float u = ( span > 1e-6f ) ? ( target - c[st] ) / span : 0.0f;
		float t = ( st + u ) / S;
		if ( target > arcLen[k] ) {
			target = arcLen[k];
		}
		s.arc = k;
		s.f = ( arcLen[k] > 1e-6f ) ? target / arcLen[k] : 0.0f;
		SegPatch_EvalArc( pts + ( 2 * k ) * 3, pts + ( 2 * k + 1 ) * 3, pts + ( 2 * k + 2 ) * 3, w[k], t, s.xyz );
	}
	return true;
}

// ---- the grid ---------------------------------------------------------------------------------------

inline bool SegPatch_BuildGrid( const patchMesh_t *p, int segH, int segV, segGrid_t &g ){
	const int W = p->width, Hh = p->height;
	if ( W < 3 || Hh < 3 || ( W & 1 ) == 0 || ( Hh & 1 ) == 0 ) {
		return false;
	}
	segH = SegPatch_ClampH( segH, SegPatch_MinH( p ) );
	segV = SegPatch_ClampV( segV );

	const int U = segH + 1, V = segV + 1;
	int u, v, j;
	std::vector<segPoint_t> R( U * Hh ); // every row of the cage, sampled
	std::vector<float> line;
	std::vector<segSamp_t> smp;

	// along the width: one equal length sampling per row
	line.resize( W * 3 );
	for ( j = 0; j < Hh; j++ )
	{
		for ( int i = 0; i < W; i++ )
			for ( int a = 0; a < 3; a++ )
				line[i * 3 + a] = p->ctrl[i][j].xyz[a];
		if ( !SegPatch_SampleLine( &line[0], W, segH, smp ) ) {
			return false;
		}
		for ( u = 0; u < U; u++ )
		{
			segPoint_t &r = R[u * Hh + j];
			for ( int a = 0; a < 3; a++ )
				r.xyz[a] = smp[u].xyz[a];
			// texture coordinates follow the length along the arc
			float f = smp[u].f;
			float b[3] = { ( 1 - f ) * ( 1 - f ), 2 * f * ( 1 - f ), f * f };
			r.st[0] = r.st[1] = 0;
			for ( int a = 0; a < 3; a++ )
			{
				r.st[0] += b[a] * p->ctrl[2 * smp[u].arc + a][j].st[0];
				r.st[1] += b[a] * p->ctrl[2 * smp[u].arc + a][j].st[1];
			}
		}
	}

	// along the height: one equal length sampling per resulting column
	g.U = U;
	g.V = V;
	g.p.resize( U * V );
	line.resize( Hh * 3 );
	for ( u = 0; u < U; u++ )
	{
		for ( j = 0; j < Hh; j++ )
			for ( int a = 0; a < 3; a++ )
				line[j * 3 + a] = R[u * Hh + j].xyz[a];
		if ( !SegPatch_SampleLine( &line[0], Hh, segV, smp ) ) {
			return false;
		}
		for ( v = 0; v < V; v++ )
		{
			segPoint_t &q = g.p[u * V + v];
			for ( int a = 0; a < 3; a++ )
				q.xyz[a] = smp[v].xyz[a];
			float f = smp[v].f;
			float b[3] = { ( 1 - f ) * ( 1 - f ), 2 * f * ( 1 - f ), f * f };
			q.st[0] = q.st[1] = 0;
			for ( int a = 0; a < 3; a++ )
			{
				q.st[0] += b[a] * R[u * Hh + 2 * smp[v].arc + a].st[0];
				q.st[1] += b[a] * R[u * Hh + 2 * smp[v].arc + a].st[1];
			}
		}
	}
	return true;
}

// ---- pieces -----------------------------------------------------------------------------------------

// cuts segs segments into chunks of at most SEGPATCH_PIECE_SEGS, as even as possible
inline void SegPatch_Split( int segs, std::vector<int> &start, std::vector<int> &count ){
	int np = ( segs + SEGPATCH_PIECE_SEGS - 1 ) / SEGPATCH_PIECE_SEGS;
	int base = segs / np, rem = segs % np, at = 0;
	for ( int i = 0; i < np; i++ )
	{
		int c = base + ( i < rem ? 1 : 0 );
		start.push_back( at );
		count.push_back( c );
		at += c;
	}
}

// the cells [u0,u1) x [v0,v1) of the grid as pieces
inline void SegPatch_RectPieces( const segGrid_t &g, int u0, int u1, int v0, int v1, std::vector<segPiece_t> &out ){
	std::vector<int> us, uc, vs, vc;
	SegPatch_Split( u1 - u0, us, uc );
	SegPatch_Split( v1 - v0, vs, vc );
	for ( size_t a = 0; a < us.size(); a++ )
	{
		for ( size_t b = 0; b < vs.size(); b++ )
		{
			segPiece_t pc;
			pc.width = 2 * uc[a] + 1;
			pc.height = 2 * vc[b] + 1;
			pc.c.resize( pc.width * pc.height );
			for ( int x = 0; x < pc.width; x++ )
			{
				for ( int y = 0; y < pc.height; y++ )
				{
					int xi = u0 + us[a] + x / 2, yi = v0 + vs[b] + y / 2;
					int nx = ( x & 1 ) ? 2 : 1, ny = ( y & 1 ) ? 2 : 1;
					segPoint_t &c = pc.c[x * pc.height + y];
					c.xyz[0] = c.xyz[1] = c.xyz[2] = c.st[0] = c.st[1] = 0;
					for ( int dx = 0; dx < nx; dx++ )
						for ( int dy = 0; dy < ny; dy++ )
						{
							const segPoint_t &s = g.p[( xi + dx ) * g.V + ( yi + dy )];
							for ( int k = 0; k < 3; k++ )
								c.xyz[k] += s.xyz[k];
							c.st[0] += s.st[0];
							c.st[1] += s.st[1];
						}
					float inv = 1.0f / ( nx * ny );
					for ( int k = 0; k < 3; k++ )
						c.xyz[k] *= inv;
					c.st[0] *= inv;
					c.st[1] *= inv;
				}
			}
			out.push_back( pc );
		}
	}
}

inline void SegPatch_Pieces( const segGrid_t &g, std::vector<segPiece_t> &out ){
	SegPatch_RectPieces( g, 0, g.U - 1, 0, g.V - 1, out );
}

// ---- seat cut -----------------------------------------------------------------------------------------
//
// A hole is cut into the grid of a segmented patch by the convex hulls of seat cutters (patchMesh_t::seat,
// planes with n.x + d <= 0 inside). It acts on the cells of the grid, which are the surface:
//   - a cell that lies inside a hull is dropped,
//   - a cell that lies outside every hull is kept (the cells of this kind are put together in rectangles),
//   - a cell that crosses a hull is clipped plane by plane; what is outside stays, with the texture
//     coordinates interpolated at the new corners, what is inside is dropped.
// Nothing closes the hole: the cutter fills it.

struct segVert_t
{
	float xyz[3];
	float st[2];
	float cu, cv;       // where in its cell the vertex is (0..1 along the width / height), for the normals
};
typedef std::vector<segVert_t> segPoly_t;

struct segRect_t
{
	int u0, u1, v0, v1;     // cells [u0,u1) x [v0,v1) that are kept as they are
};

struct segFrag_t
{
	int cu, cv;             // the cell the polygon was cut from
	segPoly_t poly;         // convex, in the winding of the cell ( (0,0) (1,0) (1,1) (0,1) )
};

struct segCut_t
{
	std::vector<segRect_t> rects;
	std::vector<segFrag_t> frags;
	int removed;            // cells dropped as a whole
};

inline float SegPatch_PlaneDist( const float *pl, const float *x ){
	return pl[0] * x[0] + pl[1] * x[1] + pl[2] * x[2] + pl[3];
}

// the point where the edge a-b meets the plane; worked out the same way from either end, so that the two
// cells on both sides of an edge get exactly the same point
inline segVert_t SegPatch_EdgeCut( const segVert_t &a, const segVert_t &b, float da, float db ){
	const segVert_t *p = &a, *q = &b;
	float dp = da, dq = db;
	bool swap = false;
	for ( int k = 0; k < 3; k++ )
	{
		if ( a.xyz[k] != b.xyz[k] ) {
			swap = a.xyz[k] > b.xyz[k];
			break;
		}
	}
	if ( swap ) {
		p = &b;
		q = &a;
		dp = db;
		dq = da;
	}
	float t = dp / ( dp - dq );
	segVert_t v;
	for ( int k = 0; k < 3; k++ )
		v.xyz[k] = p->xyz[k] + t * ( q->xyz[k] - p->xyz[k] );
	for ( int k = 0; k < 2; k++ )
		v.st[k] = p->st[k] + t * ( q->st[k] - p->st[k] );
	v.cu = p->cu + t * ( q->cu - p->cu );
	v.cv = p->cv + t * ( q->cv - p->cv );
	return v;
}

inline void SegPatch_Tidy( segPoly_t &poly ){
	segPoly_t t;
	for ( size_t i = 0; i < poly.size(); i++ )
	{
		const segVert_t &v = poly[i];
		if ( !t.empty() ) {
			const float *w = t.back().xyz;
			if ( fabsf( v.xyz[0] - w[0] ) + fabsf( v.xyz[1] - w[1] ) + fabsf( v.xyz[2] - w[2] ) < 1e-3f ) {
				continue;
			}
		}
		t.push_back( v );
	}
	while ( t.size() > 1 )
	{
		const float *a = t.front().xyz, *b = t.back().xyz;
		if ( fabsf( a[0] - b[0] ) + fabsf( a[1] - b[1] ) + fabsf( a[2] - b[2] ) < 1e-3f ) {
			t.pop_back();
		}
		else{
			break;
		}
	}
	poly.swap( t );
}

inline float SegPatch_PolyArea( const segPoly_t &poly ){
	float s[3] = { 0, 0, 0 };
	for ( size_t i = 1; i + 1 < poly.size(); i++ )
	{
		float a[3], b[3];
		for ( int k = 0; k < 3; k++ )
		{
			a[k] = poly[i].xyz[k] - poly[0].xyz[k];
			b[k] = poly[i + 1].xyz[k] - poly[0].xyz[k];
		}
		s[0] += a[1] * b[2] - a[2] * b[1];
		s[1] += a[2] * b[0] - a[0] * b[2];
		s[2] += a[0] * b[1] - a[1] * b[0];
	}
	return 0.5f * sqrtf( s[0] * s[0] + s[1] * s[1] + s[2] * s[2] );
}

// splits a polygon at a plane: what is in front of it (outside) and what is behind it (inside)
inline void SegPatch_SplitPoly( const segPoly_t &poly, const float *pl, segPoly_t &outside, segPoly_t &inside ){
	const size_t n = poly.size();
	std::vector<float> d( n );
	for ( size_t i = 0; i < n; i++ )
		d[i] = SegPatch_PlaneDist( pl, poly[i].xyz );
	for ( size_t i = 0; i < n; i++ )
	{
		const size_t j = ( i + 1 ) % n;
		if ( d[i] > 0 ) {
			outside.push_back( poly[i] );
		}
		else{
			inside.push_back( poly[i] );
		}
		if ( ( d[i] > 0 ) != ( d[j] > 0 ) ) {
			segVert_t v = SegPatch_EdgeCut( poly[i], poly[j], d[i], d[j] );
			outside.push_back( v );
			inside.push_back( v );
		}
	}
	SegPatch_Tidy( outside );
	SegPatch_Tidy( inside );
}

// the part of a polygon that is outside the hull, as convex polygons added to out
inline void SegPatch_ClipOutsideHull( const segPoly_t &poly, const seatHull_t &h, std::vector<segPoly_t> &out ){
	segPoly_t rest = poly;
	for ( int k = 0; k < h.n && rest.size() >= 3; k++ )
	{
		segPoly_t o, in;
		SegPatch_SplitPoly( rest, h.pl[k], o, in );
		if ( o.size() >= 3 && SegPatch_PolyArea( o ) > 1e-3f ) {
			out.push_back( o );
		}
		rest.swap( in );
	}
}

// Where a point lies on the outline of a polygon: s = edge index + fraction along that edge. False if it is
// further than tol from the outline.
inline bool SegPatch_Locate( const segPoly_t &A, const float *x, float tol, float *s ){
	const int n = (int)A.size();
	float best = 1e30f, bt = 0;
	int be = 0;
	for ( int e = 0; e < n; e++ )
	{
		const float *a = A[e].xyz, *b = A[( e + 1 ) % n].xyz;
		float ab[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
		float l2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
		float t = 0;
		if ( l2 > 1e-12f ) {
			t = ( ( x[0] - a[0] ) * ab[0] + ( x[1] - a[1] ) * ab[1] + ( x[2] - a[2] ) * ab[2] ) / l2;
			t = t < 0 ? 0 : ( t > 1 ? 1 : t );
		}
		float p[3] = { a[0] + t * ab[0] - x[0], a[1] + t * ab[1] - x[1], a[2] + t * ab[2] - x[2] };
		float d2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
		if ( d2 < best ) {
			best = d2;
			be = e;
			bt = t;
		}
	}
	if ( sqrtf( best ) > tol ) {
		return false;
	}
	*s = be + bt;
	return true;
}

// Newell normal of a polygon (not unit length)
inline void SegPatch_PolyNormal( const segPoly_t &p, float *N ){
	N[0] = N[1] = N[2] = 0;
	const size_t n = p.size();
	for ( size_t i = 0; i < n; i++ )
	{
		const float *c = p[i].xyz, *d = p[( i + 1 ) % n].xyz;
		N[0] += ( c[1] - d[1] ) * ( c[2] + d[2] );
		N[1] += ( c[2] - d[2] ) * ( c[0] + d[0] );
		N[2] += ( c[0] - d[0] ) * ( c[1] + d[1] );
	}
}

// the turn at b of a-b-c along the normal N (positive: convex)
inline float SegPatch_Turn( const float *a, const float *b, const float *c, const float *N ){
	float e1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] }, e2[3] = { c[0] - b[0], c[1] - b[1], c[2] - b[2] };
	float x[3] = { e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0] };
	return x[0] * N[0] + x[1] * N[1] + x[2] * N[2];
}

// A simple polygon as the fewest triangles (a convex quad stays one piece). Among the ears that can be cut off the
// one with the best shaped triangle goes first, so the result follows the shape and not the order of the vertices.
inline void SegPatch_EmitPoly( const segPoly_t &in, std::vector<segPoly_t> &out ){
	segPoly_t poly = in;
	SegPatch_Tidy( poly );
	if ( poly.size() < 3 || SegPatch_PolyArea( poly ) < 1e-3f ) {
		return;
	}
	if ( poly.size() == 3 ) {
		out.push_back( poly );
		return;
	}
	float N[3];
	SegPatch_PolyNormal( poly, N );
	const float nl = sqrtf( N[0] * N[0] + N[1] * N[1] + N[2] * N[2] );
	if ( nl < 1e-9f ) {
		out.push_back( poly );
		return;
	}
	for ( int k = 0; k < 3; k++ )
		N[k] /= nl;
	std::vector<int> idx;
	bool convex = true;
	const int n0 = (int)poly.size();
	for ( int i = 0; i < n0; i++ )
	{
		idx.push_back( i );
		if ( SegPatch_Turn( poly[( i + n0 - 1 ) % n0].xyz, poly[i].xyz, poly[( i + 1 ) % n0].xyz, N ) < -1e-4f ) {
			convex = false;
		}
	}
	if ( convex && n0 == 4 ) {
		out.push_back( poly );
		return;
	}
	std::vector<int> tris; // triangles as three indices into poly
	while ( idx.size() > 3 )
	{
		const int n = (int)idx.size();
		int bestI = -1;
		float bestQ = -1;
		for ( int i = 0; i < n; i++ )
		{
			const float *a = poly[idx[( i + n - 1 ) % n]].xyz, *b = poly[idx[i]].xyz, *c = poly[idx[( i + 1 ) % n]].xyz;
			float turn = SegPatch_Turn( a, b, c, N );
			if ( turn <= 1e-4f ) {
				continue;
			}
			bool clear = true;
			for ( int j = 0; j < n && clear; j++ )
			{
				if ( j == i || j == ( i + n - 1 ) % n || j == ( i + 1 ) % n ) {
					continue;
				}
				const float *p = poly[idx[j]].xyz;
				if ( SegPatch_Turn( a, b, p, N ) > -1e-4f && SegPatch_Turn( b, c, p, N ) > -1e-4f && SegPatch_Turn( c, a, p, N ) > -1e-4f ) {
					clear = false; // a vertex inside the ear
				}
			}
			if ( !clear ) {
				continue;
			}
			float la = 0, lb = 0, lc = 0;
			for ( int k = 0; k < 3; k++ )
			{
				la += ( b[k] - a[k] ) * ( b[k] - a[k] );
				lb += ( c[k] - b[k] ) * ( c[k] - b[k] );
				lc += ( a[k] - c[k] ) * ( a[k] - c[k] );
			}
			float q = 2.0f * turn / ( la + lb + lc + 1e-9f ); // 4 sqrt(3) area / sum of squares, up to a constant
			if ( q > bestQ + 1e-6f ) {
				bestQ = q;
				bestI = i;
			}
		}
		if ( bestI < 0 ) { // not expected; fall back to a fan
			for ( size_t i = 1; i + 1 < idx.size(); i++ )
			{
				tris.push_back( idx[0] );
				tris.push_back( idx[i] );
				tris.push_back( idx[i + 1] );
			}
			idx.clear();
			break;
		}
		tris.push_back( idx[( bestI + n - 1 ) % n] );
		tris.push_back( idx[bestI] );
		tris.push_back( idx[( bestI + 1 ) % n] );
		idx.erase( idx.begin() + bestI );
	}
	if ( idx.size() == 3 ) {
		tris.push_back( idx[0] );
		tris.push_back( idx[1] );
		tris.push_back( idx[2] );
	}

	// pairs of triangles that make a convex quad become one piece (fewer patches, the same surface); the
	// best shaped quads first
	const int nt = (int)tris.size() / 3;
	struct Cand
	{
		float score;
		int i, j, q[4];
	};
	std::vector<Cand> cands;
	for ( int i = 0; i < nt; i++ )
	{
		for ( int j = i + 1; j < nt; j++ )
		{
			for ( int e = 0; e < 3; e++ )
			{
				const int u = tris[i * 3 + e], v = tris[i * 3 + ( e + 1 ) % 3], w = tris[i * 3 + ( e + 2 ) % 3];
				for ( int f = 0; f < 3; f++ )
				{
					if ( tris[j * 3 + f] != v || tris[j * 3 + ( f + 1 ) % 3] != u ) {
						continue;
					}
					const int x = tris[j * 3 + ( f + 2 ) % 3];
					Cand c;
					c.i = i;
					c.j = j;
					c.q[0] = w;
					c.q[1] = u;
					c.q[2] = x;
					c.q[3] = v;
					float worst = 1e9f;
					for ( int k = 0; k < 4; k++ )
					{
						const float *a = poly[c.q[( k + 3 ) % 4]].xyz, *b = poly[c.q[k]].xyz, *d = poly[c.q[( k + 1 ) % 4]].xyz;
						float la = 0, lb = 0;
						for ( int m = 0; m < 3; m++ )
						{
							la += ( b[m] - a[m] ) * ( b[m] - a[m] );
							lb += ( d[m] - b[m] ) * ( d[m] - b[m] );
						}
						float s = SegPatch_Turn( a, b, d, N ) / ( sqrtf( la * lb ) + 1e-9f ); // sine of the corner
						if ( s < worst ) {
							worst = s;
						}
					}
					if ( worst > 0.05f ) {
						c.score = worst;
						cands.push_back( c );
					}
				}
			}
		}
	}
	std::vector<char> used( nt, 0 );
	for ( size_t n = 0; n < cands.size(); n++ )
	{
		size_t best = cands.size();
		for ( size_t m = 0; m < cands.size(); m++ )
			if ( !used[cands[m].i] && !used[cands[m].j] && ( best == cands.size() || cands[m].score > cands[best].score + 1e-6f ) ) {
				best = m;
			}
		if ( best == cands.size() ) {
			break;
		}
		const Cand &c = cands[best];
		used[c.i] = used[c.j] = 1;
		segPoly_t q;
		for ( int k = 0; k < 4; k++ )
			q.push_back( poly[c.q[k]] );
		out.push_back( q );
	}
	for ( int i = 0; i < nt; i++ )
	{
		if ( used[i] ) {
			continue;
		}
		segPoly_t t;
		for ( int k = 0; k < 3; k++ )
			t.push_back( poly[tris[i * 3 + k]] );
		out.push_back( t );
	}
}

// A convex polygon minus a hull. Returns 0 if the hull does not touch it (nothing added to out), 1 if it is
// all inside, 2 if what is left of it was added to out (triangles and convex quads). The part inside is the
// polygon cut by every plane, which does not depend on their order; what is left is read off the outline of that
// part and of the polygon, so both sides of a symmetric cut come out alike.
inline int SegPatch_SubtractHull( const segPoly_t &A, const seatHull_t &h, std::vector<segPoly_t> &out ){
	segPoly_t K = A;
	for ( int k = 0; k < h.n && K.size() >= 3; k++ )
	{
		segPoly_t o, in;
		SegPatch_SplitPoly( K, h.pl[k], o, in );
		K.swap( in );
	}
	if ( K.size() < 3 ) {
		return 0;
	}
	const float aK = SegPatch_PolyArea( K ), aA = SegPatch_PolyArea( A );
	if ( aK < 1e-3f ) {
		return 0;
	}
	if ( aK >= aA * 0.99999f - 1e-3f ) {
		return 1;
	}

	const int na = (int)A.size(), nk = (int)K.size();
	const float tol = 0.02f;
	std::vector<char> onA( nk ), cut( nk );
	std::vector<float> pos( nk, 0.0f );
	for ( int i = 0; i < nk; i++ )
		onA[i] = SegPatch_Locate( A, K[i].xyz, tol, &pos[i] ) ? 1 : 0;
	for ( int i = 0; i < nk; i++ )
	{
		const int j = ( i + 1 ) % nk;
		bool along = false;
		if ( onA[i] && onA[j] ) {
			float mid[3], s;
			for ( int k = 0; k < 3; k++ )
				mid[k] = 0.5f * ( K[i].xyz[k] + K[j].xyz[k] );
			along = SegPatch_Locate( A, mid, tol, &s );
		}
		cut[i] = along ? 0 : 1;
	}

	std::vector<segPoly_t> kept;
	bool ok = true;
	int chains = 0;
	for ( int i = 0; i < nk && ok; i++ )
	{
		if ( !cut[i] || cut[( i + nk - 1 ) % nk] ) {
			continue; // not the start of a run of cut edges
		}
		int j = i;
		while ( cut[j % nk] && j - i <= nk )
			j++;
		const int e = j % nk;
		if ( !onA[i] || !onA[e] || j - i > nk ) {
			ok = false;
			break;
		}
		chains++;
		// the kept part on this side: the run backwards (from where it ends to where it starts), then the outline of
		// the polygon on from the start point round to the end point
		segPoly_t poly;
		for ( int m = j; m >= i; m-- )
			poly.push_back( K[m % nk] );
		const float sP = pos[i], sQ = pos[e];
		float dq = sQ - sP;
		if ( dq <= 1e-4f ) {
			dq += na;
		}
		std::vector<std::pair<float, int> > arc;
		for ( int k = 0; k < na; k++ )
		{
			float d = k - sP;
			while ( d <= 1e-4f )
				d += na;
			if ( d < dq - 1e-4f ) {
				arc.push_back( std::make_pair( d, k ) );
			}
		}
		std::sort( arc.begin(), arc.end() );
		for ( size_t m = 0; m < arc.size(); m++ )
			poly.push_back( A[arc[m].second] );
		SegPatch_Tidy( poly );
		kept.push_back( poly );
	}
	if ( ok && chains > 0 ) {
		float sum = aK;
		for ( size_t i = 0; i < kept.size(); i++ )
			sum += SegPatch_PolyArea( kept[i] );
		if ( fabsf( sum - aA ) > 0.01f * aA + 0.01f ) {
			ok = false; // the outline was not read right: do it the long way
		}
	}
	if ( ok && chains > 0 ) {
		for ( size_t i = 0; i < kept.size(); i++ )
			SegPatch_EmitPoly( kept[i], out );
		return 2;
	}
	// the hull is inside the polygon without touching its outline, or something odd: plane by plane
	SegPatch_ClipOutsideHull( A, h, out );
	return 2;
}

inline void SegPatch_Cut( const segGrid_t &g, const seatHull_t *hulls, int nHulls, segCut_t &r ){
	r.rects.clear();
	r.frags.clear();
	r.removed = 0;
	const int CU = g.U - 1, CV = g.V - 1;
	if ( CU < 1 || CV < 1 ) {
		return;
	}
	const float eps = 0.001f;
	std::vector<char> keep( CU * CV, 0 ), used( CU * CV, 0 );
	const float cuv[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };

	for ( int u = 0; u < CU; u++ )
	{
		for ( int v = 0; v < CV; v++ )
		{
			const segPoint_t *c[4] = { &g.p[u * g.V + v], &g.p[( u + 1 ) * g.V + v], &g.p[( u + 1 ) * g.V + v + 1], &g.p[u * g.V + v + 1] };
			bool apart = true, gone = false;
			for ( int h = 0; h < nHulls && !gone; h++ )
			{
				bool sep = false, inside = true;
				for ( int k = 0; k < hulls[h].n; k++ )
				{
					int outCount = 0, inCount = 0;
					for ( int q = 0; q < 4; q++ )
					{
						float d = SegPatch_PlaneDist( hulls[h].pl[k], c[q]->xyz );
						if ( d > eps ) {
							outCount++;
						}
						if ( d <= eps ) {
							inCount++;
						}
					}
					if ( outCount == 4 ) {
						sep = true;
						break;
					}
					if ( inCount < 4 ) {
						inside = false;
					}
				}
				if ( !sep ) {
					apart = false;
					if ( inside && hulls[h].n > 0 ) {
						gone = true;
					}
				}
			}
			if ( gone ) {
				r.removed++;
				continue;
			}
			if ( apart ) {
				keep[u * CV + v] = 1;
				continue;
			}

			segPoly_t cell( 4 );
			for ( int q = 0; q < 4; q++ )
			{
				for ( int k = 0; k < 3; k++ )
					cell[q].xyz[k] = c[q]->xyz[k];
				cell[q].st[0] = c[q]->st[0];
				cell[q].st[1] = c[q]->st[1];
				cell[q].cu = cuv[q][0];
				cell[q].cv = cuv[q][1];
			}
			std::vector<segPoly_t> pieces( 1, cell );
			bool changed = false;
			for ( int h = 0; h < nHulls; h++ )
			{
				std::vector<segPoly_t> next;
				for ( size_t i = 0; i < pieces.size(); i++ )
				{
					std::vector<segPoly_t> res;
					const int st = SegPatch_SubtractHull( pieces[i], hulls[h], res );
					if ( st == 0 ) {
						next.push_back( pieces[i] ); // the hull does not reach it
					}
					else
					{
						changed = true;
						next.insert( next.end(), res.begin(), res.end() );
					}
				}
				pieces.swap( next );
			}
			if ( !changed ) {
				keep[u * CV + v] = 1; // near a hull, but not in it
				continue;
			}
			if ( pieces.empty() ) {
				r.removed++;
				continue;
			}
			for ( size_t i = 0; i < pieces.size(); i++ )
			{
				segFrag_t f;
				f.cu = u;
				f.cv = v;
				f.poly = pieces[i];
				r.frags.push_back( f );
			}
		}
	}

	// the cells that are kept, in as few rectangles as possible
	for ( int v = 0; v < CV; v++ )
	{
		for ( int u = 0; u < CU; u++ )
		{
			if ( !keep[u * CV + v] || used[u * CV + v] ) {
				continue;
			}
			int u2 = u;
			while ( u2 < CU && keep[u2 * CV + v] && !used[u2 * CV + v] )
				u2++;
			int v2 = v + 1;
			for ( ; v2 < CV; v2++ )
			{
				bool full = true;
				for ( int x = u; x < u2 && full; x++ )
					if ( !keep[x * CV + v2] || used[x * CV + v2] ) {
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
			r.rects.push_back( rc );
		}
	}
}

// a flat piece with the corners c00 c10 c11 c01 (the quad of a cell): the points in between are halfway, as
// in the pieces of the grid. Two equal corners make a triangle.
inline void SegPatch_QuadPiece( const segVert_t *c00, const segVert_t *c10, const segVert_t *c11, const segVert_t *c01, segPiece_t &pc ){
	pc.width = pc.height = 3;
	pc.c.resize( 9 );
	for ( int x = 0; x < 3; x++ )
	{
		for ( int y = 0; y < 3; y++ )
		{
			const float s = x * 0.5f, t = y * 0.5f;
			const float w00 = ( 1 - s ) * ( 1 - t ), w10 = s * ( 1 - t ), w11 = s * t, w01 = ( 1 - s ) * t;
			segPoint_t &o = pc.c[x * 3 + y];
			for ( int k = 0; k < 3; k++ )
				o.xyz[k] = w00 * c00->xyz[k] + w10 * c10->xyz[k] + w11 * c11->xyz[k] + w01 * c01->xyz[k];
			for ( int k = 0; k < 2; k++ )
				o.st[k] = w00 * c00->st[k] + w10 * c10->st[k] + w11 * c11->st[k] + w01 * c01->st[k];
		}
	}
}

inline void SegPatch_FragPieces( const segFrag_t &f, std::vector<segPiece_t> &out ){
	const segPoly_t &p = f.poly;
	segPiece_t pc;
	if ( p.size() == 4 ) {
		SegPatch_QuadPiece( &p[0], &p[1], &p[2], &p[3], pc );
		out.push_back( pc );
		return;
	}
	for ( size_t i = 1; i + 1 < p.size(); i++ )
	{
		SegPatch_QuadPiece( &p[0], &p[i], &p[i + 1], &p[i + 1], pc );
		out.push_back( pc );
	}
}

// the pieces a segmented patch is saved as: the grid, with the holes of its seat hulls cut out
inline void SegPatch_BuildPieces( const patchMesh_t *p, const segGrid_t &g, std::vector<segPiece_t> &out ){
	if ( p->seatCount <= 0 ) {
		SegPatch_Pieces( g, out );
		return;
	}
	segCut_t r;
	SegPatch_Cut( g, p->seat, p->seatCount, r );
	for ( size_t i = 0; i < r.rects.size(); i++ )
		SegPatch_RectPieces( g, r.rects[i].u0, r.rects[i].u1, r.rects[i].v0, r.rects[i].v1, out );
	for ( size_t i = 0; i < r.frags.size(); i++ )
		SegPatch_FragPieces( r.frags[i], out );
}

#endif // __SEGPATCH_H__
