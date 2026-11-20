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

inline void SegPatch_Pieces( const segGrid_t &g, std::vector<segPiece_t> &out ){
	std::vector<int> us, uc, vs, vc;
	SegPatch_Split( g.U - 1, us, uc );
	SegPatch_Split( g.V - 1, vs, vc );
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
					int xi = us[a] + x / 2, yi = vs[b] + y / 2;
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

#endif // __SEGPATCH_H__
