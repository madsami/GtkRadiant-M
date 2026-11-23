/*
   Entanglement: copies that follow the object they were made from (see entangle.h).
 */
#include "stdafx.h"
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <math.h>
#include <string.h>
#include "segpatch.h"
#include "entangle.h"
#include "straight.h"
#include "glwidget.h"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

const segGrid_t* Patch_GetSegGrid( patchMesh_t *p ); // pmesh.cpp
void Brush_RebuildBrush( brush_t *b, vec3_t vMins, vec3_t vMaxs ); // pmesh.cpp

// Two shapes are the same when no point is farther than this from its partner (the floats of a moved patch are not exact).
static const double EN_TOL = 0.02;
// An instance whose points are mostly farther than this from where the fit puts them is not a copy of the source any more.
static const double EN_FAR = 0.25;

// =============================================================================
// points and fits

struct EnP
{
	double v[3];
};
typedef std::vector<EnP> EnPts;

// x' = s * R * x + t
struct EnXf
{
	double s;
	double R[9];
	double t[3];
};

static void En_Identity( EnXf &x ){
	x.s = 1;
	for ( int i = 0; i < 9; i++ )
		x.R[i] = ( i % 4 == 0 ) ? 1 : 0;
	x.t[0] = x.t[1] = x.t[2] = 0;
}

static EnP En_Apply( const EnXf &x, const EnP &p ){
	EnP o;
	for ( int r = 0; r < 3; r++ )
		o.v[r] = x.s * ( x.R[r * 3] * p.v[0] + x.R[r * 3 + 1] * p.v[1] + x.R[r * 3 + 2] * p.v[2] ) + x.t[r];
	return o;
}

static double En_Dist( const EnP &a, const EnP &b ){
	const double dx = a.v[0] - b.v[0], dy = a.v[1] - b.v[1], dz = a.v[2] - b.v[2];
	return sqrt( dx * dx + dy * dy + dz * dz );
}

// the eigenvector of the largest eigenvalue of a symmetric 4x4 matrix (Jacobi rotations)
static void En_Eigen4( double a[4][4], double vec[4] ){
	double v[4][4];
	for ( int i = 0; i < 4; i++ )
		for ( int j = 0; j < 4; j++ )
			v[i][j] = ( i == j ) ? 1 : 0;
	for ( int sweep = 0; sweep < 60; sweep++ )
	{
		double off = 0;
		for ( int p = 0; p < 3; p++ )
			for ( int q = p + 1; q < 4; q++ )
				off += a[p][q] * a[p][q];
		if ( off < 1e-24 ) {
			break;
		}
		for ( int p = 0; p < 3; p++ )
			for ( int q = p + 1; q < 4; q++ )
			{
				if ( fabs( a[p][q] ) < 1e-300 ) {
					continue;
				}
				const double theta = ( a[q][q] - a[p][p] ) / ( 2 * a[p][q] );
				const double t = ( theta >= 0 ? 1.0 : -1.0 ) / ( fabs( theta ) + sqrt( theta * theta + 1 ) );
				const double c = 1 / sqrt( t * t + 1 ), s = t * c;
				for ( int k = 0; k < 4; k++ )
				{
					const double akp = a[k][p], akq = a[k][q];
					a[k][p] = c * akp - s * akq;
					a[k][q] = s * akp + c * akq;
				}
				for ( int k = 0; k < 4; k++ )
				{
					const double apk = a[p][k], aqk = a[q][k];
					a[p][k] = c * apk - s * aqk;
					a[q][k] = s * apk + c * aqk;
				}
				for ( int k = 0; k < 4; k++ )
				{
					const double vkp = v[k][p], vkq = v[k][q];
					v[k][p] = c * vkp - s * vkq;
					v[k][q] = s * vkp + c * vkq;
				}
			}
	}
	int best = 0;
	for ( int i = 1; i < 4; i++ )
		if ( a[i][i] > a[best][best] ) {
			best = i;
		}
	for ( int i = 0; i < 4; i++ )
		vec[i] = v[i][best];
}

// Least squares fit of f onto t (f[i] and t[i] are partners; use[i] == 0 leaves a pair out): a turn, a shift, and
// with scale also one scale factor (Horn's method).
static bool En_Fit( const EnPts &f, const EnPts &t, const std::vector<char> *use, bool scale, EnXf &x ){
	const size_t n = f.size();
	if ( n == 0 || t.size() != n ) {
		return false;
	}
	double cf[3] = { 0, 0, 0 }, ct[3] = { 0, 0, 0 };
	size_t m = 0;
	for ( size_t i = 0; i < n; i++ )
	{
		if ( use && !( *use )[i] ) {
			continue;
		}
		for ( int k = 0; k < 3; k++ )
		{
			cf[k] += f[i].v[k];
			ct[k] += t[i].v[k];
		}
		m++;
	}
	if ( m == 0 ) {
		return false;
	}
	for ( int k = 0; k < 3; k++ )
	{
		cf[k] /= m;
		ct[k] /= m;
	}
	double S[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	double sf = 0, st = 0;
	for ( size_t i = 0; i < n; i++ )
	{
		if ( use && !( *use )[i] ) {
			continue;
		}
		double a[3], b[3];
		for ( int k = 0; k < 3; k++ )
		{
			a[k] = f[i].v[k] - cf[k];
			b[k] = t[i].v[k] - ct[k];
			sf += a[k] * a[k];
			st += b[k] * b[k];
		}
		for ( int r = 0; r < 3; r++ )
			for ( int c = 0; c < 3; c++ )
				S[r][c] += a[r] * b[c];
	}
	En_Identity( x );
	if ( m >= 2 && sf > 1e-12 && st > 1e-12 ) {
		double N[4][4] = {
			{ S[0][0] + S[1][1] + S[2][2], S[1][2] - S[2][1], S[2][0] - S[0][2], S[0][1] - S[1][0] },
			{ S[1][2] - S[2][1], S[0][0] - S[1][1] - S[2][2], S[0][1] + S[1][0], S[2][0] + S[0][2] },
			{ S[2][0] - S[0][2], S[0][1] + S[1][0], -S[0][0] + S[1][1] - S[2][2], S[1][2] + S[2][1] },
			{ S[0][1] - S[1][0], S[2][0] + S[0][2], S[1][2] + S[2][1], -S[0][0] - S[1][1] + S[2][2] }
		};
		double q[4];
		En_Eigen4( N, q );
		const double len = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
		if ( len > 1e-12 ) {
			const double w = q[0] / len, qx = q[1] / len, qy = q[2] / len, qz = q[3] / len;
			x.R[0] = 1 - 2 * ( qy * qy + qz * qz );
			x.R[1] = 2 * ( qx * qy - qz * w );
			x.R[2] = 2 * ( qx * qz + qy * w );
			x.R[3] = 2 * ( qx * qy + qz * w );
			x.R[4] = 1 - 2 * ( qx * qx + qz * qz );
			x.R[5] = 2 * ( qy * qz - qx * w );
			x.R[6] = 2 * ( qx * qz - qy * w );
			x.R[7] = 2 * ( qy * qz + qx * w );
			x.R[8] = 1 - 2 * ( qx * qx + qy * qy );
		}
		if ( scale ) {
			x.s = sqrt( st / sf );
		}
	}
	for ( int r = 0; r < 3; r++ )
		x.t[r] = ct[r] - x.s * ( x.R[r * 3] * cf[0] + x.R[r * 3 + 1] * cf[1] + x.R[r * 3 + 2] * cf[2] );
	return true;
}

// The same, but the points that do not belong (edited vertices) are left out: the fit is repeated on the points that
// the last fit got right. res[i] is the distance of every point from where the final fit puts it.
static bool En_FitRobust( const EnPts &f, const EnPts &t, bool scale, EnXf &x, std::vector<double> &res ){
	const size_t n = f.size();
	std::vector<char> use( n, 1 );
	for ( int iter = 0; iter < 20; iter++ )
	{
		if ( !En_Fit( f, t, &use, scale, x ) ) {
			return false;
		}
		res.resize( n );
		for ( size_t i = 0; i < n; i++ )
			res[i] = En_Dist( En_Apply( x, f[i] ), t[i] );
		std::vector<double> sorted( res );
		std::sort( sorted.begin(), sorted.end() );
		const double med = sorted[n / 2];
		const double thr = std::max( EN_TOL, 2.5 * med );
		std::vector<char> nu( n );
		size_t cnt = 0;
		for ( size_t i = 0; i < n; i++ )
		{
			nu[i] = res[i] <= thr ? 1 : 0;
			cnt += nu[i];
		}
		if ( ( cnt < 3 && n >= 3 ) || nu == use ) {
			break;
		}
		use = nu;
	}
	return true;
}

// How many pairs does x bring (almost) together?
static size_t En_CountFit( const EnPts &f, const EnPts &t, const EnXf &x, double tol, std::vector<char> *mask ){
	size_t n = 0;
	if ( mask ) {
		mask->assign( f.size(), 0 );
	}
	for ( size_t i = 0; i < f.size(); i++ )
		if ( En_Dist( En_Apply( x, f[i] ), t[i] ) <= tol ) {
			n++;
			if ( mask ) {
				( *mask )[i] = 1;
			}
		}
	return n;
}

// Which part stayed where it was? When half of a cylinder was pulled out, the other half is the part that kept its place, and
// the fit must hold on to that part (an average over all points would move everything a little). So: try the shifts that
// bring one point onto its partner, and 400 turns worked out from three pairs, take the one that brings the most pairs
// together, and fit on those. If no transform holds for a fifth of the points (say the whole object was scaled), the
// trimmed average fit is used.
static bool En_FitConsensus( const EnPts &f, const EnPts &t, bool scale, EnXf &x, std::vector<double> &res ){
	const size_t n = f.size();
	if ( n < 4 || t.size() != n ) {
		return En_FitRobust( f, t, scale, x, res );
	}
	const double tol = 0.05;
	EnXf best;
	En_Identity( best );
	size_t bestN = En_CountFit( f, t, best, tol, NULL );

	const size_t step = n > 400 ? n / 400 : 1;
	for ( size_t i = 0; i < n; i += step )
	{
		EnXf c;
		En_Identity( c );
		for ( int k = 0; k < 3; k++ )
			c.t[k] = t[i].v[k] - f[i].v[k];
		const size_t m = En_CountFit( f, t, c, tol, NULL );
		if ( m > bestN ) {
			bestN = m;
			best = c;
		}
	}
	unsigned int rng = 12345u;
	for ( int k = 0; k < 400; k++ )
	{
		size_t idx[3];
		for ( int j = 0; j < 3; j++ )
		{
			rng = rng * 1664525u + 1013904223u;
			idx[j] = ( rng >> 8 ) % n;
		}
		if ( idx[0] == idx[1] || idx[0] == idx[2] || idx[1] == idx[2] ) {
			continue;
		}
		EnPts fs, ts;
		for ( int j = 0; j < 3; j++ )
		{
			fs.push_back( f[idx[j]] );
			ts.push_back( t[idx[j]] );
		}
		// three points on a line do not fix a turn
		double a[3], b[3];
		for ( int c = 0; c < 3; c++ )
		{
			a[c] = fs[1].v[c] - fs[0].v[c];
			b[c] = fs[2].v[c] - fs[0].v[c];
		}
		const double cr[3] = { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
		if ( cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2] < 1.0 ) {
			continue;
		}
		EnXf c;
		if ( !En_Fit( fs, ts, NULL, scale, c ) ) {
			continue;
		}
		const size_t m = En_CountFit( f, t, c, tol, NULL );
		if ( m > bestN ) {
			bestN = m;
			best = c;
		}
	}
	if ( bestN < 3 || bestN * 5 < n ) {
		return En_FitRobust( f, t, scale, x, res );
	}
	// fit on everything that agrees with the winner, a few rounds
	x = best;
	for ( int round = 0; round < 4; round++ )
	{
		std::vector<char> mask;
		En_CountFit( f, t, x, tol, &mask );
		EnXf nx;
		if ( !En_Fit( f, t, &mask, scale, nx ) ) {
			break;
		}
		x = nx;
	}
	res.resize( n );
	for ( size_t i = 0; i < n; i++ )
		res[i] = En_Dist( En_Apply( x, f[i] ), t[i] );
	return true;
}

// =============================================================================
// the links in the map

struct EnLink
{
	std::map<int, patchMesh_t*> src;                    // idx -> patch
	std::map<int, std::map<int, patchMesh_t*> > inst;   // inst -> idx -> patch
};

static bool En_IsLinked( const patchMesh_t *p ){
	return p->lnkRole != 0 && p->lnkId > 0;
}

static std::vector<brush_t*> En_AllPatchBrushes(){
	std::vector<brush_t*> v;
	for ( int list = 0; list < 2; list++ )
	{
		brush_t *head = list ? &active_brushes : &selected_brushes;
		for ( brush_t *b = head->next; b != head; b = b->next )
			if ( b->patchBrush && b->pPatch ) {
				v.push_back( b );
			}
	}
	return v;
}

static void En_Gather( std::map<int, EnLink> &links ){
	std::vector<brush_t*> all = En_AllPatchBrushes();
	for ( size_t i = 0; i < all.size(); i++ )
	{
		patchMesh_t *p = all[i]->pPatch;
		if ( !En_IsLinked( p ) ) {
			continue;
		}
		if ( p->lnkRole == 1 ) {
			links[p->lnkId].src.insert( std::make_pair( p->lnkIdx, p ) );
		}
		else{
			links[p->lnkId].inst[p->lnkInst].insert( std::make_pair( p->lnkIdx, p ) );
		}
	}
}

static int En_TopId(){
	int top = 0;
	std::vector<brush_t*> all = En_AllPatchBrushes();
	for ( size_t i = 0; i < all.size(); i++ )
		top = std::max( top, all[i]->pPatch->lnkId );
	return top;
}

// the shape of a source or an instance: its patches in the order of their numbers
struct EnShape
{
	std::vector<int> idx;
	std::vector<int> dims; // width, height, segH, segV of every patch
	EnPts pts;
};

static void En_Flatten( const std::map<int, patchMesh_t*> &m, EnShape &s ){
	for ( std::map<int, patchMesh_t*>::const_iterator it = m.begin(); it != m.end(); ++it )
	{
		const patchMesh_t *p = it->second;
		s.idx.push_back( it->first );
		s.dims.push_back( p->width );
		s.dims.push_back( p->height );
		s.dims.push_back( p->segH );
		s.dims.push_back( p->segV );
		for ( int i = 0; i < p->width; i++ )
			for ( int j = 0; j < p->height; j++ )
			{
				EnP q;
				for ( int k = 0; k < 3; k++ )
					q.v[k] = p->ctrl[i][j].xyz[k];
				s.pts.push_back( q );
			}
	}
}

static void En_Centroid( const EnPts &pts, double c[3] ){
	c[0] = c[1] = c[2] = 0;
	for ( size_t i = 0; i < pts.size(); i++ )
		for ( int k = 0; k < 3; k++ )
			c[k] += pts[i].v[k];
	if ( !pts.empty() ) {
		for ( int k = 0; k < 3; k++ )
			c[k] /= pts.size();
	}
}

// what was the shape of every source when its instances were last brought to it
static std::map<int, EnShape> g_enSnap;
static std::set<int> g_enWarned;

void Entangle_Reset(){
	g_enSnap.clear();
	g_enWarned.clear();
}

static void En_RebuildBounds( patchMesh_t *p ){
	p->bDirty = true;
	vec3_t vMin, vMax;
	Patch_CalcBounds( p, vMin, vMax );
	Brush_RebuildBrush( p->pSymbiot, vMin, vMax );
}

// =============================================================================
// the origin: a frame (position and three axes) that every linked patch carries and that goes along when the whole
// patch is moved, turned or scaled (the hooks at the end of this section), but not when single points are edited. The
// shape of a source is its control points in that frame, so moving the source changes nothing, and neither do the
// points that were left alone when others were edited. Instances place the same shape in their own frame.

struct EnFrame
{
	double o[3];
	double M[9]; // row by row; the columns are the three axes
};

static bool En_GetFrame( const patchMesh_t *p, EnFrame &f ){
	if ( !p->lnkHasF ) {
		return false;
	}
	for ( int k = 0; k < 3; k++ )
		f.o[k] = p->lnkF[k];
	for ( int c = 0; c < 3; c++ )
		for ( int r = 0; r < 3; r++ )
			f.M[r * 3 + c] = p->lnkF[3 + c * 3 + r];
	return true;
}

static void En_PutFrame( patchMesh_t *p, const EnFrame &f ){
	p->lnkHasF = 1;
	for ( int k = 0; k < 3; k++ )
		p->lnkF[k] = (float)f.o[k];
	for ( int c = 0; c < 3; c++ )
		for ( int r = 0; r < 3; r++ )
			p->lnkF[3 + c * 3 + r] = (float)f.M[r * 3 + c];
}

static bool En_SameFrame( const EnFrame &a, const EnFrame &b ){
	for ( int k = 0; k < 3; k++ )
		if ( fabs( a.o[k] - b.o[k] ) > 0.01 ) {
			return false;
		}
	for ( int k = 0; k < 9; k++ )
		if ( fabs( a.M[k] - b.M[k] ) > 0.0005 ) {
			return false;
		}
	return true;
}

// The frame that most patches of a set agree on (when one patch of an object was moved on its own, the others win; on a
// tie the lowest member number). False when no patch has a frame.
static bool En_SetFrame( const std::map<int, patchMesh_t*> &m, EnFrame &out ){
	std::vector<EnFrame> fs;
	for ( std::map<int, patchMesh_t*>::const_iterator it = m.begin(); it != m.end(); ++it )
	{
		EnFrame f;
		if ( En_GetFrame( it->second, f ) ) {
			fs.push_back( f );
		}
	}
	if ( fs.empty() ) {
		return false;
	}
	size_t best = 0, bestCount = 0;
	for ( size_t i = 0; i < fs.size(); i++ )
	{
		size_t n = 0;
		for ( size_t j = 0; j < fs.size(); j++ )
			if ( En_SameFrame( fs[i], fs[j] ) ) {
				n++;
			}
		if ( n > bestCount ) {
			bestCount = n;
			best = i;
		}
	}
	out = fs[best];
	return true;
}

static void En_PutAll( std::map<int, patchMesh_t*> &m, const EnFrame &f ){
	for ( std::map<int, patchMesh_t*>::iterator it = m.begin(); it != m.end(); ++it )
		En_PutFrame( it->second, f );
}

static bool En_Inv3( const double m[9], double inv[9] ){
	const double c0 = m[4] * m[8] - m[5] * m[7];
	const double c1 = m[5] * m[6] - m[3] * m[8];
	const double c2 = m[3] * m[7] - m[4] * m[6];
	const double det = m[0] * c0 + m[1] * c1 + m[2] * c2;
	if ( fabs( det ) < 1e-9 ) {
		return false;
	}
	const double id = 1.0 / det;
	inv[0] = c0 * id;
	inv[1] = ( m[2] * m[7] - m[1] * m[8] ) * id;
	inv[2] = ( m[1] * m[5] - m[2] * m[4] ) * id;
	inv[3] = c1 * id;
	inv[4] = ( m[0] * m[8] - m[2] * m[6] ) * id;
	inv[5] = ( m[2] * m[3] - m[0] * m[5] ) * id;
	inv[6] = c2 * id;
	inv[7] = ( m[1] * m[6] - m[0] * m[7] ) * id;
	inv[8] = ( m[0] * m[4] - m[1] * m[3] ) * id;
	return true;
}

static EnP En_ToLocal( const EnFrame &f, const double inv[9], const EnP &w ){
	const double d[3] = { w.v[0] - f.o[0], w.v[1] - f.o[1], w.v[2] - f.o[2] };
	EnP l;
	for ( int r = 0; r < 3; r++ )
		l.v[r] = inv[r * 3] * d[0] + inv[r * 3 + 1] * d[1] + inv[r * 3 + 2] * d[2];
	return l;
}

static EnP En_ToWorld( const EnFrame &f, const EnP &l ){
	EnP w;
	for ( int r = 0; r < 3; r++ )
		w.v[r] = f.o[r] + f.M[r * 3] * l.v[0] + f.M[r * 3 + 1] * l.v[1] + f.M[r * 3 + 2] * l.v[2];
	return w;
}

// the frame of an object that has none yet: the middle of its bounds, world axes
static void En_NewFrame( const EnPts &pts, EnFrame &f ){
	double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
	for ( size_t i = 0; i < pts.size(); i++ )
		for ( int k = 0; k < 3; k++ )
		{
			lo[k] = std::min( lo[k], pts[i].v[k] );
			hi[k] = std::max( hi[k], pts[i].v[k] );
		}
	for ( int k = 0; k < 3; k++ )
		f.o[k] = pts.empty() ? 0 : ( lo[k] + hi[k] ) * 0.5;
	for ( int k = 0; k < 9; k++ )
		f.M[k] = ( k % 4 == 0 ) ? 1 : 0;
}

// the frame after the object was turned / shifted / scaled by T inside its own frame: F'(l) = F(T(l))
static EnFrame En_Reanchor( const EnFrame &F, const EnXf &T ){
	EnFrame n;
	for ( int r = 0; r < 3; r++ )
		n.o[r] = F.o[r] + F.M[r * 3] * T.t[0] + F.M[r * 3 + 1] * T.t[1] + F.M[r * 3 + 2] * T.t[2];
	for ( int r = 0; r < 3; r++ )
		for ( int c = 0; c < 3; c++ )
			n.M[r * 3 + c] = T.s * ( F.M[r * 3] * T.R[c] + F.M[r * 3 + 1] * T.R[3 + c] + F.M[r * 3 + 2] * T.R[6 + c] );
	return n;
}

static double En_MaxResidual( const EnPts &from, const EnPts &to, const EnXf &T ){
	double worst = 0;
	for ( size_t i = 0; i < from.size(); i++ )
		worst = std::max( worst, En_Dist( En_Apply( T, from[i] ), to[i] ) );
	return worst;
}

// The hooks: the whole patch was moved, turned (any matrix) or scaled about vOrigin.
void Entangle_FrameMoved( patchMesh_t *p, const vec3_t vMove ){
	if ( !p->lnkHasF ) {
		return;
	}
	for ( int k = 0; k < 3; k++ )
		p->lnkF[k] += vMove[k];
}

void Entangle_FrameMatrix( patchMesh_t *p, const vec3_t vOrigin, const vec3_t vMatrix[3] ){
	if ( !p->lnkHasF ) {
		return;
	}
	float n[12];
	for ( int r = 0; r < 3; r++ )
	{
		n[r] = vMatrix[r][0] * ( p->lnkF[0] - vOrigin[0] ) + vMatrix[r][1] * ( p->lnkF[1] - vOrigin[1] ) + vMatrix[r][2] * ( p->lnkF[2] - vOrigin[2] ) + vOrigin[r];
		for ( int a = 0; a < 3; a++ )
			n[3 + a * 3 + r] = vMatrix[r][0] * p->lnkF[3 + a * 3] + vMatrix[r][1] * p->lnkF[3 + a * 3 + 1] + vMatrix[r][2] * p->lnkF[3 + a * 3 + 2];
	}
	memcpy( p->lnkF, n, sizeof( n ) );
}

void Entangle_FrameScaled( patchMesh_t *p, const vec3_t vOrigin, const vec3_t vAmt ){
	if ( !p->lnkHasF ) {
		return;
	}
	for ( int r = 0; r < 3; r++ )
	{
		p->lnkF[r] = ( p->lnkF[r] - vOrigin[r] ) * vAmt[r] + vOrigin[r];
		for ( int a = 0; a < 3; a++ )
			p->lnkF[3 + a * 3 + r] *= vAmt[r];
	}
}

// =============================================================================
// bringing the instances to the shape of the source

// Bring the instances of one link to the shape of the source. Returns true when something was changed.
static bool En_SyncLink( int id, EnLink &L ){
	if ( L.src.empty() ) {
		return false; // the source is gone (deleted, or undone away): the instances wait
	}
	EnShape cur; // the source in the world
	En_Flatten( L.src, cur );

	EnFrame Fs;
	if ( !En_SetFrame( L.src, Fs ) ) {
		En_NewFrame( cur.pts, Fs ); // a link saved without its origin: the middle of the source
	}
	double inv[9];
	if ( !En_Inv3( Fs.M, inv ) ) {
		return false;
	}
	EnPts Lcur; // the source in its own frame
	for ( size_t i = 0; i < cur.pts.size(); i++ )
		Lcur.push_back( En_ToLocal( Fs, inv, cur.pts[i] ) );

	std::map<int, EnShape>::iterator sit = g_enSnap.find( id );
	if ( sit == g_enSnap.end() ) {
		EnShape s = cur;
		s.pts = Lcur;
		sit = g_enSnap.insert( std::make_pair( id, s ) ).first; // first sight of this link: the source is what it is
	}
	const EnShape &S0 = sit->second; // the source as it was, in its frame

	bool srcChanged = false;
	const bool sameTopo = S0.idx == cur.idx && S0.dims == cur.dims;
	if ( sameTopo ) {
		double worst = 0;
		for ( size_t i = 0; i < Lcur.size(); i++ )
			worst = std::max( worst, En_Dist( Lcur[i], S0.pts[i] ) );
		if ( worst > EN_TOL ) {
			// Something moved that the hooks did not see (a command that writes the points directly). When every point
			// agrees on one turn and shift it was the whole source: the origin is moved to match, nothing was edited.
			EnXf T;
			if ( En_Fit( S0.pts, Lcur, NULL, false, T ) && En_MaxResidual( S0.pts, Lcur, T ) < 0.05 ) {
				Fs = En_Reanchor( Fs, T );
				Lcur = S0.pts;
			}
			else{
				srcChanged = true;
			}
		}
	}
	else{
		// the grid size, the segments or the patches of the source changed
		if ( S0.idx != cur.idx ) {
			std::set<int> before( S0.idx.begin(), S0.idx.end() ), now( cur.idx.begin(), cur.idx.end() );
			std::set<int> removed;
			bool added = false;
			for ( std::set<int>::iterator b = before.begin(); b != before.end(); ++b )
				if ( !now.count( *b ) ) {
					removed.insert( *b );
				}
			for ( std::set<int>::iterator n = now.begin(); n != now.end(); ++n )
				if ( !before.count( *n ) ) {
					added = true;
				}
			EnShape s = cur;
			s.pts = Lcur;
			if ( !added && !removed.empty() ) {
				// parts of the source were deleted: the same parts of every instance go too (as an undo step of their own)
				std::vector<brush_t*> victims;
				for ( std::map<int, std::map<int, patchMesh_t*> >::iterator ii = L.inst.begin(); ii != L.inst.end(); ++ii )
					for ( std::set<int>::iterator r = removed.begin(); r != removed.end(); ++r )
					{
						std::map<int, patchMesh_t*>::iterator m = ii->second.find( *r );
						if ( m != ii->second.end() && m->second->pSymbiot ) {
							victims.push_back( m->second->pSymbiot );
						}
					}
				g_enSnap[id] = s; // from now on the source is what is left of it
				En_PutAll( L.src, Fs );
				if ( victims.empty() ) {
					return false;
				}
				// Not an undo step: undoing the deletion of the source parts brings them back, and the instances then make
				// their parts again (see below), so one undo is enough.
				for ( size_t v = 0; v < victims.size(); v++ )
				{
					brush_t *b = victims[v];
					for ( brush_t *sb = selected_brushes.next; sb != &selected_brushes; sb = sb->next )
						if ( sb == b ) {
							Select_Deselect(); // a selected one cannot just disappear from under the selection
							break;
						}
					entity_t *e = b->owner;
					Patch_Delete( b->pPatch );
					Brush_Free( b );
					if ( e != world_entity && e->brushes.onext == &e->brushes ) {
						Entity_Free( e );
					}
				}
				Sys_Printf( "Entangle: link %d: %d patches deleted from the instances, following the source.\n", id, (int)victims.size() );
				return true;
			}
			if ( added ) {
				// Parts came back (undo of a delete) or were added. An instance that lacks some of them gets them made again
				// from the source, in its own place; when that is not possible the instances are left as they are.
				std::vector<size_t> offs; // where every source part starts in Lcur
				size_t o = 0;
				for ( size_t k = 0; k < cur.idx.size(); k++ )
				{
					offs.push_back( o );
					o += (size_t)cur.dims[k * 4] * cur.dims[k * 4 + 1];
				}
				bool ok = true;
				int made = 0;
				for ( std::map<int, std::map<int, patchMesh_t*> >::iterator ii = L.inst.begin(); ii != L.inst.end() && ok; ++ii )
				{
					EnShape I;
					En_Flatten( ii->second, I );
					if ( I.idx == cur.idx ) {
						continue;
					}
					std::set<int> have( I.idx.begin(), I.idx.end() );
					bool subset = !have.empty();
					for ( std::set<int>::iterator h = have.begin(); h != have.end(); ++h )
						if ( !now.count( *h ) ) {
							subset = false;
						}
					EnFrame Fi;
					if ( !subset || !En_SetFrame( ii->second, Fi ) ) {
						ok = false;
						break;
					}
					const int group = ii->second.begin()->second->groupId;
					for ( size_t k = 0; k < cur.idx.size(); k++ )
					{
						if ( have.count( cur.idx[k] ) ) {
							continue;
						}
						patchMesh_t *np = Patch_Duplicate( L.src.find( cur.idx[k] )->second );
						np->lnkId = id;
						np->lnkRole = 2;
						np->lnkInst = ii->first;
						np->lnkIdx = cur.idx[k];
						np->groupId = group;
						En_PutFrame( np, Fi );
						size_t q = offs[k];
						for ( int i = 0; i < np->width; i++ )
							for ( int j = 0; j < np->height; j++ )
							{
								const EnP w = En_ToWorld( Fi, Lcur[q++] );
								for ( int a = 0; a < 3; a++ )
									np->ctrl[i][j].xyz[a] = (float)w.v[a];
							}
						En_RebuildBounds( np );
						made++;
					}
				}
				if ( ok ) {
					g_enSnap[id] = s;
					En_PutAll( L.src, Fs );
					if ( made ) {
						Sys_Printf( "Entangle: link %d: %d patches made again in the instances, following the source.\n", id, made );
					}
					return made > 0;
				}
			}
		}
		bool segmented = true;
		for ( std::map<int, patchMesh_t*>::const_iterator it = L.src.begin(); it != L.src.end(); ++it )
			if ( it->second->segH <= 0 ) {
				segmented = false;
			}
		if ( !segmented || S0.idx != cur.idx ) {
			if ( g_enWarned.insert( id ).second ) {
				Sys_Printf( "Entangle: link %d: the number of patches of the source, or the grid of an ordinary patch, changed; the instances are left as they are.\n", id );
			}
			return false;
		}
		srcChanged = true;
	}
	g_enWarned.erase( id );
	En_PutAll( L.src, Fs );

	const EnPts &newL = srcChanged ? Lcur : S0.pts;
	const std::vector<int> &newDims = srcChanged ? cur.dims : S0.dims;

	bool changed = false;
	for ( std::map<int, std::map<int, patchMesh_t*> >::iterator it = L.inst.begin(); it != L.inst.end(); ++it )
	{
		std::map<int, patchMesh_t*> &members = it->second;
		EnShape I;
		En_Flatten( members, I );
		if ( I.idx != S0.idx ) {
			if ( g_enWarned.insert( id * 1000 + it->first ).second ) {
				Sys_Printf( "Entangle: link %d, instance %d: it does not have the same patches as the source, so it is left alone (Unlink it, or make it again).\n", id, it->first );
			}
			continue;
		}
		const bool dimsMatchOld = I.dims == S0.dims;

		EnFrame Fi;
		if ( !En_SetFrame( members, Fi ) ) {
			// no origin yet (an older file, or a copy made by hand): worked out from where it stands
			if ( dimsMatchOld ) {
				EnXf P;
				std::vector<double> res;
				if ( !En_FitConsensus( S0.pts, I.pts, true, P, res ) ) {
					continue;
				}
				Fi.o[0] = P.t[0];
				Fi.o[1] = P.t[1];
				Fi.o[2] = P.t[2];
				for ( int k = 0; k < 9; k++ )
					Fi.M[k] = P.s * P.R[k];
			}
			else{
				double c0[3], c1[3];
				En_Centroid( S0.pts, c0 );
				En_Centroid( I.pts, c1 );
				for ( int k = 0; k < 3; k++ )
					Fi.o[k] = c1[k] - c0[k];
				for ( int k = 0; k < 9; k++ )
					Fi.M[k] = ( k % 4 == 0 ) ? 1 : 0;
			}
		}
		double invI[9];
		if ( !En_Inv3( Fi.M, invI ) ) {
			continue;
		}

		bool deviated = false;
		if ( dimsMatchOld ) {
			EnPts Li;
			double worst = 0;
			for ( size_t i = 0; i < I.pts.size(); i++ )
			{
				Li.push_back( En_ToLocal( Fi, invI, I.pts[i] ) );
				worst = std::max( worst, En_Dist( Li[i], S0.pts[i] ) );
			}
			if ( worst > EN_TOL ) {
				EnXf T;
				if ( En_Fit( S0.pts, Li, NULL, true, T ) && En_MaxResidual( S0.pts, Li, T ) < 0.05 ) {
					Fi = En_Reanchor( Fi, T ); // the instance as a whole is somewhere else now: that is its new place
				}
				else{
					deviated = true; // single points of it were edited
				}
			}
		}
		En_PutAll( members, Fi );

		if ( !srcChanged && !deviated && I.dims == newDims ) {
			continue;
		}
		if ( deviated && !srcChanged ) {
			Sys_Printf( "Entangle: link %d: a change to instance %d was undone; edit the source (or Unlink the instance first).\n", id, it->first );
		}

		size_t off = 0;
		size_t k = 0;
		for ( std::map<int, patchMesh_t*>::iterator mi = members.begin(); mi != members.end(); ++mi, k++ )
		{
			patchMesh_t *p = mi->second;
			const int *d = &newDims[k * 4];
			const bool dimChange = p->width != d[0] || p->height != d[1];
			std::map<int, patchMesh_t*>::const_iterator sp = L.src.find( mi->first );
			p->width = d[0];
			p->height = d[1];
			p->segH = d[2];
			p->segV = d[3];
			for ( int i = 0; i < p->width; i++ )
				for ( int j = 0; j < p->height; j++ )
				{
					if ( dimChange && sp != L.src.end() && i < sp->second->width && j < sp->second->height ) {
						p->ctrl[i][j].st[0] = sp->second->ctrl[i][j].st[0];
						p->ctrl[i][j].st[1] = sp->second->ctrl[i][j].st[1];
					}
					const EnP q = En_ToWorld( Fi, newL[off++] );
					for ( int a = 0; a < 3; a++ )
						p->ctrl[i][j].xyz[a] = (float)q.v[a];
				}
			En_RebuildBounds( p );
		}
		changed = true;
	}

	if ( srcChanged ) {
		EnShape s = cur;
		s.pts = Lcur;
		g_enSnap[id] = s; // from now on the source is compared with this
	}
	return changed;
}

bool Entangle_Sync(){
	std::map<int, EnLink> links;
	En_Gather( links );
	bool changed = false;
	for ( std::map<int, EnLink>::iterator it = links.begin(); it != links.end(); ++it )
		if ( En_SyncLink( it->first, it->second ) ) {
			changed = true;
		}
	if ( changed ) {
		UpdatePatchInspector();
	}
	return changed;
}

// =============================================================================
// the timer

static void En_Mix( unsigned long long &h, unsigned int v ){
	h ^= v;
	h *= 1099511628211ULL;
}

static unsigned long long En_StateHash( bool *any ){
	unsigned long long h = 14695981039346656037ULL;
	*any = false;
	std::vector<brush_t*> all = En_AllPatchBrushes();
	for ( size_t i = 0; i < all.size(); i++ )
	{
		const patchMesh_t *p = all[i]->pPatch;
		if ( !En_IsLinked( p ) ) {
			continue;
		}
		*any = true;
		En_Mix( h, (unsigned int)p->lnkId );
		En_Mix( h, (unsigned int)( p->lnkRole * 65536 + p->lnkInst ) );
		En_Mix( h, (unsigned int)p->lnkIdx );
		En_Mix( h, (unsigned int)( p->width * 256 + p->height ) );
		En_Mix( h, (unsigned int)( p->segH * 256 + p->segV ) );
		for ( int x = 0; x < p->width; x++ )
			for ( int y = 0; y < p->height; y++ )
				for ( int k = 0; k < 3; k++ )
				{
					unsigned int u;
					memcpy( &u, &p->ctrl[x][y].xyz[k], 4 );
					En_Mix( h, u );
				}
	}
	return h;
}

static unsigned long long g_enLastHash = 0;
static int g_enStable = 0;
static bool g_enPending = false;

// Every 100 ms. When something entangled has changed and has then been left alone for two ticks (0.2 s), the instances
// are brought to the shape of their sources. Dragging a point of a source does not redo them at every step.
static gboolean En_Tick( gpointer ){
	if ( Straight_Active() ) {
		return TRUE; // everything is in its straight pose: the instances are brought up to date on leaving
	}
	bool any;
	unsigned long long h = En_StateHash( &any );
	if ( !any ) {
		g_enLastHash = h;
		g_enPending = false;
		return TRUE;
	}
	if ( h != g_enLastHash ) {
		g_enLastHash = h;
		g_enStable = 0;
		g_enPending = true;
		return TRUE;
	}
	if ( g_enPending && ++g_enStable >= 2 ) {
		g_enPending = false;
		MapModifiedKeep keep; // the source's edit marked the map; following it is not an edit of its own
		if ( Entangle_Sync() ) {
			Sys_UpdateWindows( W_ALL );
		}
		bool a2;
		g_enLastHash = En_StateHash( &a2 );
	}
	return TRUE;
}

void Entangle_Init(){
	g_timeout_add( 100, En_Tick, NULL );
}

// =============================================================================
// commands

static std::vector<brush_t*> En_SelectedPatches(){
	std::vector<brush_t*> v;
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch ) {
			v.push_back( b );
		}
	return v;
}

static int En_Unlink( int onlyRole ); // 0 = any role, 1 = sources only, 2 = instances only

bool Entangle_MakeSource(){
	std::vector<brush_t*> sel = En_SelectedPatches();
	std::vector<patchMesh_t*> ps;
	int skipped = 0;
	for ( size_t i = 0; i < sel.size(); i++ )
	{
		if ( En_IsLinked( sel[i]->pPatch ) ) {
			skipped++;
		}
		else{
			ps.push_back( sel[i]->pPatch );
		}
	}
	if ( ps.empty() && skipped ) {
		// the button on a source switches its link off again; instances are the other button's business
		if ( En_Unlink( 1 ) == 0 ) {
			Sys_Printf( "Entangle: the selection is entangled already (an instance); use the Instanced button to free it.\n" );
			return false;
		}
		return true;
	}
	if ( ps.empty() ) {
		Sys_Printf( "Entangle: select the patches of the object that copies should follow.\n" );
		return false;
	}
	const int id = En_TopId() + 1;
	Undo_Start( "entangle: make source" );
	Undo_AddBrushList( &selected_brushes );
	// the origin of the object: the middle of its bounds, world axes; every patch of it carries a copy
	EnPts all;
	for ( size_t i = 0; i < ps.size(); i++ )
	{
		std::map<int, patchMesh_t*> one;
		one[0] = ps[i];
		EnShape s;
		En_Flatten( one, s );
		all.insert( all.end(), s.pts.begin(), s.pts.end() );
	}
	EnFrame frame;
	En_NewFrame( all, frame );
	for ( size_t i = 0; i < ps.size(); i++ )
	{
		ps[i]->lnkId = id;
		ps[i]->lnkRole = 1;
		ps[i]->lnkInst = 0;
		ps[i]->lnkIdx = (int)i + 1;
		En_PutFrame( ps[i], frame );
	}
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_MarkMapModified();
	Sys_Printf( "Entangle: %d patches are the source of link %d%s. Copies of it follow it from now on.\n", (int)ps.size(), id,
				skipped ? " (entangled patches in the selection were left out)" : "" );
	Sys_UpdateWindows( W_ALL );
	return true;
}

// ---- Make Instanced: which copy patch is which source patch

static void En_Match( const std::vector<patchMesh_t*> &S, const std::vector<patchMesh_t*> &C,
					  size_t k, std::vector<int> &pick, std::vector<char> &used,
					  std::vector<int> &best, double &bestRms, int &leaves ){
	if ( leaves > 4000 ) {
		return;
	}
	if ( k == S.size() ) {
		leaves++;
		EnPts a, b;
		for ( size_t i = 0; i < S.size(); i++ )
		{
			std::map<int, patchMesh_t*> m1, m2;
			m1[0] = S[i];
			m2[0] = C[pick[i]];
			EnShape s1, s2;
			En_Flatten( m1, s1 );
			En_Flatten( m2, s2 );
			a.insert( a.end(), s1.pts.begin(), s1.pts.end() );
			b.insert( b.end(), s2.pts.begin(), s2.pts.end() );
		}
		EnXf x;
		if ( !En_Fit( a, b, NULL, true, x ) ) {
			return;
		}
		double sum = 0;
		for ( size_t i = 0; i < a.size(); i++ )
		{
			const double d = En_Dist( En_Apply( x, a[i] ), b[i] );
			sum += d * d;
		}
		const double rms = sqrt( sum / a.size() );
		if ( best.empty() || rms < bestRms ) {
			bestRms = rms;
			best = pick;
		}
		return;
	}
	for ( size_t c = 0; c < C.size(); c++ )
	{
		if ( used[c] || C[c]->width != S[k]->width || C[c]->height != S[k]->height || C[c]->segH != S[k]->segH || C[c]->segV != S[k]->segV ) {
			continue;
		}
		used[c] = 1;
		pick[k] = (int)c;
		En_Match( S, C, k + 1, pick, used, best, bestRms, leaves );
		used[c] = 0;
	}
}

static int En_Find( std::vector<int> &up, int i ){
	while ( up[i] != i )
	{
		up[i] = up[up[i]];
		i = up[i];
	}
	return i;
}

int Entangle_MakeInstanced(){
	std::vector<brush_t*> sel = En_SelectedPatches();
	// the button on instances only switches their link off again (a source is the other button's business)
	if ( !sel.empty() ) {
		bool onlyInstances = true;
		for ( size_t i = 0; i < sel.size(); i++ )
			if ( sel[i]->pPatch->lnkRole != 2 ) {
				onlyInstances = false;
			}
		if ( onlyInstances ) {
			return En_Unlink( 2 );
		}
	}
	std::set<int> srcIds;
	for ( size_t i = 0; i < sel.size(); i++ )
		if ( sel[i]->pPatch->lnkRole == 1 && sel[i]->pPatch->lnkId > 0 ) {
			srcIds.insert( sel[i]->pPatch->lnkId );
		}
	if ( srcIds.size() != 1 ) {
		Sys_Printf( "Entangle: select the source (a patch of it) together with the copies that should follow it%s.\n",
					srcIds.empty() ? "; Make Source first if nothing is a source yet" : ", of one source only" );
		return 0;
	}
	const int id = *srcIds.begin();
	std::map<int, EnLink> links;
	En_Gather( links );
	EnLink &L = links[id];
	std::vector<patchMesh_t*> S;
	for ( std::map<int, patchMesh_t*>::iterator it = L.src.begin(); it != L.src.end(); ++it )
		S.push_back( it->second );

	std::vector<brush_t*> cand;
	int leftOut = 0;
	for ( size_t i = 0; i < sel.size(); i++ )
	{
		if ( sel[i]->pPatch->lnkRole == 0 ) {
			cand.push_back( sel[i] );
		}
		else if ( !( sel[i]->pPatch->lnkId == id && sel[i]->pPatch->lnkRole == 1 ) ) {
			leftOut++;
		}
	}
	if ( cand.empty() ) {
		Sys_Printf( "Entangle: no copies in the selection (everything selected is the source or entangled already).\n" );
		return 0;
	}

	// the copies, grouped into objects: by group when they all have one, otherwise by touching bounds
	std::vector<int> up( cand.size() );
	for ( size_t i = 0; i < up.size(); i++ )
		up[i] = (int)i;
	bool allGrouped = true;
	for ( size_t i = 0; i < cand.size(); i++ )
		if ( cand[i]->pPatch->groupId == 0 ) {
			allGrouped = false;
		}
	for ( size_t a = 0; a < cand.size(); a++ )
		for ( size_t b = a + 1; b < cand.size(); b++ )
		{
			bool join;
			if ( allGrouped ) {
				join = cand[a]->pPatch->groupId == cand[b]->pPatch->groupId;
			}
			else{
				join = true;
				for ( int k = 0; k < 3; k++ )
					if ( cand[a]->mins[k] > cand[b]->maxs[k] + 0.5f || cand[b]->mins[k] > cand[a]->maxs[k] + 0.5f ) {
						join = false;
					}
			}
			if ( join ) {
				up[En_Find( up, (int)a )] = En_Find( up, (int)b );
			}
		}
	std::map<int, std::vector<brush_t*> > objects;
	for ( size_t i = 0; i < cand.size(); i++ )
		objects[En_Find( up, (int)i )].push_back( cand[i] );

	int inst = 0;
	for ( std::map<int, std::map<int, patchMesh_t*> >::iterator ii = L.inst.begin(); ii != L.inst.end(); ++ii )
		inst = std::max( inst, ii->first );

	struct Job { std::vector<brush_t*> brushes; std::vector<int> pick; };
	std::vector<Job> jobs;
	for ( std::map<int, std::vector<brush_t*> >::iterator it = objects.begin(); it != objects.end(); ++it )
	{
		if ( it->second.size() != S.size() ) {
			Sys_Printf( "Entangle: an object of %d patches does not fit the source of %d patches; it is left out.\n", (int)it->second.size(), (int)S.size() );
			continue;
		}
		std::vector<patchMesh_t*> C;
		for ( size_t i = 0; i < it->second.size(); i++ )
			C.push_back( it->second[i]->pPatch );
		std::vector<int> pick( S.size() ), best;
		std::vector<char> used( C.size(), 0 );
		double rms = 0;
		int leaves = 0;
		En_Match( S, C, 0, pick, used, best, rms, leaves );
		if ( best.empty() ) {
			Sys_Printf( "Entangle: an object does not have the same patches (grid sizes, segments) as the source; it is left out.\n" );
			continue;
		}
		Job j;
		j.brushes = it->second;
		j.pick = best;
		jobs.push_back( j );
	}
	if ( jobs.empty() ) {
		return 0;
	}

	Undo_Start( "entangle: make instanced" );
	Undo_AddBrushList( &selected_brushes );
	for ( size_t j = 0; j < jobs.size(); j++ )
	{
		inst++;
		for ( size_t k = 0; k < S.size(); k++ )
		{
			patchMesh_t *p = jobs[j].brushes[jobs[j].pick[k]]->pPatch;
			p->lnkId = id;
			p->lnkRole = 2;
			p->lnkInst = inst;
			p->lnkIdx = S[k]->lnkIdx;
		}
	}
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_MarkMapModified();
	Sys_Printf( "Entangle: %d objects follow the source of link %d now%s.\n", (int)jobs.size(), id,
				leftOut ? " (other entangled patches in the selection were left out)" : "" );
	Sys_UpdateWindows( W_ALL );
	return (int)jobs.size();
}

int Entangle_Unlink(){
	return En_Unlink( 0 );
}

// Free the selected entangled patches. A source takes its whole link with it (the instances become plain patches too);
// an instance is freed alone. onlyRole restricts it to the selected sources (1) or the selected instances (2).
static int En_Unlink( int onlyRole ){
	std::vector<brush_t*> sel = En_SelectedPatches();
	if ( onlyRole ) {
		std::vector<brush_t*> keep;
		for ( size_t i = 0; i < sel.size(); i++ )
			if ( En_IsLinked( sel[i]->pPatch ) && sel[i]->pPatch->lnkRole == onlyRole ) {
				keep.push_back( sel[i] );
			}
		sel = keep;
	}
	std::set<int> dissolve;
	std::vector<brush_t*> touched;
	for ( size_t i = 0; i < sel.size(); i++ )
		if ( En_IsLinked( sel[i]->pPatch ) && sel[i]->pPatch->lnkRole == 1 ) {
			dissolve.insert( sel[i]->pPatch->lnkId );
		}
	std::vector<brush_t*> all = En_AllPatchBrushes();
	for ( size_t i = 0; i < all.size(); i++ )
	{
		const patchMesh_t *p = all[i]->pPatch;
		if ( !En_IsLinked( p ) ) {
			continue;
		}
		const bool isSel = std::find( sel.begin(), sel.end(), all[i] ) != sel.end();
		if ( isSel || dissolve.count( p->lnkId ) ) {
			touched.push_back( all[i] );
		}
	}
	if ( touched.empty() ) {
		if ( !onlyRole ) {
			Sys_Printf( "Entangle: select an entangled patch first.\n" );
		}
		return 0;
	}
	Undo_Start( "entangle: unlink" );
	for ( size_t i = 0; i < touched.size(); i++ )
		Undo_AddBrush( touched[i] );
	for ( size_t i = 0; i < touched.size(); i++ )
	{
		patchMesh_t *p = touched[i]->pPatch;
		p->lnkId = p->lnkRole = p->lnkInst = p->lnkIdx = p->lnkHasF = 0;
	}
	for ( size_t i = 0; i < touched.size(); i++ )
		Undo_EndBrush( touched[i] );
	Undo_End();
	Sys_MarkMapModified();
	Sys_Printf( "Entangle: %d patches are free of their link.\n", (int)touched.size() );
	Sys_UpdateWindows( W_ALL );
	return (int)touched.size();
}

static int En_SelectLinks( bool sourceOnly ){
	std::vector<brush_t*> sel = En_SelectedPatches();
	std::set<int> ids;
	for ( size_t i = 0; i < sel.size(); i++ )
		if ( En_IsLinked( sel[i]->pPatch ) ) {
			ids.insert( sel[i]->pPatch->lnkId );
		}
	if ( ids.empty() ) {
		Sys_Printf( "Entangle: select an entangled patch first.\n" );
		return 0;
	}
	std::vector<brush_t*> add;
	for ( brush_t *b = active_brushes.next; b != &active_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch && En_IsLinked( b->pPatch ) && ids.count( b->pPatch->lnkId ) && ( !sourceOnly || b->pPatch->lnkRole == 1 ) ) {
			add.push_back( b );
		}
	for ( size_t i = 0; i < add.size(); i++ )
		Select_Brush( add[i], false, false );
	if ( !add.empty() ) {
		UpdatePatchInspector();
		UpdateSurfaceDialog();
	}
	Sys_Printf( "Entangle: %d patches added to the selection.\n", (int)add.size() );
	Sys_UpdateWindows( W_ALL );
	return (int)add.size();
}

int Entangle_SelectSource(){
	return En_SelectLinks( true );
}

int Entangle_SelectAll(){
	return En_SelectLinks( false );
}

// =============================================================================
// copies: a copy of a source, or of an instance, is an instance of the same link

static std::set<brush_t*> g_enBefore;

void Entangle_ImportBegin(){
	g_enBefore.clear();
	std::vector<brush_t*> all = En_AllPatchBrushes();
	for ( size_t i = 0; i < all.size(); i++ )
		g_enBefore.insert( all[i] );
}

void Entangle_ImportEnd(){
	std::vector<brush_t*> all = En_AllPatchBrushes();
	std::set<int> sourceHere; // the links whose source was in the map before the paste
	for ( size_t i = 0; i < all.size(); i++ )
		if ( g_enBefore.count( all[i] ) && all[i]->pPatch->lnkRole == 1 && all[i]->pPatch->lnkId > 0 ) {
			sourceHere.insert( all[i]->pPatch->lnkId );
		}
	std::map<int, int> top; // link -> highest instance number in use
	for ( size_t i = 0; i < all.size(); i++ )
		if ( En_IsLinked( all[i]->pPatch ) ) {
			top[all[i]->pPatch->lnkId] = std::max( top[all[i]->pPatch->lnkId], all[i]->pPatch->lnkInst );
		}
	std::map<std::pair<int, std::pair<int, int> >, int> renum; // (link, (role, inst) it was copied from) -> its new instance number
	int n = 0;
	for ( size_t i = 0; i < all.size(); i++ )
	{
		patchMesh_t *p = all[i]->pPatch;
		if ( g_enBefore.count( all[i] ) || !En_IsLinked( p ) || !sourceHere.count( p->lnkId ) ) {
			continue; // not a copy, or a copy of something whose source is not in the map: it stays what it was
		}
		const std::pair<int, std::pair<int, int> > key = std::make_pair( p->lnkId, std::make_pair( p->lnkRole, p->lnkInst ) );
		std::map<std::pair<int, std::pair<int, int> >, int>::iterator it = renum.find( key );
		if ( it == renum.end() ) {
			it = renum.insert( std::make_pair( key, ++top[p->lnkId] ) ).first;
		}
		p->lnkRole = 2;
		p->lnkInst = it->second;
		n++;
	}
	if ( n > 0 ) {
		Sys_Printf( "Entangle: %d pasted patches follow their source.\n", n );
	}
	g_enBefore.clear();
}

// =============================================================================
// outlines and numbers

static bool g_enOutlines = true;

void Entangle_ToggleOutlines(){
	g_enOutlines = !g_enOutlines;
	Sys_Printf( "Entangle: outlines and numbers %s.\n", g_enOutlines ? "on" : "off" );
	Sys_UpdateWindows( W_ALL );
}

bool Entangle_OutlinesOn(){
	return g_enOutlines;
}

// A colour for every link, from its number. The hues of the seat cutters (cyan) and their overlay (red-pink) are left out.
static void En_Color( int id, float rgb[3] ){
	double u = fmod( id * 0.6180339887, 1.0 ) * 0.70; // along the allowed hues
	double h = u < 0.38 ? 0.06 + u : 0.56 + ( u - 0.38 );
	const float sat = 0.9f, val = 1.0f;
	const double hh = h * 6;
	const int i = (int)floor( hh );
	const float f = (float)( hh - i );
	const float p = val * ( 1 - sat ), q = val * ( 1 - sat * f ), t = val * ( 1 - sat * ( 1 - f ) );
	switch ( i % 6 )
	{
	case 0: rgb[0] = val; rgb[1] = t; rgb[2] = p; break;
	case 1: rgb[0] = q; rgb[1] = val; rgb[2] = p; break;
	case 2: rgb[0] = p; rgb[1] = val; rgb[2] = t; break;
	case 3: rgb[0] = p; rgb[1] = q; rgb[2] = val; break;
	case 4: rgb[0] = t; rgb[1] = p; rgb[2] = val; break;
	default: rgb[0] = val; rgb[1] = p; rgb[2] = q; break;
	}
}

static bool En_SamePoint( const float *a, const float *b ){
	return fabsf( a[0] - b[0] ) + fabsf( a[1] - b[1] ) + fabsf( a[2] - b[2] ) < 0.5f;
}

static void En_Strip( const segGrid_t *g, int u0, int v0, int du, int dv, int count ){
	qglBegin( GL_LINE_STRIP );
	for ( int k = 0; k < count; k++ )
		qglVertex3fv( g->p[( u0 + du * k ) * g->V + v0 + dv * k].xyz );
	qglEnd();
}

// the border of a patch (the seam of a ring is left out); an ordinary patch has its control points' border
static void En_Outline( patchMesh_t *p ){
	const segGrid_t *g = p->segH > 0 ? Patch_GetSegGrid( p ) : NULL;
	if ( g && g->U >= 2 && g->V >= 2 ) {
		const int U = g->U, V = g->V;
		const bool ringU = En_SamePoint( g->p[0].xyz, g->p[( U - 1 ) * V].xyz ) && En_SamePoint( g->p[V - 1].xyz, g->p[( U - 1 ) * V + V - 1].xyz );
		const bool ringV = En_SamePoint( g->p[0].xyz, g->p[V - 1].xyz ) && En_SamePoint( g->p[( U - 1 ) * V].xyz, g->p[( U - 1 ) * V + V - 1].xyz );
		if ( !ringU ) {
			En_Strip( g, 0, 0, 0, 1, V );
			En_Strip( g, U - 1, 0, 0, 1, V );
		}
		if ( !ringV ) {
			En_Strip( g, 0, 0, 1, 0, U );
			En_Strip( g, 0, V - 1, 1, 0, U );
		}
		return;
	}
	const int W = p->width, H = p->height;
	if ( W < 2 || H < 2 ) {
		return;
	}
	qglBegin( GL_LINE_LOOP );
	for ( int i = 0; i < W; i++ )
		qglVertex3fv( p->ctrl[i][0].xyz );
	for ( int j = 1; j < H; j++ )
		qglVertex3fv( p->ctrl[W - 1][j].xyz );
	for ( int i = W - 2; i >= 0; i-- )
		qglVertex3fv( p->ctrl[i][H - 1].xyz );
	for ( int j = H - 2; j >= 1; j-- )
		qglVertex3fv( p->ctrl[0][j].xyz );
	qglEnd();
}

static void En_Label( const brush_t *b, const char *s, const float rgb[3] ){
	qglColor3f( rgb[0], rgb[1], rgb[2] );
	qglRasterPos3f( ( b->mins[0] + b->maxs[0] ) * 0.5f, ( b->mins[1] + b->maxs[1] ) * 0.5f, ( b->mins[2] + b->maxs[2] ) * 0.5f );
	gtk_glwidget_print_string( s );
}

void Entangle_Draw(){
	if ( !g_enOutlines || Straight_Active() ) {
		return;
	}
	std::map<int, EnLink> links;
	En_Gather( links );
	if ( links.empty() ) {
		return;
	}
	std::set<const patchMesh_t*> selected;
	std::set<int> hot; // the links that have a patch selected are drawn stronger
	for ( brush_t *b = selected_brushes.next; b != &selected_brushes; b = b->next )
		if ( b->patchBrush && b->pPatch ) {
			selected.insert( b->pPatch );
			if ( En_IsLinked( b->pPatch ) ) {
				hot.insert( b->pPatch->lnkId );
			}
		}

	qglPushAttrib( GL_ALL_ATTRIB_BITS );
	qglDisable( GL_TEXTURE_2D );
	qglDisable( GL_LIGHTING );
	qglDisable( GL_CULL_FACE );
	qglDisable( GL_DEPTH_TEST );
	qglDisable( GL_LINE_STIPPLE );
	qglEnable( GL_BLEND );
	qglBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	struct Lbl { const brush_t *b; char text[24]; float rgb[3]; };
	std::vector<Lbl> labels;

	for ( std::map<int, EnLink>::iterator it = links.begin(); it != links.end(); ++it )
	{
		const int id = it->first;
		float rgb[3];
		En_Color( id, rgb );
		const bool isHot = hot.count( id ) != 0;
		const bool orphan = it->second.src.empty();

		// the source: strong
		qglLineWidth( isHot ? 4.0f : 3.0f );
		qglColor4f( rgb[0], rgb[1], rgb[2], isHot ? 1.0f : 0.9f );
		for ( std::map<int, patchMesh_t*>::iterator m = it->second.src.begin(); m != it->second.src.end(); ++m )
		{
			if ( m->second->pSymbiot && m->second->pSymbiot->bFiltered ) {
				continue;
			}
			if ( !selected.count( m->second ) ) {
				En_Outline( m->second );
			}
		}
		if ( !it->second.src.empty() && it->second.src.begin()->second->pSymbiot ) {
			Lbl l;
			l.b = it->second.src.begin()->second->pSymbiot;
			sprintf( l.text, "%d", id );
			memcpy( l.rgb, rgb, sizeof( rgb ) );
			labels.push_back( l );
		}

		// the instances: the same colour, faint (an instance without a source is grey)
		qglLineWidth( isHot ? 2.0f : 1.0f );
		if ( orphan ) {
			qglColor4f( 0.6f, 0.6f, 0.6f, 0.5f );
		}
		else{
			qglColor4f( rgb[0], rgb[1], rgb[2], isHot ? 0.8f : 0.35f );
		}
		for ( std::map<int, std::map<int, patchMesh_t*> >::iterator ii = it->second.inst.begin(); ii != it->second.inst.end(); ++ii )
		{
			for ( std::map<int, patchMesh_t*>::iterator m = ii->second.begin(); m != ii->second.end(); ++m )
			{
				if ( m->second->pSymbiot && m->second->pSymbiot->bFiltered ) {
					continue;
				}
				if ( !selected.count( m->second ) ) {
					En_Outline( m->second );
				}
			}
			if ( !ii->second.empty() && ii->second.begin()->second->pSymbiot ) {
				Lbl l;
				l.b = ii->second.begin()->second->pSymbiot;
				sprintf( l.text, orphan ? "%d.%d?" : "%d.%d", id, ii->first );
				if ( orphan ) {
					l.rgb[0] = l.rgb[1] = l.rgb[2] = 0.7f;
				}
				else{
					memcpy( l.rgb, rgb, sizeof( rgb ) );
				}
				labels.push_back( l );
			}
		}
	}
	qglLineWidth( 1 );
	qglDisable( GL_BLEND );
	for ( size_t i = 0; i < labels.size(); i++ )
		En_Label( labels[i].b, labels[i].text, labels[i].rgb );
	qglPopAttrib();
}
