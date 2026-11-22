// Rotation frames, see rotframe.h

#include "stdafx.h"
#include <math.h>
#include "rotframe.h"
#include "cog.h"

// out = A * B (3x3, row by row)
static void RF_Mul( const float A[9], const float B[9], float out[9] ){
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
		{
			out[i * 3 + j] = A[i * 3 + 0] * B[0 * 3 + j] + A[i * 3 + 1] * B[1 * 3 + j] + A[i * 3 + 2] * B[2 * 3 + j];
		}
}

// Many small turns add up rounding errors: make the columns a clean right-handed set of unit vectors again
static void RF_Orthonormalize( float R[9] ){
	double c0[3] = { R[0], R[3], R[6] };
	double c1[3] = { R[1], R[4], R[7] };
	double l = sqrt( c0[0] * c0[0] + c0[1] * c0[1] + c0[2] * c0[2] );
	if ( l < 1e-12 ) {
		return;
	}
	for ( int k = 0; k < 3; k++ )
		c0[k] /= l;
	double d = c1[0] * c0[0] + c1[1] * c0[1] + c1[2] * c0[2];
	for ( int k = 0; k < 3; k++ )
		c1[k] -= d * c0[k];
	l = sqrt( c1[0] * c1[0] + c1[1] * c1[1] + c1[2] * c1[2] );
	if ( l < 1e-12 ) {
		return;
	}
	for ( int k = 0; k < 3; k++ )
		c1[k] /= l;
	double c2[3] = { c0[1] * c1[2] - c0[2] * c1[1], c0[2] * c1[0] - c0[0] * c1[2], c0[0] * c1[1] - c0[1] * c1[0] };
	for ( int k = 0; k < 3; k++ )
	{
		R[k * 3 + 0] = (float)c0[k];
		R[k * 3 + 1] = (float)c1[k];
		R[k * 3 + 2] = (float)c2[k];
	}
}

static bool RF_IsIdentity( const float R[9] ){
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
		{
			if ( fabs( R[i * 3 + j] - ( i == j ? 1.0f : 0.0f ) ) > 1e-5 ) {
				return false;
			}
		}
	return true;
}

static void RF_MulVec( const vec3_t M[3], const float *v, float *out ){
	float t[3];
	for ( int j = 0; j < 3; j++ )
		t[j] = DotProduct( v, M[j] );
	VectorCopy( t, out );
}

void RotFrame_PatchRotated( patchMesh_t *p, const vec3_t origin, const vec3_t M[3] ){
	// the frame
	float Mf[9], R[9], out[9];
	for ( int i = 0; i < 3; i++ )
		for ( int j = 0; j < 3; j++ )
			Mf[i * 3 + j] = M[i][j];
	if ( p->hasFrame ) {
		memcpy( R, p->frameR, sizeof( R ) );
	}
	else
	{
		for ( int i = 0; i < 9; i++ )
			R[i] = ( i % 4 == 0 ) ? 1.0f : 0.0f;
	}
	RF_Mul( Mf, R, out );
	RF_Orthonormalize( out );
	if ( RF_IsIdentity( out ) ) {
		p->hasFrame = 0; // back on the grid it was built on
	}
	else
	{
		memcpy( p->frameR, out, sizeof( out ) );
		p->hasFrame = 1;
	}

	// the numbers of a cog hold its middle (a point) and its axis and first tooth (directions) in world space
	if ( p->cogId != 0 ) {
		cogRec_t r;
		Cog_Unpack( p->cog, r );
		float t[3];
		VectorSubtract( r.C, origin, t );
		RF_MulVec( M, t, t );
		VectorAdd( t, origin, r.C );
		RF_MulVec( M, r.A, r.A );
		RF_MulVec( M, r.E1, r.E1 );
		Cog_Pack( r, p->cog );
	}
}

void RotFrame_PatchMirrored( patchMesh_t *p ){
	p->hasFrame = 0;
}

void RotFrame_FormatVec( char *out, const vec3_t v ){
	char buf[3][32];
	for ( int i = 0; i < 3; i++ )
	{
		float f = v[i];
		if ( fabs( f ) < 0.0005f ) {
			f = 0.0f;
		}
		sprintf( buf[i], "%.3f", f );
		char *e = buf[i] + strlen( buf[i] );
		while ( e > buf[i] && e[-1] == '0' )
			*--e = 0;
		if ( e > buf[i] && e[-1] == '.' ) {
			*--e = 0;
		}
	}
	sprintf( out, "%s %s %s", buf[0], buf[1], buf[2] );
}

// the facing of a point entity is a yaw in "angle" ( -1 up, -2 down ) or the middle number of "angles" ( pitch yaw roll )
void RotFrame_EntityRotated( entity_t *e, const vec3_t M[3] ){
	if ( !e || !e->eclass || !e->eclass->fixedsize ) {
		return;
	}
	const char *cls = ValueForKey( e, "classname" );
	const char *sa = ValueForKey( e, "angle" );
	const char *sas = ValueForKey( e, "angles" );
	float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
	bool useAngles = false;
	if ( sa && sa[0] ) {
		yaw = (float)atof( sa );
		if ( yaw < 0.0f ) {
			return; // up or down
		}
	}
	else if ( sas && sas[0] ) {
		if ( sscanf( sas, "%f %f %f", &pitch, &yaw, &roll ) != 3 ) {
			return;
		}
		useAngles = true;
	}
	else if ( !strncmp( cls, "light", 5 ) ) {
		return; // no facing to keep
	}
	// where the entity looks, turned like the rest
	float d[3] = { (float)cos( yaw * Q_PI / 180.0 ), (float)sin( yaw * Q_PI / 180.0 ), 0.0f };
	RF_MulVec( M, d, d );
	if ( d[0] * d[0] + d[1] * d[1] < 1e-6f ) {
		return; // now looking straight up or down
	}
	float ny = (float)( atan2( d[1], d[0] ) * 180.0 / Q_PI );
	if ( ny < 0.0f ) {
		ny += 360.0f;
	}
	if ( fabs( ny - 360.0f ) < 0.0005f ) {
		ny = 0.0f;
	}
	if ( !useAngles && !( sa && sa[0] ) && ny == 0.0f ) {
		return; // had no facing and still looks the default way: leave the keys alone
	}
	char text[96];
	if ( useAngles ) {
		vec3_t a = { pitch, ny, roll };
		RotFrame_FormatVec( text, a );
		SetKeyValue( e, "angles", text );
	}
	else
	{
		vec3_t a = { ny, 0, 0 };
		RotFrame_FormatVec( text, a );
		*strchr( text, ' ' ) = 0; // only the first number
		SetKeyValue( e, "angle", text );
	}
}

// the pivot of the mouse rotation as three axis lines through it, in the camera view
void RotFrame_DrawPivot(){
	if ( !g_bRotateMode || !g_pParentWnd || !g_pParentWnd->ActiveXY() ) {
		return;
	}
	const float *c = g_pParentWnd->ActiveXY()->RotateOrigin();
	vec3_t d;
	VectorSubtract( c, g_pParentWnd->GetCamWnd()->Camera()->origin, d );
	float s = VectorLength( d ) * 0.18f; // three times the first size
	if ( s < 24.0f ) {
		s = 24.0f;
	}
	qglDisable( GL_TEXTURE_2D );
	qglDisable( GL_DEPTH_TEST );
	qglLineWidth( 3 );
	qglBegin( GL_LINES );
	qglColor3f( 1.0f, 0.25f, 0.25f );
	qglVertex3f( c[0] - s, c[1], c[2] );
	qglVertex3f( c[0] + s, c[1], c[2] );
	qglColor3f( 0.25f, 1.0f, 0.25f );
	qglVertex3f( c[0], c[1] - s, c[2] );
	qglVertex3f( c[0], c[1] + s, c[2] );
	qglColor3f( 0.35f, 0.5f, 1.0f );
	qglVertex3f( c[0], c[1], c[2] - s );
	qglVertex3f( c[0], c[1], c[2] + s );
	qglEnd();
	qglLineWidth( 1 );
	qglPointSize( 14 );
	qglColor3f( 1.0f, 0.2f, 1.0f );
	qglBegin( GL_POINTS );
	qglVertex3fv( c );
	qglEnd();
	qglPointSize( 1 );
	qglEnable( GL_DEPTH_TEST );
}
