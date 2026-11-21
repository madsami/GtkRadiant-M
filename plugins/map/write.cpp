/*
   Copyright (C) 1999-2007 id Software, Inc. and contributors.
   For a list of contributors, see the accompanying CONTRIBUTORS file.

   This file is part of GtkRadiant.

   GtkRadiant is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   GtkRadiant is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GtkRadiant; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//
// writes quake3 map format from internal objects
//

static int g_count_entities;
static int g_count_brushes;

#include "plugin.h"
#include "segpatch.h"
extern int g_MapVersion;

void Float_Write( float data, IDataStream *out ){
	if ( data == (int)data ) {
		out->printf( "%i ", (int)data );
	}
	else{
		out->printf( "%f ", data );
	}
}

void Patch_Write( patchMesh_t *pPatch, IDataStream *out ){
	int i, j;
	const char *str;

	// write shader name and matrix dimensions
	str = pPatch->pShader->getName();
	if ( strchr( str, ' ' ) ) {
		Sys_FPrintf( SYS_WRN, "WARNING: Patch_Write: shader names with spaces are not allowed, ignoring '%s'\n", str );
		str = SHADER_NOT_FOUND;
	}
	if ( !strncmp( str, "textures/", 9 ) ) {
		str += 9;
	}
	out->printf( "patchDef2\n{\n%s\n( %i %i 0 0 0 )\n",
				 str, pPatch->width, pPatch->height );

	// write matrix
	out->printf( "(\n" );
	for ( i = 0; i < pPatch->width; i++ )
	{
		out->printf( "( " );
		for ( j = 0; j < pPatch->height; j++ )
		{
			out->printf( "( " );

			Float_Write( pPatch->ctrl[i][j].xyz[0], out );
			Float_Write( pPatch->ctrl[i][j].xyz[1], out );
			Float_Write( pPatch->ctrl[i][j].xyz[2], out );
			Float_Write( pPatch->ctrl[i][j].st[0], out );
			Float_Write( pPatch->ctrl[i][j].st[1], out );

			out->printf( ") " );
		}
		out->printf( ")\n" );
	}
	out->printf( ")\n}\n" );
}

static const char* Patch_ShaderForFile( patchMesh_t *pPatch ){
	const char *str = pPatch->pShader->getName();
	if ( strchr( str, ' ' ) ) {
		Sys_FPrintf( SYS_WRN, "WARNING: Patch_Write: shader names with spaces are not allowed, ignoring '%s'\n", str );
		str = SHADER_NOT_FOUND;
	}
	if ( !strncmp( str, "textures/", 9 ) ) {
		str += 9;
	}
	return str;
}

// A segmented patch (see segpatch.h) is saved as its cage in comment lines (//@seg), which the
// compilers ignore, followed by the generated pieces as ordinary patches, which they compile.
// Map_Read turns the comment lines back into the cage and drops the pieces.
static bool SegPatch_Write( patchMesh_t *p, IDataStream *out ){
	segGrid_t g;
	if ( !SegPatch_BuildGrid( p, p->segH, p->segV, g ) ) {
		return false;
	}
	std::vector<segPiece_t> pieces;
	SegPatch_BuildPieces( p, g, pieces );
	const char *str = Patch_ShaderForFile( p );

	if ( p->cogId != 0 ) {
		// the cog's id and numbers (see Primitive_Write); Map_Read puts them into the size line of the cage
		out->printf( "//@cog %i ", p->cogId );
		for ( int c = 0; c < COG_NPARAM; c++ )
			Float_Write( p->cog[c], out );
		out->printf( "\n" );
	}
	if ( p->seatId != 0 || p->seatCount > 0 ) {
		// seat cut (see seatcut.h): the id of the cutter this patch belongs to, then the hulls that cut a hole into it
		// ( id, plane count, the planes ). Map_Read puts them into the size line of the cage.
		out->printf( "//@seat seat %i %i ", p->seatId, p->seatCount );
		for ( int h = 0; h < p->seatCount; h++ )
		{
			out->printf( "%i %i ", p->seat[h].id, p->seat[h].n );
			for ( int k = 0; k < p->seat[h].n; k++ )
				for ( int c = 0; c < 4; c++ )
					Float_Write( p->seat[h].pl[k][c], out );
		}
		out->printf( "\n" );
	}
	out->printf( "// brush %i (segmented patch, %i x %i segments)\n", g_count_brushes++, p->segH, p->segV );
	out->printf( "//@seg {\n//@seg patchDef3\n//@seg {\n//@seg %s\n//@seg ( %i %i %i %i 0 0 0 )\n//@seg (\n",
				 str, p->width, p->height, p->segH, p->segV );
	for ( int i = 0; i < p->width; i++ )
	{
		out->printf( "//@seg ( " );
		for ( int j = 0; j < p->height; j++ )
		{
			out->printf( "( " );
			Float_Write( p->ctrl[i][j].xyz[0], out );
			Float_Write( p->ctrl[i][j].xyz[1], out );
			Float_Write( p->ctrl[i][j].xyz[2], out );
			Float_Write( p->ctrl[i][j].st[0], out );
			Float_Write( p->ctrl[i][j].st[1], out );
			out->printf( ") " );
		}
		out->printf( ")\n" );
	}
	out->printf( "//@seg )\n//@seg }\n//@seg }\n" );

	out->printf( "// segpieces begin %i\n", (int)pieces.size() );
	for ( size_t n = 0; n < pieces.size(); n++ )
	{
		const segPiece_t &pc = pieces[n];
		out->printf( "// brush %i\n{\n", g_count_brushes++ );
		out->printf( "patchDef2\n{\n%s\n( %i %i 0 0 0 )\n(\n", str, pc.width, pc.height );
		for ( int x = 0; x < pc.width; x++ )
		{
			out->printf( "( " );
			for ( int y = 0; y < pc.height; y++ )
			{
				const segPoint_t &c = pc.c[x * pc.height + y];
				out->printf( "( " );
				Float_Write( c.xyz[0], out );
				Float_Write( c.xyz[1], out );
				Float_Write( c.xyz[2], out );
				Float_Write( c.st[0], out );
				Float_Write( c.st[1], out );
				out->printf( ") " );
			}
			out->printf( ")\n" );
		}
		out->printf( ")\n}\n}\n" );
	}
	out->printf( "// segpieces end\n" );
	return true;
}

void Face_Write( face_t *face, IDataStream *out, bool bAlternateTexdef = false ){
	int i, j;
	const char *str;

	// write planepts
	for ( i = 0; i < 3; i++ )
	{
		out->printf( "( " );
		for ( j = 0; j < 3; j++ )
		{
			Float_Write( face->planepts[i][j], out );
		}
		out->printf( ") " );
	}

	if ( bAlternateTexdef ) {
		// write alternate texdef
		out->printf( "( ( " );
		for ( i = 0; i < 3; i++ )
			Float_Write( face->brushprimit_texdef.coords[0][i], out );
		out->printf( ") ( " );
		for ( i = 0; i < 3; i++ )
			Float_Write( face->brushprimit_texdef.coords[1][i], out );
		out->printf( ") ) " );
	}

	// write shader name
	str = face->texdef.GetName();
	if ( strchr( str, ' ' ) ) {
		Sys_FPrintf( SYS_WRN, "WARNING: Face_Write: shader names with spaces are not allowed, ignoring '%s'\n", str );
		str = SHADER_NOT_FOUND;
	}
	if ( !strncmp( str, "textures/", 9 ) ) {
		str += 9;
	}

	// Strip all remaining paths.
	// FIXME: Hydra - this is actually a HalfLife specific bit, not Q2 map format specific.
	if ( g_MapVersion == MAPVERSION_HL ) {
		char *pos;
		while ( ( pos = (char*)strchr( str, '/' ) ) != NULL ) {
			str = pos + 1; // to speed optimize, change the "while" to an "if"
		}
	}
	out->printf( "%s ", str );

	if ( !bAlternateTexdef ) {
		// write texdef
		out->printf( "%i %i %i %f %f ",
					 (int)face->texdef.shift[0],
					 (int)face->texdef.shift[1],
					 (int)face->texdef.rotate,
					 face->texdef.scale[0],
					 face->texdef.scale[1] );
	}

	if ( g_MapVersion == MAPVERSION_Q3 ) {
		// write surface flags
		out->printf( "%i %i %i\n",
					 face->texdef.contents,
					 face->texdef.flags,
					 face->texdef.value );
	}

	if ( ( g_MapVersion == MAPVERSION_HL ) || ( g_MapVersion == MAPVERSION_Q2 ) ) {
		// write surface flags if non-zero values.
		if ( face->texdef.contents || face->texdef.flags || face->texdef.value ) {
			out->printf( "%i %i %i\n",
						 face->texdef.contents,
						 face->texdef.flags,
						 face->texdef.value );
		}
		else
		{
			out->printf( "\n" );
		}
	}

}

void Primitive_Write( brush_t *pBrush, IDataStream *out ){
	if ( pBrush->patchBrush && pBrush->pPatch->segH > 0 && g_MapVersion == MAPVERSION_Q3 && SegPatch_Write( pBrush->pPatch, out ) ) {
		return;
	}
	if ( ( g_MapVersion == MAPVERSION_Q2 ) && ( pBrush->patchBrush ) ) {
		Sys_FPrintf( SYS_WRN, "WARNING: Primitive_Write: Patches are not supported in Quake2, ignoring Brush %d\n", g_count_brushes++ );
	}
	else
	{
		if ( pBrush->patchBrush && pBrush->pPatch->cogId != 0 && g_MapVersion == MAPVERSION_Q3 ) {
			// a patch of a cog: the id and the cog's numbers, in a comment line the compilers ignore.
			// The patch itself is written as usual. Map_Read puts the numbers back into the patch.
			out->printf( "//@cog %i ", pBrush->pPatch->cogId );
			for ( int c = 0; c < COG_NPARAM; c++ )
				Float_Write( pBrush->pPatch->cog[c], out );
			out->printf( "\n" );
		}
		out->printf( "// brush %i\n", g_count_brushes++ );
		out->printf( "{\n" );
		if ( pBrush->patchBrush ) {
			Patch_Write( pBrush->pPatch, out );
		}
		else if ( pBrush->bBrushDef ) {
			out->printf( "brushDef\n{\n" );
			for ( face_t *face = pBrush->brush_faces; face != NULL; face = face->next )
				Face_Write( face, out, true );
			out->printf( "}\n" );
		}
		else{
			for ( face_t *face = pBrush->brush_faces; face != NULL; face = face->next )
				Face_Write( face, out );
		}
		out->printf( "}\n" );
	}
}

void Entity_Write( entity_t *pEntity, IDataStream *out ){
	epair_t *pEpair;
	CPtrArray *brushes = (CPtrArray*)pEntity->pData;
	out->printf( "// entity %i\n", g_count_entities++ );
	out->printf( "{\n" );
	for ( pEpair = pEntity->epairs; pEpair != NULL; pEpair = pEpair->next )
		out->printf( "\"%s\" \"%s\"\n", pEpair->key, pEpair->value );
	g_count_brushes = 0;
	for ( int i = 0; i < brushes->GetSize(); i++ )
		Primitive_Write( (brush_t*)brushes->GetAt( i ), out );
	out->printf( "}\n" );
}

void Map_Write( CPtrArray *map, IDataStream *out ){
	g_count_entities = 0;
	for ( int i = 0; i < map->GetSize(); i++ )
		Entity_Write( (entity_t*)map->GetAt( i ), out );
}

void Map_WriteQ3( CPtrArray *map, IDataStream *out ){
	g_MapVersion = MAPVERSION_Q3;
	Map_Write( map,out );
}

void Map_WriteHL( CPtrArray *map, IDataStream *out ){
	g_MapVersion = MAPVERSION_HL;
	Map_Write( map,out );
}

void Map_WriteQ2( CPtrArray *map, IDataStream *out ){
	g_MapVersion = MAPVERSION_Q2;
	Map_Write( map,out );
}
