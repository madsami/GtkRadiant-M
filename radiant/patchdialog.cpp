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
// Patch Dialog
//
// Leonardo Zide (leo@lokigames.com)
//

#include <gdk/gdkkeysyms.h>
#include "stdafx.h"
#include "segpatch.h"
#include "cog.h"
#include "seatcut.h"
#include "patchdialog.h"
#include <glib/gi18n.h>

PatchDialog g_PatchDialog;
// is the patch inspector currently displayed/active?
bool l_bIsActive = false;
// the increment we are using for the patch inspector (this is saved in the prefs)
texdef_t *l_pPIIncrement = &g_qeglobals.d_savedinfo.m_PIIncrement;

// =============================================================================
// static functions

// segment counts: applied to every selected patch as soon as they change
static void SegApply(){
	if ( !g_PatchDialog.m_bSegListen ) {
		return;
	}
	bool on = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_PatchDialog.m_pSegCheck ) ) != FALSE;
	int h = gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_PatchDialog.m_pSegHSpin ) );
	int v = gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_PatchDialog.m_pSegVSpin ) );
	gtk_widget_set_sensitive( g_PatchDialog.m_pSegHSpin, on );
	gtk_widget_set_sensitive( g_PatchDialog.m_pSegVSpin, on );
	Undo_Start( "set patch segments" );
	Undo_AddBrushList( &selected_brushes );
	Patch_SetSegmentsSelected( on ? h : 0, v );
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_UpdateWindows( W_ALL );
}

static void OnSegChanged( GtkWidget *widget, gpointer data ){
	SegApply();
}

static void OnDone( GtkWidget *widget, gpointer data ){
	g_PatchDialog.m_Patch = NULL;
	g_PatchDialog.HideDlg();
}

// memorize the current state (that is don't try to undo our do before changing something else)
static void OnApply( GtkWidget *widget, gpointer data ){
	g_PatchDialog.UpdateData( TRUE );
	if ( g_PatchDialog.m_Patch != NULL ) {
		int r = g_PatchDialog.m_nRow;
		int c = g_PatchDialog.m_nCol;
		if ( r >= 0 && r < g_PatchDialog.m_Patch->height && c >= 0 && c < g_PatchDialog.m_Patch->width ) {
			if ( g_PatchDialog.m_Patch->pShader ) {
				g_PatchDialog.m_Patch->pShader->DecRef();
			}
			if ( g_PatchDialog.m_strName.Find( ' ' ) >= 0 ) {
				Sys_FPrintf( SYS_WRN, "WARNING: spaces in shader names are not allowed, dropping '%s'\n", g_PatchDialog.m_strName.GetBuffer() );
				g_PatchDialog.m_strName = SHADER_NOT_FOUND;
			}
			g_PatchDialog.m_Patch->pShader = QERApp_Shader_ForName( g_PatchDialog.m_strName );
			g_PatchDialog.m_Patch->d_texture = g_PatchDialog.m_Patch->pShader->getTexture();
			g_PatchDialog.m_Patch->ctrl[c][r].xyz[0] = g_PatchDialog.m_fX;
			g_PatchDialog.m_Patch->ctrl[c][r].xyz[1] = g_PatchDialog.m_fY;
			g_PatchDialog.m_Patch->ctrl[c][r].xyz[2] = g_PatchDialog.m_fZ;
			g_PatchDialog.m_Patch->ctrl[c][r].st[0] = g_PatchDialog.m_fS;
			g_PatchDialog.m_Patch->ctrl[c][r].st[1] = g_PatchDialog.m_fT;
			g_PatchDialog.m_Patch->bDirty = true;
			Sys_UpdateWindows( W_ALL );
		}
	}
}

static void OnSelchangeComboColRow( GtkWidget *widget, gpointer data ){
	if ( !g_PatchDialog.m_bListenChanged ) {
		return;
	}
	// retrieve the current m_nRow and m_nCol, other params are not relevant
	// (NOTE: UpdateData has a mechanism to avoid inifinite looping)
	g_PatchDialog.UpdateData( TRUE );
	// read the changed values ourselves
	g_PatchDialog.UpdateRowColInfo();
	// now reflect our changes
	g_PatchDialog.UpdateData( FALSE );
}

static void OnBtnPatchdetails( GtkWidget *widget, gpointer data ){
	Patch_NaturalizeSelected( true );
	Sys_UpdateWindows( W_ALL );
}

static void OnBtnPatchfit( GtkWidget *widget, gpointer data ){
	Patch_ResetTexturing( 1.0, 1.0 );
	Sys_UpdateWindows( W_ALL );
}

static void OnBtnPatchnatural( GtkWidget *widget, gpointer data ){
	Patch_NaturalizeSelected();
	Sys_UpdateWindows( W_ALL );
}

static void OnBtnPatchreset( GtkWidget *widget, gpointer data ){
	float fx, fy;
	if ( DoTextureLayout( &fx, &fy ) == IDOK ) {
		Patch_ResetTexturing( fx, fy );
	}
	Sys_UpdateWindows( W_ALL );
}

static void OnSpinChanged( GtkAdjustment *adj, gpointer data ){
	texdef_t td;

	td.rotate = 0;
	td.scale[0] = td.scale[1] = 0;
	td.shift[0] = td.shift[1] = 0;
	td.contents = 0;
	td.flags = 0;
	td.value = 0;

	if ( gtk_adjustment_get_value( adj ) == 0 ) {
		return;
	}

	if ( adj == g_object_get_data( G_OBJECT( g_PatchDialog.GetWidget() ), "hshift_adj" ) ) {
		l_pPIIncrement->shift[0] = gtk_spin_button_get_value( GTK_SPIN_BUTTON( data ) );

		if ( gtk_adjustment_get_value( adj ) > 0 ) {
			td.shift[0] = l_pPIIncrement->shift[0];
		}
		else{
			td.shift[0] = -l_pPIIncrement->shift[0];
		}
	}
	else if ( adj == g_object_get_data( G_OBJECT( g_PatchDialog.GetWidget() ), "vshift_adj" ) ) {
		l_pPIIncrement->shift[1] = gtk_spin_button_get_value( GTK_SPIN_BUTTON( data ) );

		if ( gtk_adjustment_get_value( adj ) > 0 ) {
			td.shift[1] = l_pPIIncrement->shift[1];
		}
		else{
			td.shift[1] = -l_pPIIncrement->shift[1];
		}
	}
	else if ( adj == g_object_get_data( G_OBJECT( g_PatchDialog.GetWidget() ), "hscale_adj" ) ) {
		l_pPIIncrement->scale[0] = gtk_spin_button_get_value( GTK_SPIN_BUTTON( data ) );
		if ( l_pPIIncrement->scale[0] == 0.0f ) {
			return;
		}
		// make sure scale factor is always >1 for increases and <1 for decreases
		if ( gtk_adjustment_get_value( adj ) > 0 ) {
			if ( l_pPIIncrement->scale[0] < 1 ) {
				td.scale[0] = l_pPIIncrement->scale[0];
			}
			else{
				td.scale[0] = 1.0f / l_pPIIncrement->scale[0];
			}
		}
		else
		{
			if ( l_pPIIncrement->scale[0] < 1 ) {
				td.scale[0] = 1.0f / l_pPIIncrement->scale[0];
			}
			else{
				td.scale[0] = l_pPIIncrement->scale[0];
			}
		}
	}
	else if ( adj == g_object_get_data( G_OBJECT( g_PatchDialog.GetWidget() ), "vscale_adj" ) ) {
		l_pPIIncrement->scale[1] = gtk_spin_button_get_value( GTK_SPIN_BUTTON( data ) );
		if ( l_pPIIncrement->scale[1] == 0.0f ) {
			return;
		}
		// make sure scale factor is always >1 for increases and <1 for decreases
		if ( gtk_adjustment_get_value( adj ) > 0 ) {
			if ( l_pPIIncrement->scale[1] < 1 ) {
				td.scale[1] = l_pPIIncrement->scale[1];
			}
			else{
				td.scale[1] = 1.0f / l_pPIIncrement->scale[1];
			}
		}
		else
		{
			if ( l_pPIIncrement->scale[1] < 1 ) {
				td.scale[1] = 1.0f / l_pPIIncrement->scale[1];
			}
			else{
				td.scale[1] = l_pPIIncrement->scale[1];
			}
		}
	}
	else if ( adj == g_object_get_data( G_OBJECT( g_PatchDialog.GetWidget() ), "rotate_adj" ) ) {
		l_pPIIncrement->rotate = gtk_spin_button_get_value( GTK_SPIN_BUTTON( data ) );

		if ( gtk_adjustment_get_value( adj ) > 0 ) {
			td.rotate = l_pPIIncrement->rotate;
		}
		else{
			td.rotate = -l_pPIIncrement->rotate;
		}
	}

	gtk_adjustment_set_value( adj, 0 );

	// will scale shift rotate the patch accordingly
	Patch_SetTextureInfo( &td );
	// update the point-by-point view
	OnSelchangeComboColRow( NULL,NULL );
	Sys_UpdateWindows( W_CAMERA );
}

static gint OnDialogKey( GtkWidget* widget, GdkEventKey* event, gpointer data ){
	if ( event->keyval == GDK_KEY_Return ) {
		OnApply( NULL, NULL );
		return TRUE;
	}
	else if ( event->keyval == GDK_KEY_Escape ) {
		OnDone( NULL, NULL );
		return TRUE;
	}
	return FALSE;
}

// =============================================================================
// Global Functions

void DoPatchInspector(){
	// do we need to create the dialog?
	if ( g_PatchDialog.GetWidget() == NULL ) {
		g_PatchDialog.Create();
		g_PatchDialog.UpdateData( FALSE );
	}
	g_PatchDialog.GetPatchInfo();
	if ( !l_bIsActive ) {
		g_PatchDialog.ShowDlg();
	}
}

void UpdatePatchInspector(){
	if ( l_bIsActive ) {
		g_PatchDialog.GetPatchInfo();
	}
}

void TogglePatchInspector(){
	if ( l_bIsActive ) {
		OnDone( NULL,NULL );
	}
	else{
		DoPatchInspector();
	}
}

// =============================================================================
// PatchDialog class

PatchDialog::PatchDialog (){
	m_strName = "";
	m_fS = 0.0f;
	m_fT = 0.0f;
	m_fX = 0.0f;
	m_fY = 0.0f;
	m_fZ = 0.0f;
	m_nCol = 0;
	m_nRow = 0;
	m_pSegCheck = m_pSegHSpin = m_pSegVSpin = NULL;
	m_bSegListen = false;
	m_Patch = NULL;
	m_bListenChanged = true;
}

void PatchDialog::InitDefaultIncrement( texdef_t *tex ){
	tex->SetName( SHADER_NOT_FOUND );
	tex->scale[0] = 0.5f;
	tex->scale[1] = 0.5f;
	tex->rotate = 45;
	tex->shift[0] = 8.0f;
	tex->shift[1] = 8.0f;
}

// we plug into HideDlg and ShowDlg to maintain the l_bIsActive flag
void PatchDialog::HideDlg(){
	l_bIsActive = false;
	Dialog::HideDlg();
}

void PatchDialog::ShowDlg(){
	l_bIsActive = true;
	Dialog::ShowDlg();
}

void PatchDialog::BuildDialog(){
	GtkWidget *dlg, *vbox, *vbox2, *hbox, *hbox2, *frame, *table, *label;
	GtkWidget *button, *entry, *spin, *combo, *row_label, *col_label;
	GtkAdjustment *adj;
	GList *lst, *cells;
	GtkSizeGroup *size_group;
	GtkWidget *cap_button, *set_button, *nat_button, *fit_button;

	dlg = m_pWidget;

	load_window_pos( dlg, g_PrefsDlg.mWindowInfo.posPatchWnd );

	gtk_window_set_title( GTK_WINDOW( dlg ), _( "Patch Properties" ) );
	g_signal_connect( G_OBJECT( dlg ), "delete-event", G_CALLBACK( OnDone ), NULL );
	// catch 'Esc' and 'Enter'
	g_signal_connect( G_OBJECT( dlg ), "key-press-event", G_CALLBACK( OnDialogKey ), NULL );
	gtk_window_set_transient_for( GTK_WINDOW( dlg ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );


	vbox = gtk_vbox_new( FALSE, 5 );
	gtk_container_add( GTK_CONTAINER( dlg ), vbox );
	gtk_container_set_border_width( GTK_CONTAINER( vbox ), 5 );
	gtk_widget_show( vbox );

	hbox = gtk_hbox_new( FALSE, 5 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, TRUE, TRUE, 0 );
	gtk_widget_show( hbox );

	frame = gtk_frame_new( _( "Details" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), frame, TRUE, TRUE, 0 );
	gtk_widget_show( frame );

	vbox2 = gtk_vbox_new( FALSE, 5 );
	gtk_container_add( GTK_CONTAINER( frame ), vbox2 );
	gtk_container_set_border_width( GTK_CONTAINER( vbox2 ), 5 );
	gtk_widget_show( vbox2 );

	table = gtk_table_new( 2, 2, FALSE );
	gtk_box_pack_start( GTK_BOX( vbox2 ), table, TRUE, TRUE, 0 );
	gtk_table_set_row_spacings( GTK_TABLE( table ), 5 );
	gtk_table_set_col_spacings( GTK_TABLE( table ), 5 );
	gtk_widget_show( table );

	row_label = label = gtk_label_new( _( "Row:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 0, 1,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	col_label = label = gtk_label_new( _( "Column:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 1, 2, 0, 1,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	size_group = gtk_size_group_new( GTK_SIZE_GROUP_BOTH );
	gtk_size_group_add_widget( size_group, row_label );
	gtk_size_group_add_widget( size_group, col_label );
	g_object_unref( size_group );

	combo = gtk_combo_box_text_new();
	gtk_table_attach( GTK_TABLE( table ), combo, 0, 1, 1, 2,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_widget_show( combo );
	g_signal_connect( G_OBJECT( GTK_COMBO_BOX( combo ) ), "changed",
						G_CALLBACK( OnSelchangeComboColRow ), this );
	AddDialogData( combo, &m_nRow, DLG_COMBO_BOX_INT );
	m_pRowCombo = combo;
	cells = gtk_cell_layout_get_cells( GTK_CELL_LAYOUT( combo ) );
	for( lst = cells; lst != NULL; lst = g_list_next( lst ) )
	{
		g_object_set( G_OBJECT( lst->data ), "xalign", 1.0, (char*)NULL );
	}
	g_list_free( cells );

	combo = gtk_combo_box_text_new();
	gtk_table_attach( GTK_TABLE( table ), combo, 1, 2, 1, 2,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_widget_show( combo );
	g_signal_connect( G_OBJECT( GTK_COMBO_BOX( combo ) ), "changed",
						G_CALLBACK( OnSelchangeComboColRow ), this );
	AddDialogData( combo, &m_nCol, DLG_COMBO_BOX_INT );
	m_pColCombo = combo;
	cells = gtk_cell_layout_get_cells( GTK_CELL_LAYOUT( combo ) );
	for( lst = cells; lst != NULL; lst = g_list_next( lst ) )
	{
		g_object_set( G_OBJECT( lst->data ), "xalign", 1.0, (char*)NULL );
	}
	g_list_free( cells );

	table = gtk_table_new( 5, 2, FALSE );
	gtk_box_pack_start( GTK_BOX( vbox2 ), table, TRUE, TRUE, 0 );
	gtk_table_set_row_spacings( GTK_TABLE( table ), 5 );
	gtk_table_set_col_spacings( GTK_TABLE( table ), 5 );
	gtk_widget_show( table );

	label = gtk_label_new( _( "X:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 0, 1,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_set_tooltip_text( label, _( "X-Axis" ) );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Y:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 1, 2,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_set_tooltip_text( label, _( "Y-Axis" ) );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Z:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 2, 3,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_set_tooltip_text( label, _( "Z-Axis" ) );
	gtk_widget_show( label );

	label = gtk_label_new( _( "S:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 3, 4,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_set_tooltip_text( label, _( "S-coordinates correspond to the \"x\" coordinates on the texture itself" ) );
	gtk_widget_show( label );

	label = gtk_label_new( _( "T:" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 0, 1, 4, 5,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_set_tooltip_text( label, _( "T-coordinates correspond to the \"y\" coordinates on the texture itself. The measurements are in game units." ) );
	gtk_widget_show( label );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 0, 1,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	AddDialogData( spin, &m_fX, DLG_SPIN_FLOAT );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 1, 2,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	AddDialogData( spin, &m_fY, DLG_SPIN_FLOAT );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 2, 3,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	AddDialogData( spin, &m_fZ, DLG_SPIN_FLOAT );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 3, 4,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	AddDialogData( spin, &m_fS, DLG_SPIN_FLOAT );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 4, 5,
					  (GtkAttachOptions) ( GTK_EXPAND | GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	AddDialogData( spin, &m_fT, DLG_SPIN_FLOAT );

	frame = gtk_frame_new( _( "Texturing" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), frame, TRUE, TRUE, 0 );
	gtk_widget_show( frame );

	vbox2 = gtk_vbox_new( FALSE, 5 );
	gtk_container_add( GTK_CONTAINER( frame ), vbox2 );
	gtk_container_set_border_width( GTK_CONTAINER( vbox2 ), 5 );
	gtk_widget_show( vbox2 );

	label = gtk_label_new( _( "Name:" ) );
	gtk_box_pack_start( GTK_BOX( vbox2 ), label, TRUE, TRUE, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	entry = gtk_entry_new();
//  gtk_entry_set_editable (GTK_ENTRY (entry), false);
	gtk_box_pack_start( GTK_BOX( vbox2 ), entry, TRUE, TRUE, 0 );
	gtk_widget_show( entry );
	AddDialogData( entry, &m_strName, DLG_ENTRY_TEXT );

	table = gtk_table_new( 5, 3, FALSE );
	gtk_box_pack_start( GTK_BOX( vbox2 ), table, TRUE, TRUE, 0 );
	gtk_table_set_row_spacings( GTK_TABLE( table ), 5 );
	gtk_table_set_col_spacings( GTK_TABLE( table ), 5 );
	gtk_widget_show( table );

	label = gtk_label_new( _( "Horizontal Shift Step" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 2, 3, 0, 1,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Vertical Shift Step" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 2, 3, 1, 2,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Horizontal Stretch Step" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 2, 3, 2, 3,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Vertical Stretch Step" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 2, 3, 3, 4,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	label = gtk_label_new( _( "Rotate Step" ) );
	gtk_table_attach( GTK_TABLE( table ), label, 2, 3, 4, 5,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_misc_set_alignment( GTK_MISC( label ), 0.0, 0.5 );
	gtk_widget_show( label );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 0, 1, 0, 1,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), FALSE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	// we fill in this data, if no patch is selected the widgets are unmodified when the inspector is raised
	// so we need to have at least one initialisation somewhere
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( spin ), l_pPIIncrement->shift[0] );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -8192, 8192, 1, 1, 0 ) );
	g_signal_connect( adj, "value-changed", G_CALLBACK( OnSpinChanged ), spin );
	g_object_set_data( G_OBJECT( m_pWidget ), "hshift_adj", adj );

	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 0, 1,
					  (GtkAttachOptions) ( 0 ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 0, 1, 1, 2,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), FALSE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( spin ), l_pPIIncrement->shift[1] );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -8192, 8192, 1, 1, 0 ) );
	g_signal_connect( adj, "value-changed", G_CALLBACK( OnSpinChanged ), spin );
	g_object_set_data( G_OBJECT( m_pWidget ), "vshift_adj", adj );

	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 1, 2,
					  (GtkAttachOptions) ( 0 ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 0, 1, 2, 3,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), FALSE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( spin ), l_pPIIncrement->scale[0] );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -1000, 1000, 1, 1, 0 ) );
	g_signal_connect( adj, "value-changed", G_CALLBACK( OnSpinChanged ), spin );
	g_object_set_data( G_OBJECT( m_pWidget ), "hscale_adj", adj );

	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 2, 3,
					  (GtkAttachOptions) ( 0 ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -INT_MAX, INT_MAX, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 0, 1, 3, 4,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), FALSE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( spin ), l_pPIIncrement->scale[1] );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -1000, 1000, 1, 1, 0 ) );
	g_signal_connect( adj, "value-changed", G_CALLBACK( OnSpinChanged ), spin );
	g_object_set_data( G_OBJECT( m_pWidget ), "vscale_adj", adj );

	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 3, 4,
					  (GtkAttachOptions) ( 0 ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -360, 360, 1, 10, 0 ) );
	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 4 );
	gtk_table_attach( GTK_TABLE( table ), spin, 0, 1, 4, 5,
					  (GtkAttachOptions) ( GTK_FILL ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_spin_button_set_wrap( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( spin ),  l_pPIIncrement->rotate );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( 0, -1000, 1000, 1, 1, 0 ) ); // NOTE: Arnout - this really should be 360 but can't change it anymore as it could break existing maps
	g_signal_connect( adj, "value-changed", G_CALLBACK( OnSpinChanged ), spin );
	g_object_set_data( G_OBJECT( m_pWidget ), "rotate_adj", adj );

	spin = gtk_spin_button_new( GTK_ADJUSTMENT( adj ), 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( spin ), TRUE );
	gtk_table_attach( GTK_TABLE( table ), spin, 1, 2, 4, 5,
					  (GtkAttachOptions) ( 0 ),
					  (GtkAttachOptions) ( 0 ), 0, 0 );
	gtk_entry_set_alignment( GTK_ENTRY( spin ), 1.0 ); //right
	gtk_widget_show( spin );

	hbox2 = gtk_hbox_new( FALSE, 5 );
	gtk_box_set_homogeneous( GTK_BOX( hbox2 ), TRUE );
	gtk_box_pack_start( GTK_BOX( vbox2 ), hbox2, TRUE, FALSE, 0 );
	gtk_widget_show( hbox2 );

	cap_button = button = gtk_button_new_with_label( _( "CAP" ) );
	gtk_box_pack_end( GTK_BOX( hbox2 ), button, TRUE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnBtnPatchdetails ), NULL );

	set_button = button = gtk_button_new_with_label( _( "Set..." ) );
	gtk_box_pack_end( GTK_BOX( hbox2 ), button, TRUE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnBtnPatchreset ), NULL );

	nat_button = button = gtk_button_new_with_label( _( "Natural" ) );
	gtk_box_pack_end( GTK_BOX( hbox2 ), button, TRUE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnBtnPatchnatural ), NULL );

	fit_button = button = gtk_button_new_with_label( _( "Fit" ) );
	gtk_box_pack_end( GTK_BOX( hbox2 ), button, TRUE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnBtnPatchfit ), NULL );

	size_group = gtk_size_group_new( GTK_SIZE_GROUP_BOTH );
	gtk_size_group_add_widget( size_group, cap_button );
	gtk_size_group_add_widget( size_group, set_button );
	gtk_size_group_add_widget( size_group, nat_button );
	gtk_size_group_add_widget( size_group, fit_button );
	g_object_unref( size_group );

	// segment counts of segmented patches
	frame = gtk_frame_new( _( "Segments (segmented patches)" ) );
	gtk_box_pack_start( GTK_BOX( vbox ), frame, TRUE, FALSE, 0 );
	gtk_widget_show( frame );

	hbox2 = gtk_hbox_new( FALSE, 5 );
	gtk_container_add( GTK_CONTAINER( frame ), hbox2 );
	gtk_container_set_border_width( GTK_CONTAINER( hbox2 ), 5 );
	gtk_widget_show( hbox2 );

	m_pSegCheck = gtk_check_button_new_with_label( _( "Segmented" ) );
	gtk_box_pack_start( GTK_BOX( hbox2 ), m_pSegCheck, FALSE, FALSE, 0 );
	gtk_widget_show( m_pSegCheck );
	g_signal_connect( G_OBJECT( m_pSegCheck ), "toggled", G_CALLBACK( OnSegChanged ), NULL );

	label = gtk_label_new( _( "Horizontal:" ) );
	gtk_box_pack_start( GTK_BOX( hbox2 ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( SEGPATCH_DEFAULT_H, SEGPATCH_MIN_H, SEGPATCH_MAX_SEGS, 1, 4, 0 ) );
	m_pSegHSpin = gtk_spin_button_new( adj, 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( m_pSegHSpin ), TRUE );
	gtk_box_pack_start( GTK_BOX( hbox2 ), m_pSegHSpin, FALSE, FALSE, 0 );
	gtk_widget_show( m_pSegHSpin );
	g_signal_connect( G_OBJECT( m_pSegHSpin ), "value-changed", G_CALLBACK( OnSegChanged ), NULL );

	label = gtk_label_new( _( "Vertical:" ) );
	gtk_box_pack_start( GTK_BOX( hbox2 ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );

	adj = GTK_ADJUSTMENT( gtk_adjustment_new( SEGPATCH_DEFAULT_V, SEGPATCH_MIN_V, SEGPATCH_MAX_SEGS, 1, 4, 0 ) );
	m_pSegVSpin = gtk_spin_button_new( adj, 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( m_pSegVSpin ), TRUE );
	gtk_box_pack_start( GTK_BOX( hbox2 ), m_pSegVSpin, FALSE, FALSE, 0 );
	gtk_widget_show( m_pSegVSpin );
	g_signal_connect( G_OBJECT( m_pSegVSpin ), "value-changed", G_CALLBACK( OnSegChanged ), NULL );

	m_bSegListen = true;
	gtk_widget_set_sensitive( m_pSegHSpin, FALSE );
	gtk_widget_set_sensitive( m_pSegVSpin, FALSE );

	hbox = gtk_hbox_new( FALSE, 5 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, TRUE, FALSE, 0 );
	gtk_widget_show( hbox );

	button = gtk_button_new_with_label( _( "Done" ) );
	gtk_box_pack_end( GTK_BOX( hbox ), button, FALSE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnDone ), NULL );

	button = gtk_button_new_with_label( _( "Apply" ) );
	gtk_box_pack_end( GTK_BOX( hbox ), button, FALSE, FALSE, 0 );
	gtk_widget_show( button );
	g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnApply ), NULL );
}

// sync the dialog our internal data structures
void PatchDialog::UpdateData( bool retrieve ){
	if ( m_pWidget == NULL ) {
		return;
	}

	m_bListenChanged = false;
	Dialog::UpdateData( retrieve );
	m_bListenChanged = true;
}

// show the segment counts of the selected patch(es)
void PatchDialog::UpdateSegInfo(){
	if ( m_pSegCheck == NULL ) {
		return;
	}
	int h = 0, v = 0;
	bool have = Patch_GetSegmentsSelected( &h, &v );
	m_bSegListen = false;
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( m_pSegCheck ), have && h > 0 );
	if ( have && h > 0 ) {
		gtk_spin_button_set_range( GTK_SPIN_BUTTON( m_pSegHSpin ), Patch_SegMinHSelected(), SEGPATCH_MAX_SEGS );
		gtk_spin_button_set_value( GTK_SPIN_BUTTON( m_pSegHSpin ), h );
		gtk_spin_button_set_value( GTK_SPIN_BUTTON( m_pSegVSpin ), v );
	}
	gtk_widget_set_sensitive( m_pSegCheck, have ? TRUE : FALSE );
	gtk_widget_set_sensitive( m_pSegHSpin, ( have && h > 0 ) ? TRUE : FALSE );
	gtk_widget_set_sensitive( m_pSegVSpin, ( have && h > 0 ) ? TRUE : FALSE );
	m_bSegListen = true;
}

// read the map and feed in the stuff to the dialog box
void PatchDialog::GetPatchInfo(){
	UpdateSegInfo();
	m_Patch = SinglePatchSelected();
	if ( m_Patch != NULL ) {
		gchar buffer[11];
		int i;
		GtkListStore *store;

		m_strName = m_Patch->pShader->getName();

		// fill in the numbers for Row / Col selection
		m_bListenChanged = false;

#if GTK_CHECK_VERSION( 3, 0, 0 )
		gtk_combo_box_text_remove_all( GTK_COMBO_BOX_TEXT( m_pRowCombo ) );
#else
		store = GTK_LIST_STORE( gtk_combo_box_get_model( GTK_COMBO_BOX( m_pRowCombo ) ) );
		gtk_list_store_clear( store );
#endif

		for ( i = 0; i < m_Patch->height; i++ )
		{
			g_snprintf( buffer, sizeof( buffer ), "%i", i );
			gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( m_pRowCombo ), buffer );
		}
		gtk_combo_box_set_active( GTK_COMBO_BOX( GTK_COMBO_BOX_TEXT( m_pRowCombo ) ), 0 );

		for ( i = 0; i < m_Patch->width; i++ )
		{
			g_snprintf( buffer, sizeof( buffer ), "%i", i );
			gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( m_pColCombo ), buffer );
		}
		gtk_combo_box_set_active( GTK_COMBO_BOX( GTK_COMBO_BOX_TEXT( m_pColCombo ) ), 0 );

		m_bListenChanged = true;

	}
	else{
		Sys_FPrintf( SYS_WRN, "WARNING: No patch selected.\n" );
	}
	// fill in our internal structs
	m_nRow = 0; m_nCol = 0;
	UpdateRowColInfo();
	// now update the dialog box
	UpdateData( false );
}

// read the current patch on map and initialize m_fX m_fY accordingly
// NOTE: don't call UpdateData in there, it's not meant for
void PatchDialog::UpdateRowColInfo(){
	m_fX = m_fY = m_fZ = m_fS = m_fT = 0.0;

	if ( m_Patch != NULL ) {
		// we rely on whatever active row/column has been set before we get called
		int r = m_nRow;
		int c = m_nCol;
		if ( r >= 0 && r < m_Patch->height && c >= 0 && c < m_Patch->width ) {
			m_fX = m_Patch->ctrl[c][r].xyz[0];
			m_fY = m_Patch->ctrl[c][r].xyz[1];
			m_fZ = m_Patch->ctrl[c][r].xyz[2];
			m_fS = m_Patch->ctrl[c][r].st[0];
			m_fT = m_Patch->ctrl[c][r].st[1];
		}
	}
}


// =============================================================================
// Floating window for the segment counts: shows up by itself while a segmented patch is selected and
// goes away again when none is. Watching the selection from a timer instead of hooking every place
// the selection can change keeps it right for all of them.

// =============================================================================
// Where the floating panels (segments, bevel, cog, cone) show up: stacked down the right edge of the grid
// window, each in the first free place from the top, so that panels that are open together never cover each other.
// A panel the player moved comes back where it was left.

struct PanelSlot_t
{
	GtkWidget *w;
	int ax, ay;     // where it was put when it was shown
	bool user;      // it was shown where the player had left it
};
static PanelSlot_t g_panelSlots[8];
static int g_panelSlotCount = 0;

static PanelSlot_t *PanelSlot_Get( GtkWidget *w ){
	for ( int i = 0; i < g_panelSlotCount; i++ )
		if ( g_panelSlots[i].w == w ) {
			return &g_panelSlots[i];
		}
	if ( g_panelSlotCount >= 8 ) {
		return &g_panelSlots[7];
	}
	PanelSlot_t *s = &g_panelSlots[g_panelSlotCount++];
	s->w = w;
	s->ax = s->ay = 0;
	s->user = false;
	return s;
}

// x, y: remembered position of a panel the player moved (-1: not moved, put it in the stack)
static void PanelStack_Show( GtkWidget *w, int *x, int *y ){
	if ( gtk_widget_get_visible( w ) ) {
		return;
	}
	PanelSlot_t *s = PanelSlot_Get( w );
	if ( *x >= 0 ) {
		s->user = true;
		s->ax = *x;
		s->ay = *y;
		gtk_window_move( GTK_WINDOW( w ), *x, *y );
		gtk_widget_show( w );
		return;
	}

	// the area to stack in: the grid window, else the main window
	gint left, top, right, bottom;
	GtkWidget *grid = g_pParentWnd->GetXYWnd() ? g_pParentWnd->GetXYWnd()->GetWidget() : NULL;
	GdkWindow *gw = grid ? gtk_widget_get_window( grid ) : NULL;
	if ( gw ) {
		gint ox = 0, oy = 0;
		GtkAllocation a;
		gdk_window_get_origin( gw, &ox, &oy );
		gtk_widget_get_allocation( grid, &a );
		left = ox;
		top = oy;
		right = ox + a.width;
		bottom = oy + a.height;
	}
	else
	{
		gint mx = 0, my = 0, mw = 0, mh = 0;
		gtk_window_get_position( GTK_WINDOW( g_pParentWnd->m_pWidget ), &mx, &my );
		gtk_window_get_size( GTK_WINDOW( g_pParentWnd->m_pWidget ), &mw, &mh );
		left = mx;
		top = my + 100;
		right = mx + mw - 20;
		bottom = my + mh;
	}

	// size with the window frame (the panels cannot be resized, so the request is the size)
	GtkRequisition req;
	gtk_widget_size_request( w, &req );
	const int pw = req.width + 16, ph = req.height + 40;
	const int gap = 6;

	int col = 0, px = 0, py = 0;
	for (;;)
	{
		px = right - 8 - pw - col * ( pw + gap );
		py = top + 8;
		for ( int pass = 0; pass < 32; pass++ )
		{
			bool pushed = false;
			for ( int i = 0; i < g_panelSlotCount; i++ )
			{
				GtkWidget *o = g_panelSlots[i].w;
				GdkWindow *ow = gtk_widget_get_window( o );
				if ( o == w || !ow || !gtk_widget_get_visible( o ) ) {
					continue;
				}
				GdkRectangle f;
				gdk_window_get_frame_extents( ow, &f );
				if ( px < f.x + f.width + gap && px + pw + gap > f.x && py < f.y + f.height + gap && py + ph + gap > f.y ) {
					py = f.y + f.height + gap;
					pushed = true;
				}
			}
			if ( !pushed ) {
				break;
			}
		}
		if ( py + ph <= bottom || px - ( pw + gap ) < left || col >= 3 ) {
			break;
		}
		col++; // no room left below the others: a new column to the left
	}
	s->user = false;
	s->ax = px;
	s->ay = py;
	gtk_window_move( GTK_WINDOW( w ), px, py );
	gtk_widget_show( w );
}

static void PanelStack_Hide( GtkWidget *w, int *x, int *y ){
	if ( !gtk_widget_get_visible( w ) ) {
		return;
	}
	PanelSlot_t *s = PanelSlot_Get( w );
	int cx = 0, cy = 0;
	gtk_window_get_position( GTK_WINDOW( w ), &cx, &cy );
	if ( s->user || abs( cx - s->ax ) > 8 || abs( cy - s->ay ) > 8 ) {
		*x = cx; // the player put it somewhere: it comes back there
		*y = cy;
	}
	else
	{
		*x = *y = -1; // still where the stack put it: the stack decides again
	}
	gtk_widget_hide( w );
}

static GtkWidget *g_segPanel = NULL;
static GtkWidget *g_segPanelH = NULL;
static GtkWidget *g_segPanelV = NULL;
static bool g_segPanelListen = false;
static int g_segPanelX = -1, g_segPanelY = -1;

static void OnSegPanelChanged( GtkWidget *widget, gpointer data ){
	if ( !g_segPanelListen ) {
		return;
	}
	int h = gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_segPanelH ) );
	int v = gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_segPanelV ) );
	Undo_Start( "set patch segments" );
	Undo_AddBrushList( &selected_brushes );
	Patch_SetSegmentsSelected( h, v );
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	Sys_UpdateWindows( W_ALL );
	g_PatchDialog.UpdateSegInfo(); // keep the Patch Properties window in step, if it is open
}

// The panel never takes the keyboard by itself: it only accepts focus while the pointer is over it,
// and hands the focus back to the main window as soon as the pointer leaves.
static gboolean OnSegPanelEnter( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	gtk_window_set_accept_focus( GTK_WINDOW( g_segPanel ), TRUE );
	return FALSE;
}

static gboolean OnSegPanelLeave( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	if ( event->detail == GDK_NOTIFY_INFERIOR ) {
		return FALSE; // only moved onto one of the boxes inside the panel
	}
	if ( gtk_window_is_active( GTK_WINDOW( g_segPanel ) ) ) {
		gtk_window_present( GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	}
	gtk_window_set_accept_focus( GTK_WINDOW( g_segPanel ), FALSE );
	return FALSE;
}

static GtkWidget *g_seatCutBox = NULL;      // "Cut": the selected patch is a seat cutter (see seatcut.h)
static GtkWidget *g_seatCanBox = NULL;      // "Can be cut": cutters may cut holes into it

static void OnSeatBoxToggled( GtkWidget *widget, gpointer data ){
	if ( !g_segPanelListen ) {
		return;
	}
	const bool on = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( widget ) ) ? true : false;
	const bool cutBox = ( widget == g_seatCutBox );
	Undo_Start( cutBox ? "seat cutter" : "seat cut: can be cut" );
	Undo_AddBrushList( &selected_brushes );
	if ( cutBox ) {
		Seat_SetCutterSelected( on );
	}
	else{
		Seat_SetCanBeCutSelected( on );
	}
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
}

static GtkWidget *g_seatExtractBtn = NULL;  // "Extract piece": keep what the cutter takes out of the hosts as patches

static void OnSeatExtract( GtkWidget *widget, gpointer data ){
	Undo_Start( "extract cut piece" );
	Seat_ExtractPiece();
	Undo_End();
}

static void SegPanelBuild(){	GtkWidget *hbox, *label, *vbox, *row2;
	GtkAdjustment *adj;

	g_segPanel = gtk_window_new( GTK_WINDOW_TOPLEVEL );
	gtk_window_set_title( GTK_WINDOW( g_segPanel ), _( "Segments" ) );
	gtk_window_set_transient_for( GTK_WINDOW( g_segPanel ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	gtk_window_set_type_hint( GTK_WINDOW( g_segPanel ), GDK_WINDOW_TYPE_HINT_UTILITY );
	gtk_window_set_skip_taskbar_hint( GTK_WINDOW( g_segPanel ), TRUE );
	gtk_window_set_focus_on_map( GTK_WINDOW( g_segPanel ), FALSE ); // appearing must not take the keyboard from the views
	gtk_window_set_accept_focus( GTK_WINDOW( g_segPanel ), FALSE );
	gtk_window_set_resizable( GTK_WINDOW( g_segPanel ), FALSE );
	gtk_window_set_deletable( GTK_WINDOW( g_segPanel ), FALSE );

	vbox = gtk_vbox_new( FALSE, 2 );
	gtk_container_add( GTK_CONTAINER( g_segPanel ), vbox );
	gtk_widget_show( vbox );

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_container_set_border_width( GTK_CONTAINER( hbox ), 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );

	// second row: the seat cut switches
	row2 = gtk_hbox_new( FALSE, 12 );
	gtk_container_set_border_width( GTK_CONTAINER( row2 ), 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), row2, FALSE, FALSE, 0 );
	gtk_widget_show( row2 );
	g_seatCutBox = gtk_check_button_new_with_label( _( "Cut holes in others" ) );
	gtk_widget_set_tooltip_text( g_seatCutBox, _( "Make this patch a cutter: every segmented patch it sits in gets a hole the shape of it, and follows it when it moves" ) );
	gtk_box_pack_start( GTK_BOX( row2 ), g_seatCutBox, FALSE, FALSE, 0 );
	gtk_widget_show( g_seatCutBox );
	g_seatCanBox = gtk_check_button_new_with_label( _( "Can be cut" ) );
	gtk_widget_set_tooltip_text( g_seatCanBox, _( "Off: no cutter makes a hole in this patch" ) );
	gtk_box_pack_start( GTK_BOX( row2 ), g_seatCanBox, FALSE, FALSE, 0 );
	gtk_widget_show( g_seatCanBox );
	g_seatExtractBtn = gtk_button_new_with_label( _( "Extract piece" ) );
	gtk_widget_set_tooltip_text( g_seatExtractBtn, _( "Keep what this cutter takes out of the patches it sits in: it becomes patches of their own (a group) that can be moved, say as a hatch. The hole stays" ) );
	gtk_box_pack_start( GTK_BOX( row2 ), g_seatExtractBtn, FALSE, FALSE, 0 );
	gtk_widget_show( g_seatExtractBtn );
	g_signal_connect( G_OBJECT( g_seatExtractBtn ), "clicked", G_CALLBACK( OnSeatExtract ), NULL );
	g_signal_connect( G_OBJECT( g_seatCutBox ), "toggled", G_CALLBACK( OnSeatBoxToggled ), NULL );
	g_signal_connect( G_OBJECT( g_seatCanBox ), "toggled", G_CALLBACK( OnSeatBoxToggled ), NULL );

	label = gtk_label_new( _( "Horizontal" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );
	adj = GTK_ADJUSTMENT( gtk_adjustment_new( SEGPATCH_DEFAULT_H, SEGPATCH_MIN_H, SEGPATCH_MAX_SEGS, 1, 4, 0 ) );
	g_segPanelH = gtk_spin_button_new( adj, 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( g_segPanelH ), TRUE );
	gtk_box_pack_start( GTK_BOX( hbox ), g_segPanelH, FALSE, FALSE, 0 );
	gtk_widget_show( g_segPanelH );

	label = gtk_label_new( _( "Vertical" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );
	adj = GTK_ADJUSTMENT( gtk_adjustment_new( SEGPATCH_DEFAULT_V, SEGPATCH_MIN_V, SEGPATCH_MAX_SEGS, 1, 4, 0 ) );
	g_segPanelV = gtk_spin_button_new( adj, 1, 0 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( g_segPanelV ), TRUE );
	gtk_box_pack_start( GTK_BOX( hbox ), g_segPanelV, FALSE, FALSE, 0 );
	gtk_widget_show( g_segPanelV );

	gtk_widget_add_events( g_segPanel, GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK );
	g_signal_connect( G_OBJECT( g_segPanel ), "enter-notify-event", G_CALLBACK( OnSegPanelEnter ), NULL );
	g_signal_connect( G_OBJECT( g_segPanel ), "leave-notify-event", G_CALLBACK( OnSegPanelLeave ), NULL );
	g_signal_connect( G_OBJECT( g_segPanelH ), "value-changed", G_CALLBACK( OnSegPanelChanged ), NULL );
	g_signal_connect( G_OBJECT( g_segPanelV ), "value-changed", G_CALLBACK( OnSegPanelChanged ), NULL );
}

static gboolean SegPanelTick( gpointer data ){
	if ( g_pParentWnd == NULL || g_pParentWnd->m_pWidget == NULL ) {
		return TRUE;
	}
	int h = 0, v = 0;
	bool want = Patch_GetSegmentedSelected( &h, &v );

	if ( want && g_segPanel == NULL ) {
		SegPanelBuild();
	}
	if ( g_segPanel == NULL ) {
		return TRUE;
	}

	if ( want ) {
		// show what the selected patch has, unless the player is typing in the box right now
		g_segPanelListen = false;
		gtk_spin_button_set_range( GTK_SPIN_BUTTON( g_segPanelH ), Patch_SegMinHSelected(), SEGPATCH_MAX_SEGS );
		if ( !gtk_widget_has_focus( g_segPanelH ) && gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_segPanelH ) ) != h ) {
			gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_segPanelH ), h );
		}
		if ( !gtk_widget_has_focus( g_segPanelV ) && gtk_spin_button_get_value_as_int( GTK_SPIN_BUTTON( g_segPanelV ) ) != v ) {
			gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_segPanelV ), v );
		}
		bool isCutter = false, canBeCut = false;
		if ( Seat_GetSelected( &isCutter, &canBeCut ) ) {
			gtk_widget_set_sensitive( g_seatExtractBtn, isCutter ? TRUE : FALSE );
			if ( ( gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_seatCutBox ) ) ? true : false ) != isCutter ) {
				gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( g_seatCutBox ), isCutter ? TRUE : FALSE );
			}
			if ( ( gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_seatCanBox ) ) ? true : false ) != canBeCut ) {
				gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( g_seatCanBox ), canBeCut ? TRUE : FALSE );
			}
		}
		g_segPanelListen = true;

		if ( !gtk_widget_get_visible( g_segPanel ) ) {
			PanelStack_Show( g_segPanel, &g_segPanelX, &g_segPanelY );
		}
	}
	else if ( gtk_widget_get_visible( g_segPanel ) ) {
		PanelStack_Hide( g_segPanel, &g_segPanelX, &g_segPanelY );
	}
	return TRUE;
}


// =============================================================================
// Floating window for the bevel: shows up by itself while a patch edge is picked (Alt+click). The
// subdivisions are a slider, the width has presets that are portions of the most the edge allows.

static GtkWidget *g_bevelPanel = NULL;
static GtkWidget *g_bevelSubdiv = NULL;
static GtkWidget *g_bevelWidth = NULL;
static GtkWidget *g_bevelInfo = NULL;
static GtkWidget *g_bevelSnapBox = NULL;
static const float g_bevelSnapValues[] = { 0.0f, 0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f };
static GtkWidget *g_bevelButton = NULL;
static GtkWidget *g_bevelRemoveButton = NULL;
static bool g_bevelListen = false;
static float g_bevelFraction = 0.5f;    // 0 = the width was typed in, otherwise a portion of the maximum
static float g_bevelLastMax = -1.0f;
static bool g_bevelLastBridge = false;
static float g_bevelMax = 0;            // the most the picked edge, or the bevel being edited, allows
static int g_bevelMode = 0;             // 0 hidden, 1 edges are picked (new bevel), 2 an existing bevel is selected
static brush_t *g_bevelEditStrip = NULL;
static int g_bevelX = -1, g_bevelY = -1;

void BevelPanel_Apply(){
	if ( g_bevelPanel == NULL || BevelEdge_Count() == 0 ) {
		Sys_Printf( "Bevel Edge: Alt+click a patch edge first (Ctrl+Alt+click adds more), or select a bevel to edit it.\n" );
		return;
	}
	const float width = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_bevelWidth ) );
	const int n = (int)gtk_range_get_value( GTK_RANGE( g_bevelSubdiv ) );
	Undo_Start( "bevel edge" );
	const int done = BevelEdges_Apply( width, n );
	Undo_End();
	Sys_Printf( "Bevel Edge: %d edge(s) beveled.\n", done );
	Sys_UpdateWindows( W_ALL );
}

// editing an existing bevel: every change is applied at once
static void BevelPanelLive(){
	if ( g_bevelMode != 2 || !g_bevelListen ) {
		return;
	}
	const float width = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_bevelWidth ) );
	const int n = (int)gtk_range_get_value( GTK_RANGE( g_bevelSubdiv ) );
	if ( n < 1 ) {
		BevelRec_Remove(); // no subdivisions: back to how it was before the bevel
	}
	else
	{
		BevelRec_Edit( width, n );
	}
}

static void OnBevelApply( GtkWidget *widget, gpointer data ){
	BevelPanel_Apply();
}

static void OnBevelRemove( GtkWidget *widget, gpointer data ){
	if ( g_bevelMode == 2 ) {
		BevelRec_Remove();
	}
}

static void OnBevelPreset( GtkWidget *widget, gpointer data ){
	const float f = *(const float*)data;
	if ( g_bevelMax <= 0 ) {
		return;
	}
	g_bevelFraction = f;
	g_bevelListen = false;
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_bevelWidth ), g_bevelMax * f );
	g_bevelListen = true;
	BevelPanelLive();
}

static void OnBevelWidthChanged( GtkWidget *widget, gpointer data ){
	if ( g_bevelListen ) {
		g_bevelFraction = 0; // typed in: stop following the maximum
		BevelPanelLive();
	}
}

static void OnBevelSubdivChanged( GtkWidget *widget, gpointer data ){
	BevelPanelLive();
}

static void OnBevelSnapChanged( GtkWidget *widget, gpointer data ){
	const int i = gtk_combo_box_get_active( GTK_COMBO_BOX( g_bevelSnapBox ) );
	if ( i >= 0 && i < (int)( sizeof( g_bevelSnapValues ) / sizeof( g_bevelSnapValues[0] ) ) ) {
		Bevel_SetSnap( g_bevelSnapValues[i] );
	}
	BevelPanelLive(); // a bevel that is being edited is cut again on the new grid
}

static gboolean OnBevelEnter( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	gtk_window_set_accept_focus( GTK_WINDOW( g_bevelPanel ), TRUE );
	return FALSE;
}

static gboolean OnBevelLeave( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	if ( event->detail == GDK_NOTIFY_INFERIOR ) {
		return FALSE;
	}
	if ( gtk_window_is_active( GTK_WINDOW( g_bevelPanel ) ) ) {
		gtk_window_present( GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	}
	gtk_window_set_accept_focus( GTK_WINDOW( g_bevelPanel ), FALSE );
	return FALSE;
}

static const float g_bevelPresets[4] = { 0.25f, 0.5f, 0.75f, 1.0f };

static void BevelPanelBuild(){
	GtkWidget *vbox, *hbox, *label, *button;
	static const char *names[4] = { "1/4", "1/2", "3/4", "Max" };

	g_bevelListen = true;
	g_bevelPanel = gtk_window_new( GTK_WINDOW_TOPLEVEL );
	gtk_window_set_title( GTK_WINDOW( g_bevelPanel ), _( "Bevel Edge" ) );
	gtk_window_set_transient_for( GTK_WINDOW( g_bevelPanel ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	gtk_window_set_type_hint( GTK_WINDOW( g_bevelPanel ), GDK_WINDOW_TYPE_HINT_UTILITY );
	gtk_window_set_skip_taskbar_hint( GTK_WINDOW( g_bevelPanel ), TRUE );
	gtk_window_set_focus_on_map( GTK_WINDOW( g_bevelPanel ), FALSE );
	gtk_window_set_accept_focus( GTK_WINDOW( g_bevelPanel ), FALSE );
	gtk_window_set_resizable( GTK_WINDOW( g_bevelPanel ), FALSE );
	gtk_window_set_deletable( GTK_WINDOW( g_bevelPanel ), FALSE );

	vbox = gtk_vbox_new( FALSE, 6 );
	gtk_container_set_border_width( GTK_CONTAINER( vbox ), 6 );
	gtk_container_add( GTK_CONTAINER( g_bevelPanel ), vbox );
	gtk_widget_show( vbox );

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	label = gtk_label_new( _( "Subdivisions" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );
	g_bevelSubdiv = gtk_hscale_new_with_range( 1, SEGPATCH_MAX_SEGS, 1 );
	gtk_scale_set_digits( GTK_SCALE( g_bevelSubdiv ), 0 );
	gtk_scale_set_value_pos( GTK_SCALE( g_bevelSubdiv ), GTK_POS_RIGHT );
	gtk_range_set_value( GTK_RANGE( g_bevelSubdiv ), 4 );
	gtk_widget_set_tooltip_text( g_bevelSubdiv, _( "While editing a bevel, 0 removes it" ) );
	gtk_widget_set_size_request( g_bevelSubdiv, 220, -1 );
	gtk_box_pack_start( GTK_BOX( hbox ), g_bevelSubdiv, TRUE, TRUE, 0 );
	gtk_widget_show( g_bevelSubdiv );

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	label = gtk_label_new( _( "Width" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );
	g_bevelWidth = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( 8, 0.25, 4096, 0.25, 4, 0 ) ), 0.25, 2 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( g_bevelWidth ), TRUE );
	gtk_box_pack_start( GTK_BOX( hbox ), g_bevelWidth, FALSE, FALSE, 0 );
	gtk_widget_show( g_bevelWidth );
	for ( int i = 0; i < 4; i++ )
	{
		button = gtk_button_new_with_label( names[i] );
		g_signal_connect( G_OBJECT( button ), "clicked", G_CALLBACK( OnBevelPreset ), (gpointer)&g_bevelPresets[i] );
		gtk_box_pack_start( GTK_BOX( hbox ), button, FALSE, FALSE, 0 );
		gtk_widget_show( button );
	}

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	label = gtk_label_new( _( "Snap to grid" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), label, FALSE, FALSE, 0 );
	gtk_widget_show( label );
	g_bevelSnapBox = gtk_combo_box_new_text();
	{
		static const char *snapNames[] = { "Off", "0.125", "0.25", "0.5", "1", "2", "4", "8" };
		int active = 0;
		for ( int i = 0; i < 8; i++ )
		{
			gtk_combo_box_append_text( GTK_COMBO_BOX( g_bevelSnapBox ), snapNames[i] );
			if ( fabsf( g_bevelSnapValues[i] - Bevel_GetSnap() ) < 1e-6f ) {
				active = i;
			}
		}
		gtk_combo_box_set_active( GTK_COMBO_BOX( g_bevelSnapBox ), active );
	}
	gtk_widget_set_tooltip_text( g_bevelSnapBox, _( "Put the points where the bevel meets the two patches on this grid, so that patches made on the grid afterwards (a new cylinder or cap on an open bevel) meet it exactly with no gap. Off leaves them where the cut puts them, which keeps circles exact but is usually off the grid. A bevel that is being edited is cut again with the choice made here" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_bevelSnapBox, FALSE, FALSE, 0 );
	gtk_widget_show( g_bevelSnapBox );
	g_signal_connect( G_OBJECT( g_bevelSnapBox ), "changed", G_CALLBACK( OnBevelSnapChanged ), NULL );

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	g_bevelInfo = gtk_label_new( "" );
	gtk_misc_set_alignment( GTK_MISC( g_bevelInfo ), 0.0, 0.5 );
	gtk_box_pack_start( GTK_BOX( hbox ), g_bevelInfo, TRUE, TRUE, 0 );
	gtk_widget_show( g_bevelInfo );
	g_bevelButton = gtk_button_new_with_label( _( "Bevel" ) );
	g_signal_connect( G_OBJECT( g_bevelButton ), "clicked", G_CALLBACK( OnBevelApply ), NULL );
	gtk_box_pack_end( GTK_BOX( hbox ), g_bevelButton, FALSE, FALSE, 0 );
	gtk_widget_show( g_bevelButton );
	g_bevelRemoveButton = gtk_button_new_with_label( _( "Remove bevel" ) );
	gtk_widget_set_tooltip_text( g_bevelRemoveButton, _( "Put the patches back as they were before the bevel (the same as setting the subdivisions to 0)" ) );
	g_signal_connect( G_OBJECT( g_bevelRemoveButton ), "clicked", G_CALLBACK( OnBevelRemove ), NULL );
	gtk_box_pack_end( GTK_BOX( hbox ), g_bevelRemoveButton, FALSE, FALSE, 0 );
	gtk_widget_set_no_show_all( g_bevelRemoveButton, TRUE );

	gtk_widget_add_events( g_bevelPanel, GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK );
	g_signal_connect( G_OBJECT( g_bevelPanel ), "enter-notify-event", G_CALLBACK( OnBevelEnter ), NULL );
	g_signal_connect( G_OBJECT( g_bevelPanel ), "leave-notify-event", G_CALLBACK( OnBevelLeave ), NULL );
	g_signal_connect( G_OBJECT( g_bevelWidth ), "value-changed", G_CALLBACK( OnBevelWidthChanged ), NULL );
	g_signal_connect( G_OBJECT( g_bevelSubdiv ), "value-changed", G_CALLBACK( OnBevelSubdivChanged ), NULL );
}

static gboolean BevelPanelTick( gpointer data ){
	if ( g_pParentWnd == NULL || g_pParentWnd->m_pWidget == NULL ) {
		return TRUE;
	}
	// what the panel is for right now: picked edges (a new bevel) or a selected bevel (edit it)
	int mode = 0, editN = 0;
	bool editBridge = false; // the selected strip lies between two loops (nothing was cut): only its subdivisions can be set
	bool bridge = false; // picked edges are two loops with nothing between them: Bevel makes a strip between them
	float maxW = 0, editW = 0;
	brush_t *strip = NULL;
	const int count = BevelEdge_Count();
	if ( count > 0 ) {
		mode = 1;
		maxW = BevelEdge_MaxWidth();
		bridge = maxW <= 0 && BevelEdge_BridgeCase();
	}
	else
	{
		strip = BevelRec_Selected( &editW, &maxW, &editN, &editBridge );
		if ( strip ) {
			mode = 2;
		}
	}
	g_bevelMax = maxW;

	if ( mode && g_bevelPanel == NULL ) {
		BevelPanelBuild();
	}
	if ( g_bevelPanel == NULL ) {
		return TRUE;
	}

	if ( mode ) {
		if ( mode != g_bevelMode || ( mode == 2 && strip != g_bevelEditStrip ) || ( mode == 1 && ( maxW != g_bevelLastMax || bridge != g_bevelLastBridge ) ) ) {
			g_bevelListen = false;
			const float lo = maxW < 0.25f ? maxW : 0.25f;
			gtk_spin_button_set_range( GTK_SPIN_BUTTON( g_bevelWidth ), lo, maxW > lo ? maxW : lo );
			// editing a bevel, 0 subdivisions takes it away; a new bevel needs at least one
			gtk_range_set_range( GTK_RANGE( g_bevelSubdiv ), mode == 2 ? 0 : 1, SEGPATCH_MAX_SEGS );
			char text[128];
			if ( mode == 2 ) {
				gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_bevelWidth ), editW );
				gtk_range_set_value( GTK_RANGE( g_bevelSubdiv ), editN );
				g_bevelFraction = 0;
				if ( editBridge ) {
					sprintf( text, "Editing this strip between two loops (subdivisions only)" );
				}
				else
				{
					sprintf( text, "Editing this bevel (up to %.2f wide)", maxW );
				}
			}
			else
			{
				if ( gtk_range_get_value( GTK_RANGE( g_bevelSubdiv ) ) < 1 ) {
					gtk_range_set_value( GTK_RANGE( g_bevelSubdiv ), 4 );
				}
				if ( g_bevelFraction > 0 && maxW > 0 ) {
					gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_bevelWidth ), maxW * g_bevelFraction );
				}
				if ( bridge ) {
					sprintf( text, "%d edge(s) in two loops: a strip is made between them", count );
				}
				else if ( maxW > 0 ) {
					sprintf( text, "%d edge(s), up to %.2f wide", count, maxW );
				}
				else
				{
					sprintf( text, "%d edge(s): nothing on the other side to bevel towards", count );
				}
			}
			gtk_label_set_text( GTK_LABEL( g_bevelInfo ), text );
				gtk_widget_set_sensitive( g_bevelWidth, !( mode == 2 && editBridge ) );
			gtk_widget_set_visible( g_bevelButton, mode == 1 );
			gtk_widget_set_visible( g_bevelRemoveButton, mode == 2 );
			gtk_widget_set_sensitive( g_bevelButton, maxW > 0 || bridge );
			g_bevelListen = true;
			g_bevelMode = mode;
			g_bevelEditStrip = strip;
			g_bevelLastMax = maxW;
				g_bevelLastBridge = bridge;
		}
		if ( !gtk_widget_get_visible( g_bevelPanel ) ) {
			PanelStack_Show( g_bevelPanel, &g_bevelX, &g_bevelY );
		}
	}
	else
	{
		g_bevelMode = 0;
		g_bevelEditStrip = NULL;
		g_bevelLastMax = -1.0f;
		if ( gtk_widget_get_visible( g_bevelPanel ) ) {
			PanelStack_Hide( g_bevelPanel, &g_bevelX, &g_bevelY );
		}
	}
	return TRUE;
}

// =============================================================================
// Floating window for a cog: shows up by itself while a cog is selected (Curve > Make Cog makes one from a
// cylinder). Every change takes the cog's patches away and makes them again with the new numbers.

static GtkWidget *g_cogPanel = NULL;
static GtkWidget *g_cogTeeth = NULL, *g_cogRadius = NULL, *g_cogDepth = NULL, *g_cogTipW = NULL;
static GtkWidget *g_cogCone = NULL, *g_cogCone2 = NULL;
static GtkWidget *g_cogSocket = NULL, *g_cogOuter = NULL, *g_cogClear = NULL, *g_cogFloor = NULL;
static GtkWidget *g_cogBand = NULL, *g_cogTipArc = NULL, *g_cogRootArc = NULL, *g_cogSnap = NULL;
static const float g_cogSnapSteps[5] = { 0.0f, 0.125f, 0.25f, 0.5f, 1.0f };
static GtkWidget *g_cogExtrude = NULL, *g_cogNotch = NULL, *g_cogInfo = NULL;
static bool g_cogListen = false;
static int g_cogShownId = 0;
static int g_cogX = -1, g_cogY = -1;

static void CogPanelApply(){
	if ( !g_cogListen ) {
		return;
	}
	cogRec_t r;
	int id = 0;
	if ( !Cog_Selected( &r, &id ) ) {
		return;
	}
	r.teeth = (int)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogTeeth ) );
	r.mode = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_cogNotch ) ) ? 1 : 0;
	r.cylR = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogRadius ) );
	r.depth = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogDepth ) );
	r.taper = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogTipW ) );
	r.hole = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogBand ) );
	r.cone = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogCone ) );
	r.cone2 = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogCone2 ) );
	r.socket = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_cogSocket ) ) ? 1 : 0;
	r.outerR = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogOuter ) );
	r.clear = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogClear ) );
	r.floorT = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_cogFloor ) );
	{
		const int sn = gtk_combo_box_get_active( GTK_COMBO_BOX( g_cogSnap ) );
		r.snap = ( sn >= 0 && sn < 5 ) ? g_cogSnapSteps[sn] : 0.0f;
	}
	r.tipArc = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_cogTipArc ) ) ? 1 : 0;
	r.rootArc = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( g_cogRootArc ) ) ? 1 : 0;
	char text[256];
	Cog_Describe( r, text, sizeof( text ) );
	gtk_label_set_text( GTK_LABEL( g_cogInfo ), text );
	Cog_Rebuild( r );
}

// Making the cog again takes a moment. Done inside the signal of the spin button it holds up the button's own
// events, and the button, still seeing the mouse down, steps again by itself: a click on the 5% step of the taper
// came out as 10%. So the change is made a moment later, once the events have been handled (and a quick series of
// clicks is made in one go).
static guint g_cogPending = 0;

static gboolean CogPanelDeferred( gpointer data ){
	g_cogPending = 0;
	CogPanelApply();
	return FALSE;
}

static void OnCogChanged( GtkWidget *widget, gpointer data ){
	if ( !g_cogListen ) {
		return;
	}
	if ( g_cogPending ) {
		g_source_remove( g_cogPending );
	}
	g_cogPending = g_timeout_add( 80, CogPanelDeferred, NULL );
}

static void OnCogSelectWhole( GtkWidget *widget, gpointer data ){
	Cog_SelectWhole();
}

static gboolean OnCogEnter( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	gtk_window_set_accept_focus( GTK_WINDOW( g_cogPanel ), TRUE );
	return FALSE;
}

static gboolean OnCogLeave( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	if ( event->detail == GDK_NOTIFY_INFERIOR ) {
		return FALSE;
	}
	if ( gtk_window_is_active( GTK_WINDOW( g_cogPanel ) ) ) {
		gtk_window_present( GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	}
	gtk_window_set_accept_focus( GTK_WINDOW( g_cogPanel ), FALSE );
	return FALSE;
}

static GtkWidget* CogSpin( GtkWidget *vbox, const char *label, double lo, double hi, double step, int digits, const char *tip ){
	GtkWidget *hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	GtkWidget *l = gtk_label_new( label );
	gtk_misc_set_alignment( GTK_MISC( l ), 0.0, 0.5 );
	gtk_widget_set_size_request( l, 110, -1 );
	gtk_box_pack_start( GTK_BOX( hbox ), l, FALSE, FALSE, 0 );
	gtk_widget_show( l );
	GtkWidget *s = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( lo, lo, hi, step, step * 4, 0 ) ), step, digits );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( s ), TRUE );
	gtk_widget_set_tooltip_text( s, tip );
	gtk_box_pack_start( GTK_BOX( hbox ), s, TRUE, TRUE, 0 );
	gtk_widget_show( s );
	g_signal_connect( G_OBJECT( s ), "value-changed", G_CALLBACK( OnCogChanged ), NULL );
	return s;
}

static void CogPanelBuild(){
	GtkWidget *vbox, *hbox;

	g_cogPanel = gtk_window_new( GTK_WINDOW_TOPLEVEL );
	gtk_window_set_title( GTK_WINDOW( g_cogPanel ), _( "Cog" ) );
	gtk_window_set_transient_for( GTK_WINDOW( g_cogPanel ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	gtk_window_set_type_hint( GTK_WINDOW( g_cogPanel ), GDK_WINDOW_TYPE_HINT_UTILITY );
	gtk_window_set_skip_taskbar_hint( GTK_WINDOW( g_cogPanel ), TRUE );
	gtk_window_set_focus_on_map( GTK_WINDOW( g_cogPanel ), FALSE );
	gtk_window_set_accept_focus( GTK_WINDOW( g_cogPanel ), FALSE );
	gtk_window_set_resizable( GTK_WINDOW( g_cogPanel ), FALSE );
	gtk_window_set_deletable( GTK_WINDOW( g_cogPanel ), FALSE );

	vbox = gtk_vbox_new( FALSE, 4 );
	gtk_container_set_border_width( GTK_CONTAINER( vbox ), 6 );
	gtk_container_add( GTK_CONTAINER( g_cogPanel ), vbox );
	gtk_widget_show( vbox );

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	g_cogExtrude = gtk_radio_button_new_with_label( NULL, _( "Teeth outside" ) );
	gtk_widget_set_tooltip_text( g_cogExtrude, _( "The cylinder is the body of the cog and the teeth stand out beyond it" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_cogExtrude, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogExtrude );
	g_cogNotch = gtk_radio_button_new_with_label_from_widget( GTK_RADIO_BUTTON( g_cogExtrude ), _( "Teeth inside" ) );
	gtk_widget_set_tooltip_text( g_cogNotch, _( "The cog is made inside the cylinder: the tips reach its radius and nothing goes beyond it" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_cogNotch, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogNotch );

	g_cogTeeth = CogSpin( vbox, _( "Teeth" ), 3, 64, 1, 0, _( "How many teeth round the cog" ) );
	g_cogRadius = CogSpin( vbox, _( "Cylinder radius" ), 1, 4096, 1, 2, _( "The radius of the cylinder the cog was made from. With the teeth outside it is the root, with the teeth inside it is the tip" ) );
	g_cogDepth = CogSpin( vbox, _( "Tooth depth" ), 0.25, 4096, 0.5, 2, _( "How far the tip is from the root" ) );
	g_cogTipW = CogSpin( vbox, _( "Tip taper %" ), 0, 90, 5, 0, _( "How much a tooth narrows towards its tip, as a percentage of its base width. 0 keeps the sides parallel, 90 is nearly a point. The base of a tooth is always as wide as the gap next to it, so the cog keeps the segments of the cylinder" ) );
	g_cogBand = CogSpin( vbox, _( "Hole radius" ), 0, 4096, 1, 2, _( "A hole through the middle of this radius, for a tube or axle. 0 leaves the cog solid" ) );
	g_cogCone = CogSpin( vbox, _( "Centre offset, top" ), -4096, 4096, 1, 2, _( "How much higher (positive) or lower (negative) the middle of the top end (the +axis side) is than its edge. From the middle (or the edge of the hole) out to the tips of the teeth the end slopes in a straight line. 0 leaves the end flat" ) );
	g_cogCone2 = CogSpin( vbox, _( "Centre offset, bottom" ), -4096, 4096, 1, 2, _( "The same for the bottom end (the other side of the cog)" ) );
	g_cogSocket = gtk_check_button_new_with_label( _( "Socket (cog-shaped hole)" ) );
	gtk_widget_set_tooltip_text( g_cogSocket, _( "Make a block with the cog as a hole through it instead of the cog: a cog made with the same numbers fits inside" ) );
	gtk_box_pack_start( GTK_BOX( vbox ), g_cogSocket, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogSocket );
	g_signal_connect( G_OBJECT( g_cogSocket ), "toggled", G_CALLBACK( OnCogChanged ), NULL );
	g_cogOuter = CogSpin( vbox, _( "Block radius" ), 1, 8192, 1, 2, _( "Socket only: the radius of the block round the hole. It has a corner on every tip corner of the hole, two sides to a tooth" ) );
	g_cogClear = CogSpin( vbox, _( "Clearance" ), 0, 1024, 0.25, 2, _( "Socket only: how much wider than the cog the hole is, all round, so the cog turns freely" ) );
	g_cogFloor = CogSpin( vbox, _( "Floor thickness" ), 0, 8192, 1, 2, _( "Socket only: 0 leaves the hole open right through. More than 0 makes a pocket, open at the top, with a floor under the pocket, this thick: the pocket is as deep as the cog is long, and the block ends this far below its bottom. The floor is built like the end of a cog (a centre fan and a plane for the bottom of every tooth); the top and bottom offsets slope the floor and the underside of the block" ) );
	gtk_spin_button_set_snap_to_ticks( GTK_SPIN_BUTTON( g_cogTipW ), TRUE );
	{
		GtkWidget *sh = gtk_hbox_new( FALSE, 6 );
		gtk_box_pack_start( GTK_BOX( vbox ), sh, FALSE, FALSE, 0 );
		gtk_widget_show( sh );
		GtkWidget *sl = gtk_label_new( _( "Snap to grid" ) );
		gtk_misc_set_alignment( GTK_MISC( sl ), 0.0, 0.5 );
		gtk_widget_set_size_request( sl, 110, -1 );
		gtk_box_pack_start( GTK_BOX( sh ), sl, FALSE, FALSE, 0 );
		gtk_widget_show( sl );
		g_cogSnap = gtk_combo_box_text_new();
		gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( g_cogSnap ), _( "Off" ) );
		gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( g_cogSnap ), _( "0.125 (smallest grid)" ) );
		gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( g_cogSnap ), "0.25" );
		gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( g_cogSnap ), "0.5" );
		gtk_combo_box_text_append_text( GTK_COMBO_BOX_TEXT( g_cogSnap ), "1" );
		gtk_combo_box_set_active( GTK_COMBO_BOX( g_cogSnap ), 0 );
		gtk_widget_set_tooltip_text( g_cogSnap, _( "Round every control point of the cog to this grid step, so all its corners lie on the grid" ) );
		gtk_box_pack_start( GTK_BOX( sh ), g_cogSnap, TRUE, TRUE, 0 );
		gtk_widget_show( g_cogSnap );
		g_signal_connect( G_OBJECT( g_cogSnap ), "changed", G_CALLBACK( OnCogChanged ), NULL );
	}

	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	g_cogTipArc = gtk_check_button_new_with_label( _( "Tip is an arc" ) );
	gtk_widget_set_tooltip_text( g_cogTipArc, _( "Follow the circle at the tip instead of a straight chord" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_cogTipArc, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogTipArc );
	g_cogRootArc = gtk_check_button_new_with_label( _( "Root is an arc" ) );
	gtk_widget_set_tooltip_text( g_cogRootArc, _( "Follow the circle at the root instead of a straight chord" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_cogRootArc, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogRootArc );

	{
		GtkWidget *sel = gtk_button_new_with_label( _( "Select whole cog  (Ctrl+Shift+A)" ) );
		gtk_widget_set_tooltip_text( sel, _( "Select every patch of this cog (and bevels made on it), so it can be moved or copied as one. Also on the Curve menu, the same shortcut works in the camera and the grid views" ) );
		gtk_box_pack_start( GTK_BOX( vbox ), sel, FALSE, FALSE, 0 );
		gtk_widget_show( sel );
		g_signal_connect( G_OBJECT( sel ), "clicked", G_CALLBACK( OnCogSelectWhole ), NULL );
	}

	g_cogInfo = gtk_label_new( "" );
	gtk_misc_set_alignment( GTK_MISC( g_cogInfo ), 0.0, 0.5 );
	gtk_label_set_line_wrap( GTK_LABEL( g_cogInfo ), TRUE );
	gtk_widget_set_size_request( g_cogInfo, 300, -1 );
	gtk_box_pack_start( GTK_BOX( vbox ), g_cogInfo, FALSE, FALSE, 0 );
	gtk_widget_show( g_cogInfo );

	g_signal_connect( G_OBJECT( g_cogNotch ), "toggled", G_CALLBACK( OnCogChanged ), NULL );
	g_signal_connect( G_OBJECT( g_cogTipArc ), "toggled", G_CALLBACK( OnCogChanged ), NULL );
	g_signal_connect( G_OBJECT( g_cogRootArc ), "toggled", G_CALLBACK( OnCogChanged ), NULL );
	gtk_widget_add_events( g_cogPanel, GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK );
	g_signal_connect( G_OBJECT( g_cogPanel ), "enter-notify-event", G_CALLBACK( OnCogEnter ), NULL );
	g_signal_connect( G_OBJECT( g_cogPanel ), "leave-notify-event", G_CALLBACK( OnCogLeave ), NULL );
}

// put the numbers of a cog into the widgets without rebuilding it
static void CogPanelFill( const cogRec_t &r ){
	g_cogListen = false;
	gtk_spin_button_set_range( GTK_SPIN_BUTTON( g_cogRadius ), 1, r.cylR > 4096 ? r.cylR : 4096 );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogTeeth ), r.teeth );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogRadius ), r.cylR );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogDepth ), r.depth );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogTipW ), r.taper );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogBand ), r.hole );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogCone ), r.cone );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogCone2 ), r.cone2 );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogOuter ), r.outerR );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogClear ), r.clear );
	gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_cogFloor ), r.floorT );
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( g_cogSocket ), r.socket != 0 );
	{
		int at = 0;
		for ( int i = 1; i < 5; i++ )
			if ( fabsf( r.snap - g_cogSnapSteps[i] ) < 0.001f ) {
				at = i;
			}
		gtk_combo_box_set_active( GTK_COMBO_BOX( g_cogSnap ), at );
	}
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( r.mode ? g_cogNotch : g_cogExtrude ), TRUE );
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( g_cogTipArc ), r.tipArc != 0 );
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( g_cogRootArc ), r.rootArc != 0 );
	char text[256];
	Cog_Describe( r, text, sizeof( text ) );
	gtk_label_set_text( GTK_LABEL( g_cogInfo ), text );
	g_cogListen = true;
}

static gboolean CogPanelTick( gpointer data ){
	if ( g_pParentWnd == NULL || g_pParentWnd->m_pWidget == NULL ) {
		return TRUE;
	}
	cogRec_t r;
	int id = 0;
	const bool have = Cog_Selected( &r, &id );
	if ( have && g_cogPanel == NULL ) {
		CogPanelBuild();
	}
	if ( g_cogPanel == NULL ) {
		return TRUE;
	}
	if ( have ) {
		if ( id != g_cogShownId || !gtk_widget_get_visible( g_cogPanel ) ) {
			if ( id != g_cogShownId ) {
				CogPanelFill( r );
				g_cogShownId = id;
			}
		}
		if ( !gtk_widget_get_visible( g_cogPanel ) ) {
			PanelStack_Show( g_cogPanel, &g_cogX, &g_cogY );
		}
	}
	else
	{
		g_cogShownId = 0;
		if ( gtk_widget_get_visible( g_cogPanel ) ) {
			PanelStack_Hide( g_cogPanel, &g_cogX, &g_cogY );
		}
	}
	return TRUE;
}

// Floating window for a cone (a segmented cap): shows up by itself while one is selected and sets the height of its
// tip over the plane of its ring. The half-way row follows, half way up. The inner radius turns the tip into a smaller
// ring (a hole, so the cap becomes a donut); 0 is a point again.
static GtkWidget *g_conePanel = NULL, *g_coneHeight = NULL, *g_coneInner = NULL;
static bool g_coneListen = false;
static const void *g_coneShown = NULL;
static int g_coneX = -1, g_coneY = -1;
static guint g_conePending = 0;
static int g_conePendingMask = 0; // 1: height changed, 2: inner radius changed

static gboolean ConePanelDeferred( gpointer data ){
	g_conePending = 0;
	const int mask = g_conePendingMask;
	g_conePendingMask = 0;
	if ( g_coneListen && mask ) {
		const float h = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_coneHeight ) );
		const float r = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_coneInner ) );
		Cone_Set( ( mask & 1 ) ? &h : NULL, ( mask & 2 ) ? &r : NULL );
	}
	return FALSE;
}

static void OnConeChanged( GtkWidget *widget, gpointer data ){
	if ( !g_coneListen ) {
		return;
	}
	g_conePendingMask |= ( widget == g_coneInner ) ? 2 : 1;
	if ( g_conePending ) {
		g_source_remove( g_conePending );
	}
	g_conePending = g_timeout_add( 80, ConePanelDeferred, NULL );
}

static gboolean OnConeEnter( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	gtk_window_set_accept_focus( GTK_WINDOW( g_conePanel ), TRUE );
	return FALSE;
}

static gboolean OnConeLeave( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	if ( event->detail == GDK_NOTIFY_INFERIOR ) {
		return FALSE;
	}
	if ( gtk_window_is_active( GTK_WINDOW( g_conePanel ) ) ) {
		gtk_window_present( GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	}
	gtk_window_set_accept_focus( GTK_WINDOW( g_conePanel ), FALSE );
	return FALSE;
}

static void ConePanelBuild(){
	g_conePanel = gtk_window_new( GTK_WINDOW_TOPLEVEL );
	gtk_window_set_title( GTK_WINDOW( g_conePanel ), _( "Cone" ) );
	gtk_window_set_transient_for( GTK_WINDOW( g_conePanel ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	gtk_window_set_type_hint( GTK_WINDOW( g_conePanel ), GDK_WINDOW_TYPE_HINT_UTILITY );
	gtk_window_set_skip_taskbar_hint( GTK_WINDOW( g_conePanel ), TRUE );
	gtk_window_set_focus_on_map( GTK_WINDOW( g_conePanel ), FALSE );
	gtk_window_set_accept_focus( GTK_WINDOW( g_conePanel ), FALSE );
	gtk_window_set_resizable( GTK_WINDOW( g_conePanel ), FALSE );
	gtk_window_set_deletable( GTK_WINDOW( g_conePanel ), FALSE );

	GtkWidget *vbox = gtk_vbox_new( FALSE, 4 );
	gtk_container_set_border_width( GTK_CONTAINER( vbox ), 6 );
	gtk_container_add( GTK_CONTAINER( g_conePanel ), vbox );
	gtk_widget_show( vbox );
	GtkWidget *hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	GtkWidget *l = gtk_label_new( _( "Cone height" ) );
	gtk_misc_set_alignment( GTK_MISC( l ), 0.0, 0.5 );
	gtk_box_pack_start( GTK_BOX( hbox ), l, FALSE, FALSE, 0 );
	gtk_widget_show( l );
	g_coneHeight = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( 0, -8192, 8192, 1, 8, 0 ) ), 1, 2 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( g_coneHeight ), TRUE );
	gtk_widget_set_tooltip_text( g_coneHeight, _( "How high the tip is over the plane of the ring, in units. Positive raises it on the side the patch faces (outwards for a cap made with Cap Segmented), negative sinks it. 0 is a flat cap. The half-way points follow, half way up, so the sides stay straight" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_coneHeight, TRUE, TRUE, 0 );
	gtk_widget_show( g_coneHeight );
	g_signal_connect( G_OBJECT( g_coneHeight ), "value-changed", G_CALLBACK( OnConeChanged ), NULL );
	hbox = gtk_hbox_new( FALSE, 6 );
	gtk_box_pack_start( GTK_BOX( vbox ), hbox, FALSE, FALSE, 0 );
	gtk_widget_show( hbox );
	l = gtk_label_new( _( "Inner radius" ) );
	gtk_misc_set_alignment( GTK_MISC( l ), 0.0, 0.5 );
	gtk_box_pack_start( GTK_BOX( hbox ), l, FALSE, FALSE, 0 );
	gtk_widget_show( l );
	g_coneInner = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( 0, 0, 8192, 1, 8, 0 ) ), 1, 2 );
	gtk_spin_button_set_numeric( GTK_SPIN_BUTTON( g_coneInner ), TRUE );
	gtk_widget_set_tooltip_text( g_coneInner, _( "Radius of a hole in the middle of the cap, in units. 0 is a filled cap (the tip is a point). Above 0 the cap becomes a donut: a flat ring from the edge of the cylinder in to a smaller ring of this radius. The most is just under the radius of the cap itself" ) );
	gtk_box_pack_start( GTK_BOX( hbox ), g_coneInner, TRUE, TRUE, 0 );
	gtk_widget_show( g_coneInner );
	g_signal_connect( G_OBJECT( g_coneInner ), "value-changed", G_CALLBACK( OnConeChanged ), NULL );
	gtk_widget_add_events( g_conePanel, GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK );
	g_signal_connect( G_OBJECT( g_conePanel ), "enter-notify-event", G_CALLBACK( OnConeEnter ), NULL );
	g_signal_connect( G_OBJECT( g_conePanel ), "leave-notify-event", G_CALLBACK( OnConeLeave ), NULL );
}

static gboolean ConePanelTick( gpointer data ){
	if ( g_pParentWnd == NULL || g_pParentWnd->m_pWidget == NULL ) {
		return TRUE;
	}
	float h = 0, rIn = 0, rOut = 0;
	const void *who = NULL;
	const bool have = Cone_Selected( &h, &rIn, &rOut, &who );
	if ( have && g_conePanel == NULL ) {
		ConePanelBuild();
	}
	if ( g_conePanel == NULL ) {
		return TRUE;
	}
	if ( have ) {
		if ( who != g_coneShown ) {
			g_coneListen = false;
			gtk_spin_button_set_range( GTK_SPIN_BUTTON( g_coneInner ), 0, floor( rOut * 0.99f * 100.0f ) / 100.0 );
			gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_coneHeight ), h );
			gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_coneInner ), rIn );
			g_coneListen = true;
			g_coneShown = who;
		}
		if ( !gtk_widget_get_visible( g_conePanel ) ) {
			PanelStack_Show( g_conePanel, &g_coneX, &g_coneY );
		}
	}
	else
	{
		g_coneShown = NULL;
		if ( gtk_widget_get_visible( g_conePanel ) ) {
			PanelStack_Hide( g_conePanel, &g_coneX, &g_coneY );
		}
	}
	return TRUE;
}

// Floating window for the mouse rotation (R): shows up by itself while the mode is on, with the pivot as three numbers (it
// follows the pivot when that is moved in a grid view, and moves it when typed into) and the angles to turn the selection
// by, in degrees. Apply turns the selection about the pivot: X, then Y, then Z, counter-clockwise looking down each axis
// at the pivot for a positive angle. Enter in an angle box applies as well.
static GtkWidget *g_rotPanel = NULL;
static GtkWidget *g_rotPivot[3] = { NULL, NULL, NULL };
static GtkWidget *g_rotDeg[3] = { NULL, NULL, NULL };
static bool g_rotListen = false;
static int g_rotX = -1, g_rotY = -1;

static void OnRotPivotChanged( GtkWidget *widget, gpointer data ){
	if ( !g_rotListen || !g_pParentWnd->ActiveXY() ) {
		return;
	}
	vec3_t &p = g_pParentWnd->ActiveXY()->RotateOrigin();
	for ( int i = 0; i < 3; i++ )
		p[i] = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_rotPivot[i] ) );
	Sys_UpdateWindows( W_XY | W_CAMERA );
}

static void OnRotApply( GtkWidget *widget, gpointer data ){
	if ( !g_bRotateMode || !g_pParentWnd->ActiveXY() ) {
		return;
	}
	for ( int i = 0; i < 3; i++ )
		gtk_spin_button_update( GTK_SPIN_BUTTON( g_rotDeg[i] ) );
	float deg[3];
	bool any = false;
	for ( int i = 0; i < 3; i++ )
	{
		deg[i] = (float)gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_rotDeg[i] ) );
		if ( deg[i] != 0.0f ) {
			any = true;
		}
	}
	if ( !any || selected_brushes.next == &selected_brushes ) {
		return;
	}
	Undo_Start( "rotate by angle" );
	Undo_AddBrushList( &selected_brushes );
	// Select_RotateAxis turns by the matrix of the grid views, where a positive angle about X and Z runs clockwise
	const float sign[3] = { -1.0f, 1.0f, -1.0f };
	for ( int axis = 0; axis < 3; axis++ )
	{
		if ( deg[axis] != 0.0f ) {
			Select_RotateAxis( axis, sign[axis] * deg[axis], false, true );
			g_pParentWnd->ActiveXY()->Rotation()[axis] += deg[axis];
		}
	}
	Undo_EndBrushList( &selected_brushes );
	Undo_End();
	for ( int i = 0; i < 3; i++ )
		gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_rotDeg[i] ), 0 );
	Sys_UpdateWindows( W_ALL );
}

static void OnRotCentre( GtkWidget *widget, gpointer data ){
	if ( !g_bRotateMode || !g_pParentWnd->ActiveXY() ) {
		return;
	}
	Select_GetTrueMid( g_pParentWnd->ActiveXY()->RotateOrigin() );
	Sys_UpdateWindows( W_XY | W_CAMERA );
}

static gboolean OnRotEnter( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	gtk_window_set_accept_focus( GTK_WINDOW( g_rotPanel ), TRUE );
	return FALSE;
}

static gboolean OnRotLeave( GtkWidget *widget, GdkEventCrossing *event, gpointer data ){
	if ( event->detail == GDK_NOTIFY_INFERIOR ) {
		return FALSE;
	}
	if ( gtk_window_is_active( GTK_WINDOW( g_rotPanel ) ) ) {
		gtk_window_present( GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	}
	gtk_window_set_accept_focus( GTK_WINDOW( g_rotPanel ), FALSE );
	return FALSE;
}

static void RotPanelBuild(){
	g_rotPanel = gtk_window_new( GTK_WINDOW_TOPLEVEL );
	gtk_window_set_title( GTK_WINDOW( g_rotPanel ), _( "Rotate" ) );
	gtk_window_set_transient_for( GTK_WINDOW( g_rotPanel ), GTK_WINDOW( g_pParentWnd->m_pWidget ) );
	gtk_window_set_type_hint( GTK_WINDOW( g_rotPanel ), GDK_WINDOW_TYPE_HINT_UTILITY );
	gtk_window_set_skip_taskbar_hint( GTK_WINDOW( g_rotPanel ), TRUE );
	gtk_window_set_focus_on_map( GTK_WINDOW( g_rotPanel ), FALSE );
	gtk_window_set_accept_focus( GTK_WINDOW( g_rotPanel ), FALSE );
	gtk_window_set_resizable( GTK_WINDOW( g_rotPanel ), FALSE );
	gtk_window_set_deletable( GTK_WINDOW( g_rotPanel ), FALSE );

	GtkWidget *table = gtk_table_new( 3, 5, FALSE );
	gtk_table_set_row_spacings( GTK_TABLE( table ), 4 );
	gtk_table_set_col_spacings( GTK_TABLE( table ), 4 );
	gtk_container_set_border_width( GTK_CONTAINER( table ), 6 );
	gtk_container_add( GTK_CONTAINER( g_rotPanel ), table );
	gtk_widget_show( table );

	GtkWidget *l = gtk_label_new( _( "Pivot" ) );
	gtk_misc_set_alignment( GTK_MISC( l ), 0.0, 0.5 );
	gtk_table_attach( GTK_TABLE( table ), l, 0, 1, 0, 1, GTK_FILL, GTK_FILL, 0, 0 );
	gtk_widget_show( l );
	l = gtk_label_new( _( "Turn by (deg)" ) );
	gtk_misc_set_alignment( GTK_MISC( l ), 0.0, 0.5 );
	gtk_table_attach( GTK_TABLE( table ), l, 0, 1, 1, 2, GTK_FILL, GTK_FILL, 0, 0 );
	gtk_widget_show( l );

	const char *axisName[3] = { "X", "Y", "Z" };
	for ( int i = 0; i < 3; i++ )
	{
		g_rotPivot[i] = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( 0, -65536, 65536, 1, 8, 0 ) ), 1, 2 );
		gtk_entry_set_width_chars( GTK_ENTRY( g_rotPivot[i] ), 9 );
		gtk_widget_set_tooltip_text( g_rotPivot[i], _( "The pivot of the rotation, the same point the pink marker shows. Moving it with Shift + middle mouse button in a grid view updates these; typing here moves the marker" ) );
		gtk_table_attach( GTK_TABLE( table ), g_rotPivot[i], i + 1, i + 2, 0, 1, GTK_FILL, GTK_FILL, 0, 0 );
		gtk_widget_show( g_rotPivot[i] );
		g_signal_connect( G_OBJECT( g_rotPivot[i] ), "value-changed", G_CALLBACK( OnRotPivotChanged ), NULL );

		g_rotDeg[i] = gtk_spin_button_new( GTK_ADJUSTMENT( gtk_adjustment_new( 0, -360, 360, 1, 15, 0 ) ), 1, 2 );
		gtk_entry_set_width_chars( GTK_ENTRY( g_rotDeg[i] ), 9 );
		gtk_widget_set_tooltip_text( g_rotDeg[i], _( "Degrees to turn the selection about the pivot, around the world axis named in the column. Positive turns counter-clockwise looking down the axis at the pivot. Enter applies" ) );
		gtk_table_attach( GTK_TABLE( table ), g_rotDeg[i], i + 1, i + 2, 1, 2, GTK_FILL, GTK_FILL, 0, 0 );
		gtk_widget_show( g_rotDeg[i] );
		g_signal_connect( G_OBJECT( g_rotDeg[i] ), "activate", G_CALLBACK( OnRotApply ), NULL );

		l = gtk_label_new( axisName[i] );
		gtk_table_attach( GTK_TABLE( table ), l, i + 1, i + 2, 2, 3, GTK_FILL, GTK_FILL, 0, 0 );
		gtk_widget_show( l );
	}
	GtkWidget *b = gtk_button_new_with_label( _( "Apply" ) );
	gtk_widget_set_tooltip_text( b, _( "Turn the selection by the angles, about the X, then the Y, then the Z axis through the pivot" ) );
	gtk_table_attach( GTK_TABLE( table ), b, 4, 5, 1, 2, GTK_FILL, GTK_FILL, 0, 0 );
	gtk_widget_show( b );
	g_signal_connect( G_OBJECT( b ), "clicked", G_CALLBACK( OnRotApply ), NULL );
	b = gtk_button_new_with_label( _( "Centre" ) );
	gtk_widget_set_tooltip_text( b, _( "Put the pivot back in the middle of the selection" ) );
	gtk_table_attach( GTK_TABLE( table ), b, 4, 5, 0, 1, GTK_FILL, GTK_FILL, 0, 0 );
	gtk_widget_show( b );
	g_signal_connect( G_OBJECT( b ), "clicked", G_CALLBACK( OnRotCentre ), NULL );

	gtk_widget_add_events( g_rotPanel, GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK );
	g_signal_connect( G_OBJECT( g_rotPanel ), "enter-notify-event", G_CALLBACK( OnRotEnter ), NULL );
	g_signal_connect( G_OBJECT( g_rotPanel ), "leave-notify-event", G_CALLBACK( OnRotLeave ), NULL );
}

static gboolean RotPanelTick( gpointer data ){
	if ( g_pParentWnd == NULL || g_pParentWnd->m_pWidget == NULL || g_pParentWnd->ActiveXY() == NULL ) {
		return TRUE;
	}
	const bool on = g_bRotateMode;
	if ( on && g_rotPanel == NULL ) {
		RotPanelBuild();
	}
	if ( g_rotPanel == NULL ) {
		return TRUE;
	}
	if ( on ) {
		// follow the pivot, unless a box of the pivot is being typed into
		const vec3_t &p = g_pParentWnd->ActiveXY()->RotateOrigin();
		g_rotListen = false;
		for ( int i = 0; i < 3; i++ )
			if ( !gtk_widget_has_focus( g_rotPivot[i] ) && fabs( gtk_spin_button_get_value( GTK_SPIN_BUTTON( g_rotPivot[i] ) ) - p[i] ) > 0.001 ) {
				gtk_spin_button_set_value( GTK_SPIN_BUTTON( g_rotPivot[i] ), p[i] );
			}
		g_rotListen = true;
		if ( !gtk_widget_get_visible( g_rotPanel ) ) {
			PanelStack_Show( g_rotPanel, &g_rotX, &g_rotY );
		}
	}
	else if ( gtk_widget_get_visible( g_rotPanel ) ) {
		PanelStack_Hide( g_rotPanel, &g_rotX, &g_rotY );
	}
	return TRUE;
}

void SegPanel_Init(){
	g_timeout_add( 150, SegPanelTick, NULL );
	g_timeout_add( 150, BevelPanelTick, NULL );
	g_timeout_add( 150, CogPanelTick, NULL );
	g_timeout_add( 150, ConePanelTick, NULL );
	g_timeout_add( 150, RotPanelTick, NULL );
}
