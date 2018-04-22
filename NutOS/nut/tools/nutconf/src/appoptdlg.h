#ifndef APPOPTDLG_H_
#define APPOPTDLG_H_

/* ----------------------------------------------------------------------------
 * Copyright (C) 2008-2012 by egnite GmbH
 * Copyright (C) 2004-2007 by egnite Software GmbH
 * Copyright (C) 1998, 1999, 2000 Red Hat, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * ----------------------------------------------------------------------------
 */

/*
 * $Id: appoptdlg.h 4343 2012-07-06 07:48:25Z haraldkipp $
 */

#include <wx/wx.h>
#include <wx/config.h>
#include <wx/filepicker.h>

/*!
 * \brief Settings dialog for application tree options.
 */
class CAppOptionsDialog: public wxPanel
{
DECLARE_CLASS(CAppOptionsDialog)
public:
    CAppOptionsDialog(wxWindow* parent);
    virtual bool TransferDataToWindow();
    virtual bool TransferDataFromWindow();
private:
    /*! \brief Combo box control used to select the programmer.
    */
    wxComboBox *m_cbxProgrammer;

    /*! \brief Text control used to specify the sample directory path.
    */
    wxDirPickerCtrl *m_pickAppDir;

    void OnProgrammerEnter(wxCommandEvent& event);
    void OnAppDirChange(wxCommandEvent& event);
    void PopulateProgrammer();

    DECLARE_EVENT_TABLE()

};

#endif
