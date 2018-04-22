/* ----------------------------------------------------------------------------
 * Copyright (C) 2004 by egnite Software GmbH
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
 * ----------------------------------------------------------------------------
 */

/*
 * $Id: nutconfview.cpp 4338 2012-07-05 13:55:13Z haraldkipp $
 */

#include <wx/filename.h>

#include "ids.h"
#include "nutconf.h"
#include "utils.h"
#include "nutconfhint.h"
#include "nutconfview.h"

IMPLEMENT_DYNAMIC_CLASS(CNutConfView, wxView)

BEGIN_EVENT_TABLE(CNutConfView, wxView)
END_EVENT_TABLE();

CNutConfView::CNutConfView()
{
    m_mainFrame = NULL;
    m_configTree = NULL;
    m_propertyList = NULL;
    m_infoText = NULL;
    m_expandedForFind = wxTreeItemId();
    wxFrame *frame = (wxFrame *)wxGetApp().GetTopWindow();
    SetFrame(frame);
}

bool CNutConfView::OnCreate(wxDocument* doc, long flags)
{
    if (!wxView::OnCreate(doc, flags))
        return false;

    /* Re-use existing windows. */
    NutConfApp& app = wxGetApp();
    m_mainFrame = app.GetMainFrame();
    m_configTree = m_mainFrame->GetTreeCtrl();
    m_propertyList = m_mainFrame->GetPropertyListWindow();
    m_infoText = m_mainFrame->GetInfoWindow();

    wxGetApp().GetDocManager()->ActivateView(this, true);

    return true;
}

/*!
 * \brief Draw the views.
 *
 * Pure virtual method of the base class, but nothing to do here,
 * because the controls draw themselves.
 */
void CNutConfView::OnDraw(wxDC * WXUNUSED(dc))
{
}

void CNutConfView::OnUpdate(wxView * WXUNUSED(sender), wxObject * hintObj)
{
    CConfTreeNode *node = NULL;
    wxDataViewItem sel = m_configTree->GetSelection();
    if (sel.IsOk()) {
        node = (CConfTreeNode*) sel.GetID();
    }

    CNutConfHint *hint = (CNutConfHint *) hintObj;
    int hintOp = nutNoHint;
    if (hint)
        hintOp = hint->m_op;

    switch (hintOp) {
    case nutSelChanged:
        if (node) {
            wxString content = node->GetDescription();
            content.Replace("\n", "<br />");
            m_infoText->SetPage(content);
            m_propertyList->Fill(node);
        } else {
            m_infoText->SetPage(wxEmptyString);
            m_propertyList->ClearAll();
        }
        break;
    case nutAllSaved:
        break;
    case nutFilenameChanged:
        break;
    case nutNameFormatChanged:
        break;
    case nutIntFormatChanged:
        break;
    case nutClear:
        break;
    case nutValueChanged:
        break;
    case nutExternallyChanged:
        m_configTree->Refresh();
        m_propertyList->Fill(node);
        break;
    default:
        break;
    }
}

// Clean up windows used for displaying the view.
bool CNutConfView::OnClose(bool WXUNUSED(deleteWindow))
{
    CNutConfHint hint(NULL, nutClear);
    GetDocument()->UpdateAllViews(NULL, &hint);

    if (!GetDocument()->Close())
        return false;

    wxGetApp().GetDocManager()->ActivateView(this, false);

    Activate(false);

    return true;
}

void CNutConfView::Refresh(const wxString & WXUNUSED(macroName))
{
}

void CNutConfView::Refresh(wxTreeItemId WXUNUSED(h))
{
}
