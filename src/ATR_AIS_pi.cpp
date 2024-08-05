/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  ATR_AIS PlugIn
 * Author:   Douwe Fokkema / John Hatfield
 *
 ***************************************************************************
 *   Copyright (C) 2019 by Douwe Fokkema                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 ***************************************************************************
 */

#include "ATR_AIS_pi.h"
#include "AutotrackInfoUI.h"
#include "Info.h"
#include "PreferencesDialog.h"
#include "icons.h"
#include "ocpn_plugin.h"


//#include "c:\atemp\OpenCPN_Dev\AIS_PlugIn\ATR_AIS_pi\libs\N2KParser\include\N2KParser.h"
//#include "c:\atemp\OpenCPN_Dev\AIS_PlugIn\ATR_AIS_pi\libs\N2KParser\include\N2kMsg.h"
//#include "c:\atemp\OpenCPN_Dev\AIS_PlugIn\ATR_AIS_pi\libs\N2KParser\include\N2kMessages.h"
#include "N2KParser.h"
#include "N2kMessages.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <iomanip>


#include <wx/stdpaths.h>

#define TURNRATE 20. // turnrate per second

double heading_resolve(double degrees, double offset = 0)
{
    while (degrees < offset - 180)
        degrees += 360;
    while (degrees >= offset + 180)
        degrees -= 360;
    return degrees;
}

static inline double GeodesicRadToDeg(double rads) {
    return rads * 180.0 / M_PI;
}

static inline double MS2KNOTS(double ms) {
    return ms * 1.9438444924406;
}

static inline double DEG2RAD(double deg) {
    return deg * M_PI / 180.0;
}

static inline double KNOT2MS(double kts) {
    return kts / 1.9438444924406;
}


// the class factories, used to create and destroy instances of the PlugIn

extern "C" DECL_EXP opencpn_plugin* create_pi(void* ppimgr)
{
    return new ATR_AIS_pi(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p) { delete p; }

#include "icons.h"

bool found_handle;

//-----------------------------------------------------------------------------
//
//    ATR_AIS PlugIn Implementation
//
//-----------------------------------------------------------------------------

ATR_AIS_pi::ATR_AIS_pi(void* ppimgr)
    : opencpn_plugin_118(ppimgr)
{
    // Create the PlugIn icons
    initialize_images();

    // Create the PlugIn icons  -from shipdriver
    // loads png file for the listing panel icon
    wxFileName fn;
    wxLogMessage("starting init atr_ais");
    auto path = GetPluginDataDir("ATR_AIS_pi");
    fn.SetPath(path);
    fn.AppendDir("data");
    fn.SetFullName("tracking_panel.png");

    path = fn.GetFullPath();

    wxInitAllImageHandlers();

    wxLogDebug(wxString("Using icon path: ") + path);
    if (!wxImage::CanRead(path)) {
        wxLogDebug("Initiating image handlers.");
        wxInitAllImageHandlers();
    }
    wxImage panelIcon(path);
    if (panelIcon.IsOk())
        m_panelBitmap = wxBitmap(panelIcon);
    else
        wxLogWarning("ATR_AIS panel icon has NOT been loaded");
    // End of from Shipdrive

    m_info_dialog = NULL;
    m_PreferencesDialog = NULL;
    m_pdeficon = new wxBitmap(*_img_ATR_AIS);
    m_initialized = false;
}

//---------------------------------------------------------------------------------------------------------
//
//          PlugIn initialization and de-init
//
//---------------------------------------------------------------------------------------------------------

int ATR_AIS_pi::Init(void)
{
 AddLocaleCatalog(PLUGIN_CATALOG_NAME);

    // Read Config
    wxFileConfig* pConf = GetOCPNConfigObject();
    pConf->SetPath(_T("/Settings/ATR_AIS"));

    m_heading_set = false;
    m_current_bearing = 0.;
    m_XTE = 0.;
    m_BTW = 0.;
    m_DTW = 0.;
    m_var = 0.;
    m_XTE_P = 0.;
    m_XTE_I = 0.;
    m_XTE_D = 0.;
    m_pilot_heading = -1.; // target heading of pilot in auto mode, -1 means undefined
    m_vessel_heading = -1.;
    SetStandby();

    // Mode
    preferences& p = m_prefs;

    p.max_angle = pConf->Read("MaxAngle", 30);
    p.sensitivity = pConf->Read("Sensitivity", 100);

    ShowInfoDialog();
    m_XTE_refreshed = false;
    m_route_active = false;
    m_pilot_heading = -1.; // undefined
    m_vessel_heading = -1.; // current heading of vessel according to pilot, undefined
    m_XTE = 100000.; // undefined

    m_Timer.Connect(wxEVT_TIMER,
        wxTimerEventHandler(ATR_AIS_pi::OnTimer), NULL, this);
    m_Timer.Start(1000);

    //    This PlugIn needs a toolbar icon

#ifdef PLUGIN_USE_SVG
    m_tool_id = InsertPlugInToolSVG(_T( "ATR_AIS" ), _svg_tracking,
        _svg_tracking_toggled, _svg_tracking_toggled, wxITEM_NORMAL,
        _("Tracking"), _T( "Track Following for Raymarine Evolution Pilots" ),
        NULL, TRACKING_TOOL_POSITION, 0, this);
#else
    m_tool_id = InsertPlugInTool(_T(""), _img_ATR_AIS,
        _img_ATR_AIS, wxITEM_NORMAL, _("ATR_AIS"), _T(""),
        NULL, TRACKING_TOOL_POSITION, 0, this);
#endif

    SetStandby();
    m_initialized = true;

    // initialize NavMsg listeners
    //-----------------------------

    // Heading PGN 127250
    wxDEFINE_EVENT(EVT_N2K_127250, ObservedEvt);
    NMEA2000Id id_127250 = NMEA2000Id(127250);
    listener_127250 = std::move(GetListener(id_127250, EVT_N2K_127250, this));
    Bind(EVT_N2K_127250, [&](ObservedEvt ev) { HandleN2K_127250(ev); });

    // Pilot heading
    wxDEFINE_EVENT(EVT_N2K_65360, ObservedEvt);
    NMEA2000Id id_65360 = NMEA2000Id(65360);
    listener_65360 = std::move(GetListener(id_65360, EVT_N2K_65360, this));
    Bind(EVT_N2K_65360, [&](ObservedEvt ev) { HandleN2K_65360(ev); });

    // Set Set pilot heading or set auto/standby
    wxDEFINE_EVENT(EVT_N2K_126208, ObservedEvt);
    NMEA2000Id id_126208 = NMEA2000Id(126208);
    listener_126208 = std::move(GetListener(id_126208, EVT_N2K_126208, this));
    Bind(EVT_N2K_126208, [&](ObservedEvt ev) { HandleN2K_126208(ev); });

    // From EV1 (204) indicating auto or standby state
    wxDEFINE_EVENT(EVT_N2K_126720, ObservedEvt);
    NMEA2000Id id_126720 = NMEA2000Id(126720);
    listener_126720 = std::move(GetListener(id_126720, EVT_N2K_126720, this));
    Bind(EVT_N2K_126720, [&](ObservedEvt ev) { HandleN2K_126720(ev); });

    // Vessel heading, proprietary
    wxDEFINE_EVENT(EVT_N2K_65359, ObservedEvt);
    NMEA2000Id id_65359 = NMEA2000Id(65359);
    listener_65359 = std::move(GetListener(id_65359, EVT_N2K_65359, this));
    Bind(EVT_N2K_65359, [&](ObservedEvt ev) { HandleN2K_65359(ev); });

    //init ais listeners
    //may not be needed
    InitAISNMEA0183Listeners();
    wxLogMessage("Init after InitNMEA0183Listeners");
    AISTargets = NULL;
    /*  not available??
    if (AISTargets) {  // Init may be called more than once, check for cleanup
        wxLogMessage("Init start if(AISTargets)");
        wxLogMessage(wxT("AISTargets %s", AISTargets));
        WX_CLEAR_ARRAY(*AISTargets);
        delete AISTargets;
    }
    wxLogMessage("Init after if(AISTargets)");
    AISTargets = GetAISTargetArray();
    wxLogMessage("Init after GetAISTargetArray");*/
    m_TimerAIS.Connect(wxEVT_TIMER,
        wxTimerEventHandler(ATR_AIS_pi::OnTimerAIS), NULL, this);
    m_TimerAIS.Start(30000);  //30 second AIS updates



    found_handle = false;
    for (const auto& handle : GetActiveDrivers()) {
        const auto& attributes = GetAttributes(handle);
        if (attributes.find("protocol") == attributes.end())
            continue;
        wxLogMessage(wxT("handle proto %s"), attributes.at("protocol"));
        if (attributes.at("protocol") == "nmea2000") {
            m_handleN2k = handle;
            found_handle = true;
            break;
        }
    }
    if (!found_handle) wxLogMessage(wxT("nmea2000 handle not found"));
    //std::vector<int> pgn_list = { 127250, 126208 };
    std::vector<int> pgn_list = { 127250, 126208, 129038, 129039, 129041 };
    CommDriverResult xx = RegisterTXPGNs(m_handleN2k, pgn_list);

    wxLogMessage(wxT("ATR_AIS version %i.%i.%i"), GetPlugInVersionMajor(), GetPlugInVersionMinor(), GetPlugInVersionPatch());

    return (WANTS_OVERLAY_CALLBACK | WANTS_OPENGL_OVERLAY_CALLBACK
        | WANTS_CURSOR_LATLON | WANTS_NMEA_SENTENCES | WANTS_NMEA_EVENTS
        | WANTS_AIS_SENTENCES | WANTS_PLUGIN_MESSAGING | WANTS_PREFERENCES
        | WANTS_CONFIG);
    
}

static double oldpilotheading = 0.;

void ATR_AIS_pi::InitAISNMEA0183Listeners(void) {
    // Initialize the ais comm listeners N0183

    //NMEA0183
    //VDM
    /*wxDEFINE_EVENT(EVT_N0183_VDM, ObservedEvt);
    NMEA0183Id id_VDM = NMEA0183Id(VDM);
    //NMEA0183Id id("GPGGA");  from ocpn wiki
    listener_N0183_VDM.Listen(n0183_msg_VDM, this, EVT_N0183_VDM);
    Bind(EVT_N0183_VDM, [&](ObservedEvt ev) { Handle_N0183_VDM(ev); });*/

    /*Nmea0183Msg n0183_msg_VDM("VDM");
    listener_N0183_VDM = std::move(GetLIstener(id_VDM, this, EVT_N0183_VDM);
    Bind(EVT_N0183_VDM, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //FRPOS
    /*wxDEFINE_EVENT(EVT_N0183_FRPOS, ObservedEvt);
    NMEA0183Id id_FRPOS = NMEA0183Id(FRPOS);
    listener_N0183_FRPOS.Listen(n0183_msg_FRPOS, this, EVT_N0183_FRPOS);
    Bind(EVT_N0183_FRPOS, [&](ObservedEvt ev) { Handle_N0183_FRPOS(ev); });*/
    /*
    Nmea0183Msg n0183_msg_FRPOS("FRPOS");
    listener_N0183_FRPOS.Listen(n0183_msg_FRPOS, this, EVT_N0183_FRPOS);

    Bind(EVT_N0183_FRPOS, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //CDDSC
    /*wxDEFINE_EVENT(EVT_N0183_CDDSC, ObservedEvt);
    NMEA0183Id id_CDDSC = NMEA0183Id(CDDSC);
    listener_N0183_CDDSC.Listen(n0183_msg_CDDSC, this, EVT_N0183_CDDSC);
    Bind(EVT_N0183_CDDSC, [&](ObservedEvt ev) { Handle_N0183_CDDSC(ev); });
    /*
    Nmea0183Msg n0183_msg_CDDSC("CDDSC");
    listener_N0183_CDDSC.Listen(n0183_msg_CDDSC, this, EVT_N0183_CDDSC);
    Bind(EVT_N0183_CDDSC, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //CDDSE
    /*wxDEFINE_EVENT(EVT_N0183_CDDSE, ObservedEvt);
    NMEA0183Id id_CDDSE = NMEA0183Id(CDDSE);
    listener_N0183_CDDSE.Listen(n0183_msg_CDDSE, this, EVT_N0183_CDDSE);
    Bind(EVT_N0183_CDDSE, [&](ObservedEvt ev) { Handle_N0183_CDDSE(ev); });
    /*
    Nmea0183Msg n0183_msg_CDDSE("CDDSE");
    listener_N0183_CDDSE.Listen(n0183_msg_CDDSE, this, EVT_N0183_CDDSE);
    Bind(EVT_N0183_CDDSE, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //TLL
    /*wxDEFINE_EVENT(EVT_N0183_TLL, ObservedEvt);
    NMEA0183Id id_TLL = NMEA0183Id(TLL);
    listener_N0183_TLL.Listen(n0183_msg_TLL, this, EVT_N0183_TLL);
    Bind(EVT_N0183_TLL, [&](ObservedEvt ev) { Handle_N0183_TLL(ev); });
    /*
    Nmea0183Msg n0183_msg_TLL("TLL");
    listener_N0183_TLL.Listen(n0183_msg_TLL, this, EVT_N0183_TLL);

    Bind(EVT_N0183_TLL, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //TTM
    /*wxDEFINE_EVENT(EVT_N0183_TTM, ObservedEvt);
    NMEA0183Id id_TTM = NMEA0183Id(TTM);
    listener_N0183_TTM.Listen(n0183_msg_TTM, this, EVT_N0183_TTM);
    Bind(EVT_N0183_TTM, [&](ObservedEvt ev) { Handle_N0183_TTM(ev); });
    /*
    Nmea0183Msg n0183_msg_ttm("TTM");
    listener_N0183_TTM.Listen(n0183_msg_ttm, this, EVT_N0183_TTM);
    Bind(EVT_N0183_TTM, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //OSD
    /*wxDEFINE_EVENT(EVT_N0183_OSD, ObservedEvt);
    NMEA0183Id id_OSD = NMEA0183Id(OSD);
    listener_N0183_OSD.Listen(n0183_msg_OSD, this, EVT_N0183_OSD);
    Bind(EVT_N0183_OSD, [&](ObservedEvt ev) { Handle_N0183_OSD(ev); });
    /*
    Nmea0183Msg n0183_msg_OSD("OSD");
    listener_N0183_OSD.Listen(n0183_msg_OSD, this, EVT_N0183_OSD);
    Bind(EVT_N0183_OSD, [&](ObservedEvt ev) {
        auto ptr = ev.GetSharedPtr();
        auto n0183_msg = std::static_pointer_cast<const Nmea0183Msg>(ptr);
        HandleN0183_AIS(n0183_msg);
        });*/

    //SignalK
    /*SignalkMsg sk_msg;
    listener_SignalK.Listen(sk_msg, this, EVT_SIGNALK);
    Bind(EVT_SIGNALK, [&](ObservedEvt ev) {
        HandleSignalK(UnpackEvtPointer<SignalkMsg>(ev));
        });*/

}

// wxBitmap* ATR_AIS_pi::GetPlugInBitmap() { return m_pdeficon; }

wxString ATR_AIS_pi::GetCommonName() { return _T(PLUGIN_COMMON_NAME); }
int ATR_AIS_pi::GetAPIVersionMajor() {return OCPN_API_VERSION_MAJOR; }
int ATR_AIS_pi::GetAPIVersionMinor() { return OCPN_API_VERSION_MINOR; }
wxString ATR_AIS_pi::GetShortDescription() { return _(PLUGIN_SHORT_DESCRIPTION); }
wxString ATR_AIS_pi::GetLongDescription() { return _(PLUGIN_LONG_DESCRIPTION); }
int ATR_AIS_pi::GetPlugInVersionMajor() { return PLUGIN_VERSION_MAJOR; }
int ATR_AIS_pi::GetPlugInVersionMinor() { return PLUGIN_VERSION_MINOR; }
int ATR_AIS_pi::GetPlugInVersionPatch() { return PLUGIN_VERSION_PATCH; }

wxBitmap *ATR_AIS_pi::GetPlugInBitmap() { return &m_panelBitmap; }

// wxBitmap *ATR_AIS_pi::GetPlugInBitmap() { return m_pdeficon; }

void ATR_AIS_pi::OnToolbarToolCallback(int id)
{
    if (!m_initialized) {
        return;
    }
    if (m_info_dialog->IsShown()) {
        m_info_dialog->Hide();
    } else {
        m_info_dialog->Show();
    }
}

bool ATR_AIS_pi::DeInit(void)
{
    // No logging here, will crash OpenCPN in DoLogRecord because of illegal
    // pointer to file
    if (!m_initialized)
        return true;
    if (m_PreferencesDialog) {
        delete m_PreferencesDialog;
    }
    RemovePlugInTool(m_leftclick_tool_id);

    // save config
    wxFileConfig* pConf = GetOCPNConfigObject();
    pConf->SetPath(_T("/Settings/ATR_AIS"));

    if (m_info_dialog) {
        wxPoint p = m_info_dialog->GetPosition();
        pConf->Write("PosX", p.x);
        pConf->Write("PosY", p.y);
    }
    delete m_info_dialog;

    preferences& p = m_prefs;
    pConf->Write("MaxAngle", p.max_angle);
    pConf->Write("Sensitivity", p.sensitivity);

    m_Timer.Stop();
    m_Timer.Disconnect(wxEVT_TIMER,
        wxTimerEventHandler(ATR_AIS_pi::OnTimer), NULL, this);
    m_TimerAIS.Stop();
    m_TimerAIS.Disconnect(wxEVT_TIMER,
        wxTimerEventHandler(ATR_AIS_pi::OnTimerAIS), NULL, this);
    m_initialized = false;
    return true;
}

 // Vessel heading, standard NMEA2000
void ATR_AIS_pi::HandleN2K_127250(ObservedEvt ev){
    NMEA2000Id id_127250(127250);
    std::vector<uint8_t> msg = GetN2000Payload(id_127250, ev);
    double p_h = ((unsigned int)msg[14] + 256 * (unsigned int)msg[15]) * 360.
        / 3.141 / 20000;
    m_vessel_heading = p_h + m_var;
}

// 65360 Autopilot heading. From pilot. Transmitted only when pilot is Auto.
void ATR_AIS_pi::HandleN2K_65360(ObservedEvt ev)
{ // Vessel heading, standerd NMEA2000
    NMEA2000Id id_65360(65360);
    double p_h;
    std::vector<uint8_t> msg = GetN2000Payload(id_65360, ev);
    if (m_pilot_state == STANDBY) {
        SetAuto();
    }
    p_h = ((unsigned int)msg[18] + 256 * (unsigned int)msg[19]) * 360. / 3.141
        / 20000;
    m_pilot_heading = p_h + m_var; // received heading is magnetic
    if (oldpilotheading != m_pilot_heading) {
        oldpilotheading = m_pilot_heading;
    }
}

// case 126208: // if length is 28: command to set to standby or auto
//  if length is 25: command to set to heading
// heading = ((unsigned int)msg[12] + 256 * (unsigned int)msg[13]) * 360.
// / 3.141 / 20000;
void ATR_AIS_pi::HandleN2K_126208(ObservedEvt ev)
{
    NMEA2000Id id_126208(126208);
    std::vector<uint8_t> msg = GetN2000Payload(id_126208, ev);
    int msgLen = msg.size();
    if (msgLen == 28) { // should be the heading command
        // field 5 is the address of origin
        /*if (msg[5] != NGT1ADDRESS
            && m_pilot_state
                == TRACKING)*/
        // if we did not send the heading command ourselves, switch to AUTO
        SetAuto(); // if the user presses a +/- 1 or 10 pi we will switch from
                   // Tracking to Auto
        //wxLogMessage(wxT("     #received message 126208  len= %i \n"), msgLen);
        /*wxLogMessage(
            wxT("%0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, "
                "%0x, "
                "%0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x,%0x, %0x, %0x, %0x \n"),
            msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7],
            msg[8], msg[9], msg[10], msg[11], msg[12], msg[13], msg[14],
            msg[15], msg[16], msg[17], msg[18], msg[19], msg[20], msg[21],
            msg[22], msg[23], msg[24]);*/
    }

    if (msgLen == 31) { //    messages that originate from a keystroke auto
                        //     / standly
        //wxLogMessage(wxT("ATR_AIS_pi: length 31, f23=%0x, f24=%0x, "
        //                 "f25=%0x, f26=%0x, f27=%0x, f28=%0x, f29=%0x, len=%i"),
        //    msg[23], msg[24], msg[25], msg[26], msg[27], msg[28], msg[29],
        //    msgLen);
        if (msg[25] == 0x00 && m_pilot_state != STANDBY) { // +2 done
            SetStandby();
            m_pilot_heading = -1.; // undefined
        }
        if (msg[25] == 0x40) { // AUTO     // +2 done
            if (m_pilot_state == STANDBY) {
                SetAuto();
            } else {
                if (m_route_active) {
                    SetTracking();
                }
            }
        }
    }
}

//case 126720: // message from EV1 (204) indicating auto or standby state

void ATR_AIS_pi::HandleN2K_126720(ObservedEvt ev){
    NMEA2000Id id_126720(126720);
    std::vector<uint8_t> msg = GetN2000Payload(id_126720, ev);
    int msgLen = msg.size();
    if (msgLen != 27) {
        return;
    }
    /*wxLogMessage(wxT("%0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x, %0x,%0x, %0x, %0x, %0x \n"),
      msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], msg[8],
      msg[9], msg[10], msg[11], msg[12], msg[13], msg[14], msg[15], msg[16],
      msg[17], msg[18], msg[19], msg[20], msg[21], msg[22], msg[23], msg[24]);*/

    if (msg[21] == 0x40 && m_pilot_state != STANDBY) {  // +2 done
        SetStandby();
    }
    if (msg[21] == 0x42) { // AUTO    // +2 done
        if (m_pilot_state == STANDBY) {
            SetAuto();
        }
    }
}

//  heading all the time
void ATR_AIS_pi::HandleN2K_65359(ObservedEvt ev)
{
    NMEA2000Id id_65359(65359);
    std::vector<uint8_t> msg = GetN2000Payload(id_65359, ev);
    m_vessel_heading = (((unsigned int)msg[18] + 256 * (unsigned int)msg[19])
        * 360. / 3.141 / 20000) + m_var;
}

void ATR_AIS_pi::ShowPreferencesDialog(wxWindow* parent)
{
    if (NULL == m_PreferencesDialog)
        m_PreferencesDialog = new PreferencesDialog(parent, *this);

    m_PreferencesDialog->ShowModal();

    delete m_PreferencesDialog;
    m_PreferencesDialog = NULL;
}

void ATR_AIS_pi::OnTimer(wxTimerEvent&)
{
    wxWindow* canvas = GetCanvasByIndex(0);
    if (canvas) {
        canvas->Refresh(false);
    }
}

bool ATR_AIS_pi::RenderOverlay(wxDC& dc, PlugIn_ViewPort* vp)
{
    if (!m_initialized) {
        return true;
    }
    if (m_info_dialog) {
        m_info_dialog->UpdateInfo();
    }
    if (m_XTE_refreshed) {
        m_XTE_refreshed = false;
        Compute();
    }
    return true;
}

bool ATR_AIS_pi::RenderGLOverlay(
    wxGLContext* pcontext, PlugIn_ViewPort* vp)
{
    if (!m_initialized) {
        return true;
		}
    if (m_info_dialog) {
        m_info_dialog->UpdateInfo();
    }
    if (m_XTE_refreshed) {
        m_XTE_refreshed = false;
        Compute();
    }
    return true;
}

void ATR_AIS_pi::ShowInfoDialog()
{
    if (!m_info_dialog) {
        wxFileConfig* pConf = GetOCPNConfigObject();
        pConf->SetPath(_T("/Settings/ATR_AIS"));
        wxPoint pos(pConf->Read("PosX", 0L), pConf->Read("PosY", 50));

        m_info_dialog = new InfoDialog(GetOCPNCanvasWindow(), this);
        m_info_dialog->SetPosition(pos);
        m_info_dialog->EnableHeadingButtons(false);
        m_info_dialog->EnableTrackButton(false);
        wxSize sz = m_info_dialog->GetSize();
        m_info_dialog->Show();
    }
}

void ATR_AIS_pi::ShowPreferences()
{
    if (!m_PreferencesDialog) {
        m_PreferencesDialog
            = new PreferencesDialog(GetOCPNCanvasWindow(), *this);
        wxIcon icon;
        icon.CopyFromBitmap(*_img_ATR_AIS);
        m_PreferencesDialog->SetIcon(icon);
    }
    m_PreferencesDialog->Show();
}

wxString ATR_AIS_pi::StandardPath()
{
    wxStandardPathsBase& std_path = wxStandardPathsBase::Get();
    wxString s = wxFileName::GetPathSeparator();

#if defined(__WXMSW__)
    wxString stdPath = std_path.GetConfigDir();
#elif defined(__WXGTK__) || defined(__WXQT__)
    wxString stdPath = std_path.GetUserDataDir();
#elif defined(__WXOSX__)
    wxString stdPath = (std_path.GetUserConfigDir() + s + _T("opencpn"));
#endif

    stdPath += s + _T("plugins");
    if (!wxDirExists(stdPath))
        wxMkdir(stdPath);

    stdPath += s + _T("ATR_AIS");

#ifdef __WXOSX__
    // Compatibility with pre-OCPN-4.2; move config dir to
    // ~/Library/Preferences/opencpn if it exists
    wxString oldPath = (std_path.GetUserConfigDir() + s + _T("plugins") + s
        + _T("weatherfax"));
    if (wxDirExists(oldPath) && !wxDirExists(stdPath)) {
        wxLogMessage(
            "weatherfax_pi: moving config dir %s to %s", oldPath, stdPath);
        wxRenameFile(oldPath, stdPath);
    }
#endif

    if (!wxDirExists(stdPath))
        wxMkdir(stdPath);

    stdPath += s; // is this necessary?
    return stdPath;
}

void ATR_AIS_pi::SetPositionFixEx(PlugIn_Position_Fix_Ex& pfix)
{
    m_var = pfix.Var;
}

void ATR_AIS_pi::SetActiveLegInfo(Plugin_Active_Leg_Info& leg_info)
{
    //wxLogMessage(wxString("ATR_AIS_pi: SetActiveLegInfo called xte=%f, BTW= %f, DTW= %f, name= %s"),
     //   leg_info.Xte, leg_info.Btw, leg_info.Dtw, leg_info.wp_name);
    m_XTE = leg_info.Xte;
    if (isnan(m_XTE)) {
        m_XTE = 0.;
        //wxLogMessage(wxString("ATR_AIS_pi: m_XTE is NaN"));
    }
    if (m_XTE > -0.000001 && m_XTE < 0.)
        m_XTE = 0.;
    m_XTE_refreshed = true;
    //wxLogMessage(wxString("XTE refreshed"));
    m_route_active = true; // when SetActiveLegInfo is called a route must be active
    if (!isnan(leg_info.Btw)) {
        m_BTW = leg_info.Btw;
    }
    if (!isnan(leg_info.Dtw)) {
        m_DTW = leg_info.Dtw;
    }
}

void ATR_AIS_pi::SetPlugInMessage(
    wxString& message_id, wxString& message_body)
{
    if (message_id == wxS("ATR_AIS_pi")) {
        return; // nothing yet
    } else if (message_id == wxS("AIS")) {
        //??
        GetAISTargets();
        SendAISMessages(AISTargets);
    } else if (message_id == "OCPN_RTE_ACTIVATED") {
        ResetXTE();
        if (m_pilot_state == TRACKING) {
            SetStandby();
        }
        m_route_active = true;
        m_info_dialog->EnableTrackButton(true);
    } else if (message_id == "OCPN_WPT_ARRIVED") {
    } else if (message_id == "OCPN_RTE_DEACTIVATED"
        || message_id == "OCPN_RTE_ENDED") {
        m_route_active = false;
        m_XTE = 100000.; // undefined
        wxCommandEvent event;
        if (m_info_dialog) {
            if (m_pilot_state == TRACKING) {
                m_info_dialog->OnStandby(event);
            }
            m_info_dialog->EnableTrackButton(false);
        }
    }
}

void ATR_AIS_pi::SetStandby()
{
    m_pilot_state = STANDBY;
    if (m_info_dialog) {
        m_info_dialog->EnableHeadingButtons(false);
        m_info_dialog->EnableTrackButton(true);
    }
}

void ATR_AIS_pi::SetAuto()
{
    m_pilot_state = AUTO;
    if (m_info_dialog) {
        m_info_dialog->EnableHeadingButtons(true);
    }
}

void ATR_AIS_pi::SetTracking()
{
    m_pilot_state = TRACKING;
    if (m_info_dialog) {
        m_info_dialog->EnableHeadingButtons(true);
    }
    ResetXTE(); // reset local XTE calculations
    ZeroXTE(); // zero XTE on OpenCPN
}

void ATR_AIS_pi::Compute()
{
    double dist;
    double DTW = m_DTW * 1852.;
    double XTE_for_correction;
    if (isnan(m_BTW))
        return;
    if (isnan(m_XTE) || m_XTE == 100000.)
        return;
    if (m_pilot_state != TRACKING ) {
        return;
    }
    if (!m_route_active) return;
    dist = 50.; // in meters
    double dist_nm = dist / 1852.;
    // integration of XTE, but prevent increase of m_XTE_I when XTE is large
    if (m_XTE > -0.25 * dist_nm && m_XTE < 0.25 * dist_nm) {
        m_XTE_I += m_XTE;
    } else if (m_XTE > -0.5 * dist_nm && m_XTE < 0.5 * dist_nm) {
        m_XTE_I += 0.5 * m_XTE;
    } else if (m_XTE > -dist_nm && m_XTE < dist_nm) {
        m_XTE_I += 0.2 * m_XTE;
    } else {
    }; // do nothing for now

    m_XTE_D = m_XTE - m_XTE_P; // difference
    m_XTE_P = m_XTE; // proportional used as previous xte next timw

    if (m_XTE_I > 0.5 * dist_nm / I_FACTOR) { // in NM
        m_XTE_I = 0.5 * dist_nm / I_FACTOR;
    }
    if (m_XTE_I < -0.5 * dist_nm / I_FACTOR) { // in NM
        m_XTE_I = -0.5 * dist_nm / I_FACTOR;
    }

    XTE_for_correction = m_XTE + I_FACTOR * m_XTE_I + D_FACTOR * m_XTE_D;
    XTE_for_correction *= m_prefs.sensitivity / 100.;

    //wxLogMessage(wxT(" XTE_for_correction=%f, 5 * m_XTE=%f,  I_FACTOR *    m_XTE_I=%f, D_FACTOR * m_XTE_D=%f"),
     // XTE_for_correction, 5 * m_XTE, I_FACTOR * m_XTE_I, D_FACTOR *
     // m_XTE_D);
    if (DTW < 50.) {
        XTE_for_correction *= DTW / 50.;
    }
    if (DTW < 0.) {
        XTE_for_correction = 0.;
    }
    double gamma,
        new_bearing; // angle for correction of heading relative to BTW
    if (dist > 1.) {
        gamma = atan(XTE_for_correction * 1852. / dist) / (2. * 3.1416) * 360.;
    }
    else {
        gamma = 0.;
    }
    double max_angle = m_prefs.max_angle;
    // wxLogMessage(wxT("ATR_AIS initial gamma=%f, btw=%f,
    // dist=%f, max_angle= %f, XTE_for_correction=%f"), gamma, m_BTW, dist,
    // max_angle, XTE_for_correction);
    new_bearing = m_BTW + gamma; // bearing of next wp

    if (gamma > max_angle) {
        new_bearing = m_BTW + max_angle;
    } else if (gamma < -max_angle) {
        new_bearing = m_BTW - max_angle;
    }
    // don't turn too fast....

    if (!m_heading_set) { // after reset accept any turn
        m_current_bearing = new_bearing;
        m_heading_set = true;
    } else {
        while (new_bearing >= 360.)
            new_bearing -= 360.;
        while (new_bearing < 0.)
            new_bearing += 360.;
        double turnrate = TURNRATE;

        // turn left or right?
        double turn = new_bearing - m_current_bearing;

        if (turn < -180.)
            turn += 360;
        if (turn > 80. || turn < -80.)
            turnrate = 2 * TURNRATE;
        if (turn < -turnrate || (turn > 180. && turn < 360 - turnrate)) {
            // turn left
            m_current_bearing -= turnrate;
        } else if (turn > turnrate && turn <= 180.) {
            // turn right
            m_current_bearing += turnrate;
        } else {
            // go almost straight, correction < TURNRATE
            m_current_bearing = new_bearing;
        }
    }
    while (m_current_bearing >= 360.)
        m_current_bearing -= 360.;
    while (m_current_bearing < 0.)
        m_current_bearing += 360.;
    SetPilotHeading(
        m_current_bearing - m_var); // the commands used expect magnetic heading
    m_pilot_heading = m_current_bearing; // This should not be needed, pilot heading
                             // will come from pilot. For testing only.
    SendHSC(m_current_bearing);
}

void ATR_AIS_pi::ChangePilotHeading(int degrees)
{
    if (m_pilot_state == STANDBY) {
        return;
    }
    if (m_pilot_state
        == TRACKING) { // N.B.: for the pilot AUTO and TRACKING is the same
        SetAuto();
    }
    double new_pilot_heading = m_pilot_heading + (double)degrees;
    if (new_pilot_heading >= 360.)
        new_pilot_heading -= 360.;
    if (new_pilot_heading < 0.)
        new_pilot_heading += 360.;
    SetPilotHeading(
        new_pilot_heading - m_var); // send magnitic heading to Raymarine
    m_pilot_heading
        = new_pilot_heading; // this should not be needed, pilot heading
                             // will come from pilot. For testing only.
    SendHSC(new_pilot_heading);
}

// NMEA0183    NMEA0183;

void ATR_AIS_pi::SendHSC(double course)
{

    // For autopilots that aaccept this message, I do not know if they exist
    // Used for testing with a modified shipdriver_pi

    /*
    HSC - Heading Steering Command

    1   2 3   4  5
    |   | |   |  |
    $--HSC, x.x, T, x.x, M, *hh<CR><LF>

    Field Number :

    1 Heading Degrees, True

    2. T = True

    3. Heading Degrees, Magnetic

    4. M = Magnetic

    Checksum */

    wxString nmea;
    char sentence[40];
    char checksum = 0;
    char* p;

    snprintf(sentence, sizeof(sentence), "AUHSC,%.1f,T", course);

    for (p = sentence; *p; p++) {
        checksum ^= *p;
    }
    nmea.Printf(wxT("$%s*%02X\r\n"), sentence, (unsigned)checksum);
    PushNMEABuffer(nmea);
}

void ATR_AIS_pi::SetPilotHeading(double heading)
{
    // wxLogMessage(wxT("ATR_AIS_pi SetAutopilotHeading = %f"),
    // heading);

    // commands for NGT-1 in Canboat format
    //std::string standby_command
    //    = "Z,3,126208,7,204,17,01,63,ff,00,f8,04,01,3b,07,03,04,04,00,00,"
    //      "05,ff,ff"; // set standby
    // string auto_command =
    // "Z,3,126208,7,204,17,01,63,ff,00,f8,04,01,3b,07,03,04,04,40,00,05,ff,ff";
    // // set auto
    //std::string msg2 = " Z,3,126208,7,204,14,01,50,ff,00,f8,03,01,3b,07,03,04,06,00,00"; // set 0 magnetic
    // string msg3 =
    // "Z,3,126208,7,204,14,                   01,50,ff,00,f8,03,01,3b,07,03,04,06,9f,3e";
    // //set 92 magnetic string msg4 =
    // "Z,3,126208,7,204,14,01,50,ff,00,f8,03,01,3b,07,03,04,06,4e,3f";
    // //set 93 example only, magnetic

    std::shared_ptr<std::vector<uint8_t>> payload(new std::vector<uint8_t>(
                { 0x01, 0x50, 0xff, 0x00, 0xf8, 0x03, 0x01, 0x3b, 0x07, 0x03, 0x04, 0x06, 0x00, 0x00 }));
    //wxLogMessage(wxT("$$$ payload %0x, %0x, %0x"), payload->at(0), payload->at(1), payload->at(2));

    double heading_normal = heading;
    while (heading_normal < 0)  heading_normal += 360;
    while (heading_normal >= 360)
        heading_normal -= 360;
    uint16_t heading_radials1000 = (uint16_t)(heading_normal
        * 174.53); // heading to be set in thousands of radials
     //wxLogMessage(wxT("ATR_AIS_pi SetAutopilotHeading2 radials = %i %000x"), heading_radials1000, heading_radials1000);
    uint8_t byte0, byte1;
    byte0 = heading_radials1000 & 0xff;
    byte1 = heading_radials1000 >> 8;
    payload-> at(12) = byte0;
    payload-> at(13) = byte1;
    //wxLogMessage(wxT("ATR_AIS_pi SetAutopilotHeading byte0 = %0x, byte1 = %0x"), byte0, byte1);
    int PGN = 126208;
    WriteCommDriverN2K(m_handleN2k, PGN, 0xcc, 6, payload);
    std::fstream ftest;
    ftest.open("C:\\atemp\\OpenCPN_Dev\\aistest.txt", std::ios::app);
    time_t now = time(0);
    char* dt = ctime(&now);
    ftest << PGN << " : ";
    ftest << payload << " : ";
    ftest << dt;
    ftest << "\n\n";
    wxLogMessage("Pilot heading message sent");
    ftest.close();
}

void ATR_AIS_pi::SetPilotAuto(){
    std::shared_ptr<std::vector<uint8_t>> payload(new std::vector<uint8_t>({
        01, 0x63, 0xff, 0x00, 0xf8, 0x04, 0x01, 0x3b, 0x07, 0x03, 0x04, 0x04, 0x40, 0x00, 0x05, 0xff, 0xff}));
    //  01, 0x63, 0xff, 0x00, 0xf8, 0x04, 0x01, 0x3b, 0x07, 0x03, 0x04, 0x04, 0x00, 0x00, 0x05, 0xff, 0xff
    int PGN = 126208;
    WriteCommDriverN2K(m_handleN2k, PGN, 0xcc, 3, payload);
    std::fstream ftest;
    ftest.open("C:\\atemp\\OpenCPN_Dev\\aistest.txt", std::ios::app);
    time_t now = time(0);
    char* dt = ctime(&now);
    ftest << PGN << " : ";
    ftest << payload << " : ";
    ftest << dt;
    ftest << "\n\n";
    wxLogMessage("Pilot auto message sent");
    ftest.close();
}

void ATR_AIS_pi::SetPilotStandby()
{
    std::string standby_command
        = "Z,3,126208,7,204,17,01,63,ff,00,f8,04,01,3b,07,03,04,04,00,00,"
          "05,ff,ff"; // set standby
    std::shared_ptr<std::vector<uint8_t>> payload(
        new std::vector<uint8_t>({ 01, 0x63, 0xff, 0x00, 0xf8, 0x04, 0x01, 0x3b, 0x07, 0x03, 0x04, 0x04, 0x00, 0x00, 0x05, 0xff, 0xff }));
    // length = 17
    int PGN = 126208;
    WriteCommDriverN2K(m_handleN2k, PGN, 0xcc, 6, payload);
    std::fstream ftest;
    ftest.open("C:\\atemp\\OpenCPN_Dev\\aistest.txt", std::ios::app);
    time_t now = time(0);
    char* dt = ctime(&now);
    ftest << PGN << " : ";
    ftest << payload << " : ";
    ftest << dt;
    ftest << "\n\n";
    wxLogMessage("Pilot standby message sent");
    ftest.close();
}

void ATR_AIS_pi::SetP70Tracking()
{  // not tested, not used yet
    std::shared_ptr<std::vector<uint8_t>> payload(new std::vector<uint8_t>(
        { 0x3b, 0x9f, 0xf0, 0x81, 0x84, 0x46, 0x27, 0x9d, 0x4a, 0x00, 0x00, 0x02, 0x08, 0x4e }));
    wxLogMessage(wxT("ATR_AIS_pi set :Tracking"));
    int PGN = 126208;
    WriteCommDriverN2K(m_handleN2k, PGN, 0xcc, 6, payload);
    std::fstream ftest;
    ftest.open("C:\\atemp\\OpenCPN_Dev\\aistest.txt", std::ios::app);
    time_t now = time(0);
    char* dt = ctime(&now);
    ftest << PGN << " : ";
    ftest << payload << " : ";
    ftest << dt;
    ftest << "\n\n";
    wxLogMessage("P70 tracking message sent");
    ftest.close();
}

//**********************************************************
//*****   AIS   *************
//**********************************************************

void ATR_AIS_pi::OnTimerAIS(wxTimerEvent&)
{
    wxLogMessage("Start AIS ontimer calling GetAISTargets");
    GetAISTargets();
    wxLogMessage("after GetAISTargets");
    //wxLogMessage(wxT("AISTargetArray  %s", AISTargets));
    SendAISMessages(AISTargets);
    wxLogMessage("After SendAISMessage");
}

void ATR_AIS_pi::SendAISMessages(ArrayOfPlugIn_AIS_Targets* AisTargets) {
    ArrayOfPlugIn_AIS_Targets::iterator   it;
    PlugIn_AIS_Target *t;
    for (it = (*AISTargets).begin(); it != (*AISTargets).end(); ++it) {
        t = *it;
        wxLogMessage("AIS message send requested");
        SendAISN2k(t);
    }
}


ArrayOfPlugIn_AIS_Targets* ATR_AIS_pi::GetAISTargets() {
    wxLogMessage("start GetAISTargets");
    //if (AISTargets) {
        wxLogMessage("AISTargets exists");
        WX_CLEAR_ARRAY(*AISTargets);
        //delete AISTargets;
        wxLogMessage("AISTaregts deleted");
    //}
    //else {
        //wxLogMessage("AISTargets empty");
    //}
    AISTargets = GetAISTargetArray();
    if (AISTargets) {
        wxLogMessage("AIS targets acquired");
    }
    else {
        wxLogMessage("No AIS targets acquired");
    }
    
    return AISTargets;
}

void ATR_AIS_pi::SendAISN2k(PlugIn_AIS_Target* pTData) {

    tN2kMsg xxN2kMsg;
    std::fstream ftest;
    ftest.open("C:\\atemp\\OpenCPN_Dev\\aistest.txt", std::ios::app);
    time_t now = time(0);
    char* dt = ctime(&now);
    // test to see if handle available
    /*if (!found_n2k_handle) {
        GetN2kHandle();
    }*/
    if (!found_handle) {
        ftest << "no  handle";
        ftest << dt;
        ftest << "\n\n";
        ftest.close();
        return;
    }
    //std::shared_ptr<std::vector<uint8_t>> n2kpayload(new std::vector<uint8_t>);
    //std::shared_ptr<std::vector<uint8_t>> n2kpayload(new std::vector<uint8_t>(200));
    
    int i;
    // FILE *ftest;
    // ftest = fopen("C:\\atemp\\OpenCPN_Dev\\aistest.txt", "a");
    /*time_t nowtime;
    nowtime = time(NULL);
    if (nowtime - starttesttime > 120) {  // send test every 2 mins
        uint8_t MessageID = 1;
        tN2kAISRepeat Repeat =
            tN2kAISRepeat(0);  //?? 0=default 3=do not repeat any more  range 0-3
        uint32_t UserID = 338123456;
        double Latitude = -21.09707;  // position on slade island
        double Longitude = 149.244133;
        bool Accuracy = 1;     // assume high, <10m  0= low >10m
        bool RAIM = 0;         // not in use
        uint8_t Seconds = 60;  // not known
        double COG = DEG2RAD(223.4);
        double SOG = 6.123;  // KNOT2MS(pTData->SOG);
        double Heading = DEG2RAD(254.3);
        double ROT = 0.0;
        tN2kAISNavStatus NavStatus = N2kaisns_Under_Way_Motoring;
        // class A position report
        SetN2kPGN129038(xxN2kMsg, MessageID, Repeat, UserID, Latitude, Longitude,
            Accuracy, RAIM, Seconds, COG, SOG, Heading, ROT,
            NavStatus);
        int PGN = 129038;
        // for (const auto& ch : N2kMsg.Data) n2kpayload.push_back(ch);
        // N2kMsg.DataLen = sizeof(N2kMsg.Data);
        ftest << PGN << " : ";
        for (i = 0; i < xxN2kMsg.DataLen; i++) {
            //n2kpayload.push_back(xxN2kMsg.Data[i]);
            n2kpayload->at(i) = xxN2kMsg.Data[i];
            ftest << "0x" << std::setw(2) << std::setfill('0') << std::hex
                << int(xxN2kMsg.Data[i]) << " ";
        }
        ftest << " : " << dt;
        ftest << "\n";

        WriteCommDriverN2K(m_handleN2k, PGN, 0xff, 6,n2kpayload);  // was 0xcc for autopilot, 0xff=any
        // destination, 0x02=element display
        // fprintf(ftest, "%d %s\n",PGN, n2kpayload);
        ftest << PGN << " : ";
        ftest << n2kpayload << " : ";
        ftest << dt;
        ftest << "\n\n";
        starttesttime = nowtime;
    }*/

    if (pTData->Class == AIS_CLASS_A) {
        uint8_t MessageID = 1;      //pTData->MID;
        tN2kAISRepeat Repeat =
            tN2kAISRepeat(0);  //?? 0=default 3=do not repeat any more  range 0-3
        uint32_t UserID = pTData->MMSI;
        double Latitude = pTData->Lat;
        double Longitude = pTData->Lon;
        bool Accuracy = 1;     // assume high, <10m  0= low >10m
        bool RAIM = 0;         // not in use
        uint8_t Seconds = 60;  // not known
        // double COG = pTData->COG;
        double COG = DEG2RAD(pTData->COG);
        //***********//check to see if DEG2RAD required heading and
        // COG//***************//
        double SOG = pTData->SOG;  // KNOT2MS(pTData->SOG);
        // double Heading = pTData->HDG;
        double Heading = DEG2RAD(pTData->HDG);
        double ROT = pTData->ROTAIS;
        tN2kAISNavStatus NavStatus = tN2kAISNavStatus(pTData->NavStatus);
        // class A position report
        SetN2kPGN129038(xxN2kMsg, MessageID, Repeat, UserID, Latitude, Longitude,
            Accuracy, RAIM, Seconds, COG, SOG, Heading, ROT,
            NavStatus);
        int PGN = 129038;
        std::shared_ptr<std::vector<uint8_t>> n2kpayload(new std::vector<uint8_t>(xxN2kMsg.DataLen,0));
        ftest << PGN << " : ";
        for (i = 0; i < xxN2kMsg.DataLen; i++) {
            n2kpayload->at(i) = xxN2kMsg.Data[i];
            ftest << "0x" << std::setw(2) << std::setfill('0') << std::hex<< int(xxN2kMsg.Data[i]) << " ";
        }
        ftest << " : " << dt;
        ftest << "\n";
        
        WriteCommDriverN2K(m_handleN2k, PGN, 0xff, 6,n2kpayload);  // was 0xcc for autopilot, 0xff=any
        
        ftest << PGN << " : ";
        ftest << n2kpayload << " : ";
        ftest << dt;
        ftest << "\n\n";
        wxLogMessage("AIS Class A message sent");
    }
    else if (pTData->Class == AIS_CLASS_B) {
        uint8_t MessageID = 18;    //pTData->MID;
        tN2kAISRepeat Repeat = tN2kAISRepeat(0);
        uint32_t UserID = pTData->MMSI;
        double Latitude = pTData->Lat;
        double Longitude = pTData->Lon;
        bool Accuracy = 1;
        bool RAIM = 0;
        uint8_t Seconds = 60;
        double COG = DEG2RAD(pTData->COG);
        double SOG = pTData->SOG;  // KNOT2MS(pTData->SOG);
        tN2kAISTransceiverInformation AISTransceiverInformation =
            tN2kAISTransceiverInformation::N2kaisown_information_not_broadcast;
        double Heading = DEG2RAD(pTData->HDG);
        tN2kAISUnit Unit = tN2kAISUnit(0);  // assume SOTDMA
        bool Display = 0;                   // no display available
        bool DSC = 0;                       // not equipped with DSC
        bool Band = 0;                      // irrevelant if msg22==0;
        bool Msg22 = 0;                     // no freq management
        tN2kAISMode Mode = tN2kAISMode(0);  // autonomous mode
        bool State = 0;  // guess should be filled with 1100000000000000110;
        // class B position report
        xxN2kMsg.DataLen = 0;
        for (i = 0; i < xxN2kMsg.MaxDataLen; i++) {
            xxN2kMsg.Data[i] = 0;
        }
        SetN2kPGN129039(xxN2kMsg, MessageID, Repeat, UserID, Latitude, Longitude,Accuracy, RAIM, Seconds, COG, SOG,AISTransceiverInformation, Heading, Unit, Display, DSC,Band, Msg22, Mode, State);
        int PGN = 129039;
        std::shared_ptr<std::vector<uint8_t>> n2kpayload(new std::vector<uint8_t>(xxN2kMsg.DataLen, 0));
        ftest << PGN << " : ";
        for (i = 0; i < xxN2kMsg.DataLen; i++) {
            n2kpayload->at(i) = xxN2kMsg.Data[i];
            ftest << "0x" << std::setw(2) << std::setfill('0') << std::hex<< int(xxN2kMsg.Data[i]) << " ";
        }
        ftest << " : " << dt;
        ftest << "\n";
        WriteCommDriverN2K(m_handleN2k, PGN, 0xff, 6, n2kpayload);
        ftest << PGN << " : ";
        ftest << n2kpayload << " : ";
        ftest << dt;
        ftest << "\n\n";
        wxLogMessage("AIS Class B message sent");
    }
    else if (pTData->Class == AIS_ATON) {
        tN2kAISAtoNReportData vvN2kData;
        vvN2kData.MessageID = 21;      //pTData->MID;
        vvN2kData.Repeat = tN2kAISRepeat(0);
        vvN2kData.UserID = pTData->MMSI;
        vvN2kData.Longitude = pTData->Lon;
        vvN2kData.Latitude = pTData->Lat;
        vvN2kData.Accuracy = 1;
        vvN2kData.RAIM = 0;
        vvN2kData.Seconds = 60;
        vvN2kData.Length = 0;
        vvN2kData.Beam = 0;
        vvN2kData.PositionReferenceStarboard = 0;
        vvN2kData.PositionReferenceTrueNorth = 0;
        vvN2kData.AtoNType = tN2kAISAtoNType(0);  // 0=not available
        vvN2kData.OffPositionIndicator = 0;
        vvN2kData.VirtualAtoNFlag = 0;         // 0=real  1=virtual
        vvN2kData.AssignedModeFlag = 0;        // 0=autonomous
        vvN2kData.GNSSType = tN2kGNSStype(0);  // undefined
        vvN2kData.AtoNStatus = 0;              // 00000000
        vvN2kData.AISTransceiverInformation =
            tN2kAISTransceiverInformation(0);  // guess
        strcpy(vvN2kData.AtoNName, pTData->ShipName);

        // AtoN
        SetN2kPGN129041(xxN2kMsg, vvN2kData);
        int PGN = 129041;
        std::shared_ptr<std::vector<uint8_t>> n2kpayload(new std::vector<uint8_t>(xxN2kMsg.DataLen, 0));
        ftest << PGN << " : ";
        for (i = 0; i < xxN2kMsg.DataLen; i++) {
            n2kpayload->at(i) = xxN2kMsg.Data[i];
            ftest << "0x" << std::setw(2) << std::setfill('0') << std::hex<< int(xxN2kMsg.Data[i]) << " ";
        }
        ftest << " : " << dt;
        ftest << "\n";
        WriteCommDriverN2K(m_handleN2k, PGN, 0x02, 6, n2kpayload);
        ftest << PGN << " : ";
        ftest << n2kpayload << " : ";
        ftest << dt;
        ftest << "\n\n";
        wxLogMessage("AIS AtoN message sent");
    }
    ftest.close();
}
