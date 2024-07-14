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

#ifndef _ATRAISPI_H_
#define _ATRAISPI_H_

#include "wx/wx.h"

#include <wx/fileconf.h>

#include "version.h"
#include "wxWTranslateCatalog.h"
//#include "observable_evtvar.h"

#define ABOUT_AUTHOR_URL 
#define OPC wxS("opencpn-ATR_AIS_pi")

#include <list>
#include <map>


#ifndef WXINTL_NO_GETTEXT_MACRO
#ifdef OPC
#ifdef _
#undef _
#endif // _
#define _(s) wxGetTranslation((s),OPC)
#endif // OPC
#else 
#define _(s) wxGetTranslation((s))
#endif // WXINTL_NO_GETTEXT_MACRO


#include "ocpn_plugin.h"

#ifdef __MSVC__
//#include <string>
#include "msvcdefs.h"
#endif

//----------------------------------
//    The PlugIn Class Definition
//----------------------------------

//#define MY_API_VERSION_MAJOR 1
//#define MY_API_VERSION_MINOR 18

class apDC;
class ConsoleCanvas;
class PreferencesDialog;
class InfoDialog;
/*class PlugIn_AIS_Target {
public:
    int MMSI;
    int Class;
    int NavStatus;
    double SOG;
    double COG;
    double HDG;
    double Lon;
    double Lat;
    int ROTAIS;
    char CallSign[8];  // includes terminator
    char ShipName[21];
    unsigned char ShipType;
    int IMO;
};*/

//      Describe Transponder Class
typedef enum ais_transponder_class {
    AIS_CLASS_A = 0,
    AIS_CLASS_B,
    AIS_ATON,        // Aid to Navigation   pjotrc 2010/02/01
    AIS_BASE,        // Base station
    AIS_GPSG_BUDDY,  // GpsGate Buddy object
    AIS_DSC,         // DSC target
    AIS_SART,        // SART
    AIS_ARPA,        // ARPA radar target
    AIS_APRS         // APRS position report
} _ais_transponder_class;


//WX_DEFINE_ARRAY_PTR(PlugIn_AIS_Target*, ArrayOfPlugIn_AIS_Targets);

class ObservableListener;

class ATR_AIS_pi : public wxEvtHandler, public opencpn_plugin_118
{
  //friend InfoDialog;
   

private:
    //std::shared_ptr<ObservableListener> listener;

    void HandleN2K_65360 (ObservedEvt ev); // Pilot heading
    void HandleN2K_126208(ObservedEvt ev); // Set Set pilot heading or set auto/standby
    void HandleN2K_126720(ObservedEvt ev); // From EV1 (204) indicating auto or standby state
    void HandleN2K_65359 (ObservedEvt ev); // Vessel heading, proprietary
    void HandleN2K_127250(ObservedEvt ev); // Vessel heading, standerd NMEA2000

    std::shared_ptr<ObservableListener> listener_65360;  // Autopilot heading if auto
    std::shared_ptr<ObservableListener> listener_126208; // Set pilot heading or set auto/standby
    std::shared_ptr<ObservableListener> listener_126720; // From EV1 (204) indicating auto or standby state
    std::shared_ptr<ObservableListener> listener_65359;  // Vessel heading
    std::shared_ptr<ObservableListener> listener_127250; // Vessel heading

    /*std::string prio127245;
    std::string prio127257;
    std::string prio128259;
    std::string prio128267;
    std::string prio129029;
    std::string prioN2kPGNsat;
    std::string prio130306;*/

    wxString m_self;

    std::shared_ptr< ObservableListener> listener_N0183_VDM;
    std::shared_ptr< ObservableListener> listener_N0183_FRPOS;
    std::shared_ptr< ObservableListener> listener_N0183_CDDSC;
    std::shared_ptr< ObservableListener> listener_N0183_CDDSE;
    std::shared_ptr< ObservableListener> listener_N0183_TLL;
    std::shared_ptr< ObservableListener> listener_N0183_TTM;
    std::shared_ptr< ObservableListener> listener_N0183_OSD;
    std::shared_ptr< ObservableListener> listener_SignalK;


public:
  ATR_AIS_pi(void *ppimgr);
  int Init(void);
  bool DeInit(void);
  DriverHandle m_handleN2k;

  int GetAPIVersionMajor();
  int GetAPIVersionMinor();
  int GetPlugInVersionMajor();
  int GetPlugInVersionMinor();
  int GetPlugInVersionPatch();
  wxBitmap *GetPlugInBitmap();
  wxString GetCommonName();
  wxString GetShortDescription();
  wxString GetLongDescription();
  wxBitmap m_panelBitmap;  

  void ShowPreferencesDialog(wxWindow* parent);

  //wxBitmap* GetPlugInBitmap();
 

  bool RenderOverlay(wxDC &dc, PlugIn_ViewPort *vp);
  bool RenderGLOverlay(wxGLContext *pcontext, PlugIn_ViewPort *vp);

  void ShowInfoDialog();
  void ShowPreferences();
  void ChangePilotHeading(int degrees);
  void SetStandby();
  void SetAuto();
  void SetTracking();
  void SetPilotHeading(double heading);
  void SetPilotAuto();
  void SetPilotStandby();
  void SetP70Tracking();

  static wxString StandardPath();

  PlugIn_Position_Fix_Ex &LastFix() { return m_lastfix; }

  ArrayOfPlugIn_AIS_Targets* GetAISTargets();
  void OnTimerAIS(wxTimerEvent&);
  void SendAISMessages(ArrayOfPlugIn_AIS_Targets* AisTargets);
  void SendAISN2k(PlugIn_AIS_Target* pTData);
  ArrayOfPlugIn_AIS_Targets *AISTargets;
  void InitAISNMEA0183Listeners(void);

  DriverHandle m_N2khandle;
  double m_XTE, m_BTW, m_DTW;
  bool m_XTE_refreshed;
  bool m_heading_set;
  double m_var;
  bool m_initialized;
  bool m_route_active;
  double m_pilot_heading;  // target heading of pilot in auto mode
  double m_vessel_heading;        // current heading of vessel according to pilot
  
  double m_XTE_P, m_XTE_I, m_XTE_D;   // proportional, integral and differential factors
  //enum PilotState { UNKNOWN, STANDB, TRACKING, AUTO, test} m_pilot_state;  does not function in some classes 
  uint16_t m_pilot_state; // 0 standby, 1 auto, 2 tracking
#define STANDBY 0
#define AUTO 1
#define TRACKING 2
#define I_FACTOR 0.0075   // was 0.3, instable occillations
#define D_FACTOR 5.

public:
  void ResetXTE() {
      m_XTE = 0.;  m_XTE_P = 0.;  m_XTE_I = 0.; m_XTE_D = 0.; m_heading_set = false;
  }
  
    // these are stored to the config
    struct preferences {
        double max_angle;
        double sensitivity;
    } m_prefs;
    
    PreferencesDialog *m_PreferencesDialog;
    InfoDialog *m_info_dialog;
    void OnToolbarToolCallback(int id);

    // Icons
    wxString m_shareLocn;
    int m_tool_id;
    wxBitmap *m_pdeficon;
#define TRACKING_TOOL_POSITION -1  // Request default positioning of toolbar tool

protected:   
    wxPoint m_cursor_position;
    PlugIn_Position_Fix_Ex m_lastfix;
    void OnTimer(wxTimerEvent &);
    wxTimer m_Timer;
    wxTimer m_TimerAIS;

private:
    void SetPositionFixEx(PlugIn_Position_Fix_Ex &pfix);
    void SetPlugInMessage(wxString &message_id, wxString &message_body);
    void SetActiveLegInfo(Plugin_Active_Leg_Info &leg_info);
    void Compute();
    void SendHSC(double course);
    int m_leftclick_tool_id;
    double m_current_bearing;
};

#endif
