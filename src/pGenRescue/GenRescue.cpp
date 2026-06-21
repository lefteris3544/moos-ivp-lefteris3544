/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#include <iterator>
#include "GenRescue.h"
#include "MBUtils.h"
#include "ColorParse.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "GeomUtils.h"
#include "PathUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFieldGenerator.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GenRescue::GenRescue()
{
  // Initialize state variables
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_x_set = 0;
  m_nav_y_set = 0;
  m_plan_pending = false;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    string sval = msg.GetString();

    bool handled = true;
    if((key == "SWIMMER_ALERT") || (key == "MOB_ALERT")) 
      handled = handleMailNewSwimmer(sval);
    else if(key == "FOUND_SWIMMER") 
      handled = handleMailFoundSwimmer(sval);
    else if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_set = true;
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_set = true;
    }

    else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
      handled = false;
    
    if(!handled)
      reportRunWarning("Unhandled Mail: " + key +"=" + sval);
    
  }
  return(true);
}
 
//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  if(m_plan_pending && m_nav_x_set && m_nav_y_set)
    postShortestPath();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp(); 

  STRING_LIST sParams;
  m_MissionReader.GetConfiguration(GetAppName(), sParams);
  
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string sLine  = *p;
    string param  = tolower(biteStringX(sLine, '='));
    string value  = sLine;
    if(param == "vname")
      m_vname = value;
  }
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables()

void GenRescue::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SWIMMER_ALERT", 0);
  Register("MOB_ALERT", 0);
  Register("FOUND_SWIMMER", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//---------------------------------------------------------
// Procedure: handleMailNewSwimmer()

bool GenRescue::handleMailNewSwimmer(string str)
{
  string id = tokStringParse(str, "id", ',', '=');
  double x = tokDoubleParse(str, "x", ',', '=');
  double y = tokDoubleParse(str, "y", ',', '=');

  if(id == "")
    return(false);

  if(m_swimmers.count(id) == 0) {
    XYPoint point(x, y);
    point.set_label(id);
    m_swimmers[id] = point;
    m_plan_pending = true;
    reportEvent("New swimmer: id=" + id);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailFoundSwimmer()

bool GenRescue::handleMailFoundSwimmer(string str)
{
  string id = tokStringParse(str, "id", ',', '=');

  if(id == "")
    return(false);

  if(m_found_swimmers.count(id) == 0) {
    m_found_swimmers.insert(id);
    m_plan_pending = true;
    reportEvent("Found swimmer: id=" + id);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: postShortestPath()

void GenRescue::postShortestPath()
{
  XYSegList pending;

  map<string, XYPoint>::const_iterator p;
  for(p=m_swimmers.begin(); p!=m_swimmers.end(); p++) {
    string id = p->first;
    if(m_found_swimmers.count(id) == 0)
      pending.add_vertex(p->second.x(), p->second.y());
  }

  if((pending.size() == 0) && (m_swimmers.size() == 0)) {
    m_plan_pending = false;
    return;
  }

  if(pending.size() == 0) {
    postNullPath();
    return;
  }

  m_path = greedyPath(pending, m_nav_x, m_nav_y);
  m_path.set_label("rescue_path_" + m_vname);
  postPath();
  m_plan_pending = false;
}

//---------------------------------------------------------
// Procedure: postNullPath()
//   Purpose: If a found swimmer represents the last swimmer
//            to be found, then post a survey update essentially
// 

void GenRescue::postNullPath()
{
  XYSegList segl;
  segl.add_vertex(m_nav_x, m_nav_y);
  
  segl.set_label("rescue_path_" + m_vname);
  m_path = segl;
  postPath();
  m_plan_pending = false;
}

//---------------------------------------------------------
// Procedure: postPath()

void GenRescue::postPath()
{
  Notify("VIEW_SEGLIST", m_path.get_spec());

  string update_var = "SURVEY_UPDATE";
  string update_str = "points = " + m_path.get_spec_pts();

  Notify(update_var, update_str);
  reportEvent("SURVEY_UPDATE=" + update_str);
}


//---------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport()
{
  m_msgs << "Known swimmers: " << m_swimmers.size() << endl;
  m_msgs << "Found swimmers: " << m_found_swimmers.size() << endl;
  m_msgs << "Path points:    " << m_path.size() << endl;
  m_msgs << "NAV_X/Y set:    " << boolToString(m_nav_x_set && m_nav_y_set) << endl;
  return(true);
}
