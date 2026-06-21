/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#include <iterator>
#include <algorithm>
#include <limits>
#include "GenRescue.h"
#include "MBUtils.h"
#include "ColorParse.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "PathUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFieldGenerator.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"

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
  m_planner = "greedy";
  m_concede_adversary = false;
  m_concede_count = 0;
  m_concede_path_count = 0;
  m_concede_radius = 0;
  m_concede_heading_window = 0;
  m_contact_x = 0;
  m_contact_y = 0;
  m_contact_hdg = 0;
  m_contact_xy_set = false;
  m_contact_hdg_set = false;
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
    else if(key == "NODE_REPORT")
      handled = handleMailNodeReport(sval);
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
    else if(param == "planner")
      m_planner = tolower(value);
    else if(param == "concede_adversary")
      setBooleanOnString(m_concede_adversary, value);
    else if(param == "concede_count")
      m_concede_count = (unsigned int)atoi(value.c_str());
    else if(param == "concede_path_count")
      m_concede_path_count = (unsigned int)atoi(value.c_str());
    else if(param == "concede_radius")
      m_concede_radius = atof(value.c_str());
    else if(param == "concede_heading_window")
      m_concede_heading_window = atof(value.c_str());
  }

  if(m_vname == "")
    m_vname = m_host_community;
  
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
  Register("NODE_REPORT", 0);
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
// Procedure: handleMailNodeReport()

bool GenRescue::handleMailNodeReport(string str)
{
  NodeRecord record = string2NodeRecord(str);

  if(!record.valid("name,x,y"))
    return(false);

  if(record.getName() == m_vname)
    return(true);

  m_contact_name = record.getName();
  m_contact_x = record.getX();
  m_contact_y = record.getY();
  m_contact_xy_set = true;

  if(record.isSetHeading()) {
    m_contact_hdg = record.getHeading();
    m_contact_hdg_set = true;
  }

  if(m_concede_adversary)
    m_plan_pending = true;

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
  m_conceded_swimmers = getConcededSwimmers();
  XYSegList pending = buildPendingPath();

  if((pending.size() == 0) &&
     (m_found_swimmers.size() < m_swimmers.size()) &&
     (m_conceded_swimmers.size() > 0)) {
    m_conceded_swimmers.clear();
    pending = buildPendingPath();
  }

  if((pending.size() == 0) && (m_swimmers.size() == 0)) {
    m_plan_pending = false;
    return;
  }

  if(pending.size() == 0) {
    postNullPath();
    return;
  }

  if(m_planner == "two_hop")
    m_path = buildGreedyTwoHopPath(pending, m_nav_x, m_nav_y);
  else
    m_path = greedyPath(pending, m_nav_x, m_nav_y);

  m_path.set_label("rescue_path_" + m_vname);
  postPath();
  m_plan_pending = false;
}

//---------------------------------------------------------
// Procedure: buildPendingPath()

XYSegList GenRescue::buildPendingPath() const
{
  XYSegList pending;

  map<string, XYPoint>::const_iterator p;
  for(p=m_swimmers.begin(); p!=m_swimmers.end(); p++) {
    string id = p->first;
    if((m_found_swimmers.count(id) == 0) &&
       (m_conceded_swimmers.count(id) == 0))
      pending.add_vertex(p->second.x(), p->second.y());
  }

  return(pending);
}

//---------------------------------------------------------
// Procedure: buildGreedyTwoHopPath()

XYSegList GenRescue::buildGreedyTwoHopPath(XYSegList pending,
                                           double start_x,
                                           double start_y) const
{
  XYSegList path;
  double curr_x = start_x;
  double curr_y = start_y;

  while(pending.size() > 0) {
    unsigned int best_ix = 0;
    double best_score = numeric_limits<double>::max();

    for(unsigned int i=0; i<pending.size(); i++) {
      double ix = pending.get_vx(i);
      double iy = pending.get_vy(i);
      double score = distPointToPoint(curr_x, curr_y, ix, iy);

      if(pending.size() > 1) {
        double nearest_next = numeric_limits<double>::max();
        for(unsigned int j=0; j<pending.size(); j++) {
          if(i == j)
            continue;
          double jx = pending.get_vx(j);
          double jy = pending.get_vy(j);
          double dist = distPointToPoint(ix, iy, jx, jy);
          if(dist < nearest_next)
            nearest_next = dist;
        }
        score += nearest_next;
      }

      if(score < best_score) {
        best_score = score;
        best_ix = i;
      }
    }

    double vx = pending.get_vx(best_ix);
    double vy = pending.get_vy(best_ix);
    path.add_vertex(vx, vy);
    pending.delete_vertex(best_ix);
    curr_x = vx;
    curr_y = vy;
  }

  return(path);
}

//---------------------------------------------------------
// Procedure: getConcededSwimmers()

set<string> GenRescue::getConcededSwimmers() const
{
  set<string> conceded;

  if(!m_concede_adversary || !m_contact_xy_set)
    return(conceded);

  vector<pair<double, string> > candidates;
  XYSegList adversary_pending;
  map<string, string> point_to_id;

  map<string, XYPoint>::const_iterator p;
  for(p=m_swimmers.begin(); p!=m_swimmers.end(); p++) {
    string id = p->first;
    XYPoint point = p->second;

    if(m_found_swimmers.count(id) != 0)
      continue;

    string point_key = doubleToString(point.x(), 3) + "," +
      doubleToString(point.y(), 3);
    point_to_id[point_key] = id;
    adversary_pending.add_vertex(point.x(), point.y());

    double dist = distPointToPoint(m_contact_x, m_contact_y,
                                   point.x(), point.y());

    bool concede = false;
    if((m_concede_radius > 0) && (dist <= m_concede_radius))
      concede = true;

    if((m_concede_heading_window > 0) && m_contact_hdg_set) {
      double bearing = relAng(m_contact_x, m_contact_y,
                              point.x(), point.y());
      double delta = angle180(bearing - m_contact_hdg);
      if(delta < 0)
        delta *= -1;
      if(delta <= m_concede_heading_window)
        concede = true;
    }

    if(concede)
      conceded.insert(id);

    if(m_concede_count > 0)
      candidates.push_back(pair<double, string>(dist, id));
  }

  sort(candidates.begin(), candidates.end());

  for(unsigned int i=0; i<candidates.size(); i++) {
    string id = candidates[i].second;
    if(i < m_concede_count)
      conceded.insert(id);
  }

  if(m_concede_path_count > 0) {
    XYSegList adversary_path = greedyPath(adversary_pending,
                                          m_contact_x, m_contact_y);
    unsigned int path_count = m_concede_path_count;
    if(path_count > adversary_path.size())
      path_count = adversary_path.size();

    for(unsigned int i=0; i<path_count; i++) {
      string point_key = doubleToString(adversary_path.get_vx(i), 3) + "," +
        doubleToString(adversary_path.get_vy(i), 3);
      if(point_to_id.count(point_key) != 0)
        conceded.insert(point_to_id[point_key]);
    }
  }

  return(conceded);
}

//---------------------------------------------------------
// Procedure: stringFromSet()

string GenRescue::stringFromSet(set<string> values) const
{
  string result;

  set<string>::const_iterator p;
  for(p=values.begin(); p!=values.end(); p++) {
    if(result != "")
      result += ",";
    result += *p;
  }

  if(result == "")
    result = "none";

  return(result);
}

//---------------------------------------------------------
// Procedure: getPendingSwimmerCount()

unsigned int GenRescue::getPendingSwimmerCount() const
{
  unsigned int pending = 0;

  map<string, XYPoint>::const_iterator p;
  for(p=m_swimmers.begin(); p!=m_swimmers.end(); p++) {
    string id = p->first;
    if((m_found_swimmers.count(id) == 0) &&
       (m_conceded_swimmers.count(id) == 0))
      pending++;
  }

  return(pending);
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
  m_msgs << "Vehicle:             " << m_vname << endl;
  m_msgs << "Known swimmers:      " << m_swimmers.size() << endl;
  m_msgs << "Pending swimmers:    " << getPendingSwimmerCount() << endl;
  m_msgs << "Found swimmers:      " << m_found_swimmers.size()
         << " [" << stringFromSet(m_found_swimmers) << "]" << endl;
  m_msgs << "Conceded swimmers:   " << m_conceded_swimmers.size()
         << " [" << stringFromSet(m_conceded_swimmers) << "]" << endl;
  m_msgs << "Path points:         " << m_path.size() << endl;
  m_msgs << "Planner:             " << m_planner << endl;
  m_msgs << "Concede adversary:   " << boolToString(m_concede_adversary) << endl;
  m_msgs << "Concede nearest N:   " << m_concede_count << endl;
  m_msgs << "Concede path N:      " << m_concede_path_count << endl;
  m_msgs << "Concede radius:      " << doubleToStringX(m_concede_radius, 1) << endl;
  m_msgs << "Concede hdg window:  "
         << doubleToStringX(m_concede_heading_window, 1) << endl;
  m_msgs << "Contact:             " << m_contact_name << endl;
  if(m_contact_xy_set) {
    m_msgs << "Contact position:    "
           << doubleToStringX(m_contact_x, 1) << ","
           << doubleToStringX(m_contact_y, 1) << endl;
  }
  else
    m_msgs << "Contact position:    unset" << endl;
  if(m_contact_hdg_set)
    m_msgs << "Contact heading:     "
           << doubleToStringX(m_contact_hdg, 1) << endl;
  else
    m_msgs << "Contact heading:     unset" << endl;
  m_msgs << "NAV_X/Y set:         "
         << boolToString(m_nav_x_set && m_nav_y_set) << endl;
  return(true);
}
