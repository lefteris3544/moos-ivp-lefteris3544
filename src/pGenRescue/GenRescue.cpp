/************************************************************/
/*    NAME: Terry Alifragkis                                */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: June 23, 2026                                   */
/************************************************************/

#include <iterator>
#include <algorithm>
#include <limits>
#include <cmath>
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
  m_returning = false;
  m_planner = "greedy";
  m_concede_adversary = false;
  m_concede_count = 0;
  m_concede_path_count = 0;
  m_concede_radius = 0;
  m_concede_heading_window = 0;
  m_competitive_weight = 0.35;
  m_competitive_lost_margin = 10;
  m_contact_replan_interval = 1.0;
  m_path_repost_interval = 5.0;
  m_field_cx = -96.5;
  m_field_cy = -19.5;
  m_field_margin = 4.0;
  m_max_speed = 1.2;
  m_adaptive_speed = false;
  m_cruise_speed = 1.2;
  m_slow_speed = 1.2;
  m_pivot_speed = 1.2;
  m_sharp_turn_angle = 70;
  m_pivot_turn_angle = 115;
  m_slow_down_range = 25;
  m_speed_post_interval = 1.2;
  m_contact_x = 0;
  m_contact_y = 0;
  m_contact_hdg = 0;
  m_contact_xy_set = false;
  m_contact_hdg_set = false;
  m_last_contact_replan_time = 0;
  m_wpt_index = -1;
  m_wpt_dist = 0;
  m_wpt_stat_received = false;
  m_current_speed = -1;
  m_last_speed_post_time = 0;
  m_last_path_repost_time = 0;

  initMap();
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
    else if(key == "WPT_STAT")
      handleMailWptStat(sval);
    else if(key == "RETURN") {
      bool return_val = false;
      handled = setBooleanOnString(return_val, sval);
      if(handled) {
        m_returning = return_val;
        if(!m_returning && (getPendingSwimmerCount() > 0))
          m_plan_pending = true;
      }
    }
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

  if(m_returning) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }
  
  if(m_plan_pending && m_nav_x_set && m_nav_y_set)
    postShortestPath();

  bool have_pending = (getPendingSwimmerCount() > 0);
  double elapsed = MOOSTime() - m_last_path_repost_time;
  if(have_pending && m_nav_x_set && m_nav_y_set &&
     (m_path_repost_interval > 0) &&
     (elapsed >= m_path_repost_interval))
    postShortestPath();

  updateAdaptiveSpeed();

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
    else if(param == "competitive_weight")
      m_competitive_weight = atof(value.c_str());
    else if(param == "competitive_lost_margin")
      m_competitive_lost_margin = atof(value.c_str());
    else if(param == "contact_replan_interval")
      m_contact_replan_interval = atof(value.c_str());
    else if(param == "path_repost_interval")
      m_path_repost_interval = atof(value.c_str());
    else if(param == "field_margin")
      m_field_margin = atof(value.c_str());
    else if(param == "max_speed")
      m_max_speed = atof(value.c_str());
    else if(param == "adaptive_speed")
      setBooleanOnString(m_adaptive_speed, value);
    else if(param == "cruise_speed")
      m_cruise_speed = atof(value.c_str());
    else if(param == "slow_speed")
      m_slow_speed = atof(value.c_str());
    else if(param == "pivot_speed")
      m_pivot_speed = atof(value.c_str());
    else if(param == "sharp_turn_angle")
      m_sharp_turn_angle = atof(value.c_str());
    else if(param == "pivot_turn_angle")
      m_pivot_turn_angle = atof(value.c_str());
    else if(param == "slow_down_range")
      m_slow_down_range = atof(value.c_str());
    else if(param == "speed_post_interval")
      m_speed_post_interval = atof(value.c_str());
  }

  if(m_vname == "")
    m_vname = m_host_community;

  if(m_max_speed <= 0)
    m_max_speed = 1.2;
  
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
  Register("WPT_STAT", 0);
  Register("RETURN", 0);
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
    double sx = x;
    double sy = y;
    getSafePoint(x, y, sx, sy);

    XYPoint point(sx, sy);
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
  m_contacts[m_contact_name] = XYPoint(m_contact_x, m_contact_y);

  if(record.isSetHeading()) {
    m_contact_hdg = record.getHeading();
    m_contact_hdg_set = true;
  }

  bool competitive = (m_planner == "competitive");
  bool have_pending = (m_swimmers.size() > m_found_swimmers.size());
  double elapsed = MOOSTime() - m_last_contact_replan_time;
  bool replan_due = (elapsed >= m_contact_replan_interval);

  if(m_concede_adversary || (competitive && have_pending && replan_due)) {
    m_plan_pending = true;
    m_last_contact_replan_time = MOOSTime();
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailWptStat()

void GenRescue::handleMailWptStat(string stat)
{
  if(strContains(stat, "behavior-name=waypt_survey") &&
     strContains(stat, "index=")) {
    m_wpt_index = atoi(tokStringParse(stat, "index", ',', '=').c_str());
    string dist = tokStringParse(stat, "dist", ',', '=');
    if(isNumber(dist))
      m_wpt_dist = atof(dist.c_str());
    m_wpt_stat_received = true;
  }
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

  if(m_planner == "competitive")
    m_path = buildCompetitivePath(pending, m_nav_x, m_nav_y);
  else if(m_planner == "two_hop")
    m_path = buildGreedyTwoHopPath(pending, m_nav_x, m_nav_y);
  else
    m_path = greedyPath(pending, m_nav_x, m_nav_y);

  m_path.set_label("rescue_path_" + m_vname);
  postPath();
  m_last_path_repost_time = MOOSTime();
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

      if(!segmentIsSafe(curr_x, curr_y, ix, iy))
        score += 500.0;

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
// Procedure: buildCompetitivePath()

XYSegList GenRescue::buildCompetitivePath(XYSegList pending,
                                          double start_x,
                                          double start_y) const
{
  if(!m_contact_xy_set)
    return(buildGreedyTwoHopPath(pending, start_x, start_y));

  XYSegList path;
  double curr_x = start_x;
  double curr_y = start_y;

  while(pending.size() > 0) {
    unsigned int best_ix = 0;
    double best_score = numeric_limits<double>::max();
    bool have_winnable = false;

    for(unsigned int i=0; i<pending.size(); i++) {
      double ix = pending.get_vx(i);
      double iy = pending.get_vy(i);
      double own_dist = distPointToPoint(curr_x, curr_y, ix, iy);
      if(!segmentIsSafe(curr_x, curr_y, ix, iy))
        own_dist += 500.0;
      double contact_dist = getNearestContactDist(ix, iy);
      bool lost = ((contact_dist + m_competitive_lost_margin) < own_dist);
      if(!lost)
        have_winnable = true;
    }

    for(unsigned int i=0; i<pending.size(); i++) {
      double ix = pending.get_vx(i);
      double iy = pending.get_vy(i);
      double own_dist = distPointToPoint(curr_x, curr_y, ix, iy);
      if(!segmentIsSafe(curr_x, curr_y, ix, iy))
        own_dist += 500.0;
      double contact_dist = getNearestContactDist(ix, iy);
      bool lost = ((contact_dist + m_competitive_lost_margin) < own_dist);
      if(lost && have_winnable)
        continue;

      double own_advantage = contact_dist - own_dist;
      double score = own_dist;
      if(own_advantage > 0)
        score -= (m_competitive_weight * own_advantage);

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
        score += 0.25 * nearest_next;
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
// Procedure: getNearestContactDist()

double GenRescue::getNearestContactDist(double x, double y) const
{
  if(m_contacts.size() == 0)
    return(distPointToPoint(m_contact_x, m_contact_y, x, y));

  double nearest = numeric_limits<double>::max();
  map<string, XYPoint>::const_iterator p;
  for(p=m_contacts.begin(); p!=m_contacts.end(); p++) {
    double dist = distPointToPoint(p->second.x(), p->second.y(), x, y);
    if(dist < nearest)
      nearest = dist;
  }

  return(nearest);
}

//---------------------------------------------------------
// Procedure: initMap()

void GenRescue::initMap()
{
  m_field_poly.clear();
  m_obstacles.clear();

  m_field_poly.push_back(RescuePoint{-205, -4});
  m_field_poly.push_back(RescuePoint{ -76,-86});
  m_field_poly.push_back(RescuePoint{ -16,  6});
  m_field_poly.push_back(RescuePoint{ -79,  4});

  m_obstacles.push_back(RescueObstacle{"buoy_1", -35.0,  -6.0, 5.0, 10.0});
  m_obstacles.push_back(RescueObstacle{"buoy_2", -62.5, -17.9, 5.0, 10.0});
  m_obstacles.push_back(RescueObstacle{"buoy_3", -95.0, -28.0, 5.0, 10.0});
}

//---------------------------------------------------------
// Procedure: pointSegDist()

double GenRescue::pointSegDist(double px, double py,
                               double x1, double y1,
                               double x2, double y2) const
{
  double vx = x2 - x1;
  double vy = y2 - y1;
  double wx = px - x1;
  double wy = py - y1;

  double len2 = (vx * vx) + (vy * vy);
  if(len2 <= 0.0001)
    return(distPointToPoint(px, py, x1, y1));

  double t = ((wx * vx) + (wy * vy)) / len2;
  if(t < 0)
    t = 0;
  if(t > 1)
    t = 1;

  double cx = x1 + (t * vx);
  double cy = y1 + (t * vy);

  return(distPointToPoint(px, py, cx, cy));
}

//---------------------------------------------------------
// Procedure: pointInField()

bool GenRescue::pointInField(double x, double y) const
{
  bool inside = false;
  unsigned int n = m_field_poly.size();

  if(n < 3)
    return(true);

  for(unsigned int i=0, j=n-1; i<n; j=i++) {
    double xi = m_field_poly[i].x;
    double yi = m_field_poly[i].y;
    double xj = m_field_poly[j].x;
    double yj = m_field_poly[j].y;

    bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 0.000001) + xi);

    if(intersect)
      inside = !inside;
  }

  return(inside);
}

//---------------------------------------------------------
// Procedure: fieldBoundaryDist()

double GenRescue::fieldBoundaryDist(double x, double y) const
{
  if(m_field_poly.size() < 2)
    return(9999);

  double best = numeric_limits<double>::max();
  unsigned int n = m_field_poly.size();

  for(unsigned int i=0; i<n; i++) {
    unsigned int j = (i + 1) % n;

    double d = pointSegDist(x, y,
                            m_field_poly[i].x, m_field_poly[i].y,
                            m_field_poly[j].x, m_field_poly[j].y);
    if(d < best)
      best = d;
  }

  return(best);
}

//---------------------------------------------------------
// Procedure: pointIsSafe()

bool GenRescue::pointIsSafe(double x, double y) const
{
  if(!pointInField(x, y))
    return(false);

  if(fieldBoundaryDist(x, y) < m_field_margin)
    return(false);

  for(unsigned int i=0; i<m_obstacles.size(); i++) {
    const RescueObstacle& obs = m_obstacles[i];
    if(distPointToPoint(x, y, obs.x, obs.y) < obs.target_radius)
      return(false);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: segmentIsSafe()

bool GenRescue::segmentIsSafe(double x1, double y1,
                              double x2, double y2) const
{
  if(!pointIsSafe(x2, y2))
    return(false);

  for(unsigned int i=1; i<=20; i++) {
    double t = (double)(i) / 20.0;
    double x = x1 + t * (x2 - x1);
    double y = y1 + t * (y2 - y1);

    if(!pointInField(x, y))
      return(false);

    if(fieldBoundaryDist(x, y) < 1.0)
      return(false);
  }

  for(unsigned int i=0; i<m_obstacles.size(); i++) {
    const RescueObstacle& obs = m_obstacles[i];
    double d = pointSegDist(obs.x, obs.y, x1, y1, x2, y2);
    if(d < obs.segment_radius)
      return(false);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: getSafePoint()

void GenRescue::getSafePoint(double x, double y,
                             double& sx, double& sy) const
{
  sx = x;
  sy = y;

  if(pointIsSafe(sx, sy))
    return;

  double vx = m_field_cx - x;
  double vy = m_field_cy - y;
  double vd = sqrt((vx * vx) + (vy * vy));

  if(vd > 0.001) {
    double cx = x + 4.0 * vx / vd;
    double cy = y + 4.0 * vy / vd;

    if(pointIsSafe(cx, cy)) {
      sx = cx;
      sy = cy;
      return;
    }
  }

  double best_x = sx;
  double best_y = sy;
  double best_d = numeric_limits<double>::max();
  const double pi_val = 3.14159265358979323846;

  for(double rad=1.0; rad<=5.0; rad+=0.5) {
    for(unsigned int k=0; k<72; k++) {
      double ang = (2.0 * pi_val * (double)(k)) / 72.0;
      double cx = x + rad * cos(ang);
      double cy = y + rad * sin(ang);

      if(!pointIsSafe(cx, cy))
        continue;

      double cd = distPointToPoint(x, y, cx, cy);
      if(cd < best_d) {
        best_d = cd;
        best_x = cx;
        best_y = cy;
      }
    }
  }

  if(best_d < numeric_limits<double>::max()) {
    sx = best_x;
    sy = best_y;
    return;
  }

  if(vd > 0.001) {
    sx = x + 4.0 * vx / vd;
    sy = y + 4.0 * vy / vd;
  }
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
  if(!m_returning) {
    Notify("DEPLOY", "true");
    Notify("STATION_KEEP", "false");
  }

  m_wpt_stat_received = false;
  m_wpt_index = -1;
  postSpeedUpdate(m_max_speed);
}

//---------------------------------------------------------
// Procedure: updateAdaptiveSpeed()

void GenRescue::updateAdaptiveSpeed()
{
  if(!m_wpt_stat_received || (m_wpt_index < 0))
    return;
  if(m_path.size() == 0)
    return;

  double elapsed = MOOSTime() - m_last_speed_post_time;
  bool changed = (fabs(m_max_speed - m_current_speed) > 0.01);
  if(changed || (elapsed >= m_speed_post_interval))
    postSpeedUpdate(m_max_speed);
}

//---------------------------------------------------------
// Procedure: postSpeedUpdate()

void GenRescue::postSpeedUpdate(double speed)
{
  Notify("SURVEY_UPDATE", "speed=" + doubleToStringX(speed, 2));
  m_current_speed = speed;
  m_last_speed_post_time = MOOSTime();
}

//---------------------------------------------------------
// Procedure: calcTurnAngle()

double GenRescue::calcTurnAngle(unsigned int ix) const
{
  if((m_path.size() < 2) || ((ix + 1) >= m_path.size()))
    return(0);

  double ax = m_nav_x;
  double ay = m_nav_y;
  if(ix > 0) {
    ax = m_path.get_vx(ix - 1);
    ay = m_path.get_vy(ix - 1);
  }
  double bx = m_path.get_vx(ix);
  double by = m_path.get_vy(ix);
  double cx = m_path.get_vx(ix + 1);
  double cy = m_path.get_vy(ix + 1);

  double hdg1 = relAng(ax, ay, bx, by);
  double hdg2 = relAng(bx, by, cx, cy);
  double delta = angle180(hdg2 - hdg1);
  if(delta < 0)
    delta *= -1;

  return(delta);
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
  m_msgs << "Returning:           " << boolToString(m_returning) << endl;
  m_msgs << "Planner:             " << m_planner << endl;
  m_msgs << "Concede adversary:   " << boolToString(m_concede_adversary) << endl;
  m_msgs << "Concede nearest N:   " << m_concede_count << endl;
  m_msgs << "Concede path N:      " << m_concede_path_count << endl;
  m_msgs << "Concede radius:      " << doubleToStringX(m_concede_radius, 1) << endl;
  m_msgs << "Concede hdg window:  "
         << doubleToStringX(m_concede_heading_window, 1) << endl;
  m_msgs << "Competitive weight:  "
         << doubleToStringX(m_competitive_weight, 2) << endl;
  m_msgs << "Lost margin/replan:  "
         << doubleToStringX(m_competitive_lost_margin, 1) << " / "
         << doubleToStringX(m_contact_replan_interval, 1) << endl;
  m_msgs << "Path repost interval:"
         << doubleToStringX(m_path_repost_interval, 1) << endl;
  m_msgs << "Field margin:        "
         << doubleToStringX(m_field_margin, 1) << endl;
  m_msgs << "Adaptive speed:      disabled" << endl;
  m_msgs << "Max speed:           "
         << doubleToStringX(m_max_speed, 2) << endl;
  m_msgs << "Configured speeds:   "
         << doubleToStringX(m_cruise_speed, 2) << " / "
         << doubleToStringX(m_slow_speed, 2) << " / "
         << doubleToStringX(m_pivot_speed, 2) << endl;
  m_msgs << "Sharp/Pivot angle:   "
         << doubleToStringX(m_sharp_turn_angle, 1) << " / "
         << doubleToStringX(m_pivot_turn_angle, 1) << endl;
  m_msgs << "Slow down range:     "
         << doubleToStringX(m_slow_down_range, 1) << endl;
  m_msgs << "Current speed:       "
         << doubleToStringX(m_current_speed, 2) << endl;
  if(m_wpt_stat_received) {
    unsigned int ix = 0;
    if(m_wpt_index > 0)
      ix = (unsigned int)m_wpt_index;
    m_msgs << "WPT index/dist/turn: "
           << m_wpt_index << " / "
           << doubleToStringX(m_wpt_dist, 1) << " / "
           << doubleToStringX(calcTurnAngle(ix), 1) << endl;
  }
  else
    m_msgs << "WPT index/dist/turn: unset" << endl;
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
  m_msgs << "Known contacts:      " << m_contacts.size() << endl;
  m_msgs << "NAV_X/Y set:         "
         << boolToString(m_nav_x_set && m_nav_y_set) << endl;
  return(true);
}
