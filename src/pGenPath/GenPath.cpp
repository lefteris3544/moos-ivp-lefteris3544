/************************************************************/
/*    NAME: Lefteris                                       */
/*    ORGN: MIT, Cambridge MA                              */
/*    FILE: GenPath.cpp                                    */
/*    DATE: December 29th, 1963                            */
/************************************************************/

#include <cmath>
#include <limits>
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYSegList.h"
#include "GenPath.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GenPath::GenPath()
{
  m_updates_var   = "WPT_UPDATE";
  m_activate_var  = "VISIT";
  m_visit_radius  = 3.0;
  m_default_nav_x = 0;
  m_default_nav_y = 0;
  m_adaptive_speed = true;
  m_cruise_speed = 0.5;
  m_slow_speed = 0.25;
  m_pivot_speed = 0.15;
  m_sharp_turn_angle = 70;
  m_pivot_turn_angle = 115;
  m_slow_down_range = 25;
  m_speed_post_interval = 1.0;
  m_pivot_turn = true;
  m_pivot_heading_error = 45;
  m_pivot_release_error = 12;
  m_pivot_min_dist = 5;
  m_pivot_thrust = 80;

  m_invalid_points  = 0;
  m_tours_generated = 0;
  m_first_received  = false;
  m_last_received   = false;
  m_nav_received    = false;
  m_heading_received = false;
  m_desired_heading_received = false;
  m_wpt_stat_received = false;
  m_tour_generated  = false;
  m_tour_started    = false;
  m_pivot_active    = false;
  m_nav_x           = 0;
  m_nav_y           = 0;
  m_nav_heading     = 0;
  m_desired_heading = 0;
  m_heading_error   = 0;
  m_pivot_dir       = 0;
  m_wpt_index       = -1;
  m_wpt_dist        = 0;
  m_current_speed   = -1;
  m_last_speed_post_time = 0;
  m_last_update_post_time = 0;
}

//---------------------------------------------------------
// Destructor

GenPath::~GenPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "VISIT_POINT")
      handleVisitPoint(msg.GetString());
    else if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_received = true;
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_received = true;
    }
    else if(key == "NAV_HEADING") {
      m_nav_heading = msg.GetDouble();
      m_heading_received = true;
    }
    else if(key == "DESIRED_HEADING") {
      m_desired_heading = msg.GetDouble();
      m_desired_heading_received = true;
    }
    else if(key == "WPT_STAT")
      handleWptStat(msg.GetString());
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenPath::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(m_tour_generated && !m_tour_started) {
    double elapsed = MOOSTime() - m_last_update_post_time;
    if(elapsed >= 1.0)
      postTourUpdate();
  }
  bool pivoting = updatePivotControl();
  if(!pivoting)
    updateAdaptiveSpeed();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = stripBlankEnds(line);

    bool handled = false;
    if(param == "updates_var") {
      m_updates_var = value;
      handled = true;
    }
    else if(param == "activate_var") {
      m_activate_var = value;
      handled = true;
    }
    else if(param == "visit_radius") {
      m_visit_radius = atof(value.c_str());
      handled = true;
    }
    else if(param == "default_nav_x") {
      m_default_nav_x = atof(value.c_str());
      handled = true;
    }
    else if(param == "default_nav_y") {
      m_default_nav_y = atof(value.c_str());
      handled = true;
    }
    else if(param == "adaptive_speed") {
      m_adaptive_speed = setBooleanOnString(m_adaptive_speed, value);
      handled = true;
    }
    else if(param == "cruise_speed") {
      m_cruise_speed = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "slow_speed") {
      m_slow_speed = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_speed") {
      m_pivot_speed = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "sharp_turn_angle") {
      m_sharp_turn_angle = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_turn_angle") {
      m_pivot_turn_angle = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "slow_down_range") {
      m_slow_down_range = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "speed_post_interval") {
      m_speed_post_interval = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_turn") {
      m_pivot_turn = setBooleanOnString(m_pivot_turn, value);
      handled = true;
    }
    else if(param == "pivot_heading_error") {
      m_pivot_heading_error = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_release_error") {
      m_pivot_release_error = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_min_dist") {
      m_pivot_min_dist = atof(value.c_str());
      handled = isNumber(value);
    }
    else if(param == "pivot_thrust") {
      m_pivot_thrust = atof(value.c_str());
      handled = isNumber(value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("WPT_STAT", 0);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint()

void GenPath::handleVisitPoint(string point)
{
  point = stripBlankEnds(point);

  if(point == "firstpoint") {
    m_points.clear();
    m_tour.clear();
    m_invalid_points = 0;
    m_first_received = true;
    m_last_received  = false;
    m_tour_generated = false;
    m_tour_started   = false;
    m_wpt_stat_received = false;
    m_wpt_index = -1;
    m_current_speed = -1;
    endPivotControl();
    m_latest_update  = "";
    return;
  }

  if(point == "lastpoint") {
    m_last_received = true;
    generateTour();
    return;
  }

  VisitPoint visit_point;
  if(!parseVisitPoint(point, visit_point)) {
    m_invalid_points++;
    reportRunWarning("Invalid VISIT_POINT: " + point);
    return;
  }

  m_points.push_back(visit_point);
}

//---------------------------------------------------------
// Procedure: handleWptStat()

void GenPath::handleWptStat(string stat)
{
  if(strContains(stat, "behavior-name=waypt_survey") &&
     strContains(stat, "index=")) {
    m_tour_started = true;
    m_wpt_index = atoi(tokStringParse(stat, "index", ',', '=').c_str());
    string dist = tokStringParse(stat, "dist", ',', '=');
    if(isNumber(dist))
      m_wpt_dist = atof(dist.c_str());
    m_wpt_stat_received = true;
  }
}

//---------------------------------------------------------
// Procedure: parseVisitPoint()

bool GenPath::parseVisitPoint(string point, VisitPoint& visit_point) const
{
  bool ok_x = false;
  bool ok_y = false;

  vector<string> svector = parseString(point, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string pair  = stripBlankEnds(svector[i]);
    string key   = tolower(stripBlankEnds(biteStringX(pair, '=')));
    string value = stripBlankEnds(pair);

    if(key == "x") {
      visit_point.x = atof(value.c_str());
      ok_x = isNumber(value);
    }
    else if(key == "y") {
      visit_point.y = atof(value.c_str());
      ok_y = isNumber(value);
    }
    else if(key == "id")
      visit_point.id = value;
  }

  visit_point.visited = false;
  return(ok_x && ok_y);
}

//---------------------------------------------------------
// Procedure: generateTour()

void GenPath::generateTour()
{
  if(m_points.empty()) {
    reportRunWarning("Cannot generate tour: no visit points received.");
    return;
  }

  m_tour = buildGreedyTour();

  XYSegList seglist;
  for(unsigned int i=0; i<m_tour.size(); i++)
    seglist.add_vertex(m_tour[i].x, m_tour[i].y);

  m_latest_update = "points = " + seglist.get_spec();
  m_tours_generated++;
  m_tour_generated = true;
  m_tour_started = false;
  postTourUpdate();
}

//---------------------------------------------------------
// Procedure: updatePivotControl()

bool GenPath::updatePivotControl()
{
  if(!m_pivot_turn || !m_tour_generated || !m_tour_started ||
     !m_nav_received || !m_heading_received) {
    endPivotControl();
    return(false);
  }

  if(!m_desired_heading_received && (!m_wpt_stat_received || (m_wpt_index < 0))) {
    endPivotControl();
    return(false);
  }

  if(!m_desired_heading_received && ((unsigned int)m_wpt_index >= m_tour.size())) {
    endPivotControl();
    return(false);
  }

  double desired_heading = m_desired_heading;
  if(!m_desired_heading_received) {
    unsigned int ix = (unsigned int)m_wpt_index;
    desired_heading = calcHeadingToPoint(m_tour[ix]);
  }
  m_heading_error = angle180(desired_heading - m_nav_heading);
  double abs_error = fabs(m_heading_error);

  bool far_enough = (!m_wpt_stat_received || (m_wpt_dist >= m_pivot_min_dist));
  if(!far_enough) {
    endPivotControl();
    return(false);
  }

  if(m_pivot_active &&
     ((abs_error <= m_pivot_release_error) ||
      (m_heading_error * m_pivot_dir < 0))) {
    endPivotControl();
  }

  if(!m_pivot_active && (abs_error >= m_pivot_heading_error)) {
    m_pivot_active = true;
    m_pivot_dir = (m_heading_error >= 0) ? 1.0 : -1.0;
  }

  if(!m_pivot_active)
    return(false);

  double thrust = vclip(m_pivot_thrust, 0, 100);

  Notify("THRUST_MODE_DIFFERENTIAL", "true");
  Notify("DESIRED_THRUST_L", m_pivot_dir * thrust);
  Notify("DESIRED_THRUST_R", -m_pivot_dir * thrust);

  if(fabs(m_current_speed) > 0.01)
    postSpeedUpdate(0);

  return(true);
}

//---------------------------------------------------------
// Procedure: endPivotControl()

void GenPath::endPivotControl()
{
  if(!m_pivot_active)
    return;

  Notify("THRUST_MODE_DIFFERENTIAL", "false");
  Notify("DESIRED_THRUST_L", 0.0);
  Notify("DESIRED_THRUST_R", 0.0);
  m_pivot_active = false;
  m_pivot_dir = 0;
  m_current_speed = -1;
}

//---------------------------------------------------------
// Procedure: updateAdaptiveSpeed()

void GenPath::updateAdaptiveSpeed()
{
  if(!m_adaptive_speed || !m_tour_generated || !m_tour_started)
    return;
  if(!m_wpt_stat_received || (m_wpt_index < 0))
    return;

  unsigned int ix = (unsigned int)m_wpt_index;
  if(ix >= m_tour.size())
    ix = m_tour.size() - 1;

  double turn_angle = calcTurnAngle(ix);
  bool close_to_turn = (m_wpt_dist <= m_slow_down_range);
  bool sharp_turn = (turn_angle >= m_sharp_turn_angle);
  bool pivot_turn = (turn_angle >= m_pivot_turn_angle);
  double target_speed = m_cruise_speed;
  if(close_to_turn && pivot_turn)
    target_speed = m_pivot_speed;
  else if(close_to_turn && sharp_turn)
    target_speed = m_slow_speed;

  double elapsed = MOOSTime() - m_last_speed_post_time;
  bool changed = (std::fabs(target_speed - m_current_speed) > 0.01);
  if(changed || (elapsed >= m_speed_post_interval))
    postSpeedUpdate(target_speed);
}

//---------------------------------------------------------
// Procedure: postSpeedUpdate()

void GenPath::postSpeedUpdate(double speed)
{
  Notify(m_updates_var, "speed=" + doubleToStringX(speed, 2));
  m_current_speed = speed;
  m_last_speed_post_time = MOOSTime();
}

//---------------------------------------------------------
// Procedure: postTourUpdate()

void GenPath::postTourUpdate()
{
  if(m_latest_update == "")
    return;

  Notify(m_updates_var, m_latest_update);
  if(m_activate_var != "")
    Notify(m_activate_var, "true");
  Notify("LOITER", "false");
  Notify("RETURN", "false");
  Notify("STATION_KEEP", "false");
  m_last_update_post_time = MOOSTime();
}

//---------------------------------------------------------
// Procedure: buildGreedyTour()

vector<VisitPoint> GenPath::buildGreedyTour() const
{
  vector<VisitPoint> unvisited = m_points;
  vector<VisitPoint> tour;

  double curr_x = m_nav_received ? m_nav_x : m_default_nav_x;
  double curr_y = m_nav_received ? m_nav_y : m_default_nav_y;

  while(!unvisited.empty()) {
    unsigned int best_ix = 0;
    double best_dist = numeric_limits<double>::max();

    for(unsigned int i=0; i<unvisited.size(); i++) {
      double dist = distToPoint(curr_x, curr_y, unvisited[i]);
      if(dist < best_dist) {
        best_dist = dist;
        best_ix = i;
      }
    }

    VisitPoint next_point = unvisited[best_ix];
    next_point.visited = false;
    tour.push_back(next_point);
    curr_x = next_point.x;
    curr_y = next_point.y;
    unvisited.erase(unvisited.begin() + best_ix);
  }

  return(tour);
}

//---------------------------------------------------------
// Procedure: calcHeadingToPoint()

double GenPath::calcHeadingToPoint(const VisitPoint& point) const
{
  double dx = point.x - m_nav_x;
  double dy = point.y - m_nav_y;
  double heading = atan2(dx, dy) * 180.0 / M_PI;
  return(angle360(heading));
}

//---------------------------------------------------------
// Procedure: calcTurnAngle()

double GenPath::calcTurnAngle(unsigned int ix) const
{
  if((ix == 0) || ((ix + 1) >= m_tour.size()))
    return(0);

  double ax = m_tour[ix].x - m_tour[ix-1].x;
  double ay = m_tour[ix].y - m_tour[ix-1].y;
  double bx = m_tour[ix+1].x - m_tour[ix].x;
  double by = m_tour[ix+1].y - m_tour[ix].y;
  double amag = sqrt((ax * ax) + (ay * ay));
  double bmag = sqrt((bx * bx) + (by * by));
  if((amag <= 0) || (bmag <= 0))
    return(0);

  double dot = ((ax * bx) + (ay * by)) / (amag * bmag);
  if(dot > 1)
    dot = 1;
  else if(dot < -1)
    dot = -1;

  double direction_change = acos(dot) * 180.0 / M_PI;
  return(direction_change);
}

//---------------------------------------------------------
// Procedure: angle180()

double GenPath::angle180(double angle) const
{
  angle = angle360(angle);
  if(angle > 180)
    angle -= 360;
  return(angle);
}

//---------------------------------------------------------
// Procedure: angle360()

double GenPath::angle360(double angle) const
{
  while(angle < 0)
    angle += 360;
  while(angle >= 360)
    angle -= 360;
  return(angle);
}

//---------------------------------------------------------
// Procedure: distToPoint()

double GenPath::distToPoint(double x, double y, const VisitPoint& point) const
{
  double dx = x - point.x;
  double dy = y - point.y;
  return(sqrt((dx * dx) + (dy * dy)));
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenPath::buildReport() 
{
  m_msgs << "Visit Radius:        " << m_visit_radius << endl;
  m_msgs << "Updates Var:         " << m_updates_var << endl;
  m_msgs << "Activate Var:        " << m_activate_var << endl;
  m_msgs << "Total Points:        " << m_points.size() << endl;
  m_msgs << "Invalid Points:      " << m_invalid_points << endl;
  m_msgs << "First Point:         " << boolToString(m_first_received) << endl;
  m_msgs << "Last Point:          " << boolToString(m_last_received) << endl;
  m_msgs << "NAV_X/Y Received:    " << boolToString(m_nav_received) << endl;
  m_msgs << "Tour Generated:      " << boolToString(m_tour_generated) << endl;
  m_msgs << "Tour Started:        " << boolToString(m_tour_started) << endl;
  m_msgs << "Tours Generated:     " << m_tours_generated << endl;
  m_msgs << "Current NAV:         " << m_nav_x << "," << m_nav_y << endl;
  m_msgs << "Adaptive Speed:      " << boolToString(m_adaptive_speed) << endl;
  m_msgs << "Cruise/Slow/Pivot:   " << m_cruise_speed << " / "
         << m_slow_speed << " / " << m_pivot_speed << endl;
  m_msgs << "Sharp/Pivot Angle:   " << m_sharp_turn_angle << " / "
         << m_pivot_turn_angle << endl;
  m_msgs << "Slow Down Range:     " << m_slow_down_range << endl;
  m_msgs << "Pivot Turn:          " << boolToString(m_pivot_turn) << endl;
  m_msgs << "Pivot Start/Done:    " << m_pivot_heading_error << " / "
         << m_pivot_release_error << endl;
  m_msgs << "Pivot Active:        " << boolToString(m_pivot_active) << endl;
  m_msgs << "Pivot Direction:     " << m_pivot_dir << endl;
  m_msgs << "Heading/Desired/Err: " << m_nav_heading << " / "
         << m_desired_heading << " / " << m_heading_error << endl;
  m_msgs << "WPT Index/Dist:      " << m_wpt_index << " / " << m_wpt_dist << endl;
  m_msgs << "Current Speed Cmd:   " << m_current_speed << endl;

  ACTable actab(3);
  actab << "ID | X | Y";
  actab.addHeaderLines();
  unsigned int max_rows = (m_points.size() < 8) ? m_points.size() : 8;
  for(unsigned int i=0; i<max_rows; i++)
    actab << m_points[i].id << doubleToStringX(m_points[i].x, 1)
          << doubleToStringX(m_points[i].y, 1);
  m_msgs << actab.getFormattedString();

  return(true);
}
