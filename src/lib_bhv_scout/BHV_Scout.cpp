/*****************************************************************/
/*    NAME: M.Benjamin                                           */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <cctype>
#include <ctime>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: stringToDouble()

static bool stringToDouble(string str, double& result)
{
  str = stripBlankEnds(str);
  if(str == "")
    return(false);

  char *end = 0;
  result = strtod(str.c_str(), &end);

  while((end != 0) && (*end != '\0')) {
    if(!isspace(*end))
      return(false);
    end++;
  }

  return(true);
}

//-----------------------------------------------------------
// Procedure: distPointToSegment()

static double distPointToSegment(double px, double py,
				 double ax, double ay,
				 double bx, double by)
{
  double dx = bx - ax;
  double dy = by - ay;
  double len_sq = (dx * dx) + (dy * dy);

  if(len_sq <= 0)
    return(hypot(px - ax, py - ay));

  double pct = (((px - ax) * dx) + ((py - ay) * dy)) / len_sq;
  if(pct < 0)
    pct = 0;
  else if(pct > 1)
    pct = 1;

  double closest_x = ax + (pct * dx);
  double closest_y = ay + (pct * dy);
  return(hypot(px - closest_x, py - closest_y));
}

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "scout");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;
  m_ptx  = 0;
  m_pty  = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 
  m_desired_speed  = 1; 
  m_capture_radius = 10;

  m_pt_set           = false;
  m_point_ix         = 0;
  m_use_config_points = false;
  m_repeat_points    = true;
  m_zig_active       = false;
  m_zig_start_time   = 0;
  m_zig_last_switch  = 0;
  m_zig_heading      = 0;
  m_zig_side         = 1;

  m_zig_enabled        = true;
  m_zig_duration       = 12;
  m_zig_angle          = 45;
  m_avoid_rescue_trail = true;
  m_rescue_trail_gap   = 10;
  m_rescue_avoid_radius = 18;
  m_rescue_trail_max_pts = 80;
  m_random_search_after_points = true;
  m_visited_avoid_radius = 22;
  m_random_search_tries = 120;

  m_scout_points.push_back(XYPoint(-76, -70));
  m_scout_points.push_back(XYPoint(-185, -10));
  m_scout_points.push_back(XYPoint(-45, -5));
  m_scout_points.push_back(XYPoint(-39, -17));
  m_scout_points.push_back(XYPoint(-182, -17));
  m_scout_points.push_back(XYPoint(-162, -29));
  m_scout_points.push_back(XYPoint(-47, -29));
  m_scout_points.push_back(XYPoint(-55, -41));
  m_scout_points.push_back(XYPoint(-142, -41));
  m_scout_points.push_back(XYPoint(-123, -53));
  m_scout_points.push_back(XYPoint(-62, -53));
  m_scout_points.push_back(XYPoint(-69, -65));
  m_scout_points.push_back(XYPoint(-104, -65));
  m_scout_points.push_back(XYPoint(-85, -77));
  m_scout_points.push_back(XYPoint(-75, -77));

  m_use_config_points = true;

  srand(time(NULL));
  
  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("RESCUE_REGION");
  addInfoVars("SCOUTED_SWIMMER");
  addInfoVars("NODE_REPORT", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam() - handle behavior configuration parameters

bool BHV_Scout::setParam(string param, string val) 
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true;
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else if(param == "points")
    handled = setScoutPoints(val);
  else if(param == "repeat")
    handled = setRepeat(val);
  else if(param == "zig_enabled")
    handled = setBooleanOnString(m_zig_enabled, val);
  else if(param == "zig_duration")
    handled = setPosDoubleOnString(m_zig_duration, val);
  else if(param == "zig_angle")
    handled = setDoubleOnString(m_zig_angle, val);
  else if(param == "avoid_rescue_trail")
    handled = setBooleanOnString(m_avoid_rescue_trail, val);
  else if(param == "rescue_trail_gap")
    handled = setPosDoubleOnString(m_rescue_trail_gap, val);
  else if(param == "rescue_avoid_radius")
    handled = setPosDoubleOnString(m_rescue_avoid_radius, val);
  else if(param == "random_search_after_points")
    handled = setBooleanOnString(m_random_search_after_points, val);
  else if(param == "visited_avoid_radius")
    handled = setPosDoubleOnString(m_visited_avoid_radius, val);
  else if(param == "random_search_tries")
    handled = setUIntOnString(m_random_search_tries, val);
  else
    handled = false;
  
  return(handled);
}

//---------------------------------------------------------------
// Procedure: setScoutPoints()

bool BHV_Scout::setScoutPoints(string val)
{
  vector<string> specs = parseString(val, ':');
  vector<XYPoint> new_points;

  for(unsigned int i=0; i<specs.size(); i++) {
    string spec = stripBlankEnds(specs[i]);
    vector<string> xy = parseString(spec, ',');

    if(xy.size() != 2)
      return(false);

    double x, y;
    if(!stringToDouble(xy[0], x) || !stringToDouble(xy[1], y))
      return(false);

    new_points.push_back(XYPoint(x, y));
  }

  if(new_points.empty())
    return(false);

  m_scout_points = new_points;
  m_use_config_points = true;
  m_point_ix = 0;
  m_pt_set = false;

  string msg = "Loaded " + uintToString(m_scout_points.size()) + " scout points";
  postEventMessage(msg);
  return(true);
}

//---------------------------------------------------------------
// Procedure: setRepeat()

bool BHV_Scout::setRepeat(string val)
{
  val = tolower(stripBlankEnds(val));

  if((val == "forever") || (val == "loop")) {
    m_repeat_points = true;
    return(true);
  }

  return(setBooleanOnString(m_repeat_points, val));
}

//---------------------------------------------------------------
// Procedure: angle360()

double BHV_Scout::angle360(double angle) const
{
  while(angle < 0)
    angle += 360;
  while(angle >= 360)
    angle -= 360;
  return(angle);
}

//---------------------------------------------------------------
// Procedure: updateRescueTrail()

void BHV_Scout::updateRescueTrail()
{
  if(!m_avoid_rescue_trail || (m_tmate == ""))
    return;

  if(!getBufferVarUpdated("NODE_REPORT"))
    return;

  string report = getBufferStringVal("NODE_REPORT");
  if(report == "")
    return;

  string vname = tokStringParse(report, "NAME", ',', '=');
  if(vname != m_tmate)
    return;

  double x, y;
  string xstr = tokStringParse(report, "X", ',', '=');
  string ystr = tokStringParse(report, "Y", ',', '=');
  if(!stringToDouble(xstr, x) || !stringToDouble(ystr, y))
    return;

  addRescueTrailPoint(x, y);
}

//---------------------------------------------------------------
// Procedure: addRescueTrailPoint()

bool BHV_Scout::addRescueTrailPoint(double x, double y)
{
  if(!m_rescue_trail.empty()) {
    XYPoint last_pt = m_rescue_trail.back();
    double dist = hypot(x - last_pt.x(), y - last_pt.y());
    if(dist < m_rescue_trail_gap)
      return(false);
  }

  m_rescue_trail.push_back(XYPoint(x, y));
  if(m_rescue_trail.size() > m_rescue_trail_max_pts)
    m_rescue_trail.erase(m_rescue_trail.begin());

  return(true);
}

//---------------------------------------------------------------
// Procedure: pointNearRescueTrail()

bool BHV_Scout::pointNearRescueTrail(double x, double y) const
{
  for(unsigned int i=0; i<m_rescue_trail.size(); i++) {
    double dist = hypot(x - m_rescue_trail[i].x(), y - m_rescue_trail[i].y());
    if(dist <= m_rescue_avoid_radius)
      return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: segmentNearRescueTrail()

bool BHV_Scout::segmentNearRescueTrail(double ax, double ay,
				       double bx, double by) const
{
  for(unsigned int i=0; i<m_rescue_trail.size(); i++) {
    double dist = distPointToSegment(m_rescue_trail[i].x(),
				     m_rescue_trail[i].y(),
				     ax, ay, bx, by);
    if(dist <= m_rescue_avoid_radius)
      return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: scoutPointConflictsWithRescueTrail()

bool BHV_Scout::scoutPointConflictsWithRescueTrail(const XYPoint& pt) const
{
  if(!m_avoid_rescue_trail || m_rescue_trail.empty())
    return(false);

  if(pointNearRescueTrail(pt.x(), pt.y()))
    return(true);

  return(segmentNearRescueTrail(m_osx, m_osy, pt.x(), pt.y()));
}

//---------------------------------------------------------------
// Procedure: addVisitedPoint()

void BHV_Scout::addVisitedPoint(double x, double y)
{
  for(unsigned int i=0; i<m_visited_points.size(); i++) {
    double dist = hypot(x - m_visited_points[i].x(), y - m_visited_points[i].y());
    if(dist <= m_capture_radius)
      return;
  }

  m_visited_points.push_back(XYPoint(x, y));
}

//---------------------------------------------------------------
// Procedure: pointNearVisited()

bool BHV_Scout::pointNearVisited(double x, double y) const
{
  for(unsigned int i=0; i<m_visited_points.size(); i++) {
    double dist = hypot(x - m_visited_points[i].x(), y - m_visited_points[i].y());
    if(dist <= m_visited_avoid_radius)
      return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: updateZigLeg()

void BHV_Scout::updateZigLeg()
{
  if(!m_zig_enabled)
    return;

  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);

  if(!m_zig_active) {
    m_zig_active = true;
    m_zig_start_time = m_curr_time;
    m_zig_last_switch = m_curr_time;
    m_zig_side = 1;
    postZigPulse();
  }
  else if((m_curr_time - m_zig_last_switch) >= m_zig_duration) {
    m_zig_last_switch = m_curr_time;
    m_zig_side *= -1;
    postZigPulse();
  }

  m_zig_heading = angle360(rel_ang_to_wpt + (m_zig_side * m_zig_angle));
  m_zig_start_time = m_curr_time;

  string msg = "ZigLeg heading: " + doubleToStringX(m_zig_heading, 1);
  postEventMessage(msg);
}

//---------------------------------------------------------------
// Procedure: postZigPulse()

void BHV_Scout::postZigPulse()
{
  string pulse = "x=" + doubleToString(m_osx, 2);
  pulse += ",y=" + doubleToString(m_osy, 2);
  pulse += ",radius=" + doubleToString(m_zig_pulse_range, 2);
  pulse += ",duration=" + doubleToString(m_zig_pulse_duration, 2);
  pulse += ",label=scout_zigleg";
  pulse += ",edge_color=orange,fill_color=orange";

  postMessage("VIEW_RANGE_PULSE", pulse);
}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  updateRescueTrail();

  if(!getBufferVarUpdated("SCOUTED_SWIMMER"))
    return;

  string report = getBufferStringVal("SCOUTED_SWIMMER");
  if(report == "")
    return;

  if(m_tmate == "") {
    postWMessage("Mandatory Teammate name is null");
    return;
  }
  postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
  // Part 1: Get vehicle position from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_curr_time = getBufferCurrTime();
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }
  
  // Part 2: Determine if the vehicle has reached the destination 
  // point and if so, declare completion.
  if(!updateScoutPoint())
    return(0);

  double dist = hypot((m_ptx-m_osx), (m_pty-m_osy));
  //postEventMessage("Dist=" + doubleToStringX(dist,1));
  if(dist <= m_capture_radius) {
    addVisitedPoint(m_ptx, m_pty);
    m_pt_set = false;
    postViewPoint(false);

    if(m_use_config_points) {
      if((m_point_ix + 1) < m_scout_points.size()) {
	m_point_ix++;
	updateScoutPoint();
	m_zig_active = false;
	return(0);
      }

      if(m_random_search_after_points) {
	m_use_config_points = false;
	updateScoutPoint();
	m_zig_active = false;
	return(0);
      }

      if(m_repeat_points) {
	m_point_ix = 0;
	updateScoutPoint();
	m_zig_active = false;
	return(0);
      }

      setComplete();
    }
    else {
      updateScoutPoint();
      m_zig_active = false;
    }

    return(0);
  }

  // Part 3: Post the waypoint as a string for consumption by 
  // a viewer application.
  postViewPoint(true);

  // Part 4: Build the IvP function 
  IvPFunction *ipf = buildFunction();
  if(ipf == 0) 
    postWMessage("Problem Creating the IvP Function");
  
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

bool BHV_Scout::updateScoutPoint()
{
  if(m_pt_set)
    return(true);

  if(m_use_config_points) {
    if(m_scout_points.empty())
      return(false);

    if(m_point_ix >= m_scout_points.size())
      m_point_ix = 0;

    unsigned int start_ix = m_point_ix;
    bool found_point = false;
    for(unsigned int i=0; i<m_scout_points.size(); i++) {
      unsigned int ix = start_ix + i;
      if(ix >= m_scout_points.size()) {
	if(!m_repeat_points)
	  break;
	ix = ix % m_scout_points.size();
      }

      if(!scoutPointConflictsWithRescueTrail(m_scout_points[ix])) {
	m_point_ix = ix;
	found_point = true;
	break;
      }

      string skip_msg = "Skipping scout pt " + uintToString(ix+1) +
	" near rescue trail";
      postEventMessage(skip_msg);
    }

    if(!found_point)
      postWMessage("All scout points near rescue trail; using scheduled point");

    m_ptx = m_scout_points[m_point_ix].x();
    m_pty = m_scout_points[m_point_ix].y();
    m_pt_set = true;

    string msg = "Scout pt " + uintToString(m_point_ix+1) + "/" +
      uintToString(m_scout_points.size()) + ": " +
      doubleToStringX(m_ptx) + "," + doubleToStringX(m_pty);
    postEventMessage(msg);
    return(true);
  }

  string region_str = getBufferStringVal("RESCUE_REGION");
  if(region_str == "")
    postWMessage("Unknown RESCUE_REGION");
  else
    postRetractWMessage("Unknown RESCUE_REGION");

  XYPolygon region = string2Poly(region_str);
  if(!region.is_convex()) {
    postWMessage("Badly formed RESCUE_REGION");
    return(false);
  }
  m_rescue_region = region;

  double ptx = 0;
  double pty = 0;
  bool ok = false;
  bool relaxed_visited_check = false;
  for(unsigned int i=0; i<m_random_search_tries; i++) {
    ok = randPointInPoly(m_rescue_region, ptx, pty);
    if(!ok)
      break;

    if(pointNearVisited(ptx, pty))
      continue;
    if(pointNearRescueTrail(ptx, pty))
      continue;

    break;
  }

  if(ok && (pointNearVisited(ptx, pty) || pointNearRescueTrail(ptx, pty))) {
    relaxed_visited_check = true;
    for(unsigned int i=0; i<m_random_search_tries; i++) {
      ok = randPointInPoly(m_rescue_region, ptx, pty);
      if(!ok)
	break;
      if(!pointNearRescueTrail(ptx, pty))
	break;
    }
  }

  if(!ok) {
    postWMessage("Unable to generate scout point");
    return(false);
  }
    
  m_ptx = ptx;
  m_pty = pty;
  m_pt_set = true;
  string msg = "Random scout pt: " + doubleToStringX(ptx) + "," +
    doubleToStringX(pty);
  if(relaxed_visited_check)
    msg += " (visited spacing relaxed)";
  postEventMessage(msg);
  return(true);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color("orange");
  pt.set_label(m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
  if(!m_pt_set)
    return(0);
  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);  
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);
  updateZigLeg();
  if(m_zig_enabled && m_zig_active)
    rel_ang_to_wpt = m_zig_heading;

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);  
  crs_zaic.setValueWrap(true);
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);
  ivp_function->setPWT(m_priority_wt);

  return(ivp_function);
}
