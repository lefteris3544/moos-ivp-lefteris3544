/*****************************************************************/
/*    NAME: M.Benjamin                                           */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <algorithm>
#include <cctype>
#include <ctime>
#include <limits>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"

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
  m_region_set       = false;
  m_candidates_built = false;
  m_rescue_pos_set   = false;
  m_rescue_x         = 0;
  m_rescue_y         = 0;
  m_mission_start_time = 0;
  m_last_discovery_time = 0;
  m_local_search_until = 0;
  m_selected_score   = 0;
  m_selected_high_value = false;
  m_discovered_unreg_count = 0;
  m_rescued_count = 0;

  m_zig_enabled        = true;
  m_zig_duration       = 12;
  m_zig_angle          = 45;
  m_zig_pulse_range    = 12;
  m_zig_pulse_duration = 4;
  m_avoid_rescue_trail = true;
  m_rescue_trail_gap   = 10;
  m_rescue_avoid_radius = 18;
  m_rescue_trail_max_pts = 80;
  m_random_search_after_points = true;
  m_visited_radius = 22;
  m_visited_avoid_radius = 22;
  m_random_search_tries = 120;
  m_grid_spacing = 18;
  m_lane_spacing = 28;
  m_registered_avoid_radius = 18;
  m_local_search_radius = 12;
  m_local_search_time = 10;
  m_early_phase_time = 40;
  m_urgency_queue_threshold = 1;
  m_use_rescue_path = true;
  m_debug = false;
  m_distance_weight = 0.25;
  m_age_weight = 0.08;
  m_high_value_score = 95;

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
  addInfoVars("FOUND_SWIMMER, RESCUED_SWIMMER", "no_warning");
  addInfoVars("UFRM_GAME_STATUS, RESCUE_PATH, VIEW_SEGLIST", "no_warning");
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
  else if((param == "visited_radius") || (param == "visited_avoid_radius")) {
    handled = setPosDoubleOnString(m_visited_radius, val);
    if(handled)
      m_visited_avoid_radius = m_visited_radius;
  }
  else if(param == "random_search_tries")
    handled = setUIntOnString(m_random_search_tries, val);
  else if(param == "grid_spacing") {
    handled = setPosDoubleOnString(m_grid_spacing, val);
    if(handled)
      m_candidates_built = false;
  }
  else if(param == "lane_spacing") {
    handled = setPosDoubleOnString(m_lane_spacing, val);
    if(handled)
      m_candidates_built = false;
  }
  else if(param == "registered_avoid_radius")
    handled = setPosDoubleOnString(m_registered_avoid_radius, val);
  else if(param == "local_search_radius") {
    handled = setPosDoubleOnString(m_local_search_radius, val);
    if(handled)
      m_zig_pulse_range = m_local_search_radius;
  }
  else if(param == "local_search_time") {
    handled = setPosDoubleOnString(m_local_search_time, val);
    if(handled)
      m_zig_pulse_duration = m_local_search_time;
  }
  else if(param == "early_phase_time")
    handled = setPosDoubleOnString(m_early_phase_time, val);
  else if(param == "urgency_queue_threshold")
    handled = setUIntOnString(m_urgency_queue_threshold, val);
  else if(param == "use_rescue_path")
    handled = setBooleanOnString(m_use_rescue_path, val);
  else if(param == "debug")
    handled = setBooleanOnString(m_debug, val);
  else if(param == "distance_weight")
    handled = setPosDoubleOnString(m_distance_weight, val);
  else if(param == "age_weight")
    handled = setPosDoubleOnString(m_age_weight, val);
  else if(param == "high_value_score")
    handled = setDoubleOnString(m_high_value_score, val);
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
// Procedure: updateRescueRegion()

bool BHV_Scout::updateRescueRegion()
{
  string region_str = getBufferStringVal("RESCUE_REGION");
  if(region_str == "") {
    if(!m_region_set)
      postWMessage("Unknown RESCUE_REGION");
    return(false);
  }

  postRetractWMessage("Unknown RESCUE_REGION");

  if(m_region_set && (region_str == m_region_spec))
    return(true);

  XYPolygon region = string2Poly(region_str);
  if(!region.is_convex()) {
    postWMessage("Badly formed RESCUE_REGION");
    return(false);
  }

  m_rescue_region = region;
  m_region_spec = region_str;
  m_region_set = true;
  m_candidates_built = false;
  buildSearchCandidates();
  return(true);
}

//---------------------------------------------------------------
// Procedure: buildSearchCandidates()

void BHV_Scout::buildSearchCandidates()
{
  m_candidates.clear();

  if(!m_region_set)
    return;

  double min_x = m_rescue_region.get_min_x();
  double max_x = m_rescue_region.get_max_x();
  double min_y = m_rescue_region.get_min_y();
  double max_y = m_rescue_region.get_max_y();

  double row_gap = m_grid_spacing;
  if(m_lane_spacing > 0)
    row_gap = m_lane_spacing;

  double start_x = min_x + (m_grid_spacing * 0.5);
  double start_y = min_y + (row_gap * 0.5);

  bool left_to_right = true;
  for(double y=start_y; y<=max_y; y+=row_gap) {
    vector<ScoutCandidate> row;
    for(double x=start_x; x<=max_x; x+=m_grid_spacing) {
      if(!m_rescue_region.contains(x, y))
	continue;

      ScoutCandidate cand;
      cand.x = x;
      cand.y = y;
      cand.score = 0;
      cand.last_visit = 0;
      cand.visited = false;
      row.push_back(cand);
    }

    if(!left_to_right)
      reverse(row.begin(), row.end());

    for(unsigned int i=0; i<row.size(); i++)
      m_candidates.push_back(row[i]);

    left_to_right = !left_to_right;
  }

  for(unsigned int i=0; i<m_scout_points.size(); i++) {
    if(!m_rescue_region.contains(m_scout_points[i].x(), m_scout_points[i].y()))
      continue;

    ScoutCandidate cand;
    cand.x = m_scout_points[i].x();
    cand.y = m_scout_points[i].y();
    cand.score = 0;
    cand.last_visit = 0;
    cand.visited = false;
    m_candidates.push_back(cand);
  }

  m_candidates_built = true;
  if(m_debug) {
    string msg = "candidates=" + uintToString((unsigned int)m_candidates.size());
    postMessage("SCOUT_DEBUG", msg);
  }
}

//---------------------------------------------------------------
// Procedure: getScoutMode()

string BHV_Scout::getScoutMode() const
{
  if(m_curr_time < m_local_search_until)
    return("LOCAL_SEARCH");

  double mission_elapsed = m_curr_time - m_mission_start_time;
  unsigned int pending = 0;
  if(m_discovered_unreg_count > m_rescued_count)
    pending = m_discovered_unreg_count - m_rescued_count;

  if((m_discovered_unreg_count > 0) &&
     (pending <= m_urgency_queue_threshold) &&
     (mission_elapsed > m_early_phase_time))
    return("URGENCY");

  if(mission_elapsed <= m_early_phase_time)
    return("EARLY_SWEEP");

  return("ADAPTIVE_GRID");
}

//---------------------------------------------------------------
// Procedure: candidateScore()

double BHV_Scout::candidateScore(const ScoutCandidate& cand,
				 string& reason) const
{
  string mode = getScoutMode();
  double dist = hypot(cand.x - m_osx, cand.y - m_osy);
  double visited_dist = nearestVisitedDist(cand.x, cand.y);
  double rescue_dist = nearestRescueDist(cand.x, cand.y);
  double reg_dist = nearestRegisteredDist(cand.x, cand.y);
  double known_dist = nearestKnownSwimmerDist(cand.x, cand.y);

  double age = 0;
  if(cand.visited && (cand.last_visit > 0))
    age = m_curr_time - cand.last_visit;
  else
    age = m_curr_time - m_mission_start_time + 60;

  double score = 100;
  reason = "unexplored";

  if(cand.visited)
    score -= 35;
  score += m_age_weight * age;

  if(mode == "EARLY_SWEEP") {
    score += 0.18 * dist;
    reason += ",broad";
  }
  else if(mode == "URGENCY") {
    score += 0.28 * dist;
    score += 25;
    reason += ",urgent_far";
  }
  else {
    score -= m_distance_weight * dist;
    reason += ",near_enough";
  }

  if(visited_dist < m_visited_radius) {
    score -= (m_visited_radius - visited_dist) * 2.2;
    reason += ",visited_penalty";
  }

  if(rescue_dist < m_rescue_avoid_radius) {
    score -= (m_rescue_avoid_radius - rescue_dist) * 3.0;
    reason += ",rescue_overlap";
  }

  if(reg_dist < m_registered_avoid_radius) {
    score -= (m_registered_avoid_radius - reg_dist) * 2.5;
    reason += ",registered_penalty";
  }

  if(known_dist < m_registered_avoid_radius) {
    score -= (m_registered_avoid_radius - known_dist) * 1.3;
    reason += ",known_swimmer_penalty";
  }

  return(score);
}

//---------------------------------------------------------------
// Procedure: selectBestCandidate()

bool BHV_Scout::selectBestCandidate()
{
  if(!updateRescueRegion())
    return(false);

  if(!m_candidates_built)
    buildSearchCandidates();

  if(m_candidates.empty())
    return(false);

  double best_score = -numeric_limits<double>::max();
  unsigned int best_ix = 0;
  string best_reason;

  for(unsigned int i=0; i<m_candidates.size(); i++) {
    string reason;
    double score = candidateScore(m_candidates[i], reason);
    m_candidates[i].score = score;

    if(score > best_score) {
      best_score = score;
      best_ix = i;
      best_reason = reason;
    }
  }

  m_ptx = m_candidates[best_ix].x;
  m_pty = m_candidates[best_ix].y;
  m_pt_set = true;
  m_selected_score = best_score;
  m_selected_reason = best_reason;
  m_selected_high_value = (best_score >= m_high_value_score);

  postScoutDebug(getScoutMode(), best_score, best_reason);
  return(true);
}

//---------------------------------------------------------------
// Procedure: handleKnownReports()

void BHV_Scout::handleKnownReports()
{
  if(getBufferVarUpdated("SCOUTED_SWIMMER")) {
    string report = getBufferStringVal("SCOUTED_SWIMMER");
    addSwimmerReport(report, "SCOUTED_SWIMMER");
  }

  if(getBufferVarUpdated("FOUND_SWIMMER")) {
    string report = getBufferStringVal("FOUND_SWIMMER");
    addSwimmerReport(report, "FOUND_SWIMMER");
  }

  if(getBufferVarUpdated("RESCUED_SWIMMER")) {
    string report = getBufferStringVal("RESCUED_SWIMMER");
    addSwimmerReport(report, "RESCUED_SWIMMER");
  }
}

//---------------------------------------------------------------
// Procedure: addSwimmerReport()

void BHV_Scout::addSwimmerReport(string report, string source)
{
  if(report == "")
    return;

  string id = tokStringParse(report, "id", ',', '=');
  if(id == "")
    id = tokStringParse(report, "ID", ',', '=');

  string type = tolower(tokStringParse(report, "type", ',', '='));
  if(type == "")
    type = tolower(tokStringParse(report, "TYPE", ',', '='));

  string xstr = tokStringParse(report, "x", ',', '=');
  string ystr = tokStringParse(report, "y", ',', '=');
  if(xstr == "")
    xstr = tokStringParse(report, "X", ',', '=');
  if(ystr == "")
    ystr = tokStringParse(report, "Y", ',', '=');

  double x, y;
  bool have_xy = (stringToDouble(xstr, x) && stringToDouble(ystr, y));

  if((source == "FOUND_SWIMMER") || (source == "RESCUED_SWIMMER")) {
    if(id != "") {
      for(unsigned int i=0; i<m_found_swimmer_ids.size(); i++) {
	if(m_found_swimmer_ids[i] == id)
	  return;
      }
      m_found_swimmer_ids.push_back(id);
      m_rescued_count++;
    }
    if(have_xy)
      addKnownSwimmer(id, "found", x, y);
    return;
  }

  if(!have_xy)
    return;

  if(id == "")
    id = doubleToStringX(x, 1) + ":" + doubleToStringX(y, 1);

  bool added = addKnownSwimmer(id, type, x, y);

  if(type != "reg") {
    if(added)
      m_discovered_unreg_count++;
    m_last_discovery_time = m_curr_time;
    m_local_search_until = m_curr_time + m_local_search_time;
  }
}

//---------------------------------------------------------------
// Procedure: addKnownSwimmer()

bool BHV_Scout::addKnownSwimmer(string id, string type, double x, double y)
{
  vector<ScoutSwimmer>& swimmers =
    (type == "reg") ? m_registered_swimmers : m_known_swimmers;

  for(unsigned int i=0; i<swimmers.size(); i++) {
    bool same_id = ((id != "") && (swimmers[i].id == id));
    bool same_xy = (hypot(x - swimmers[i].x, y - swimmers[i].y) <= 2.0);
    if(same_id || same_xy) {
      swimmers[i].x = x;
      swimmers[i].y = y;
      swimmers[i].type = type;
      return(false);
    }
  }

  ScoutSwimmer swimmer;
  swimmer.id = id;
  swimmer.type = type;
  swimmer.x = x;
  swimmer.y = y;
  swimmers.push_back(swimmer);

  return(true);
}

//---------------------------------------------------------------
// Procedure: updateRescuePath()

void BHV_Scout::updateRescuePath()
{
  if(!m_use_rescue_path)
    return;

  string report;
  if(getBufferVarUpdated("RESCUE_PATH"))
    report = getBufferStringVal("RESCUE_PATH");
  else if(getBufferVarUpdated("VIEW_SEGLIST"))
    report = getBufferStringVal("VIEW_SEGLIST");
  else
    return;

  if((m_tmate != "") && !strContains(report, "rescue_path_" + m_tmate))
    return;

  XYSegList segl = string2SegList(report);
  if(segl.size() == 0)
    return;

  m_rescue_path.clear();
  for(unsigned int i=0; i<segl.size(); i++)
    m_rescue_path.push_back(XYPoint(segl.get_vx(i), segl.get_vy(i)));
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

  m_rescue_x = x;
  m_rescue_y = y;
  m_rescue_pos_set = true;
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

  for(unsigned int i=0; i<m_candidates.size(); i++) {
    double dist = hypot(x - m_candidates[i].x, y - m_candidates[i].y);
    if(dist <= m_visited_radius) {
      m_candidates[i].visited = true;
      m_candidates[i].last_visit = m_curr_time;
    }
  }
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
// Procedure: nearestVisitedDist()

double BHV_Scout::nearestVisitedDist(double x, double y) const
{
  double nearest = numeric_limits<double>::max();
  for(unsigned int i=0; i<m_visited_points.size(); i++) {
    double dist = hypot(x - m_visited_points[i].x(), y - m_visited_points[i].y());
    if(dist < nearest)
      nearest = dist;
  }

  return(nearest);
}

//---------------------------------------------------------------
// Procedure: nearestRescueDist()

double BHV_Scout::nearestRescueDist(double x, double y) const
{
  double nearest = numeric_limits<double>::max();

  if(m_rescue_pos_set)
    nearest = hypot(x - m_rescue_x, y - m_rescue_y);

  for(unsigned int i=0; i<m_rescue_trail.size(); i++) {
    double dist = hypot(x - m_rescue_trail[i].x(), y - m_rescue_trail[i].y());
    if(dist < nearest)
      nearest = dist;
  }

  for(unsigned int i=0; i<m_rescue_path.size(); i++) {
    double dist = hypot(x - m_rescue_path[i].x(), y - m_rescue_path[i].y());
    if(dist < nearest)
      nearest = dist;
  }

  return(nearest);
}

//---------------------------------------------------------------
// Procedure: nearestRegisteredDist()

double BHV_Scout::nearestRegisteredDist(double x, double y) const
{
  double nearest = numeric_limits<double>::max();
  for(unsigned int i=0; i<m_registered_swimmers.size(); i++) {
    double dist = hypot(x - m_registered_swimmers[i].x,
			y - m_registered_swimmers[i].y);
    if(dist < nearest)
      nearest = dist;
  }

  return(nearest);
}

//---------------------------------------------------------------
// Procedure: nearestKnownSwimmerDist()

double BHV_Scout::nearestKnownSwimmerDist(double x, double y) const
{
  double nearest = numeric_limits<double>::max();
  for(unsigned int i=0; i<m_known_swimmers.size(); i++) {
    double dist = hypot(x - m_known_swimmers[i].x,
			y - m_known_swimmers[i].y);
    if(dist < nearest)
      nearest = dist;
  }

  return(nearest);
}

//---------------------------------------------------------------
// Procedure: postScoutDebug()

void BHV_Scout::postScoutDebug(string mode, double score, string reason)
{
  if(!m_debug)
    return;

  string pt = "x=" + doubleToStringX(m_ptx, 1) +
    ",y=" + doubleToStringX(m_pty, 1);

  string debug = "mode=" + mode;
  debug += ",pt=" + pt;
  debug += ",score=" + doubleToStringX(score, 2);
  debug += ",visited=" + uintToString((unsigned int)m_visited_points.size());
  debug += ",candidates=" + uintToString((unsigned int)m_candidates.size());
  debug += ",known=" + uintToString((unsigned int)m_known_swimmers.size());
  debug += ",registered=" + uintToString((unsigned int)m_registered_swimmers.size());
  debug += ",reason=" + reason;

  postMessage("SCOUT_MODE", mode);
  postMessage("SCOUT_NEXT_PT", pt);
  postMessage("SCOUT_SCORE", doubleToStringX(score, 2));
  postMessage("SCOUT_DEBUG", debug);
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
// Procedure: shouldUseZig()

bool BHV_Scout::shouldUseZig() const
{
  if(!m_zig_enabled)
    return(false);

  string mode = getScoutMode();
  if(mode == "LOCAL_SEARCH")
    return(true);

  if(mode == "URGENCY")
    return(false);

  double mission_elapsed = m_curr_time - m_mission_start_time;
  if((mission_elapsed > m_early_phase_time) && m_selected_high_value)
    return(true);

  return(false);
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
  handleKnownReports();
  updateRescueTrail();
  updateRescuePath();

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
  if(m_mission_start_time <= 0) {
    m_mission_start_time = m_curr_time;
    m_last_discovery_time = m_curr_time;
  }
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

    if(m_region_set || updateRescueRegion()) {
      updateScoutPoint();
      m_zig_active = false;
      return(0);
    }

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

  if(m_random_search_after_points && updateRescueRegion())
    return(selectBestCandidate());

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
  if(shouldUseZig())
    updateZigLeg();
  else
    m_zig_active = false;
  if(shouldUseZig() && m_zig_active)
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
