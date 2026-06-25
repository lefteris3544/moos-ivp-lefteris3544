/*****************************************************************/
/*    NAME: M.Benjamin,                                          */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: April 30th 2022                                      */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_SCOUT_HEADER
#define BHV_SCOUT_HEADER

#include <string>
#include <vector>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYPolygon.h"

struct ScoutCandidate {
  double x;
  double y;
  double score;
  double last_visit;
  bool   visited;
};

struct ScoutSwimmer {
  std::string id;
  std::string type;
  double x;
  double y;
};

class BHV_Scout : public IvPBehavior {
public:
  BHV_Scout(IvPDomain);
  ~BHV_Scout() {};
  
  bool         setParam(std::string, std::string);
  void         onIdleState();
  IvPFunction* onRunState();
  void         onEveryState(std::string);
  
protected:
  IvPFunction* buildFunction();
  bool         updateScoutPoint();
  bool         setScoutPoints(std::string);
  bool         setRepeat(std::string);
  double       angle360(double) const;
  bool         updateRescueRegion();
  void         buildSearchCandidates();
  bool         selectBestCandidate();
  double       candidateScore(const ScoutCandidate&, std::string&) const;
  std::string  getScoutMode() const;
  void         handleKnownReports();
  void         addSwimmerReport(std::string, std::string);
  bool         addKnownSwimmer(std::string, std::string, double, double);
  void         updateRescuePath();
  void         updateRescueTrail();
  bool         addRescueTrailPoint(double, double);
  bool         pointNearRescueTrail(double, double) const;
  bool         segmentNearRescueTrail(double, double, double, double) const;
  bool         scoutPointConflictsWithRescueTrail(const XYPoint&) const;
  void         addVisitedPoint(double, double);
  bool         pointNearVisited(double, double) const;
  double       nearestVisitedDist(double, double) const;
  double       nearestRescueDist(double, double) const;
  double       nearestRegisteredDist(double, double) const;
  double       nearestKnownSwimmerDist(double, double) const;
  void         postScoutDebug(std::string, double, std::string);
  void         updateZigLeg();
  bool         shouldUseZig() const;
  void         postZigPulse();
  void         postViewPoint(bool viewable=true);

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_curr_time;

  double   m_ptx;
  double   m_pty;
  bool     m_pt_set;
  unsigned int m_point_ix;
  bool     m_zig_active;
  double   m_zig_start_time;
  double   m_zig_last_switch;
  double   m_zig_heading;
  double   m_zig_side;
  std::vector<XYPoint> m_rescue_trail;
  std::vector<XYPoint> m_rescue_path;
  std::vector<XYPoint> m_visited_points;
  std::vector<ScoutCandidate> m_candidates;
  std::vector<ScoutSwimmer> m_known_swimmers;
  std::vector<ScoutSwimmer> m_registered_swimmers;
  std::vector<std::string> m_found_swimmer_ids;

  XYPolygon m_rescue_region;
  bool     m_region_set;
  bool     m_candidates_built;
  bool     m_rescue_pos_set;
  double   m_rescue_x;
  double   m_rescue_y;
  double   m_mission_start_time;
  double   m_last_discovery_time;
  double   m_local_search_until;
  double   m_selected_score;
  bool     m_selected_high_value;
  std::string m_selected_reason;
  std::string m_region_spec;
  unsigned int m_discovered_unreg_count;
  unsigned int m_rescued_count;

protected: // Config variables
  double m_capture_radius;
  double m_desired_speed;
  bool   m_use_config_points;
  bool   m_repeat_points;
  bool   m_zig_enabled;
  double m_zig_duration;
  double m_zig_angle;
  double m_zig_pulse_range;
  double m_zig_pulse_duration;
  bool   m_avoid_rescue_trail;
  double m_rescue_trail_gap;
  double m_rescue_avoid_radius;
  unsigned int m_rescue_trail_max_pts;
  bool   m_random_search_after_points;
  double m_visited_radius;
  double m_visited_avoid_radius;
  unsigned int m_random_search_tries;
  double m_grid_spacing;
  double m_lane_spacing;
  double m_registered_avoid_radius;
  double m_local_search_radius;
  double m_local_search_time;
  double m_early_phase_time;
  unsigned int m_urgency_queue_threshold;
  bool   m_use_rescue_path;
  bool   m_debug;
  double m_distance_weight;
  double m_age_weight;
  double m_high_value_score;

  std::vector<XYPoint> m_scout_points;

  std::string m_tmate;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
