/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.h                                     */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#ifndef P_GEN_RESCUE_HEADER
#define P_GEN_RESCUE_HEADER

#include <vector>
#include <string>
#include <map>
#include <set>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue() {};

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();
  void RegisterVariables();
  
 protected:
  bool handleMailNewSwimmer(std::string);
  bool handleMailFoundSwimmer(std::string);
  bool handleMailNodeReport(std::string);
  void handleMailWptStat(std::string);
  void postShortestPath();
  void postNullPath();
  void postPath();
  void updateAdaptiveSpeed();
  void postSpeedUpdate(double);
  double calcTurnAngle(unsigned int) const;
  XYSegList buildPendingPath() const;
  XYSegList buildGreedyTwoHopPath(XYSegList, double, double) const;
  XYSegList buildCompetitivePath(XYSegList, double, double) const;
  double getNearestContactDist(double, double) const;
  std::set<std::string> getConcededSwimmers() const;
  std::string stringFromSet(std::set<std::string>) const;
  unsigned int getPendingSwimmerCount() const;

 private: // Config variables
  std::string m_vname;
  std::string m_planner;
  bool        m_concede_adversary;
  unsigned int m_concede_count;
  unsigned int m_concede_path_count;
  double      m_concede_radius;
  double      m_concede_heading_window;
  double      m_competitive_weight;
  double      m_competitive_lost_margin;
  double      m_contact_replan_interval;
  bool        m_adaptive_speed;
  double      m_cruise_speed;
  double      m_slow_speed;
  double      m_pivot_speed;
  double      m_sharp_turn_angle;
  double      m_pivot_turn_angle;
  double      m_slow_down_range;
  double      m_speed_post_interval;
  
 private: // State variables
  XYSegList  m_path;
  std::map<std::string, XYPoint> m_swimmers;
  std::set<std::string> m_found_swimmers;
  std::set<std::string> m_conceded_swimmers;
  bool       m_plan_pending;
  double     m_nav_x;
  double     m_nav_y;
  bool       m_nav_x_set;
  bool       m_nav_y_set;
  std::string m_contact_name;
  double     m_contact_x;
  double     m_contact_y;
  double     m_contact_hdg;
  bool       m_contact_xy_set;
  bool       m_contact_hdg_set;
  double     m_last_contact_replan_time;
  std::map<std::string, XYPoint> m_contacts;
  int        m_wpt_index;
  double     m_wpt_dist;
  bool       m_wpt_stat_received;
  double     m_current_speed;
  double     m_last_speed_post_time;
};

#endif 
