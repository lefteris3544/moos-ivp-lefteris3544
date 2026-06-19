/************************************************************/
/*    NAME: Lefteris                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenPath_HEADER
#define GenPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <vector>

struct VisitPoint {
  double x;
  double y;
  std::string id;
  bool visited;
};

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void handleVisitPoint(std::string point);
   void handleWptStat(std::string stat);
   bool parseVisitPoint(std::string point, VisitPoint& visit_point) const;
   void generateTour();
   void postTourUpdate();
   bool updatePivotControl();
   void endPivotControl();
   void updateAdaptiveSpeed();
   void postSpeedUpdate(double speed);
   std::vector<VisitPoint> buildGreedyTour() const;
   double calcHeadingToPoint(const VisitPoint& point) const;
   double calcTurnAngle(unsigned int ix) const;
   double angle180(double angle) const;
   double angle360(double angle) const;
   double distToPoint(double x, double y, const VisitPoint& point) const;

 private: // Configuration variables
   std::string m_updates_var;
   std::string m_activate_var;
   double m_visit_radius;
   double m_default_nav_x;
   double m_default_nav_y;
   bool   m_adaptive_speed;
   double m_cruise_speed;
   double m_slow_speed;
   double m_pivot_speed;
   double m_sharp_turn_angle;
   double m_pivot_turn_angle;
   double m_slow_down_range;
   double m_speed_post_interval;
   bool   m_pivot_turn;
   double m_pivot_heading_error;
   double m_pivot_release_error;
   double m_pivot_min_dist;
   double m_pivot_thrust;

 private: // State variables
   std::vector<VisitPoint> m_points;
   std::vector<VisitPoint> m_tour;
   unsigned int m_invalid_points;
   unsigned int m_tours_generated;
   bool m_first_received;
   bool m_last_received;
   bool m_nav_received;
   bool m_heading_received;
   bool m_desired_heading_received;
   bool m_wpt_stat_received;
   bool m_tour_generated;
   bool m_tour_started;
   bool m_pivot_active;
   double m_nav_x;
   double m_nav_y;
   double m_nav_heading;
   double m_desired_heading;
   double m_heading_error;
   double m_pivot_dir;
   int    m_wpt_index;
   double m_wpt_dist;
   double m_current_speed;
   double m_last_speed_post_time;
   double m_last_update_post_time;
   std::string m_latest_update;
};

#endif 
