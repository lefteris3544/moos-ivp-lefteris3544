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
  void         updateRescueTrail();
  bool         addRescueTrailPoint(double, double);
  bool         pointNearRescueTrail(double, double) const;
  bool         segmentNearRescueTrail(double, double, double, double) const;
  bool         scoutPointConflictsWithRescueTrail(const XYPoint&) const;
  void         updateZigLeg();
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

  XYPolygon m_rescue_region;

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

  std::vector<XYPoint> m_scout_points;

  std::string m_tmate;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
