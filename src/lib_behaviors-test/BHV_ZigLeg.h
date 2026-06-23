/************************************************************/
/*    NAME: Lefteris                                       */
/*    ORGN: MIT                                            */
/*    FILE: BHV_ZigLeg.h                                   */
/*    DATE: June 2026                                      */
/************************************************************/

#ifndef BHV_ZIGLEG_HEADER
#define BHV_ZIGLEG_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};

  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();

protected:
  bool         updateInfoIn();
  void         postRangePulse();
  IvPFunction* buildCourseZAIC();
  double       angle360(double) const;

protected: // Configuration parameters
  double       m_zig_duration;
  double       m_zig_angle;
  double       m_zig_delay;
  double       m_pulse_range;
  double       m_pulse_duration;

protected: // State variables
  double       m_osx;
  double       m_osy;
  double       m_osh;
  double       m_curr_time;
  double       m_prev_wpt_index;
  double       m_wpt_change_time;
  double       m_zig_start_time;
  double       m_start_time;
  double       m_zig_heading;
  bool         m_have_wpt_index;
  bool         m_zig_pending;
  bool         m_zig_active;
  bool         m_initial_zig_done;
};

#ifdef WIN32
  #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
  #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_ZigLeg(domain);}
}

#endif
