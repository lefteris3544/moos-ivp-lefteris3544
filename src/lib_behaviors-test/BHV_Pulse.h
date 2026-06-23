/************************************************************/
/*    NAME: Lefteris                                       */
/*    ORGN: MIT                                            */
/*    FILE: BHV_Pulse.h                                    */
/*    DATE: June 2026                                      */
/************************************************************/

#ifndef BHV_PULSE_HEADER
#define BHV_PULSE_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Pulse : public IvPBehavior {
public:
  BHV_Pulse(IvPDomain);
  ~BHV_Pulse() {};

  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();

protected:
  bool         updateInfoIn();
  void         postRangePulse();

protected: // Configuration parameters
  double       m_pulse_range;
  double       m_pulse_duration;
  double       m_pulse_delay;

protected: // State variables
  double       m_osx;
  double       m_osy;
  double       m_curr_time;
  double       m_prev_wpt_index;
  double       m_wpt_change_time;
  double       m_start_time;
  bool         m_have_wpt_index;
  bool         m_pulse_pending;
  bool         m_initial_pulse_posted;
};

#ifdef WIN32
  #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
  #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_Pulse(domain);}
}

#endif
