/************************************************************/
/*    NAME: Lefteris                                       */
/*    ORGN: MIT                                            */
/*    FILE: BHV_Pulse.cpp                                  */
/*    DATE: June 2026                                      */
/************************************************************/

#include <cstdlib>
#include "BHV_Pulse.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include "XYRangePulse.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Pulse::BHV_Pulse(IvPDomain domain) :
  IvPBehavior(domain)
{
  IvPBehavior::setParam("name", "pulse");
  m_domain = subDomain(m_domain, "course,speed");

  m_pulse_range    = 20;
  m_pulse_duration = 4;
  m_pulse_delay    = 5;

  m_osx              = 0;
  m_osy              = 0;
  m_curr_time        = 0;
  m_prev_wpt_index   = 0;
  m_wpt_change_time  = 0;
  m_start_time       = -1;
  m_have_wpt_index   = false;
  m_pulse_pending    = false;
  m_initial_pulse_posted = false;

  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("WPT_INDEX", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Pulse::setParam(string param, string val)
{
  param = tolower(param);
  double dval = atof(val.c_str());

  if((param == "pulse_range") && isNumber(val) && (dval > 0)) {
    m_pulse_range = dval;
    return(true);
  }
  else if((param == "pulse_duration") && isNumber(val) && (dval > 0)) {
    m_pulse_duration = dval;
    return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: updateInfoIn()

bool BHV_Pulse::updateInfoIn()
{
  bool ok_osx, ok_osy;
  m_osx = getBufferDoubleVal("NAV_X", ok_osx);
  m_osy = getBufferDoubleVal("NAV_Y", ok_osy);
  m_curr_time = getBufferCurrTime();

  if(!ok_osx || !ok_osy) {
    postWMessage("No ownship NAV_X/NAV_Y info in info_buffer.");
    return(false);
  }

  bool ok_wpt_index;
  double wpt_index = getBufferDoubleVal("WPT_INDEX", ok_wpt_index);
  if(!ok_wpt_index) {
    if(!m_have_wpt_index) {
      m_wpt_change_time = m_curr_time;
      m_pulse_pending = true;
      m_have_wpt_index = true;
    }
    return(true);
  }

  if(!m_have_wpt_index) {
    m_prev_wpt_index = wpt_index;
    m_wpt_change_time = m_curr_time;
    m_pulse_pending = true;
    m_have_wpt_index = true;
    return(true);
  }

  if(wpt_index != m_prev_wpt_index) {
    m_prev_wpt_index = wpt_index;
    m_wpt_change_time = m_curr_time;
    m_pulse_pending = true;
  }

  return(true);
}

//---------------------------------------------------------------
// Procedure: postRangePulse()

void BHV_Pulse::postRangePulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_pulse");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(m_curr_time);
  pulse.set_duration(m_pulse_duration);
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_edge_size(1);

  postMessage("VIEW_RANGE_PULSE", pulse.get_spec());
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_Pulse::onRunState()
{
  if(!updateInfoIn())
    return(0);

  if(m_start_time < 0)
    m_start_time = m_curr_time;

  if(!m_initial_pulse_posted && ((m_curr_time - m_start_time) >= m_pulse_delay)) {
    postRangePulse();
    m_initial_pulse_posted = true;
    m_pulse_pending = false;
    return(0);
  }

  if(m_pulse_pending && ((m_curr_time - m_wpt_change_time) >= m_pulse_delay)) {
    postRangePulse();
    m_pulse_pending = false;
    m_initial_pulse_posted = true;
  }

  return(0);
}
