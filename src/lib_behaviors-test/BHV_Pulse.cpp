/************************************************************/
/*    NAME: Terry Alifragkis                               */
/*    ORGN: MIT                                            */
/*    FILE: BHV_Pulse.cpp                                  */
/*    DATE: June 2026                                      */
/************************************************************/

#include <cstdlib>
#include "BHV_Pulse.h"
#include "BuildUtils.h"
#include "MBUtils.h"

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
  m_pulse_delay    = 10;

  m_osx              = 0;
  m_osy              = 0;
  m_curr_time        = 0;
  m_prev_wpt_index   = 0;
  m_wpt_change_time  = 0;
  m_have_wpt_index   = false;
  m_pulse_pending    = false;

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
  if(!ok_wpt_index)
    return(true);

  if(!m_have_wpt_index) {
    m_prev_wpt_index = wpt_index;
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
  string pulse = "x=" + doubleToString(m_osx, 2);
  pulse += ",y=" + doubleToString(m_osy, 2);
  pulse += ",radius=" + doubleToString(m_pulse_range, 2);
  pulse += ",duration=" + doubleToString(m_pulse_duration, 2);
  pulse += ",label=bhv_pulse";
  pulse += ",edge_color=yellow,fill_color=yellow";

  postMessage("VIEW_RANGE_PULSE", pulse);
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_Pulse::onRunState()
{
  if(!updateInfoIn())
    return(0);

  if(m_pulse_pending && ((m_curr_time - m_wpt_change_time) >= m_pulse_delay)) {
    postRangePulse();
    m_pulse_pending = false;
  }

  return(0);
}
