/************************************************************/
/*    NAME: Terry Alifragkis                               */
/*    ORGN: MIT                                            */
/*    FILE: BHV_ZigLeg.cpp                                 */
/*    DATE: June 2026                                      */
/************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_ZigLeg.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  IvPBehavior::setParam("name", "zigleg");
  m_domain = subDomain(m_domain, "course");

  m_zig_duration   = 50;
  m_zig_angle      = 60;
  m_zig_delay      = 20;
  
  m_osx             = 0;
  m_osy             = 0;
  m_osh             = 0;
  m_curr_time       = 0;
  m_prev_wpt_index  = 0;
  m_wpt_change_time = 0;
  m_zig_start_time  = 0;
  m_zig_heading     = 0;
  m_have_wpt_index  = false;
  m_zig_pending     = false;
  m_zig_active      = false;

  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("WPT_INDEX", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  param = tolower(param);
  double dval = atof(val.c_str());

  if((param == "zig_duration") && isNumber(val) && (dval > 0)) {
    m_zig_duration = dval;
    return(true);
  }
  else if((param == "zig_angle") && isNumber(val)) {
    m_zig_angle = dval;
    return(true);
  }

  return(false);
}

//---------------------------------------------------------------
// Procedure: updateInfoIn()

bool BHV_ZigLeg::updateInfoIn()
{
  bool ok_osx, ok_osy, ok_osh;
  m_osx = getBufferDoubleVal("NAV_X", ok_osx);
  m_osy = getBufferDoubleVal("NAV_Y", ok_osy);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok_osh);
  m_curr_time = getBufferCurrTime();

  if(!ok_osx || !ok_osy || !ok_osh) {
    postWMessage("No ownship NAV_X/NAV_Y/NAV_HEADING info in info_buffer.");
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
    m_zig_pending = true;
  }

  return(true);
}

//---------------------------------------------------------------
// Procedure: postRangePulse()

void BHV_ZigLeg::postRangePulse()
{
  string pulse = "x=" + doubleToString(m_osx, 2);
  pulse += ",y=" + doubleToString(m_osy, 2);
  pulse += ",radius=" + doubleToString(m_pulse_range, 2);
  pulse += ",duration=" + doubleToString(m_pulse_duration, 2);
  pulse += ",label=bhv_zigleg";
  pulse += ",edge_color=orange,fill_color=orange";

  postMessage("VIEW_RANGE_PULSE", pulse);
}

//---------------------------------------------------------------
// Procedure: angle360()

double BHV_ZigLeg::angle360(double angle) const
{
  while(angle < 0)
    angle += 360;
  while(angle >= 360)
    angle -= 360;
  return(angle);
}

//---------------------------------------------------------------
// Procedure: buildCourseZAIC()

IvPFunction* BHV_ZigLeg::buildCourseZAIC()
{
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(m_zig_heading);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);
  crs_zaic.setValueWrap(true);

  if(!crs_zaic.stateOK()) {
    string warning = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warning);
    return(0);
  }

  return(crs_zaic.extractIvPFunction());
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_ZigLeg::onRunState()
{
  if(!updateInfoIn())
    return(0);

  if(m_zig_pending && ((m_curr_time - m_wpt_change_time) >= m_zig_delay)) {
    postRangePulse();
    m_zig_start_time = m_curr_time;
    m_zig_heading = angle360(m_osh + m_zig_angle);
    m_zig_pending = false;
    m_zig_active = true;
  }

  if(!m_zig_active)
    return(0);

  if((m_curr_time - m_zig_start_time) > m_zig_duration) {
    m_zig_active = false;
    return(0);
  }

  IvPFunction *ipf = buildCourseZAIC();
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}
