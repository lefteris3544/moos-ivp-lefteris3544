/************************************************************/
/*    NAME: Terry Alifragkis                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                 */
/*    DATE: December 29th, 1963                             */
/*    DATE MOD: June 19th, 2026*/
/************************************************************/

#include <iterator>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPoint.h"
#include "PointAssign.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PointAssign::PointAssign()
{
  m_assign_by_region = false;

  m_points_received = 0;
  m_points_assigned = 0;
  m_invalid_points  = 0;
  m_first_received  = false;
  m_last_received   = false;
}

//---------------------------------------------------------
// Destructor

PointAssign::~PointAssign()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "VISIT_POINT") 
       handleVisitPoint(msg.GetString());

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool PointAssign::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PointAssign::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "vname") {
      string vname = tolower(stripBlankEnds(value));
      if(vname != "") {
        m_vnames.push_back(vname);
        m_vehicle_counts.push_back(0);
      }
      handled = true;
    }
    else if(param == "assign_by_region") {
      m_assign_by_region = (tolower(value) == "true");
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if(m_vnames.empty())
    reportConfigWarning("No vehicles configured. Add vname lines.");
  else {
    reverse(m_vnames.begin(), m_vnames.end());
    reverse(m_vehicle_counts.begin(), m_vehicle_counts.end());
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
}


//---------------------------------------------------------
// Procedure: handleVisitPoint()

void PointAssign::handleVisitPoint(string point)
{
  point = stripBlankEnds(point);

  if(point == "firstpoint") {
    m_points_received = 0;
    m_points_assigned = 0;
    m_invalid_points  = 0;
    m_first_received  = true;
    m_last_received   = false;

    for(unsigned int i=0; i<m_vehicle_counts.size(); i++)
      m_vehicle_counts[i] = 0;

    postToAllVehicles(point);
    return;
  }

  if(point == "lastpoint") {
    m_last_received = true;
    postToAllVehicles(point);
    return;
  }

  m_points_received++;

  double x = 0;
  double y = 0;
  string id;
  if(!parseVisitPoint(point, x, y, id)) {
    m_invalid_points++;
    reportRunWarning("Invalid VISIT_POINT: " + point);
    return;
  }

  if(m_vnames.empty()) {
    reportRunWarning("No vehicles available for point assignment.");
    return;
  }

  unsigned int vehicle_ix = 0;
  if(m_assign_by_region && (m_vnames.size() >= 2)) {
    double mid_x = (-25.0 + 200.0) / 2.0;
    vehicle_ix = (x < mid_x) ? 0 : 1;
  }
  else
    vehicle_ix = m_points_assigned % m_vnames.size();

  postToVehicle(vehicle_ix, point, x, y, id);
  m_points_assigned++;
}

//---------------------------------------------------------
// Procedure: postToVehicle()

void PointAssign::postToVehicle(unsigned int ix, string point,
                                double x, double y, string id)
{
  if(ix >= m_vnames.size())
    return;

  Notify(getVisitVar(ix), point);
  m_vehicle_counts[ix]++;

  if(id != "") {
    XYPoint view_point(x, y);
    view_point.set_label(id);
    view_point.set_color("vertex", getColor(ix));
    view_point.set_param("vertex_size", "4");
    Notify("VIEW_POINT", view_point.get_spec());
  }
}

//---------------------------------------------------------
// Procedure: postToAllVehicles()

void PointAssign::postToAllVehicles(string point)
{
  for(unsigned int i=0; i<m_vnames.size(); i++)
    Notify(getVisitVar(i), point);
}

//---------------------------------------------------------
// Procedure: parseVisitPoint()

bool PointAssign::parseVisitPoint(string point, double& x, double& y,
                                  string& id) const
{
  string x_str  = tokStringParse(point, "x", ',', '=');
  string y_str  = tokStringParse(point, "y", ',', '=');
  string id_str = tokStringParse(point, "id", ',', '=');

  if(!isNumber(x_str) || !isNumber(y_str) || (id_str == ""))
    return(false);

  x  = atof(x_str.c_str());
  y  = atof(y_str.c_str());
  id = id_str;
  return(true);
}

//---------------------------------------------------------
// Procedure: getVisitVar()

string PointAssign::getVisitVar(unsigned int ix) const
{
  if(ix >= m_vnames.size())
    return("");

  return("VISIT_POINT_" + toupper(m_vnames[ix]));
}

//---------------------------------------------------------
// Procedure: getColor()

string PointAssign::getColor(unsigned int ix) const
{
  vector<string> colors;
  colors.push_back("yellow");
  colors.push_back("cyan");
  colors.push_back("orange");
  colors.push_back("magenta");

  return(colors[ix % colors.size()]);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool PointAssign::buildReport() 
{
  m_msgs << "Assign by region: " << boolToString(m_assign_by_region) << endl;
  m_msgs << "First received:   " << boolToString(m_first_received) << endl;
  m_msgs << "Last received:    " << boolToString(m_last_received) << endl;
  m_msgs << "Points received:  " << m_points_received << endl;
  m_msgs << "Points assigned:  " << m_points_assigned << endl;
  m_msgs << "Invalid points:   " << m_invalid_points << endl << endl;

  ACTable actab(3);
  actab << "Vehicle | Output Var | Points";
  actab.addHeaderLines();
  for(unsigned int i=0; i<m_vnames.size(); i++)
    actab << m_vnames[i] << getVisitVar(i) << m_vehicle_counts[i];
  m_msgs << actab.getFormattedString();

  return(true);
}


