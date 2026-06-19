/************************************************************/
/*    NAME: Lefteris                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <vector>

class PointAssign : public AppCastingMOOSApp
{
 public:
   PointAssign();
   ~PointAssign();

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
   void postToVehicle(unsigned int ix, std::string point,
                      double x=0, double y=0, std::string id="");
   void postToAllVehicles(std::string point);
   bool parseVisitPoint(std::string point, double& x, double& y,
                        std::string& id) const;
   std::string getVisitVar(unsigned int ix) const;
   std::string getColor(unsigned int ix) const;

 private: // Configuration variables
   std::vector<std::string> m_vnames;
   bool m_assign_by_region;

 private: // State variables
   unsigned int m_points_received;
   unsigned int m_points_assigned;
   unsigned int m_invalid_points;
   bool m_first_received;
   bool m_last_received;
   std::vector<unsigned int> m_vehicle_counts;
};

#endif 
