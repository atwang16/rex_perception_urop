/************************************************************/
/*    NAME: Austin Wang                                              */
/*    ORGN: MIT                                             */
/*    FILE: ClusterPointCloud.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef ClusterPointCloud_HEADER
#define ClusterPointCloud_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class ClusterPointCloud : public CMOOSApp
{
 public:
   ClusterPointCloud();
   ~ClusterPointCloud();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
