/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SearchPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SearchPath_HEADER
#define SearchPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYSegList.h"


class SearchPath : public AppCastingMOOSApp
{
 public:
   SearchPath();
   ~SearchPath();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   bool dbg_print(const char *format, ...);

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
    std::string m_region_str;
    std::string m_lawnmower;
    XYSegList m_region;
    std::string m_output_key;
    double m_sensor_range;
    double m_rescue_range;
    double m_nav_x;
    double m_nav_y;
    void handleNodeReport(std::string msg);
    std::string region2lawnmower(std::string region_str, bool north_south);
    bool m_region_set;


    bool m_debug;
    uint8_t m_verbose;
    FILE* m_debug_stream;
    static const uint16_t m_fname_buff_size = 128;
    char m_fname[m_fname_buff_size];

 private: // State variables
};

#endif 
