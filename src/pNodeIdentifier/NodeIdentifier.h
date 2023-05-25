/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NodeIdentifier.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef NodeIdentifier_HEADER
#define NodeIdentifier_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeMessage.h"
#include <string>

class NodeIdentifier : public AppCastingMOOSApp
{
 public:
   NodeIdentifier();
   ~NodeIdentifier();

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


    std::string m_vname;
    std::string m_dest_name;
    std::string m_ally_name;


    std::string m_bad_vname;
    bool m_debug;
    uint8_t m_verbose;
    FILE* m_debug_stream;
    static const uint16_t m_fname_buff_size = 128;
    char m_fname[m_fname_buff_size];

 private: // State variables
};

#endif 
