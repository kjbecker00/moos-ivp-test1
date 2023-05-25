/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NodeIdentifier.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <stdio.h>  //printf, snprintf, fflush, fopen, fclose
#include "MBUtils.h"
#include "ACTable.h"
#include "NodeIdentifier.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

NodeIdentifier::NodeIdentifier()
{
}

//---------------------------------------------------------
// Destructor

NodeIdentifier::~NodeIdentifier()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool NodeIdentifier::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "FOO") 
       cout << "great!";


     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool NodeIdentifier::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NodeIdentifier::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // m_dest_name = m_ally_name;

  std::string m_moos_varname = "ALLY_NAME";
  std::string m_msg_contents = m_vname;
  

  std::string msg ;
  msg = "src_node=" + m_vname;
  msg += ",dest_node=" + m_ally_name;
  msg += ",var_name=" + m_moos_varname;
  msg += ",string_val=" + m_msg_contents;
  
  Notify("NODE_MESSAGE_LOCAL", msg);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NodeIdentifier::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if (param == "ally_name"){
      m_ally_name = value;
      handled = true;
    }
    else if(param == "vname") {
      m_vname = value;
      handled = true;
    }

    // Handling debugging tools
    else if (param == "verbose")
    {
      m_verbose = stoi(value);
      handled = true;
    }
    else if (param == "debug")
    {
      m_debug = (value == tolower("true"));
      handled = true;
    }
    // where we want to throw debug errors
    else if (param == "debug_stream")
    {
      if (value == "stdout")
      {
        m_debug_stream = stdout;
        memset(m_fname, m_fname_buff_size, '\0');
        handled = true;
      }
      else if (value == "stderr")
      {
        m_debug_stream = stderr;
        memset(m_fname, m_fname_buff_size, '\0');
        handled = true;
      }
      else if (value == "any")
      {
        time_t rawtime;
        struct tm *timeinfo;
        memset(m_fname, m_fname_buff_size, '\0');
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char fmt[m_fname_buff_size];
        memset(fmt, m_fname_buff_size, '\0');
        strftime(fmt, m_fname_buff_size, "%F_%T", timeinfo);
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_DATA.dbg",fmt);
        handled = true;
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void NodeIdentifier::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  // Register("NODE_REPORT_LOCAL", 0);
}


//---------------------------------------------------------
// Procedure: dbg_print()

bool NodeIdentifier::dbg_print(const char *format, ...)
{

  std::cout << "Debug_print" << std::endl;
  // wrap formatted print statements to go to stdout, stderr, or a file
  // if its a file, handle it cleanly
  if (m_debug == true)
  {
    va_list args;
    va_start(args, format);
    if (m_fname[0] == '\0')
    {
      vfprintf(m_debug_stream, format, args);
    }
    else
    {
      m_debug_stream = fopen(m_fname, "a");
      vfprintf(m_debug_stream, format, args);
      fclose(m_debug_stream);
    }
    return true;
  }
  return false;
}


//------------------------------------------------------------
// Procedure: buildReport()

bool NodeIdentifier::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




