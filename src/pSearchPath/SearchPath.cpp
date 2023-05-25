/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SearchPath.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <stdio.h>  //printf, snprintf, fflush, fopen, fclose
#include "MBUtils.h"
#include "ACTable.h"
// #include "XYFormatUtilsSeglr.h"
#include "XYFormatUtilsSegl.h"
#include "XYFormatUtilsPoly.h"
#include "XYPolygon.h"
#include "SearchPath.h"
// #include "Point.hpp"
// #include "Point_Filter.hpp"
// #include "NodeRecordUtils.h"
// #include "NodeRecord.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

SearchPath::SearchPath()
{
  m_region_str = "";
  m_sensor_range = 5;
  m_region_set = false;
  m_output_key = "SURVEY_UPDATE";
}

//---------------------------------------------------------
// Destructor

SearchPath::~SearchPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SearchPath::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "FOO") 
       cout << "great!";
      else if (key == "RESCUE_REGION"){
        m_region_str = msg.GetString();
        m_region_set = true;
      }
      else if (key == "NODE_REPORT_LOCAL"){
        handleNodeReport(msg.GetString());
      }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SearchPath::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SearchPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  if (m_region_set){
    // String looks like:
    // pts={-31,-37:-7,-92:95,-46:72,10},edge_color=gray90,vertex_color=dodger_blue,vertex_size=5
    m_lawnmower = region2lawnmower(m_region_str, false);
    std::string lmp = stringLawnmower2SegList(m_lawnmower).get_spec();
    std::string spec = lmp;
    std::string lawn = biteStringX(spec, '{'); // remove everything before the first {
    lawn = biteStringX(spec, '}'); // remove everything after the last }
    std::string segllawn = "seglist = " + lawn; // remove everything after the last }
    std::string region = biteStringX(m_region_str, '{'); // remove everything before the first {
    region = "polygon = " + biteStringX(m_region_str, '}'); // remove everything after the last }

    // outputs the region and lawnmower such that it can be viewed with:
    // geoviewer lawnmower.txt
    if (m_debug && m_verbose == 1){
      FILE *fp;
      fp = fopen("lawnmower.txt","w");
      fprintf(fp,"%s",segllawn.c_str());
      fprintf(fp,"%s",("\n"+region).c_str());
      fclose(fp);
    }
    Notify(m_output_key,  "points="+lawn);
    m_region_set = false;
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}


void SearchPath::handleNodeReport(std::string msg)
{
  // NodeRecord node = string2NodeRecord(msg, true);
    m_nav_x = atof(tokStringParse(msg, "X", ',', '=').c_str());
    m_nav_y = atof(tokStringParse(msg, "Y", ',', '=').c_str());
}


//---------------------------------------------------------
    // String looks like:
    // pts={-31,-37:-7,-92:95,-46:72,10},edge_color=gray90,vertex_color=dodger_blue,vertex_size=5
std::string SearchPath::region2lawnmower(std::string region_str, bool north_south){

  XYPolygon polygon = string2Poly(region_str);

  if(!polygon.is_convex()) {
    reportRunWarning("Polygon is not convex!");
  }

  // center of region
  double x = polygon.get_center_x();
  double y = polygon.get_center_y();
  // vehicle position
  double x0 = m_nav_x;
  double y0 = m_nav_y;
  
  // width of region
  double width = 0;
  double height = 0;
  double angle = 0;
  if (polygon.size() != 4){
    reportRunWarning("Polygon is not a rectangle!");
  }

  for (int i = 1; i < 3; i++){
    double vx = polygon.get_vx(i);
    double vy = polygon.get_vy(i);
    double px = polygon.get_vx(i-1);
    double py = polygon.get_vy(i-1);
    double dx = vx - px;
    double dy = vy - py;
    double dl = hypot(dx, dy);


    if (abs(dx) > abs(dy)){
      width = dl;
      angle = -atan2(dy, dx)*180/M_PI; // relative to x axis
    }
    else{
      height = dl;
    }
  }

  // if north-south, take the floor the # of passes in width
  if (!north_south){
    height = floor(height/(2*m_sensor_range))*2*m_sensor_range;
  }
  else{
    width = floor(width/(2*m_sensor_range))*2*m_sensor_range;
  }
  height = height - 2*m_sensor_range;
  width = width - 2*m_sensor_range;

  std::string lawn_mower_str = "format=lawnmower, ";
  lawn_mower_str += "label=searchVehicle, ";
  lawn_mower_str += "lane_width=" + std::to_string(m_sensor_range*2) + ", ";
  lawn_mower_str += "rows=";
  lawn_mower_str += (north_south) ? "north-south, " : "east-west, "; 
  lawn_mower_str += "x=" + std::to_string(x) + ", ";
  lawn_mower_str += "y=" + std::to_string(y) + ", ";
  // lawn_mower_str += "height=" + std::to_string(height-2*m_sensor_range) + ", ";
  // lawn_mower_str += "width=" + std::to_string(width-2*m_sensor_range) + ", ";
  lawn_mower_str += "height=" + std::to_string(height-0*m_sensor_range) + ", ";
  lawn_mower_str += "width=" + std::to_string(width-0*m_sensor_range) + ", ";
  lawn_mower_str += "startx=" + std::to_string(x0) + ", ";
  lawn_mower_str += "starty=" + std::to_string(y0) + ", ";
  lawn_mower_str += "degs=" + std::to_string(angle) + ", ";
  
  return lawn_mower_str;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SearchPath::OnStartUp()
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
    else if(param == "bar") {
      handled = true;
    }
    else if(param == "rescue_range") {
      handled = true;
      m_rescue_range = stod(value);
    }
    else if(param == "sensor_range") {
      handled = true;
      m_sensor_range = stod(value);
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

void SearchPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("RESCUE_REGION", 0);
}


//---------------------------------------------------------
// Procedure: dbg_print()

bool SearchPath::dbg_print(const char *format, ...)
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

bool SearchPath::buildReport() 
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




