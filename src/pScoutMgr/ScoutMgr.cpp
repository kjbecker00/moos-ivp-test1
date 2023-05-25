/************************************************************/
/*    NAME: Kevin Becker, Ray Turrisi                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ScoutMgr.cpp                                   */
/*    CIRC: Spring 2023                                     */
/************************************************************/

#include "ScoutMgr.h"
#include "ACTable.h"
#include "MBUtils.h"
#include "XYSegList.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "XYFormatUtilsPoly.h"
#include <chrono>
#include <fstream>
#include <iterator>
#include <set>
#include <stdio.h>  //printf, snprintf, fflush, fopen, fclose
#include <stdlib.h> //string conversions (might already be included by default with MBUtils?)
#include <string.h> //memset, memcpy, NULL
#include <time.h>
#include <algorithm>
#include "XYPoint.h"
#include "NodeMessage.h"
using namespace std;

//---------------------------------------------------------
// Constructor()

ScoutMgr::ScoutMgr()
{
  m_input_key = "SWIMMER_ALERT";
  m_found_key = "FOUND_SWIMMER";
  m_output_key = "TRAIL_INFO";
  m_startup_key = "STARTUP";
  m_ally_name_key = "ALLY_NAME";
  m_replan_key = "REPLAN_SEARCH";
  m_antagonize_key = "ANTAGONIZE";
  m_release_ratio = 0.5;
  m_antagonize = true;
  m_vname = "";
  m_output_msg = "";

  m_turning_radius = 5;
  m_hdg_offset = 45;

  m_verbose = false;

  // Points of swimmers that have NOT been found by ANYONE
  m_unvisited_points = std::vector<Point>();
  // Points of swimmers that have been visited by ME
  // m_visited_points = std::vector<Point>();
  // Point of any swimmer that has ever existed (visited or unvisited)
  m_all_points = std::vector<Point>();
  // Points that the vehicle will visit (sorted)
  m_target_points = std::vector<Point>();

  // Vehicle position
  m_nav_x = -0.1;
  m_nav_y = -0.1;

  m_visit_radius = 5;

  // m_publish_points = true;

  m_local_node_report_key = "NODE_REPORT_LOCAL";
  m_mode = "greedy"; // default
  m_replan_interval = 10; //seconds
  m_knows_ally = false;
}

//---------------------------------------------------------
// Destructor

ScoutMgr::~ScoutMgr() {}

//---------------------------------------------------------
// dbg print

bool ScoutMgr::dbg_print(const char *format, ...)
{
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

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ScoutMgr::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    string sval = msg.GetString();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (key == m_input_key)
    {
      handleNewPoint(sval);
    }
    else if (key == m_found_key)
    {
      handleFoundSwimmer(sval);
      m_iterations_since_new_point = 0;
    }
    else if (key == m_local_node_report_key)
    {
      handleNewNodeReport(sval);
    }
    else if (key == "NODE_REPORT")
    {
      handleNewEnemyNodeReport(sval);
    }
    else if (key == "RESCUE_REGION")
    {
      handleOpRegion(sval);
    }
    else if (key == m_ally_name_key)
    {
      m_ally_name = sval;
      if(m_knows_ally == false) {
        m_publish_points = true;
        m_knows_ally = true;
      }
    }
    else if (key == m_replan_key)
    {
      m_publish_points = true;
    }
  }
  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ScoutMgr::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ScoutMgr::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  bool enough_points =  m_all_points.size()>5;
  bool release_condition = m_unvisited_points.size() < m_release_ratio*m_all_points.size();
  std::cout << "m_unvisited_points.size()="<<to_string(m_unvisited_points.size())<< " all_points.size()="<<to_string(m_all_points.size())<<" all_pts*release_ratio=" <<to_string(m_release_ratio*m_all_points.size())<<" ratio="<<to_string(m_release_ratio)<< std::endl;

  if (enough_points){
    if (release_condition)
    {
      std::cout << "Releasing" << std::endl;
      m_antagonize = false;
    }
  }
  
  Notify("ANTAGONIZE", boolToString(m_antagonize));

  if (m_output_msg != "")
  {
    if (m_antagonize){
      std::cout << "Printing output_key=" << m_output_key << std::endl;
      Notify(m_output_key, m_output_msg);
    }
    m_last_output_msg = m_output_msg;
    m_output_msg = "";
  }
  // m_iterations_since_new_point++;
  AppCastingMOOSApp::PostReport();
  return (true);
}

// //---------------------------------------------------------
// // Procedure: publish_list()
// //            publishes the list of points
// void ScoutMgr::publish_list(const std::vector<Point> &list,
//                              std::string out_key)
// {
//   // XYSegList my_seglist;
//   std::string out_str = "points=";
//   for (int i = 0; i < list.size(); i++)
//   {
//     out_str += to_string(list[i].m_x) + "," + to_string(list[i].m_y) + ":";
//   }
//   // removes last : in out_str
//   out_str.pop_back();

//   Notify(out_key, out_str);
// }

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ScoutMgr::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    string orig = *p;
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    // TODO: Need to add error checking on parameters
    bool handled = false;
    if (param == "input_key")
    {
      m_input_key = value;
      handled = true;
    }
    if (param == "startup_key")
    {
      m_startup_key = value;
      handled = true;
    }
    if (param == "release_ratio")
    {
      m_release_ratio = stod(value);
      if (m_release_ratio > 1 || m_release_ratio < 0)
      {
        reportConfigWarning("Release ratio must be between 1 and 0. Defaulting to 0.5");
        m_release_ratio = 0.5;
      }
      handled = true;
    }
    {
      m_startup_key = value;
      handled = true;
    }
    if (param == "output_key")
    {
      m_output_key = value;
      handled = true;
    }
    if (param == "turning_radius")
    {
      m_turning_radius = stod(value);
      handled = true;
    }
    else if (param == "visit_radius")
    {
      m_visit_radius = stod(value);
      handled = true;
    }
    else if (param == "heading_offset")
    {
      m_hdg_offset = stod(value);
      handled = true;
    }
    else if (param == "turning_radius")
    {
      m_turning_radius = stod(value);
      handled = true;
    }
    else if (param == "mode")
    {
      m_mode = tolower(value);
      handled = true;
    }
    else if (param == "vname")
    {
      m_vname = value;
      handled = true;
    }
    else if (param == "ally_vname")
    {
      m_ally_name = value;
      handled = true;
    }
    else if (param == "enemy_vnames")
    {
      std::vector<std::string> vnames = parseString(value, ',');
      for (int i = 0; i < vnames.size(); i++)
      {
        m_enemy_names.push_back(vnames[i]);
      }
      handled = true;
    }
    else if (param == "debug")
    {
      m_debug = (value == tolower("true")) ? true : false;
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
        snprintf(m_fname, m_fname_buff_size, "DBG_%s_%s_DATA.dbg",
                 m_vname.c_str(), fmt);
        handled = true;
      }
    }
    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  // Notify(m_startup_key, "true");
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ScoutMgr::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_input_key, 0);
  Register(m_local_node_report_key, 0);
  Register("NODE_REPORT", 0);
  Register(m_found_key, 0);
  Register(m_ally_name_key, 0);
  Register(m_replan_key, 0);
  Register("RESCUE_REGION", 0);
}

//------------------------------------------------------------
// Procedure: clear_points()
void ScoutMgr::clear_points()
{
  if (m_all_points.size() > 0)
  {
    // m_visited_points.clear();
    m_unvisited_points.clear();
    m_all_points.clear();
  }
}

//------------------------------------------------------------
// Procedure: add_point()
void ScoutMgr::handleNewPoint(std::string point_string)
{
  // takes in a  string in the form of "X= , Y= " and parses it into a point
  // TODO: Need to check to make sure the point types are valid - also are
  // currently converting points to strictly integers, where they are actually
  // floating point numbers. Wrap in try/catch/except
  double x = stod(tokStringParse(point_string, "x", ',', '='));
  double y = stod(tokStringParse(point_string, "y", ',', '='));
  string id = tokStringParse(point_string, "id", ',', '=');
  Point new_point(x, y, id);

  bool point_was_added = true;
  // iterate through the missed points and see if any are within range
  for (int i = 0; i < m_all_points.size(); i++)
  {
    Point p = m_all_points[i];

    // if a point is repeated, assume we have all the points already. Do not add
    // it, and sort the list
    if (p.m_x == new_point.m_x && p.m_y == new_point.m_y)
    {
      point_was_added = false; // if a point is repeated
    }
  }
  // if a point was added, recompute the target points
  if (point_was_added)
  {
    dbg_print("Adding point %s\n", new_point.repr().c_str());
    m_all_points.push_back(new_point);
    m_unvisited_points.push_back(new_point);
    m_publish_points = true;
  }
}

//------------------------------------------------------------
// Procedure: sort_all_points()
//       sorts the list of all points
void ScoutMgr::sort_all_points_list()
{
  m_all_points = sortList(0, 0, m_all_points);
  dbg_print("%0.2f: Sorted all points\n", MOOSTime());
}

//------------------------------------------------------------
// Procedure: gen_target_points()
void ScoutMgr::gen_target_points()
{

}

//------------------------------------------------------------
// Procedure: sortList()
//       sorts the list greedily. Useful for visualizing closest points,
//       mainly for visualizing the point lists in a general priority.

// TODO: If this is slow on the pi where the process hangs, we can try KDTrees
// for an approximate closest point
std::vector<Point> ScoutMgr::sortList(double x, double y,
                                       std::vector<Point> unsorted_list)
{

  std::vector<Point> sorted = std::vector<Point>();

  while (!unsorted_list.empty())
  {
    int next_pt_index = get_closest_point_to(x, y, unsorted_list);
    Point p = unsorted_list[next_pt_index];
    // resets the point id before adding it
    sorted.push_back(p);

    // sets the current x and y
    x = p.m_x;
    y = p.m_y;

    unsorted_list.erase(unsorted_list.begin() + next_pt_index);
  }
  return sorted;
}

OrderedLookup ScoutMgr::study_greedy_bhv(Point point, const std::vector<Point> &swimmers)
{
  OrderedLookup predicted_order;
  std::vector<Point> greedy_list = sortList(point.m_x, point.m_y, swimmers);
  double accumulated_distance = 0;
  for (uint8_t i = 0; i < greedy_list.size(); i++)
  {
    // The initial point to the first point in the greedy list
    if (i == 0)
    {
      accumulated_distance = point.rel_dist(greedy_list[i]);
    }
    // The preceding point to the next list
    else
    {
      accumulated_distance += greedy_list[i - 1].rel_dist(greedy_list[i]);
    }

    predicted_order.id_to_idx[greedy_list[i].m_id] = i;
    predicted_order.travel_distance.push_back(accumulated_distance);
  }

  predicted_order.sorted_points = greedy_list;

  return predicted_order;
}


//------------------------------------------------------------
// Procedure: get_closest_point_to()
int ScoutMgr::get_closest_point_to(double x, double y,
                                    const std::vector<Point> &points)
{
  double min_dist = -1;
  int winning_index = -1;
  for (int i = 0; i < points.size(); i++)
  {
    Point p = points[i];
    double dist = sqrt(pow(p.m_x - x, 2) + pow(p.m_y - y, 2));
    if ((dist < min_dist) || (min_dist == -1))
    {
      min_dist = dist;
      winning_index = i;
    }
  }
  return winning_index;
}

//------------------------------------------------------------
// Procedure: handleNewNodeReport()
void ScoutMgr::handleNewNodeReport(std::string report)
{
  m_nav_x = stod(tokStringParse(report, "X", ',', '='));
  m_nav_y = stod(tokStringParse(report, "Y", ',', '='));
  m_nav_heading = stod(tokStringParse(report, "HDG", ',', '='));
  m_team = tokStringParse(report, "COLOR", ',', '=');
  // updateMissedPoints(); for scout only!
}

void ScoutMgr::handleOpRegion(std::string sval)
{
  XYPolygon region = string2Poly(sval);
  m_center_x = region.get_centroid_pt().x();
  m_center_y = region.get_centroid_pt().y();
}

//------------------------------------------------------------
// Procedure: calculateTargetBearing()
void ScoutMgr::calculateTargetBearing(double x, double y, double hdg)
{
  double rel_x = x - m_center_x;
  double rel_y = y - m_center_y;

  double my_hdg_1 = (90.0-hdg+m_hdg_offset)*M_PI/180.0;
  double my_hdg_2 = (90.0-hdg-m_hdg_offset)*M_PI/180.0;

  double s_hdg_1 = sin(my_hdg_1);
  double s_hdg_2 = sin(my_hdg_2);

  double c_hdg_1 = cos(my_hdg_1);
  double c_hdg_2 = cos(my_hdg_2);

  double dot_product_1 = rel_x*c_hdg_1 + rel_y*s_hdg_1;
  double dot_product_2 = rel_x*c_hdg_2 + rel_y*s_hdg_2;

  if (dot_product_1 > dot_product_2)
  {
    m_target_bearing = m_hdg_offset;
  }
  else
  {
    m_target_bearing = - m_hdg_offset;
  }
}


//------------------------------------------------------------
// Procedure: handleNewEnemyNodeReport()
void ScoutMgr::handleNewEnemyNodeReport(std::string sval)
{
  if (m_antagonize)
  {
    NodeRecord enemy_node_record = string2NodeRecord(sval);
    m_enemy_node_reports[enemy_node_record.getName()] = enemy_node_record;
    // std::cout <<"New Node Report: " << enemy_node_record.getName() << std::endl;
    if (tolower(enemy_node_record.getName()) != tolower(m_ally_name)){
      // std::cout <<"Not ally! " << std::endl;
      if (tolower(enemy_node_record.getType()) == "kayak"){
        calculateTargetBearing(enemy_node_record.getX(), enemy_node_record.getY(), enemy_node_record.getHeading());
        m_output_msg = "name="+enemy_node_record.getName()+"#contact="+enemy_node_record.getName()+"#trail_angle="+std::to_string(m_target_bearing)+"#trail_angle_type=relative";
        std::cout <<"Is enemy search vehicle. m_out_msg=" << m_output_msg<< std::endl;
      }
    }
  }
  else
  {
    std::cout <<"Not antagonizing anymore, so ignoring"<< std::endl;
    // Not antagonizing, so do nothing
  }
}



//------------------------------------------------------------
// Procedure: handleFoundSwimmer()
void ScoutMgr::handleFoundSwimmer(std::string report)
{
  // TODO: Fix this to where we evaluate the next intended point which we go to
  // visit, with the
  //  current location of the vehicle. We can check the latest captured point by
  //  the current vehicle with a waypoint behavior flag
  string id = tokStringParse(report, "id", ',', '=');
  std::string finder = tokStringParse(report, "finder", ',', '=');

  // iterate through the unvisited points and see if any are within range
  for (int i = 0; i < m_unvisited_points.size(); i++)
  {
    Point p = m_unvisited_points[i];
    if (p.m_id == id)
    {
      // remove the point from the unvisited points
      m_unvisited_points.erase(m_unvisited_points.begin() + i);
    }
  }
}

// TODO: Fix with a hashed set to improve performance
bool ScoutMgr::idInList(const std::vector<Point> &list, std::string id)
{
  for (int i = 0; i < list.size(); i++)
  {
    Point p = list[i];
    if (p.m_id == id)
    {
      return true;
    }
  }
  return false;
}

// //------------------------------------------------------------
// // Procedure: updateMissedPoints()
// void ScoutMgr::updateMissedPoints()
// {
//   // iterate through the missed points and see if any are within range
//   for (int i = 0; i < m_unvisited_points.size(); i++)
//   {
//     Point p = m_unvisited_points[i];

//     double v_x = m_nav_x;
//     double v_y = m_nav_y;
//     std::string v_name = m_vname;

//     double dist = pow(p.m_x - v_x, 2) + pow(p.m_y - v_y, 2);
//     if (dist < pow(m_visit_radius, 2))
//     {
//       // remove the point from the missed point, add to visited
//       std::string output = "id=" + p.m_id + ", finder=" + v_name;
//       Notify(m_found_key, output);
//       m_unvisited_points.erase(m_unvisited_points.begin() + i);
//       m_visited_points.push_back(p);
//       i--;
//     }
//   }
// }

//------------------------------------------------------------
// Procedure: buildReport()

bool ScoutMgr::buildReport()
{

  m_msgs << "Listening to             " << m_input_key << std::endl;
  m_msgs << "Poking on startup        " << m_startup_key << std::endl;
  // if (m_publish_points_key != "")
  //   m_msgs << "Publish points key = " << m_publish_points_key << " (currently
  //   " << m_publish_points << ")" << std::endl;

  m_msgs << m_local_node_report_key << ":  x=" << m_nav_x << " y= " << m_nav_y << std::endl;
  m_msgs << m_output_key << ": " << m_last_output_msg << std::endl;
  m_msgs << "Antagonize =" << m_antagonize << std::endl;
  m_msgs << "Releasing once " << to_string(m_release_ratio*m_all_points.size()) << " points remain. Currently at "<<to_string(m_unvisited_points.size())<<" points." << std::endl;
  m_msgs << "Polygon center: " << to_string(m_center_x) << "," << to_string(m_center_y) << std::endl;

  m_msgs << "\n_____________________________________\nUnvisited points:"
         << std::endl;
  for (int i = 0; i < m_unvisited_points.size(); i++)
  {
    m_msgs << m_unvisited_points[i].m_x << "," << m_unvisited_points[i].m_y
           << " id=" << m_unvisited_points[i].m_id << std::endl;
  }
  m_msgs << "\n_____________________________________\nAll points:" << std::endl;
  for (int i = 0; i < m_all_points.size(); i++)
  {
    m_msgs << m_all_points[i].m_x << "," << m_all_points[i].m_y
           << " id=" << m_all_points[i].m_id << std::endl;
  }

  return (true);
}
