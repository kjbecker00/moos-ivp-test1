/************************************************************/
/*    NAME: Kevin Becker, Ray Turrisi                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.cpp                                   */
/*    CIRC: Spring 2023                                     */
/************************************************************/

#include "GenRescue.h"
#include "ACTable.h"
#include "MBUtils.h"
#include "XYSegList.h"
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

GenRescue::GenRescue()
{
  m_input_key = "SWIMMER_ALERT";
  m_found_key = "FOUND_SWIMMER";
  m_output_key = "UPDATES_VAR";
  m_startup_key = "STARTUP";
  m_ally_name_key = "ALLY_NAME";
  m_replan_key = "REPLAN_SEARCH";
  m_vname = "";
  m_ctrl_authority_key = "CTRL_AUTH_RESCUE";

  m_turning_radius = 5;

  m_verbose = false;

  // Points of swimmers that have NOT been found by ANYONE
  m_unvisited_points = std::vector<Point>();
  // Points of swimmers that have been visited by ME
  m_visited_points = std::vector<Point>();
  // Point of any swimmer that has ever existed (visited or unvisited)
  m_all_points = std::vector<Point>();
  // Points that the vehicle will visit (sorted)
  m_target_points = std::vector<Point>();

  // Vehicle position
  m_nav_x = -0.1;
  m_nav_y = -0.1;

  m_visit_radius = 5;

  m_publish_points = true;

  m_local_node_report_key = "NODE_REPORT_LOCAL";
  m_node_report_key = "NODE_REPORT";
  m_mode = "greedy"; // default
  m_replan_interval = 10; //seconds
  m_knows_ally = false;
  m_ctrling_self = true;
}

//---------------------------------------------------------
// Destructor

GenRescue::~GenRescue() {}

//---------------------------------------------------------
// dbg print

//A variadic alternative to printf,fprintf, or stdout, which can write to any output
//which is specified at runtime
bool GenRescue::dbg_print(const char *format, ...)
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

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
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
      if(m_ctrling_self == false && m_knows_ally) {

        //If we receive a new input, and if we weren't controlling our self, and if we do know our ally
        //we reclaim control and let our ally know
        m_ctrling_self = true;
        Notify(m_ctrl_authority_key, "RESCUE");

        string m_moos_varname  = m_ctrl_authority_key;  // previously set name of MOOS variable to send
        string m_msg_contents = "RESCUE";  // previously set contents of message;
        string msg;
        msg += "src_node="   + m_vname;
        msg += ",dest_node=" + m_ally_name;
        msg += ",var_name="  + m_moos_varname;
        msg += ",string_val=" + m_msg_contents;

        Notify("NODE_MESSAGE_LOCAL", msg); 
      }
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
    else if (key == m_node_report_key)
    {
      handleNewEnemyNodeReport(sval);
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

bool GenRescue::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Checks the cycleflag (if the first loop has been completed)
  //Replans every ten seconds, or if something else said we should replan
  if ((m_last_replan_time+m_replan_interval) < MOOSTime()) {
    gen_target_points();
    //Other things can also set publish points to true
    m_publish_points = true;
  }

  if (m_publish_points)
  {
    if (m_target_points.size() > 0)
    {
      m_last_replan_time = MOOSTime();
      // Checks if there are still unvisited points. If so, publishes them
      publish_list(m_target_points, m_output_key);
      m_publish_points = false;
    }
    else if (m_target_points.size() == 1)
    {
        Notify(m_output_key, "point=" + to_string(m_target_points[0].m_x) + "," + to_string(m_target_points[0].m_y));
    } else {
      //We have no target points, let the scout know that we are ready to help and let it plan for us
      string m_moos_varname  = m_ctrl_authority_key;  // previously set name of MOOS variable to send
      string m_msg_contents = "SCOUT";  // previously set contents of message;
      
      string msg;
      msg += "src_node="   + m_vname;
      msg += ",dest_node=" + m_ally_name;
      msg += ",var_name="  + m_moos_varname;
      msg += ",string_val=" + m_msg_contents;

      Notify("NODE_MESSAGE_LOCAL", msg); 
    }
    m_publish_points = false;
  }
  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: publish_list()
//            publishes the list of points
void GenRescue::publish_list(const std::vector<Point> &list,
                             std::string out_key)
{
  // XYSegList my_seglist;
  std::string out_str = "points=";
  for (int i = 0; i < list.size(); i++)
  {
    out_str += to_string(list[i].m_x) + "," + to_string(list[i].m_y) + ":";
  }
  // removes last : in out_str
  out_str.pop_back();

  Notify(out_key, out_str);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenRescue::OnStartUp()
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
  Notify(m_startup_key, "true");
  Notify(m_ctrl_authority_key, "RESCUE");
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenRescue::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_input_key, 0);
  Register(m_local_node_report_key, 0);
  Register(m_node_report_key, 0);
  Register(m_found_key, 0);
  Register(m_ally_name_key, 0);
  Register(m_replan_key, 0);
  Register(m_ctrl_authority_key, 0);
}

//------------------------------------------------------------
// Procedure: clear_points()
void GenRescue::clear_points()
{
  if (m_all_points.size() > 0)
  {
    m_visited_points.clear();
    m_unvisited_points.clear();
    m_all_points.clear();
  }
}

//------------------------------------------------------------
// Procedure: add_point()
void GenRescue::handleNewPoint(string point_string)
{
  // takes in a  string in the form of "X= , Y= " and parses it into a point
  // TODO: Need to check to make sure the point types are valid - also are
  // currently converting points to strictly integers, where they are actually
  // floating point numbers. Wrap in try/catch/except
  double x = stod(tokStringParse(point_string, "x", ',', '='));
  double y = stod(tokStringParse(point_string, "y", ',', '='));
  string id = tokStringParse(point_string, "id", ',', '=');
  
  Point new_point(x, y, id);

  if(m_known_swimmers.count(id) == 0) {
    m_known_swimmers.insert(id);
    m_all_points.push_back(new_point);
    m_unvisited_points.push_back(new_point);
    m_publish_points = true;
  }
  // bool point_was_added = true;
  // // iterate through the missed points and see if any are within range
  // for (int i = 0; i < m_all_points.size(); i++)
  // {
  //   Point p = m_all_points[i];

  //   // if a point is repeated, assume we have all the points already. Do not add
  //   // it, and sort the list
  //   if (p.m_x == new_point.m_x && p.m_y == new_point.m_y)
  //   {
  //     point_was_added = false; // if a point is repeated
  //   }
  // }
  // // if a point was added, recompute the target points
  // if (point_was_added)
  // {
  //   dbg_print("Adding point %s\n", new_point.repr().c_str());
  //   m_all_points.push_back(new_point);
  //   m_unvisited_points.push_back(new_point);
  //   std::cout << "Republishing the list, since it was sorted" << std::endl;
  //   m_publish_points = true;
  // }
}

//------------------------------------------------------------
// Procedure: sort_all_points()
//       sorts the list of all points
void GenRescue::sort_all_points_list()
{
  m_all_points = sortList(0, 0, m_all_points);
}

//------------------------------------------------------------
// Procedure: gen_target_points()
void GenRescue::gen_target_points()
{
  if (m_mode == "greedy")
  {
    m_target_points = sortList(m_nav_x, m_nav_y, m_unvisited_points);
  }
  else if (m_mode == "algorithm2")
  {
    // m_target_points =
    //     process_list_algorithm2(m_unvisited_points, m_enemy_node_reports);
  }
  else if (m_mode == "heuristic_search")
  {
    m_target_points =
        dfs_heuristic_search(m_nav_x, m_nav_y, m_unvisited_points, m_enemy_node_reports);
  }
  else if (m_mode == "filtered_heuristic_search")
  {
    m_target_points =
        dfs_heuristic_search(m_nav_x, m_nav_y, m_unvisited_points, m_enemy_node_reports);
    m_target_points = m_point_filter.combine_nearby_points(m_nav_x, m_nav_y, m_visit_radius, m_target_points);
    m_target_points = m_point_filter.smooth_turning(m_nav_x, m_nav_y, m_turning_radius, m_target_points);
  }
}

//------------------------------------------------------------
// Procedure: sortList()
//       sorts the list greedily. Useful for visualizing closest points,
//       mainly for visualizing the point lists in a general priority.

std::vector<Point> GenRescue::sortList(double x, double y,
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

OrderedLookup GenRescue::study_greedy_bhv(Point point, const std::vector<Point> &swimmers)
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

std::vector<Point> GenRescue::cluster_special_swimmer(std::vector<Point> swimmers)
{
  // TODO: Need to fill this out (decided to not implement)
  return swimmers;
}
/*
std::vector<Point> GenRescue::dfs_advantage_maximizing_search(double x, double y,
                                                   std::vector<Point> swimmers,
                                                   std::map<std::string, NodeRecord> &enemy_node_reports) {

/*
  This algorithm simply searches the two closest points it can, and then chooses the point which puts the vehicle in a maximizing or minimizing position
  We check all points we can get to first before the opponent, and of the closest, we check to see which one puts us in the most advantageous position
    We consider where we will be, and where the adversary will be, and then at the line between us, in the center and perpendicular to it, we draw a new line which partitions the plane
    There are either more points on our side or the adversaries side, we choose the point which puts more points on our side than the opponents side
    We simulate the distance the adversary would have traveled by the time we travel a certain distance
    If we are an advantageous position, we consider the points which will keep us in a maximizing position and are closest to the enemy that we can get to before them
    If we are not in an advantageous position, then we consider only the points which will put us in a maximizing position
    We keep considering all the points we can get to first which are closest to the opponent


}
*/

std::vector<Point> GenRescue::dfs_heuristic_search(double x, double y,
                                                   std::vector<Point> swimmers,
                                                   std::map<std::string, NodeRecord> &enemy_node_reports)
{

  //-> Cluster swimmers around a special swimmer (a strategy which can be changed)
  std::chrono::time_point<std::chrono::system_clock> test_start, now;
  std::chrono::duration<double> duration;
  test_start = std::chrono::system_clock::now();
  m_max_num_solutions = 1 << 31;
  m_previous_searches = 0;
  std::vector<Point> capture_points = cluster_special_swimmer(swimmers);

  // TODO: Consider fringe cases for a very small number of points - error handling/edge cases
  if (capture_points.size() <= 3)
  {
    return sortList(x, y, capture_points);
  }

  Point current_point(x, y, "CURRENT_POINT");

  NodeRecord enemy_node_record;
  for (const auto &[key, value] : enemy_node_reports)
  {

    //For each node record
    if (tolower(value.getType()) == "kayak" && !(value.getName().compare(m_ally_name)))
    {
      //If the node report is that of kayak and is not our ally, then we use this node record (redundant)
      //We share this with our scout vehicle
      enemy_node_record = value;

      string m_hostname = m_vname;      // previously set name of ownship
      string m_dest_name = m_ally_name;     // previously set name of vehicle to communicate
      string m_moos_varname  = "SEARCHER_CONTACT_INFO";  // previously set name of MOOS variable to send
      string m_msg_contents = "contact="+value.getName()+",ignore_name="+m_vname+"match_type=kayak";  // previously set contents of message;
      
      string msg;
      msg += "src_node="   + m_hostname;
      msg += ",dest_node=" + m_dest_name;
      msg += ",var_name="  + m_moos_varname;
      msg += ",string_val=" + m_msg_contents;

      Notify("NODE_MESSAGE_LOCAL", msg); 
    } else if(tolower(value.getType()) == "heron" && !(value.getName().compare(m_ally_name))) {
      //If the vehicle type is a heron, and its not our ally vehicle, then we share this
      //do nothing - we don't really care about their scout vehicle
      //this is a placeholder block for now 
    }
  }

  Point enemy_point(enemy_node_record.getX(), enemy_node_record.getY(), "ENEMY_POINT");

  //-> Get a lookup table for the predicted opponents order in capturing a set of points
  OrderedLookup opponent_order = study_greedy_bhv(enemy_point, swimmers);
  OrderedLookup our_greedy_order = study_greedy_bhv(current_point, swimmers);

  // We want it to decay pretty rapidly since there will be a lot of replanning anyways and its hard
  //to predict far out
  double final_discount_target = 0.01;

  //-> Obtain the discount factor which is appropriate for this number of swimmers
  double opponent_discount_factor = Utils::smart_discount_factor(final_discount_target, swimmers.size());

  // With this set of capture points, update each of these points' scores based on our high level assessment

  // cache kernels
  //-> Get kernel functions which can be applied in the continuous domain
  std::vector<Kernel> kernels;

  // TODO: Make this sigma a parameter

  double sigma = 50; // 50 meters
  for (Point point : swimmers)
  {
    kernels.push_back(Kernel(point.m_x, point.m_y, sigma));
  }

  // Place holder, these different point scoring heuristics get applied to each point before we conduct our search

  // TODO: A part of the opponent compensation is not invariant to its order, especially when we concede points.
  // Need to fix this at some other time
  std::vector<std::string> point_score_heuristics = {"opponent_compensation", "gaussian_kernel", "proximity"};

  //-> For each point, apply all the heuristics we have included in this model
  // These heuristics MUST be invariant to the order they are applied in this method, if things must be applied
  // sequentially, they must be applied in the same block
  // Elevation as if you were to imagine a 2-D gaussian kernel returning a Z value, we save this to normalize it later
  double max_elevation = 0;

  uint8_t point_idx = 0;

  std::vector<uint8_t> concession_points;

  // go through all the points and apply individual scoring heuristics
  double max_dist_to_ref = 0;
  double min_dist_to_ref = 999999999.9;

  for (Point &point : capture_points)
  {
    // For a point, apply each heuristic
    for (auto i : point_score_heuristics)
    {
      // Apply the kernels for all the points on an individual point
      // They accumulate, where presumeably, a cluster of points will form the largest 'mountain' in the grid
      // And the center of this 'mountain' should be the center point
      if (i == "gaussian_kernel")
      {
        for (Kernel &k : kernels)
        {
          double elevation = k.apply_kernel(point.m_x, point.m_y);
          point.m_score.m_density_elevation += elevation;
        }
        if (point.m_score.m_density_elevation > max_elevation)
          max_elevation = point.m_score.m_density_elevation;
      }
      else if (i == "opponent_compensation")
      {
        /*
          - This precisely considers what would happen if our enemy did a greedy search, and if we did a greedy search, and then loosely measures who would
          get there first, by looking at the accumulated distance in order to get to that point.
          - If our order in following a greedy path is larger than what it would take for them to get there, we penalize this point proportional to how likely
          they will obtain it firstsince it isn't really worth it
          - Consider the following:
             We both share a closest first point in the greedy search, but they are closer - why even bother going to this point (penalize it)
             If we are closer to this first point, there is no penalty and we should go for it to block them
             If towards the end we would share a point and they would technically be closer, it is discounted since we can't really look that far out
        */
        //for this point, consider the index it would occur in the greedy search
        uint8_t their_idx = opponent_order.id_to_idx[point.m_id];
        uint8_t our_idx = our_greedy_order.id_to_idx[point.m_id];
        //assign this to the point
        point.m_score.m_predicted_opponent_idx = their_idx;

        //For this point, if our opponent would achieve it before us, we penalize it
        //With this, if it is their first two points, and there are more than eight points in the map, we simply ignore them
        if (our_greedy_order.travel_distance[our_idx] > opponent_order.travel_distance[their_idx])
        {

          if (their_idx < 2 && capture_points.size() > 2)
          {
            // If this is one of their first two points and they are closer than us to it, simply concede it
            concession_points.push_back(point_idx);
            XYPoint conceded_point(point.m_x, point.m_y);
            char sbuff[64];
            memset(sbuff, '\0', sizeof(char)*64);
            snprintf(sbuff, 64, "Concession point: %d", concession_points.size());
            conceded_point.set_label(sbuff);
            conceded_point.set_vertex_size(8);
            conceded_point.set_vertex_color_size("yellow", 20);
            m_Comms.Notify("VIEW_POINT", conceded_point.get_spec());
          }
          point.m_score.apply_opponent_penalty(opponent_discount_factor);
        }
      }
      else if (i == "proximity")
      {
        /*
          Here we setup the proximity score, by looking at distance to the current point
          and then normalize this later with min/max normalization
        */
        double c_dist = current_point.dist(point);
        if (c_dist > max_dist_to_ref)
        {
          max_dist_to_ref = c_dist;
        }
        else if (c_dist < min_dist_to_ref)
        {
          min_dist_to_ref = c_dist;
        }
        point.m_score.m_proximity_score = c_dist;
      }
      else if (i == "other")
      {
        // do nothing!
      }
    }
    point_idx++;
  }

  // Remove the points we will forfeit
  // TODO: If the enemy is not heading towards the predicted conceded points, then we add them back to our list of points and replan
  uint8_t concession_idx = 0;
  for (uint8_t i : concession_points)
  {
    capture_points.erase(capture_points.begin() + i - concession_idx);
    concession_idx++;
  }

  // Now go through all the points and apply all the finalized normalization metrics

  for (Point &point : capture_points)
  {
    // squish the elevation
    point.m_score.m_density_elevation /= max_elevation; // essentially normalizes it, the points with the max elevation will have the greatest score
    // min/max scale the proximity score (i.e. closest point to current point has a score of 1 ((min-min)/(max-min)))
    point.m_score.m_proximity_score = 1 - (point.m_score.m_proximity_score - min_dist_to_ref) / (max_dist_to_ref - min_dist_to_ref);
    point.m_score.update_total_score();
  }

  // Now find the weighted center of points
  // Currently not used, was going to be, but no longer
  double c_x = 0;
  double c_y = 0;
  double num_x, num_y = 0;
  double denom = 0;
  for (Point &point : capture_points)
  {
    num_x += point.m_x * point.m_score.m_density_elevation;
    num_y += point.m_y * point.m_score.m_density_elevation;
    denom += point.m_score.m_density_elevation;
  }
  c_x = num_x / denom;
  c_y = num_y / denom;
  Point center_of_elevation(c_x, c_y, "CENTER_OF_SWIMMERS");
  dbg_print("Center of swimmers: %s\n", center_of_elevation.repr().c_str());
  kernels.clear();

  // Now that we have a set of weighted points, get ready to search for an optimal path recursively, from here, deploy the first three nodes

  std::sort(capture_points.begin(), capture_points.end(), Utils::compare_scores);

  m_best_path_data.m_path_length = 99999999;
  m_best_path_data.m_point_score = 0;

  for (int i = 0; (i < 2) && i < capture_points.size(); i++)
  {
    std::vector<Point> remaining_points(capture_points);
    Point next_point = remaining_points[remaining_points.size() - i - 1];
    remaining_points.pop_back();

    // Initialize the DFS's path and path data
    std::vector<Point> path = {next_point};
    PathData path_data;
    path_data.m_path_length = current_point.dist(next_point);
    path_data.m_point_score *= next_point.m_score.m_total_score;
    // TODO: Setup path continuity/momentum
    path_data.m_path_continuity = 1;
    dfs_heuristic_recurse(path, remaining_points, path_data);
  }
  dbg_print("Best path score: %0.4f\n", m_best_path_score);
  dbg_print("%0.2f: Path \n", MOOSTime(), current_point.repr().c_str());
  for (Point p : m_best_path)
  {
    dbg_print("\t- {%s, %s}", p.repr().c_str(), p.m_score.repr().c_str());
  }

  dbg_print("\n");
  now = std::chrono::system_clock::now();
  duration = now - test_start;
  dbg_print("Time to compute: %0.6f\n", duration.count());
  return m_best_path;
}

void GenRescue::dfs_heuristic_recurse(std::vector<Point> path, std::vector<Point> remaining_capture_points, PathData path_data)
{

  //-> If we reach the base case, evaluate the path and either save it or backtrack
  if (remaining_capture_points.size() == 0)
  {
    m_previous_searches++;
    // then we have all the points in our path (at the moment)
    // TODO: This is where we evaluate the final path,
    if (path_data.m_point_score * path_data.m_path_continuity > m_best_path_data.m_point_score * m_best_path_data.m_path_continuity)
    {
      // Save the path and the path data based on these conditions
      //dbg_print("%0.2f: Found new best path\n\tOld: L = %0.2f\n\tNew: L = %0.2f\n", MOOSTime(), m_best_path_data.m_path_length, path_data.m_path_length);
      //dbg_print("%0.2f: Found new best path\n\tOld: S = %0.2f\n\tNew: S = %0.2f\n", MOOSTime(), m_best_path_data.m_point_score, path_data.m_point_score);
      m_best_path = path;
      m_best_path_data = path_data;
    }
    return;
  }

  //-> We are not at the base case, but evaluate conditions for pruning the path

  // If the current path length is up to 1.5 times that of the best path length, prune (TODO: tune number later)
  double bad_path_length_scale = 1.5;
  if (path_data.m_path_length >= m_best_path_data.m_path_length * bad_path_length_scale || path_data.m_point_score < m_best_path_data.m_point_score)
  {
    return;
  }

  //-> If we are not at the base case and not pruning this path, then we proceed to the algorithm once more

  // Sort the points and get the n closest points, the closest points are stored in the back
  Point current_point = path.back();

  // double prev_heading;
  double max_dist_to_ref = 0;
  double min_dist_to_ref = 999999999.9;

  // if (path.size() > 1)
  // {
  //   Point nd_to_last = *(path.end() - 2);
  //   prev_heading = atan2(current_point.m_y - nd_to_last.m_y, current_point.m_x - nd_to_last.m_x);
  // }
  // else
  // {
  //   prev_heading = m_nav_heading;
  // }

  if (remaining_capture_points.size() == 1)
  {
    remaining_capture_points[0].m_score.m_proximity_score = 1;
    remaining_capture_points[0].m_score.update_total_score();
  }
  else
  {
    uint8_t point_idx = 0;
    for (Point &p : remaining_capture_points)
    {
      double c_dist = current_point.dist(p);
      if (c_dist > max_dist_to_ref)
      {
        max_dist_to_ref = c_dist;
      }
      else if (c_dist < min_dist_to_ref)
      {
        min_dist_to_ref = c_dist;
      }
      p.m_score.m_proximity_score = c_dist;
      p.m_score.m_proximity_score = pow(static_cast<double>(point_idx) / static_cast<double>(remaining_capture_points.size() - 1), 3);
      p.m_score.update_total_score();
      point_idx++;
    }
    for (Point &p : remaining_capture_points)
    {
      double c_dist = current_point.dist(p);
      if (c_dist > max_dist_to_ref)
      {
        max_dist_to_ref = c_dist;
      }
      else if (c_dist < min_dist_to_ref)
      {
        min_dist_to_ref = c_dist;
      }
      p.m_score.m_proximity_score = c_dist;
      // p.m_score.m_proximity_score = pow(static_cast<double>(point_idx) / static_cast<double>(remaining_capture_points.size() - 1),3);
      p.m_score.m_proximity_score = 1 - (p.m_score.m_proximity_score - min_dist_to_ref) / (max_dist_to_ref - min_dist_to_ref);
      p.m_score.update_total_score();
      point_idx++;
    }
  }

  std::sort(remaining_capture_points.begin(), remaining_capture_points.end(), Utils::compare_scores);

  //->For the top candidate paths, explore them
  for (int i = 0; (i < 2) && i < remaining_capture_points.size(); i++)
  {
    std::vector<Point> new_remaining_points(remaining_capture_points);
    Point next_point = new_remaining_points[new_remaining_points.size() - i - 1];
    new_remaining_points.pop_back();
    path.push_back(next_point);

    path_data.m_path_length += current_point.dist(next_point);
    // this is a form of discounting the points later in the line - they are not as penalizing per-se
    path_data.m_point_score *= pow(next_point.m_score.m_total_score, static_cast<double>(remaining_capture_points.size()));

    double angle_to = atan2(next_point.m_y - current_point.m_y, next_point.m_x - current_point.m_x);

    // double continuity = Utils::get_continuity(prev_heading, angle_to);

    // // TODO: Setup path continuity/momentum
    // path_data.m_path_continuity *= pow(continuity, 0.5);
    dfs_heuristic_recurse(path, new_remaining_points, path_data);
  }
}

//------------------------------------------------------------
// Procedure: get_closest_point_to()
int GenRescue::get_closest_point_to(double x, double y,
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
void GenRescue::handleNewNodeReport(string report)
{
  m_nav_x = stod(tokStringParse(report, "X", ',', '='));
  m_nav_y = stod(tokStringParse(report, "Y", ',', '='));
  m_nav_heading = stod(tokStringParse(report, "HDG", ',', '='));
  updateMissedPoints();
}

//------------------------------------------------------------
// Procedure: handleNewEnemyNodeReport()
void GenRescue::handleNewEnemyNodeReport(std::string sval)
{
  NodeRecord enemy_node_record = string2NodeRecord(sval);
  m_enemy_node_reports[enemy_node_record.getName()] = enemy_node_record;
}

//------------------------------------------------------------
// Procedure: handleFoundSwimmer()
void GenRescue::handleFoundSwimmer(std::string report)
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

      // to prevent the vehicle from re-sorting and re-publishing
      // the path for every found swimmer
      if (finder != m_vname && !m_mode.compare("heursitic_search"))
      {
        gen_target_points();
        m_publish_points = true;
      } else {
        //if we are in the heuristic search, we want to keep replanning
        gen_target_points();
        m_publish_points = true;
      }
    }
  }
}

// TODO: Fix with a hashed set to improve performance
bool GenRescue::idInList(const std::vector<Point> &list, std::string id)
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

//------------------------------------------------------------
// Procedure: updateMissedPoints()
void GenRescue::updateMissedPoints()
{
  // iterate through the missed points and see if any are within range
  for (int i = 0; i < m_unvisited_points.size(); i++)
  {
    Point p = m_unvisited_points[i];

    double v_x = m_nav_x;
    double v_y = m_nav_y;
    std::string v_name = m_vname;

    double dist = pow(p.m_x - v_x, 2) + pow(p.m_y - v_y, 2);
    if (dist < pow(m_visit_radius, 2))
    {
      // remove the point from the missed point, add to visited
      std::string output = "id=" + p.m_id + ", finder=" + v_name;
      Notify(m_found_key, output);
      m_unvisited_points.erase(m_unvisited_points.begin() + i);
      m_visited_points.push_back(p);
      i--;
    }
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport()
{

  m_msgs << "Listening to             " << m_input_key << std::endl;
  m_msgs << "Publishing points to     " << m_output_key << std::endl;
  m_msgs << "Poking on startup        " << m_startup_key << std::endl;
  // if (m_publish_points_key != "")
  //   m_msgs << "Publish points key = " << m_publish_points_key << " (currently
  //   " << m_publish_points << ")" << std::endl;

  m_msgs << m_local_node_report_key << ":  x=" << m_nav_x << " y= " << m_nav_y
         << std::endl;

  m_msgs
      << "\n_____________________________________\nTarget points (algorithm = "
      << m_mode << "):" << std::endl;
  for (int i = 0; i < m_target_points.size(); i++)
  {
    m_msgs << m_target_points[i].m_x << "," << m_target_points[i].m_y
           << " id=" << m_target_points[i].m_id << std::endl;
  }
  m_msgs << "\n_____________________________________\nUnvisited points:"
         << std::endl;
  for (int i = 0; i < m_unvisited_points.size(); i++)
  {
    m_msgs << m_unvisited_points[i].m_x << "," << m_unvisited_points[i].m_y
           << " id=" << m_unvisited_points[i].m_id << std::endl;
  }
  m_msgs << "\n_____________________________________\nVisited points (visit "
            "radius="
         << m_visit_radius << "):" << std::endl;
  for (int i = 0; i < m_visited_points.size(); i++)
  {
    m_msgs << m_visited_points[i].m_x << "," << m_visited_points[i].m_y
           << " id=" << m_visited_points[i].m_id << std::endl;
  }
  m_msgs << "\n_____________________________________\nAll points:" << std::endl;
  for (int i = 0; i < m_all_points.size(); i++)
  {
    m_msgs << m_all_points[i].m_x << "," << m_all_points[i].m_y
           << " id=" << m_all_points[i].m_id << std::endl;
  }

  return (true);
}
