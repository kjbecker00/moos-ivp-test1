/************************************************************/
/*    NAME: Ray Turrisi                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SwimSearch.cpp                                  */
/*    CIRC: May 15, 2023                                    */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SwimSearch.h"
#include "NodeMessage.h"
#include "XYFormatUtilsPoly.h"
#include <algorithm>
#include <random>
#include <time.h>
#include <chrono>
using namespace std;

//---------------------------------------------------------
// Constructor()

SwimSearch::SwimSearch()
{
  //Keys for subscriptions and parameters
  m_input_key = "SWIMMER_ALERT";
  m_found_key = "FOUND_SWIMMER";
  m_rescue_region_key = "RESCUE_REGION";
  m_output_key = "SEARCH_UPDATES";
  m_ally_name_key = "ALLY_NAME";
  m_local_node_report_key = "NODE_REPORT_LOCAL";
  m_node_report_key = "NODE_REPORT";
  m_scouted_swimmer_key = "SCOUTED_SWIMMER";
  m_antagonize_key = "ANTAGONIZE";
  m_ctrl_authority_rescue_key = "CTRL_AUTH_RESCUE";
  m_vname = "";

  // Point of any swimmer that has ever existed (visited or unvisited)
  m_all_points = std::vector<Point>();

  // Vehicle position and heading
  m_nav_x = -0.1;
  m_nav_y = -0.1;
  m_nav_heading = 0;

  // Default rescue ranges
  m_rescue_rng_min = 3;
  m_rescue_rng_min = 5;

  //State description parameters
  m_knows_ally = false;
  m_antagonizing = true;
  m_grid_initialized = false;
  m_planning_for_rescue = false;
}

//---------------------------------------------------------
// Destructor

SwimSearch::~SwimSearch()
{
}

//---------------------------------------------------------
// dbg print

bool SwimSearch::dbg_print(const char *format, ...)
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

bool SwimSearch::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    string sval = msg.GetString();

    if (key == m_input_key)
    {
      handleNewPoint(sval);
    }
    else if (key == m_found_key)
    {
      handleFoundSwimmer(sval);
    }
    else if (key == m_local_node_report_key)
    {
      handleNewNodeReport(sval);
    }
    else if (key == m_node_report_key)
    {
      handleOtherNodeReport(sval);
    }
    else if (key == m_ctrl_authority_rescue_key)
    {
      //If the rescue vehicle forfeits control authority, then we are also planning for the rescue vehicle
      if (sval == "SCOUT")
      {
        m_planning_for_rescue = true;
      }
      else
      {
        m_planning_for_rescue = false;
      }
    }
    else if (key == m_antagonize_key)
    {
      //Is the scout vehicle currently antagonizing or not
      if (sval == "true")
      {
        m_antagonizing = true;
      }
      else
      {
        m_antagonizing = false;
      }
    }
    else if (key == m_scouted_swimmer_key)
    {
      string m_moos_varname = "SWIMMER_ALERT"; // previously set name of MOOS variable to send
      string m_msg_contents = sval;            // previously set contents of message;

      string msg;
      msg += "src_node=" + m_vname;
      msg += ",dest_node=" + m_ally_name;
      msg += ",var_name=" + m_moos_varname;
      msg += ",string_val=\"" + m_msg_contents + "\"";

      Notify("NODE_MESSAGE_LOCAL", msg);
      //sendMessage(m_ally_name, m_moos_varname, sval);
    }
    else if (key == m_rescue_region_key)
    {
      m_rescue_region = string2Poly(sval);
      if (!m_rescue_region.is_convex())
      {
        dbg_print("Polygon is not convex\n");
        // postWMessage("Badly formed RESCUE_REGION");
      }

      if (m_grid_initialized == false)
      {
        m_grid_initialized = true;
        // Now we have to initialize our searchgrid
        // We obtain the bottom most point, and this will be our pin point
        // From this pin point, we will find the smallest angle to the other points, to construct our angle of rotation

        // Maybe this following process is inefficient, but I am trying to get the bottom most point,
        //  and then get the smallest rotation we need to apply to make a mapping function between
        //  continuous domain and the discrete representation

        // Get the lowest point
        int num_points = m_rescue_region.size();
        std::vector<XYPoint> points;
        XYSegList seglst = m_rescue_region.exportSegList();
        for (int i = 0; i < num_points; i++)
        {
          points.push_back(m_rescue_region.get_point(i));
        }

        double min_y = 1000000;
        XYPoint min_point;
        int min_idx;
        for (int i = 0; i < points.size(); i++)
        {
          XYPoint p = points[i];
          if (p.get_vy() < min_y)
          {
            min_point = p;
            min_y = p.get_vy();
            min_idx = i;
          }
        }

        // we have the lowest point in the grid
        points.erase(points.begin() + min_idx);

        // Find the hypotenuse index
        double longest_dist = -1;
        int hyp_idx;
        for (int i = 0; i < points.size(); i++)
        {
          XYPoint p = points[i];
          double dist = pow(pow(min_point.get_vy() - p.get_vy(), 2) + pow(min_point.get_vx() - p.get_vx(), 2), 0.5);
          if (dist > longest_dist)
          {
            longest_dist = dist;
            hyp_idx = i;
          }
        }

        // We can't do anything with the opposite point
        points.erase(points.begin() + hyp_idx);

        // Find the smallest rotation angle needed, so the point we have is the bottom left point
        // in our grid
        double min_angle = 10000;
        int min_angle_idx;

        // For the remaining two points, get the smallest angle, and its idx
        for (int i = 0; i < points.size(); i++)
        {
          XYPoint p = points[i];
          double ang_between = atan2(p.get_vy() - min_point.get_vy(), p.get_vx() - min_point.get_vx());
          if (ang_between < min_angle)
          {
            min_angle = ang_between;
            min_angle_idx = i;
          }
        }

        // Now we have the mininum rotation angle needed, and the point which with our pin point makes
        // the bottom of the rectangle
        XYPoint bottom_right = points[min_angle_idx];
        Point pin_point(min_point.get_vx(), min_point.get_vy());
        points.erase(points.begin() + min_angle_idx);

        XYPoint top_left = points[0];

        double length = pow(pow(min_point.get_vy() - bottom_right.get_vy(), 2) + pow(min_point.get_vx() - bottom_right.get_vx(), 2), 0.5);
        double height = pow(pow(min_point.get_vy() - top_left.get_vy(), 2) + pow(min_point.get_vx() - top_left.get_vx(), 2), 0.5);

        m_search_grid_stp_size = 0.5;

        min_angle = std::fmod(2 * M_PI + min_angle, 2 * M_PI);
        m_search_grid = SearchGrid(pin_point, length, height, m_search_grid_stp_size, min_angle);
        for (double i = 0; i < height; i += (2 * m_rescue_rng_min * 0.9))
        {
          for (double j = 0; j < length; j += (2 * m_rescue_rng_min * 0.9))
          {
            double nxt_x = pin_point.m_x + j * cos(min_angle) - i * sin(min_angle);
            double nxt_y = pin_point.m_y + j * sin(min_angle) + i * cos(min_angle);
            m_sample_points.push_back(Point(nxt_x, nxt_y));
          }
        }
      }
    }
  }
  return (true);
}

void SwimSearch::handleOtherNodeReport(string sval)
{
  // Track the other non-scout vehicles, and include this in our data structure
  NodeRecord other_node_record = string2NodeRecord(sval);
  if (m_grid_initialized)
  {
    // If it is a heron
    if (!(tolower(other_node_record.getType()) == "heron"))
    {
      // if its our ally, we need to track its current position
      if (other_node_record.getName() == m_ally_name)
      {
        double x = other_node_record.getX();
        double y = other_node_record.getY();
        m_ally_heading = other_node_record.getHeading();
        m_ally_point = Point(x, y);
        m_search_grid.reduce_search_space(x, y, m_rescue_rng_min, 255);
      }
      else
      {
        // if it is the enemy rescue vehicle, then we just need to update our grid
        double x = other_node_record.getX();
        double y = other_node_record.getY();
        m_search_grid.reduce_search_space(x, y, m_rescue_rng_min, 255);
      }
    }
    else
    {
      // do nothing (other scout)
    }
  }
}

void SwimSearch::handleNewNodeReport(string report)
{
  m_nav_x = stod(tokStringParse(report, "X", ',', '='));
  m_nav_y = stod(tokStringParse(report, "Y", ',', '='));
  m_nav_heading = stod(tokStringParse(report, "HDG", ',', '='));
  m_current_point = Point(m_nav_x, m_nav_y);

  if (m_grid_initialized)
  {
    m_search_grid.reduce_search_space(m_nav_x, m_nav_y, m_rescue_rng_min, 255);
    m_search_grid.reduce_search_space(m_nav_x, m_nav_y, m_rescue_rng_max, 4);
  }
}

void SwimSearch::handleNewPoint(string sval)
{
  double x = stod(tokStringParse(sval, "x", ',', '='));
  double y = stod(tokStringParse(sval, "y", ',', '='));
  string id = tokStringParse(sval, "id", ',', '=');

  if (m_known_swimmers.count(id) == 0)
  {
    m_known_swimmers.insert(id);
    Point p(x, y, id);
    m_all_points.push_back(p);
  }
}

void SwimSearch::handleFoundSwimmer(string sval)
{
  // We don't need to use this yet
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SwimSearch::OnConnectToServer()
{
  registerVariables();
  return (true);
}

std::vector<Point> SwimSearch::find_fast_path(int depth, const Point &centroid, std::vector<Point> point_set, std::vector<Point> path, double prev_heading)
{
  // check for base case

  if (point_set.size() == 1)
  {
    path.push_back(point_set[0]);
  }
  else if (depth == 0)
  {
    point_set.pop_back();
    Point p = path.back();
    double heading = atan2(path.back().m_y - point_set.back().m_y, path.back().m_x - point_set.back().m_x);
    std::sort(point_set.begin(), point_set.end(), [&centroid, &p, &heading](const Point &A, const Point &B)
              { return Utils::compare_score_A(A, B, centroid, p, heading); });
    path.push_back(point_set.back());
    return path;
  }
  else
  {
    // sort the points w/rt to how close they are to this current point and how straight they are w/rt to our heading
    point_set.pop_back();
    Point p = path.back();
    double heading = atan2(path.back().m_y - point_set.back().m_y, path.back().m_x - point_set.back().m_x);
    std::sort(point_set.begin(), point_set.end(), [&centroid, &p, &heading](const Point &A, const Point &B)
              { return Utils::compare_score_A(A, B, centroid, p, heading); });
    path.push_back(point_set.back());
    depth--;
    return find_fast_path(depth, centroid, point_set, path, heading);
  }
  return path;
}

std::vector<Point> SwimSearch::find_greedy_path(std::vector<Point> points, std::vector<Point> path)
{
  // check for base case
  if (points.size() == 1)
  {
    path.push_back(points[0]);
    return path;
  }
  else
  {
    points.pop_back();
    Point p = path.back();
    std::sort(points.begin(), points.end(), [&p](const Point &A, const Point &B)
              { return Utils::compare_distance_to_ref(A, B, p); });
    while ((points.size() > 1) && (p.dist(points.back()) < m_rescue_rng_min * 2))
      points.pop_back();
    path.push_back(points.back());
    return find_greedy_path(points, path);
  }
}

std::vector<Point> SwimSearch::get_path_between_centroids(Point reference_point, double visit_radius, bool greedy)
{
  // Get the clusters of the current map
  m_latest_search_clusters = m_search_grid.get_clusters();

  std::sort(m_latest_search_clusters.begin(), m_latest_search_clusters.end(), Utils::compare_by_mass);

  // Get all the clusters with centroids larger than half the search radii area (get rid of really small points, but still care about the little guy)
  std::vector<Point> new_search_points;

  // Discretize the search area representation to correspond to the clusters representation
  int search_area = static_cast<int>(M_PI * pow((visit_radius / m_search_grid_stp_size), 2));

  // If the largest element in the clusters has a mass greater than the search area
  std::default_random_engine generator(static_cast<unsigned int>(MOOSTime()));

  bool skipping = (m_latest_search_clusters.back().elements.size() > search_area);
  // bool skipping = false;
  for (auto it = m_latest_search_clusters.rbegin(); it != m_latest_search_clusters.rend(); ++it)
  {
    if (skipping && (it->mass < search_area))
      continue;
    Point p = m_search_grid.grid_to_map(it->centroid.first, it->centroid.second);
    new_search_points.push_back(p);
    int hydraulic_radius = static_cast<int>(2 * it->elements.size() / (it->edges_elements.size()));
    for (int i = 0; i < hydraulic_radius; i++)
    {
      // Approximate the hydraulic diameter of the lump, take the mass, divide by the mass to point conversion
      std::normal_distribution<double> distribution(0.0, hydraulic_radius * m_search_grid_stp_size);
      double xOffset = distribution(generator);
      double yOffset = distribution(generator);
      double nxt_x = p.m_x + xOffset;
      double nxt_y = p.m_y + yOffset;
      Point in_grid = m_search_grid.map_to_grid(nxt_x, nxt_y);

      std::pair<int, int> p_in_c(in_grid.m_y_idx, in_grid.m_x_idx);

      //These functions are not the same, a point upon insertion will get clamped to a wall, which 
      //means that elements which are out of bounds may be on the edge of a cluster, but a point may not actually be 
      //inside the map. This will be updated in another implementation

      //If the point is not in the cluster or it isn't in the search grid
      if ((it->elements.count(p_in_c) == 0) || !m_search_grid.is_point_in_map(in_grid.m_x,in_grid.m_y)) {
        continue;
      }

      new_search_points.push_back(in_grid);
    }
  }

  // Now we have all the centroids of all the clusters, now sort them in a greedy search so we have the fastest route between them
  if (!greedy)
  {
    //If we are not doing a greedy search, return them in their default order from largest centroid to least
    return new_search_points;
  }
  else
  {
    //Otherwise, sort them w/rt to the reference point (scout vehicle location or the rescue vehicle location)
    std::vector<Point> path;
    path.push_back(reference_point);
    return find_greedy_path(new_search_points, path);
  }
}

//---------------------------------------------------------
// Procedure: SendMessage()
// Sends message to some other vehicle

void SwimSearch::sendMessage(std::string dest, std::string var_name, std::string msg_contents) {
      string msg;
      msg += "src_node=" + m_vname;
      msg += ",dest_node=" + dest;
      msg += ",var_name=" + var_name;
      msg += ",string_val=\"" + msg_contents + "\"";
      Notify("NODE_MESSAGE_LOCAL", msg);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SwimSearch::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  /*
    Maintain a state machine which progresses through the following -
     - Waits until we have the rescue region, and when we do, we form our grid data structure
     - When we have the rescue region, while it is getting updated in OnNewMail, we also
  */

  std::chrono::time_point<std::chrono::system_clock> test_start, now;
  std::chrono::duration<double> duration;
  test_start = std::chrono::system_clock::now();

  while (m_all_points.size())
  {
    Point p = m_all_points.back();
    m_all_points.pop_back();
    m_search_grid.reduce_search_space(p.m_x, p.m_y, m_rescue_rng_min, 128);
  }

  if ((m_last_replan_time + m_replan_interval) <= MOOSTime())
  {
    m_replan = true;
  }

  if (m_replan && m_grid_initialized)
  {
    Point center_of_uncertainty = m_search_grid.get_centroid();
    XYPoint center_of_uncertainty_m(center_of_uncertainty.m_x, center_of_uncertainty.m_y);
    center_of_uncertainty_m.set_label("Center of Uncertainty");
    center_of_uncertainty_m.set_vertex_size(8);
    center_of_uncertainty_m.set_vertex_color_size("green", 20);
    Notify("CENTER_OF_UNCERTAINTY", center_of_uncertainty_m.get_spec());
    m_Comms.Notify("VIEW_POINT", center_of_uncertainty_m.get_spec());

    if (m_antagonizing == false)
    {
      // clean the sample points in the grid
      if (m_latest_search_clusters.size() > 0)
      {
        int i = 0;
        for (auto c : m_latest_search_clusters)
        {
          if ((m_search_grid_stp_size * c.mass / 255) < M_PI * pow(m_rescue_rng_min, 2))
            continue;
          Point map_point = m_search_grid.grid_to_map(c.centroid.first, c.centroid.second);
          XYPoint cluster_center(map_point.m_x, map_point.m_y);
          cluster_center.set_label("C:" + to_string(i));
          cluster_center.set_vertex_size(8);
          cluster_center.set_vertex_color_size("red", 12);
          std::string msg = cluster_center.get_spec() + ",duration=" + to_string(m_replan_interval);
          m_Comms.Notify("VIEW_POINT", msg);
          i++;
        }
      }

      std::vector<Point> path = get_path_between_centroids(m_current_point, m_rescue_rng_min, true);

      std::string next_path = "points=";
      for (int i = 0; i < path.size(); i++)
      {
        next_path += std::to_string(path[i].m_x) + "," + std::to_string(path[i].m_y) + ":";
        XYPoint new_point(path[i].m_x, path[i].m_y);
        new_point.set_label("C:" + to_string(i));
        new_point.set_vertex_size(8);
        new_point.set_vertex_color_size("red", 12);
        std::string msg = new_point.get_spec() + ",duration=" + to_string(m_replan_interval);
        m_Comms.Notify("VIEW_POINT", msg);
      }

      next_path.pop_back();
      Notify(m_output_key, next_path);
    }

    if (m_planning_for_rescue)
    {

      std::vector<Point> path = get_path_between_centroids(m_ally_point, m_rescue_rng_min, true);

      if((m_current_point.dist(m_ally_point) < 40) && !m_antagonizing) {
        //If our two vehicles are close, have the rescue vehicle to the reversed path
        std::reverse(path.begin(), path.end());
      } 

      std::string next_path = "points=";
      for (int i = 0; i < path.size(); i++)
      {
        next_path += std::to_string(path[i].m_x) + "," + std::to_string(path[i].m_y) + ":";
        XYPoint new_point(path[i].m_x, path[i].m_y);
        new_point.set_label("C:" + to_string(i));
        new_point.set_vertex_size(8);
        new_point.set_vertex_color_size("red", 12);
        std::string msg = new_point.get_spec() + ",duration=" + to_string(m_replan_interval);
        m_Comms.Notify("VIEW_POINT", msg);
      }
      next_path.pop_back();

      string msg;
      msg += "src_node=" + m_vname;
      msg += ",dest_node=" + m_ally_name;
      msg += ",var_name=SURVEY_UPDATE";
      msg += ",string_val=\"" + next_path + "\"";

      Notify("NODE_MESSAGE_LOCAL", msg);
      //sendMessage(m_ally_name, "SURVEY_UPDATE", next_path);
      Notify(m_output_key, next_path);
      m_planning_for_rescue = false;
    }

    m_last_replan_time = MOOSTime();
    m_replan = false;
  }

  if (m_debug == true)
  {
    std::string tmpname = "current_search.pgm";
    m_search_grid.save_grid(tmpname);
  }

  now = std::chrono::system_clock::now();
  duration = now - test_start;
  
  dbg_print("Time to compute: %0.6f\n", duration.count());

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SwimSearch::OnStartUp()
{
  /*
    TODO:
      - Parse and set parameters for search
  */
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
    if (param == "output_key")
    {
      m_output_key = value;
      handled = true;
    }
    else if (param == "rescue_rng_min")
    {
      m_rescue_rng_min = stod(value);
      handled = true;
    }
    else if (param == "rescue_rng_max")
    {
      m_rescue_rng_max = stod(value);
      handled = true;
    }
    else if (param == "vname")
    {
      m_vname = value;
      handled = true;
    }
    else if (param == "ally_name")
    {
      m_ally_name = value;
      handled = true;
      m_knows_ally = true;
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
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SwimSearch::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_input_key, 0);
  Register(m_local_node_report_key, 0);
  Register(m_node_report_key, 0);
  Register(m_ally_name_key, 0);
  Register(m_found_key, 0);
  Register(m_rescue_region_key, 0);
  Register(m_scouted_swimmer_key, 0);
  Register(m_ctrl_authority_rescue_key, 0);
  Register(m_antagonize_key, 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool SwimSearch::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one"
        << "two"
        << "three"
        << "four";
  m_msgs << actab.getFormattedString();

  return (true);
}
