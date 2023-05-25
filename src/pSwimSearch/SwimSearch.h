/************************************************************/
/*    NAME: Ray Turrisi                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SwimSearch.cpp                                  */
/*    CIRC: May 15, 2023                                    */
/************************************************************/

#ifndef SwimSearch_HEADER
#define SwimSearch_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <cstdarg> //va_list, va_start, va_end
#include "grid.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include <set>

class SwimSearch : public AppCastingMOOSApp
{
 public:
   SwimSearch();
   ~SwimSearch();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool dbg_print(const char* format, ...);
   void handleNewPoint(std::string point_str);
   void handleFoundSwimmer(std::string swimmer_str);
   void handleNewNodeReport(std::string node_report_str);
   void handleOtherNodeReport(std::string node_report_str);
   void sendMessage(std::string dest, std::string var_name, std::string msg_contents);
   
   std::vector<Point> find_fast_path(int depth, const Point &centroid, std::vector<Point> point_set, std::vector<Point> path, double prev_heading);
   std::vector<Point> find_greedy_path(std::vector<Point> points, std::vector<Point> path);
   //std::vector<Point> get_receding_path(int dilation_radius, int visit_radius); //TODO: This will be the interesting one
   std::vector<Point> get_path_between_centroids(Point reference_point, double visit_radius, bool greedy);

 private: // Configuration variables

 private: // State variables

  
  // tracks the swimmers
  std::vector<Point> m_all_points;
  std::vector<std::string> m_enemy_names;
  std::map<std::string, NodeRecord> m_enemy_node_reports;

  std::string m_input_key;
  std::string m_output_key;
  std::string m_vname;
  std::string m_scouted_swimmer_key;
  std::string m_ally_name;
  std::string m_found_key;
  std::string m_rescue_region_key;
  std::string m_local_node_report_key;
  std::string m_node_report_key;
  std::string m_ally_name_key;
  std::string m_antagonize_key;
  std::string m_ctrl_authority_rescue_key;
  std::vector<Point> m_sample_points;
  std::set<std::string> m_known_swimmers;
  std::vector<Cluster> m_latest_search_clusters;

  double m_rescue_rng_min;
  double m_rescue_rng_max;

  bool m_grid_initialized;
  bool m_planning_for_rescue;
  // vehicle position
  double m_nav_x;
  double m_nav_y;
  
  Point m_current_point;
  Point m_ally_point;
  double m_ally_heading;
  double m_nav_heading;
  double m_last_replan_time;
  double m_replan_interval = 20;
  bool m_replan;
  bool m_antagonizing;
  bool m_debug;
  FILE* m_debug_stream;
  static const uint16_t m_fname_buff_size = 128;
  char m_fname[m_fname_buff_size];
  bool m_knows_ally;
  XYPolygon m_rescue_region;
  SearchGrid m_search_grid;
  double m_search_grid_stp_size;

};

#endif 
