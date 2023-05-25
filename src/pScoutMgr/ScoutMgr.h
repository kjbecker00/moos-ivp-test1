/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ScoutMgr.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ScoutMgr_HEADER
#define ScoutMgr_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "Point.hpp"
#include <iostream>
#include <cstdarg> //va_list, va_start, va_end
#include <map>

class ScoutMgr : public AppCastingMOOSApp
{
 public:
   ScoutMgr();
   ~ScoutMgr();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();


    // Handling incoming mail
    void handleNewPoint(std::string point_string);
    void handleNewNodeReport(std::string sval);
    void handleNewEnemyNodeReport(std::string sval);
    void handleFoundSwimmer(std::string report);


    void gen_target_points(); // where the magic happens
    OrderedLookup study_greedy_bhv(Point point, const std::vector<Point> &swimmers);

    // Helper functions
    void publish_all_points();
    // void updateMissedPoints(); // runs on every local node report
    void sort_all_points_list(); // sorts the all_points vector. Not in use.
    void clear_points(); // a way of wiping everything. Not in use.

    void handleOpRegion(std::string sval);
    void calculateTargetBearing(double x, double y, double hdg);

    void postViewPoint(double x, double y, std::string label, std::string color);

    int get_closest_point_to(double x, double y, const std::vector<Point> &points);
    std::vector<Point> sortList(double x, double y, std::vector<Point> unsorted_list);
    void publish_list(const std::vector <Point> &list, std::string out_key);

    bool idInList(const std::vector<Point> &list, std::string id);
    bool dbg_print(const char* format, ...);


 private: // Configuration variables

 private: // State variables

    // tracks the swimmers
    std::vector<Point> m_all_points;
    std::string m_region_str;
    std::string m_output_msg;
    std::string m_last_output_msg;
    std::vector<Point> m_unvisited_points;
    std::vector<Point> m_visited_points;
    std::vector<std::string> m_enemy_names;
    std::map<std::string, NodeRecord> m_enemy_node_reports;

    // sorted list of points fed to waypt bhv
    std::vector<Point> m_target_points;
    
    // vehicle configuration
    std::string m_vname;
    std::string m_ally_name;
    std::string m_team;
    std::string m_mode;

    double m_hdg_offset;
    double m_target_bearing;

    double m_center_x;
    double m_center_y;

    // used to turn on/off the bully flag
    double m_release_ratio;
    bool m_antagonize;
    std::string m_antagonize_key;

    // stores the name of each MOOSDB var name
    std::string m_input_key;
    std::string m_found_key;
    std::string m_local_node_report_key;
    std::string m_node_report_key;
    std::string m_output_key;
    std::string m_startup_key;
    std::string m_ally_name_key;
    std::string m_replan_key;
    
    // vehicle position
    double m_nav_x = 0;
    double m_nav_y = 0;
    double m_nav_heading = 0;

    double m_turning_radius;

    double m_visit_radius;
    bool m_publish_points;
    bool m_verbose;
    bool m_debug;
    FILE* m_debug_stream;
    static const uint16_t m_fname_buff_size = 128;
    char m_fname[m_fname_buff_size];
    double epsilon = 0.5;

    uint64_t m_iterations_since_new_point = 0;
    bool m_first_enemy_node_report = false;

    std::vector<Point> m_best_path;
    PathData m_best_path_data;
    double m_best_path_score;
    double m_search_start_time;
    double m_level_discount;
    uint64_t m_max_num_solutions;
    uint_least64_t m_previous_searches;
    double m_last_replan_time;
    double m_replan_interval;
    bool m_knows_ally;
};

#endif 
