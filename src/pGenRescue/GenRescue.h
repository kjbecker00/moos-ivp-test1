/************************************************************/
/*    NAME: Kevin Becker, Ray Turrisi                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.h                                     */
/*    CIRC: Spring 2023                                     */
/************************************************************/

#ifndef GenRescue_HEADER
#define GenRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "gr_utils.h"
#include "Point_Filter.hpp"
#include <iostream>
#include <cstdarg> //va_list, va_start, va_end
#include <map>
#include <set> 

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue();

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

    /**
     * @brief For a starting point and a collection of swimmers, analyze some 
     * high level information about the path
     * 
     * @param point Starting point
     * @param swimmers Collection of swimmers
     * @return OrderedLookup Returns the sorted points, a mapping from an id 
     * to the order they are visited, and the distance traveled to achieve each point in the proposed
     * greedy order
     */
    OrderedLookup study_greedy_bhv(Point point, const std::vector<Point> &swimmers);

    /**
     * @brief Would have clustered swimmers that were really close together - not necessary or used
     * 
     * @param swimmers 
     * @return std::vector<Point> 
     */
    std::vector<Point> cluster_special_swimmer(std::vector<Point> swimmers);

    // Helper functions
    void publish_all_points();
    void updateMissedPoints(); // runs on every local node report
    void sort_all_points_list(); // sorts the all_points vector. Not in use.
    void clear_points(); // a way of wiping everything. Not in use.

    void postViewPoint(double x, double y, std::string label, std::string color);

    int get_closest_point_to(double x, double y, const std::vector<Point> &points);
    std::vector<Point> sortList(double x, double y, std::vector<Point> unsorted_list);

    /**
     * @brief A computationally efficient depth-first-search heuristic search
     * See the *.cpp file for detailed walk through of steps
     * 
     * At a high level, from a starting point, determines a suboptimal path based on a policy
     * This policy considers: 
     *  - Our opponents search order
     *  - The path length
     *  - The spatial density of points
     * We explore the top three candidates from each point, and explore their children top candidates 
     * until a complete path is formed. The policy is bounded between 0 and 1 to enable 
     * effective pruning for considering these normalized scores. i.e. we can guarantee 
     * that the product is **always** decreasing, which is powerful for pruning since it is always maximally 
     * bounded towards the target we are optimizing for. If at any point, a path is less than the best path, 
     * it is guaranteed to be worse for all its descendents and can be definitively pruned.
     * 
     * This function seeds the recursive part
     * 
     * @param x 
     * @param y 
     * @param swimmers 
     * @param enemy_node_reports 
     * @return std::vector<Point> A collection of ordered target points
     */
    std::vector<Point> dfs_heuristic_search(double x, double y, 
                                            std::vector<Point> swimmers, 
                                            std::map<std::string, NodeRecord> &enemy_node_reports);

    /**
     * @brief This is the recursive part which explores all the top paths and does the pruning
     * 
     * @param path 
     * @param remaining_capture_points 
     * @param path_data 
     */
    void dfs_heuristic_recurse(std::vector<Point> path, std::vector<Point> remaining_capture_points, PathData path_data);

    void publish_list(const std::vector <Point> &list, std::string out_key);

    bool idInList(const std::vector<Point> &list, std::string id);

    /**
     * @brief Debug utility which is an alternative to printf, and writes to any file buffer
     * which can be specified at runtime. i.e. a parameter can determine if it is writing 
     * to a file, std::out, std::err, or at all
     * 
     * @param format 
     * @param ... 
     * @return true 
     * @return false 
     */
    bool dbg_print(const char* format, ...);


 private: // Configuration variables

 private: // State variables

    // tracks the swimmers
    std::vector<Point> m_all_points;
    std::vector<Point> m_unvisited_points;
    std::vector<Point> m_visited_points;
    std::vector<std::string> m_enemy_names;
    std::set<std::string> m_known_swimmers;
    std::map<std::string, NodeRecord> m_enemy_node_reports;

    // sorted list of points fed to waypt bhv
    std::vector<Point> m_target_points;
    
    // vehicle configuration
    std::string m_vname;
    std::string m_ally_name;
    std::string m_team;
    std::string m_mode;

    // stores the name of each MOOSDB var name
    std::string m_input_key;
    std::string m_found_key;
    std::string m_local_node_report_key;
    std::string m_node_report_key;
    std::string m_output_key;
    std::string m_startup_key;
    std::string m_ally_name_key;
    std::string m_replan_key;
    std::string m_ctrl_authority_key;

    Point_Filter m_point_filter = Point_Filter();
    
    // vehicle position
    double m_nav_x = 0;
    double m_nav_y = 0;
    double m_nav_heading = 0;

    double m_turning_radius;

    double m_visit_radius;
    bool m_publish_points;
    bool m_verbose;

    //For the debug utility tools
    bool m_debug;
    FILE* m_debug_stream;
    static const uint16_t m_fname_buff_size = 128;
    char m_fname[m_fname_buff_size];

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
    bool m_ctrling_self;
    
};

#endif 
