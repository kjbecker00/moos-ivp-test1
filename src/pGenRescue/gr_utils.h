#pragma once

#include <string>
#include <map> 
#include <vector> 

#define SCF static const double
#define SCI static const int

/**
 * @brief Collection of constants referenced throughout the programs which rely on the context
 * of this file
 * 
 */
namespace Constants {
    SCF pi = 3.14159265359;
}


/**
 * @brief A data transfer object, which bundles data related to the weight of an individual point
 * 
 */
class PointScore {
    private:

    protected:

    public:
    uint8_t m_group_size;
    double m_density_elevation;
    double m_total_score;
    uint8_t m_greedy_idx;
    double m_predicted_opponent_penalty;
    uint8_t m_predicted_opponent_idx;
    double m_proximity_score;

    PointScore();

    void update_total_score();

    void apply_opponent_penalty(double gamma);

    std::string repr();
};

/**
 * @brief This class is essentially a function with unique data, which we cache to apply kernels to 
 * points in a continuous space
 * 
 */
class Kernel {
    double cx = 0;
    double cy = 0;
    double sigma = 0;
    public:
        Kernel();


        /**
         * @brief Instantiate a gaussian kernel function
         * 
         * @param x 
         * @param y 
         * @param s sigma spread for the distribution - heavily scaled
         */
        Kernel(double x, double y, double s);
        
        /**
         * @brief Gets the kernel value for an x/y position w/rt to where this kernel is defined
         * 
         * @param x 
         * @param y 
         * @return double 
         */
        double apply_kernel(double x, double y);
};


/**
 * @brief A data transfer object for defining the parameters for a path
 * 
 */
class PathParameters {
    private:

    protected:

    public:
        PathParameters();
        double discount_factor;
        double path_weight;
        double momentum_weight;
};

/**
 * @brief A data transfer object which accrues different properties for a path
 * 
 */
class PathData {
    public:
        PathData();

        double m_path_length;
        double m_path_continuity;
        double m_point_score;

};

/**
 * @brief A point object with utilities for using them and representing them
 * 
 */
class Point {
  private:
    static const u_int16_t m_buff_size = 64;
    char sbuff[m_buff_size];

  protected:

  public:

    Point();

    /**
     * @brief Construct a new Point object with an x/y position
     * 
     * @param x 
     * @param y 
     */
    Point(double x, double y);

    /**
     * @brief Construct a new Point object with an x/y position and an id
     * 
     * @param x 
     * @param y 
     * @param id 
     */
    Point(double x, double y, std::string id);

    double m_x, m_y;
    std::string m_id;
    PointScore m_score;

    /**
     * @brief Get the string representation of the object
     * 
     * @return std::string 
     */
    std::string repr();

    /**
     * @brief Calculate the euclidian distance from this point to another
     * 
     * @param point 
     * @return double 
     */
    double dist(const Point &point) const;

    /**
     * @brief Calculate the relative euclidian distance from this point to another
     * 
     * @param point 
     * @return double 
     */
    double rel_dist(const Point &point) const;

    bool operator==(const Point &other) const
    {
        return (m_x == other.m_x && m_y == other.m_y);
    };
};

/**
 * @brief Collection of static utility functions which don't have a great home
 * 
 */
class Utils {
    public:

    /**
     * @brief Obtain a discount factor for a desired target across a set of points
     * 
     * @param end_target Target discount factor
     * @param n_points The number of points which this discount factor should decay across
     * @return double 
     */
    static double smart_discount_factor(double end_target, uint8_t n_points);

    /**
     * @brief Used for sorting a collection of points by their distance with respect to some reference point
     * Best point in the back
     * 
     * @param option_A 
     * @param option_B 
     * @param reference_point 
     * @return true 
     * @return false 
     */
    static bool compare_distance_to_ref(const Point& option_A, const Point& option_B, const Point& reference_point);

    /**
     * @brief Utility function for sorting/comparing the scores of a collection of points
     * 
     * @param option_A 
     * @param option_B 
     * @return true 
     * @return false 
     */
    static bool compare_scores(const Point& option_A, const Point& option_B);

    /**
     * @brief Not used, but compute the relative continuity-ness between a leading and exiting angle into a point
     * 
     * @param angle_a Leading angle
     * @param angle_b Exiting angle
     * @return double - Continuity between 0 and 1
     */
    static double get_continuity(double angle_a, double angle_b);
};

/**
 * @brief A bundled instance for a collection of points with a variety of accessors
 * for an analysis for the same context
 * 
 * This is specifically used for cross referencing two greedy paths, where for
 * some ordered set of points, there is an accumulated distance, an order for a point
 * by its id, where the id points to the index it can be found, and then the distance
 * either vehicle would have traveled by the time it reached a point can be accessed directly
 * 
 */
class OrderedLookup {
    public:
        OrderedLookup();
        /**
         * @brief Links a point ID to the index it would have been visited
         * 
         */
        std::map<std::string, uint8_t> id_to_idx;
        /**
         * @brief The points with their id in the order in which they would be visited
         * 
         */
        std::vector<Point> sorted_points;
        /**
         * @brief Monotonically increasing travel distance with index
         * associativity for the order in which each point would be visited 
         * 
         */
        std::vector<double> travel_distance;
};