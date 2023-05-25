/************************************************************/
/*    NAME: Ray Turrisi                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: grid.h                                          */
/*    CIRC: May 15, 2023                                    */
/************************************************************/

#pragma once
#include <vector>
#include <string>
#include <utility>
#include <unordered_set>
#include <map>


class KernelElement
{
    /**
     * @brief A kernel element maintains the offset indices and an associated 
     * elevation value if there is one, specifically for applying sum or subtracted 
     * differences over an existing grid.
     */
public:
    int dx = 0;
    int dy = 0;
    uint8_t elevation = 0;

    /**
     * @brief Construct a new Kernel Element object
     * 
     */
    KernelElement();

    /**
     * @brief Construct a new Kernel Element object specifying the dX and dY from a center point
     * 
     * @param x difference in x
     * @param y difference in y
     */
    KernelElement(int x, int y);

    /**
     * @brief Construct a new Kernel Element object specifying the dX and dY from a center point
     * 
     * @param x difference in x
     * @param y difference in y
     * @param _elevation an assigned elevation for this difference in x and y from a center point
     */
    KernelElement(int x, int y, uint8_t _elevation);
};

class Point
{
    /**
     * @brief A simple point class which contains the location in meters and its 
     * associated index in the grid. Includes additional utilities for calculating
     * the distance to another point.
     */
private:
    static const u_int16_t m_buff_size = 64;
    char sbuff[m_buff_size];

protected:
public:
    Point();

    /**
     * @brief Construct a new Point object
     * 
     * @param x meters in the workspace
     * @param y meters in the workspace
     */
    Point(double x, double y);

    /**
     * @brief Construct a new Point object referencing indexes in the grid, not the workspace
     * 
     * @param x_idx assumes index in the grid
     * @param y_idx assumes index in the search grid
     */
    Point(int x_idx, int y_idx);

    /**
     * @brief Construct a new Point object with a label
     * 
     * @param x 
     * @param y 
     * @param id 
     */
    Point(double x, double y, std::string id);

    double m_x, m_y;
    int m_x_idx, m_y_idx;

    /**
     * @brief Obtain the string representation of the point in the workspace
     * 
     * @return std::string 
     */
    std::string repr();

    /**
     * @brief Computes the distance between this point and another point
     * 
     * @param point Another point object which is used within the context of the workspace
     * @return double 
     */
    double dist(const Point &point) const;

    /**
     * @brief Relative distance (does not square root) - for performant applications with many instances use this
     * 
     * @param point 
     * @return double 
     */
    double rel_dist(const Point &point) const;

    bool operator==(const Point &other) const;
};

/**
 * @brief Generic hash function for an std::pair of two integers, required for an unordered set
 * 
 */
struct hashFunction
{
  size_t operator()(const std::pair<int , 
                    int> &x) const
  {
    return x.first ^ x.second;
  }
};

/**
 * @brief A cluster can take an arbitrary shape, and is represented by a hashed unordered set. This
 * is purely a data-transfer object
 * 
 */
class Cluster
{
public:

    /**
     * @brief The total elements in the cluster
     * 
     */
    std::unordered_set<std::pair<int,int>, hashFunction > elements;
    /**
     * @brief Exclusively the edge elements in the cluster
     * 
     */
    std::unordered_set<std::pair<int,int>, hashFunction > edges_elements;

    /**
     * @brief The centroid of the cluster
     * 
     */
    std::pair<int,int> centroid;

    /**
     * @brief The total "mass" of the cluster - the sum of all elements which have values between 0 and 255 individually
     * 
     */
    int mass;
};


/**
 * @brief This search grid class has utilities for taking a polygon (currently exclusively a rectangle),
 * which can cache kernel operators, apply kernels across the grid, extract disjoint clusters in the grid,
 * and support methods which map between the workspace and the discretized representation
 * 
 */
class SearchGrid
{
private:
protected:
public:

    double m_length_meters;
    double m_height_meters;
    double m_resolution_meters;
    int m_length_idcs;
    int m_height_idcs;

    //Transformation parameters
    double m_slant_rads; //The rotation angle
    Point m_bottom_left_pin; //The point in the workspace which all other points are defined w/rt to

    // Remember that by convention, we access discrete elements by row and then column, which is geometrically opposite of our x/y notation
    std::vector<std::vector<uint8_t> > m_grid;
    

    int prev_dilation_radius;

    //Cached kernel accessors with no elevation
    std::map<int, std::vector<KernelElement> > kernel_accessors;

    //Cached kernel accessors with a specific elevation applied to the kernel (i.e. the elevation may have a seeding function such)
    //as a gaussian distribution
    std::map<std::pair<int, int>, std::vector<KernelElement> > kernel_accessors_w_weight;

    SearchGrid();
    /**
     * @brief Construct a new Search Grid object - the correct constructor to define a search grid object
     * 
     * @param pin_point The reference point in which workspace parameters are translated by and rotated about
     * @param length_m The length of the workspace in meters
     * @param height_m The height of the workspace in meters
     * @param resolution_m The resolution of the grid in meters
     * @param slant_rads The slant of the workspace which rotates the workspace to a rectangle with zero rotation in a cartesian plane
     */
    SearchGrid(Point pin_point, double length_m, double height_m, double resolution_m, double slant_rads);

    /**
     * @brief Checks if a point is in a map
     * 
     * @param x coordinate in meters
     * @param y coordinate in meters
     * @return true Is in the workspace
     * @return false Is not in the workspace
     */
    bool is_point_in_map(double x, double y);

    /**
     * @brief Given an x,y coordinate in meters, returns a complete point object with the coordinates in meters
     * and the corresponding index. If a point is outside the workspace, places it to the nearest edge in the 
     * search grid - TODO: will change this in the future to return true or false, and update a reference
     * 
     * @param x 
     * @param y 
     * @return Point 
     */
    Point map_to_grid(double x, double y);

    /**
     * @brief Returns row/col index pair to a complete point coordinate in the workspace
     * 
     * @param row 
     * @param col 
     * @return Point 
     */
    Point grid_to_map(int row, int col);

    /**
     * @brief Returns the centroid point of the entire workspace
     * 
     * @return Point 
     */
    Point get_centroid();

    /**
     * @brief Given a dilation/kernel radius, and a center point, will return the local centroid
     * 
     * @param center 
     * @param dilation_radius_idcs The radius in which you want to inspect about a center point
     * @return Point 
     */
    Point get_local_centroid(Point center, int dilation_radius_idcs);

    /**
     * @brief Returns the associated value for a point in the workspace in the associated grid
     * 
     * @param p 
     * @return int 
     */
    int get_value(Point p);

    /**
     * @brief Checks to see if a point is or is not over some threshold value
     * 
     * @param p 
     * @param threshold 
     * @return true 
     * @return false 
     */
    bool over_threshold(Point p, int threshold);

    /**
     * @brief Returns the remaining uncertainty of the workspace/grid between 0 and 1
     * 
     * @return double Uncertainty value between 0 and 1
     */
    double get_remaining_uncertainty();

    /**
     * @brief Returns a collection of clusters within a fragmented workspace
     * 
     * @return std::vector<Cluster> 
     */
    std::vector<Cluster> get_clusters();

    /**
     * @brief Caches a discrete kernel with a set of local accessors to be used later
     * 
     * @param dilation_radius_idcs 
     */
    void compose_discrete_kernel(int dilation_radius_idcs);

    /**
     * @brief Caches a discrete kernel with a set of local accessors and associated elevation value to be used later
     * 
     * @param dilation_radius_idcs 
     * @param elevation 
     */
    void compose_discrete_kernel_w_weights(int dilation_radius_idcs, uint8_t elevation);

    /**
     * @brief Applies a kernel to some center point, with some requested dilation in meters, with a desired elevation change
     * 
     * @param x 
     * @param y 
     * @param dilation_m Kernel radius in meters
     * @param elevation Elevation change to apply to search grid between 0 and 255
     */
    void reduce_search_space(double x, double y, double dilation_m, uint8_t elevation);

    /**
     * @brief Computes the gradient angle of a local kernel
     * 
     * @param point_by_idx Exclusively takes a point defined as a pair by index in row/column order
     * @param dilation_radius Kernel radius by number of indices
     * @return double - angle in radians
     */
    double get_kernel_gradient(std::pair<int,int> point_by_idx, int dilation_radius);

    /**
     * @brief Saves the current grid to a pgm file
     * 
     * @param fname The filename to write the pgm file to - must include *.pgm
     */
    void save_grid(std::string fname);

    /**
     * @brief Returns meta data describing the current search grid - used for debugging
     * 
     * @return std::string 
     */
    std::string repr();

};

class Utils
{
public:
    /**
     * @brief A utility function for sorting, which compares two scores of a point
     * This specific function considers a value of a point as defined by its distance 
     * to a centroid, a reference point (source leading to two comparable points), and 
     * some initial heading
     * 
     * NOTE: This is no longer used
     * 
     * @param A 
     * @param B 
     * @param centroid 
     * @param reference_point 
     * @param reference_angle 
     * @return true 
     * @return false 
     */
    static bool compare_score_A(const Point &A, const Point &B, const Point &centroid, const Point &reference_point, const double &reference_angle);

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
     * @brief Used for sorting clusters by their mass, with the largest mass in the back
     * 
     * @param A 
     * @param B 
     * @return true 
     * @return false 
     */
    static bool compare_by_mass(const Cluster &A, const Cluster &B);
};