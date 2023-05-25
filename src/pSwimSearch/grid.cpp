/************************************************************/
/*    NAME: Ray Turrisi                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: grid.cpp                                        */
/*    CIRC: May 15, 2023                                    */
/************************************************************/

#include "grid.h"
#include <cmath>
#include <fstream>
#include <utility>
#include <iostream>
#include <cstring>

KernelElement::KernelElement()
{
    dx = 0;
    dy = 0;
    elevation = 0;
}
KernelElement::KernelElement(int x, int y)
{
    dx = x;
    dy = y;
    elevation = 0;
}
KernelElement::KernelElement(int x, int y, uint8_t _elevation)
{
    dx = x;
    dy = y;
    elevation = _elevation;
}

// Hash function specialization for MyClass

Point::Point()
{
    m_x = -9999.9;
    m_y = -9999.9;
};

Point::Point(double x, double y)
{
    m_x = x;
    m_y = y;
};
Point::Point(int x_idx, int y_idx)
{
    m_x_idx = x_idx;
    m_y_idx = y_idx;
};
Point::Point(double x, double y, std::string id)
{
    m_x = x;
    m_y = y;
};

double m_x, m_y;
int m_x_idx, m_y_idx;

std::string Point::repr()
{
    memset(sbuff, 64, '\0');
    snprintf(sbuff, m_buff_size, "<%0.2f, %0.2f>", m_x, m_y);
    return std::string(sbuff);
};

double Point::dist(const Point &point) const
{
    return sqrt(pow(m_x - point.m_x, 2.0) + pow(m_y - point.m_y, 2.0));
};

double Point::rel_dist(const Point &point) const
{
    return pow(m_x - point.m_x, 2.0) + pow(m_y - point.m_y, 2.0);
};

bool Point::operator==(const Point &other) const
{
    return (m_x == other.m_x && m_y == other.m_y);
};

SearchGrid::SearchGrid()
{
    m_length_meters = 0;
    m_height_meters = 0;
    m_resolution_meters = 0;
    m_length_idcs = 0;
    m_height_idcs = 0;
    m_slant_rads = 0;
    m_bottom_left_pin = Point(0, 0);
    m_grid = std::vector<std::vector<uint8_t> >();
}
SearchGrid::SearchGrid(Point pin_point, double length_m, double height_m, double resolution_m, double slant_rads)
{
    m_length_meters = length_m;
    m_height_meters = height_m;
    m_resolution_meters = resolution_m;
    m_length_idcs = static_cast<int>(length_m / resolution_m);
    m_height_idcs = static_cast<int>(height_m / resolution_m);
    m_slant_rads = slant_rads;
    m_grid.resize(m_height_idcs);
    for (int i = 0; i < m_height_idcs; i++)
    {
        // Populate the grid with the maximum value
        m_grid[i].resize(m_length_idcs, 255);
    }
    m_bottom_left_pin = pin_point;
};

bool SearchGrid::is_point_in_map(double x, double y)
{
    //Translates the point and then rotates it from the map/workspace to the grid
    double tmp_x = x - m_bottom_left_pin.m_x;
    double tmp_y = y - m_bottom_left_pin.m_y;

    // rotate it the point into this new frame
    double r_x = tmp_x * cos(-m_slant_rads) - tmp_y * sin(-m_slant_rads);
    double r_y = tmp_x * sin(-m_slant_rads) + tmp_y * cos(-m_slant_rads);

    // in this grids new frame, discretize it and get the indices
    Point p;
    p.m_x = x;
    p.m_y = y;

    int x_idx = static_cast<int>((r_x / m_length_meters) * m_length_idcs);
    int y_idx = static_cast<int>((r_y / m_height_meters) * m_height_idcs);

    if ((x_idx < 0) || (x_idx >= m_length_idcs) || (y_idx < 0) || y_idx >= m_height_idcs)
    {
        return false;
    }
    return true;
}

Point SearchGrid::map_to_grid(double x, double y)
{
    /*
    Maps in the continous map domain to the discrete grid domain
    TODO: In another implementation, this function should return true or false, and take a reference to a point to update
    Where we can do error checking on whether or not a point is out of bounds
    */
    // translate the points distance w/rt to our origin
    double tmp_x = x - m_bottom_left_pin.m_x;
    double tmp_y = y - m_bottom_left_pin.m_y;

    // rotate it the point into this new frame
    double r_x = tmp_x * cos(-m_slant_rads) - tmp_y * sin(-m_slant_rads);
    double r_y = tmp_x * sin(-m_slant_rads) + tmp_y * cos(-m_slant_rads);

    // in this grids new frame, discretize it and get the indices
    Point p;
    p.m_x = x;
    p.m_y = y;

    int x_idx = static_cast<int>((r_x / m_length_meters) * m_length_idcs);

    //Clamps a point to the edge of the grid if it was out of bounds
    if (x_idx < 0)
        x_idx = 0;
    if (x_idx >= m_length_idcs)
        x_idx = m_length_idcs - 1;
    p.m_x_idx = x_idx;

    int y_idx = static_cast<int>((r_y / m_height_meters) * m_height_idcs);
    if (y_idx < 0)
        y_idx = 0;
    if (y_idx >= m_height_idcs)
        y_idx = m_height_idcs - 1;
    p.m_y_idx = y_idx;

    return p;
}

Point SearchGrid::grid_to_map(int row, int col)
{
    /*
    Maps from the discrete grid domain to the continous map domain
    */

    Point p;
    p.m_x_idx = col;
    p.m_y_idx = row;

    double l = static_cast<double>(col) * m_resolution_meters;
    double h = static_cast<double>(row) * m_resolution_meters;
    double x = m_bottom_left_pin.m_x + l * cos(m_slant_rads) - h * sin(m_slant_rads);
    double y = m_bottom_left_pin.m_y + l * sin(m_slant_rads) + h * cos(m_slant_rads);
    p.m_x = x;
    p.m_y = y;
    return p;
}

Point SearchGrid::get_centroid()
{
    //Computes the centroid of the grid
    double total_mass = 0;
    double cx = 0;
    double cy = 0;
    int cx_idx = 0;
    int cy_idx = 0;
    for (int i = 0; i < m_length_idcs; i++)
    {
        for (int j = 0; j < m_height_idcs; j++)
        {
            cx += m_grid[j][i] * i;
            cy += m_grid[j][i] * j;
            total_mass += m_grid[j][i];
        }
    }
    cx /= total_mass;
    cy /= total_mass;
    cx_idx = static_cast<int>(cx);
    cy_idx = static_cast<int>(cy);
    return grid_to_map(cy_idx, cx_idx);
}

Point SearchGrid::get_local_centroid(Point center, int dilation_radius_idcs)
{

    // Get the local centroid within a desired kernel radius

    // See if we already have a set of accessors, if we don't, cache them
    if (kernel_accessors.count(dilation_radius_idcs) == 0)
    {
        compose_discrete_kernel(dilation_radius_idcs);
    }
    std::vector<KernelElement> accessors = kernel_accessors[dilation_radius_idcs];

    double local_mass = 0;
    double cx = 0;
    double cy = 0;
    int cx_idx = 0;
    int cy_idx = 0;
    Point center_in_grid = map_to_grid(center.m_x, center.m_y);

    // The local center is calculated, and then offset later
    for (const KernelElement &ka : accessors)
    {
        int x = center_in_grid.m_x_idx + ka.dx;
        int y = center_in_grid.m_y_idx + ka.dy;
        if (x < 0 ||
            x >= m_length_idcs ||
            y < 0 ||
            y >= m_height_idcs)
        {
            continue;
        }
        local_mass += m_grid[y][x];
        cx += ka.dx * m_grid[y][x];
        cy += ka.dy * m_grid[y][x];
    }
    cx /= local_mass;
    cy /= local_mass;
    cx_idx = center_in_grid.m_x_idx + static_cast<int>(cx);
    cy_idx = center_in_grid.m_y_idx + static_cast<int>(cy);
    return grid_to_map(cy_idx, cx_idx);
}

int SearchGrid::get_value(Point p)
{
    Point p2 = map_to_grid(p.m_x, p.m_y);
    return m_grid[p2.m_y_idx][p2.m_x_idx];
}
bool SearchGrid::over_threshold(Point p, int threshold)
{
    Point p2 = map_to_grid(p.m_x, p.m_y);
    int value = m_grid[p2.m_y_idx][p2.m_x_idx];
    return value > threshold;
}

double SearchGrid::get_remaining_uncertainty()
{
    double total_probability = 0;
    for (int i = 0; i < m_length_idcs; i++)
    {
        for (int j = 0; j < m_height_idcs; j++)
        {
            total_probability += m_grid[j][i];
        }
    }
    return total_probability / (255.0 * m_length_idcs * m_height_idcs);
}

std::vector<Cluster> SearchGrid::get_clusters()
{
    /*
        Get all the clusters within the search grid
        We assume that there will be multiple segmentations, and that there
        will also be multiple unexplored clusters
        * - - - - - - - - - - - - - - - - - - - - - - *
        |             /xxxx/         \xxxxx\          |
        |            /xxxx/___________\xxxxx\         |
        |           /xxxxxxxxxxxxxxxxxxxxxxxx\        |
        |___________|xxxxxxxxxxxxxxxxxxxxxxxx|        |
        |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|        |
        |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|        |
        |xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|        |
        |````````````````````````````````\xxxx\       |
        |                                 \xxxx\      |
        * - - - - - - - - - - - - - - - - - - - - - - *

        In this grid, we would have one blob, and four clusters (three medium, one small)
        Assume the clear spaces have an intensity of 255, while the others have an intensity of 0
    */

    // Get all the unvisited points in the grid
    std::unordered_set<std::pair<int, int>, hashFunction> unvisited_points;
    for (int row = 0; row < m_grid.size(); row++)
    {
        for (int col = 0; col < m_grid[0].size(); col++)
        {
            if (m_grid[row][col] == 0)
                continue;
            unvisited_points.insert(std::make_pair(row, col));
        }
    }

    std::unordered_set<std::pair<int, int>, hashFunction> observed_points;
    

    //Define the types of neighbors we will be exploring. Currently we are only consider the adjacent neighbors (faster)
    std::vector<std::pair<int, int>> neighbor_offsets;
    neighbor_offsets.push_back(std::make_pair(0, 1));  // col+1
    neighbor_offsets.push_back(std::make_pair(0, -1)); // col-1
    neighbor_offsets.push_back(std::make_pair(1, 0));  // row+1
    neighbor_offsets.push_back(std::make_pair(-1, 0)); // row-1

#if 0
        neighbor_offsets.push_back(std::make_pair(1,1)); //col+1, row+1
        neighbor_offsets.push_back(std::make_pair(-1,1)); //row-1, col+1
        neighbor_offsets.push_back(std::make_pair(1,1)); //row+1, col+1
        neighbor_offsets.push_back(std::make_pair(-1,-1)); //row-1, col-1
#endif

    // Now that we have all the unvisited points in the grid, cluster them
    std::vector<Cluster> clusters;
    // While we have unvisited points
    while (unvisited_points.size() > 0)
    {
        // We start a cluster
        Cluster c_cluster;

        // We prepare the neighbors datastructure
        std::unordered_set<std::pair<int, int>, hashFunction> unexplored_neighbors;

        // Seed the datastructure with a 'random' point inside the unvisited points set
        std::pair<int, int> seed = *(std::next(unvisited_points.begin(), 0));

        // Move the seed from the unvisited set to the unexplored neighbors set
        unvisited_points.erase(seed);
        unexplored_neighbors.insert(seed);
        c_cluster.elements.insert(seed);

        // While we have unexplored neighbors, we branch out while populating the cluster
        while (unexplored_neighbors.size() > 0)
        {
            // Take an unexplored point from previous neighbors for which we do not know how to characterize it yet
            std::pair<int, int> unexp_point = *(std::next(unexplored_neighbors.begin(), 0));

            // By the end of this loop, this point will have provided its possible neighbors, and if it has any neighbor which 
            // is on a boundary, it is I
            unexplored_neighbors.erase(unexp_point);
            
            // For each of the neighbors we can consider, use the neighbor's condition to define this point
            std::vector<std::pair<int, int>> new_neighbors;

            for (auto n : neighbor_offsets)
            {
                // Look at where its neighbors will be
                int nxt_r = unexp_point.first + n.first;
                int nxt_c = unexp_point.second + n.second;

                // Make sure we didn't step to a point we've already been
                std::pair<int, int> next_pt(nxt_r, nxt_c);

                // If the next proposed point has already been observed or if it is already in the cluster, skip this point
                //TODO: check how necessary it is to use the c_cluster check
                if (observed_points.count(next_pt) > 0 || c_cluster.elements.count(next_pt) > 0)
                    continue;

                // Check to see if this next index set would put us out of bounds
                bool out_of_bounds = (nxt_r < 0 || nxt_c < 0 || (nxt_r >= m_height_idcs) || (nxt_c >= m_length_idcs));

                // If we are out of bounds, then this current point would be an edge point of the cluster

                bool has_explored_neighbor = false;

                // If it doesn't put us out of bounds, we can consider the neighbor to see if it has been explored
                if (!out_of_bounds)
                {
                    // If the neighbor has a value of purely zero, then it has been explored fully
                    has_explored_neighbor = (m_grid[nxt_r][nxt_c] == 0);
                }

                // If the point puts us out of bounds or the neighbor has been visited, then this is an edge point
                if (out_of_bounds || has_explored_neighbor)
                {
                    // If it meets either of these conditions, we can conclusively say it is an edge point
                    c_cluster.edges_elements.insert(std::make_pair(unexp_point.first, unexp_point.second));
                }
                else
                {
                    // If this neighbor point we just observed is not out of bounds and has not been visited yet, it means it is another point to branch off from
                    // Later, it will be included in the cluster
                    new_neighbors.push_back(next_pt);
                }
            }

            //For the neighbors we just collected, do something with them
            for (int i = new_neighbors.size() - 1; i >= 0; i--)
            {
                //We may have neighbors that haven't been visited yet, just observed
                observed_points.insert(new_neighbors[i]);

                //Each of these neighbors are now part of this cluster
                c_cluster.elements.insert(new_neighbors[i]);
                
                //These neighbors haven't been quantified yet, but they are now up for consideration in this cluster
                unexplored_neighbors.insert(new_neighbors[i]);

                //These points have been visited/observed, so remove thm from the total unvisited list since they can't be
                //apart of another cluster
                unvisited_points.erase(new_neighbors[i]);
            }
        }
        // This is unnecessary, but is here for safety
        if (c_cluster.elements.size() == 0)
            continue;

        // If we are here, it means that we exhausted all the neighbors from some seed of an initial point from the seed of the unvisited points list

        // Update the cluster to have a complete set of data (including centroid of the cluster)
        double cr = 0;
        double cc = 0;
        double mass = 0;
        for (std::pair<int, int> elem : c_cluster.elements)
        {
            mass += m_grid[elem.first][elem.second];
            cr += m_grid[elem.first][elem.second] * elem.first;
            cc += m_grid[elem.first][elem.second] * elem.second;
        }
        cr /= mass;
        cc /= mass;

        //Update the properties of this cluster
        c_cluster.centroid = std::make_pair(static_cast<int>(cr), static_cast<int>(cc));
        c_cluster.mass = static_cast<int>(mass);

        //Save this cluster, and then move onto another point which hasn't been visited yet to seed the next cluster
        clusters.push_back(c_cluster);
    }
    return clusters;
}

void SearchGrid::compose_discrete_kernel(int dilation_radius_idcs)
{
    // Generate a set of kernel accessors for a desired dilation radius
    std::unordered_set<std::string> quarter_idcs_set;
    std::vector<KernelElement> quarter_idcs;
    for (int i = 0; i < dilation_radius_idcs; i++)
    {
        for (double rds = 0; rds < M_PI / 2; rds += (M_PI / 400.0))
        {
            int x_diff = static_cast<int>(static_cast<double>(i) * cos(rds));
            int y_diff = static_cast<int>(static_cast<double>(i) * sin(rds));
            std::string key = std::to_string(x_diff) + "," + std::to_string(y_diff);
            if (quarter_idcs_set.count(key) > 0)
            {
                continue;
            }
            quarter_idcs_set.insert(key);
            quarter_idcs.push_back(KernelElement(x_diff, y_diff));
        }
    }

    // Compute a index difference lookup for quickly applying kernels
    kernel_accessors[dilation_radius_idcs] = std::vector<KernelElement>(4 * quarter_idcs.size());
    for (KernelElement diff_idx : quarter_idcs)
    {
        kernel_accessors[dilation_radius_idcs].push_back(diff_idx);
        kernel_accessors[dilation_radius_idcs].push_back(KernelElement(-diff_idx.dx, -diff_idx.dy));
        if ((diff_idx.dx == 0) || (diff_idx.dy == 0))
            continue;
        kernel_accessors[dilation_radius_idcs].push_back(KernelElement(-diff_idx.dx, diff_idx.dy));
        kernel_accessors[dilation_radius_idcs].push_back(KernelElement(diff_idx.dx, -diff_idx.dy));
    }
}

void SearchGrid::compose_discrete_kernel_w_weights(int dilation_radius_idcs, uint8_t elevation)
{

    // Generate a kernel with a particular affect to apply
    std::pair<int, int> key(dilation_radius_idcs, elevation);

    // If we don't have the set of accessors yet, generate them
    if (kernel_accessors.count(dilation_radius_idcs) == 0)
    {
        compose_discrete_kernel(dilation_radius_idcs);
    }

    // Copy the accessor set
    kernel_accessors_w_weight[key] = kernel_accessors[dilation_radius_idcs];

    // Apply the desired elevation effect
    for (KernelElement &ke : kernel_accessors_w_weight[key])
    {
        ke.elevation = elevation;
    }
}

void SearchGrid::reduce_search_space(double x, double y, double dilation_m, uint8_t elevation)
{
    Point p = map_to_grid(x, y);
    uint8_t dilation_radius_idcs = static_cast<int>(dilation_m / m_resolution_meters);
    m_grid[p.m_y_idx][p.m_x_idx] = 0;
    // If this dilation has not been used yet, generate a lookup for it
    std::pair<int, int> unique_rep(dilation_radius_idcs, elevation);
    if (kernel_accessors_w_weight.count(unique_rep) == 0)
    {
        compose_discrete_kernel_w_weights(dilation_radius_idcs, elevation);
    }
    // Now apply the kernel to the discrete grid
    int accessor_idx = 0;
    for (KernelElement idx_diff : kernel_accessors_w_weight[unique_rep])
    {
        // Make sure that the accessor is in bounds
        if ((p.m_x_idx + idx_diff.dx >= m_length_idcs) ||
            (p.m_x_idx + idx_diff.dx < 0) ||
            (p.m_y_idx + idx_diff.dy >= m_height_idcs) ||
            (p.m_y_idx + idx_diff.dy < 0))
        {
            continue;
        }
        else
        {
            //  If it is in bounds, we dilate around the swimmer
            int c_cell = m_grid[p.m_y_idx + idx_diff.dy][p.m_x_idx + idx_diff.dx];
            if (c_cell - idx_diff.elevation < 0)
            {
                m_grid[p.m_y_idx + idx_diff.dy][p.m_x_idx + idx_diff.dx] = 0;
            }
            else
            {
                m_grid[p.m_y_idx + idx_diff.dy][p.m_x_idx + idx_diff.dx] -= idx_diff.elevation;
            }
        }
    }
}

double SearchGrid::get_kernel_gradient(std::pair<int, int> point_by_idx, int dilation_radius)
{
    /*
        TODO: Generally need to make the use of classes more consistent, including replacing pairs with points
        Calculate the directional gradient, returning the angle in radians in the world frame
    */

    // If this dilation has not been used yet, generate a lookup for it
    Point p(point_by_idx.first, point_by_idx.second);

    if (kernel_accessors.count(dilation_radius) == 0)
    {
        compose_discrete_kernel(dilation_radius);
    }

    // Now apply the kernel to the discrete grid
    int accessor_idx = 0;
    double cx_certainty = 0;
    double cy_certainty = 0;
    double certainty = 0;
    double cx_uncertainty = 0;
    double cy_uncertainty = 0;
    double uncertainty = 0;
    for (KernelElement idx_diff : kernel_accessors[dilation_radius])
    {
        int nxt_x = p.m_x_idx + idx_diff.dx;
        int nxt_y = p.m_y_idx + idx_diff.dy;
        // If we go out of bounds, this is complementary to certainty (we know it isn't there)
        if ((nxt_x >= m_length_idcs) ||
            (nxt_x + idx_diff.dx < 0) ||
            (nxt_y + idx_diff.dy >= m_height_idcs) ||
            (nxt_y + idx_diff.dy < 0))
        {
            // Weights the center of certainty towards this region
            certainty += 255;
            cx_certainty = 255 * nxt_x;
            cy_certainty = 255 * nxt_y;
        }
        else
        {
            // Otherwise, we are in bound, and going to weight them by their inverses
            certainty += (255 - m_grid[nxt_y][nxt_x]);
            cx_certainty += (255 - m_grid[nxt_y][nxt_x]) * nxt_x;
            cy_certainty += (255 - m_grid[nxt_y][nxt_x]) * nxt_y;

            uncertainty += m_grid[nxt_y][nxt_x];
            cx_uncertainty += m_grid[nxt_y][nxt_x] * nxt_x;
            cy_uncertainty += m_grid[nxt_y][nxt_x] * nxt_y;
        }
    }
    // Calculate the centers of local certainty and uncertainty
    cx_certainty /= certainty;
    cy_certainty /= certainty;
    cx_uncertainty /= uncertainty;
    cy_uncertainty /= uncertainty;

    // Calculate the angle from certainty to uncertainty
    double angle_towards_uncertainty = atan2(cy_uncertainty - cy_certainty, cx_uncertainty - cx_certainty);
    return angle_towards_uncertainty;
}

void SearchGrid::save_grid(std::string fname)
{
    std::ofstream file(fname.c_str(), std::ios::binary);

    if (file)
    {
        // Write PGM header
        file << "P5\n"
             << m_length_idcs << " " << m_height_idcs << "\n255\n";

        // for (const std::vector<uint8_t> &row : m_grid)
        for (auto row = m_grid.rbegin(); row != m_grid.rend(); ++row)
        {
            file.write(reinterpret_cast<const char *>((row->data())), sizeof(char) * row->size());
        }

        // Write image data
        file.close();
    }
    else
    {
        std::cerr << "Failed to open file: " << fname << std::endl;
    }
}

std::string SearchGrid::repr()
{
    char sbuff[256];
    memset(sbuff, 256, '\0');
    snprintf(sbuff, 256, "Search Grid: \
        \n\t- Length: %0.2f (%d idcs)\
        \n\t- Height: %0.2f (%d idcs)\
        \n\t- Resolution: %0.2f\
        \n\t- Slant: %0.1f\
        \n\t- Pin Point: %s\n\t",
             m_length_meters, m_length_idcs, m_height_meters, m_height_idcs, m_resolution_meters, m_slant_rads * 180.0 / M_PI, m_bottom_left_pin.repr().c_str());
    return std::string(sbuff);
};

bool Utils::compare_score_A(const Point &A, const Point &B, const Point &centroid, const Point &reference_point, const double &reference_angle)
{
    //A crude method for composing a fast path which has a bias of maintaining forward momentum and being close to the centroid of uncertainty
    double distance_A = reference_point.rel_dist(A);
    double distance_B = reference_point.rel_dist(B);

    double dscale = distance_A + distance_B;
    distance_A /= dscale;
    distance_B /= dscale;
    double a_norm = 1 - distance_A;
    double b_norm = 1 - distance_B;

    double distance_A_C = A.dist(centroid);
    double distance_B_C = B.dist(centroid);
    double cscale = distance_A_C + distance_B_C;
    distance_A_C /= cscale;
    distance_B_C /= cscale;
    double ac_norm = 1 - distance_A_C;
    double bc_norm = 1 - distance_B_C;
    double angle_to_A = atan2(A.m_y - reference_point.m_y, A.m_x - reference_point.m_x);
    double angle_to_B = atan2(B.m_y - reference_point.m_y, B.m_x - reference_point.m_x);
    double norm_cont_A = pow(1 + cos(angle_to_A - reference_angle), 3);
    double norm_cont_B = pow(1 + cos(angle_to_B - reference_angle), 3);
    return norm_cont_A * ac_norm * a_norm > norm_cont_B * bc_norm * b_norm;
}

bool Utils::compare_distance_to_ref(const Point &option_A, const Point &option_B, const Point &reference_point)
{
    // utility for sorting a vector from greatest to least
    double distance_A = reference_point.rel_dist(option_A);
    double distance_B = reference_point.rel_dist(option_B);
    return distance_A > distance_B;
}

bool Utils::compare_by_mass(const Cluster &A, const Cluster &B)
{
    return A.mass < B.mass;
}
