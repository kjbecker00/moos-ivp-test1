#pragma once

#include <string>
#include <cmath>

#define SCF static const double
#define SCI static const int

namespace Constants {
    SCF pi = 3.14159265359;
}


class PointScore {
    private:

    protected:

    public:
    uint8_t m_group_size = 1; //multiplied directly, the number of points that this point represents
    double m_density_elevation = 0.00; //summed and then multiplied
    double m_total_score = 1; //multiplied directly
    uint8_t m_greedy_idx = 0; //simple integer used as vector accessor for the order we would have gone in if we did a greedy search
    double m_predicted_opponent_penalty = 1; //default, no penalty, multiplied directly
    uint8_t m_predicted_opponent_idx = 0; //simple integer used as vector accessor, the order in which the opponent would have grabbed this if they did a greedy search
    double m_proximity_score = 0; //Assigned later, between 1 and 0
    
    void update_total_score() {
        //Density elevation: Represents the point density w/rt to all the other points in the grid - square rooted so it doesn't weigh as much
        //Group side: Not used right not, but would directly represent the number of points that this point represents
        //Predicted opponent penalty: Loosely represents the probability in which our enemy will capture it before us
        //Proximity score: min/max normalized w/rt to how close this point is compared to all others (closest point = 1, furthest point > 0) - squared to be heavily penalized
        m_total_score=pow(m_density_elevation, 1)*m_group_size*pow(m_predicted_opponent_penalty, 0.5)*pow(m_proximity_score,2.0);
    }

    void apply_opponent_penalty(double gamma) {
        m_predicted_opponent_penalty = 1-pow(gamma,m_predicted_opponent_idx);
    }

    std::string repr() {
        char sbuff[256];
        memset(sbuff, 256,'\0');
        snprintf(sbuff, 256, "<Score: Total: %0.2f, O.Penalty: %0.2f, Elevation: %0.2f, Prox: %0.2f\n>", m_total_score, m_predicted_opponent_penalty, m_density_elevation, m_proximity_score);
        return std::string(sbuff);
    };
};

class Kernel {
    double cx = 0;
    double cy = 0;
    double sigma = 0;
    public:
        Kernel(double x,double y, double s) {
            cx = x;
            cy = y;
            sigma = s;
        }

        double apply_kernel(double x, double y) {
            return ((pow(sigma,3.0))/(2*Constants::pi*pow(sigma, 2.0)))*exp(-(pow((x-cx),2.0)-pow((y-cy), 2.0))/(2*pow(sigma, 2.0)));
        }
};


class PathParameters {
    private:

    protected:

    public:
        double discount_factor = 1;
        double path_weight = 1;
        double momentum_weight = 1;
};

class PathData {
    public:
        PathData() {
            m_path_length = 0;
            m_path_continuity = 0;
            m_point_score = 1;
        }

        double m_path_length = 0;
        double m_path_continuity = 0;
        double m_point_score = 1;

};

class Point {
  private:
    static const u_int16_t m_buff_size = 64;
    char sbuff[m_buff_size];

  protected:

  public:
    Point() {
      m_x = -9999.9;
      m_y = -9999.9;
      m_id = "Nobl and Greenough School";
    };

    Point(double x, double y) {
        m_x = x;
        m_y = y;
        m_id = "UNK";
    };
    Point(double x, double y, std::string id) {
        m_x = x;
        m_y = y;
        m_id = id;
    };

    //TODO: Include a constructor which takes a standard x,y,point mayble?

    //TODO: Make a method for making an 
    double m_x, m_y;
    std::string m_id;

    PointScore m_score;

    std::string repr() {
        memset(sbuff, 64,'\0');
        snprintf(sbuff, m_buff_size, "<%0.2f, %0.2f, %s>", m_x, m_y, m_id.c_str());
        return std::string(sbuff);
    };

    double dist(const Point &point) const {
        return sqrt(pow(m_x - point.m_x, 2.0) + pow(m_y- point.m_y, 2.0));
    };

    double rel_dist(const Point &point) const {
        return pow(m_x - point.m_x,2.0) + pow(m_y - point.m_y, 2.0);
    };

    bool operator==(const Point& other) const {
        return (m_x == other.m_x && m_y == other.m_y);
    };
};

class Utils {
    public:
    /**
     * @brief Calculates a discount factor where at the last point, it will be weighted with the 
     * end target provided. i.e. After 20 points we want the value of the discount factor for the last point
     * to be 0.01, therefore we use a discount factor of 0.794. With this,
     * the first penalization cost is 1, then 0.794, ...
     * 
     * @param end_target Target final discount of the last point
     * @param n_points Number of points
     * @return double gamma
     */
    static double smart_discount_factor(double end_target, uint8_t n_points) {
        return exp(log(end_target)/n_points);
    }
    static bool compare_distance_to_ref(const Point& option_A, const Point& option_B, const Point& reference_point) {
        //utility for sorting a vector from greatest to least
        double distance_A = reference_point.rel_dist(option_A);
        double distance_B = reference_point.rel_dist(option_B);
        return distance_A > distance_B;
    }

    static bool compare_scores(const Point& option_A, const Point& option_B) {
        //utility for sorting a vector from greatest to least
        return option_A.m_score.m_total_score < option_B.m_score.m_total_score;
    }

    static double get_continuity(double angle_a, double angle_b) {
        double angle_diff = angle_b - angle_a;
        double continuity = 1+cos(angle_diff);
        return continuity;
    }
};

class OrderedLookup {
    public:
        std::map<std::string, uint8_t> id_to_idx;
        std::vector<Point> sorted_points;
        std::vector<double> travel_distance;
};