#pragma once

#include <string>
#include <cmath>
#include "gr_utils.h"
#include <stdio.h>

class Point_Filter{

  public:

    Point_Filter(){};
    ~Point_Filter(){};

    // Algorithm 1 combines nearby points
    //   1) If they have overlapping capture radii, the points are combined into one
    std::vector<Point> combine_nearby_points(double nav_x, double nav_y, double combine_radius, std::vector<Point>& sorted)
    {

        if (sorted.size() < 2){
            std::cout << "Not enough points to combine nearby points" << std::endl;
            return sorted;
        }
        std::vector<Point> output;
        Point prev = Point(nav_x, nav_y);
        bool append_last;
        for (int i = 0; i < sorted.size()-1; i++){
            Point current = sorted[i];
            Point next = sorted[i+1];
            Point add_as_current;
            double dist = current.dist(next);
            append_last = false;
            int last_index = sorted.size()-1;

            // if the points are too close together (with some tolerence), it can replace the two points with one in between
            double tol_factor = 1.0;
            if (dist / 2.0 < combine_radius * tol_factor){
            // adds the midpoint and skips to the next point
            add_as_current = Point((current.m_x + next.m_x)/2, (current.m_y + next.m_y)/2);
            add_as_current.m_id = current.m_id + "&" + next.m_id;
            if (i+2 == last_index){
                append_last = true;
            }
            i ++;
            }
            // adds the current point if they aren't close together
            else{
            // if next is the last point, append the last point at the end
            if (i+1 == last_index){
                append_last = true; 
            }
            add_as_current = current;
            }
            // appends to list
            output.push_back(add_as_current);
            prev = add_as_current;
        }
        // Adds the last point
        if (append_last){
            std::cout << "Appending the last point" << std::endl;
            output.push_back(sorted[sorted.size()-1]);
        }
        else
            std::cout << "NOT appending the last point" << std::endl;
        return output;
    };


    // Adds an intermediate point to smooth the turning
    std::vector<Point> smooth_turning(double x, double y, double r, std::vector<Point>& sorted)
    {
    std::vector<Point> output;
    Point prev = Point(x, y);

    if (sorted.size() < 2){
        std::cout << "Not enough points to smooth turning" << std::endl;
        return sorted;
    }

    for (int i = 1; i < sorted.size()-1; i++){
        Point current = sorted[i];
        Point next = sorted[i+1];
        double d = current.dist(next);

        // if the points are too close together, it adds another point to improve approach angle
        if ((d < 2.0 * r) && (r > prev.dist(current))){  // - m_visit_radius)){     maybe try to cut the corner here?
            Point intermediate = circle_tangent_point(r, prev, current, next);
            if ((intermediate.m_x != current.m_x) && (intermediate.m_y != current.m_y)){
                std::cout << "Points: " << current.m_x << ", " << current.m_y << " and " << next.m_x << ", " << next.m_y << " require intermediate point. Adding " << intermediate.m_x << "," << intermediate.m_y << std::endl;
                // adds the intermidiate point before the current point
                output.push_back(intermediate);
            }
        }
        // normal function. If the points are far enough apart, it will go straight for it
        output.push_back(current);
        prev = current;
    }
    output.push_back(sorted[sorted.size()-1]);
    return output;
    };

    private:

    //------------------------------------------------------------
    // Procedure: circle_tangent_point()
    Point  circle_tangent_point(double r, Point prev, Point current, Point next){

    // previous point
    double x0 = prev.m_x;
    double y0 = prev.m_y;
    // current point
    double x1 = current.m_x;
    double y1 = current.m_y;
    // next point
    double x2 = next.m_x;
    double y2 = next.m_y;

    std::cout << "x0 = " << x0 << std::endl;
    std::cout << "y0 = " << y0 << std::endl;
    std::cout << "x1 = " << x1 << std::endl;
    std::cout << "y1 = " << y1 << std::endl;
    std::cout << "x2 = " << x2 << std::endl;
    std::cout << "y2 = " << y2 << std::endl;

    // distance from current to next
    double dnext = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    // halfway (intermediate) point
    double xi = (x2+x1) / 2.0;
    double yi = (y2+y1) / 2.0;
    // angle between current and next
    double tau = -atan((y2-y1)/(x2-x1));
    // angle between current point and center of the tangent circle
    double alpha1c = M_PI/2.0 - tau;
    double alpha2c = -1.0*(M_PI/2.0 + tau);
    // center of the tangent circle
    double x1c = x1 + r*cos(alpha1c);
    double y1c = y1 + r*sin(alpha1c);
    double x2c = x1 + r*cos(alpha2c);
    double y2c = y1 + r*sin(alpha2c);


    std::cout << "r = " << r << std::endl;
    std::cout << "dnext = " << dnext << std::endl;
    std::cout << "xi = " << xi << std::endl;
    std::cout << "yi = " << yi << std::endl;
    std::cout << "tau= " << tau << std::endl;
    std::cout << "alpha1c = " << alpha1c << std::endl;
    std::cout << "alpha2c = " << alpha2c << std::endl;
    std::cout << "x1c = " << x1c << std::endl;
    std::cout << "y1c = " << y1c << std::endl;
    std::cout << "x2c = " << x2c << std::endl;
    std::cout << "y2c = " << y2c << std::endl;

    // distance from the center of the tangent circle to the prev point
    double d0c1 = sqrt((x0-x1c)*(x0-x1c) + (y0-y1c)*(y0-y1c));
    double d0c2 = sqrt((x0-x2c)*(x0-x2c) + (y0-y2c)*(y0-y2c));

    // distance from the center of the tangent circle to the previous point
    if (r > d0c1){
        std::cout << "r21 > d0c1" << std::endl;
        return current;
    }
    if (r > d0c2){
        std::cout << "r21 > d0c1" << std::endl;
        return current;
    }
    double r21 = sqrt(pow(d0c1,2) - pow(r,2));
    double r22 = sqrt(pow(d0c2,2) - pow(r,2));

    // trig black magic
    double h = sqrt(pow(r,2) + pow((dnext/2),2));
    double a0c1 = (pow(r,2)-pow(r21,2)+pow(d0c1,2)) / (2*d0c1);
    double a0c2 = (pow(r,2)-pow(r22,2)+pow(d0c2,2)) / (2*d0c2);
    double htau1=sqrt(pow(r,2)-pow(a0c1,2));
    double htau2=sqrt(pow(r,2)-pow(a0c2,2));

    // midpoint between the two tangent points
    double xm1 = x1c + (a0c1*(x0-x1c))/d0c1;
    double ym1 = y1c + (a0c1*(y0-y1c))/d0c1;
    double xm2 = x2c + (a0c2*(x0-x2c))/d0c2;
    double ym2 = y2c + (a0c2*(y0-y2c))/d0c2;
    // tangent pt 1
    double xt11 = xm1 + htau1*(y0-y1c)/d0c1;
    double yt11 = ym1 - htau1*(x0-x1c)/d0c1;
    double xt12 = xm2 + htau2*(y0-y2c)/d0c2;
    double yt12 = ym2 - htau2*(x0-x2c)/d0c2;
    // tangent pt 2
    double xt21 = xm1 - htau1*(y0-y1c)/d0c1;
    double yt21 = ym1 + htau1*(x0-x1c)/d0c1;
    double xt22 = xm2 - htau2*(y0-y2c)/d0c2;
    double yt22 = ym2 + htau2*(x0-x2c)/d0c2;

    std::cout << "d0c1 = " << d0c1 << std::endl;
    std::cout << "r21 = " << r21 << std::endl;
    std::cout << "h = " << h << std::endl;
    std::cout << "a0c1 = " << a0c1 << std::endl;
    std::cout << "htau1 = " << htau1 << std::endl;
    std::cout << "xm1 = " << xm1 << std::endl;
    std::cout << "ym1 = " << ym1 << std::endl;

    std::cout << "\nd0c2 = " << d0c2 << std::endl;
    std::cout << "r22 = " << r22 << std::endl;
    std::cout << "h = " << h << std::endl;
    std::cout << "a0c2 = " << a0c2 << std::endl;
    std::cout << "htau2 = " << htau2 << std::endl;
    std::cout << "xm2 = " << xm2 << std::endl;
    std::cout << "ym2 = " << ym2 << std::endl;


    
    if (prev.rel_dist(Point(x1c,y1c)) > prev.rel_dist(Point(x2c,y2c))){
        // circle 2 is closer (either 12 or 22)
        std::cout << "Circle 2" << std::endl;
        std::cout << "\nxt21 = " << xt21 << std::endl;
        std::cout << "yt21 = " << yt21 << std::endl;
        std::cout << "xt12 = " << xt12 << std::endl;
        std::cout << "yt12 = " << yt12 << std::endl;
        double vectorcurrnext_x = x2 - x1;
        double vectorcurrnext_y = y2 - y1;
        double vector_t12_x = xt12 - x1;
        double vector_t12_y = yt12 - y1;
        double vector_t22_x = xt22 - x1;
        double vector_t22_y = yt22 - y1;
        std::cout << "Vector currnext: " << vectorcurrnext_x << ", " << vectorcurrnext_y << std::endl;
        std::cout << "Vector t12: " << vector_t12_x << ", " << vector_t12_y << std::endl;
        std::cout << "Vector t22: " << vector_t22_x << ", " << vector_t22_y << std::endl;
        double vector_t12_mag = sqrt(vector_t12_x*vector_t12_x + vector_t12_y*vector_t12_y);
        double vector_t22_mag = sqrt(vector_t22_x*vector_t22_x + vector_t22_y*vector_t22_y);
        double dot_t12 = (vectorcurrnext_x*vector_t12_x + vectorcurrnext_y*vector_t12_y)/vector_t12_mag;
        double dot_t22 = vectorcurrnext_x*vector_t22_x + vectorcurrnext_y*vector_t22_y/vector_t22_mag;
        std::cout << "dot_t12 = " << dot_t12 << std::endl;
        std::cout << "dot_t22 = " << dot_t22 << std::endl;
        if (dot_t12 < dot_t22)
            return Point(xt12, yt12);
        else
            return Point(xt22, yt22);
    }
    else{
        // circle 1 is closer (either 11 or 21)
        std::cout << "Circle 1" << std::endl;
        std::cout << "\nxt22 = " << xt22 << std::endl;
        std::cout << "yt22 = " << yt22 << std::endl;
        std::cout << "xt11 = " << xt11 << std::endl;
        std::cout << "yt11 = " << yt11 << std::endl;
        double vectorcurrnext_x = x2 - x1;
        double vectorcurrnext_y = y2 - y1;
        double vector_t11_x = xt11 - x1;
        double vector_t11_y = yt11 - y1;
        double vector_t21_x = xt21 - x1;
        double vector_t21_y = yt21 - y1;
        std::cout << "Vector currnext: " << vectorcurrnext_x << ", " << vectorcurrnext_y << std::endl;
        std::cout << "Vector t11: " << vector_t11_x << ", " << vector_t11_y << std::endl;
        std::cout << "Vector t21: " << vector_t21_x << ", " << vector_t21_y << std::endl;
        double vector_t21_mag = sqrt(vector_t21_x*vector_t21_x + vector_t21_y*vector_t21_y);
        double vector_t11_mag = sqrt(vector_t11_x*vector_t11_x + vector_t11_y*vector_t11_y);
        double dot_t11 = (vectorcurrnext_x*vector_t11_x + vectorcurrnext_y*vector_t11_y)/vector_t11_mag;
        double dot_t21 = (vectorcurrnext_x*vector_t21_x + vectorcurrnext_y*vector_t21_y)/vector_t21_mag;
        std::cout << "dot_t11 = " << dot_t11 << std::endl;
        std::cout << "dot_t21 = " << dot_t21 << std::endl;
        if (dot_t11 < dot_t21)
            return Point(xt11, yt11);
        else
            return Point(xt21, yt21);
    }

    };

};