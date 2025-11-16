#ifndef SIMPLE_HULL_HPP
#define SIMPLE_HULL_HPP

#include "dse_classes.h"
#include <cmath>

// Simple geometry namespace
namespace ljgeo
{

    
        // Simple 2D point; r caches the polar angle during sorting.
    class Point2D
    {
    public:
        double x,y,r;
        Point2D(const Point2D &other)
        {
            x = other.x;
            y = other.y;
            r = other.r;
        }
        
        Point2D()
        {
            x = y = r = 0;
        }
 
        Point2D(int x, int y)
        {
            this->x = x;
            this->y = y;
            r = 0;
        }
   
        Point2D(double x, double y)
        {
            this->x = x;
            this->y = y;
            r = 0;
        }

        

                // Euclidean dist between two points.
        static double dist(Point2D p, Point2D q)
        {
            double x1 = q.x - p.x, y1 = q.y - p.y;
            return sqrt(x1*x1 + y1*y1);
        }

    };


    
        // Convex hull of a set of 2D points.
    class SimpleHull
    {
        vector<Point2D> points_;
        vector<int> convex_points_index_;
        size_t last_i_;
        bool finished_;

        
                // Compare two points by angle relative to the base point.
        class convex_points_less
        {
            const Point2D base;
        public:
            
            convex_points_less(const Point2D &b)
                : base(b)
            {
            }

            
            
            
            bool operator()(const Point2D &a, const Point2D &b)
            {
                if(IS_ZERO(a.r - b.r))
                    return Point2D::dist(a, base) > Point2D::dist(b, base);

                return a.r < b.r;
            }
        };
    
    public:
                // Constructor: start with an empty, unfinished hull.
        SimpleHull()
        {
            finished_ = false;
            last_i_ = 0;
        }

        
                // Remove all points and reset internal state.
        void clear()
        {
            finished_ = false;
            last_i_ = 0;
            points_.clear();
            convex_points_index_.clear();
        }

        
                // Access the i-th raw input point.
        Point2D &getPoint(size_t i)
        {
            return points_[i];
        }

        
                // Number of input points.
        size_t getPointCount()
        {
            return points_.size();
        }

        
                // Access the i-th vertex on the convex hull.
        Point2D &getHullPoint(size_t i)
        {
            return points_[convex_points_index_[i]];
        }

        
                // Number of vertices on the convex hull.
        size_t getHullCount()
        {
            return convex_points_index_.size();
        }

        
                // Add one input point and mark the hull as dirty.
        void addPoint(double x, double y)
        {
            points_.push_back(Point2D(x,y));

            
            finished_ = false;  
            convex_points_index_.clear();
        }

        
                // Remove one input point and mark the hull as dirty.
        void removePoint(size_t i)
        {
            points_.erase(i);

            
            finished_ = false;  
            convex_points_index_.clear();
        }

        
                // Find the starting point (lowest y, then lowest x).
        Point2D &findStartPoint()
        {
            size_t idx = 0;
            for(size_t i = 0; i < points_.size(); i++)
                if ((points_[i].x < points_[idx].x) || (points_[i].x == points_[idx].x && points_[i].y < points_[idx].y))
                    idx = i;

            return points_[idx];
        }

        
                // Pre-compute polar angles and sort points counterclockwise.
        void sortByAngle(Point2D &base)
        {
            
            for(size_t i = 0; i < points_.size(); i++)
            {
                double t = (base.y - points_[i].y) / Point2D::dist(base, points_[i]);
                if(IS_ZERO(t)) points_[i].r = dPI/2;
                else if(IS_ZERO(t-1)) points_[i].r = 0;
                else if(IS_ZERO(t+1)) points_[i].r = dPI;
                else points_[i].r = acos(t);
                
            }

            
            base.r = -1;

            
            quick_sort(points_, 0, points_.size() - 1, convex_points_less(base));

            

        }

        
        
        
        
        
                // Reset indices and sort points before running the algorithm.
        void initHull()
        {
            convex_points_index_.clear();
            last_i_ = 0;
            finished_ = false;

            sortByAngle(findStartPoint());

            if(points_.size() != 0)
                convex_points_index_.push_back(0);
        }

        
                // True when the hull is complete.
        bool isDone()
        {
            return finished_;
        }

        
                // Check whether p -> q -> r makes a left turn or is collinear.
        bool isLeftTurn(Point2D &p, Point2D &q, Point2D &r)
        {
            double x1 = q.x - p.x, y1 = q.y - p.y;
            double x2 = r.x - q.x, y2 = r.y - q.y;
            return (x1*y2 - x2*y1 >= 0) ;
        }

        
                // Whether there is another candidate point to consider.
        bool hasNext()
        {
            return last_i_ + 1 < points_.size();
        }

        
                // Index of the next candidate point in the sorted list.
        int nextIndex()
        {
            return last_i_ + 1;
        }

        
                // One step of the hull construction. Returns false when isDone.
        bool stepNext()
        {

            
            if(!hasNext())
            {
                finished_ = true;
                return false;
            }


            
            
            
            if(convex_points_index_.size() < 2)
            {
                convex_points_index_.push_back(++last_i_);
                return true;
            }

            
            Point2D &p1 = points_[convex_points_index_[convex_points_index_.size() - 2]];
            Point2D &p2 = points_[convex_points_index_[convex_points_index_.size() - 1]];
            
            Point2D &p3 = points_[last_i_ + 1];
        

            
            
            
            
            if(IS_ZERO(p2.r - p3.r))
            {
                last_i_++;
                return true;
            }

            
            
            if(!isLeftTurn(p1, p2, p3))
            {
               convex_points_index_.pop_back();
               return true;
            }
            else  
            {
                convex_points_index_.push_back(++last_i_);
                return true;
            }

        }
        
        
                // Run the full convex hull algorithm.
        void solve()
        {
            
            for(initHull();stepNext();)
                ;
        }
    };
}
#endif

