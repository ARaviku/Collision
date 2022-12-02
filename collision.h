#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <vector>

using Edge = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// returns true if the equation a*t^2+b*t+c=0 has a solution between 0 and 1 (inclusive)
// HINT: recall your solution to `quadratic` in HW1
bool quadratic_has_valid_solution(double const a, double const b, double const c)
{
    // --- Your code here
    double det = (b * b) - (4 * a * c);
    if (det >= 0)
    {
        double x = (-b + sqrt(det)) / (2 * a);
        double y = (-b - sqrt(det)) / (2 * a);
        return ((x >= 0 && x <= 1)||(y >= 0 && y <= 1));
    }
    else
    {
        return false;
    }
    // ---
}

class Disc; // forward-declare the type Disc, otherwise we can't use it in the Obstacle base class

class Obstacle
{
public:
    // returns true if the robot is collides with the obstacle
    virtual bool check_collision(Disc const &robot) const = 0;

    // returns true if whether the point p is within this disc
    virtual bool contains(Eigen::Vector2d const &p) const = 0;
};

class Disc : public Obstacle
{
public:
    Disc(double x, double y, double radius) : pos_(x, y), radius_(radius) {}

    // override check_collision (HINT: use the `norm` function!)
    bool check_collision(Disc const &robot) const override
    {
        // --- Your code here
        Eigen::Vector2d r_vec = robot.pos_;
        double r_rad = robot.radius_;
        Eigen::Vector2d o_vec = pos_;
        double o_rad = radius_;

        Eigen::Vector2d diff = o_vec - r_vec;
        double diff_v = diff.norm();

        double dist_r = o_rad + r_rad;
        return (diff_v <= dist_r);

        // ---
    }

    // returns true if the point p is within this disc
    bool contains(Eigen::Vector2d const &p) const override
    {
        // --- Your code here
        return ((p - pos_).norm() <= (radius_));
        // ---
    }

    Eigen::Vector2d pos_;
    double radius_;
};
class Rectangle : public Obstacle
{
public:
    Rectangle(double x1, double y1, double x2, double y2) : bottom_left_(x1, y1), top_right_(x2, y2),
                                                            edges_{{{x1, y1}, {x2, y1}}, {{x2, y1}, {x2, y2}}, {{x2, y2}, {x1, y2}}, {{x1, y2}, {x1, y1}}},
                                                            corners_{{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}}{}

    // override check_collision
    // HINT: use the `Rectangle::check_intersection_with_edge`, `Rectangle::contains`, and `Disc::contains` functions
    // --- Your code here
    bool check_collision(Disc const &robot) const override
    {
        // --- Your code here
        bool dinrect = contains(robot.pos_);
        if (dinrect)
        {
            return true;
        }
        
        for (int i = 0; i < corners_.size(); i++)
        {
            if (robot.contains(corners_[i]))
            {
                return true;
            }
            if(check_intersection_with_edge(edges_[i], robot))
            {
                return true;
            }
        }

        return false;
        // ---
    }

    // Override the `contains` function
    // --- Your code here
    bool contains(Eigen::Vector2d const &p) const override
    {
        // --- Your code here
        return ((bottom_left_(0) <= p(0) && top_right_(0) >= p(0)) && (bottom_left_(1) <= p(1) && top_right_(1) >= p(1)));
        // ---
    }
    // ---

    // (HINT: use the `quadratic_has_valid_solution` function)
    bool check_intersection_with_edge(Edge const &e, Disc const &disc) const
    {
        // --- Your code here
        Eigen::Vector2d d_vec = e.second - e.first;
        Eigen::Vector2d f_vec = e.first - disc.pos_ ;
        double a_term = d_vec.dot(d_vec);
        double b_term = 2 * (f_vec.dot(d_vec));
        double c_term = (f_vec.dot(f_vec)) - (disc.radius_ * disc.radius_);
        return quadratic_has_valid_solution(a_term, b_term, c_term);
        // ---
    }

    Eigen::Vector2d bottom_left_;
    Eigen::Vector2d top_right_;
    std::vector<Edge> edges_;
    std::vector<Eigen::Vector2d> corners_;
};

bool check_collisions(std::vector<Obstacle const *> const &obstacles, Disc const &robot)
{
    // --- Your code here
    for (int i = 0; i < obstacles.size(); i++)
    {
        if ((obstacles[i]->check_collision(robot))){
            return true;
        }
    }

    return false;
    // ---
}