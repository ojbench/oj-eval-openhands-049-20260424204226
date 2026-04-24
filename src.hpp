#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"
#include <cmath>
#include <algorithm>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////
    
    double time_to_collision(const Vec &my_velocity, int other_id) {
        Vec other_pos = monitor->get_pos_cur(other_id);
        Vec other_v = monitor->get_v_cur(other_id);
        double other_r = monitor->get_r(other_id);
        
        Vec delta_pos = pos_cur - other_pos;
        Vec delta_v = my_velocity - other_v;
        
        double a = delta_v.norm_sqr();
        if (a < 1e-9) {
            double dist = delta_pos.norm();
            double min_dist = r + other_r;
            return (dist < min_dist + 0.1) ? 0.0 : 1e9;
        }
        
        double b = 2.0 * delta_pos.dot(delta_v);
        double c = delta_pos.norm_sqr() - (r + other_r + 0.02) * (r + other_r + 0.02);
        
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return 1e9;
        
        double t = (-b - sqrt(discriminant)) / (2 * a);
        return (t > -0.01) ? t : 1e9;
    }
    
    bool is_safe_velocity(const Vec &velocity, double time_horizon = 0.15) {
        if (velocity.norm() > v_max + 0.01) return false;
        
        int n = monitor->get_robot_number();
        for (int i = 0; i < n; ++i) {
            if (i == id) continue;
            double ttc = time_to_collision(velocity, i);
            if (ttc < time_horizon) {
                return false;
            }
        }
        return true;
    }
    
    Vec avoid_collision_velocity(const Vec &preferred_v) {
        int n = monitor->get_robot_number();
        Vec avoidance_v = Vec(0, 0);
        
        for (int i = 0; i < n; ++i) {
            if (i == id) continue;
            
            Vec other_pos = monitor->get_pos_cur(i);
            Vec other_v = monitor->get_v_cur(i);
            double other_r = monitor->get_r(i);
            
            Vec delta_pos = pos_cur - other_pos;
            double dist = delta_pos.norm();
            double safe_dist = r + other_r + 1.0;
            
            if (dist < safe_dist) {
                Vec repulsion = delta_pos.normalize() * (safe_dist - dist) / safe_dist;
                avoidance_v += repulsion * 10.0;
            }
            
            Vec relative_v = preferred_v - other_v;
            Vec future_delta = delta_pos + relative_v * 0.2;
            double future_dist = future_delta.norm();
            
            if (future_dist < safe_dist) {
                Vec repulsion = future_delta.normalize() * (safe_dist - future_dist) / safe_dist;
                avoidance_v += repulsion * 5.0;
            }
        }
        
        return avoidance_v;
    }

public:

    Vec get_v_next() {
        /// TODO: You need to decide the speed of the robot at the next moment.
        ///       You can obtain information about the robot being processed in the member variable of class Controller.
        ///       You can obtain information about the other robot by accessing the interface of *monitor.
        /// Warning: You cannot use any static variable or global variable!
        ///          You should not try to output any information!
        ///          You cannot modify any code that is not allowed to be modified!
        ///          All illegal behavior will be voided.
        
        Vec to_target = pos_tar - pos_cur;
        double dist_to_target = to_target.norm();
        
        if (dist_to_target < 0.01) {
            return Vec(0, 0);
        }
        
        Vec desired_v = to_target.normalize() * std::min(v_max, dist_to_target / 0.1);
        
        if (is_safe_velocity(desired_v, 0.12)) {
            return desired_v;
        }
        
        Vec avoidance = avoid_collision_velocity(desired_v);
        Vec adjusted_v = desired_v + avoidance;
        double adjusted_speed = adjusted_v.norm();
        if (adjusted_speed > v_max) {
            adjusted_v = adjusted_v.normalize() * v_max;
        }
        
        if (is_safe_velocity(adjusted_v, 0.10)) {
            return adjusted_v;
        }
        
        double best_score = -1e9;
        Vec best_v = Vec(0, 0);
        
        Vec direct_to_target = to_target.normalize();
        for (int speed_idx = 0; speed_idx <= 6; ++speed_idx) {
            double speed = v_max * speed_idx / 6.0;
            Vec test_v = direct_to_target * speed;
            
            if (is_safe_velocity(test_v, 0.10)) {
                double score = speed * 2.0 - (1.0 - speed / v_max) * 0.3;
                if (score > best_score) {
                    best_score = score;
                    best_v = test_v;
                }
            }
        }
        
        int num_angles = 12;
        for (int angle_idx = 1; angle_idx < num_angles; ++angle_idx) {
            double angle = 2.0 * 3.14159265358979323846 * angle_idx / num_angles;
            double dir_x = cos(angle);
            double dir_y = sin(angle);
            Vec direction(dir_x, dir_y);
            
            for (int speed_idx = 1; speed_idx <= 4; ++speed_idx) {
                double speed = v_max * speed_idx / 4.0;
                Vec test_v = direction * speed;
                
                if (is_safe_velocity(test_v, 0.10)) {
                    Vec future_pos = pos_cur + test_v * 0.1;
                    double future_dist = (pos_tar - future_pos).norm();
                    double angle_to_target = to_target.normalize().dot(direction);
                    double score = -future_dist * 0.8 + speed * 0.3 + angle_to_target * 0.5;
                    
                    if (score > best_score) {
                        best_score = score;
                        best_v = test_v;
                    }
                }
            }
        }
        
        return best_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP
