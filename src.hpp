#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"

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
    
    bool will_collide(const Vec &my_velocity, int other_id, double time_step = 0.1) {
        Vec other_pos = monitor->get_pos_cur(other_id);
        Vec other_v = monitor->get_v_cur(other_id);
        double other_r = monitor->get_r(other_id);
        
        Vec delta_pos = pos_cur - other_pos;
        Vec delta_v = my_velocity - other_v;
        
        double a = delta_v.norm_sqr();
        double b = 2.0 * delta_pos.dot(delta_v);
        double c = delta_pos.norm_sqr() - (r + other_r) * (r + other_r);
        
        if (c < 0.01) return true;
        
        if (a < 1e-9) {
            return c < 0.01;
        }
        
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;
        
        double t1 = (-b - sqrt(discriminant)) / (2 * a);
        double t2 = (-b + sqrt(discriminant)) / (2 * a);
        
        return (t1 >= -0.01 && t1 <= time_step + 0.01) || (t2 >= -0.01 && t2 <= time_step + 0.01) || (t1 < 0 && t2 > time_step);
    }
    
    bool is_safe_velocity(const Vec &velocity) {
        if (velocity.norm() > v_max + 0.01) return false;
        
        int n = monitor->get_robot_number();
        for (int i = 0; i < n; ++i) {
            if (i == id) continue;
            if (will_collide(velocity, i)) {
                return false;
            }
        }
        return true;
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
        
        Vec desired_v = to_target.normalize() * v_max;
        
        if (is_safe_velocity(desired_v)) {
            return desired_v;
        }
        
        double best_score = -1e9;
        Vec best_v = Vec(0, 0);
        
        int num_angles = 16;
        int num_speeds = 5;
        
        for (int angle_idx = 0; angle_idx < num_angles; ++angle_idx) {
            double angle = 2.0 * 3.14159265358979323846 * angle_idx / num_angles;
            Vec direction(cos(angle), sin(angle));
            
            for (int speed_idx = 0; speed_idx <= num_speeds; ++speed_idx) {
                double speed = v_max * speed_idx / num_speeds;
                Vec test_v = direction * speed;
                
                if (is_safe_velocity(test_v)) {
                    Vec future_pos = pos_cur + test_v * 0.1;
                    double future_dist = (pos_tar - future_pos).norm();
                    double score = -future_dist + speed * 0.1;
                    
                    if (score > best_score) {
                        best_score = score;
                        best_v = test_v;
                    }
                }
            }
        }
        
        Vec direct_to_target = to_target.normalize();
        for (int speed_idx = 0; speed_idx <= num_speeds; ++speed_idx) {
            double speed = v_max * speed_idx / num_speeds;
            Vec test_v = direct_to_target * speed;
            
            if (is_safe_velocity(test_v)) {
                Vec future_pos = pos_cur + test_v * 0.1;
                double future_dist = (pos_tar - future_pos).norm();
                double score = -future_dist + speed * 0.1;
                
                if (score > best_score) {
                    best_score = score;
                    best_v = test_v;
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
