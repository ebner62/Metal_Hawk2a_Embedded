#pragma once
#include <Arduino.h>
#include <cmath> 

// --- MATH CONSTANTS & STRUCTS ---
const int RADIUS = 6371000;

struct Point2D {
    double x;
    double y;
};


// --- GPS DEFINITIONS ---
inline double dmsToRadians(float d, float m, float s) {
    double decimalDegrees = d + (m/60.0) + (s/3600.0);
    return decimalDegrees * (M_PI/180.0);
}

// --- NAVIGATION FUNCTIONS ---

// Input lat_rad and lon_rad from the main file, outputs a struct which lets us define the points p0, p1, and p2. (p2 WILL ALWAYS REMAIN THE SAME)
// p0, and p1 can change and later on i use & which is a pointer that will update those values everywhere in the code
// Initially we just need to define p0 the moment the can releases at whatever the gps location is.
inline Point2D gps_to_meters(double lat_rad, double lon_rad, double TARGET_LAT_RAD, double TARGET_LON_RAD) { 
    Point2D local_pos;
    local_pos.y = (lat_rad - TARGET_LAT_RAD) * RADIUS;
    local_pos.x = (lon_rad - TARGET_LON_RAD) * RADIUS * std::cos((TARGET_LAT_RAD + lat_rad)/2.0);
    return local_pos;
}

// I can't think of an instance where this function itself needs to be called in the main loop, its just a helper function used in the following functions
// Maybe use this in the main loop if u want to see when we should pull the brakes in order for egg drop
inline double get_distance(Point2D a, Point2D b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx*dx + dy*dy);
}

// Shouldn't need to call this in the main loop, it just makes the flight trajectory which is used the following functions
inline Point2D quadBezier(double t, Point2D p0, Point2D p1, Point2D p2) {
    double u = 1.0 - t;
    Point2D p;
    p.x = (u*u) * p0.x + 2*u*t * p1.x + (t*t) * p2.x;
    p.y = (u*u) * p0.y + 2*u*t * p1.y + (t*t) * p2.y;
    return p;
}

inline Point2D get_active_target(Point2D current_pos, Point2D p0, Point2D p1, Point2D p2) {
    double closest_t = 0;
    double min_dist = 9999999; 
    
    for (double t = 0; t <= 1.0; t += 0.05) { // Upgraded to 20 steps for smoother tracking
        Point2D curve_pt = quadBezier(t, p0, p1, p2);
        double dx = curve_pt.x - current_pos.x;
        double dy = curve_pt.y - current_pos.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_t = t;
        }
    }

    double target_t = closest_t + 0.10; // 10% lookahead
    if (target_t > 1.0) target_t = 1.0;

    return quadBezier(target_t, p0, p1, p2);
}

inline double get_distance_to_curve(Point2D current_pos, Point2D p0, Point2D p1, Point2D p2) {
    double min_dist = 9999999.0;
    for (double t = 0; t<= 1; t += 0.05) {
        Point2D curve_pt = quadBezier(t, p0, p1, p2);
        double dist = get_distance(current_pos, curve_pt);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}


// This function will calculate the cross product value for turning between -1 < x < 1. The magnitude of the value gives a refrence of how much the servos needed to be turned.
// In this main code  this value will be multiplied by some factor within the servo.write command. This function also accounts for antiparallelism through the dot product
inline double calculate_steering_error(Point2D current_pos, double current_heading_rad, Point2D p0, Point2D p1, Point2D p2) {
    // What is the current target along the bezier curve?
    Point2D active_target = get_active_target(current_pos, p0, p1, p2);

    // Create the target vector to that target
    double Tx = active_target.x - current_pos.x;
    double Ty = active_target.y - current_pos.y;
    double target_mag = sqrt(Tx*Tx + Ty*Ty);
    if (target_mag > 0) {Tx /= target_mag; Ty /= target_mag;}

    double Hx = sin(current_heading_rad);
    double Hy = cos(current_heading_rad);

    double cross_product = (Hx * Ty) - (Hy * Tx); // Left-Right error
    double dot_product = (Hx * Tx) + (Hy * Ty); // Front-back check

    double final_error = cross_product;
    if (dot_product < -0.5 && std::abs(cross_product) < 0.1) {
        //Target is directly behind us. Force a hard left turn.
        final_error = 1.0; 
    } 
    else if (dot_product < 0.0) {
        // Target is behind us, amplify the turn to whip around faster
        if (final_error > 0) final_error = 1.0;
        if (final_error < 0) final_error = -1.0;
    }

    // Returns a positive number to turn Left, negative to turn Right
    return final_error;

}
    
// This bool function uses a pass-by refrence pointer which will change the starting point of the bezier curve in all functions and redraw the flight path in case like
// i.e we get pushed further than some dist along the original curve. Also handles the scenario incase we need to change the gate in which we enter
inline bool check_and_redraw_path(Point2D current_pos, Point2D &p0, Point2D &p1, Point2D p2, Point2D gate1_pos, Point2D gate2_pos, double max_drift_meters = 20.0) {
    
    /* For gate 1_pos and gate2_pos use the gps_to_meters() function and input the valid parameters to get those values. The parameters are just const
    GATE1_LAT_RAD, GATE1_LON_RAD, and the same for gate 2.
    If we want we can just hard code it into the functions parameters so we don't forget but it would be better to pass it in so if we change the hardcoded
    original version it changes it everywhere thats needed*/
    
    double drift_distance = get_distance_to_curve(current_pos, p0, p1, p2);
    if (drift_distance > max_drift_meters) {
        p0 = current_pos;
        if (get_distance(p0, gate1_pos) < get_distance(p0, gate2_pos)) { // This logic changes the intermediate gate depending on which one we are closer to now
            p1 = gate1_pos;
        } else { 
            p1 = gate2_pos;
        }
        return true;
    }
    return false;
}

// Returns true ONLY if we are facing the target within an acceptable tolerance

inline bool is_aligned_for_braking(Point2D current_pos, double current_heading_rad, Point2D target_pos, double parallel_tolerance = 0.25) {
    
    double Tx = target_pos.x - current_pos.x;
    double Ty = target_pos.y - current_pos.y;
    
    double target_mag = std::sqrt(Tx*Tx + Ty*Ty);
    if (target_mag > 0) {
        Tx /= target_mag; 
        Ty /= target_mag;
    }

    double Hx = std::sin(current_heading_rad);
    double Hy = std::cos(current_heading_rad);

    double cross_product = (Hx * Ty) - (Hy * Tx); // 0 = parallel
    double dot_product = (Hx * Tx) + (Hy * Ty);   // 1 = facing forward, -1 = facing backward

    // Check if we are parallel enough AND actually facing the target
    if (std::abs(cross_product) <= parallel_tolerance && dot_product > 0.8) {
        return true; 
    }
    
    return false;
}

// This is the PID which incorporates the calculate_steering_error() function 

struct PIDController {
    double Kp;
    double Ki;
    double Kd;

    double integral_sum;
    double last_error;
    double last_derivative;
    unsigned long last_time;
    double filter_alpha;

    PIDController(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        integral_sum = 0;
        last_error = 0;
        last_time = 0;
        last_derivative = 0;
        filter_alpha = 0.5;
    }

    double compute(double current_error) {

        if (std::abs(current_error) < 0.05) { 
        current_error = 0.0; 
        }
        
        unsigned long current_time = millis();

        if (last_time == 0) {
            last_time = current_time;
            last_error = current_error;
            return 0.0;
        }
        double dt = (current_time - last_time) / 1000.0;
        if (dt <= 0.001) return 0.0;

        double P = Kp * current_error;

        integral_sum += (current_error * dt);
        if (integral_sum > 1.0) integral_sum = 1.0;
        if (integral_sum < -1.0) integral_sum = -1.0;
        double I = Ki * integral_sum;

        double raw_derivative = (current_error - last_error) / dt;
        double filtered_derivative = (filter_alpha * raw_derivative) + ((1 - filter_alpha) * last_derivative);
        double D = Kd * filtered_derivative;

        last_derivative = filtered_derivative;
        last_error = current_error;
        last_time = current_time;

        return P + I + D;
    }

};