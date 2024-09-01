#include "chassis.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "util.hpp"


void Chassis::set_x(double x) {
    target.x = x;
}

void Chassis::set_y(double y) {
    target.y = y;
}

void Chassis::set_theta(double a) { set_angle(a); }

void Chassis::set_pose(double x, double y, double a) {
    current.x = x;
    current.y = y;
    set_theta(a);
}
void Chassis::reset_odom() {
    set_x(0);
    set_y(0);
    set_theta(0);
}

void Chassis::print_pose() {
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "X:%.2f Y:%.2f", current.x, current.y);
    pros::lcd::clear_line(2);
    pros::lcd::print(2, "Theta:%.1f", get_gyro());
    pros::lcd::clear_line(3);
    pros::lcd::print(3, "vEnc:%d", "hEnc:%d", vertical_position(), horizontal_position());
} 



Chassis::pose Chassis::projected_point(double distance, double xTarget, double yTarget, double theta, e_dir direction) {
    //x abd y error
    double xError = xTarget - current.x;
    double yError = yTarget - current.y;

    //angle from the hypot to the x axis
    
    double angle = atan(slope(xError, yError));
    
    /*
    double flip = direction == REV ? M_PI : 0;
    if (current.x > target.x) { flip = M_PI; }
    */
    pose projected;
    //calculate the x and y of the point projected off target
    projected.x = distance*cos(angle) + target.x;
    projected.y = distance*sin(angle) + target.y;
    return projected;
}

void Chassis::tracking_task() {
 
    double verticalCurrentPosition = 0, horizontalCurrentPosition = 0;
    double deltaVertical = 0, deltaHorizontal = 0;
    double verticalPrevious = 0, horizontalPrevious = 0;
    double radiusVertical = 0, radiusHorizontal = 0, hypotenuse = 0, hypotenuse2 = 0;
    double beta = 0, alpha = 0, theta = 0, lastTheta = 0;
    double Xa = 0, Ya = 0, Xb = 0, Yb = 0;
    reset_odom();
    while (true) {

      if (!imu.is_calibrating()) {
        //current position of the encoders
        verticalCurrentPosition = vertical_position() / TICKS_PER_INCH;
        horizontalCurrentPosition = horizontal_position() / TICKS_PER_INCH;
        
        //amount the left, right, and center side of the robot moved.
        deltaVertical = verticalCurrentPosition - verticalPrevious;
        deltaHorizontal = horizontalCurrentPosition - horizontalPrevious;

        //update the last values
        verticalPrevious = verticalCurrentPosition;
        horizontalPrevious = horizontalCurrentPosition;

        theta = to_rad(get_gyro()) - lastTheta; //angle traveled in radians
          lastTheta = to_rad(get_gyro());

        //Local Translation Vector
        if (theta != 0) {
            radiusVertical = deltaVertical / theta; //radius of right side arc
            beta = theta / 2.0; //half of angle traveled
            hypotenuse = 2.0 * (radiusVertical + RIGHT_OFFSET) * sin(beta); //local y-axis translation

            radiusHorizontal = deltaHorizontal / theta; //radius of the center arc
            hypotenuse2 = 2.0 * (radiusHorizontal + CENTER_OFFSET) * sin(beta); //local x-axis translation

        } else {
            hypotenuse = deltaVertical;
            hypotenuse2 = deltaHorizontal;
            beta = 0;
        }

        alpha = current.angleRad + beta; //average orientation (global ending angle of robot) (angle rad represents theta0)

        //update global positions
        Xa = hypotenuse * sin(alpha);
        Ya = hypotenuse * cos(alpha);

        Xb = hypotenuse2 * cos(alpha); // cos(alpha) = cos(-alpha)
        Yb = hypotenuse2 * -sin(alpha); // -sin(alpha) = sin(-alpha)

        current.x += Xa + Xb;
        current.y += Ya + Yb;

        current.angleRad += theta;
        current.angleDeg = util::to_deg(current.angleRad);
        
        util::print_with_delay(([this] { this->print_pose(); }), 20);

        //printf("x:%f y:%f theta:%f", x_pos, y_pos, angle_deg);
      }
      
    pros::delay(5);
    }
 
}





/*


double c_curPos = 0;
 double c = 0;
 double c_last = 0, lastTheta = 0;
 double radius_c = 0, h = 0;
 double beta = 0, alpha = 0, theta = 0;
 double Xa = 0, Ya = 0;
 double angle_rad = 0.0;
 reset_odom();
 
 while (false) {
  if (!imu.is_calibrating() && !is_driver) {
  ///
  //Single Wheel Tracking with one Encoder and IMU
  ///
  //current position of the encoder
  c_curPos = vertical_position() / TICKS_PER_INCH;

  //delta of center tracker and delta of theta
  c = c_curPos - c_last;
    c_last = c_curPos;
  theta = to_rad(get_gyro()) - lastTheta; //change in angle in radians
    lastTheta = to_rad(get_gyro());

  //Local Translation Vector
  if (theta != 0) {
   radius_c = c / theta; //radius of tracking wheel arc
   beta = theta / 2.0; //half of the change in angle
   h = 2.0 * (radius_c + CENTER_OFFSET) * sin(beta); 

  }else {
    h = c;
    beta = 0;
   }
  alpha = angle_rad + beta; //average orientation (global ending angle of robot) (angle rad represents theta0)

  //update global positions
  Xa = h * sin(alpha);
  Ya = h * cos(alpha);

  pose.x += Xa;
  pose.y += Ya;

  angle_rad += theta;
  angle_deg = get_gyro();

  print_pose();
  pros::delay(5);
 }
 
 
 */





 /*
 
     double l_curPos = 0, r_curPos = 0, c_curPos = 0;
    double l = 0, r = 0, c = 0;
    double l_last = 0, r_last = 0, c_last = 0;
    double radius_r = 0, radius_c = 0, h = 0, h2 = 0;
    double beta = 0, alpha = 0, theta = 0;
    double Xa = 0, Ya = 0, Xb = 0, Yb = 0;
    reset_odom();
    while (true) {

        //current position of the encoders
        l_curPos = left_position() / TICKS_PER_INCH;
        r_curPos = right_position() / TICKS_PER_INCH;
        c_curPos = center_position() / TICKS_PER_INCH;
        
        //amount the left, right, and center side of the robot moved.
        l = l_curPos - l_last;
        r = r_curPos - r_last;
        c = c_curPos - c_last;

        //update the last values
        l_last = l_curPos;
        r_last = r_curPos;
        c_last = c_curPos;
        theta = (l - r) / (LEFT_OFFSET + RIGHT_OFFSET); //angle traveled in radians

        //Local Translation Vector
        if (theta != 0) {
            radius_r = r / theta; //radius of right side arc
            beta = theta / 2.0; //half of angle traveled
            h = 2.0 * (radius_r + RIGHT_OFFSET) * sin(beta); //local y-axis translation

            radius_c = c / theta; //radius of the center arc
            h2 = 2.0 * (radius_c + CENTER_OFFSET) * sin(beta); //local x-axis translation

        } else {
            h = r;
            h2 = c;
            beta = 0;
        }

        alpha = angle_rad + beta; //average orientation (global ending angle of robot) (angle rad represents theta0)

        //update global positions
        Xa = h * sin(alpha);
        Ya = h * cos(alpha);

        Xb = h2 * cos(alpha); // cos(alpha) = cos(-alpha)
        Yb = h2 * -sin(alpha); // -sin(alpha) = sin(-alpha)

        pose.x += Xa + Xb;
        pose.y += Ya + Yb;

        angle_rad += theta;
        angle_deg = util::to_deg(angle_rad);

        //util::print_with_delay(([this] { this->print_pose(); }), 10);

        //printf("x:%f y:%f theta:%f", x_pos, y_pos, angle_deg);
        pros::delay(10);
    }
 
 */