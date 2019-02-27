#include "high_way_deicder.h"

HighWayDecider::HighWayDecider(const double ego_s,
                               const double ego_d,
                               const double ego_speed,
                               const int pre_size,
                               const std::vector<std::vector<double>>& sensor_fusion,
                               double speed_limit)
  : pre_size_(pre_size),
    sensor_fusion_(sensor_fusion),
    speed_limit_(speed_limit)
{
  ego_info_.s = ego_s;
  ego_info_.d = ego_d;
  ego_info_.speed = ego_speed;
  if(ego_d < 0) {
    //if ego_d < 0, it drive to the revise lane, it's an error.
    ego_info_.lane = -1;
  } else {
    int ego_lane = floor(ego_d / 4);
    ego_info_.lane = ego_lane;
  }

}

ChangeLineType HighWayDecider::changeLineDecider() {
  //need to change line or not.
  //  if(!hasBlockingByOthers())
  //    return ChangeLineType::None;
  if(blocking_obstacles_.size() <= 0)
    return ChangeLineType::None;

  std::vector<double> blocking_obstale = getNearestBlockingSpeed();

  if(ego_info_.s - blocking_obstale[0] > 60)
    return ChangeLineType::None;
  //check the speed difference between the nearest blocking cars.
  //only speed difference is bigger than 5 mph, we change line.
  if(speed_limit_ - blocking_obstale[1] > 3) {
    //first, we choose left lane to changline.
    //Because in china, we always only allow overtake the car in the left
    if(isChangeLineSafe(ChangeLineType::Left))
      return ChangeLineType::Left;
    //second, if left is not safe, we check the right lane.
    else if(isChangeLineSafe(ChangeLineType::Right))
      return ChangeLineType::Right;
    else
      return ChangeLineType::None;
  }
  return ChangeLineType::None;
}

bool HighWayDecider::hasBlockingByOthers() {
  blocking_obstacles_.clear();
  bool has_blocking = false;
  //get the obstacle in the ego lane.
  for(uint i = 0; i < sensor_fusion_.size(); ++i) {
    //get obstacle'd in frenet coordinate
    float d = sensor_fusion_[i][6];
    if(d <= (2 + 4*ego_info_.lane + 2) && d >= (2 + 4*ego_info_.lane - 2)) {
      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion_[i][5];

      check_car_s += (double)(pre_size_ * 0.02 * check_speed);
      if((check_car_s > ego_info_.s) && ((check_car_s - ego_info_.s) < 30)) {
        //ref_speed = 29.5;
        blocking_obstacles_.push_back(sensor_fusion_[i]);
      }
      if((check_car_s > ego_info_.s) && ((check_car_s - ego_info_.s) < 30) && ego_info_.speed > check_speed)
        has_blocking = true;
    }
  }
  return has_blocking;
}

bool HighWayDecider::isChangeLineSafe(ChangeLineType change_line_type) {
  //get the aim lane'd boundary.
  std::vector<double> lane_boundary;

  if(change_line_type == ChangeLineType::Left) {
    //the lane 0 don't have left lane.
    if(ego_info_.lane <= 0)
      return false;

    lane_boundary.push_back(2 + 4*(ego_info_.lane - 1) + 2);
    lane_boundary.push_back(2 + 4*(ego_info_.lane - 1) - 2);
  }else if(change_line_type == ChangeLineType::Right) {
    //the lane 2 don't have right lane.
    if(ego_info_.lane >= 2)
      return false;
    lane_boundary.push_back(2 + 4*(ego_info_.lane + 1) + 2);
    lane_boundary.push_back(2 + 4*(ego_info_.lane + 1) - 2);
  } else {
    lane_boundary.push_back(2 + 4*(ego_info_.lane) + 2);
    lane_boundary.push_back(2 + 4*(ego_info_.lane) - 2);
  }

  for(uint i = 0; i < sensor_fusion_.size(); ++i) {
    //get obstacle'd in frenet coordinate
    float d = sensor_fusion_[i][6];
    if(d <= lane_boundary[0] && d >= lane_boundary[1]) {

      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion_[i][5];

      check_car_s += (double)(pre_size_ * 0.02 * check_speed);
      double min_forward_distance = 10,
          min_backward_distance = 10;
      double forward_safe_distance = min_forward_distance;
      double backward_safe_distance = min_backward_distance;
      double safe_time = 5.; //s
      forward_safe_distance = safe_time * (ego_info_.speed - check_speed);
      if(forward_safe_distance < min_forward_distance)
        forward_safe_distance = min_forward_distance;

      backward_safe_distance = safe_time * (check_speed - ego_info_.speed);
      if(backward_safe_distance < min_backward_distance)
        backward_safe_distance = min_backward_distance;

      if((check_car_s - ego_info_.s < forward_safe_distance) && ((ego_info_.s - check_car_s) < backward_safe_distance)) {
        //ref_speed = 29.5;
        return false;
      }
    }
  }

  return true;
}

std::vector<double> HighWayDecider::getNearestBlockingSpeed() {
  double min_s = 100000; //the max s in map is 6548.
  double min_speed = 0;
  for(uint i = 0; i < blocking_obstacles_.size(); ++i) {
    if(blocking_obstacles_[i][5] > min_s) {
      min_s = blocking_obstacles_[i][5];
      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      min_speed = check_speed;
    }
  }
  std::vector<double> result;
  result.push_back(min_s);
  result.push_back(min_speed);
  return result;
}
