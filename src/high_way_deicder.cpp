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
  //

  //first, we choose left lane to
}

bool HighWayDecider::hasBlockingByOthers() {
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
        has_blocking = true;
      }
    }
  }
  return has_blocking;
}
