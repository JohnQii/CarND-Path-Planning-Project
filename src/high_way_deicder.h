#ifndef CHANGE_LINE_SAFTEY_H
#define CHANGE_LINE_SAFTEY_H
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
enum ChangeLineType {None, Left, Right};

struct EgoInfo {
  double s;
  double d;
  double speed;
  double lane;
  double x;
  double y;
};

class HighWayDecider
{
public:
  explicit HighWayDecider(const double ego_s,
                          const double ego_d,
                          const double ego_speed,
                          const int pre_size,
                          const std::vector<std::vector<double>>& sensor_fusion,
                          double speed_limit = 50);
  /**
   * @brief change_line_decider
   * the decider to determine which line to change(left right)
   */
  ChangeLineType changeLineDecider();
  /**
   * @brief hasBlockingByOthers
   * @return is blocking by other slow car in the ego'lane.
   */
  bool hasBlockingByOthers();
  bool hasBlockingBySlowObstacles();
  /**
   * @brief isChangeLineSafe is change to left/right line is safe or not.
   * @param change_line_type
   * @return
   */
  bool isChangeLineSafe(ChangeLineType change_line_type);

  std::vector<double> getNearestBlockingSpeed();
private:
  double speed_limit_;
  //ego state
  EgoInfo ego_info_;
  int pre_size_;
  //in global coordinate.
  std::vector<std::vector<double> > sensor_fusion_;
  std::vector<std::vector<double> > blocking_obstacles_;
  //the nearest blocking obstacle
  std::vector<double> blocking_obstacle_;
};

#endif // CHANGE_LINE_SAFTEY_H
