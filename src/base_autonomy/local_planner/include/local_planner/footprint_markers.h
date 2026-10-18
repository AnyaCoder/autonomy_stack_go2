#ifndef LOCAL_PLANNER_FOOTPRINT_MARKERS_H
#define LOCAL_PLANNER_FOOTPRINT_MARKERS_H

#include "local_planner/footprint_collision.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace footprint_collision {

// Render the same planar, yaw-only geometry used by the collision checker.
// Height is deliberately not invented: the checker has no 3-D body model.
inline visualization_msgs::msg::MarkerArray markers(
    const Footprint& footprint, const std_msgs::msg::Header& header,
    const Pose& pose, double z) {
  visualization_msgs::msg::MarkerArray result;
  auto base = [&](int id, const char* ns, int type, double r, double g, double b) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.id = id;
    m.ns = ns;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pose.x;
    m.pose.position.y = pose.y;
    m.pose.position.z = z;
    m.pose.orientation.z = std::sin(pose.yaw/2);
    m.pose.orientation.w = std::cos(pose.yaw/2);
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1;
    m.lifetime.nanosec = 500000000;
    return m;
  };
  auto outline = [&](int id, const char* ns, double margin, double r, double g, double b) {
    auto m = base(id, ns, visualization_msgs::msg::Marker::LINE_STRIP, r,g,b);
    m.scale.x = 0.012;
    const double hx = footprint.length/2+margin, hy = footprint.width/2+margin;
    for (const Point& corner : {Point{-hx,-hy}, Point{hx,-hy}, Point{hx,hy},
                                Point{-hx,hy}, Point{-hx,-hy}}) {
      geometry_msgs::msg::Point p;
      p.x = corner.x; p.y = corner.y;
      m.points.push_back(p);
    }
    result.markers.push_back(m);
  };
  outline(0, "body_envelope", 0, 0.1,0.4,1.0);
  outline(1, "measurement_margin", footprint.measurement_margin, 0.0,1.0,1.0);
  outline(2, "collision_envelope", footprint.margin(), 1.0,0.55,0.0);
  auto fill = base(3, "collision_area", visualization_msgs::msg::Marker::CUBE, 1.0,0.55,0.0);
  fill.scale.x = footprint.length+2*footprint.margin();
  fill.scale.y = footprint.width+2*footprint.margin();
  fill.scale.z = 0.004; // A planar overlay, not a claimed physical body height.
  fill.color.a = 0.12;
  result.markers.push_back(fill);
  auto heading = base(4, "body_forward", visualization_msgs::msg::Marker::ARROW, 1.0,1.0,1.0);
  heading.scale.x = 0.015; heading.scale.y = 0.035; heading.scale.z = 0.05;
  geometry_msgs::msg::Point origin, front;
  front.x = footprint.length/2;
  heading.points = {origin,front};
  result.markers.push_back(heading);
  return result;
}
} // namespace footprint_collision
#endif
