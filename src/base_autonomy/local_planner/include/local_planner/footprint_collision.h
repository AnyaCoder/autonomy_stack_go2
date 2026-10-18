#ifndef LOCAL_PLANNER_FOOTPRINT_COLLISION_H
#define LOCAL_PLANNER_FOOTPRINT_COLLISION_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace footprint_collision {
struct Point { double x, y; };
struct Pose { double x, y, yaw; };

// Standing/walking envelope, centred on vehicle (not the forward lidar).
struct Footprint {
  double length = 0.70;
  double width = 0.40;
  double measurement_margin = 0.02;
  double tracking_margin = 0.03;
  double sampling_step = 0.01;
  double margin() const { return measurement_margin + tracking_margin; }
  double radius() const { return std::hypot(length / 2 + margin(), width / 2 + margin()); }
  void validate() const {
    if (!std::isfinite(length) || !std::isfinite(width) || length <= 0 || width <= 0 ||
        !std::isfinite(measurement_margin) || measurement_margin < 0 ||
        !std::isfinite(tracking_margin) || tracking_margin < 0 ||
        !std::isfinite(sampling_step) || sampling_step <= 0)
      throw std::invalid_argument("Invalid vehicle footprint or collision margin");
  }
};

// Buckets accelerate queries but retain original points: no voxel-centroid shift.
class ObstacleIndex {
 public:
  void clear() { buckets_.clear(); }
  void add(double x, double y) {
    if (std::isfinite(x) && std::isfinite(y)) buckets_[key(cell(x), cell(y))].push_back({x,y});
  }
  bool collides(const Pose& p, const Footprint& f, double sweep_padding = 0) const {
    const double hx = f.length / 2 + f.margin() + sweep_padding;
    const double hy = f.width / 2 + f.margin() + sweep_padding;
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    const double bx = std::abs(c)*hx + std::abs(s)*hy;
    const double by = std::abs(s)*hx + std::abs(c)*hy;
    for (int ix = cell(p.x-bx); ix <= cell(p.x+bx); ++ix) {
      for (int iy = cell(p.y-by); iy <= cell(p.y+by); ++iy) {
        const auto it = buckets_.find(key(ix,iy));
        if (it == buckets_.end()) continue;
        for (const auto& q : it->second) {
          const double dx=q.x-p.x, dy=q.y-p.y;
          if (std::abs(c*dx+s*dy) <= hx && std::abs(-s*dx+c*dy) <= hy) return true;
        }
      }
    }
    return false;
  }
  // Pad each sampled rectangle by the maximum intervening corner displacement.
  // This covers the continuous sweep, including obstacles between sample poses.
  bool sweep(const Pose& from, const Pose& to, const Footprint& f) const {
    const double da = std::atan2(std::sin(to.yaw-from.yaw), std::cos(to.yaw-from.yaw));
    const double travel = std::hypot(to.x-from.x,to.y-from.y) + f.radius()*std::abs(da);
    const int n = std::max(1, static_cast<int>(std::ceil(travel/f.sampling_step)));
    const double padding = travel/(2*n);
    for (int i=0; i<=n; ++i) {
      const double t=static_cast<double>(i)/n;
      if (collides({from.x+t*(to.x-from.x),from.y+t*(to.y-from.y),from.yaw+t*da},f,padding)) return true;
    }
    return false;
  }
  // Body-frame twist, held for reaction time, then linearly braked to rest.
  bool command(const Pose& start, double vx, double vy, double wz, const Footprint& f,
               double reaction, double deceleration, double angular_deceleration) const {
    const double braking=std::max(std::hypot(vx,vy)/deceleration,std::abs(wz)/angular_deceleration);
    const double duration=reaction+braking;
    const int n=std::max(1,static_cast<int>(std::ceil(duration/0.02)));
    const double dt=duration/n;
    Pose previous=start;
    for (int i=0;i<n;++i) {
      const double t=(i+0.5)*dt;
      const double scale=t<=reaction || braking==0 ? 1 : std::max(0.0,1-(t-reaction)/braking);
      const double dyaw=wz*scale*dt;
      const double mid=previous.yaw+dyaw/2;
      const double chord=std::abs(dyaw)<1e-9 ? 1.0 : std::sin(dyaw/2)/(dyaw/2);
      Pose next{previous.x+scale*dt*chord*(std::cos(mid)*vx-std::sin(mid)*vy),
                previous.y+scale*dt*chord*(std::sin(mid)*vx+std::cos(mid)*vy),previous.yaw+dyaw};
      // Bound deviation of the circular arc from its chord, too.
      Footprint arc=f;
      arc.tracking_margin += std::hypot(vx,vy)*scale*dt*std::abs(dyaw)/8;
      if (sweep(previous,next,arc)) return true;
      previous=next;
    }
    return false;
  }
  double safeCommandScale(const Pose& start, double vx, double vy, double wz,
                          const Footprint& f, double reaction, double deceleration,
                          double angular_deceleration) const {
    if (!command(start,vx,vy,wz,f,reaction,deceleration,angular_deceleration)) return 1;
    if (collides(start,f)) return 0;
    double lo=0,hi=1;
    for (int i=0;i<10;++i) {
      const double mid=(lo+hi)/2;
      if (command(start,mid*vx,mid*vy,mid*wz,f,reaction,deceleration,angular_deceleration)) hi=mid;
      else lo=mid;
    }
    return lo;
  }
 private:
  static int cell(double x) { return static_cast<int>(std::floor(x/0.10)); }
  static std::uint64_t key(int x,int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x))<<32) |
            static_cast<std::uint32_t>(y);
  }
  std::unordered_map<std::uint64_t,std::vector<Point>> buckets_;
};
}  // namespace footprint_collision
#endif
