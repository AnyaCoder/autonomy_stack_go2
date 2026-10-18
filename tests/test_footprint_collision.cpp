#include "local_planner/footprint_collision.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <iostream>
using namespace footprint_collision;

int main() {
  Footprint f;
  f.validate();
  ObstacleIndex obstacles;
  // 60 cm corridor: straight walking fits; turning sideways does not.
  for (double x=-1; x<=2; x+=0.02) {
    obstacles.add(x,0.30);
    obstacles.add(x,-0.30);
  }
  assert(!obstacles.sweep({0,0,0},{1,0,0},f));
  assert(obstacles.sweep({0,0,0},{0,0,1.57079632679},f));
  assert(!obstacles.command({0,0,0},0.35,0,0,f,0.2,0.8,1.0));
  obstacles.clear();
  obstacles.add(0.30,0.30);
  assert(!obstacles.collides({0,0,0},f));
  assert(obstacles.command({0,0,0},0,0,0.5,f,0.2,0.8,1.0));
  assert(!obstacles.command({0,0,0},0,0,-0.5,f,0.2,0.8,1.0));
  // A thin obstacle between samples/endpoints must not be skipped.
  obstacles.clear();
  obstacles.add(0.5,0);
  assert(!obstacles.collides({0,0,0},f));
  assert(!obstacles.collides({1,0,0},f));
  assert(obstacles.sweep({0,0,0},{1,0,0},f));
  // Slow to a stable admissible speed instead of alternating full speed/stop.
  const double safe=obstacles.safeCommandScale({0,0,0},0.35,0,0,f,0.2,0.8,1.0);
  assert(safe>0 && safe<1);
  assert(!obstacles.command({0,0,0},safe*0.35,0,0,f,0.2,0.8,1.0));
  assert(obstacles.safeCommandScale({0,0,0},safe*0.35,0,0,f,0.2,0.8,1.0)==1);
  // Rear corner and negative spatial-index coordinates.
  obstacles.clear();
  obstacles.add(-0.30,-0.30);
  assert(obstacles.sweep({0,0,0},{0,0,0.5},f));
  obstacles.clear();
  obstacles.add(0.399,0.249);
  assert(obstacles.collides({0,0,0},f));
  obstacles.clear();
  obstacles.add(0.406,0);
  assert(!obstacles.collides({0,0,0},f));
  assert(obstacles.command({0,0,0},0.35,0,0,f,0.2,0.8,1.0));
  // Combined translation and rotation must be checked, not each in isolation.
  obstacles.clear();
  obstacles.add(0.43,0.27);
  assert(obstacles.command({0,0,0},0.35,0,0.5,f,0.2,0.8,1.0));
  // An obstacle inside the body envelope stays blocked at every path scale.
  obstacles.clear();
  obstacles.add(0.2,0.24);
  for (double scale : {0.5,0.75,1.0})
    assert(obstacles.sweep({0,0,0},{scale,0,0},f));
  // Broadphase buckets must agree with a brute-force oriented rectangle.
  for (int k=0; k<2000; ++k) {
    const double x=std::sin(k*0.7), y=std::cos(k*0.43), yaw=k*0.31;
    obstacles.clear(); obstacles.add(x,y);
    const double bx=std::cos(yaw)*x+std::sin(yaw)*y;
    const double by=-std::sin(yaw)*x+std::cos(yaw)*y;
    assert(obstacles.collides({0,0,yaw},f)==
        (std::abs(bx)<=0.40 && std::abs(by)<=0.25));
  }
  // Representative 252-candidate cycle, with retained dense wall points.
  obstacles.clear();
  for (double x=-3;x<3;x+=0.02) { obstacles.add(x,0.6); obstacles.add(x,-0.6); }
  const auto begin=std::chrono::steady_clock::now();
  int hits=0;
  for (int candidate=0;candidate<252;++candidate) {
    const double angle=(candidate%36)*0.174532925;
    Pose before{0,0,angle};
    for (int j=1;j<=100;++j) {
      Pose next{j*0.0075*std::cos(angle),j*0.0075*std::sin(angle),angle};
      if (obstacles.sweep(before,next,f)) { ++hits; break; }
      before=next;
    }
  }
  std::cout << "Footprint scenarios passed; 252 candidates: "
            << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()
            << " ms (" << hits << " blocked)\n";
}
