// Deterministic offline adapter for the repository scan matcher; no GNSS/TF input.
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include "aqua_sonar_loc/scan_matcher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

template<class T> T read(std::istream & input) {
  T value;input.read(reinterpret_cast<char*>(&value),sizeof(value));
  if (!input) throw std::runtime_error("truncated registered point stream");
  return value;
}
int main(int argc,char** argv) {
  try {
    if (argc!=4) throw std::runtime_error("input.bin trajectory.csv events.csv");
    std::ifstream input(argv[1],std::ios::binary);
    char magic[8];input.read(magic,8);
    if (!input || std::string(magic,8)!="MBESPC01") throw std::runtime_error("wrong input contract");
    const auto frames=read<uint32_t>(input);
    auto matcher=aqua_sonar_loc::create_scan_matcher("gicp");
    aqua_sonar_loc::ScanMatcherConfig cfg;
    cfg.backend="gicp";cfg.max_correspondence_distance=1.5;cfg.max_iterations=80;
    cfg.transformation_epsilon=1e-6;cfg.max_fitness_score=3.0;
    cfg.max_translation_step_m=1.0;cfg.max_rotation_step_rad=0.4;
    cfg.submap_size=1;cfg.use_motion_prior=false;matcher->configure(cfg);
    std::ofstream trajectory(argv[2]),events(argv[3]);
    trajectory<<std::setprecision(17)<<"timestamp_ns,native_ordinal,x,y,z,qx,qy,qz,qw\n";
    events<<"timestamp_ns,native_ordinal,status\n";
    for(uint32_t i=0;i<frames;++i) {
      const auto ns=read<int64_t>(input);const auto ordinal=read<uint32_t>(input);const auto n=read<uint32_t>(input);
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.height=1;cloud.width=n;cloud.is_dense=true;cloud.is_bigendian=false;
      cloud.point_step=12;cloud.row_step=n*12;cloud.data.resize(n*12);
      const char* names[]={"x","y","z"};
      for(uint32_t k=0;k<3;++k) {sensor_msgs::msg::PointField f;f.name=names[k];f.offset=k*4;f.count=1;f.datatype=f.FLOAT32;cloud.fields.push_back(f);}
      input.read(reinterpret_cast<char*>(cloud.data.data()),cloud.data.size());
      if(!input) throw std::runtime_error("truncated points");
      if(n<60) {events<<ns<<','<<ordinal<<",insufficient_common_points\n";continue;}
      aqua_sonar_loc::CloudSummary summary;summary.accepted=true;
      summary.total_points=n;summary.finite_xyz_points=n;summary.in_range_points=n;
      const auto result=matcher->match(cloud,summary);
      events<<ns<<','<<ordinal<<','<<result.status<<'\n';
      if (!result.success) continue;
      const auto & t=result.odom_to_base.translation;const auto & q=result.odom_to_base.rotation;
      trajectory<<ns<<','<<ordinal<<','<<t.x<<','<<t.y<<','<<t.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<'\n';
    }
    if(input.peek()!=EOF) throw std::runtime_error("unregistered trailing input");
    trajectory.flush();events.flush();
    if(!trajectory||!events) throw std::runtime_error("output write failed");
    return 0;
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 2;}
}
