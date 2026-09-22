#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <flann/flann.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

struct Less {
  bool operator()(const Point& a, const Point& b) const {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }
};

Cloud load(const std::string& path) {
  Cloud cloud;
  // PCL 1.10 rejects a valid zero-point PCD; the cleaner can emit one.
  std::ifstream header(path, std::ios::binary);
  if (!header) throw std::runtime_error("cannot load " + path);
  std::string line;
  bool declared_empty = false;
  while (std::getline(header, line)) {
    std::istringstream fields(line);
    std::string key;
    fields >> key;
    if (key == "POINTS") {
      std::size_t points = 0;
      if (fields >> points) declared_empty = points == 0;
    }
    if (key == "DATA") {
      if (declared_empty) return cloud;
      break;
    }
  }
  if (pcl::io::loadPCDFile<Point>(path,cloud) != 0) throw std::runtime_error("cannot load " + path);
  for(const auto& p:cloud.points) if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
    throw std::invalid_argument("comparison requires finite XYZ");
  return cloud;
}

void save(const std::string& path, Cloud& cloud) {
  cloud.width=static_cast<std::uint32_t>(cloud.points.size()); cloud.height=1;
  if(cloud.empty()) {
    std::ofstream out(path);
    out << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n";
    if(!out) throw std::runtime_error("cannot save " + path);
  } else if(pcl::io::savePCDFileBinary(path,cloud)!=0) throw std::runtime_error("cannot save " + path);
}

std::string directed(const Cloud& source, const Cloud& target, int threads) {
  std::ostringstream out; out << std::setprecision(12);
  out << "{\"source_points\":" << source.size() << ",\"target_points\":" << target.size();
  if(source.empty() || target.empty()) {
    out << ",\"p50_m\":null,\"p95_m\":null,\"p99_m\":null,\"max_m\":null,"
        << "\"fraction_gt_0_025m\":null,\"fraction_gt_0_05m\":null,\"fraction_gt_0_10m\":null,"
        << "\"unmatched_points\":" << (target.empty()?source.size():0) << '}';
    return out.str();
  }
  std::vector<float> data; data.reserve(target.size()*3);
  for(const auto& p:target.points) data.insert(data.end(),{p.x,p.y,p.z});
  flann::Matrix<float> dataset(data.data(),target.size(),3);
  flann::Index<flann::L2<float>> tree(dataset,flann::KDTreeSingleIndexParams(10,true)); tree.buildIndex();
  flann::SearchParams params(-1,0,true); params.cores=threads;
  std::vector<float> queries(4096*3), squared(4096);
  std::vector<std::size_t> indices(4096);
  std::vector<double> distances; distances.reserve(source.size());
  for(std::size_t begin=0;begin<source.size();begin+=4096) {
    const auto rows=std::min<std::size_t>(4096,source.size()-begin);
    for(std::size_t i=0;i<rows;++i) {
      queries[i*3]=source[begin+i].x; queries[i*3+1]=source[begin+i].y; queries[i*3+2]=source[begin+i].z;
    }
    flann::Matrix<float> q(queries.data(),rows,3), d(squared.data(),rows,1);
    flann::Matrix<std::size_t> ids(indices.data(),rows,1);
    if(tree.knnSearch(q,ids,d,1,params)!=static_cast<int>(rows)) throw std::runtime_error("incomplete nearest neighbor result");
    for(std::size_t i=0;i<rows;++i) distances.push_back(std::sqrt(static_cast<double>(std::max(0.0F,squared[i]))));
  }
  std::sort(distances.begin(),distances.end());
  auto quantile=[&](double p) {
    const double position=p*(distances.size()-1);
    const auto left=static_cast<std::size_t>(std::floor(position)),right=static_cast<std::size_t>(std::ceil(position));
    return distances[left]+(distances[right]-distances[left])*(position-left);
  };
  auto fraction=[&](double limit) {return static_cast<double>(distances.end()-std::upper_bound(distances.begin(),distances.end(),limit))/distances.size();};
  out << ",\"p50_m\":" << quantile(0.5) << ",\"p95_m\":" << quantile(0.95)
      << ",\"p99_m\":" << quantile(0.99) << ",\"max_m\":" << distances.back()
      << ",\"fraction_gt_0_025m\":" << fraction(0.025)
      << ",\"fraction_gt_0_05m\":" << fraction(0.05)
      << ",\"fraction_gt_0_10m\":" << fraction(0.10) << ",\"unmatched_points\":0}";
  return out.str();
}

int main(int argc,char** argv) {
  try {
    if(argc<4 || argc>5) {
      std::cerr << "Usage: pct_map_cleaner_compare OLD.pcd NEW.pcd OUTPUT_PREFIX [THREADS=1]\n";
      return 2;
    }
    int threads=argc==5?std::stoi(argv[4]):1;
    if(threads<1) throw std::invalid_argument("threads must be positive");
    const std::string prefix=argv[3];
    for(const auto& suffix:{".json","-old-only.pcd","-new-only.pcd"})
      if(std::filesystem::exists(prefix+suffix)) throw std::invalid_argument("output already exists: " + prefix+suffix);
    const auto parent=std::filesystem::path(prefix).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
    auto old=load(argv[1]), current=load(argv[2]);
    const auto forward=directed(old,current,threads), reverse=directed(current,old,threads);
    std::sort(old.points.begin(),old.points.end(),Less{});
    std::sort(current.points.begin(),current.points.end(),Less{});
    Cloud old_only,new_only;
    std::set_difference(old.points.begin(),old.points.end(),current.points.begin(),current.points.end(),
                        std::back_inserter(old_only.points),Less{});
    std::set_difference(current.points.begin(),current.points.end(),old.points.begin(),old.points.end(),
                        std::back_inserter(new_only.points),Less{});
    save(prefix+"-old-only.pcd",old_only); save(prefix+"-new-only.pcd",new_only);
    std::ostringstream report;
    report << "{\"old_points\":" << old.size() << ",\"new_points\":" << current.size()
           << ",\"old_only_points\":" << old_only.size() << ",\"new_only_points\":" << new_only.size()
           << ",\"old_to_new\":" << forward << ",\"new_to_old\":" << reverse << "}\n";
    std::ofstream output(prefix+".json"); output << report.str();
    if(!output) throw std::runtime_error("cannot write comparison report");
    std::cout << report.str(); return 0;
  } catch(const std::exception& error) {std::cerr << error.what() << '\n'; return 1;}
}
