#include "pct_map_cleaner.hpp"
#include "reference/reference_cleaner.hpp"
#include <flann/flann.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>

using namespace pct_map_cleaner;
namespace old = reference_pct_map_cleaner;

void check(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}

CleanerConfig disabled() {
  CleanerConfig c;
  c.pre_sampling_enable = c.sor_enable = c.ror_enable = false;
  c.floating_cluster_enable = c.uniform_sampling_enable = c.ground_completion_enable = false;
  c.threads = 1;
  return c;
}

old::CleanerConfig old_config(const CleanerConfig& c) {
  old::CleanerConfig result;
#define COPY(field) result.field = c.field
  COPY(pre_sampling_enable); COPY(pre_spacing); COPY(max_pre_rep_error);
  COPY(sor_enable); COPY(sor_neighbors); COPY(sor_std_ratio);
  COPY(ror_enable); COPY(ror_min_neighbors); COPY(ror_radius);
  COPY(floating_cluster_enable); COPY(cluster_eps); COPY(cluster_min_points);
  COPY(cluster_max_points); COPY(cluster_max_bbox_x); COPY(cluster_max_bbox_y); COPY(cluster_max_bbox_z);
  COPY(uniform_sampling_enable); COPY(final_spacing);
  COPY(ground_completion_enable); COPY(ground_spacing); COPY(ground_support_radius);
  COPY(ground_z_tolerance); COPY(ground_min_support_directions); COPY(ground_min_support_cells);
  COPY(ground_max_hole_area); COPY(threads); COPY(save_intermediate);
#undef COPY
  return result;
}

bool same(const PointCloudXYZ& a, const PointCloudXYZ& b) {
  if (a.points.size() != b.points.size()) return false;
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    if (std::memcmp(&a.points[i].x, &b.points[i].x, sizeof(float)) != 0 ||
        std::memcmp(&a.points[i].y, &b.points[i].y, sizeof(float)) != 0 ||
        std::memcmp(&a.points[i].z, &b.points[i].z, sizeof(float)) != 0) return false;
  }
  return true;
}

PointCloudXYZ run(const CleanerConfig& c, const PointCloudXYZ& input) {
  flann::seed_random(1234);
  auto result = PctMapCleaner(c).clean(input);
  double sum = 0;
  for (const auto& stage : result.report.stage_times) sum += stage.second;
  check(sum == result.report.total_time, "detail timers must not be added to total");
  return result.cloud;
}

PointCloudXYZ reference(const CleanerConfig& c, const PointCloudXYZ& input) {
  old::PointCloudXYZ old_input;
  for (const auto& p : input.points) old_input.points.push_back({p.x, p.y, p.z});
  flann::seed_random(1234);
  auto result = old::PctMapCleaner(old_config(c)).clean(std::move(old_input));
  PointCloudXYZ output;
  for (const auto& p : result.cloud.points) output.points.push_back({p.x, p.y, p.z});
  return output;
}

void equivalence(const std::string& name, const CleanerConfig& c, const PointCloudXYZ& input) {
  const auto expected = reference(c, input);
  for (int threads : {1, 4, 8, 16}) {
    auto config = c;
    config.threads = threads;
    check(same(expected, run(config, input)), name + " output differs at threads=" + std::to_string(threads));
  }
}

void fails(const CleanerConfig& c, const PointCloudXYZ& input, const std::string& name) {
  bool current_threw = false, old_threw = false;
  try { run(c, input); } catch (const std::exception&) { current_threw = true; }
  try { reference(c, input); } catch (const std::exception&) { old_threw = true; }
  check(current_threw && old_threw, name + " expected exceptions");
}

void test_pre_and_finite() {
  auto c = disabled();
  const auto inf = std::numeric_limits<float>::infinity();
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  equivalence("finite", c, {{{0,0,0}, {nan,0,0}, {0,inf,0}, {-inf,0,0}, {1,2,3}}});
  fails(c, {}, "empty");
  fails(c, {{{nan,0,0}, {inf,0,0}}}, "no finite points");
  c.pre_sampling_enable = true;
  c.pre_spacing = 1;
  c.max_pre_rep_error = 2;
  const PointCloudXYZ input{{{0.25F,0.5F,0.5F}, {0.75F,0.5F,0.5F},
      {0.25F,0.5F,0.5F}, {-0.5F,0,0}, {-1,0,0}, {1,0,0},
      {std::nextafter(1.0F,0.0F),0,0}, {std::nextafter(-1.0F,-2.0F),0,0}}};
  equivalence("pre ties negative boundaries", c, input);
  const PointCloudXYZ pair{{{0.25F,0.5F,0.5F}, {0.75F,0.5F,0.5F}}};
  c.max_pre_rep_error = 0.5;
  equivalence("pre equal threshold", c, pair);
  check(run(c,pair).points.front().x == 0.25F, "pre tie uses first point");
  c.max_pre_rep_error = std::nextafter(0.5, 1.0);
  equivalence("pre above threshold", c, pair);
  c.max_pre_rep_error = std::nextafter(0.5, 0.0);
  fails(c,pair,"pre below threshold");
}

void test_ror() {
  auto c = disabled(); c.ror_enable = true; c.ror_radius = 1.0;
  PointCloudXYZ input{{{0,0,0}, {1,0,0}, {-1,0,0}, {0,0,0},
      {std::nextafter(1.0F,2.0F),0,0}, {3,0,0}}};
  for (int k : {1,2,3,4,5,6}) {
    c.ror_min_neighbors = k;
    equivalence("ror counts and boundary",c,input);
  }
  c.ror_min_neighbors = 1;
  check(run(c,{{{0,0,0}}}).points.empty(), "ror must exclude self");
  check(run(c,{{{0,0,0}, {1,0,0}}}).points.size() == 2, "ror includes radius boundary");
  check(run(c,{{{0,0,0}, {0,0,0}}}).points.size() == 2, "ror counts coincident neighbors");
}

void test_completion() {
  auto c = disabled(); c.ground_completion_enable = true;
  c.final_spacing = c.ground_spacing = 0.25;
  c.ground_support_radius = 0.75;
  c.ground_z_tolerance = 0.125;
  c.ground_max_hole_area = 0.25;
  PointCloudXYZ plane;
  for (int x=-6;x<=6;++x) for (int y=-6;y<=6;++y) {
    if (x==0 && y==0) continue;
    plane.points.push_back({(x+0.5F)*0.25F,(y+0.5F)*0.25F,0.125F});
  }
  equivalence("completion small hole", c, plane);
  check(run(c,plane).points.size() == plane.points.size()+1, "one-cell hole filled");
  PointCloudXYZ big;
  for (const auto& p : plane.points) if (std::abs(p.x-0.125F)>0.5F || std::abs(p.y-0.125F)>0.5F) big.points.push_back(p);
  equivalence("completion large hole",c,big);
  check(run(c,big).points.size()==big.points.size(), "large component not filled");
  PointCloudXYZ multilayer = plane;
  // Lower layer has a hole even though upper layer already occupies the XY cell.
  for(int x=-6;x<=6;++x) for(int y=-6;y<=6;++y)
    multilayer.points.push_back({(x+0.5F)*0.25F,(y+0.5F)*0.25F,2.125F});
  equivalence("completion multilayer",c,multilayer);
  check(run(c,multilayer).points.size()==multilayer.points.size()+1,"occupied XY still fills missing layer");
  c.ground_min_support_cells = 1000;
  equivalence("completion insufficient cells",c,plane);
  check(same(run(c,plane),plane),"insufficient cells must not fill");
  c.ground_min_support_cells = 4;
  PointCloudXYZ one_side;
  for(const auto& p:plane.points) if(p.x>0.125F) one_side.points.push_back(p);
  equivalence("completion direction support",c,one_side);
  PointCloudXYZ heights{{{-0.125F,0.125F,0}, {0.375F,0.125F,0.125F},
                        {0.125F,-0.125F,0}, {0.125F,0.375F,0.125F}}};
  equivalence("completion even median tolerance boundary",c,heights);
  heights.points.back().z = std::nextafter(0.125F,1.0F);
  equivalence("completion above height tolerance",c,heights);
}

PointCloudXYZ random_cloud(std::size_t count) {
  std::mt19937 gen(97);
  std::uniform_real_distribution<float> value(-3.0F,3.0F);
  PointCloudXYZ input;
  for(std::size_t i=0;i<count;++i) input.points.push_back({value(gen),value(gen),value(gen)});
  return input;
}

void test_query_means() {
  // Exercise full and partial blocks; compare the old int/single-query API
  // with the size_t/batch API on precisely the same random forest.
  auto input = random_cloud(4113);
  std::vector<float> data;
  for(const auto& p:input.points) data.insert(data.end(),{p.x,p.y,p.z});
  flann::Matrix<float> dataset(data.data(),input.points.size(),3);
  flann::seed_random(1234);
  flann::Index<flann::L2<float>> tree(dataset,flann::KDTreeIndexParams(4)); tree.buildIndex();
  const std::size_t k=21;
  std::vector<int> one_ids(k); std::vector<float> one_dist(k);
  std::vector<double> expected(input.points.size());
  for(std::size_t i=0;i<input.points.size();++i) {
    flann::Matrix<float> q(&data[i*3],1,3), d(one_dist.data(),1,k);
    flann::Matrix<int> ids(one_ids.data(),1,k);
    int found=tree.knnSearch(q,ids,d,k,flann::SearchParams(64));
    double sum=0; int used=0;
    for(int j=0;j<found;++j) if(one_ids[j]!=static_cast<int>(i)) {
      sum+=std::sqrt(std::max(0.0F,one_dist[j])); if(++used==20) break;
    }
    expected[i]=used==0?0:sum/used;
  }
  for(int threads:{1,4,8,16}) {
    std::vector<std::size_t> ids(4096*k); std::vector<float> distances(4096*k);
    flann::SearchParams params(64); params.cores=threads;
    for(std::size_t start=0;start<input.points.size();start+=4096) {
      const auto rows=std::min<std::size_t>(4096,input.points.size()-start);
      flann::Matrix<float> q(&data[start*3],rows,3), d(distances.data(),rows,k);
      flann::Matrix<std::size_t> indices(ids.data(),rows,k);
      check(tree.knnSearch(q,indices,d,k,params)==static_cast<int>(rows*k),"batch returned total");
      for(std::size_t row=0;row<rows;++row) {
        double sum=0; int used=0;
        for(std::size_t j=0;j<k;++j) if(ids[row*k+j]!=start+row) {
          sum+=std::sqrt(std::max(0.0F,distances[row*k+j])); if(++used==20) break;
        }
        check(sum/used==expected[start+row],"per-point SOR means differ");
      }
    }
  }
}

PointCloudXYZ brute_sor(const PointCloudXYZ& input, const CleanerConfig& c) {
  std::vector<double> means;
  for(std::size_t i=0;i<input.points.size();++i) {
    std::vector<float> distances;
    const float a[]={input.points[i].x,input.points[i].y,input.points[i].z};
    for(std::size_t j=0;j<input.points.size();++j) if(i!=j) {
      const float b[]={input.points[j].x,input.points[j].y,input.points[j].z};
      distances.push_back(flann::L2<float>{}(a,b,3));
    }
    std::sort(distances.begin(),distances.end()); double total=0;
    for(int j=0;j<c.sor_neighbors;++j) total+=std::sqrt(std::max(0.0F,distances[j]));
    means.push_back(total/c.sor_neighbors);
  }
  double mean=std::accumulate(means.begin(),means.end(),0.0)/means.size(),variance=0;
  for(double d:means) variance+=(d-mean)*(d-mean);
  double threshold=mean+c.sor_std_ratio*std::sqrt(variance/means.size());
  PointCloudXYZ output;
  for(std::size_t i=0;i<input.points.size();++i) if(means[i]<=threshold) output.points.push_back(input.points[i]);
  return output;
}

void test_exact_distances() {
  auto input = random_cloud(127);
  input.points.insert(input.points.end(), {{0,0,0}, {0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}});
  std::vector<float> data;
  for (const auto& p : input.points) data.insert(data.end(), {p.x,p.y,p.z});
  flann::Matrix<float> dataset(data.data(),input.points.size(),3);
  flann::Index<flann::L2<float>> tree(dataset,flann::KDTreeSingleIndexParams(10,true));
  tree.buildIndex();
  constexpr std::size_t k = 9;
  std::vector<std::size_t> ids(input.points.size()*k);
  std::vector<float> distances(input.points.size()*k);
  flann::Matrix<std::size_t> indices(ids.data(),input.points.size(),k);
  flann::Matrix<float> results(distances.data(),input.points.size(),k);
  flann::SearchParams params(-1,0,true); params.cores=4;
  tree.knnSearch(dataset,indices,results,k,params);
  for (std::size_t i=0;i<input.points.size();++i) {
    std::vector<float> expected;
    for (std::size_t j=0;j<input.points.size();++j)
      expected.push_back(flann::L2<float>{}(&data[i*3],&data[j*3],3));
    std::sort(expected.begin(),expected.end());
    for (std::size_t j=0;j<k;++j)
      check(distances[i*k+j]==expected[j], "single tree neighbor distances vs brute force");
  }
  auto c=disabled(); c.sor_enable=true; c.sor_search_backend=SorSearchBackend::SingleExact;
  input=random_cloud(4113);
  auto expected=run(c,input);
  for (int threads:{4,8,16}) { c.threads=threads; check(same(expected,run(c,input)),"exact block thread consistency"); }
}

void test_sor_and_pipeline() {
  auto c=disabled(); c.sor_enable=true; c.sor_neighbors=20; c.sor_std_ratio=1.0;
  equivalence("legacy SOR blocks",c,random_cloud(4113));
  equivalence("SOR small cloud",c,random_cloud(20));
  auto input=random_cloud(257);
  input.points.insert(input.points.end(),{{0,0,0},{0,0,0},{1,0,0},{-1,0,0},{0,1,0},{0,-1,0}});
  c.sor_search_backend=SorSearchBackend::SingleExact;
  auto expected=brute_sor(input,c);
  for(int threads:{1,4,8,16}) { c.threads=threads; check(same(expected,run(c,input)),"exact SOR vs brute force"); }
  PointCloudXYZ structures;
  for(int x=0;x<12;++x) for(int y=0;y<12;++y) structures.points.push_back({x*0.05F,y*0.05F,0});
  for(int z=1;z<12;++z) structures.points.push_back({0,0,z*0.05F});
  structures.points.insert(structures.points.end(),{{10,10,10},{-10,-10,-10}});
  c.sor_neighbors=4; c.threads=1;
  const auto old_output=reference(c,structures), exact_output=run(c,structures);
  auto thin=[](const PointCloudXYZ& cloud){return std::count_if(cloud.points.begin(),cloud.points.end(),[](const auto& p){return p.z>0 && p.z<1;});};
  auto noise=[](const PointCloudXYZ& cloud){return std::count_if(cloud.points.begin(),cloud.points.end(),[](const auto& p){return std::abs(p.x)>5;});};
  check(thin(exact_output)>=thin(old_output),"exact backend loses thin synthetic structure");
  check(noise(exact_output)<=noise(old_output),"exact backend retains more synthetic noise");
  CleanerConfig all; all.pre_spacing=0.05; all.max_pre_rep_error=0.1;
  all.final_spacing=all.ground_spacing=0.1;
  equivalence("all stages enabled",all,random_cloud(513));
  auto bad=c; bad.sor_search_backend=static_cast<SorSearchBackend>(999);
  bool threw=false; try { PctMapCleaner cleaner(bad); } catch(const std::invalid_argument&) { threw=true; }
  check(threw,"invalid backend enum");
}

int main() {
  try {
    test_pre_and_finite(); test_ror(); test_completion(); test_query_means(); test_exact_distances(); test_sor_and_pipeline();
    std::cout << "All point-cloud regression tests passed\n";
    return 0;
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
