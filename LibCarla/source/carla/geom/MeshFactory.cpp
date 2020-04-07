// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/geom/MeshFactory.h>

#include <vector>

#include <carla/geom/Vector3D.h>
#include <carla/geom/Rtree.h>
#include <iostream>
#include <fstream>

namespace carla {
namespace geom {

  /// We use this epsilon to shift the waypoints away from the edges of the lane
  /// sections to avoid floating point precision errors.
  static constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

  std::unique_ptr<Mesh> MeshFactory::Generate(const road::Road &road) const {
    geom::Mesh out_mesh;
    for (auto &&lane_section : road.GetLaneSections()) {
      out_mesh += *Generate(lane_section);
    }
    return std::make_unique<Mesh>(out_mesh);
  }

  std::unique_ptr<Mesh> MeshFactory::Generate(const road::LaneSection &lane_section) const {
    geom::Mesh out_mesh;
    for (auto &&lane_pair : lane_section.GetLanes()) {
      out_mesh += *Generate(lane_pair.second);
    }
    return std::make_unique<Mesh>(out_mesh);
  }

  std::unique_ptr<Mesh> MeshFactory::Generate(const road::Lane &lane) const {
    RELEASE_ASSERT(road_param.resolution > 0.0);
    // The lane with lane_id 0 have no physical representation in OpenDRIVE
    geom::Mesh out_mesh;
    if (lane.GetId() == 0) {
      return std::make_unique<Mesh>(out_mesh);
    }
    const auto end_distance = lane.GetDistance() + lane.GetLength() - EPSILON;
    double current_s = lane.GetLaneSection()->GetDistance() + EPSILON;

    std::vector<geom::Vector3D> vertices;
    if (lane.IsStraight()) {
      // Mesh optimization: If the lane is straight just add vertices at the
      // begining and at the end of it
      const auto edges = lane.GetCornerPositions(current_s, road_param.extra_lane_width);
      vertices.push_back(edges.first);
      vertices.push_back(edges.second);
    } else {
      // Iterate over the lane's 's' and store the vertices based on it's width
      do {
        // Get the location of the edges of the current lane at the current waypoint
        const auto edges = lane.GetCornerPositions(current_s, road_param.extra_lane_width);
        vertices.push_back(edges.first);
        vertices.push_back(edges.second);

        // Update the current waypoint's "s"
        current_s += road_param.resolution;
      } while(current_s < end_distance);
    }

    // This ensures the mesh is constant and have no gaps between roads,
    // adding geometry at the very end of the lane
    if (end_distance - (current_s - road_param.resolution) > EPSILON) {
      current_s = end_distance;
      const auto edges = lane.GetCornerPositions(current_s, road_param.extra_lane_width);
      vertices.push_back(edges.first);
      vertices.push_back(edges.second);
    }

    // Add the adient material, create the strip and close the material
    out_mesh.AddMaterial(
        lane.GetType() == road::Lane::LaneType::Sidewalk ? "sidewalk" : "road");
    out_mesh.AddTriangleStrip(vertices);
    out_mesh.EndMaterial();
    return std::make_unique<Mesh>(out_mesh);
  }

  std::unique_ptr<Mesh> MeshFactory::Generate(const road::Junction &junction, const road::MapData &map) const {
    geom::Mesh out_mesh;

    // Get meshes
    //unique ptr
    std::vector<geom::Mesh> lane_meshes;
    for(const auto pair : junction.GetConnections()) {
      const auto &connection = pair.second;
      const auto &road = map.GetRoads().at(connection.connecting_road);
//debug assert (connection.connecting_road)
      for (auto &&lane_section : road.GetLaneSections()) {
        for (auto &&lane_pair : lane_section.GetLanes()) {
          lane_meshes.push_back(*Generate(lane_pair.second));
        }
      }
      //out_mesh += *Generate(road);
    }

    carla::log_warning("Num meshes: ", lane_meshes.size());

    // Build rtree structure
    struct VertexInfo {
      Mesh::vertex_type * vertex;
      size_t lane_mesh_idx;
      bool is_static;
    };
    using Rtree = geom::PointCloudRtree<VertexInfo>;
    using Point = Rtree::BPoint;
    Rtree rtree;
    size_t lane_mesh_idx = 0;
    for(auto &mesh : lane_meshes) {
      //for(auto& vertex : mesh.GetVertices()) {
      for(size_t i = 0; i < mesh.GetVerticesNum(); ++i) {
        auto& vertex = mesh.GetVertices()[i];
        Point point(vertex.x, vertex.y, vertex.z);
        if (i < 2 || i >= mesh.GetVerticesNum() - 2) {
          rtree.InsertElement({point, {&vertex, lane_mesh_idx, true}});
        } else {
          rtree.InsertElement({point, {&vertex, lane_mesh_idx, false}});
        }
      }
      ++lane_mesh_idx;
    }


    // struct TriangleInfo {
    //   Mesh::vertex_type *v1, *v2, *v3;
    //   size_t lane_mesh_idx;
    // };
    // using BPoint = boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>;
    // using BBox = boost::geometry::model::box<BPoint>;
    // using BPolygon = boost::geometry::model::polygon<BPoint, true, true>;
    // using PolyTreeElement = std::pair<BBox, TriangleInfo>;
    // using RtreePoly = boost::geometry::index::rtree<PolyTreeElement, boost::geometry::index::linear<16>>;
    // RtreePoly _rtree;
    // lane_mesh_idx = 0;
    // for(auto &mesh : lane_meshes) {
    //   for(size_t idx = 0; idx < mesh.GetIndexesNum() - 2; idx+=3) {
    //     auto &v1 = mesh.GetVertices()[idx + 0];
    //     auto &v2 = mesh.GetVertices()[idx + 1];
    //     auto &v3 = mesh.GetVertices()[idx + 2];
    //     BPoint pv1(v1.x, v1.y, v1.z);
    //     BPoint pv2(v2.x, v2.y, v2.z);
    //     BPoint pv3(v3.x, v3.y, v3.z);
    //     BPolygon poly;
    //     poly.outer().push_back(pv1);poly.outer().push_back(pv2);poly.outer().push_back(pv3);
    //     BBox box = boost::geometry::return_envelope<BBox>(poly);
    //     _rtree.insert({box, {&v1, &v2, &v3, lane_mesh_idx}});
    //   }
    //   ++lane_mesh_idx;
    // }

    // Find neighbors for each vertex and their weight
    struct VertexWeight {
      Mesh::vertex_type* vertex;
      double weight;
    };
    struct VertexNeighbors {
      Mesh::vertex_type* vertex;
      std::vector<VertexWeight> neighbors;
    };
    std::vector<VertexNeighbors> vertices_neighborhoods;
    lane_mesh_idx = 0;
    for(auto &mesh : lane_meshes) {
      for(size_t i = 0; i < mesh.GetVerticesNum(); ++i) {
        if (i > 2 && i < mesh.GetVerticesNum() - 2) {
          auto& vertex = mesh.GetVertices()[i];
          Point point(vertex.x, vertex.y, vertex.z);
          auto closest_vertices = rtree.GetNearestNeighbours(point, 20);
          VertexNeighbors vertex_neighborhood;
          vertex_neighborhood.vertex = &vertex;
          for(auto& close_vertex : closest_vertices) {
            auto &vertex_info = close_vertex.second;
            if(&vertex == vertex_info.vertex) {
              continue;
            }
            // compute weight
            double distance3D = geom::Math::Distance(vertex, *vertex_info.vertex);
            if(distance3D > 5) {
              continue;
            }
            if(abs(distance3D) < std::numeric_limits<double>::epsilon()) {
              continue;
            }
            double weight = geom::Math::Clamp(1.0 / (distance3D), 0.0, 100000.0);

            if(lane_mesh_idx == vertex_info.lane_mesh_idx) {
              weight *= 2;
              if(vertex_info.is_static) {
                weight *= 2;
              }
            }

            vertex_neighborhood.neighbors.push_back({vertex_info.vertex, weight});
          }
          vertices_neighborhoods.push_back(vertex_neighborhood);
        }
      }
      ++lane_mesh_idx;
    }

    // auto Laplacian = [&](const VertexInfo &vertex_info, std::vector<Rtree::TreeElement> &neighbors) -> double {
    //   double sum = 0;
    //   double sum_weight = 0;
    //   for(auto &element : neighbors) {
    //     double distance3D = geom::Math::Distance(*element.second.vertex, *vertex_info.vertex);
    //     //double distance = abs(element.second.vertex->z - vertex_info.vertex->z);
    //     if(abs(distance3D) < std::numeric_limits<double>::epsilon()) {
    //       continue;
    //     }
    //     double weight = geom::Math::Clamp(1.0 / distance3D, 0.0, 0.5);
    //     if(vertex_info.lane_mesh_idx == element.second.lane_mesh_idx){
    //       weight = 1;
    //     }else{
    //       weight = 0.5;
    //     }
    //     sum += (element.second.vertex->z - vertex_info.vertex->z)*weight;
    //     sum_weight += weight;
    //   }
    //   if(sum_weight > 0)
    //     return sum / sum_weight;
    //   else
    //     return 0;
    // };
    // // Run iterative algorithm
    // double lambda = 1.0;
    // int iterations = 30;
    // std::ifstream param_file("/home/axel/params.txt");
    // if(param_file){
    //   std::string line;
    //   std::getline(param_file, line);
    //   lambda = std::atof(line.c_str());
    //   std::getline(param_file, line);
    //   iterations = std::atoi(line.c_str());
    // }
    // carla::log_warning("iterations: ", iterations, "lambda:", lambda);
    // for(int iter = 0; iter < iterations; ++iter) {
    //   lane_mesh_idx = 0;
    //   for(auto &mesh : lane_meshes) {
    //     for(size_t i = 0; i < mesh.GetVerticesNum(); ++i) {
    //       auto& vertex = mesh.GetVertices()[i];
    //       Point point(vertex.x, vertex.y, vertex.z);
    //       auto neighbors = rtree.GetNearestNeighbours(point, 10);
    //       if (i > 2 && i < mesh.GetVerticesNum() - 2) {
    //         vertex.z += static_cast<float>(lambda*Laplacian({&vertex, lane_mesh_idx}, neighbors));
    //       }
    //     }
    //     ++lane_mesh_idx;
    //   }
    // }

    // Laplacian function
    auto Laplacian = [&](const Mesh::vertex_type* vertex, const std::vector<VertexWeight> &neighbors) -> double {
      double sum = 0;
      double sum_weight = 0;
      for(auto &element : neighbors) {
        sum += (element.vertex->z - vertex->z)*element.weight;
        sum_weight += element.weight;
      }
      if(sum_weight > 0)
        return sum / sum_weight;
      else
        return 0;
    };
    // Run iterative algorithm
    double lambda = 0.5;
    int iterations = 100;
    std::ifstream param_file("/home/axel/params.txt");
    if(param_file){
      std::string line;
      std::getline(param_file, line);
      lambda = std::atof(line.c_str());
      std::getline(param_file, line);
      iterations = std::atoi(line.c_str());
    }
    carla::log_warning("iterations: ", iterations, "lambda:", lambda);
    for(int iter = 0; iter < iterations; ++iter) {
      for (auto& vertex_neighborhood : vertices_neighborhoods) {
        auto * vertex = vertex_neighborhood.vertex;
        vertex->z += static_cast<float>(lambda*Laplacian(vertex, vertex_neighborhood.neighbors));
      }
    }

    for(auto &mesh : lane_meshes) {
      out_mesh += mesh;
    }
    return std::make_unique<Mesh>(out_mesh);
  }

} // namespace geom
} // namespace carla
