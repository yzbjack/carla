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

    std::vector<geom::Mesh> lane_meshes;
    for(const auto pair : junction.GetConnections()) {
      const auto &connection = pair.second;
      const auto &road = map.GetRoads().at(connection.connecting_road);

      for (auto &&lane_section : road.GetLaneSections()) {
        for (auto &&lane_pair : lane_section.GetLanes()) {
          lane_meshes.push_back(*Generate(lane_pair.second));
        }
      }
      //out_mesh += *Generate(road);
    }

    carla::log_warning("Num meshes: ", lane_meshes.size());

    struct VertexInfo {
      Mesh::vertex_type * vertex;
      bool is_static;
    };
    using Rtree = geom::PointCloudRtree<VertexInfo>;
    using Point = Rtree::BPoint;
    Rtree rtree;
    for(auto &mesh : lane_meshes) {
      //for(auto& vertex : mesh.GetVertices()) {
      for(size_t i = 0; i < mesh.GetVerticesNum(); ++i) {
        auto& vertex = mesh.GetVertices()[i];
        Point point(vertex.x, vertex.y, vertex.z);
        if (i < 2 || i >= mesh.GetVerticesNum() - 2) {
          rtree.InsertElement({point, {&vertex, true}});
        } else {
          rtree.InsertElement({point, {&vertex, false}});
        }
      }
    }

    auto Laplacian = [](Mesh::vertex_type & vertex, std::vector<Rtree::TreeElement> &neighbors) -> double {
      double sum = 0;
      for(auto &element : neighbors) {
        sum += (element.second.vertex->z - vertex.z);
      }
      return sum / neighbors.size();
    };

    double lambda = 0.0;
    int iterations = 30;
    std::ifstream param_file("$HOME/params.txt");
    if(param_file){
      std::string line;
      std::getline(param_file, line);
      lambda = std::atof(line.c_str());
      std::getline(param_file, line);
      iterations = std::atoi(line.c_str());
    }
    carla::log_warning("iterations: ", iterations, "lambda:", lambda);
    for(int iter = 0; iter < iterations; ++iter) {
      for(auto &mesh : lane_meshes) {
        for(size_t i = 0; i < mesh.GetVerticesNum(); ++i) {
          auto& vertex = mesh.GetVertices()[i];
          Point point(vertex.x, vertex.y, vertex.z);
          auto neighbors = rtree.GetNearestNeighbours(point, 10);
          neighbors.erase(neighbors.begin());
          if (i > 2 && i < mesh.GetVerticesNum() - 2) {
            vertex.z += static_cast<float>(lambda*Laplacian(vertex, neighbors));
          }
        }
      }
    }

    for(auto &mesh : lane_meshes) {
      out_mesh += mesh;
    }
    return std::make_unique<Mesh>(out_mesh);
  }

} // namespace geom
} // namespace carla
