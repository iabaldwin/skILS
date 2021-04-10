#include <iostream>
#include <deque>
#include <array>
#include <fstream>

#include <Eigen/Geometry>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <raylib.h>
#include <rlgl.h>

#include <json/reader.h>
#include <json/writer.h>

#include <gdal_priv.h>
#include <gdalwarper.h>

namespace {

  typedef std::array<double, 2> Coordinate;
  typedef std::array<Coordinate, 4> Quad;

  enum AXIS {
    X = 0,
    Y = 1,
    Z = 2
  };

  Coordinate centre(const Quad& quad) {
    return Coordinate{(quad.at(0).at(AXIS::X) + quad.at(2).at(AXIS::X)) / 2.0,
      (quad.at(0).at(AXIS::Y) + quad.at(2).at(AXIS::Y)) / 2.0};
  }

  std::ostream& operator<<(std::ostream& stream, const Coordinate & coordinate) {
    for (auto && axis : coordinate) {
      stream << axis << " ";
    }
    return stream;
  }

  std::ostream& operator<<(std::ostream& stream, const Quad & quad) {
    for (auto && coordinate	: quad) {
      stream << coordinate;
    }
    return stream;
  }

  struct Geometry {
    Coordinate project(const Coordinate& pixel) const {
      Coordinate projected;
      projected.at(AXIS::X) = projection.at(0) + pixel.at(AXIS::X) * projection.at(1) + pixel.at(AXIS::Y) * projection.at(2);
      projected.at(AXIS::Y) = projection.at(3) + pixel.at(AXIS::X) * projection.at(4) + pixel.at(AXIS::Y) * projection.at(5);
      return projected;
    }

    const std::array<double, 6> projection;
    const int rows;
    const int cols;
  };


  Quad derive_bounds(const Geometry& geometry) {
    Quad bounds;
    bounds.at(0) = geometry.project(Coordinate{0, geometry.cols});
    bounds.at(1) = geometry.project(Coordinate{geometry.rows, geometry.cols});
    bounds.at(2) = geometry.project(Coordinate{geometry.rows, 0});
    bounds.at(3) = geometry.project(Coordinate{0, 0});
    return bounds;
  }

  template <typename NumericType, int Dimension=2, int Transformation=Eigen::Affine>
    void operator*=(Coordinate& coordinate, const Eigen::Transform<NumericType, Dimension, Transformation>& transform) {
      Eigen::Vector2d result = transform * Eigen::Vector2d{coordinate.at(AXIS::X), coordinate.at(AXIS::Y)};
      coordinate.at(AXIS::X) = result[AXIS::X];
      coordinate.at(AXIS::Y) = result[AXIS::Y];
    }

  template <typename NumericType, int Dimension=2>
    void operator*=(Quad& coordinates, const Eigen::Transform<NumericType, 2, Eigen::Affine>& transform) {
      for (auto&& coordinate : coordinates) {
        coordinate *= transform;
      }
    }

  Eigen::Transform<double, 2, Eigen::Affine> centre(const Geometry& geometry) {
    const Coordinate projected = geometry.project(Coordinate{geometry.rows /2, geometry.cols /2});
    Eigen::Isometry2d c;
    c.linear().setIdentity();
    c.translation() = -1*Eigen::Vector2d{projected.at(AXIS::X), projected.at(AXIS::Y)};
    return c;
  }

  Eigen::Transform<double, 2, Eigen::Affine> scale(double scale) {
    Eigen::Transform<double, 2, Eigen::Affine> scaling;
    scaling.linear().setIdentity();
    scaling.translation().setZero();
    scaling.scale(scale);
    return scaling;
  }

  std::pair<Geometry, Image> readGeoTiff(const std::string input) {
    GDALAllRegister();
    GDALDataset * dataset = (GDALDataset*) GDALOpen(input.c_str(), GA_ReadOnly);
    CHECK_NOTNULL(dataset);
    const int rows = GDALGetRasterYSize(dataset);
    const int cols = GDALGetRasterXSize(dataset);
    std::cout << dataset->GetProjectionRef() << std::endl;

    std::vector<std::vector<unsigned char>> rgb{3};
    {
      for (int band_idx = 1; band_idx <= 3; ++band_idx) {
        const auto band = dataset->GetRasterBand(band_idx);
        GDALDataType bandType = GDALGetRasterDataType(band);
        int num_bytes = GDALGetDataTypeSizeBytes(bandType);
        std::vector<unsigned char> buffer(num_bytes*cols);
        for(int row=0; row<rows; row++) {
          CHECK_EQ(dataset->GetRasterBand(band_idx)->RasterIO(
                GF_Read,
                0, row,				 // x-offset (start of the row), y-offset
                cols, 1, 			 // x-size, y-size (one row in this case)
                buffer.data(), // buffer output
                cols, 1,			 // width/height of the image buffer
                bandType,			 // buffer-type
                0, 0), CE_None);
          rgb.at(band_idx-1).insert(rgb.at(band_idx-1).end(), buffer.begin(), buffer.end());
        }
      }
    }

    CHECK_EQ(rgb.at(0).size(), 5000 * 5000);

    Image image;
    image.width = 5000;
    image.height = 5000;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;

    uint8_t * buffer = new uint8_t[5000*5000*4];
    for (std::size_t i = 0; i < 5000*5000; ++i) {
      buffer[i * 3 + 0] = rgb.at(0).at(i);
      buffer[i * 3 + 1] = rgb.at(1).at(i);
      buffer[i * 3 + 2] = rgb.at(2).at(i);
      buffer[i * 3 + 3] = 0;
    }
    image.data = buffer;

    std::array<double, 6> projection;
    CHECK(dataset->GetGeoTransform(projection.data()) == CE_None);
    GDALClose(dataset);
    GDALDestroyDriverManager();
    Geometry geometry{.projection = projection,
      .rows = rows,
      .cols = cols};
    return {geometry, image};
  }

  Quad readApronGroundTruth(const std::string& file, const Geometry& geometry) {
    Json::CharReaderBuilder builder;
    Json::CharReader * reader = builder.newCharReader();
    CHECK_NOTNULL(reader);
    Json::Value coordinates;

    std::ifstream input(file);
    CHECK(input.good());
    std::string contents((std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    input.close();

    std::string errors;
    reader->parse(contents.c_str(),
        contents.c_str() + contents.size(),
        &coordinates, &errors);

    Quad output;
    for (auto && coordinate : coordinates) {
      Coordinate pixel;
      pixel.at(AXIS::X) = coordinate[0].asDouble();
      pixel.at(AXIS::Y) = 5000 - coordinate[1].asDouble();
      static int index = 0;
      output.at(index++) = geometry.project(pixel);;
    }
    return output;
  }

  std::pair<Vector3, Vector3> generateILS(const Quad& apron, float scale = 10) {
    Vector3 start{(apron.at(0).at(AXIS::X) + apron.at(3).at(AXIS::X)) / 2.0,
      0,
      (apron.at(0).at(AXIS::Y) + apron.at(3).at(AXIS::Y)) / 2.0};

    Vector3 direction{(apron.at(0).at(AXIS::X) - apron.at(1).at(AXIS::X)),
      0.3,
      (apron.at(0).at(AXIS::Y) - apron.at(1).at(AXIS::Y))};
    Vector3 end;
    end.x = start.x + (scale * direction.x);
    end.y = start.y + (scale * direction.y);
    end.z = start.z + (scale * direction.z);
    return {start, end};
  }


  std::deque<std::pair<Vector3, Vector3>> generateObservations(const std::pair<Vector3, Vector3>& ILS) {
    Eigen::Vector3d start = {std::get<0>(ILS).x,
      std::get<0>(ILS).y,
      std::get<0>(ILS).z};

    Eigen::Vector3d end = {std::get<1>(ILS).x,
      std::get<1>(ILS).y,
      std::get<1>(ILS).z};

    const double range = (end-start).norm();
    const Eigen::Vector3d delta = (end - start) / range;
    std::deque<std::pair<Vector3, Vector3>> observations;

    for (int i = 1; i <= std::floor(range); ++i) {
      Eigen::Vector3d sample = start + i * delta;
      observations.push_back({Vector3{sample[0], sample[1], sample[2]},
          std::get<0>(ILS)});
    }
    return observations;
  }

  void DrawTile(const Texture2D * const texture, const Quad& geometry) {
    constexpr float vertical_offset{0.0};
    rlPushMatrix();
    rlBegin(RL_QUADS);
    rlEnableTexture(texture->id);
    const auto lower_left = geometry.at(0);
    rlTexCoord2f(0.f, 0.f); rlVertex3f(lower_left.at(AXIS::X), vertical_offset, lower_left.at(AXIS::Y));
    const auto upper_left = geometry.at(3);
    rlTexCoord2f(0.f, 1.f); rlVertex3f(upper_left.at(AXIS::X), vertical_offset, upper_left.at(AXIS::Y));
    const auto upper_right = geometry.at(2);
    rlTexCoord2f(1.f, 1.f); rlVertex3f(upper_right.at(AXIS::X), vertical_offset, upper_right.at(AXIS::Y));
    const auto lower_right = geometry.at(1);
    rlTexCoord2f(1.f, 0.f); rlVertex3f(lower_right.at(AXIS::X), vertical_offset, lower_right.at(AXIS::Y));
    rlDisableTexture();
    rlEnd();
    rlPopMatrix();
  }

  void DrawApron(const Quad& coordinates) {
    constexpr double vertical_offset{0.0};
    DrawLine3D({coordinates.at(0).at(AXIS::X), vertical_offset, coordinates.at(0).at(AXIS::Y)},
        {coordinates.at(1).at(AXIS::X), vertical_offset, coordinates.at(1).at(AXIS::Y)},
        RED);
    DrawLine3D({coordinates.at(1).at(AXIS::X), vertical_offset, coordinates.at(1).at(AXIS::Y)},
        {coordinates.at(2).at(AXIS::X), vertical_offset, coordinates.at(2).at(AXIS::Y)},
        RED);
    DrawLine3D({coordinates.at(2).at(AXIS::X), vertical_offset, coordinates.at(2).at(AXIS::Y)},
        {coordinates.at(3).at(AXIS::X), vertical_offset, coordinates.at(3).at(AXIS::Y)},
        RED);
    DrawLine3D({coordinates.at(3).at(AXIS::X), vertical_offset, coordinates.at(3).at(AXIS::Y)},
        {coordinates.at(0).at(AXIS::X), vertical_offset, coordinates.at(0).at(AXIS::Y)},
        RED);
  }

  std::vector<Vector2> toScreen(const Quad input, const Camera & camera) {
    std::vector<Vector2> result;
    int counter = 0;
    for (auto && coordinate : input) {
      const Vector3 point{coordinate.at(0),
        0,
        coordinate.at(1)};
      const Vector2 projected = GetWorldToScreen(point, camera);
      result.push_back({projected.x, projected.y});
    }
    return result;
  }
}

DEFINE_string(coordinates, "", "Coordinates file delineating apron");
DEFINE_bool(drawApron, false, "Draw the ground-truth apron?");
DEFINE_bool(freeFly, false, "Free-fly camera?");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(not FLAGS_coordinates.empty());

  const int width = 2 * 900;
  const int height = width;
  InitWindow(width, height, "skILS");

  auto background_image = LoadImage("/Users/ianbaldwin/Downloads/background.png");
  ImageResize(&background_image, width, height);
  const auto background_texture = LoadTextureFromImage(background_image);

  Camera3D camera;
  camera.position = (Vector3){ 16.0f, 18.0f, 0.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.type = CAMERA_PERSPECTIVE;                   // Camera mode type
  SetCameraMode(camera, CAMERA_FREE);
  SetTargetFPS(60);

  const std::vector<std::string> tiles {
    "/Users/ianbaldwin/Downloads/CA/2015/201502_san_francisco_ca_0x3000m_utm_cnir/vol003/10seg565625.tif",
    "/Users/ianbaldwin/Downloads/CA/2015/201502_san_francisco_ca_0x3000m_utm_cnir/vol004/10seg550625.tif"
  };

  Geometry* reference{nullptr};
  std::vector<std::pair<Quad, Texture>> tile_geometries;

  for (auto && tile : tiles) {
    const auto [geometry, image] = readGeoTiff(tile);
    const auto bounds = derive_bounds(geometry);
    const auto airfield = LoadTextureFromImage(image);
    if (not reference) {
      reference = new Geometry{geometry};
    }
    tile_geometries.push_back({bounds, airfield});
  }

  auto apron = readApronGroundTruth(FLAGS_coordinates, *reference);

  // Generate centering and scaling transforms
  const auto centering = centre(*reference);
  constexpr float factor = 1/50.0;
  const auto scaling = scale(factor);

  // Centre & scale apron
  apron *= centering;
  apron *= scaling;
  const Coordinate apron_centre = centre(apron);

  for (auto& tile : tile_geometries) {
    // Centre & scale tiles
    auto & geometry = std::get<0>(tile);
    geometry *= centering;
    geometry *= scaling;
    LOG(INFO) << geometry;
  }

  const auto ILS = generateILS(apron);
  const auto observations = generateObservations(ILS);

  while (not WindowShouldClose()) {
    static int counter = 0;
    static int render = 0;
    
    if (not FLAGS_freeFly) {
      auto [pose, target] = observations.at(counter);
      camera.position = pose;
      camera.target = target;
      if (++render > 20) {
        render = 0;
        ++counter;
        if (counter >= observations.size()) {
          counter = 0;
        }
      }
    } else {
      UpdateCamera(&camera);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);
    DrawTexture(background_texture, 0, 0, WHITE);

    BeginMode3D(camera);
    DrawGrid(10, 1.0f);
    for (const auto & tile : tile_geometries) {
      DrawTile(&std::get<1>(tile), std::get<0>(tile));
    }
    if (FLAGS_drawApron) {
      DrawApron(apron);
    }
    DrawLine3D(std::get<0>(ILS), std::get<1>(ILS), BLUE);
    EndMode3D();

    const auto projections = toScreen(apron, camera);

    if (FLAGS_drawApron) {
      DrawLineV(projections.at(0), projections.at(1), RED);
      DrawLineV(projections.at(1), projections.at(2), RED);
      DrawLineV(projections.at(2), projections.at(3), RED);
      DrawLineV(projections.at(3), projections.at(0), RED);
    }
    EndDrawing();

    if (render == 0 and not FLAGS_freeFly) {
      static int screenshots = 0;
      std::stringstream ss;
      ss << std::setw(3) << std::setfill('0') << screenshots++;
      TakeScreenshot((std::string{"data"} + std::string{"/"} + ss.str() + std::string{".png"}).c_str());

      Json::Value points(Json::arrayValue);;
      for (const auto & projection : projections) {
        Json::Value value;
        value["x"] = projection.x;
        value["y"] = projection.y;
        points.append(value);
      }

      Json::Value output;
      output["points"] = points;
      output["image"] = ss.str();
      Json::StreamWriterBuilder builder;
      std::ofstream jsonOutput{std::string{"data"} + std::string{"/"} + ss.str() + ".json"};
      jsonOutput << Json::writeString(builder, output);
      jsonOutput.close();
    }
  }
  CloseWindow();
}
