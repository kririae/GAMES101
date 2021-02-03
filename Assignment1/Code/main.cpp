#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

// #define NDEBUG

constexpr double MY_PI = 3.1415926;

// Eigen::Matrix4f
// get_view_matrix(const Eigen::Vector3f &eye_pos, const Eigen::Vector3f &look_at, const Eigen::Vector3f &up_dir) {
//
//   Eigen::Vector3f a, b, c;
//   a = look_at.cross(up_dir);
//   b = up_dir;
//   c = look_at;
//   Eigen::Matrix4f view;
//   view << a(0), a(1), a(2), 0,
//           b(0), b(1), b(2), 0,
//           c(0), c(1), c(2), 0,
//           0, 0, 0, 1;
//
//   Eigen::Matrix4f translate;
//   translate << 1, 0, 0, -eye_pos[0],
//           0, 1, 0, -eye_pos[1],
//           0, 0, 1, -eye_pos[2],
//           0, 0, 0, 1;
//
//   view = view * translate;
//
//   return view;
// }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0],
      0, 1, 0, -eye_pos[1],
      0, 0, 1, -eye_pos[2],
      0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

  float deg = rotation_angle * MY_PI / 180.0f;

  model(0, 0) = std::cos(deg);
  model(0, 1) = -std::sin(deg);
  model(1, 0) = -model(0, 1);
  model(1, 1) = model(0, 0);

  return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
  zNear = -zNear;
  zFar = -zFar;
  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f move = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();

  // Initialize the $M_{proj}$
  projection(0, 0) = zNear;
  projection(1, 1) = zNear;
  projection(3, 3) = 0.0f;
  projection(3, 2) = 1.0f;
  projection(2, 2) = zNear + zFar;
  projection(2, 3) = -zNear * zFar;

  // The size of the Cube after projection transformation
  float _x, _y, _z;
  _y = 2.0f * std::tan(eye_fov * MY_PI / 180.0f / 2.0f) * zNear;
  _x = _y * aspect_ratio;
  _z = std::fabs(zNear - zFar);

  move(2, 3) = -(zNear + zFar) / 2.0f;
  scale(0, 0) = 1.0f / _x;
  scale(1, 1) = 1.0f / _y;
  scale(2, 2) = 1.0f / _z;

#ifndef NDEBUG
  std::cout << "Projection: " << std::endl;
  std::cout << projection << std::endl;
  std::cout << "Scale" << std::endl;
  std::cout << scale << std::endl;
  std::cout << "Move" << std::endl;
  std::cout << move << std::endl;
#endif

  return scale * move * projection;
}

int main(int argc, const char **argv) {

  int w = 500, h = 500;
  float angle = 0;
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]); // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    } else
      return 0;
  }

  rst::rasterizer r(w, h);

  Eigen::Vector3f eye_pos = {0, 0, 2};
  Eigen::Vector3f look_at = {0, 0, -1};
  Eigen::Vector3f up_dir = {0, 1, 0};

  std::vector<Eigen::Vector3f> pos{{2, 0, -2},
                                   {0, 2, -2},
                                   {-2, 0, -2}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);

#ifdef NDEBUG
  int key = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(h, w, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(h, w, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << "\n";

    if (key == 'a') {
      angle += 5;
    } else if (key == 'd') {
      angle -= 5;
    }
  }
#endif

#ifndef NDEBUG
  r.set_model(get_model_matrix(angle));
  r.set_view(get_view_matrix(eye_pos, look_at, up_dir));
  r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

  r.draw(pos_id, ind_id, rst::Primitive::Triangle);

  cv::Mat image(h, w, CV_32FC3, r.frame_buffer().data());
  image.convertTo(image, CV_8UC3, 1.0f);
  std::cout << "Show image" << std::endl;
  cv::imshow("image", image);
#endif

  return 0;
}
