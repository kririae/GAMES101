#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(const Eigen::Vector3f &eye_pos,
                                const Eigen::Vector3f &look_at,
                                const Eigen::Vector3f &up_dir) {
  Eigen::Vector3f a, b, c;
  a = look_at.cross(up_dir);
  b = up_dir;
  c = look_at;
  Eigen::Matrix4f view;
  view << a(0), a(1), a(2), 0,
      b(0), b(1), b(2), 0,
      c(0), c(1), c(2), 0,
      0, 0, 0, 1;

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0],
      0, 1, 0, -eye_pos[1],
      0, 0, 1, -eye_pos[2],
      0, 0, 0, 1;

  view = view * translate;

  return view;
}

Eigen::Matrix4f get_model_matrix(float angle) {
  Eigen::Matrix4f rotation;
  angle = angle * MY_PI / 180.f;
  rotation << cos(angle), 0, sin(angle), 0,
      0, 1, 0, 0,
      -sin(angle), 0, cos(angle), 0,
      0, 0, 0, 1;

  Eigen::Matrix4f scale;
  scale << 2.5, 0, 0, 0,
      0, 2.5, 0, 0,
      0, 0, 2.5, 0,
      0, 0, 0, 1;

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
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
  _y = 2.f * std::tan(eye_fov * MY_PI / 180.f / 2.f) * zNear;
  _x = _y * aspect_ratio;
  _z = std::fabs(zNear - zFar);

  move(2, 3) = -(zNear + zFar) / 2.0f;
  scale(0, 0) = 1.0f / _x;
  scale(1, 1) = 1.0f / _y;
  scale(2, 2) = 1.0f / _z;

  return scale * move * projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload) {
  return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload &payload) {
  Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
  Eigen::Vector3f result;
  result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
  return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f &vec, const Eigen::Vector3f &axis) {
  auto costheta = vec.dot(axis);
  return (2 * costheta * axis - vec).normalized();
}

struct light {
  Eigen::Vector3f position;
  Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
  Eigen::Vector3f return_color = {0, 0, 0};
  if (payload.texture) {
    return_color = payload.texture->getColor(payload.tex_coords(0), payload.tex_coords(1));
  }

  Eigen::Vector3f texture_color;
  texture_color << return_color.x(), return_color.y(), return_color.z();

  Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
  Eigen::Vector3f kd = texture_color / 255.f;
  Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

  auto l1 = light{{20, 20, 20}, {500, 500, 500}};
  auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

  std::vector<light> lights = {l1, l2};
  Eigen::Vector3f amb_light_intensity{10, 10, 10};
  Eigen::Vector3f eye_pos{0, 0, 5};

  float p = 150;

  Eigen::Vector3f color = texture_color;
  Eigen::Vector3f point = payload.view_pos;
  Eigen::Vector3f normal = payload.normal;

  Eigen::Vector3f result_color = {0, 0, 0};

  for (auto &light : lights) {
    Eigen::Vector3f l = light.position - point;
    Eigen::Vector3f v = eye_pos - point;
    l = -l;
    v = -v;

    float r2 = l.squaredNorm();
    l.normalize();
    v.normalize();
    Eigen::Vector3f h = (l + v);
    h.normalize();
    auto intensity = light.intensity / r2;

    Eigen::Vector3f ambient = {0, 0, 0},
        specular = {0, 0, 0},
        diffuse = {0, 0, 0};

    ambient = ka.cwiseProduct(amb_light_intensity);
    diffuse = kd.cwiseProduct(intensity) * std::max(0.f, normal.dot(l));
    specular = ks.cwiseProduct(intensity) * fastpow(std::max(0.f, normal.dot(h)), p);

    result_color += ambient;
    result_color += specular;
    result_color += diffuse;
  }

  return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload) {
  auto ka = Eigen::Vector3f(0.005, 0.005, 0.005);
  auto kd = payload.color;
  auto ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

  auto l1 = light{{20, 20, 20}, {500, 500, 500}};
  auto l2 = light{{-20, 20, 0}, {500, 500, 500}};
  auto l3 = light{{0, -20, 0}, {1000, 1000, 1000}};

  std::vector<light> lights = {l1, l2};
  Eigen::Vector3f amb_light_intensity{10, 10, 10};
  Eigen::Vector3f eye_pos{0, 0, 5};

  float p = 150;

  Eigen::Vector3f color = payload.color;
  Eigen::Vector3f point = payload.view_pos;
  Eigen::Vector3f normal = payload.normal;

  Eigen::Vector3f result_color = {0, 0, 0};
  for (auto &light : lights) {
    Eigen::Vector3f l = point - light.position;
    Eigen::Vector3f v = point - eye_pos;

    float r2 = l.squaredNorm();
    l.normalize();
    v.normalize();
    Eigen::Vector3f h = (l + v);
    h.normalize();
    auto intensity = light.intensity / r2;

    Eigen::Vector3f ambient = {0, 0, 0},
        specular = {0, 0, 0},
        diffuse = {0, 0, 0};

    ambient = ka.cwiseProduct(amb_light_intensity);
    diffuse = kd.cwiseProduct(intensity) * std::max(0.f, normal.dot(l));
    specular = ks.cwiseProduct(intensity) * fastpow(std::max(0.f, normal.dot(h)), p);

    result_color += ambient;
    result_color += specular;
    result_color += diffuse;
  }
  // std::cout << result_color << std::endl;

  return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload) {

  Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
  Eigen::Vector3f kd = payload.color;
  Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

  auto l1 = light{{20, 20, 20}, {500, 500, 500}};
  auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

  std::vector<light> lights = {l1, l2};
  Eigen::Vector3f amb_light_intensity{10, 10, 10};
  Eigen::Vector3f eye_pos{0, 0, 5};

  float p = 150;

  Eigen::Vector3f color = payload.color;
  Eigen::Vector3f point = payload.view_pos;
  Eigen::Vector3f normal = payload.normal;

  float kh = 0.2, kn = 0.1;

  auto &x = normal.x(),
      &y = normal.y(),
      &z = normal.z();
  auto d = std::sqrt(x * x + z * z);
  Eigen::Vector3f t = {x * y / d, -d, z * y / d};
  Eigen::Vector3f b = normal.cross(t);
  Eigen::Matrix3f TBN;
  TBN << t, b, normal;
  // std::cout << t.dot(normal) << " ";

  // Texture mapping
  assert(payload.texture);
  auto &u = payload.tex_coords.x(),
      &v = payload.tex_coords.y();
  auto col = payload.texture->getColor(u, v);
  auto dU = kh * kn * (payload.texture->getColor(u + 1.f / payload.texture->width, v).norm() - col.norm());
  auto dV = kh * kn * (payload.texture->getColor(u, v + 1.f / payload.texture->height).norm() - col.norm());
  Eigen::Vector3f ln = {-dU, -dV, 1.f};
  normal = TBN * ln;
  normal.normalize();

  Eigen::Vector3f result_color = {0, 0, 0};

  for (auto &light : lights) {
    Eigen::Vector3f l = point - light.position;
    Eigen::Vector3f v = point - eye_pos;

    float r2 = l.squaredNorm();
    l.normalize();
    v.normalize();
    Eigen::Vector3f h = (l + v);
    h.normalize();
    auto intensity = light.intensity / r2;

    Eigen::Vector3f ambient = {0, 0, 0},
        specular = {0, 0, 0},
        diffuse = {0, 0, 0};

    ambient = ka.cwiseProduct(amb_light_intensity);
    diffuse = kd.cwiseProduct(intensity) * std::max(0.f, normal.dot(l));
    specular = ks.cwiseProduct(intensity) * fastpow(std::max(0.f, normal.dot(h)), p);

    result_color += ambient;
    result_color += specular;
    result_color += diffuse;
  }

  return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload) {

  Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
  Eigen::Vector3f kd = payload.color;
  Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

  auto l1 = light{{20, 20, 20}, {500, 500, 500}};
  auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

  std::vector<light> lights = {l1, l2};
  Eigen::Vector3f amb_light_intensity{10, 10, 10};
  Eigen::Vector3f eye_pos{0, 0, 5};

  float p = 150;

  Eigen::Vector3f color = payload.color;
  Eigen::Vector3f point = payload.view_pos;
  Eigen::Vector3f normal = payload.normal;
  normal.normalize();

  float kh = 0.2, kn = 0.1;

  auto &x = normal.x(),
      &y = normal.y(),
      &z = normal.z();
  auto d = std::sqrt(x * x + z * z);
  Eigen::Vector3f t = {x * y / d, -d, z * y / d};
  Eigen::Vector3f b = normal.cross(t);
  Eigen::Matrix3f TBN;
  TBN << t, b, normal;

  // Texture mapping
  assert(payload.texture);
  auto &u = payload.tex_coords.x(),
      &v = payload.tex_coords.y();
  auto col = payload.texture->getColor(u, v);
  auto dU = kh * kn * (payload.texture->getColor(u + 1.f / payload.texture->width, v).norm() - col.norm());
  auto dV = kh * kn * (payload.texture->getColor(u, v + 1.f / payload.texture->height).norm() - col.norm());
  Eigen::Vector3f ln = {-dU, -dV, 1.f};
  normal = TBN * ln;
  normal.normalize();

  Eigen::Vector3f result_color = {0, 0, 0};
  result_color = normal;

  return result_color * 255.f;
}

int main(int argc, const char **argv) {
  // return 0;

  std::vector<Triangle *> TriangleList;

  float angle = 140.0;
  bool command_line = false;

  std::string filename = "output.png";
  objl::Loader Loader;
  std::string obj_path = "../models/spot/";

  // Load .obj File
  // bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
  bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
  for (auto mesh:Loader.LoadedMeshes) {
    for (int i = 0; i < mesh.Vertices.size(); i += 3) {
      Triangle *t = new Triangle();
      for (int j = 0; j < 3; j++) {
        t->setVertex(j,
                     Vector4f(mesh.Vertices[i + j].Position.X,
                              mesh.Vertices[i + j].Position.Y,
                              mesh.Vertices[i + j].Position.Z,
                              1.0));
        t->setNormal(j,
                     Vector3f(mesh.Vertices[i + j].Normal.X,
                              mesh.Vertices[i + j].Normal.Y,
                              mesh.Vertices[i + j].Normal.Z));
        t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
      }
      TriangleList.push_back(t);
    }
  }

  rst::rasterizer r(700, 700);

  auto texture_path = "hmap.jpg";
  r.set_texture(Texture(obj_path + texture_path));

  std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

  if (argc >= 2) {
    command_line = true;
    filename = std::string(argv[1]);

    if (argc == 3 && std::string(argv[2]) == "texture") {
      std::cout << "Rasterizing using the texture shader\n";
      active_shader = texture_fragment_shader;
      texture_path = "spot_texture.png";
      r.set_texture(Texture(obj_path + texture_path));
    } else if (argc == 3 && std::string(argv[2]) == "normal") {
      std::cout << "Rasterizing using the normal shader\n";
      active_shader = normal_fragment_shader;
    } else if (argc == 3 && std::string(argv[2]) == "phong") {
      std::cout << "Rasterizing using the phong shader\n";
      active_shader = phong_fragment_shader;
    } else if (argc == 3 && std::string(argv[2]) == "bump") {
      std::cout << "Rasterizing using the bump shader\n";
      active_shader = bump_fragment_shader;
    } else if (argc == 3 && std::string(argv[2]) == "displacement") {
      std::cout << "Rasterizing using the bump shader\n";
      active_shader = displacement_fragment_shader;
    }
  }

  Eigen::Vector3f eye_pos = {0, 0, 5};
  Eigen::Vector3f look_at = {0, 0, -1};
  Eigen::Vector3f up_dir = {0, 1, 0};

  r.set_vertex_shader(vertex_shader);
  active_shader = texture_fragment_shader;
  r.set_fragment_shader(active_shader);

  int key = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);
    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos, look_at, up_dir));
    r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

    r.draw(TriangleList);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imwrite(filename, image);

    return 0;
  }

  double ave = 0;
  while (key != 27) {
    struct timeval t1, t2;
    gettimeofday(&t1, nullptr);
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos, look_at, up_dir));
    r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

    //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
    r.draw(TriangleList);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imshow("image", image);
    cv::imwrite(filename, image);
    key = cv::waitKey(10);

    gettimeofday(&t2, nullptr);
    double dur = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0;

    // angle = 180;
    ++frame_count;
    ave = (ave * (frame_count - 1) + 1 / dur) / frame_count;
    std::cout << "average FPS: " << ave << " Angle: " << angle << std::endl;
    if (key == 'a') {
      angle -= 10;
    } else if (key == 'd') {
      angle += 10;
    }

    // angle = 180;
    // 129 121 255
    // break;
  }
  return 0;
}
