#include <GL/gl.h>
#include <GL/glew.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <glog/logging.h>
#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <nappear/mesh.h>
#include <ngeotc/chart.h>
#include <ngeotc/texgen.h>
#include <nimage/image.h>
#include <nmath/mat3x3.h>
#include <nmath/matrix.h>
#include <vector>

#define CCL_NAMESPACE_BEGIN namespace ccl {
#define CCL_NAMESPACE_END }

#include "device/device.h"
#include "render/background.h"
#include "render/bake.h"
#include "render/buffers.h"
#include "render/camera.h"
#include "render/integrator.h"
#include "render/mesh.h"
#include "render/object.h"
#include "render/scene.h"
#include "render/session.h"
#include "util/util_image.h"

#undef CCL_NAMESPACE_END
#undef CCL_NAMESPACE_BEGIN

#include "src/proto.h"
#include "src/proto/level.pb.h"
#include "src/proto/vec3.pb.h"
#include "src/ui/utigl/glwindow.h"

using namespace ccl;

static bool WriteRender(const uchar* pixels, int w, int h, int channels) {
  return true;
}

static bool WriteBuffer(const std::string& output_path,
                        const std::vector<float>& pixels, int w, int h) {
  float max_value = 0;
  for (int i = 0; i < (int)pixels.size(); i += 4) {
    max_value = std::max(max_value, pixels[i]);
    max_value = std::max(max_value, pixels[i + 1]);
    max_value = std::max(max_value, pixels[i + 2]);
  }
  nacb::Image8 output(w, h, 3);
  int i = 0;
  std::cerr << "\nMax:" << max_value << "\n";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      for (int c = 0; c < 3; ++c, ++i) {
        output(x, y, c) =
            (uint8_t)(std::min(255.0f, 255.0f * pixels[i] / max_value));
      }
      ++i;
    }
  }
  std::cout << "Writing image:" << output_path << "\n";
  return output.save(output_path.c_str());
}

// Cycles appears to be in static buffer mode. Paging in the input here is not
// necessary.
void ReadRenderTile(RenderTile& rtile) {}

void WriteRenderTile(RenderTile& rtile) {
  static int i = 0;
  std::cout << ".";
  ++i;
  if (i % 100 == 0)
    std::cout << "\n";
}

void ExtendBaryImage(const nappear::Mesh& mesh, nacb::Image32& tri,
                     nacb::Imagef& bary, int iterations) {
  for (int i = 0; i < iterations; ++i) {
    nacb::Imagef bary_new = bary.copy();
    nacb::Image32 tri_new = tri.copy();
    for (int y = 0; y < tri.h; ++y) {
      for (int x = 0; x < tri.w; ++x) {
        int t = tri(x, y);
        if (t < 0 || t >= (int)mesh.faces.size()) {
          int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          for (int ni = 0; ni < 4; ++ni) {
            int x2 = x + neigh[ni][0];
            int y2 = y + neigh[ni][1];
            if (x2 < 0 || y2 < 0 || x2 >= tri.w || y2 >= tri.h)
              continue;
            const int t2 = tri(x2, y2);
            if (t2 >= 0 && t2 < (int)mesh.faces.size()) {
              tri_new(x, y, 0) = tri(x2, y2, 0);
              bary_new(x, y, 0) = bary(x2, y2, 0);
              bary_new(x, y, 1) = bary(x2, y2, 1);
              bary_new(x, y, 2) = bary(x2, y2, 2);
              break;
            }
          }
        }
      }
    }
    tri = tri_new;
    bary = bary_new;
  }
}

void ImproveBaryImage(const nappear::Mesh& mesh, const nacb::Image32& tri,
                      const nacb::Imagef& bary) {
  std::vector<nacb::Mat3x3> sys((int)mesh.faces.size());
  const double w = tri.w;
  const double h = tri.h;

  for (int i = 0; i < (int)mesh.faces.size(); ++i) {
    const nacb::Vec2d d1 =
        mesh.tvert[mesh.faces[i].tci[1]] - mesh.tvert[mesh.faces[i].tci[0]];
    const nacb::Vec2d d2 =
        mesh.tvert[mesh.faces[i].tci[2]] - mesh.tvert[mesh.faces[i].tci[0]];
    nacb::Mat3x3 m = nacb::Mat3x3::eye();
    m(0, 0) = d1.x;
    m(1, 0) = d1.y;
    m(0, 1) = d2.x;
    m(1, 1) = d2.y;
    nacb::Mat3x3 t = nacb::Mat3x3::eye();
    t(0, 2) = -mesh.tvert[mesh.faces[i].tci[0]].x;
    t(1, 2) = -mesh.tvert[mesh.faces[i].tci[0]].y;

    sys[i] = (m.inverse() * t);
  }

  int fixed = 0;
  int same = 0;
  for (int y = 0; y < tri.h; ++y) {
    for (int x = 0; x < tri.w; ++x) {
      int t = tri(x, y);
      if (t >= 0 && t < (int)mesh.faces.size()) {
        nacb::Vec3d bc = sys[t] * nacb::Vec3d((0.5 + x) / w, (0.5 + y) / h, 1);
        if (bc.x >= 0 && bc.y >= 0 && bc.x + bc.y <= 1) {
          float a = (float)(1.0 - bc.x - bc.y);
          bary(x, y, 0) = a;
          bary(x, y, 1) = (float)bc.x;
          bary(x, y, 2) = (float)bc.y;
          fixed++;
        } else {
          same++;
        }
      }
    }
  }
  std::cout << "Fixed:" << fixed << ", same = " << same << "\n";
}

/**
   Return the bary-centric coordinate image for the mesh
   of size tw, th (zeros where there is no triangle).

   If timage!=null then fill it with the triangle indices (-1
   where there is no triangle).
*/
nacb::Imagef GetBaryImage(const nappear::Mesh& mesh, int tw, int th,
                          bool useTexCoords, nacb::Image32* timage) {
  nacb::Imagef bary(tw, th, 4);
  nacb::Image8 trgb(tw, th, 3);

  if (timage != 0) {
    *timage = nacb::Image32(tw, th, 1);
  }

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  glViewport(0, 0, tw, th);

  // Set-up texture matrices
  if (useTexCoords) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
  }
  glDisable(GL_CULL_FACE);

  glBegin(GL_TRIANGLES);
  for (int i = 0; i < (int)mesh.faces.size(); i++) {
    for (int k = 0; k < 3; k++) {
      glColor3f(k == 0, k == 1, k == 2);
      if (useTexCoords)
        glVertex2dv(mesh.tvert[mesh.faces[i].tci[k]].data);
      else
        glVertex3dv(mesh.vert[mesh.faces[i].vi[k]].data);
    }
  }
  glEnd();

  glReadPixels(0, 0, tw, th, bary.channelToGL(), bary.typeToGL(), bary.data);

  if (timage) {
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glBegin(GL_TRIANGLES);
    for (int i = 0; i < (int)mesh.faces.size(); i++) {
      for (int k = 0; k < 3; k++) {
        double r = double(i % 256) / 255.0;
        double g = double((i >> 8) % 256) / 255.0;
        double b = double((i >> 16) % 256) / 255.0;
        glColor3d(r, g, b);

        if (useTexCoords)
          glVertex2dv(mesh.tvert[mesh.faces[i].tci[k]].data);
        else
          glVertex3dv(mesh.vert[mesh.faces[i].vi[k]].data);
      }
    }
    glEnd();

    glReadPixels(0, 0, tw, th, trgb.channelToGL(), trgb.typeToGL(), trgb.data);
  }

  if (useTexCoords) {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }

  for (int y = 0; y < bary.h; y++) {
    for (int x = 0; x < bary.w; x++) {
      nacb::Vec3f co(bary(x, y, 0), bary(x, y, 1), bary(x, y, 2));
      double len = co.x + co.y + co.z;
      if (len < 1e-2) {
        bary(x, y, 0) = bary(x, y, 1) = bary(x, y, 2) = bary(x, y, 3) = 0;
        if (timage)
          (*timage)(x, y) = uint32_t(-1);
      } else {
        co *= (1.0 / len);

        if (timage) {
          int ti = int(trgb(x, y, 0)) + (int(trgb(x, y, 1)) << 8) +
                   (int(trgb(x, y, 2)) << 16);
          (*timage)(x, y) = ti;
        }
        for (int k = 0; k < 3; k++)
          bary(x, y, k) = co.data[k];
        bary(x, y, 3) = 1.0;
      }
    }
  }
  return bary;
}

ccl::Mesh* ConvertMesh(Scene* scene, const nappear::Mesh& mesh) {
  ccl::Mesh* cc_mesh = new Mesh();
  vector<float3> P;

  for (const auto& v : mesh.vert) {
    float3 p;
    p[0] = (float)v.x;
    p[1] = (float)v.y;
    p[2] = (float)v.z;
    P.push_back(p);
  }

  cc_mesh->subdivision_type = Mesh::SUBDIVISION_NONE;
  size_t num_triangles = mesh.faces.size();
  cc_mesh->reserve_mesh(mesh.vert.size(), num_triangles);
  cc_mesh->verts = P;

  const bool smooth = !true;
  for (const auto& face : mesh.faces) {
    cc_mesh->add_triangle(face.vi[0], face.vi[1], face.vi[2], 0, smooth);
  }

  // Taken from cycles_xml.cpp. Not sure if we need it.
  if (cc_mesh->need_attribute(scene, ATTR_STD_GENERATED)) {
    Attribute* attr = cc_mesh->attributes.add(ATTR_STD_GENERATED);
    memcpy(attr->data_float3(), cc_mesh->verts.data(),
           sizeof(float3) * cc_mesh->verts.size());
  }
  return cc_mesh;
}

static Scene* scene_init(const nappear::Mesh& mesh, Session* session) {
  Scene* scene = new Scene({}, session->device);

  ccl::Mesh* cc_mesh = ConvertMesh(scene, mesh);
  scene->geometry.push_back(cc_mesh);

  Object* object = new Object();
  object->geometry = cc_mesh;
  object->name = "hi";
  scene->objects.push_back(object);

  scene->camera->compute_auto_viewplane();
  scene->camera->width = 64;
  scene->camera->height = 64;
  return scene;
}

void BuildPrimitiveMaps(const nappear::Mesh& mesh, int bake_width,
                        int bake_height, std::vector<float>* primitive,
                        std::vector<float>* diff) {
  nacb::Imagef bary;
  nacb::Image32 timage;
  bary = GetBaryImage(mesh, bake_width, bake_height, true, &timage);
  bary.save("/tmp/bary.png");
  ImproveBaryImage(mesh, timage, bary);
  bary.save("/tmp/bary_after.png");
  ExtendBaryImage(mesh, timage, bary, 1);

  primitive->resize(bake_width * bake_height * 4, 0);
  diff->resize(bake_width * bake_height * 4, 0);

  for (int y = 0; y < bary.w; ++y) {
    for (int x = 0; x < bary.h; ++x) {
      const int tri = timage(x, y);
      if (tri >= 0 && tri < (int)mesh.faces.size()) {
        (*primitive)[(y * bake_width + x) * 4] = 0;
        (*primitive)[(y * bake_width + x) * 4 + 1] = __int_as_float(tri);
        (*primitive)[(y * bake_width + x) * 4 + 2] = bary(x, y, 0);
        (*primitive)[(y * bake_width + x) * 4 + 3] = bary(x, y, 1);

        // These derivatives are used for sampling random points within the
        // pixel
        if (x + 1 < timage.w && tri == (int)timage(x + 1, y)) {
          (*diff)[(y * bake_width + x) * 4 + 0] =
              (bary(x + 1, y, 0) - bary(x, y, 0));
          (*diff)[(y * bake_width + x) * 4 + 2] =
              (bary(x + 1, y, 1) - bary(x, y, 1));
        }
        if (y + 1 < timage.h && tri == (int)timage(x, y + 1)) {
          (*diff)[(y * bake_width + x) * 4 + 1] =
              (bary(x, y + 1, 0) - bary(x, y, 0));
          (*diff)[(y * bake_width + x) * 4 + 3] =
              (bary(x, y + 1, 1) - bary(x, y, 1));
        }
      }
    }
  }
}

BufferParams GetBufferParams(int bake_width, int bake_height) {
  BufferParams buffer_params;
  buffer_params.width = bake_width;
  buffer_params.height = bake_height;
  buffer_params.full_width = bake_width;
  buffer_params.full_height = bake_height;
  return buffer_params;
}

bool SessionInitAndBake(const SessionParams& session_params,
                        const nappear::Mesh& mesh,
                        const std::string& output_path, const int bake_width,
                        const int bake_height, int samples) {
  auto* session = new Session(session_params);
  auto* scene = scene_init(mesh, session);
  session->scene = scene;

  std::vector<float> primitive;
  std::vector<float> diff;
  BuildPrimitiveMaps(mesh, bake_width, bake_height, &primitive, &diff);

  // Can we bake in diffuse too?
  const ShaderEvalType shader_type = SHADER_EVAL_AO;
  const int bake_pass_filter = BAKE_FILTER_AO;

  // Which object to bake AO for?
  if (!scene->objects.size()) {
    std::cerr << "Unable to find object to bake\n";
    return false;
  }

  Object* object = scene->objects.front();
  std::cout << "object:" << object->name.string() << "\n";

  // Initialize bake manager to do what we want (has to be by name)
  scene->bake_manager->set(scene, object->name.string(), shader_type,
                           bake_pass_filter);

  // Create a combined pass (by name) so we can get the results.
  Pass::add(PASS_COMBINED, scene->film->passes, "Combined");

  // Setup tile callbacks (maybe not strictly necessary...)
  session->read_bake_tile_cb = ReadRenderTile;
  session->write_render_tile_cb = WriteRenderTile;

  // Update session.
  if (!session->progress.get_cancel()) {
    BufferParams buffer_params = GetBufferParams(bake_width, bake_height);
    buffer_params.passes = scene->film->passes;
    session->tile_manager.set_samples(samples);
    session->reset(buffer_params, samples);
    session->buffers->reset(buffer_params);
    // Can bind progress callbacks here using:
    // session->progress.set_update_callback
  }

  // Setup input parameters (hack by birkbeck). The buffers were always getting
  // set to zero before.
  if (!session->buffers->set_pass_rect_data(PASS_BAKE_PRIMITIVE, primitive)) {
    std::cerr << "Cannot set pass rect data\n";
    return false;
  }
  if (!session->buffers->set_pass_rect_data(PASS_BAKE_DIFFERENTIAL, diff)) {
    std::cerr << "Cannot set pass rect data\n";
    return false;
  }

  session->start();
  session->wait();

  // Get the results
  std::vector<float> pixels(bake_width * bake_height * 4, 0);
  float exposure = 1.0;
  session->buffers->get_pass_rect("Combined", exposure, samples, 4, &pixels[0]);
  WriteBuffer(output_path, pixels, bake_width, bake_height);

  session->read_bake_tile_cb = function_null;
  session->write_render_tile_cb = function_null;

  std::cout << "\nDone";
  delete session; // This crashes
  return true;
}

int ArgMax(const nacb::Vec3d& e) {
  int i = 0;
  for (int k = 0; k < 3; ++k) {
    if (fabs(e.data[k]) > fabs(e.data[i]))
      i = k;
  }
  return i;
}

template <class VecType>
void RemoveDuplicates(const std::vector<VecType>& input_vert,
                      std::vector<VecType>* output_vert,
                      std::unordered_map<int, int>* vert_mapping) {
  // There are relatively few number of mesh points.
  // So this doesn't need to be fast.
  const double kThresh = 1e-6;
  for (int i = 0; i < (int)input_vert.size(); ++i) {
    if (vert_mapping->count(i))
      continue;
    (*vert_mapping)[i] = output_vert->size();

    for (int j = i + 1; j < (int)input_vert.size(); ++j) {
      if (vert_mapping->count(j))
        continue;
      const double len = (input_vert[j] - input_vert[i]).len();
      if (len <= kThresh) {
        (*vert_mapping)[j] = output_vert->size();
      }
    }
    output_vert->push_back(input_vert[i]);
  }
}

// Simplify the mesh by removing duplicate vertices
nappear::Mesh RemoveDuplicates(const nappear::Mesh& mesh) {
  std::unordered_map<int, int> vert_mapping;
  std::vector<nacb::Vec3d> vert;
  RemoveDuplicates(mesh.vert, &vert, &vert_mapping);

  std::unordered_map<int, int> tvert_mapping;
  std::vector<nacb::Vec2d> tvert;
  RemoveDuplicates(mesh.tvert, &tvert, &tvert_mapping);

  nappear::Mesh output_mesh;
  output_mesh.vert = vert;
  output_mesh.tvert = tvert;

  for (int i = 0; i < (int)mesh.faces.size(); ++i) {
    auto face = mesh.faces[i];
    for (int k = 0; k < 3; ++k) {
      face.vi[k] = vert_mapping[face.vi[k]];
      face.tci[k] = tvert_mapping[face.tci[k]];
    }
    output_mesh.faces.push_back(face);
  }
  output_mesh.initNormals(/*flat=*/true);
  return output_mesh;
}

nappear::Mesh ConvertLevelToMesh(const state::Level& level) {
  std::vector<Chart> charts(6 * level.boxes_size() + 1);
  int face_i = 0;

  nacb::Vec3d min_bounds(1e10, 1e10, 1e10);
  nacb::Vec3d max_bounds(-1e10, -1e10, -1e10);
  // The texture packing attempts to rescale (based on area of world to
  // area of texture size) and if the bounds are greater than 1, it scales
  // down by the max bounds. We can trick it to not do this and to keep
  // the relative world scaling by scaling down the world coordinates.
  nacb::Mat3x3 global_sc = nacb::Mat3x3::diag(1.0 / 128, 1.0 / 128, 1.0 / 128);
  for (const auto& box : level.boxes()) {
    const nacb::Vec3d pos = global_sc * CreateVec3d(box.pos());
    const nacb::Vec3d size = global_sc * CreateVec3d(box.size());

    nacb::Vec3d v[8] = {
        {-1.000000, 1.000000, -1.000000},  {1.000000, 1.000000, -1.000000},
        {-1.000000, 1.000000, 1.000000},   {1.000000, 1.000000, 1.000000},
        {-1.000000, -1.000000, -1.000000}, {1.000000, -1.000000, -1.000000},
        {-1.000000, -1.000000, 1.000000},  {1.000000, -1.000000, 1.000000}};
    int faces[6][4] = {
        {1, 5, 7, 3}, {4, 3, 7, 8}, {8, 7, 5, 6},
        {6, 2, 4, 8}, {2, 1, 3, 4}, {6, 5, 1, 2},
    };
    nacb::Mat3x3 sc = nacb::Mat3x3::diag(size.x / 2, size.y / 2, size.z / 2);
    for (int i = 0; i < 6; ++i) {
      charts[face_i].init(2, 4);
      charts[face_i].vert[0] = sc * v[faces[i][0] - 1] + pos;
      charts[face_i].vert[1] = sc * v[faces[i][1] - 1] + pos;
      charts[face_i].vert[2] = sc * v[faces[i][2] - 1] + pos;
      charts[face_i].vert[3] = sc * v[faces[i][3] - 1] + pos;

      nacb::Vec3d e1 = charts[face_i].vert[1] - charts[face_i].vert[0];
      nacb::Vec3d e2 = charts[face_i].vert[3] - charts[face_i].vert[0];
      const int e1_max = ArgMax(e1);
      const int e2_max = ArgMax(e2);
      charts[face_i].tc[0] = nacb::Vec2f(0, 0);
      charts[face_i].tc[1] = nacb::Vec2f(size.data[e1_max], 0);
      charts[face_i].tc[2] = nacb::Vec2f(size.data[e1_max], size.data[e2_max]);
      charts[face_i].tc[3] = nacb::Vec2f(0, size.data[e2_max]);

      charts[face_i].poly[0][0] = 0;
      charts[face_i].poly[0][1] = 1;
      charts[face_i].poly[0][2] = 2;
      charts[face_i].poly[1][0] = 0;
      charts[face_i].poly[1][1] = 2;
      charts[face_i].poly[1][2] = 3;

      for (int k = 0; k < 4; ++k) {
        min_bounds = min_bounds.min(charts[face_i].vert[k]);
        max_bounds = max_bounds.max(charts[face_i].vert[k]);
      }
      face_i++;
    }
  }

  Chart chart;
  chart.init(2, 4);
  max_bounds.y = min_bounds.y;
  nacb::Vec3d delta = max_bounds - min_bounds;
  chart.vert[0] = min_bounds;
  chart.vert[1] = min_bounds + nacb::Vec3d(0, 0, delta.z);
  chart.vert[2] = min_bounds + nacb::Vec3d(delta.x, 0, delta.z);
  chart.vert[3] = min_bounds + nacb::Vec3d(delta.x, 0, 0);
  chart.tc[0] = nacb::Vec2f(0, 0);
  chart.tc[1] = nacb::Vec2f(0, delta.z);
  chart.tc[2] = nacb::Vec2f(delta.x, delta.z);
  chart.tc[3] = nacb::Vec2f(delta.x, 0);
  chart.poly[0][0] = 0;
  chart.poly[0][1] = 1;
  chart.poly[0][2] = 2;
  chart.poly[1][0] = 0;
  chart.poly[1][1] = 2;
  chart.poly[1][2] = 3;
  charts[face_i] = chart;

  scaleCharts(&(charts[0]), charts.size());
  packChartsNew(&(charts[0]), charts.size(), 1024, 1024, 4);

  nappear::Mesh mesh;
  nacb::Mat3x3 sc = global_sc.inverse();
  for (int i = 0; i < (int)charts.size(); ++i) {
    const int vert_base = (int)mesh.vert.size();
    for (int j = 0; j < charts[i].nvert; ++j) {
      mesh.vert.push_back(sc * charts[i].vert[j]);
      mesh.tvert.push_back(charts[i].tc[j]);
    }
    for (int j = 0; j < charts[i].npoly; ++j) {
      nappear::Mesh::Face face;
      for (int face_i = 0; face_i < 2; ++face_i) {
        face.vi[0] = charts[i].poly[face_i][0] + vert_base;
        face.vi[1] = charts[i].poly[face_i][1] + vert_base;
        face.vi[2] = charts[i].poly[face_i][2] + vert_base;
        face.tci[0] = charts[i].poly[face_i][0] + vert_base;
        face.tci[1] = charts[i].poly[face_i][1] + vert_base;
        face.tci[2] = charts[i].poly[face_i][2] + vert_base;
        mesh.faces.push_back(face);
      }
    }
  }
  mesh.initNormals(true);
  return mesh;
}

DEFINE_string(input_mesh, "", "Path to input object");
DEFINE_string(output_mesh, "", "Path to output mesh");
DEFINE_string(output_intermediate_mesh, "", "Path to output intermediate mesh");
DEFINE_string(level_filename, "", "Path to level filename");
DEFINE_string(output_path, "", "Path to output filename");
DEFINE_string(devicename, "CPU", "Devicename");
DEFINE_int32(num_samples, 64, "Number of samples to use");
DEFINE_int32(bake_width, 1024, "Output image width");
DEFINE_int32(bake_height, 1024, "Output image height");

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  DeviceType device_type = Device::type_from_string(FLAGS_devicename.c_str());
  auto devices = Device::available_devices(DEVICE_MASK(device_type));

  bool device_available = false;
  SessionParams session_params;
  // Strange things will happen if these parameters are not set like this...
  session_params.write_render_cb = WriteRender;
  session_params.background = true;
  session_params.progressive = true;

  if (!devices.empty()) {
    session_params.device = devices.front();
    device_available = true;
  }
  if (session_params.device.type == DEVICE_NONE || !device_available) {
    LOG(ERROR) << "Unknown device: %s\n" << FLAGS_devicename;
    return -1;
  } else if (FLAGS_num_samples < 0) {
    LOG(ERROR) << "Invalid number of samples:" << FLAGS_num_samples;
    return -2;
  } else if (FLAGS_output_path.empty()) {
    LOG(ERROR) << "No file path specified\n";
    return -1;
  }

  GLWindow win(FLAGS_bake_width, FLAGS_bake_height);
  nappear::Mesh mesh;
  if (!FLAGS_level_filename.empty()) {
    state::Level level;
    if (!Proto::ReadProto(FLAGS_level_filename, &level)) {
      LOG(ERROR) << "Error loading level:" << FLAGS_level_filename;
      return -1;
    }
    mesh = ConvertLevelToMesh(level);
    if (!FLAGS_output_intermediate_mesh.empty()) {
      mesh.saveObj(FLAGS_output_intermediate_mesh.c_str());
    }
    if (!FLAGS_output_mesh.empty()) {
      LOG(INFO) << "Removing duplicate verts and saving to "
                << FLAGS_output_mesh;
      RemoveDuplicates(mesh).saveObj(FLAGS_output_mesh.c_str());
    }
  } else {
    LOG(INFO) << "Loading mesh from:" << FLAGS_input_mesh;
    if (!mesh.readObj(FLAGS_input_mesh.c_str())) {
      LOG(ERROR) << "Unable to load mesh:" << FLAGS_input_mesh;
      return -1;
    }
  }

  if (!SessionInitAndBake(session_params, mesh, FLAGS_output_path,
                          FLAGS_bake_width, FLAGS_bake_height,
                          FLAGS_num_samples)) {
    LOG(ERROR) << "Error baking\n";
    return -1;
  }
  return 0;
}
