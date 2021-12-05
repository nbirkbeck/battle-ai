#ifndef _LEVEL_H_
#define _LEVEL_H_ 1

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "src/proto/vec3.pb.h"
#include <nmath/vec3.h>

inline nacb::Vec3d CreateVec3d(const state::Vec3& v) {
  return nacb::Vec3d(v.x(), v.y(), v.z());
}

inline void SetVec3(state::Vec3* v, const nacb::Vec3d& value) {
  v->set_x(value.x);
  v->set_y(value.y);
  v->set_z(value.z);
}

class Proto {
public:
  template <class T>
  static bool ReadProto(const std::string& filename, T* proto) {
    using namespace google::protobuf;
    if (filename.size() == 0) {
      LOG(ERROR) << "Need at least one argument (the input file)";
      return false;
    }
    const int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
      LOG(ERROR) << "Unable to open input file: " << filename;
      return false;
    }
    io::FileInputStream fstream(fd);
    if (!TextFormat::Parse(&fstream, proto)) {
      LOG(ERROR) << "Unable to parse input:" << filename;
      return false;
    }
    return true;
  }
};

#endif // _LEVEL_H_
