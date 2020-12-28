#ifndef _LEVEL_H_
#define _LEVEL_H_ 1

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <fcntl.h>

class Level {
 public:
  static bool ReadProto(const std::string& filename, state::Level* level) {
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
    if (!TextFormat::Parse(&fstream, level)) {
      LOG(ERROR) << "Unable to parse input as a level:" << filename;
      return false;
    }
    LOG(INFO) << "Loaded level.";
    LOG(INFO) << level->DebugString();
    return true;
  }
};

#endif  // _LEVEL_H_
