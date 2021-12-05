#include "src/proto/level.pb.h"
#include "src/world.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <nimage/image.h>

DEFINE_string(filename, "", "Path to input filename");
DEFINE_bool(build_map, false, "Build access map");

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  state::Level level;
  if (!Proto::ReadProto<state::Level>(FLAGS_filename, &level)) {
    LOG(ERROR) << "Error loading level.";
    return -1;
  }
  World world;
  if (!world.LoadFromProto(level)) {
    LOG(ERROR) << "Error loading level.";
    return -1;
  }
  std::cout << "Successfully loaded:" << FLAGS_filename << std::endl;

  if (FLAGS_build_map) {
    nacb::Imagef image = world.BuildAccessibilityMap(16);
    image.write("/tmp/map.png");

    for (int i = 0; i < 100; ++i) {
      for (int j = 0; j < 10; ++j) {
        world.Step(1.0 / 60.0);
      }
      char name[1024];
      nacb::Imagef image = world.BuildAccessibilityMap(16);
      snprintf(name, sizeof(name), "/tmp/image-%04d.png", i);
      image.write(name);
    }
  }
  return 0;
}
