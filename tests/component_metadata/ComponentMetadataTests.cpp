#include "PassiveRig.h"
#include <algorithm>
#include <filesystem>

using namespace mavmoos::test;
namespace fs=std::filesystem;
namespace {
constexpr char uri[]="mftp:///component_metadata/general.json";

void announce(PassiveRig& rig,double at,uint32_t boot,uint32_t crc,uint8_t component=196,
              std::string_view url=uri)
{
  mavlink_component_metadata_t packet{};
  packet.time_boot_ms=boot;
  packet.file_crc=crc;
  std::copy(url.begin(),url.end(),packet.uri);
  rig.receive(rig.encode(packet,mavlink_msg_component_metadata_encode_status,1,component),at);
}

void tests(char** argv)
{
  PassiveRig rig(argv[1]);
  const fs::path directory=argv[2];
  const auto general=directory/"general.json";
  const auto install=[&](const char* name) {
    fs::copy_file(directory/name,general,fs::copy_options::overwrite_existing);
  };
  const auto crc=[&](int index) {return static_cast<uint32_t>(std::stoul(argv[index]));};
  const auto a=crc(3),b=crc(4),new_schema=crc(5),duplicate=crc(6);
  const std::vector<ModuleSetting> config{
    {"mirror_directory",directory.string()},{"sample_timeout",".45"},{"publish_interval",".05"}};

  require(rig.plugin->configureModule(config).status==ModuleConfigStatus::Invalid,
    "metadata worked without a selected source");
  rig.plugin->configure("component_metadata_component","196");
  require(rig.plugin->configureModule({}).status==ModuleConfigStatus::Invalid,"mirror was optional");
  require(rig.plugin->configureModule(config).status==ModuleConfigStatus::Accepted,"private mirror failed configuration");

  install("a");
  announce(rig,100.02,1000,a);
  rig.capture("prebind",100.06,1);
  rig.heartbeat(100.16);
  announce(rig,100.17,1000,a,197);
  rig.capture("wrong_component",100.19,1);
  announce(rig,100.22,1000,a);
  rig.capture("verified",100.31,1);

  rig.heartbeat(100.57);
  announce(rig,100.63,1000,a);
  rig.capture("repeat_stale",100.72,1);

  install("b");
  announce(rig,100.79,1100,a);
  rig.capture("checksum",100.86,1);
  announce(rig,100.90,1200,b);
  rig.capture("updated",100.99,1);
  announce(rig,101.03,1200,a);
  rig.capture("conflict",101.12,1);
  announce(rig,101.14,1199,b);
  rig.capture("regressed",101.175,1);

  install("version");
  announce(rig,101.18,1300,new_schema);
  rig.capture("version",101.24,1);
  install("large");
  announce(rig,101.30,1400,1);
  rig.capture("size",101.37,1);

  fs::remove(general);
  fs::create_symlink(directory/"a",general);
  announce(rig,101.41,1500,a);
  rig.capture("symlink",101.49,1);
  fs::remove(general);

  install("duplicate");
  announce(rig,101.53,1600,duplicate);
  rig.capture("duplicate_key",101.60,1);
  announce(rig,101.65,1700,a,196,"mftp:///other/general.json");
  rig.capture("unsupported_uri",101.73,1);

  rig.host.state.heartbeat_valid=false;
  rig.capture("cleared",101.86,1);
  rig.heartbeat(101.98);
  install("a");
  announce(rig,102.02,5,a);
  rig.capture("rebound",102.12,1);
}
}

int main(int argc,char** argv)
{
  try {require(argc==7,"pass module, fixture directory and four CRCs");tests(argv);return 0;}
  catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
