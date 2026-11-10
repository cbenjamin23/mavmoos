#include "ModuleSelection.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void check(bool good,const char* why){if(!good)throw std::runtime_error(why);}
bool has(const ModuleSelection& s,const std::string& name){return std::find(s.modules.begin(),s.modules.end(),name)!=s.modules.end();}
template<class F> void rejects(F operation){bool rejected=false;try{operation();}catch(const std::invalid_argument&){rejected=true;}check(rejected,"invalid selection accepted");}
}
int main()
{
  try {
    auto s=selectModules({},false,false);
    check(s.modules==std::vector<std::string>{"telemetry"}&&!s.native_actions&&!s.helm,"RX defaults broadened");
    s=selectModules({},false,true);check(s.modules.size()==2&&has(s,"arducopter_control")&&!s.native_actions&&s.helm,"shadow helm defaults changed");
    s=selectModules({},true,false);check(s.modules.size()==3&&has(s,"parameters")&&s.native_actions&&!s.helm,"native simulation defaults changed");
    s=selectModules({},true,true);check(s.modules.size()==3&&s.native_actions&&s.helm,"combined defaults absent");
    s=selectModules({"commands"},true,true);check(s.modules==std::vector<std::string>{"arducopter_control"}&&s.native_actions&&!s.helm&&!s.notices.empty(),"commands-only alias enabled helm");
    s=selectModules({"helm"},true,false,true);check(!s.native_actions&&s.helm,"helm-only alias enabled native command surface");
    s=selectModules({"commands","parameters","helm","telemetry"},true,false);
    check(s.modules.size()==3&&s.native_actions&&s.helm,"legacy pair did not collapse to one provider");
    s=selectModules({"arducopter_control"},true,false);check(s.native_actions&&s.helm,"canonical complete control defaults absent");
    s=selectModules({"arducopter_control"},true,false,true);check(s.native_actions&&!s.helm,"explicit helm disable ignored");
    s=selectModules({"arducopter_control"},true,true,false,false,true);check(!s.native_actions&&s.helm,"explicit control facets ignored");
    s=selectModules({"parameters"},true,true);check(s.modules.size()==1&&!s.native_actions&&!s.helm,"parameter-only loaded control");
    rejects([]{selectModules({"commands","arducopter_control"},true,true);});
    rejects([]{selectModules({"helm","arducopter_control"},true,true);});
    rejects([]{selectModules({"parameters","parameters"},true,true);});
    rejects([]{selectModules({"../control"},true,true);});
    rejects([]{selectModules({"telemetry"},false,false,false,true,{});});
    s=selectModules({},true,true,false,{},{},"ardurover");
    check(has(s,"ardurover_control")&&!has(s,"arducopter_control")&&s.native_actions&&s.helm,"Rover implicit provider selection");
    s=selectModules({"ardurover_control"},true,false,false,{},{},"ardurover");
    check(s.modules==std::vector<std::string>{"ardurover_control"}&&s.native_actions&&s.helm,"Rover canonical facets");
    s=selectModules({},false,true,false,{},{},"ardurover");
    check(has(s,"ardurover_control")&&!s.native_actions&&s.helm,"Rover shadow facets");
    s=selectModules({},true,true,false,{},{},"ardurover_boat");
    check(has(s,"ardurover_control")&&!has(s,"arducopter_control")&&s.native_actions&&s.helm,"boat uses Rover provider");
    s=selectModules({"ardurover_control"},false,true,false,{},{},"ardurover_boat");
    check(s.modules==std::vector<std::string>{"ardurover_control"}&&s.helm,"boat canonical facets");
    rejects([]{selectModules({"arducopter_control"},true,true,false,{},{},"ardurover_boat");});
    rejects([]{selectModules({"ardurover_control"},true,true);});
    rejects([]{selectModules({"arducopter_control"},true,true,false,{},{},"ardurover");});
    rejects([]{selectModules({"commands"},true,true,false,{},{},"ardurover");});
    s=selectModules({},false,false,false,{},{},"arduplane");
    check(s.modules==std::vector<std::string>{"telemetry"}&&!s.helm&&!s.native_actions,"Plane receive-only default changed");
    s=selectModules({},true,true,false,{},{},"arduplane");
    check(has(s,"arduplane_control")&&!has(s,"ardurover_control")&&s.native_actions&&s.helm,"Plane provider selection");
    s=selectModules({"arduplane_control"},false,true,false,{},{},"arduplane");
    check(s.modules==std::vector<std::string>{"arduplane_control"}&&s.helm,"Plane shadow provider selection");
    rejects([]{selectModules({"arduplane_control"},true,true);});
    rejects([]{selectModules({"ardurover_control","arduplane_control"},true,true,false,{},{},"arduplane");});
    rejects([]{selectModules({"helm"},true,true,false,{},{},"arduplane");});
    const std::vector<std::string> quadplane={"telemetry","arduquadplane_control","missions"};
    s=selectModules(quadplane,true,false,false,{},{},"arduplane_quadplane");
    check(s.modules==quadplane&&s.native_actions&&!s.helm,"QuadPlane native-only default changed");
    s=selectModules(quadplane,true,false,false,{},true,"arduplane_quadplane");
    check(s.modules==quadplane&&s.native_actions&&s.helm,"QuadPlane opt-in helm must coexist with native actions");
    rejects([]{selectModules({"arduplane_control"},true,true,false,{},{},"arduplane_quadplane");});
    rejects([]{selectModules({},true,true,false,{},{},"anything");});
    std::cout<<"PASS module selection: default modes, exact legacy facets, Rover profiles and invalid combinations\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
