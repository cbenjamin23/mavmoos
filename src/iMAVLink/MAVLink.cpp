/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: MAVLink.cpp                                     */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "MAVLink.h"
#include "CopterFrame.h"
#include "OwnedFtpRoot.h"
#include "MBUtils.h"
#include <sstream>
#include <filesystem>
#include <cctype>
#include <algorithm>
#include <set>
#include <iomanip>
#include <stdexcept>

bool MAVLink::OnNewMail(MOOSMSG_LIST& mail)
{
  AppCastingMOOSApp::OnNewMail(mail);
  applyConnectionEvents();
  if(m_runtime) m_runtime->checkpoint(mavmoos::monotonicSeconds());
  for(auto& msg : mail) {
    if(m_runtime && msg.GetKey()!="APPCAST_REQ") {
      const auto now=mavmoos::monotonicSeconds();
      mavmoos::runtime::Mail event;
      event.key=msg.GetKey(); event.numeric=msg.IsDouble();
      if(event.numeric) event.value=msg.GetDouble(); else event.text=msg.GetString();
      event.source=msg.GetSource(); event.received_at=now;
      event.published_at=now-(MOOSTime()-msg.GetTime());
      m_runtime->mail(event,msg.GetTime());
    } else if(msg.GetKey() != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + msg.GetKey());
  }
  return true;
}

bool MAVLink::OnDisconnectFromServer()
{
  // MOOS invokes connection callbacks on its communication thread. Only signal the owner.
  m_disconnect_epoch.fetch_add(1,std::memory_order_release);
  return true;
}

bool MAVLink::OnConnectToServer()
{
  m_registration_pending.store(true,std::memory_order_release);
  return true;
}

void MAVLink::applyConnectionEvents()
{
  const auto epoch=m_disconnect_epoch.load(std::memory_order_acquire);
  if(epoch!=m_applied_disconnect_epoch) {
    if(m_runtime) m_runtime->disconnect(mavmoos::monotonicSeconds(),"moos_connection_lost");
    m_applied_disconnect_epoch=epoch;
  }
  if(m_registration_pending.exchange(false,std::memory_order_acq_rel)) {
    registerVariables(); m_last_capabilities.clear();
  }
}

bool MAVLink::handleConfig(const std::string& param, const std::string& value)
{
  unsigned integer = 0;
  if(param.rfind("plugin.",0)==0) {
    auto local=param.substr(7);
    const auto dot=local.find('.');
    if(dot==std::string::npos||dot==0||dot+1==local.size()||local.find('.',dot+1)!=std::string::npos||
       value.size()>4096||m_module_config.size()>32) return false;
    const auto owner=local.substr(0,dot),key=local.substr(dot+1);
    auto& batch=m_module_config[owner];
    if(batch.size()>=32) return false;
    batch.push_back({key,value});return true;
  }
  if(param=="firmware_requests") {
    if(value!="true"&&value!="false") return false;
    m_firmware_requests=value=="true"; return true;
  }
  if(param=="control_mission_execution"||param=="plane_native_autoland") {
    if(value!="true"&&value!="false")return false;
    m_plugin_config.emplace_back(param,value);return true;
  }
  if(param=="simulation_only" || param=="shadow_mode" || param=="helm_enabled") {
    if(value!="true" && value!="false") return false;
    if(param=="simulation_only") m_simulation=value=="true";
    else if(param=="shadow_mode") m_shadow=value=="true";
    else { m_helm_enabled=value=="true"; m_helm_explicit=true; }
    return true;
  }
  if(param=="auto_recovery_directory") {
    if(!m_auto_recovery_directory.empty() || value.empty() || value.front()!='/' ||
       value.size()>4096 || value.find('\0')!=std::string::npos) return false;
    m_auto_recovery_directory=value;
    return true;
  }
  if(param=="control_replay_directory") {
    if(!m_control_replay_directory.empty() || value.empty() || value.front()!='/' ||
       value.size()>4096 || value.find('\0')!=std::string::npos) return false;
    m_control_replay_directory=value;
    return true;
  }
  if(param=="control_native_actions"||param=="control_helm") {
    if(value!="true"&&value!="false") return false;
    (param=="control_native_actions"?m_control_native_override:m_control_helm_override)=value=="true";
    return true;
  }
  if(param=="sitl_pid") { if(!mavmoos::parseUnsigned(value,0,2147483647,integer)) return false; m_sitl_pid=integer; return true; }
  if(param=="plugin_directory") { m_plugin_directory=value; return !value.empty(); }
  if(param=="plugins") {
    std::istringstream names(value);
    std::set<std::string> unique;
    std::string name;
    while(std::getline(names,name,',')) {
      name=stripBlankEnds(name);
      if(name.empty()) return false;
      for(unsigned char c:name) if(!std::isalnum(c) && c!='_') return false;
      if(!unique.insert(name).second) return false;
    }
    m_plugins.assign(unique.begin(),unique.end()); return !m_plugins.empty();
  }
  if(param=="vehicle_profile") {
    const bool px4=mavmoos::runtime::px4::isProfile(value);
    const bool quadplane=value=="arduplane_quadplane";
    if(value!="arducopter" && value!="ardurover" && value!="ardurover_boat" && value!="arduplane" && !quadplane && !px4 && value!="ardusub") return false;
    m_vehicle_profile=value;
    m_config.simulator_profile=value=="ardurover_boat"?"ardurover":quadplane?"arduplane":px4?"px4":value;
    return true;
  }
  if(param=="copter_frame") {
    if(mavmoos::runtime::copterFrameType(value)==MAV_TYPE_GENERIC) return false;
    m_copter_frame=value;
    return true;
  }
  if(param=="facing_policy") {
    if(value!="course" && value!="keep" && value!="heading") return false;
    m_facing_policy=value; m_plugin_config.emplace_back(param,value); return true;
  }
  if(param=="lat_origin" || param=="long_origin" || param=="helm_source" ||
     param=="freshness_timeout" || param=="handoff_window" || param=="field_spread" || param=="nav_position_timeout" ||
     param=="command_ack_timeout" || param=="rover_return_timeout" || param=="takeoff_timeout" || param=="land_timeout" || param=="takeoff_height" ||
     param=="control_copter_adjustments" || param=="command_speed_min" || param=="control_precision_loiter" || param=="dead_man_source" ||
     param=="parameter_timeout" || param=="parameter_list_timeout" || param=="facing_heading" ||
     param=="operation_helm_enabled" || param=="operation_helm_tasks" || param=="operation_helm_adjustments" ||
     param=="operation_timeout" || param=="operation_radius" ||
     param=="operation_speed" || param=="operation_helm_source" ||
     param=="mission_timeout" || param=="mission_response_timeout" ||
     param=="status_timeout" || param=="status_publish_interval" || param=="status_text_timeout" ||
     param=="attitude_timeout" || param=="attitude_publish_interval" ||
     param=="rc_io_timeout" || param=="rc_io_publish_interval" ||
     param=="imu_timeout" || param=="imu_publish_interval" ||
     param=="distance_sensor_timeout" || param=="distance_sensor_publish_interval") {
    m_plugin_config.emplace_back(param,value); return true;
  }
  if(param=="parameter_list_mode") {
    if(value!="paced" && value!="streamed") return false;
    m_plugin_config.emplace_back(param,value); return true;
  }
  // MOOS consumes these standard scheduling settings; validate without owning them.
  if(param == "apptick" || param == "commstick") {
    double tick = 0;
    return mavmoos::parseSeconds(value, 0.1, 100, tick);
  }
  if(param == "bind_address") { m_config.bind_address = value; return true; }
  if(param == "peer_address") { m_config.peer_address = value; return true; }
  if(param == "bind_port" || param == "peer_port") {
    if(!mavmoos::parseUnsigned(value, 1, 65535, integer)) return false;
    (param == "bind_port" ? m_config.bind_port : m_config.peer_port) = integer;
    return true;
  }
  if(param == "target_system" || param == "target_component") {
    if(!mavmoos::parseUnsigned(value, 1, 255, integer)) return false;
    (param == "target_system" ? m_config.target_system : m_config.target_component) = integer;
    return true;
  }
  if(param == "landing_target_component") {
    if(m_config.landing_target_component || !mavmoos::parseUnsigned(value,1,255,integer)) return false;
    m_config.landing_target_component=integer;
    m_plugin_config.emplace_back(param,value);
    return true;
  }
  if(param == "camera_component" || param == "gimbal_component") {
    auto& component=param=="camera_component"?m_config.camera_component:m_config.gimbal_component;
    if(component || !mavmoos::parseUnsigned(value,2,255,integer)) return false;
    component=integer;
    m_plugin_config.emplace_back(param,value);
    return true;
  }
  if(param=="remote_id_component" || param=="component_metadata_component") {
    auto& component=param=="remote_id_component"?m_config.remote_id_component:m_config.component_metadata_component;
    if(component || !mavmoos::parseUnsigned(value,1,255,integer)) return false;
    component=integer;
    m_plugin_config.emplace_back(param,value);
    return true;
  }
  if(param == "expected_type" || param == "expected_autopilot") {
    if(!mavmoos::parseUnsigned(value, 0, 255, integer)) return false;
    if(param=="expected_type") {
      m_config.expected_type=integer;
      m_expected_type_explicit=true;
    } else {m_config.expected_autopilot=integer;m_expected_autopilot_explicit=true;}
    return true;
  }
  if(param == "max_datagrams_per_iter") {
    if(!mavmoos::parseUnsigned(value, 1, 128, integer)) return false;
    m_config.max_datagrams_per_iter = integer;
    return true;
  }
  if(param == "max_queue_age")
    return mavmoos::parseSeconds(value,0.01,2,m_config.max_queue_age);
  if(param == "heartbeat_timeout")
    return mavmoos::parseSeconds(value, 0.01, 60, m_config.heartbeat_timeout);
  if(param == "position_timeout")
    return mavmoos::parseSeconds(value, 0.01, 60, m_config.position_timeout);
  if(param == "landed_timeout")
    return mavmoos::parseSeconds(value, 0.01, 60, m_config.landed_timeout);
  if(param == "estimator_timeout")
    return mavmoos::parseSeconds(value, 0.01, 60, m_config.estimator_timeout);
  if(param == "publish_interval")
    return mavmoos::parseSeconds(value, 0.01, 60, m_config.publish_interval);
  return false;
}

bool MAVLink::OnStartUp()
{
  if(!AppCastingMOOSApp::OnStartUp())
    m_config_error = true;
  STRING_LIST params;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), params)) {
    reportConfigWarning("No config block found for " + GetAppName());
    m_config_error = true;
  }
  for(auto line : params) {
    const auto original = line;
    const auto raw_param = stripBlankEnds(biteStringX(line, '='));
    const auto param = tolower(raw_param);
    const auto value = stripBlankEnds(line);
    if(param.rfind("plugin.",0)==0&&param!=raw_param) {
      reportConfigWarning("module settings require lowercase canonical names: "+original);
      m_config_error=true;continue;
    }
    if(!handleConfig(param, value)) {
      reportUnhandledConfigWarning(original);
      m_config_error = true;
    }
  }
  const bool copter=m_vehicle_profile=="arducopter";
  const bool explicit_copter_frame=!m_copter_frame.empty();
  const unsigned copter_type=mavmoos::runtime::copterFrameType(explicit_copter_frame?m_copter_frame:"quad");
  const bool non_quad_copter=copter && copter_type!=MAV_TYPE_QUADROTOR;
  if(!copter && explicit_copter_frame) {
    reportConfigWarning("copter_frame requires the arducopter vehicle_profile");
    m_config_error=true;
  }
  const bool plane=m_vehicle_profile=="arduplane";
  const bool quadplane=m_vehicle_profile=="arduplane_quadplane";
  const bool sub=m_vehicle_profile=="ardusub";
  const bool px4=mavmoos::runtime::px4::isProfile(m_vehicle_profile);
  const bool px4_fixed_wing=m_vehicle_profile==mavmoos::runtime::px4::fixed_wing_profile;
  const bool px4_vtol=m_vehicle_profile==mavmoos::runtime::px4::standard_vtol_profile;
  const bool px4_rover=m_vehicle_profile==mavmoos::runtime::px4::rover_profile;
  const int expected_autopilot=px4?MAV_AUTOPILOT_PX4:MAV_AUTOPILOT_ARDUPILOTMEGA;
  const int expected_type=(quadplane||px4_vtol)?MAV_TYPE_VTOL_FIXEDROTOR:px4_rover?MAV_TYPE_GROUND_ROVER:
    (plane||px4_fixed_wing)?MAV_TYPE_FIXED_WING:copter?int(copter_type):px4?MAV_TYPE_QUADROTOR:sub?MAV_TYPE_SUBMARINE:
    (m_vehicle_profile=="ardurover_boat"?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER);
  if((!copter || explicit_copter_frame) && !m_expected_type_explicit) m_config.expected_type=expected_type;
  if((px4 || explicit_copter_frame) && !m_expected_autopilot_explicit) m_config.expected_autopilot=expected_autopilot;
  if((!copter || explicit_copter_frame) &&
     (m_config.expected_type!=expected_type || m_config.expected_autopilot!=expected_autopilot)) {
    reportConfigWarning("vehicle_profile and copter_frame require their matching firmware heartbeat identity");
    m_config_error=true;
  }
  const auto errors = mavmoos::validateConfig(m_config);
  for(const auto& error : errors)
    reportConfigWarning(error);
  const auto require_component=[this](const char* plugin,const char* setting,uint8_t component) {
    const bool selected=std::find(m_plugins.begin(),m_plugins.end(),plugin)!=m_plugins.end();
    if(selected != (component != 0)) {
      reportConfigWarning(std::string(plugin)+" and "+setting+" must be configured together");
      m_config_error=true;
    }
  };
  require_component("landing_target","landing_target_component",m_config.landing_target_component);
  require_component("camera_status","camera_component",m_config.camera_component);
  require_component("gimbal_status","gimbal_component",m_config.gimbal_component);
  require_component("remote_id","remote_id_component",m_config.remote_id_component);
  require_component("component_metadata","component_metadata_component",m_config.component_metadata_component);
  m_telemetry = std::make_unique<mavmoos::Telemetry>(m_config, m_session);
  if(!copter && (!px4 || px4_fixed_wing || px4_vtol || px4_rover) && m_facing_policy!="course") {
    reportConfigWarning("Rover, fixed-wing and ArduSub profiles support course guidance"); m_config_error=true;
  }
  if(m_simulation && (m_config.bind_address!="127.0.0.1" || m_config.peer_address!="127.0.0.1" ||
     !mavmoos::UdpReceiver::isSitlEndpoint(m_sitl_pid,m_config.peer_port,m_config.simulator_profile) || m_config.target_system!=1 || m_config.target_component!=1 ||
     m_config.expected_type!=expected_type || m_config.expected_autopilot!=expected_autopilot ||
     (px4 && (m_config.bind_port!=14540 || m_config.peer_port!=14580)))) {
    reportConfigWarning("simulation_only requires loopback, exact profile-native PID/socket, target 1/1 and matching firmware identity");
    m_config_error=true;
  }
  if(m_config_error || !errors.empty()) {
    m_reason = "invalid_or_missing_configuration";
  } else if(!m_receiver.open(m_config)) {
    m_state = "SOCKET_ERROR";
    m_reason = "udp_open_failed";
    reportConfigWarning("UDP open failed; errno=" + std::to_string(m_receiver.stats().last_errno));
  } else {
    m_state = m_shadow?"SHADOW":m_simulation?"SITL_CONTROL":"RX_ONLY";
    m_reason = "waiting_for_heartbeat";
    reportEvent(m_state+": fixed peer; explicit simulation effects only");
    m_runtime=std::make_unique<mavmoos::runtime::Runtime>(
      [this](const std::string& key,const std::string& text,double value,bool numeric) {
        if(numeric) Notify(key,value); else Notify(key,text);
      },[](const uint8_t*,std::size_t) {return false;},m_simulation,m_shadow,
      [] { return mavmoos::monotonicSeconds(); },m_config.expected_type,m_config.expected_autopilot,true);
    m_runtime->setFirmwareRequestsEnabled(m_firmware_requests);
    m_runtime->setBoundedSend([this](const uint8_t* data,std::size_t size,double expires,
                                   const mavmoos::runtime::Runtime::SendEligibility& eligible) {
      const auto current=[this,&eligible](double at) {
        return m_state=="SITL_CONTROL" &&
          m_applied_disconnect_epoch==m_disconnect_epoch.load(std::memory_order_acquire) && eligible(at);
      };
      return m_receiver.sendSimulationFrame(data,size,m_sitl_pid,expires,current);
    });
    if(m_plugin_directory.empty()) m_plugin_directory=MAVMOOS_PLUGIN_DIRECTORY;
    const auto fail_startup=[this](const std::string& reason) {
      reportConfigWarning(reason);m_runtime.reset();m_loaded_plugins.clear();m_helm_enabled=false;
      m_receiver.close();m_state="CONFIG_ERROR";m_reason="plugin_configuration_failed";
    };
    try {
      const auto selected=mavmoos::runtime::selectModules(m_plugins,m_simulation,m_helm_enabled,m_helm_explicit,
                                                         m_control_native_override,m_control_helm_override,m_vehicle_profile);
      m_plugins=selected.modules;
      for(const auto& notice:selected.notices) reportEvent(notice);
      for(const auto& name:m_plugins) {
        const auto path=std::filesystem::path(m_plugin_directory)/("mavmoos_"+name+MAVMOOS_PLUGIN_SUFFIX);
        if(!m_runtime->load(path.string(),name))
          throw std::runtime_error("plugin load failed: "+path.string()+": "+m_runtime->lastConfigurationError());
      }
      if(!copter) m_runtime->configure("vehicle_profile",m_vehicle_profile);
      if(explicit_copter_frame) m_runtime->configure("copter_frame",m_copter_frame);
      m_runtime->configure("control_native_actions",selected.native_actions?"true":"false");
      m_runtime->configure("control_helm",selected.helm?"true":"false");
      double origin=0;
      if(m_MissionReader.GetValue("LatOrigin",origin)) m_runtime->configure("lat_origin",std::to_string(origin));
      if(m_MissionReader.GetValue("LongOrigin",origin)) m_runtime->configure("long_origin",std::to_string(origin));
      for(const auto& setting:m_plugin_config) {
        if(setting.first=="parameter_list_mode" && setting.second=="streamed")
          if(!m_runtime->streamedParameterBenchCompatible())
            throw std::invalid_argument("streamed parameter lists require bench-compatible capabilities without vehicle control");
        m_runtime->configure(setting.first,setting.second);
        if(setting.first=="control_mission_execution"&&setting.second=="true"&&
           !m_runtime->hasCapability("control.mission.start"))
          throw std::invalid_argument("mission execution requires an enabled native mission control provider");
      }
      for(const auto& setting:m_plugin_config) {
        const auto& [key,value]=setting;
        const bool unsupported =
          (key=="control_copter_adjustments"&&value=="true"&&!m_runtime->hasCapability("control.guided.speed_adjustment")) ||
          (key=="control_precision_loiter"&&value=="true"&&!m_runtime->hasCapability("control.precision_loiter.enable")) ||
          (key=="dead_man_source"&&!m_runtime->hasCapability("control.return.dead_man_rtl")) ||
          (key=="operation_helm_tasks"&&!m_runtime->hasCapability("control.operations.helm.survey")&&
            !m_runtime->hasCapability("control.operations.helm.voronoi")&&
            !m_runtime->hasCapability("control.operations.helm.return")&&
            !m_runtime->hasCapability("control.operations.helm.waypoint")) ||
          (key=="operation_helm_adjustments"&&value=="true"&&!m_runtime->hasCapability("control.operations.helm.adjustment"));
        if(unsupported) throw std::invalid_argument(key+" requires the corresponding enabled Copter facet and configuration");
        if(key=="plane_native_autoland"&&value=="true"&&
           (!plane||!m_runtime->hasCapability("control.land.native_autoland")))
          throw std::invalid_argument("Plane native AUTOLAND requires the enabled conventional mission provider");
      }
      m_runtime->configure("shadow_mode",m_shadow?"true":"false");
      // Keep old Rover mission files and their already recorded IDs usable.
      if(auto rover_settings=m_module_config.find("ardurover_control");rover_settings!=m_module_config.end() &&
         m_simulation && !m_shadow && (m_vehicle_profile=="ardurover" || m_vehicle_profile=="ardurover_boat")) {
        auto& settings=rover_settings->second;
        for(const auto& setting:settings) if(setting.key=="native_replay_directory") {
          if(!m_control_replay_directory.empty() || setting.value.empty())
            throw std::invalid_argument("specify one control replay directory");
          m_control_replay_directory=setting.value;
        }
        settings.erase(std::remove_if(settings.begin(),settings.end(),[](const auto& setting) {
          return setting.key=="native_replay_directory";
        }),settings.end());
        if(settings.empty()) m_module_config.erase(rover_settings);
      }
      m_runtime->configureModules(m_module_config);
      m_loaded_plugins=m_runtime->moduleNames();
      m_helm_enabled=m_runtime->hasCapability("control.navigation.helm");
      // Recheck policies against final descriptors after every startup batch.
      if(!m_runtime->controlProfile().empty()&&m_runtime->controlProfile()!=m_vehicle_profile)
        throw std::invalid_argument("vehicle-control provider does not match selected vehicle_profile");
      if(non_quad_copter && m_simulation && !m_shadow) {
        const std::set<std::string> loaded(m_loaded_plugins.begin(),m_loaded_plugins.end());
        const bool other=std::any_of(loaded.begin(),loaded.end(),[](const auto& name) {
          return name!="telemetry" && name!="arducopter_control" && name!="parameters" && name!="missions";
        });
        if(other || !loaded.count("telemetry") || m_runtime->controlProfile()!="arducopter" ||
           (m_runtime->hasCapability("control.mission.start") && !loaded.count("missions")))
          throw std::invalid_argument("Copter hexa and octa require dedicated telemetry, control and verified mission composition");
      }
      if(plane && m_simulation && !m_shadow && m_runtime->controlProfile()!="arduplane")
        throw std::invalid_argument("Plane SITL effects require the conventional control provider");
      if(quadplane && m_simulation && !m_shadow) {
        const std::set<std::string> required={"telemetry","arduquadplane_control","missions"};
        const std::set<std::string> loaded(m_loaded_plugins.begin(),m_loaded_plugins.end());
        if(loaded!=required || selected.helm!=m_helm_enabled || m_firmware_requests ||
           !m_runtime->hasCapability("control.mission.start"))
          throw std::invalid_argument("QuadPlane effects require the dedicated mission composition and matching helm facet");
      }
      if(px4 && !px4_rover && m_simulation && !m_shadow && (m_loaded_plugins.size()!=2 ||
         std::find(m_loaded_plugins.begin(),m_loaded_plugins.end(),"telemetry")==m_loaded_plugins.end() ||
         m_runtime->controlProfile()!=m_vehicle_profile || !m_helm_enabled ||
         (selected.native_actions!=m_runtime->hasCapability("control.px4.native.arm")) || m_firmware_requests))
        throw std::invalid_argument("PX4 effects require a dedicated stock-helm SITL profile");
      if(px4_rover && m_simulation && !m_shadow) {
        const bool other=std::any_of(m_loaded_plugins.begin(),m_loaded_plugins.end(),[](const auto& name) {
          return name!="telemetry" && name!="px4_rover_control" && name!="missions";
        });
        const auto contains=[this](const char* name) {
          return std::find(m_loaded_plugins.begin(),m_loaded_plugins.end(),name)!=m_loaded_plugins.end();
        };
        if(other || !contains("telemetry") || m_runtime->controlProfile()!=m_vehicle_profile ||
           !m_helm_enabled || m_firmware_requests ||
           (m_runtime->hasCapability("control.mission.start") && !contains("missions")))
          throw std::invalid_argument("PX4 Ackermann requires dedicated telemetry, control and verified mission composition");
      }
      if(sub && m_simulation && !m_shadow && (m_loaded_plugins.size()!=2 ||
         std::find(m_loaded_plugins.begin(),m_loaded_plugins.end(),"telemetry")==m_loaded_plugins.end() ||
         m_runtime->controlProfile()!="ardusub" || !m_helm_enabled || m_firmware_requests))
        throw std::invalid_argument("ArduSub effects require the dedicated stock-helm underwater SITL profile");
      const auto configured_profile=m_runtime->controlProfile();
      if(m_simulation && !m_shadow && !configured_profile.empty()) {
        std::string community;
        m_MissionReader.GetValue("Community",community);
        const auto scope=community+"/"+GetAppName();
        const auto route=scope+"\n"+configured_profile+"\n"+std::to_string(m_config.expected_type)+"/"+
          std::to_string(m_config.expected_autopilot)+"/"+std::to_string(m_config.target_system)+"/"+
          std::to_string(m_config.target_component)+"\n";
        const bool create=m_control_replay_directory.empty();
        if(create) {
          std::string name=".mavmoos-control-replay-";
          for(unsigned char c:scope) {
            constexpr char hex[]="0123456789abcdef";
            name+=hex[c>>4];name+=hex[c&15];
          }
          m_control_replay_directory=(std::filesystem::canonical(std::filesystem::absolute(GetMissionFileName()).parent_path())/name).string();
        }
        try { m_runtime->configureControlReplay(m_control_replay_directory,route,create); }
        catch(const std::invalid_argument& error) {
          reportConfigWarning(error.what());
          Notify("MAVMOOS_CONTROL_REPLAY_STATUS","state=unavailable");
        }
      }
      const bool rover_mission=(configured_profile=="ardurover" || configured_profile=="ardurover_boat") &&
        m_runtime->hasCapability("control.mission.start");
      if(rover_mission && std::find(m_loaded_plugins.begin(),m_loaded_plugins.end(),"missions")==m_loaded_plugins.end())
        throw std::invalid_argument("ArduRover mission execution requires the missions provider");
      if(m_runtime->hasCapability("control.mission.start")&&!selected.native_actions)
        throw std::invalid_argument("mission execution requires enabled native control");
      if(m_runtime->hasCapability("control.mission.start") && m_auto_recovery_directory.empty())
        throw std::invalid_argument("mission execution requires auto_recovery_directory");
      if(plane&&m_runtime->hasCapability("control.mission.start")&&
         std::find(m_loaded_plugins.begin(),m_loaded_plugins.end(),"missions")==m_loaded_plugins.end())
        throw std::invalid_argument("Plane mission execution requires verified mission transfer");
      if(!m_auto_recovery_directory.empty()) {
        if(!m_simulation || m_shadow ||
           (configured_profile!="arducopter" && configured_profile!=mavmoos::runtime::px4::rover_profile &&
            !rover_mission && !quadplane && !plane))
          throw std::invalid_argument("auto_recovery_directory requires an owned mission control profile");
        m_runtime->configureAutoRecovery(m_auto_recovery_directory,m_session);
      }
      for(const auto& setting:m_plugin_config)
        if(setting.first=="parameter_list_mode"&&setting.second=="streamed"&&
           !m_runtime->streamedParameterBenchCompatible())
          throw std::invalid_argument("streamed parameter lists require bench-compatible capabilities without vehicle control");
      if(m_runtime->hasCapability("mavlink_files.get") && m_simulation && !m_shadow) {
        std::string reason;
        auto root=mavmoos::OwnedFtpRoot::capture(m_sitl_pid,&reason);
        if(!root) throw std::invalid_argument(reason);
        const auto owned=std::make_shared<mavmoos::OwnedFtpRoot>(std::move(*root));
        m_runtime->enableFileTransferBench([owned](auto action,const auto& path) {
          using mavmoos::runtime::FileTransferAction;
          if(action==FileTransferAction::List) return owned->allowsList();
          if(action==FileTransferAction::OpenReadOnly) return owned->allowsOpen(path);
          return owned->stillBound();
        });
      }
      if(m_runtime->hasCapability(mavmoos::runtime::external_navigation_capability) && m_simulation && !m_shadow)
        m_runtime->enableExternalNavigationBench();
      if(m_simulation && !m_shadow) {
        const auto camera=m_runtime->hasCapability("camera.capture.single")?m_config.camera_component:0;
        const auto gimbal=m_runtime->hasCapability("gimbal.control.point")?m_config.gimbal_component:0;
        if(camera || gimbal) {
          m_runtime->enablePayloadBench(camera,gimbal);
          m_telemetry->enablePayloadResponses(camera!=0,gimbal!=0);
        }
      }
      // Inspect the final subscriptions before releasing the startup effect guard.
      const auto subscriptions=m_runtime->subscriptions();
      m_runtime->finishStartup();
      AppCastingMOOSApp::RegisterVariables();
      for(const auto& variable:subscriptions) Register(variable,0);
      m_telemetry->setTimedMessageHandler([this](const mavlink_message_t& msg,double effective_at,double read_at) {
        m_runtime->message(msg,effective_at,read_at);
      });
    } catch(const std::exception& e) {
      fail_startup(e.what());
    } catch(...) { fail_startup("module startup threw an unknown exception"); }
  }
  if(!m_runtime) registerVariables();
  return true;
}

void MAVLink::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  if(m_runtime) for(const auto& variable:m_runtime->subscriptions()) Register(variable,0);
}

bool MAVLink::Iterate()
{
  AppCastingMOOSApp::Iterate();
  applyConnectionEvents();
  if(m_runtime) m_runtime->checkpoint(mavmoos::monotonicSeconds());
  if(m_receiver.isOpen())
    m_receiver.poll(*m_telemetry);
  const double now = mavmoos::monotonicSeconds();
  if(m_runtime) m_runtime->tick(now);
  if(m_runtime) {
    const auto catalog=m_runtime->capabilities();
    if(catalog!=m_last_capabilities) { Notify("MAVMOOS_CAPABILITIES",catalog); m_last_capabilities=catalog; }
  }
  if(m_state == "RX_ONLY" || m_state == "SHADOW" || m_state == "SITL_CONTROL")
    m_reason = m_telemetry->heartbeatFresh(now) ? "observing" :
      (m_telemetry->identityState() == "MISMATCH" ? "identity_mismatch" : "heartbeat_unavailable_or_stale");
  if(m_last_publish < 0 || now - m_last_publish >= m_config.publish_interval) {
    postObservations(now);
    m_last_publish = now;
  }
  AppCastingMOOSApp::PostReport();
  return true;
}

std::string MAVLink::status(double now) const
{
  const auto& rx = m_receiver.stats();
  const auto& decode = m_telemetry->stats();
  std::ostringstream out;
  out << std::setprecision(16);
  out << "schema=1,session=" << m_session << ",state=" << m_state << ",reason=" << m_reason
      << (m_state=="SITL_CONTROL"?",tx_capability=simulation_only,tx_enabled=true":",tx_capability=simulation_only,tx_enabled=false,mavlink_tx_attempts=0,mavlink_tx_accepted=0")
      << ",helm=" << (m_helm_enabled?"enabled":"disabled") << ",identity=" << m_telemetry->identityState()
      << ",loaded_modules=" << m_loaded_plugins.size()
      << ",max_datagrams_per_iter=" << m_config.max_datagrams_per_iter
      << ",max_queue_age_s=" << m_config.max_queue_age
      << ",max_datagram_bytes=" << mavmoos::UdpReceiver::max_datagram_bytes
      << ",heartbeat_timeout_s=" << m_config.heartbeat_timeout
      << ",position_timeout_s=" << m_config.position_timeout
      << ",landed_timeout_s=" << m_config.landed_timeout
      << ",estimator_timeout_s=" << m_config.estimator_timeout
      << ",publish_interval_s=" << m_config.publish_interval
      << ",heartbeat_fresh=" << (m_telemetry->heartbeatFresh(now) ? "true" : "false")
      << ",rx_datagrams=" << rx.datagrams << ",rx_bytes_copied=" << rx.bytes
      << ",wrong_peer=" << rx.wrong_peer << ",truncated=" << rx.truncated
      << ",rx_errors=" << rx.errors << ",last_errno=" << rx.last_errno
      << ",budget_hits=" << rx.budget_hits << ",frames=" << decode.frames
      << ",rx_age_basis=QUEUE_EMPTY_FLOOR,rx_polls=" << rx.polls
      << ",empty_events=" << rx.empty_events
      << ",queue_stale_dropped=" << rx.queue_stale_dropped
      << ",queue_clock_dropped=" << rx.queue_clock_dropped
      << ",queue_draining=" << (rx.draining?"true":"false")
      << ",queue_epoch_at=" << rx.queue_epoch_at << ",last_read_at=" << rx.last_read_at
      << ",last_effective_at=" << rx.last_effective_at
      << ",last_queue_age_upper_s=" << rx.last_queue_age_upper_s
      << ",max_queue_age_upper_s=" << rx.max_queue_age_upper_s
      << ",last_poll_gap_s=" << rx.last_poll_gap_s << ",max_poll_gap_s=" << rx.max_poll_gap_s
      << ",invalid_timestamps=" << decode.invalid_timestamps
      << ",bad_frames=" << decode.bad_frames << ",incomplete_datagrams=" << decode.incomplete_datagrams
      << ",empty_or_noise=" << decode.empty_or_noise << ",wrong_target=" << decode.wrong_target
      << ",unsupported_flags=" << decode.unsupported_flags
      << ",unsupported_messages=" << decode.unsupported_messages
      << ",unbound_reports=" << decode.unbound_reports << ",duplicates=" << decode.duplicates
      << ",observations=" << decode.observations << ",invalid_observations=" << decode.invalid_observations;
  return out.str();
}

void MAVLink::postObservations(double now)
{
  for(auto family : mavmoos::Telemetry::families())
    Notify(mavmoos::Telemetry::variable(family), m_telemetry->record(family, now));
  Notify("MAVMOOS_STATUS", status(now));
}

bool MAVLink::buildReport()
{
  const double now = mavmoos::monotonicSeconds();
  m_msgs << "MAVMOOS modular bridge (receive-only default; isolated simulation control)\n"
         << "Bind: " << m_config.bind_address << ':' << m_config.bind_port
         << "  Peer: " << m_config.peer_address << ':' << m_config.peer_port << '\n'
         << "Target: " << unsigned(m_config.target_system) << '/' << unsigned(m_config.target_component)
         << "  Expected type/autopilot: " << m_config.expected_type << '/' << m_config.expected_autopilot << '\n'
         << "RX limit: " << m_config.max_datagrams_per_iter << " datagrams/Iterate, "
         << mavmoos::UdpReceiver::max_datagram_bytes << " bytes/datagram\n";
  if(m_telemetry) {
    m_msgs << status(now) << "\n\n";
    for(auto family : mavmoos::Telemetry::families())
      m_msgs << mavmoos::Telemetry::variable(family) << ":\n" << m_telemetry->record(family, now) << '\n';
  }
  return true;
}
