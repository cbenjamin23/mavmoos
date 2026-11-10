/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ProtocolSupport.h                               */
/*    DATE: November 4, 2026                                */
/************************************************************/
#pragma once
#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace protocol_faults {
using namespace mavmoos::runtime;
inline void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
inline std::string quote(const std::string& input)
{
  constexpr char hex[]="0123456789abcdef";std::string out="\"";
  for(unsigned char c:input){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32||c>=127){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}return out+'"';
}
inline std::string decimal(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
inline std::map<std::string,std::string> fields(const std::string& text)
{
  std::map<std::string,std::string> result;std::istringstream in(text);std::string part;
  while(std::getline(in,part,',')){const auto at=part.find('=');if(at!=std::string::npos)result[part.substr(0,at)]=part.substr(at+1);}return result;
}
inline std::string wire(const mavlink_message_t& message)
{
  constexpr char hex[]="0123456789abcdef";uint8_t buffer[MAVLINK_MAX_PACKET_LEN]{};
  const auto count=mavlink_msg_to_send_buffer(buffer,&message);std::string out;out.reserve(count*2);
  for(unsigned i=0;i<count;++i){out+=hex[buffer[i]>>4];out+=hex[buffer[i]&15];}return out;
}
inline mavlink_message_t decode(const uint8_t* bytes,std::size_t count)
{
  mavlink_message_t buffer{},message{};mavlink_status_t parse{},result{};unsigned found=0;
  for(std::size_t i=0;i<count;++i)if(mavlink_frame_char_buffer(&buffer,&parse,bytes[i],&message,&result)==MAVLINK_FRAMING_OK)++found;
  require(found==1,"outbound bytes are not exactly one complete MAVLink frame");return message;
}
struct Publication{std::string key,text;double at=0;};
struct Frame{mavlink_message_t message{};double at=0,expires=0;bool accepted=false;};
inline std::string publicationJson(const Publication& p){return "{\"key\":"+quote(p.key)+",\"text\":"+quote(p.text)+",\"at\":"+decimal(p.at)+"}";}
inline std::string frameJson(const Frame& p){return "{\"msgid\":"+std::to_string(p.message.msgid)+",\"wire_hex\":"+quote(wire(p.message))+",\"at\":"+decimal(p.at)+",\"expires\":"+decimal(p.expires)+",\"accepted\":"+(p.accepted?"true":"false")+"}";}
inline unsigned command(const mavlink_message_t& m){if(m.msgid!=76)return 0;mavlink_command_long_t p{};mavlink_msg_command_long_decode(&m,&p);return p.command;}
inline unsigned index(const mavlink_message_t& m){mavlink_param_request_read_t p{};mavlink_msg_param_request_read_decode(&m,&p);return uint16_t(p.param_index);}
inline MissionItem home(){MissionItem p;p.seq=0;p.frame=MAV_FRAME_GLOBAL;p.x=423584560;p.y=-710875890;p.z=10;return p;}
inline MissionItem waypoint(){auto p=home();p.seq=1;p.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;p.x+=100;p.z=5;return p;}
inline std::string rows(){return "0:16:0:0:1:0:0:0:0:423584560:-710875890:10|1:16:3:0:1:0:0:0:0:423584660:-710875890:5";}
} // namespace protocol_faults
