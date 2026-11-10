/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: main.cpp                                        */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "MAVLink.h"
#include "MAVLink_Info.h"

using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command = argv[0];

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-v") || (argi=="--version") || (argi=="-version"))
      showReleaseInfoAndExit();
    else if((argi=="-e") || (argi=="--example") || (argi=="-example"))
      showExampleConfigAndExit();
    else if((argi == "-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi == "-i") || (argi == "--interface"))
      showInterfaceAndExit();
    else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    else if(i==2)
      run_command = argi;
  }

  if(mission_file == "")
    showHelpAndExit();

  cout << termColor("green");
  cout << "iMAVLink launching as " << run_command << endl;
  cout << termColor() << endl;

  MAVLink MAVLink;

  MAVLink.Run(run_command.c_str(), mission_file.c_str());

  return(0);
}
