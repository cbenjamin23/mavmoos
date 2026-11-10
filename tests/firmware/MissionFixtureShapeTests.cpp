#include "MissionFixtureShape.h"
#include <iostream>

int main() {
  const auto require=[](bool value){if(!value)throw std::runtime_error("mission fixture shape control failed");};
  try {
    const mavmoos::test::MissionFixtureShape empty("empty"),home("bound_home_only");
    require(empty.count()==0&&home.count()==1);
    require(empty.complete({{43,1},{47,1}}));
    require(!empty.complete({{43,1},{51,1},{47,1}}));
    require(!empty.complete({{43,1}}));
    require(!home.complete({{43,1},{47,1}}));
    require(home.complete({{43,2},{51,1},{47,1}}));
    require(!home.complete({{43,1},{51,1},{47,1},{44,1}}));
    require(!home.complete({{43,1},{51,0},{47,1}}));
    bool rejected=false;try{mavmoos::test::MissionFixtureShape unknown("auto");}catch(const std::runtime_error&){rejected=true;}
    require(rejected);
    std::cout<<"mission fixture enum/count/read-message controls PASS\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
