#include "FawkesConnector.h"
#include <blackboard/remote.h>
#include <netcomm/fawkes/client.h>
#include <interfaces/SkillerInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
#include <string>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <cstdio>
#include <sstream>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

using namespace fawkes;
namespace fawkes_ros {


FawkesConnector::FawkesConnector()
{

}

FawkesConnector::~FawkesConnector()
{
  
}


}//end of namespace