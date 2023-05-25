/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_FAST.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <iostream>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_FAST.h"
#include "XYRangePulse.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_FAST::BHV_FAST(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");
  

  // Add any variables this behavior needs to subscribe for
  // addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  // addInfoVars("WPT_INDEX", "no_warning");

  // m_pulse_posted = false;
  // m_pulse_range = 10;
  // m_wait_time = 5;
  // m_pulse_duration = 2;  
  // m_offset_angle = 45;
  // m_zig_time = 2;

}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_FAST::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if ((param == "speed") && isNumber(val) && (double_val >= 0)){
    m_speed = double_val;
    return(true);
  }
  else if ((param == "pwt") && isNumber(val) && (double_val >= 0)){
    m_priority_wt = double_val;
    return(true);
  }
  
  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_FAST::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_FAST::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_FAST::onIdleState()
{

}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_FAST::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_FAST::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_FAST::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_FAST::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateValues()
//   Purpose: update member variables

// void BHV_FAST::updateValues()
// {
//   // bool found;
//   // m_osx = getBufferDoubleVal("NAV_X", found);
//   // m_osy = getBufferDoubleVal("NAV_Y", found);
// }


//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_FAST::onRunState()
{
  // updateValues();


  IvPFunction *ipf = 0;

  // Step 1 - Create the IvPDomain, the function's domain
  IvPDomain domain;
  domain.addDomain("speed", 0, m_speed*2, m_speed*10);

  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  double maxy = 100;
  spd_zaic.setSummit(m_speed); // summit x value
  spd_zaic.setPeakWidth(m_speed/2.0); // delta x
  spd_zaic.setMinMaxUtil(0, maxy); // miny maxy
  spd_zaic.setBaseWidth(m_speed/2.0); // delta x
  spd_zaic.setSummitDelta(maxy/2.0);   // delta y (from summit to intermediate)
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  else
    ipf = spd_zaic.extractIvPFunction();

  ipf->setPWT(m_priority_wt);

  return(ipf);
}
