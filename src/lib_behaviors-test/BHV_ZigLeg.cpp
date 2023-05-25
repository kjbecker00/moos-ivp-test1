/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <iostream>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_ZigLeg.h"
#include "XYRangePulse.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("WPT_INDEX", "no_warning");

  m_pulse_posted = false;
  m_pulse_range = 10;
  m_wait_time = 5;
  m_pulse_duration = 2;  
  m_offset_angle = 45;
  m_zig_time = 2;

}


double os_cn_rel_bng = relBearing(m_osx, m_osy, m_osh, m_cnx, m_cny);



//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if ((param == "pulse_range") && isNumber(val) && (double_val >= 0)){
    m_pulse_range = double_val;
    return(true);
  }
  else if ((param == "pulse_duration") && isNumber(val) && (double_val >= 0)){
    m_pulse_duration = double_val;
    return(true);
  }
  else if ((param == "wait_time") && isNumber(val) && (double_val >= 0)){
    m_wait_time = double_val;
    return(true);
  }
  else if ((param == "offset_angle") && isNumber(val) && (double_val >= 0)){
    m_offset_angle = double_val;
    return(true);
  }
  else if ((param == "zig_time") && isNumber(val) && (double_val >= 0)){
    m_zig_time = double_val;
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

void BHV_ZigLeg::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState()
{

}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateValues()
//   Purpose: update member variables

void BHV_ZigLeg::updateValues()
{
  bool found;
  m_osx = getBufferDoubleVal("NAV_X", found);
  m_osy = getBufferDoubleVal("NAV_Y", found);
  m_osh = getBufferDoubleVal("NAV_HDG", found);
  m_contact_node = getBufferDoubleVal("NODE_REPORT_CONTACT", found);

  if (found && (m_contact_node.find(m_to_disrupt) != std::string::npos)){
    m_cnx = stod(tokStringParse(m_contact_node, "X", ',', '='));
    m_cny = stod(tokStringParse(m_contact_node, "Y", ',', '='));
    m_cnhdg = stod(tokStringParse(m_contact_node, "HDG", ',', '='));
  }


}



//---------------------------------------------------------------
// Procedure: toPostPulse()
//   Purpose: determines if a pulse should be posted

bool BHV_ZigLeg::toPostPulse()
{
  m_curr_time = getBufferCurrTime();
  // bool found;

  // if (!found)
  //   return false;

  double os_cn_rel_bng = relBearing(m_osx, m_osy, m_osh, m_cnx, m_cny);
  if (os_cn_rel_bng )

  if (m_pulse_posted == false && os_cn_rel_bng){
    m_pulse_posted = true;
    m_des_hdg = getBufferDoubleVal("NAV_HEADING", found) + m_offset_angle;
    return true;
  }



  // if there is a change in wpt, resets the timer
  if (m_wpt_val != new_wpt_val){
    m_wpt_val = new_wpt_val;
    m_waypt_change_time = m_curr_time;
    m_pulse_posted = false;
  }

  if (m_curr_time - m_waypt_change_time > m_wait_time && (m_pulse_posted==false)){
    m_pulse_posted = true;
    m_des_hdg = getBufferDoubleVal("NAV_HEADING", found) + m_offset_angle;
    return true;
  }
  return false;
}


//---------------------------------------------------------------
// Procedure: toPostPulse()
//   Purpose: determines if a pulse should be posted

bool BHV_ZigLeg::toPostZig()
{
  m_curr_time = getBufferCurrTime();
  // bool found;
  // int new_wpt_val = (int)(getBufferDoubleVal("WPT_INDEX", found));
  // if (!found)
  //   return false;

  if (m_pulse_posted == false)
    return false;

  double delta = m_curr_time - (m_waypt_change_time+m_wait_time);
  // m_des_hdg = getBufferDoubleVal("NAV_HEADING", found) + m_offset_angle;

  if (delta < m_zig_time){
    return true;
  }
  return false;
}


//---------------------------------------------------------------
// Procedure: postPulse()
//   Purpose: post a pulse

void BHV_ZigLeg::postPulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);                // Presumably m_osx is ownship's x position
  pulse.set_y(m_osy);                // Presumably m_osy is ownship's y position
  pulse.set_label("BHV_ZigLeg");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(m_curr_time);       
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  string spec = pulse.get_spec();
  postMessage("VIEW_RANGE_PULSE", spec);
}


//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_ZigLeg::onRunState()
{
  updateValues();


  // Part 1: Check if a pulse should be generated and post
  if (toPostPulse()){
    postPulse();
  }


  IvPFunction *ipf = 0;

  // Part 2: Build the IvP function
  if (toPostZig()){
  // if (true){

    // Step 1 - Create the IvPDomain, the function's domain
    IvPDomain domain;
    domain.addDomain("course", 0, 360, 361);
    // Step 2 - Create the ZAIC_PEAK with the domain and variable name
    ZAIC_PEAK  zaic_peak(domain, "course");
    // Step 3 - Configure the ZAIC_PEAK parameters
    zaic_peak.setSummit(m_des_hdg);
    zaic_peak.setMinMaxUtil(20, 120);
    zaic_peak.setBaseWidth(10);
    zaic_peak.setPeakWidth(10);
    zaic_peak.setSummitDelta(20);
    // Step 4 - Extract the IvP function
    if(zaic_peak.stateOK())
      ipf = zaic_peak.extractIvPFunction();
    else
      std::cout << zaic_peak.getWarnings();
  }


  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}
