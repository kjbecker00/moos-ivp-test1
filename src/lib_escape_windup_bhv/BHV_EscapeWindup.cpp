/************************************************************/
/*    NAME: Ray Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_EscapeWindup.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_EscapeWindup.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYRangePulse.h"
using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_EscapeWindup::BHV_EscapeWindup(IvPDomain domain) :
  IvPBehavior(domain)
{

  m_domain = domain;
  // Provide a default behavior name
  IvPBehavior::setParam("name", "BHV_EscapeWindup");

  // Declare the behavior decision space
  //m_domain = subDomain(m_domain, "course, speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");

  m_speed_threshold = 0.2;
  m_priority_weight = 100;
  m_heading_accumulator = 0;
  m_pwt_accumulator = 0;
  m_nav_x = 0;
  m_nav_y = 0;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_EscapeWindup::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  if((param == "speed_threshold") && isNumber(val)) {
    // Set local member variables here
    return(setNonNegDoubleOnString(m_speed_threshold, val));
  }
  else if (param == "priority_weight") {
    return(setNonNegDoubleOnString(m_priority_weight, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_EscapeWindup::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_EscapeWindup::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_EscapeWindup::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_EscapeWindup::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_EscapeWindup::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_EscapeWindup::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_EscapeWindup::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_EscapeWindup::onRunState()
{
  double speed, heading;
  bool ok1, ok2;

  m_nav_x = getBufferDoubleVal("NAV_X", ok1);
  m_nav_y = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }

  speed = getBufferDoubleVal("NAV_SPEED", ok1);
  heading = getBufferDoubleVal("NAV_HEADING", ok2);

  

  if (!ok1 || !ok2) {
    postWMessage("No ownship SPEED/HEADING info available in info_buffer");
    return(0);
  }

  bool m_active = (speed < m_speed_threshold);
  if (speed >= m_speed_threshold) {
  //if (false) {
    m_heading_accumulator = 0;
    m_pwt_accumulator = 0;
    m_must_break = false;
    return(0);
  } else {
    if(m_must_break == false) {
      m_must_break = true;
      XYRangePulse my_pulse(m_nav_x, m_nav_y);      
      my_pulse.set_label("Slowing down");
      my_pulse.set_duration(6);
      my_pulse.set_edge_size(1);
      my_pulse.set_rad(50);
      my_pulse.set_fill(0.9);
      my_pulse.set_color("edge", "red");
      my_pulse.set_color("fill", "purple");
      string str = my_pulse.get_spec();
      postMessage("VIEW_RANGE_PULSE", str);
    }
    m_heading_accumulator+=1;
    m_pwt_accumulator+=5;
    if (m_heading_accumulator > 90){
      m_heading_accumulator = 90;
      //pwt will grow unbounded - windup and breakout!!
    }
  }
  // Create IvP function
  ZAIC_PEAK speed_zaic(m_domain, "speed");
  speed_zaic.setPeakWidth(0.5);
  speed_zaic.setBaseWidth(1.0);
  speed_zaic.setSummit(speed + 1);
  speed_zaic.setValueWrap(true);
  IvPFunction *speed_ipf = speed_zaic.extractIvPFunction();

  if(speed_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + speed_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double new_heading_ccw = (double)(static_cast<int>(heading - m_heading_accumulator + 360)%360);
  double new_heading_cw = (double)(static_cast<int>(heading + m_heading_accumulator + 360)%360);

  ZAIC_PEAK heading_zaic_ccw(m_domain, "course");
  heading_zaic_ccw.setPeakWidth(0);
  heading_zaic_ccw.setBaseWidth(90);
  heading_zaic_ccw.setSummit(new_heading_ccw);
  heading_zaic_ccw.setValueWrap(true);
  IvPFunction *heading_ipf_ccw = heading_zaic_ccw.extractIvPFunction();

  if(heading_zaic_ccw.stateOK() == false) {
    string warnings = "Course CCW ZAIC problems " + heading_zaic_ccw.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  ZAIC_PEAK heading_zaic_cw(m_domain, "course");
  heading_zaic_cw.setPeakWidth(0);
  heading_zaic_cw.setBaseWidth(90);
  heading_zaic_cw.setSummit(new_heading_cw);
  heading_zaic_cw.setValueWrap(true);
  IvPFunction *heading_ipf_cw = heading_zaic_cw.extractIvPFunction();

  if(heading_zaic_cw.stateOK() == false) {
    string warnings = "Course CW ZAIC problems " + heading_zaic_cw.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  // Set priority weight
  if (speed_ipf)
    speed_ipf->setPWT(m_priority_weight+m_pwt_accumulator);
  if (heading_ipf_ccw)
    heading_ipf_ccw->setPWT(m_priority_weight+m_pwt_accumulator);
  if (heading_ipf_cw)
  heading_ipf_cw->setPWT(m_priority_weight+m_pwt_accumulator);

  // Combine IvP functions
  IvPFunction *ipf = 0;

  if (speed_ipf && heading_ipf_ccw && heading_ipf_cw) {
    OF_Coupler coupler;
    //IvPFunction *heading_ipf = coupler.couple(heading_ipf_ccw, heading_ipf_cw, 30, 70);
    ipf = coupler.couple(speed_ipf, heading_ipf_cw, 50, 50);
  }

  if(ipf) {
    ipf->setPWT(m_priority_weight+m_pwt_accumulator);
  } 

  return(ipf);
}

