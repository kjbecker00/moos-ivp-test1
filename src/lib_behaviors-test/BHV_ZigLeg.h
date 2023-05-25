/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Pulse_HEADER
#define Pulse_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions
  void         postPulse();
  bool         toPostPulse();
  void         updateValues();
  bool         toPostZig();


protected: // Configuration parameters

protected: // State variables
  double m_waypt_change_time;
  double m_curr_time;
  int m_wpt_val;
  double m_pulse_duration;
  double m_pulse_range;
  bool m_pulse_posted;
  double m_wait_time;
  double m_offset_angle;
  double m_zig_time;
  double m_des_hdg;


  double push_duration = 0;
  double push_range = 10;


};


#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ZigLeg(domain);}
}
#endif
