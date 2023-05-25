/************************************************************/
/*    NAME: Kevin Becker                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_FAST.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Pulse_HEADER
#define Pulse_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_FAST : public IvPBehavior {
public:
  BHV_FAST(IvPDomain);
  ~BHV_FAST() {};
  
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


protected: // Configuration parameters

protected: // State variables
  double m_speed;
  double m_curr_time;
  double m_priority_wt;
};


#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_FAST(domain);}
}
#endif
