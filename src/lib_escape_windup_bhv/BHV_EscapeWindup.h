/************************************************************/
/*    NAME: Ray Turrisi                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_EscapeWindup.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef EscapeWindup_HEADER
#define EscapeWindup_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_EscapeWindup : public IvPBehavior {
public:
  BHV_EscapeWindup(IvPDomain);
  ~BHV_EscapeWindup() {};
  
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
double m_speed_threshold;
double m_priority_weight;
double m_heading_accumulator;
double m_pwt_accumulator;
double m_nav_x;
double m_nav_y;
bool m_must_break;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_EscapeWindup(domain);}
}
#endif
