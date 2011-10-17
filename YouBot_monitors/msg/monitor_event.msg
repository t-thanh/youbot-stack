time stamp

#
# event name                            | type  | configuration parameter                                 |
#---------------------------------------+-------+---------------------------------------------------------|
# jntforceX.e_LIM_EXCEEDED_<bool>       | edge  | force                                                   |
#                                       |       |                                                         |
# cartforce[XYZ].e_LIM_EXCEEDED_<bool>  | edge  | force                                                   |
# carttorque[XYZ].e_LIM_EXCEEDED_<bool> | edge  | torque                                                  |
#                                       |       |                                                         |
# jntposX.e_LIM_EXCEEDED_<bool>         | edge  | percentage of operational range or angle                |
# jntposX.e_POS_REACHED                 | level | acceptable deviation to consider current pos as reached |
#                                       |       |                                                         |
# jntvelX.e_LIM_EXCEEDED_<bool>         | edge  | velocity                                                |
# jntcurX.e_LIM_EXCEEDED_<bool>         | edge  | current                                                 |
#                                       |       |                                                         |
# cartpos.e_LIM_EXCEEDED                | level | surface                                                 |
string monitor_event
