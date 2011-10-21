/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator
 *
 *  file:  include\common\xxfuncs.h
 *  subm:  interaction_control
 *  model: ModelControl
 *  expmt: ModelControl
 *  date:  October 21, 2011
 *  time:  3:19:46 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/

#ifndef XX_FUNCS_H
#define XX_FUNCS_H

/* our own include files */
#include "xxtypes.h"

/* global constants */
extern XXDouble xx_logarithm_2;
extern XXDouble xx_logarithm_10;
extern XXBoolean initialize;
extern XXBoolean major;

/* 20-sim function prototypes */
XXDouble XXAbsolute (XXDouble argument);
XXDouble XXArcCosineHyperbolic (XXDouble argument);
XXDouble XXArcSineHyperbolic (XXDouble argument);
XXDouble XXArcTangentHyperbolic (XXDouble argument);
XXDouble XXExponent2 (XXDouble argument);
XXDouble XXExponent10 (XXDouble argument);
XXDouble XXIntegerDivide (XXDouble argument1, XXDouble argument2);
XXDouble XXIntegerModulo (XXDouble argument1, XXDouble argument2);
XXDouble XXLogarithm2 (XXDouble argument);
XXDouble XXLogarithm10 (XXDouble argument);
XXDouble XXPow2 (XXDouble argument);
XXDouble XXPower (XXDouble argument1, XXDouble argument2);
XXDouble XXRandom (XXDouble argument);
XXDouble XXRamp (XXDouble argument, XXDouble time);
XXDouble XXRound (XXDouble argument);
XXDouble XXSign (XXDouble argument);
XXDouble XXStep (XXDouble argument, XXDouble time);
XXDouble XXImpulse (XXDouble arg1, XXDouble arg2, XXDouble time, XXDouble stepsize);
XXDouble XXXor (XXDouble argument1, XXDouble argument2);

/* 20-sim stub prototypes */
XXDouble XXData (XXString name, XXInteger column, XXInteger id);
XXDouble XXTable (XXString name, XXDouble argument, XXInteger id);
XXBoolean XXEvent (XXDouble argument, XXInteger id);
XXBoolean XXEventUp (XXDouble argument, XXInteger id);
XXBoolean XXEventDown (XXDouble argument, XXInteger id);
XXBoolean XXFrequencyEvent (XXDouble argument, XXInteger id);
XXBoolean XXFrequencyEvent1 (XXDouble argument1, XXDouble argument2, XXInteger id);
XXBoolean XXTimeEvent (XXDouble argument, XXInteger id);
XXDouble XXTimeDelay (XXDouble argument, XXDouble time, XXInteger id);
XXBoolean XXWarning (XXString message, XXInteger id);
XXBoolean XXStopSimulation (XXString message, XXInteger id);

#endif

