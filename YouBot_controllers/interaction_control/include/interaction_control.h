#pragma once

/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  include\interaction_control.h
 *  subm:  interaction_control
 *  model: ModelControl
 *  expmt: ModelControl
 *  date:  October 21, 2011
 *  time:  1:00:09 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 *
 **********************************************************/

/* This file describes the model functions
 that are supplied for computation.

 The model itself is the interaction_control.cpp file
 */

/* 20-sim include files */
#include "xxfuncs.h"
#include "xxmatrix.h"
#include "xxmodel.h"
#include "xxinteg.h"

/* parameter parsing include */
#include "tinyxml.h"

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>

#include "Adapter20Sim.h"

namespace ModelControl
{
	using namespace common20sim;

	class interaction_control: virtual Submodel20sim , public RTT::TaskContext
	{
	public:
		//enum stateflags_interaction_control {initialrun, mainrun, finished};

		/**
		 * interaction_control constructor
		 */
		interaction_control(string name = "interaction_control");

		/**
		 * interaction_control destructor
		 */
		virtual ~interaction_control(void);

		/**
		 * interaction_control configuration code and returns false if startup fails
		 */
		bool configureHook ();

		/**
		 * interaction_control startUp code and returns false if startup fails
		 */
		bool startHook ();

		/**
		 * interaction_control Calculation executed in this Hook.
		 */
		void updateHook ();

		/**
		 * interaction_control Terminate
		 */
		void stopHook ();

		XXDouble GetTime(void)
		{
			return time;
		}

		//		stateflags_interaction_control state;

		virtual bool setPeriod(RTT::Seconds s);

	protected:
		/**
		 * CalculateDynamic()
		 * This function calculates the dynamic equations of the model.
		 * These equations are called from the integration method
		 * to calculate the new model rates (that are then integrated).
		 */
		void CalculateDynamic (void);

	private:
		/* internal submodel computation methods */

		XXDouble u [22 + 1]; // Optimization for CopyInputsToVariables
		XXDouble y [8 + 1]; //Optimization for CopyVariablesToOutputs

		/**
		 * CalculateInitial()
		 * This function calculates the initial equations of the model.
		 * These equations are calculated before anything else
		 */
		void CalculateInitial (void);

		/**
		 * CalculateStatic()
		 * This function calculates the static equations of the model.
		 * These equations are only dependent from parameters and constants
		 */
		void CalculateStatic (void);

		/**
		 * CalculateInput()
		 * This function calculates the input equations of the model.
		 * These equations are dynamic equations that must not change
		 * in calls from the integration method (like random and delay).
		 */
		void CalculateInput (void);

		/**
		 * CalculateOutput()
		 * This function calculates the output equations of the model.
		 * These equations are not needed for calculation of the rates
		 * and are kept separate to make the dynamic set of equations smaller.
		 * These dynamic equations are called often more than one time for each
		 * integration step that is taken. This makes model computation much faster.
		 */
		void CalculateOutput (void);

		/**
		 * CalculateFinal()
		 * This function calculates the final equations of the model.
		 * These equations are calculated after all the calculations
		 * are performed
		 */
		void CalculateFinal (void);

		/**
		 * CopyInputsToVariables
		 * This private function copies the input variables from the input vector
		 * @param u	This is the array with all input signals for this submodel
		 */
		void CopyInputsToVariables ();

		/**
		 * CopyVariablesToOutputs
		 * This private function copies the output variables to the output vector
		 * @param y	This is the array with all output signals from this submodel
		 */
		void CopyVariablesToOutputs ();

		/**
		 * @brief setupParametersAndStates()
		 * Loads the parameter and state definitions from a 20sim generated xml file.
		 * Properties can be automatically overridden by an Orocos config xml file.
		 */
		void setupParametersAndStates();

		/**
		 * @brief Helper function to cleanup created property bags.
		 * @return Deleted any sub property bags.
		 * @param p - The PropertyBag from which sub property bags should be removed.
		 * @note - Does not delete any other Property<T>'s.
		 */
		void cleanupPropertyBags(RTT::PropertyBag* p);

		RungeKutta4 myintegmethod; ///< pointer to the integration method for this submodel

		/**
		 * OROCOS Ports for input and ouput
		 */

		vector<Adapter20Sim<RTT::InputPort<flat_matrix_t > > > inputPorts;
		vector<Adapter20Sim<RTT::OutputPort<flat_matrix_t > > > outputPorts;
		vector<Adapter20Sim<RTT::Property<RTT::types::carray<double> > > > propertyPorts;

		RTT::PropertyBag* createHierarchicalPropertyBags( const char * name );
	private:
		void loadMatrixValue(const char * input,XVMatrix *output);
		bool initializeComputation();
		string TSim_config_xml;

	};

}

