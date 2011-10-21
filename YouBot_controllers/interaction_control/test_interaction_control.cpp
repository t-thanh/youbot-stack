/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  test_interaction_control.cpp
 *  subm:  interaction_control
 *  model: ModelControl
 *  expmt: ModelControl
 *  date:  October 21, 2011
 *  time:  3:01:21 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/

/* This file is a demo application of how the submodel function can
 * be used. It uses the global time variables that are used by both
 * the submodel and the integration method.
 *
 * PLEASE NOTE: THIS IS AN EXAMPLE WHERE ALL INPUTS ARE ZERO !
 * USE YOUR OWN INPUTS INSTEAD!! ALSO THE SUBMODEL MIGHT SIMPLY
 * NOT WORK CORRECTLY WITH INPUTS THAT ARE ZERO.
 */

#include <stdio.h>

/* 20-sim submodel class include file */
#include "interaction_control.h"

/* Orocos include file */
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace Orocos;
using namespace RTT;
using namespace std;
//using namespace ModelControl;
/* the main function */
int ORO_main(int argc, char** argv)
{
        Logger::In in("main()");

	if ( log().getLogLevel() < Logger::Info ) {
	        log().setLogLevel( Logger::Info );
        	log(Info) << argv[0] << " manually raises LogLevel to 'Info' (5). See also file 'orocos.log'."<<endlog();
    	}

		ModelControl::interaction_control my20simSubmodel;

        my20simSubmodel.setActivity(new Activity( 2 ));

        /* initialize the submodel */
        my20simSubmodel.configure();

        /* start the submodel */
        my20simSubmodel.start();

        TaskBrowser browser(&my20simSubmodel);

        browser.loop();

	return 0;
}

