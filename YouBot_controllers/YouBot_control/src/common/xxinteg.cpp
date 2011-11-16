/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  src\common\xxinteg.cpp
 *  subm:  YouBot_control
 *  model: RCCMotionStack
 *  expmt: RCCMotionStack
 *  date:  November 16, 2011
 *  time:  9:49:52 pm
 *  user:  Campuslicentie
 *  from:  Universiteit Twente
 *  build: 4.1.2.2
 **********************************************************/

/* This file describes the integration methods
   that are supplied for computation.

   Currently only Euler, RungeKutta2 and RungeKutta4 are supplied,
   but it is easy for the user to add their own
   integration methods with these as examples.
*/

/* the system include files */
#include <stdlib.h>
#include <memory.h>
#include <math.h>

/* our own include files */
#include "xxinteg.h"
#include "xxmodel.h"


/*********************************************************************
 * Discrete integration method
 *********************************************************************/
Discrete::Discrete(void)
{
	model = NULL;
}

Discrete::~Discrete(void)
{
}

/*
 * the initialization of the Discrete integration method
 */
void Discrete::Initialize (Submodel20sim* themodel)
{
	model = themodel;
	major = true;
	model->major = true;
}

/* the termination of the Discrete integration method */
void Discrete::Terminate ()
{
	/* nothing to be done */
}

/* the Discrete integration method itself */
void Discrete::Step ()
{
	XXInteger index;

	/* for each of the supplied states */
	for (index = 0; index < model->number_states; index++)
	{
		/* just a move of the new state */
		model->s [index] = model->R [index];
	}

	/* increment the simulation time */
	model->time += model->step_size;
	major = true;
	model->major = true;

	/* evaluate the dynamic part to calculate the new rates */
	model->CalculateDynamic ();
}

/*********************************************************************
 * Euler integration method
 *********************************************************************/
Euler::Euler(void)
{
	model = NULL;
}

Euler::~Euler(void)
{
}

/*
 * the initialization of the Euler integration method
 */
void Euler::Initialize(Submodel20sim* themodel)
{
	model = themodel;
	major = true;
	model->major = true;
}

/* the termination of the Euler integration method */
void Euler::Terminate ()
{
	/* nothing to be done */
}

/* the Euler integration method itself */
void Euler::Step ()
{
	XXInteger index;

	/* for each of the supplied states */
	for (index = 0; index < model->number_states; index++)
	{
		/* calculate the new state */
		model->s [index] += model->R [index] * model->step_size;
	}

	/* increment the simulation time */
	model->time += model->step_size;
	major = true;
	model->major = true;

	/* evaluate the dynamic part to calculate the new rates */
	model->CalculateDynamic ();
}


/*********************************************************************
 * RungeKutta2 integration method
 *********************************************************************/
RungeKutta2::RungeKutta2(void)
{
	q0 = NULL;
	model = NULL;
}

RungeKutta2::~RungeKutta2(void)
{
	if (q0)
		delete[] q0;
}

/*
 * the initialization of the RungeKutta2 integration method
 */
void RungeKutta2::Initialize (Submodel20sim* themodel)
{
	model = themodel;
	
	//allocate and empty the q0 array
	q0 = new XXDouble [model->number_states + 1];

	memset (q0, '\0', (model->number_states + 1) * sizeof (XXDouble));
}

/* the termination of the RungeKutta2 integration method */
void RungeKutta2::Terminate ()
{
}

/* the Runge-Kutta-2 integration method itself */
void RungeKutta2::Step ()
{
	XXInteger index;
	XXDouble rktime;

	/* check if we have states at all */
	if (model->number_states > 0)
	{
		/*********************************************************************************/
		/*  		calculate intermediate result										 */
		/*********************************************************************************/

		/* cache the simulation time */
		rktime = model->time;

		/* the q0 will hold the value of the last evaluation. This is used multiple times
		   so remember the complete state array in this vector. */
		memcpy (q0, model->s, model->number_states * sizeof (XXDouble));

		/* calculate f (states, t) * 1/2 * dt  =  rates * 1/2 * dt  */
		for (index = 0; index < model->number_states; index++)
		{
			/* set the new states to use */
			model->s [index] = q0[index] + model->R[index] * 0.5 * model->step_size;
		}

		model->time = rktime + 0.5 * model->step_size;
		major = false;
		model->major = false;
		model->CalculateDynamic ();

		/* for each state */
		for (index = 0; index < model->number_states; index++)
		{
			/*********************************************************************************/
			/*  		calculate the next state from the intermediate results          	 */
			/*********************************************************************************/

			/* calculate the next state = classical Runge-Kutta integration step */
			model->s [index] = q0 [index] + model->R [index] * model->step_size;
		}
		model->time = rktime + model->step_size;
	}
	else
	{
		/* no states in the model */
		/* increment the simulation time */
		model->time += model->step_size;
	}

	major = true;
	model->major = true;

	/* evaluate the derivative model to calculate the new rates */
	model->CalculateDynamic ();
}


/*********************************************************************
 * RungeKutta4 integration method
 *********************************************************************/
RungeKutta4::RungeKutta4(void)
{
	model = NULL;
	q0 = NULL;
	q1 = NULL;
	q2 = NULL;
	q3 = NULL;
	q4 = NULL;

}

RungeKutta4::~RungeKutta4(void)
{
	if (q0)
	{
		delete[] q0;
		delete[] q1;
		delete[] q2;
		delete[] q3;
		delete[] q4;
	}
}


/*********************************************************************
 * the initialization of the RungeKutta4 integration method
 */
void RungeKutta4::Initialize (Submodel20sim* themodel)
{
	model = themodel;
	
	/* allocate internal arrays */
	q0 = new XXDouble [model->number_states + 1];
	q1 = new XXDouble [model->number_states + 1];
	q2 = new XXDouble [model->number_states + 1];
	q3 = new XXDouble [model->number_states + 1];
	q4 = new XXDouble [model->number_states + 1];

	/* empty our internal arrays */
	memset (q0, '\0', (model->number_states + 1) * sizeof (XXDouble));
	memset (q1, '\0', (model->number_states + 1) * sizeof (XXDouble));
	memset (q2, '\0', (model->number_states + 1) * sizeof (XXDouble));
	memset (q3, '\0', (model->number_states + 1) * sizeof (XXDouble));
	memset (q4, '\0', (model->number_states + 1) * sizeof (XXDouble));
}

/* the termination of the RungeKutta4 integration method */
void RungeKutta4::Terminate (void)
{
	/* nothing yet */
}

/* the Runge-Kutta-4 integration method itself */
void RungeKutta4::Step ()
{
	XXInteger index;
	XXDouble rktime;

	/* check if we have states at all */
	if (model->number_states > 0)
	{
		/*********************************************************************************/
		/*  		calculate intermediate state results q1, q2, q3 and q4	        	 */
		/*********************************************************************************/

		/* cache the simulation time */
		rktime = model->time;

		/* the q0 will hold the value of the last evaluation. This is used multiple times
		   so remember the complete state array in this vector. */
		memcpy (q0, model->s, model->number_states * sizeof (XXDouble));

		/* calculate q1 = f (states, t) * dt  =  rates * dt  */
		for (index = 0; index < model->number_states; index++)
		{
			/* set the intermediate q1 */
			q1 [index] = model->R [index] * model->step_size;

			/* set the new states to use  for q2 */
			model->s [index] = q0 [index] + q1 [index] / 2;
		}

		/* calculate q2 = f (states + q1 / 2, t + dt / 2) * dt  */
		model->time = rktime + model->step_size / 2;

		major = false;
		model->major = false;

		model->CalculateDynamic ();
		memcpy (q2, model->R, model->number_states * sizeof (XXDouble));

		/* for each state */
		for (index = 0; index < model->number_states; index++)
		{
			/* set the ultimate q2 */
			q2 [index] = q2 [index] * model->step_size;

			/* set the new states to use */
			model->s [index] = q0 [index] + q2 [index] / 2;
		}

		/* calculate q3 = f (states + q2 / 2, t + dt / 2) * dt  */
		model->CalculateDynamic ();
		memcpy (q3, model->R, model->number_states * sizeof (XXDouble));

		/* for each state */
		for (index = 0; index < model->number_states; index++)
		{
			/* set the ultimate q3 */
			q3 [index] = q3 [index] * model->step_size;

			/* set the new states */
			model->s [index] = q0 [index] + q3 [index];
		}

		/* calculate q4 = f (states + q3, t + dt) * dt */
		model->time = rktime + model->step_size;
		model->CalculateDynamic ();
		memcpy (q4, model->R, model->number_states * sizeof (XXDouble));

		/* for each state */
		for (index = 0; index < model->number_states; index++)
		{
			/* set the ultimate q4 */
			q4 [index] = q4 [index] * model->step_size;

			/*********************************************************************************/
			/*  		calculate the next state from the intermediate results          	 */
			/*********************************************************************************/

			/* calculate the next state = classical Runge-Kutta integration step */
			model->s [index] = q0 [index] +
			q1 [index] / 6 + q2 [index] / 3 + q3 [index] / 3 + q4 [index] / 6;
		}
	}
	else
	{
		/* no states in the model */
		/* increment the simulation time */
		model->time += model->step_size;
	}

	major = true;
	model->major = true;

	/* evaluate the derivative model to calculate the new rates */
	model->CalculateDynamic ();
}

