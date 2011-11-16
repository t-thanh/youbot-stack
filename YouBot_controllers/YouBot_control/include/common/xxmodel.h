#ifndef SUBMODEL20SIM_H
#define SUBMODEL20SIM_H

#include "xxtypes.h"
#include "xxmatrix.h"


class Submodel20sim
{
	protected:
		friend class IntegrationMethod;
		friend class Discrete;
		friend class Euler;
		friend class RungeKutta2;
		friend class RungeKutta4;
		virtual void CalculateDynamic (void) {};

		bool     initialize;
		bool     major;

	public:
		virtual ~Submodel20sim(){};
		
		XXDouble step_size;
		XXDouble start_time;
		XXDouble finish_time;
		XXDouble time;

		/* the variable count */
		int number_constants;
		int number_parameters;
		int number_initialvalues;
		int number_variables;
		int number_states;
		int number_rates;
		int number_matrices;
		int number_unnamed;

		/* the variable arrays are allocated in the derived submodel class */
		XXDouble* C;					/* constants */
		XXDouble* P;					/* parameters */
		XXDouble* I;				/* initial values */
		XXDouble* V;					/* variables */
		XXDouble* s;						/* states */
		XXDouble* R;						/* rates (or new states) */
		XXMatrix* M;					/* matrices */
		XXDouble* U;					/* unnamed */
		XXDouble* workarray;
};

#endif 	// SUBMODEL20SIM_H

