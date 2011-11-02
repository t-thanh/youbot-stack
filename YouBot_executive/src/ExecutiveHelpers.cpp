#include "ExecutiveHelpers.hpp"

//#include <rtt/TaskContext.hpp>
#include <tf/tf.h>
#include <cassert>

namespace YouBot
{
//	using namespace RTT;
	using namespace std;

	void homogeneous_to_xyzypr(const vector<double>& H, vector<double>& xyzypr)
	{
		assert(H.size() == 16 && xyzypr.size() == 6);

		xyzypr[0] = H[3];
		xyzypr[1] = H[7];
		xyzypr[2] = H[11];

		btMatrix3x3 rotMatrix(H[0], H[1], H[2], H[4], H[5], H[6], H[8], H[9], H[10]);

		btScalar y, p, r;
		rotMatrix.getEulerYPR(y, p, r);
		xyzypr[3] = y;
		xyzypr[4] = p;
		xyzypr[5] = r;
	}

	void Multiply(const vector<double>& H, const vector<double>& r, vector<double>& output)
	{
		using namespace std;

		if(H.size() != 16 || r.size() != 4 || output.size() != 4)
		{
			 __throw_out_of_range(__N("Multiply::vector::_M_range_check"));
		}

		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				output[i]+=H[i*4+j]*r[j];
			}
		}
	}

	void Sum(const vector<double>& lhs,const vector<double>& rhs, vector<double>& output)
	{
		using namespace std;

		if(lhs.size() != rhs.size() || output.size() != rhs.size())
		{
			 __throw_out_of_range(__N("Sum::vector::_M_range_check"));
		}

		for(int i=0;i<rhs.size();i++)
		{
			output[i]=rhs[i]*lhs[i];
		}
	}


}
