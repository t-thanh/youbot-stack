#pragma once

#include <vector>

namespace YouBot
{
	using namespace std;

	void homogeneous_to_xyzypr(const vector<double>& H, vector<double>& xyzypr);

	void Multiply(const vector<double>& H, const vector<double>& r, vector<double>& output);
	void Sum(const vector<double>& lhs, const vector<double>& rhs, vector<double>& output);
}
