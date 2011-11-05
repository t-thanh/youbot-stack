#pragma once

#include <vector>
#include "xxmatrix.h"
#include <memory.h>
#include <iostream>
#include "rtt/types/carray.hpp"

namespace common20sim
{

	class XVMatrix {

	private:
		double* const mat;
		RTT::types::carray<double> mat_carray;
		std::size_t rows;
		std::size_t columns;

		bool sizeCheck(std::size_t size);
		XVMatrix(); //unusable -> fixed size!
		XVMatrix& operator=(const XVMatrix& ass); // not implemented

	public:
		XVMatrix(XXMatrix& mat_source);
		XVMatrix(double* mat_source,std::size_t rows, std::size_t columns);
		XVMatrix(const XVMatrix& copy);
		virtual ~XVMatrix();

		std::size_t getColumns();
		std::size_t getRows();
		std::vector<double> getVector();
		RTT::types::carray<double>& getCArray();
		std::size_t size() const;
		std::size_t capacity() const;

		double operator() (std::size_t row, std::size_t column) const;
		double& operator() (std::size_t row, std::size_t column);
		void setValues(std::vector<double>& inputs);
		void setValues(RTT::types::carray<double>& inputs);
		void setValues(double& inputs,std::size_t size);
		double at(std::size_t position) const;
		double& at(std::size_t postion);
		//Only used to fake vector<vector> behaviour
		// the values can not be changed
		// it returns the copy of the vector
	//	inline std::vector<double> operator[](std::size_t row);
	//	RTT::types::carray<double>& operator[](std::size_t row);
		//double& operator[] (std::size_t row, std::size_t column);

	};

}
namespace RTT{
std::ostream& operator<<(std::ostream& os, common20sim::XVMatrix& input);
}

