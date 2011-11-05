#include "XVMatrix.h"

#include <rtt/RTT.hpp>

namespace common20sim
{
	/**
	* Implements constructor that uses provided link to a specific addresss
	*/
	XVMatrix::XVMatrix(double * mat_source, std::size_t rows, std::size_t columns) :
			mat(mat_source), mat_carray(mat_source, rows * columns), rows(rows), columns(columns)
	{
	}

	/**
	* default constructor not really usable since the size of the matrix cann't be extended
	*/
	XVMatrix::XVMatrix() : mat(0), rows(0), columns(0)
	{
	}

	/**
	* constructor from XXMatrix accessible in 20 sim
	*/
	XVMatrix::XVMatrix(XXMatrix & mat_source) :
			mat(mat_source.mat), mat_carray(mat_source.mat, mat_source.rows * mat_source.columns), rows(mat_source.rows), columns(mat_source.columns)
	{
	}

	/**
	* shallow copy constructor points to the same memory address
	*/
	XVMatrix::XVMatrix(const XVMatrix & copy) :
		mat(copy.mat), mat_carray(copy.mat_carray), rows(copy.rows), columns(copy.columns)
	{
	}

	/**
	*  Note that invoking default the destructor you clean up memory so the matrixes are destroyed thus,
	*  it might create a problem for the code that use that matrix
	*/
	XVMatrix::~XVMatrix()
	{}

	/**
	*  although the passed by value return is done; carray carry the pointer to the data, so the data still can be modified
	*/
	RTT::types::carray<double>& XVMatrix::getCArray()
	{
		return mat_carray;
	}

	XVMatrix& XVMatrix::operator=(const XVMatrix& ass)
	{
		RTT::log(RTT::Error) << "Assignment operator not implemented.";
		return *this;
	}

	double& XVMatrix::at(std::size_t position)
	{
		return mat[position];
	}

	double XVMatrix::at(std::size_t position) const
	{
		return mat[position];
	}

	/**
	* simple access to the elements of the matrix const
	*/
	double XVMatrix::operator ()(std::size_t row, std::size_t column) const
	{
		return mat[row * columns + column];
	}

	/**
	* simple access to the elements of the matrix modifiable
	*/
	double& XVMatrix::operator ()(std::size_t row, std::size_t column)
	{
		return mat[row * columns + column];
	}

	/**
	*  added for compliance with RTT
	*/
	std::size_t XVMatrix::capacity() const
	{
		return rows * columns;
	}

	bool XVMatrix::sizeCheck(std::size_t size)
	{
		return (size == this->size());
	}

	/**
	*  modifying the vector will have no effect on the internal data
	*  all values are copied
	*/
	std::vector<double>  XVMatrix::getVector()
	{
		return std::vector<double>(mat,mat+size());
	}

	std::size_t XVMatrix::size() const
	{
		return columns*rows;
	}

	std::size_t XVMatrix::getColumns()
	{
		return columns;
	}

	std::size_t XVMatrix::getRows()
	{
		return rows;
	}

	void XVMatrix::setValues(std::vector<double> & inputs)
	{
		if (sizeCheck(inputs.size()))
		{
			memcpy(mat, inputs.data(), size() * sizeof(double));
		}
		else
		{
			std::__throw_out_of_range(__N("XVMatrix::_M_range_check"));
		}
	}

	void XVMatrix::setValues(double & inputs, std::size_t size)
	{
		if (sizeCheck(size))
		{
			memcpy(mat, &inputs, this->size() * sizeof(double));
		}
		else
		{
			std::__throw_out_of_range(__N("XVMatrix::_M_range_check"));
		}
	}

	void XVMatrix::setValues(RTT::types::carray<double> & inputs)
	{
		if (sizeCheck(inputs.count()))
		{
			memcpy(mat, inputs.address(), size() * sizeof(double));
		}
		else
		{
			std::__throw_out_of_range(__N("XVMatrix::_M_range_check"));
		}
	}

}

namespace RTT
{
	std::ostream& operator<<(std::ostream& os, common20sim::XVMatrix& input)
	{
		os << input.at(0);
		for(size_t i=1;i<input.size();i++)
		{
			os << ", " << input.at(i);
		}
		return os;
	}
}

