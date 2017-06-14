#ifndef __MATRIX_HPP__
#define __MATRIX_HPP__

#include "Array.hpp"

namespace SystemToolkit
{

namespace Types
{

	template<class T>
	class Matrix : private Array<T>
	{
	public:
		Matrix();
		Matrix(size_t, size_t);
		Matrix(size_t, size_t, T);
		Matrix(const Matrix&);
		virtual ~Matrix();

		Matrix& operator=(const Matrix&);
		Matrix operator*(const Matrix&);
		T& operator()(int, int);
		const T& operator()(int, int) const;

		void SetRowCount(size_t);
		void SetColCount(size_t);

		size_t GetRowCount() const;
		size_t GetColCount() const;

	protected:
		void AllocateMatrix();
		void CopyMatrixData(const Matrix&);

		size_t matrixRows;
		size_t matrixCols;
		bool allocated;
	};

	template<class T>
	Matrix<T>::Matrix()
	:matrixRows(0)
	,matrixCols(0)
	,allocated(false)
	{

	}

	template<class T>
	Matrix<T>::Matrix(size_t rows, size_t cols)
	:Array<T>(rows*cols)
	,matrixRows(rows)
	,matrixCols(cols)
	,allocated(true)
	{

	}

	template<class T>
	Matrix<T>::Matrix(size_t rows, size_t cols, T value)
	:Array<T>(rows*cols, value)
	,matrixRows(rows)
	,matrixCols(cols)
	,allocated(true)
	{

	}

	template<class T>
	Matrix<T>::Matrix(const Matrix &other)
	:Array<T>(other)
	,matrixRows(other.matrixRows)
	,matrixCols(other.matrixCols)
	,allocated(true)
	{

	}
	
	template<class T>
	Matrix<T>::~Matrix()
	{
		matrixRows = matrixCols = 0;
		allocated = false;
	}

	template<class T>
	Matrix<T>& Matrix<T>::operator=(const Matrix &other)
	{
		Array<T>::operator=(other);
		matrixRows = other.matrixRows;
		matrixCols = other.matrixCols;
		allocated = other.allocated;

		return *this;
	}
	
	template<class T>
	Matrix<T> Matrix<T>::operator*(const Matrix &other)
	{
		T total;

		if((!(allocated && other.allocated)) || (other.matrixRows != matrixCols))
			throw ERROR_INVALID_REQUEST;

		Matrix result(matrixRows, other.matrixCols);

		for(size_t cRight=0; cRight < other.matrixCols; cRight++)
		{
			for(size_t rLeft=0; rLeft < matrixRows; rLeft++)
			{
				total = 0;
				for(size_t rRight=0; rRight < other.matrixRows; rRight++)
					total += this->operator()(rLeft, rRight) * other(rRight, cRight);

				result(rLeft, cRight) = total;
			}
		}

		return result;
	}
	
	template<class T>
	T& Matrix<T>::operator()(int row, int col)
	{
		if((row < 0) || (row >= matrixRows) || (col < 0) || (col >= matrixCols))
			throw ERROR_INVALID_REQUEST;

		return Array<T>::operator[](row * matrixCols + col);
	}
	
	template<class T>
	const T& Matrix<T>::operator()(int row, int col) const
	{
		if((row < 0) || (row >= matrixRows) || (col < 0) || (col >= matrixCols))
			throw ERROR_INVALID_REQUEST;

		return Array<T>::operator[](row * matrixCols + col);
	}

	template<class T>
	void Matrix<T>::SetRowCount(size_t rowCount)
	{
		size_t oldCount = matrixRows;

		matrixRows = rowCount;
		Array<T>::usedSize = matrixRows * matrixCols;
		if(matrixRows > oldCount)
			AllocateMatrix();
	}

	template<class T>
	void Matrix<T>::SetColCount(size_t colCount)
	{
		size_t oldCount = matrixCols;

		matrixCols = colCount;
		Array<T>::usedSize = matrixRows * matrixCols;
		if(matrixCols > oldCount)
			AllocateMatrix();
	}

	template<class T>
	size_t Matrix<T>::GetRowCount() const
	{
		return matrixRows;
	}

	template<class T>
	size_t Matrix<T>::GetColCount() const
	{
		return matrixCols;
	}

	template<class T>
	void Matrix<T>::AllocateMatrix()
	{
		if((matrixRows == 0) && (matrixCols == 0))
		        throw ERROR_INVALID_REQUEST;

		if(matrixRows == 0)
			matrixRows = 1;
		else if(matrixCols == 0)
			matrixCols = 1;

		Array<T>::usedSize = matrixRows * matrixCols;

		if(!allocated)
		{
			Array<T>::AllocateArray(false, false);
			allocated = true;
		}
		else
			Array<T>::AllocateArray(true, false);
	}

	template<class T>
	void Matrix<T>::CopyMatrixData(const Matrix &other)
	{
		memcpy(Array<T>::data, other.data, matrixRows * matrixCols * sizeof(T));
	}
}

}

#endif
