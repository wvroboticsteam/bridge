#include "../Matrix.hpp"
#include <stdio.h>

using namespace SystemToolkit::Types;

void PrintMatrix(Matrix<double>);

int main()
{
	try
	{
		Matrix<double> rotation(3, 3, 0);
		Matrix<double> point(3, 1);
		Matrix<double> result;

		rotation(0,0) = 1;
		rotation(1,2) = -1;
		rotation(2,1) = 1;

		point(0,0) = 1;
		point(1,0) = 2;
		point(2,0) = 3;

		result = rotation * point;

		printf("Rotation:\n");
		PrintMatrix(rotation);

		printf("\n\nPoint:\n");
		PrintMatrix(point);

		printf("\n\nRotated Point:\n");
		PrintMatrix(result);
	}
	catch(ERROR_TYPES e)
	{
		printf("caught error: %d\n", e);
	}
}

void PrintMatrix(Matrix<double> mat)
{
	for(size_t r = 0; r < mat.GetRowCount(); r++)
	{
		for(size_t c = 0; c < mat.GetColCount(); c++)
			printf("%.3f ", mat(r,c));
		printf("\n");
	}
}
