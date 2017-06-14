#include "../Array.hpp"
#include <stdio.h>

using namespace SystemToolkit::Types;

void PrintArray(Array<int> *array, bool withDebug)
{
	char buffer[100];
	if(withDebug)
	{
		array->GenerateDebugString(buffer, 100);
		printf("%s", buffer);
	}

	for(size_t i=0; i<array->GetSize(); i++)
		printf("%2d ", (*array)[i]);
	printf("\n");
}

int main()
{
	try
	{
		Array<int> intArray(20, 0);

		PrintArray(&intArray, true);

		for(int i=0; i<20; i++)
			intArray[i] = i;

		PrintArray(&intArray, true);

		for(int i=0; i<11; i++)
			intArray.Push(i);

		PrintArray(&intArray, true);

		for(int i=0; i<5; i++)
			intArray.Pop();

		PrintArray(&intArray, true);

		intArray.Clear(true);

		PrintArray(&intArray, true);
		printf("Is empty: %s\n", intArray.IsEmpty() ? "True" : "False");

		intArray.Push(1);
		intArray.Push(2);
		intArray.Push(3);
		PrintArray(&intArray, true);
		intArray.Erase(1);
		PrintArray(&intArray, true);
		intArray.Erase(1);
		PrintArray(&intArray, true);
	}
	catch(ERROR_TYPES e)
	{
		printf("caught error: %d\n", e);
	}

	return 0;
}
