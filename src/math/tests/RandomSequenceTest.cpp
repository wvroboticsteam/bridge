#include "math/RandomSequence.hpp"
#include "math/MathTypes.hpp"

using namespace SystemToolkit::Math;

void PrintSequence(const RandomSequence*);

int main()
{
	try
	{
		RandomSequence sequence(1, 10, false);
		
		sequence.GenerateSequence();
		PrintSequence(&sequence);
		
		sequence.SetSeed(5);
		sequence.GenerateSequence();
		PrintSequence(&sequence);
		
		sequence.SetSeed(1);
		sequence.GenerateSequence();
		PrintSequence(&sequence);
		
		sequence.SetRange(0, 50);
		sequence.GenerateSequence();
		PrintSequence(&sequence);
		
		sequence.SetRange(0, 10);
		sequence.SetUnique(true);
		sequence.GenerateSequence();
		PrintSequence(&sequence);
	}
	catch(ERROR_TYPES e)
	{
		printf("Caught error: %d\n", e);
	}

	return 0;
}

void PrintSequence(const RandomSequence *sequence)
{
	printf("Sequence: ");
	for(size_t i=0; i<sequence->GetSize(); i++)
		printf("%2d ", (*sequence)[i]);
	printf("\n");
}
