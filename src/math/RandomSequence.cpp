#include "RandomSequence.hpp"
#include "MathTypes.hpp"

using namespace SystemToolkit::Types;

namespace SystemToolkit
{

namespace Math
{

	RandomSequence::RandomSequence()
	:sequenceSize(0)
	,sequenceSeed(1)
	,sequenceMinValue(0)
	,sequenceMaxValue(0)
	,sequenceValuesUnique(false)
	,sequenceGenerated(false)
	{

	}

	RandomSequence::RandomSequence(int seed, size_t size, bool unique)
	:sequenceSize(size)
	,sequenceSeed(seed)
	,sequenceMinValue(0)
	,sequenceMaxValue(size)
	,sequenceValuesUnique(unique)
	,sequenceGenerated(false)
	{

	}

	RandomSequence::RandomSequence(int seed, size_t size, int min, int max, bool unique)
	:sequenceSize(size)
	,sequenceSeed(seed)
	,sequenceMinValue(min)
	,sequenceMaxValue(max)
	,sequenceValuesUnique(unique)
	,sequenceGenerated(false)
	{

	}

	RandomSequence::~RandomSequence()
	{

	}

	void RandomSequence::PrintSequence()
	{
		printf("Sequence: \n");
		for(size_t i=0; i<sequenceSize; i++)
			printf("%d: %d\n", (int)i, sequence[i]);
	}

	void RandomSequence::SetSize(size_t size)
	{
		sequenceSize = size;
	}

	void RandomSequence::SetRange(int min, int max)
	{
		sequenceMinValue = min;
		sequenceMaxValue = max;
	}

	void RandomSequence::SetMinimum(int min)
	{
		sequenceMinValue = min;
	}

	void RandomSequence::SetMaximum(int max)
	{
		sequenceMaxValue = max;
	}

	void RandomSequence::SetUnique(bool unique)
	{
		sequenceValuesUnique = unique;
	}

	void RandomSequence::SetSeed(int seed)
	{
		sequenceSeed = seed;
	}

	size_t RandomSequence::GetSize() const
	{
		return sequence.GetSize();
	}

	void RandomSequence::GetRange(int &min, int &max) const
	{
		min = sequenceMinValue;
		max = sequenceMaxValue;
	}

	int RandomSequence::GetMinimum() const
	{
		return sequenceMinValue;
	}

	int RandomSequence::GetMaximum() const
	{
		return sequenceMaxValue;
	}

	bool RandomSequence::GetUnique() const
	{
		return sequenceValuesUnique;
	}

	int RandomSequence::GetSeed() const
	{
		return sequenceSeed;
	}

	bool RandomSequence::GetGeneratedStatus() const
	{
		return sequenceGenerated;
	}

	const Array<int>* RandomSequence::GetSequencePointer() const
	{
		return &sequence;
	}

	int& RandomSequence::operator[](size_t index)
	{
		if(!sequenceGenerated)
			throw ERROR_INVALID_REQUEST;

		return sequence[index];
	}

	const int& RandomSequence::operator[](size_t index) const
	{
		if(!sequenceGenerated)
			throw ERROR_INVALID_REQUEST;

		return sequence[index];
	}

	bool RandomSequence::Contains(int value, size_t searchLength)
	{
		for(size_t i=0; i<searchLength; i++)
			if(sequence[i] == value)
				return true;

		return false;
	}

	void RandomSequence::GenerateSequence()
	{
		int value;
		int max = sequenceMaxValue - sequenceMinValue;
		size_t counter = 0;
		srand(sequenceSeed);

		sequenceGenerated = false;
		sequence = Array<int>(sequenceSize);

		while(counter < sequenceSize)
		{
			value = (rand() % max) + sequenceMinValue;
			if(sequenceValuesUnique)
			{
				if(!Contains(value, counter))
				{
					sequence[counter] = value;
					counter++;
				}
			}
			else
			{
				sequence[counter] = value;
				counter++;
			}
		}

		sequenceGenerated = true;
	}

}

}
