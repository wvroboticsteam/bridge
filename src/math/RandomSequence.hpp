#ifndef __RANDOMSEQUENCE_HPP__
#define __RANDOMSEQUENCE_HPP__

#include "../types/Array.hpp"

namespace SystemToolkit
{

namespace Math
{
	// range is [x, y)
	class RandomSequence
	{
	public:
		RandomSequence();
		RandomSequence(int, size_t, bool);
		RandomSequence(int, size_t, int, int, bool);
		virtual ~RandomSequence();

		void SetSize(size_t);
		void SetRange(int, int);
		void SetMinimum(int);
		void SetMaximum(int);
		void SetUnique(bool);
		void SetSeed(int);

		size_t GetSize() const;
		void GetRange(int&, int&) const;
		int GetMinimum() const;
		int GetMaximum() const;
		bool GetUnique() const;
		int GetSeed() const;
		bool GetGeneratedStatus() const;
		const Types::Array<int>* GetSequencePointer() const;

		int& operator[](size_t);
		const int& operator[](size_t) const;
		void GenerateSequence();
		bool Contains(int, size_t);
		
		void PrintSequence();

	protected:
		size_t sequenceSize;
		int sequenceSeed;
		int sequenceMinValue;
		int sequenceMaxValue;
		bool sequenceValuesUnique;
		bool sequenceGenerated;
		Types::Array<int> sequence;
	};
}

}			

#endif
