#ifndef __ARRAY_HPP__
#define __ARRAY_HPP__

#include "TypesTypes.hpp"
#include "../math/StandardMath.hpp"
#include <stdlib.h>
#include <new>
#include <errno.h>
#include <algorithm>
#include <string.h>
#include <stdio.h>

namespace SystemToolkit
{

namespace Types
{

	template<class T>
	class Array
	{
	public:
		Array();
		Array(const Array<T>&);
		Array(size_t);
		Array(size_t, T);
		Array(size_t, T, bool);
		virtual ~Array();

		Array<T>& operator=(const Array<T>&);

		void Push(T);
		T Pop();

		T& operator[](size_t);
		const T& operator[](size_t) const;

		bool IsEmpty() const;
		size_t GetSize() const;
		void Clear(bool=false);
		T Erase(int);
		size_t Find(T);

		bool GenerateDebugString(char*, size_t);

	protected:
		void AllocateArray(bool, bool);

		const float ALLOCATION_FACTOR;

		T *data;
		bool allowResize;
		size_t allocatedSize;
		size_t usedSize;
	};

	template<class T>
	Array<T>::Array():ALLOCATION_FACTOR(1.25)
			 ,data(NULL)
			 ,allowResize(true)
			 ,allocatedSize(0)
			 ,usedSize(0)
	{

	}

	template<class T>
	Array<T>::Array(const Array<T> &other)
	:ALLOCATION_FACTOR(other.ALLOCATION_FACTOR)
	,data(NULL)
	,allowResize(other.allowResize)
	,allocatedSize(other.allocatedSize)
	,usedSize(other.usedSize)
	{
		AllocateArray(false, true);
		for(size_t i=0; i<usedSize; i++)
			data[i] = other.data[i];
	}

	template<class T>
	Array<T>::Array(size_t size)
	:ALLOCATION_FACTOR(1.25)
	,data(NULL)
	,allowResize(true)
	,allocatedSize(size)
	,usedSize(size)
	{
		AllocateArray(false, false);
		T defaultElement;
		if(data != NULL)
			std::fill(data, data + size, defaultElement);
	}

	template<class T>
	Array<T>::Array(size_t size, T value)
	:ALLOCATION_FACTOR(1.25)
	,data(NULL)
	,allowResize(true)
	,allocatedSize(size)
	,usedSize(size)
	{
		AllocateArray(false, false);
		if(data != NULL)
			std::fill(data, data + size, value);
	}

	template<class T>
	Array<T>::Array(size_t size, T value, bool lockCount)
	:ALLOCATION_FACTOR(1.25)
	,data(NULL)
	,allowResize(lockCount)
	,allocatedSize(size)
	,usedSize(size)
	{
		AllocateArray(false, false);
		if(data != NULL)
		{
			std::fill(data, data + size, value);		
			usedSize = size;
		}
	}

	template<class T>
	Array<T>::~Array()
	{
		if(data != NULL)
		{
			delete[] data;
			data = NULL;
		}
	}

	template<class T>
	Array<T>& Array<T>::operator=(const Array<T> &other)
	{
		allowResize = other.allowResize;
		allocatedSize = other.allocatedSize;
		usedSize = other.usedSize;
		AllocateArray(false, true);
		for(size_t i=0; i<other.usedSize; i++)
			data[i] = other.data[i];

		return *this;
	}

	template<class T>
	size_t Array<T>::Find(T target)
	{
		for(size_t i=0; i<usedSize; i++)
			if(data[i] == target)
				return i;
		return -1;
	}

	template<class T>
	bool Array<T>::GenerateDebugString(char *buffer, size_t bufferSize)
	{
		size_t chars;

		memset(buffer, 0, bufferSize);
		chars = snprintf(buffer, bufferSize - 1, "AF: %.2f, AR: %s, AS: %d, US: %d\n", 
				 ALLOCATION_FACTOR, (allowResize) ? "True" : "False", (int)allocatedSize, 
				 (int)usedSize);

		if((chars > 0) && (chars < (bufferSize - 1)))
			return true;

		return false;
	}

	template<class T>
	void Array<T>::Push(T element)
	{
		// array not allocated
		if(data == NULL)
			AllocateArray(false, false);

		// full
		if(usedSize == allocatedSize)
		{
			if(!allowResize)
				throw ERROR_OUT_OF_MEMORY;
			else
				AllocateArray(true, false);
		}

		// add the element
		data[usedSize] = element;
		usedSize++;
	}

	template<class T>
	T Array<T>::Pop()
	{
		size_t index;
		if(usedSize == 0)
			throw ERROR_INVALID_REQUEST;

		index = usedSize-1;
		usedSize--;
		return data[index];
	}

	template<class T>
	T Array<T>::Erase(int index)
	{
		if((index < 0) || (index >= usedSize))
			throw ERROR_INVALID_REQUEST;

		T ret = data[index];
		usedSize--;

		if(index != usedSize)
			memmove(data + index, data + index + 1, (usedSize - index) * sizeof(T));

		return ret;
	}

	template<class T>
	T& Array<T>::operator[](size_t index)
	{
		if((index < 0) || (index >= usedSize))
			throw ERROR_INVALID_REQUEST;

		return data[index];
	}

	template<class T>
	const T& Array<T>::operator[](size_t index) const
	{
		if((index < 0) || (index >= usedSize))
			throw ERROR_INVALID_REQUEST;

		return data[index];
	}

	template<class T>
	bool Array<T>::IsEmpty() const
	{
		return (usedSize == 0);
	}

	template<class T>
	void Array<T>::Clear(bool deallocateResources)
	{
		usedSize = 0;
		if(deallocateResources && (data != NULL))
		{
			allocatedSize = 0;
			delete[] data;
			data = NULL;
		}
	}

	template<class T>
	size_t Array<T>::GetSize() const
	{
		return usedSize;
	}

	template<class T>
	void Array<T>::AllocateArray(bool resize, bool copyConstruction)
	{
		T *tempArray;
		size_t workingSize;

		if(resize)
		{
			(copyConstruction || (!allowResize))  ? workingSize = allocatedSize : workingSize = Math::Round(allocatedSize * ALLOCATION_FACTOR);
			
			tempArray = new(std::nothrow) T[workingSize];
			allocatedSize = workingSize;

			if(data != NULL)
			{
				memcpy(tempArray, data, sizeof(T) * usedSize);
				delete[] data;
			}

			data = tempArray;
		}
		else
		{
			if(data != NULL)
				delete[] data;

			if(allocatedSize == 0)
				allocatedSize = 25;

			(copyConstruction || (!allowResize)) ? workingSize = allocatedSize : workingSize = Math::Round(allocatedSize * ALLOCATION_FACTOR);
			data = new(std::nothrow) T[Math::Round(allocatedSize * ALLOCATION_FACTOR)];
			allocatedSize = workingSize;
		}
		
		if(data == NULL)
			throw GenerateSystemDescriptiveError(errno);
	}

}

}

#endif
