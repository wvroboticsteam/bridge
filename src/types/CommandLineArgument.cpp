#include "CommandLineArgument.hpp"
#include <string.h>
#include <stdio.h>

namespace SystemToolkit
{

namespace Types
{

	CommandLineArgument::CommandLineArgument()
	:hasData(false)
	,value(0)
	,data(NULL)
	{

	}

	CommandLineArgument::CommandLineArgument(char val)
	:hasData(false)
	,value(val)
	,data(NULL)
	{

	}

	CommandLineArgument::CommandLineArgument(char val, bool hData, const char *dat)
	:data(NULL)
	{
		SetData(dat);
		value = val;
		hasData = hData;
	}

	CommandLineArgument::CommandLineArgument(const CommandLineArgument &other)
	{
		CopyObject(other);
	}

	CommandLineArgument::~CommandLineArgument()
	{
		ReleaseResources();
	}

	CommandLineArgument& CommandLineArgument::operator=(const CommandLineArgument &other)
	{
		ReleaseResources();
		CopyObject(other);
		return *this;
	}

	bool CommandLineArgument::operator==(const CommandLineArgument &other)
	{
		if(other.value == value)
			return true;

		return false;
	}

	bool CommandLineArgument::GetHasData() const
	{
		return hasData;
	}

	const char* CommandLineArgument::GetData() const
	{
		return data;
	}

	bool CommandLineArgument::GenerateDebugString(char *buffer, int bufferSize)
	{
		int ret;
		memset(buffer, 0, bufferSize);

		if(!hasData)
			ret = snprintf(buffer, bufferSize, "Value: %c, hasData: FALSE, data: NULL", value);
		else
			ret = snprintf(buffer, bufferSize, "Value: %c, hasData: TRUE, data: %s", value, data);

		if((ret > 0) && ret < (bufferSize - 1))
			return true;

		return false;
	}

	char* CommandLineArgument::GetData()
	{
		return data;
	}

	char CommandLineArgument::GetValue() const
	{
		return value;
	}

	void CommandLineArgument::SetHasData(bool val)
	{
		hasData = val;
	}

	void CommandLineArgument::SetValue(char val)
	{
		value = val;
	}

	void CommandLineArgument::SetData(const char *val)
	{
		ReleaseResources();

		if(val != NULL)
		{
			data = new char[strlen(val) + 1];
			strcpy(data, val);
		}
	}

	void CommandLineArgument::ReleaseResources()
	{
		if(data != NULL)
		{
			delete[] data;
			data = NULL;
		}

		hasData = false;
	}

	void CommandLineArgument::CopyObject(const CommandLineArgument &other)
	{
		value = other.value;
		SetData(other.data);
		hasData = other.hasData;
	}

}

}
