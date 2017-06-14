#ifndef __COMMANDLINEARGUMENT_HPP__
#define __COMMANDLINEARGUMENT_HPP__

namespace SystemToolkit
{

namespace Types
{

	class CommandLineArgument
	{
	public:
		CommandLineArgument();
		CommandLineArgument(char);
		CommandLineArgument(char, bool, const char*);
		CommandLineArgument(const CommandLineArgument&);
		~CommandLineArgument();

		CommandLineArgument& operator=(const CommandLineArgument&);
		bool operator==(const CommandLineArgument&);

		bool GetHasData() const;
		char GetValue() const;
		const char* GetData() const;
		char* GetData();

		void SetHasData(bool);
		void SetValue(char);
		void SetData(const char*);

		bool GenerateDebugString(char*, int);

	private:
		void CopyObject(const CommandLineArgument&);
		void ReleaseResources();

		bool hasData;
		char value;
		char *data;
	};

}

}

#endif
