//  Portions Copyright 2010 Hao Lei, Xuesong Zhou

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of DTALite.

//    DTALite is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    DTALite is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with DTALite.  If not, see <http://www.gnu.org/licenses/>.
#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>

using std::string;
using std::ifstream;
using std::vector;
using std::map;
using std::istringstream;
using std::ostringstream;

#include <iostream>
#include <fstream>
using namespace std;

template <typename T>
string NumberToString ( T Number )
{
	ostringstream ss;
	ss << Number;
	return ss.str();
}




class CCSVParser
{
public : ifstream inFile;

	 string mFileName;
	 vector<int> LineIntegerVector;
	bool IsFirstLineHeader;
	char Delimiter;

private:
	vector<string> LineFieldsValue;

	map<string,int> FieldsIndices;

	vector<string> ParseLine(string line);

public:

	void  ConvertLineStringValueToIntegers()
	{
		LineIntegerVector.clear();
		for(unsigned i = 0; i < LineFieldsValue.size(); i++)
		{
			std::string si = LineFieldsValue[i];
			int value = atoi(si.c_str ());

			if(value>=1)
				LineIntegerVector.push_back(value);

		}
	}

	CCSVParser(void);
	bool OpenCSVFile(string fileName, bool b_required = true);
	void CloseCSVFile(void);
	bool ReadRecord();

	template <class T> bool GetValueByFieldNameRequired(string field_name, T& value)
	{
		bool required_field = true;
		bool print_out = false;
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			if(required_field)
			{
				cout << "Field " << field_name << " in file " << mFileName << " does not exist. Please check the file."  << endl;

				g_ProgramStop();
			}
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			if(FieldsIndices[field_name] >= LineFieldsValue.size())  // no value is read for index FieldsIndices[field_name]
			{
				return false;
			}
			string str_value = LineFieldsValue[FieldsIndices[field_name]];

			if (str_value.length() <= 0)
			{
				return false;
			}

			istringstream ss(str_value);

			T converted_value;
			ss >> converted_value;

			if(print_out)
			{
				cout << "Field " << field_name << " = " << converted_value << endl;
			}

			if (/*!ss.eof() || */ ss.fail())
			{
				return false;
			}

			value = converted_value;
			return true;
		}
	}


	template <class T> bool GetValueByFieldNameWithPrintOut(string field_name, T& value)
	{
		bool required_field = true;
		bool print_out = true;
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			if(required_field)
			{
				cout << "Field " << field_name << " in File " << mFileName << " does not exist."  << endl;

				g_ProgramStop();
			}
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			if(FieldsIndices[field_name] >= LineFieldsValue.size())  // no value is read for index FieldsIndices[field_name]
			{
				cout << "Missing value for " << field_name << " in File " << mFileName <<  endl;

				return false;
			}
			string str_value = LineFieldsValue[FieldsIndices[field_name]];

			if (str_value.length() <= 0)
			{
				return false;
			}

			istringstream ss(str_value);

			T converted_value;

			ss >> std::noskipws;
			ss >> converted_value;

			if(print_out)
			{
				cout << "Field " << field_name << " = " << converted_value << endl;
			}

			if (/*!ss.eof() || */ ss.fail())
			{
				return false;
			}

			value = converted_value;
			return true;
		}
	}

	template <class T> bool GetValueByFieldName(string field_name, T& value, bool with_whitespace = false)
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			if(FieldsIndices[field_name] >= LineFieldsValue.size())  // no value is read for index FieldsIndices[field_name]
			{
				return false;
			}
			string str_value = LineFieldsValue[FieldsIndices[field_name]];

			if (str_value.length() <= 0)
			{
				return false;
			}

			istringstream ss(str_value);

			if(with_whitespace)
			{
			ss >> std::noskipws;
			}
			T converted_value;
			ss >> converted_value;

			if (/*!ss.eof() || */ ss.fail())
			{
				return false;
			}

			value = converted_value;

			return true;
		}
	}

	bool GetValueByFieldNameWithPrintOut(string field_name, string& value)
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			int index = FieldsIndices[field_name];
			int size = LineFieldsValue.size();

			if(FieldsIndices[field_name]>= LineFieldsValue.size())
			{
				return false;
			}
			if (LineFieldsValue[FieldsIndices[field_name]].length() <= 0)
			{
				return false;
			}
			string str_value = LineFieldsValue[FieldsIndices[field_name]];



			value = str_value;
			return true;
		}
	}

	bool GetValueByFieldName(string field_name, string& value)
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{
			if (LineFieldsValue.size() == 0)
			{
				return false;
			}

			int index = FieldsIndices[field_name];
			int size = LineFieldsValue.size();

			if(FieldsIndices[field_name]>= LineFieldsValue.size())
			{
				return false;
			}
			if (LineFieldsValue[FieldsIndices[field_name]].length() <= 0)
			{
				return false;
			}
			string str_value = LineFieldsValue[FieldsIndices[field_name]];



			value = str_value;
			return true;
		}
	}

	~CCSVParser(void);
};

class CCSVWriter
{
public : 
	ofstream outFile;
	char Delimiter;
	int FieldIndex;
	bool IsFirstLineHeader;
	map<int,string> LineFieldsValue;
	vector<string> LineFieldsName;
	vector<string> LineFieldsCategoryName;
	map<string,int> FieldsIndices;  

	bool row_title;

public:
	void SetRowTitle(bool flag)
	{
		row_title = flag;
	}

	bool OpenCSVFile(string fileName, bool b_required=true);
	void CloseCSVFile(void);
	template <class T> bool SetValueByFieldName(string field_name, T& value)  // by doing so, we do not need to exactly follow the sequence of field names
	{
		if (FieldsIndices.find(field_name) == FieldsIndices.end())
		{
			return false;
		}
		else
		{

			LineFieldsValue[FieldsIndices[field_name]] = NumberToString(value);

			return true;
		}
	}

	void Reset()
	{

		LineFieldsValue.clear();
		LineFieldsName.clear();
		LineFieldsCategoryName.clear();
		FieldsIndices.clear();

	}
	void SetFieldName(string field_name)
	{ 
		FieldsIndices[field_name] = LineFieldsName.size();
		LineFieldsName.push_back (field_name);
		LineFieldsCategoryName.push_back(" ");

	}

	void SetFieldNameWithCategoryName(string field_name,string category_name)
	{ 
		FieldsIndices[field_name] = LineFieldsName.size();
		LineFieldsName.push_back (field_name);
		LineFieldsCategoryName.push_back(category_name);

	}


	void WriteTextString(CString textString)
	{
		if (!outFile.is_open()) 
			return;
		outFile << textString << endl;

	}

	void WriteTextLabel(CString textString)
	{
		if (!outFile.is_open()) 
			return;
		outFile << textString;

	}

	template <class T>  void WriteNumber(T value)
	{
		if (!outFile.is_open()) 
			return;
		outFile << NumberToString(value) << endl;
	}

	template <class T>  void WriteParameterValue(CString textString, T value)
	{
		if (!outFile.is_open()) 
			return;

		outFile << textString <<"=,"<< NumberToString(value) << endl;
	}

	void WriteNewEndofLine()
	{
		if (!outFile.is_open()) 
			return;
		outFile << endl;
	}


	void WriteHeader(bool bCategoryNameLine = true, bool bRowTitle = true)
	{
		if (!outFile.is_open()) 
			return;


		if(bCategoryNameLine == true)
		{
			for(unsigned int i = 0; i< FieldsIndices.size(); i++)
			{
				outFile << LineFieldsCategoryName[i] << ",";
			}
			outFile << endl;

		}

		if(bRowTitle == true)
			outFile << ",";

		for(unsigned int i = 0; i< FieldsIndices.size(); i++)
		{
			outFile << LineFieldsName[i] << ",";
		}

		outFile << endl;
	}
	void WriteRecord()
	{
		if (!outFile.is_open()) 
			return;

		for(unsigned int i = 0; i< FieldsIndices.size(); i++)
		{
			string str ;
			if(LineFieldsValue.find(i) != LineFieldsValue.end()) // has been initialized
				outFile << LineFieldsValue[i].c_str () << ",";
			else
				outFile << ' ' << ",";
		}

		LineFieldsValue.clear();

		outFile << endl;
	}

	CCSVWriter::CCSVWriter()
	{
		row_title = false;
		FieldIndex = 0;
		Delimiter = ',';
		IsFirstLineHeader = true;
	}

	CCSVWriter::~CCSVWriter(void)
	{
		if (outFile.is_open()) outFile.close();
	}


	CCSVWriter::CCSVWriter(string fileName)
	{
		Open(fileName);

	};

	void CCSVWriter::Open(string fileName)
	{
		outFile.open(fileName.c_str());

		if (outFile.is_open()==false)
		{
			cout << "File " << fileName.c_str() << " cannot be opened." << endl;
			getchar();
			exit(0);
		}

	};

	void CCSVWriter::OpenAppend(string fileName)
	{
		outFile.open(fileName.c_str(), fstream::app);

		if (outFile.is_open()==false)
		{
			cout << "File " << fileName.c_str() << " cannot be opened." << endl;
			getchar();
			exit(0);
		}

	};
};



