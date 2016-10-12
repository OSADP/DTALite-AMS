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

#include "StdAfx.h"
#include "CSVParser.h"

#include <fstream>
#include <sstream>

using std::ifstream;
using std::istringstream;
extern void g_ProgramStop();

CCSVParser::CCSVParser(void)
{
	Delimiter = ',';
	IsFirstLineHeader = true;
}

CCSVParser::~CCSVParser(void)
{
	if (inFile.is_open()) inFile.close();
}


bool CCSVParser::OpenCSVFile(string fileName,bool b_required)
{
	mFileName = fileName;
	inFile.open(fileName.c_str());
	
	if (inFile.is_open())
	{
		if (IsFirstLineHeader)
		{
			string s;
			std::getline(inFile,s);
			vector<string> FieldNames = ParseLine(s);

			for (size_t i=0;i<FieldNames.size();i++)
			{
				string tmp_str = FieldNames.at(i);
				size_t start = tmp_str.find_first_not_of(" ");
				
				string name;
				if (start == string::npos)
				{
					name = "";
				}
				else
				{
					name = tmp_str.substr(start);
					TRACE("%s,",name.c_str ());
				}


				FieldsIndices[name] = (int) i;
			}
		}

		return true;
	}
	else
	{
		if(b_required)
		{
		
		cout << "File " << fileName << " does not exist. Please check."  << endl;
		g_ProgramStop();
		}
		return false;
	}
}

void CCSVParser::CloseCSVFile(void)
{
	inFile.close();
}

bool CCSVParser::ReadRecord()
{
	LineFieldsValue.clear();

	if (inFile.is_open())
	{
		string s;
		std::getline(inFile,s);
		if (s.length() > 0)
		{
			LineFieldsValue = ParseLine(s);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

vector<string> CCSVParser::ParseLine(string line)
{
	vector<string> SeperatedStrings;
	string subStr;
	istringstream ss(line);


	if (line.find_first_of('"') == string::npos)
	{

		while (std::getline(ss,subStr,Delimiter))
		{
			SeperatedStrings.push_back(subStr);
		}
	}
	else
	{
		while (line.length() > 0)
		{
			size_t n1 = line.find_first_of(',');
			size_t n2 = line.find_first_of('"');

			if (n1 == string::npos && n2 == string::npos) //last field without double quotes
			{
				subStr = line;
				SeperatedStrings.push_back(subStr);
				break;
			}

			if (n1 == string::npos && n2 != string::npos) //last field with double quotes
			{
				size_t n3 = line.find_first_of('"',n2+1); // second double quote

				//extract content from double quotes
				subStr = line.substr(n2+1, n3-n2-1);
				SeperatedStrings.push_back(subStr);

				break;
			}
	
			if (n1 != string::npos && (n1 < n2 || n2 == string::npos))
			{
					subStr = line.substr(0,n1);
					SeperatedStrings.push_back(subStr);
					if (n1 < line.length()-1)
					{
						line = line.substr(n1+1);
					}
					else // comma is the last char in the line string, push an empty string to the back of vector
					{
						SeperatedStrings.push_back("");
						break;
					}
			}

			if (n1 != string::npos && n2 != string::npos && n2 < n1)
			{
				size_t n3 = line.find_first_of('"',n2+1); // second double quote
				subStr = line.substr(n2+1, n3-n2-1);
				SeperatedStrings.push_back(subStr);
				size_t idx = line.find_first_of(',',n3+1);

				if (idx != string::npos)
				{
					line = line.substr(idx+1);
				}
				else
				{
					break;
				}
			}
		}

	}

	return SeperatedStrings;
}


///////////////////////////////////////




