//  Portions Copyright 2010 Xuesong Zhou, Jinjin Tang, Pengfei (Taylor) Li

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

// DTALite.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "DTALite.h"
#include "GlobalData.h"
#include "CSVParser.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

#define _max_number_of_movements  32
static string g_movement_column_name[_max_number_of_movements] = { "NBL2", "NBL", "NBT", "NBR", "NBR2",
"SBL2", "SBL", "SBT", "SBR", "SBR2",
"EBL2", "EBL", "EBT", "EBR", "EBR2",
"WBL2", "WBL", "WBT", "WBR", "WBR2",
"NEL", "NET", "NER",
"NWL", "NWT", "NWR",
"SEL", "SET", "SER",
"SWL", "SWT", "SWR" };



class CCSVSignalParser
{
public:
	char Delimiter;
	bool IsFirstLineHeader;
	ifstream inFile;
	vector<string> LineFieldsValue;
	vector<string> Headers;
	map<string, int> FieldsIndices;

	vector<int> LineIntegerVector;

public:
	void  ConvertLineStringValueToIntegers()
	{
		LineIntegerVector.clear();
		for (unsigned i = 0; i < LineFieldsValue.size(); i++)
		{
			std::string si = LineFieldsValue[i];
			int value = atoi(si.c_str());

			if (value >= 1)
				LineIntegerVector.push_back(value);

		}
	}
	vector<string> GetHeaderVector()
	{
		return Headers;
	}

	int m_EmptyLineCount;
	bool m_bSignalCSVFile;
	string m_DataHubSectionName;
	bool m_bLastSectionRead;

	bool m_bSkipFirstLine;  // for DataHub CSV files

	CCSVSignalParser::CCSVSignalParser(void)
	{
		Delimiter = ',';
		IsFirstLineHeader = true;
		m_bSkipFirstLine = false;
		m_bSignalCSVFile = false;
		m_bLastSectionRead = false;
		m_EmptyLineCount++;
	}

	CCSVSignalParser::~CCSVSignalParser(void)
	{
		if (inFile.is_open()) inFile.close();
	}


	bool CCSVSignalParser::OpenCSVFile(string fileName, bool bIsFirstLineHeader)
	{
		inFile.clear();
		inFile.open(fileName.c_str());

		IsFirstLineHeader = bIsFirstLineHeader;
		if (inFile.is_open())
		{
			if (m_bSkipFirstLine)
			{
				string s;
				std::getline(inFile, s);
			}
			if (IsFirstLineHeader)
			{
				string s;
				std::getline(inFile, s);

				if (s.length() == 0)
					return true;

				vector<string> FieldNames = ParseLine(s);

				for (size_t i = 0; i<FieldNames.size(); i++)
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
						TRACE("%s,", name.c_str());
					}
					Headers.push_back(name);
					FieldsIndices[name] = (int)i;
				}
			}

			return true;
		}
		else
		{
			return false;
		}
	}

	bool CCSVSignalParser::ReadSectionHeader(string s)
	{
		//skip // data 

		Headers.clear();
		FieldsIndices.clear();

		if (s.length() == 0)
			return true;

		vector<string> FieldNames = ParseLine(s);

		for (size_t i = 0; i<FieldNames.size(); i++)
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
				TRACE("%s,", name.c_str());
			}
			Headers.push_back(name);
			FieldsIndices[name] = (int)i;
		}


		return true;

	}
	void CCSVSignalParser::CloseCSVFile(void)
	{
		inFile.close();
	}

	vector<string> CCSVSignalParser::GetLineRecord()
	{
		return LineFieldsValue;
	}

	vector<string> CCSVSignalParser::GetHeaderList()
	{
		return Headers;
	}

	bool CCSVSignalParser::ReadRecord()
	{
		LineFieldsValue.clear();

		if (inFile.is_open())
		{
			string s;
			std::getline(inFile, s);
			if (s.length() > 0)
			{
				if (m_bSignalCSVFile && s.find("node_id") != string::npos)  // DataHub single csv file
				{
					LineFieldsValue = ParseLine(s);

					if (LineFieldsValue.size() >= 1 && LineFieldsValue[0].find("[") != string::npos)
					{
						m_DataHubSectionName = LineFieldsValue[0];

					}

					//re-read section header
					ReadSectionHeader(s);
					LineFieldsValue.clear();
					std::getline(inFile, s);

					LineFieldsValue = ParseLine(s);

				}
				else
				{
					LineFieldsValue = ParseLine(s);

				}
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

	vector<string> CCSVSignalParser::ParseLine(string line)
	{
		vector<string> SeperatedStrings;
		string subStr;
		istringstream ss(line);


		if (line.find_first_of('"') == string::npos)
		{

			while (std::getline(ss, subStr, Delimiter))
			{
				SeperatedStrings.push_back(subStr);
			}

			if (line.at(line.length() - 1) == ',')
			{
				SeperatedStrings.push_back("");
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
					size_t n3 = line.find_first_of('"', n2 + 1); // second double quote

					//extract content from double quotes
					subStr = line.substr(n2 + 1, n3 - n2 - 1);
					SeperatedStrings.push_back(subStr);

					break;
				}

				if (n1 != string::npos && (n1 < n2 || n2 == string::npos))
				{
					subStr = line.substr(0, n1);
					SeperatedStrings.push_back(subStr);
					if (n1 < line.length() - 1)
					{
						line = line.substr(n1 + 1);
					}
					else // comma is the last char in the line string, push an empty string to the back of vector
					{
						SeperatedStrings.push_back("");
						break;
					}
				}

				if (n1 != string::npos && n2 != string::npos && n2 < n1)
				{
					size_t n3 = line.find_first_of('"', n2 + 1); // second double quote
					subStr = line.substr(n2 + 1, n3 - n2 - 1);
					SeperatedStrings.push_back(subStr);
					size_t idx = line.find_first_of(',', n3 + 1);

					if (idx != string::npos)
					{
						line = line.substr(idx + 1);
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
	template <class T> bool GetValueBySectionKeyFieldName(string file_name, string section_name, string key_name, string field_name, T& value)
	{
		OpenCSVFile(file_name);
		while (ReadRecord())
		{
			if (LineFieldsValue[0] != section_name || LineFieldsValue[1] != key_name)
				continue;

			if (FieldsIndices.find(field_name) == FieldsIndices.end())
			{
				CloseCSVFile();
				return false;
			}
			else
			{
				if (LineFieldsValue.size() == 0)
				{
					CloseCSVFile();
					return false;
				}

				int size = (int)(LineFieldsValue.size());
				if (FieldsIndices[field_name] >= size)
				{
					CloseCSVFile();
					return false;
				}

				string str_value = LineFieldsValue[FieldsIndices[field_name]];

				if (str_value.length() <= 0)
				{
					CloseCSVFile();
					return false;
				}

				istringstream ss(str_value);

				T converted_value;
				ss >> converted_value;

				if (/*!ss.eof() || */ ss.fail())
				{

					CloseCSVFile();
					return false;
				}

				value = converted_value;
				CloseCSVFile();
				return true;
			}
		}
		CloseCSVFile();

		return false;
	}
	template <class T> bool GetValueByFieldName(string field_name, T& value, bool NonnegativeFlag = true)
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

			int size = (int)(LineFieldsValue.size());
			if (FieldsIndices[field_name] >= size)
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

			if (/*!ss.eof() || */ ss.fail())
			{
				return false;
			}

			if (NonnegativeFlag && converted_value<0)
				converted_value = 0;

			value = converted_value;
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

			unsigned int index = FieldsIndices[field_name];
			if (index >= LineFieldsValue.size())
			{
				return false;
			}
			string str_value = LineFieldsValue[index];

			if (str_value.length() <= 0)
			{
				return false;
			}

			value = str_value;
			return true;
		}
	}

};

using namespace std;

extern int g_number_of_warnings ;

void g_ProhibitMovement(int up_node_id, int node_id , int dest_node_id) 
{
			string movement_id = GetMovementStringID(up_node_id, node_id , dest_node_id);

			if(g_NodeNametoIDMap.find(node_id)== g_NodeNametoIDMap.end())
				return;
			int node_no = g_NodeNametoIDMap[node_id];
			g_NodeVector[node_no].m_MovementMap[movement_id].in_link_from_node_id = up_node_id;
			g_NodeVector[node_no].m_MovementMap[movement_id].in_link_to_node_id = node_id ; 
			g_NodeVector[node_no].m_MovementMap[movement_id].out_link_to_node_id = dest_node_id;

			g_NodeVector[node_no].m_MovementMap[movement_id].b_turning_prohibited = true;   // assign movement to individual node

			g_number_of_prohibited_movements++;
}

void g_ReadAMSMovementData()
{


	CCSVParser parser_movement;

	int count = 0;
	int zero_effective_green_time_error_count = 0;

	if (parser_movement.OpenCSVFile("AMS_movement.csv", false))  // not required
	{
		int up_node_id = 0;
		int dest_node_id = 0;


		while (parser_movement.ReadRecord())
		{
			int node_id = 0;
			parser_movement.GetValueByFieldName("node_id", node_id);
			if (g_NodeNametoIDMap.find(node_id) == g_NodeNametoIDMap.end())
				continue;  // skip this record


			std::string turn_type;

			parser_movement.GetValueByFieldName("turn_type", turn_type);

			std::string turn_direction;
			parser_movement.GetValueByFieldName("turn_direction", turn_direction);

			parser_movement.GetValueByFieldName("up_node_id", up_node_id);
			parser_movement.GetValueByFieldName("dest_node_id", dest_node_id);


			string strid = GetLinkStringID(up_node_id, node_id);
			if (g_LinkMap.find(strid) != g_LinkMap.end())
			{
				// map from turn direction to link id
				g_NodeVector[g_NodeNametoIDMap[node_id]].m_Movement2LinkIDStringMap[turn_direction] = strid;

			}

			int prohibited_flag = 0;

			parser_movement.GetValueByFieldName("prohibited_flag", prohibited_flag);


			if (prohibited_flag == 1)
			{
				g_ShortestPathWithMovementDelayFlag = true; // with movement input

				g_ProhibitMovement(up_node_id, node_id, dest_node_id);


				continue; // do not need to check further 
			}

		}

		parser_movement.CloseCSVFile();
	}

	
}

void g_ReadAMSSignalData()
{

	/* read data block per node:
	 movement-specific phase number, effective green time 
	 movement-specific green start time according to standard dual ring sequence

	 look up: 

	 step 1 
	 update node-specific cycle time using AMS_Movement's cycle tim

	 step 2:
	 when read through turn movement, we overwrite link-based m_EffectiveGreenTime_In_Second and m_EffectiveGreenTime_In_Second
		pLink->m_EffectiveGreenTime_In_Second ,
		pLink->m_GreenStartTime_In_Second,  
	 update saturation flow rate:


	 step 3:
	 if this a left turn movment and left-turn # of lanes >0
	m_LeftTurn_EffectiveGreenTime_In_Second
	*/


	CCSVSignalParser parser_signal;

	int count = 0;
	int zero_effective_green_time_error_count = 0;

	if (parser_signal.OpenCSVFile("AMS_signal_control.csv", false))  // not required
	{
		int up_node_id = 0;
		int dest_node_id = 0;
		parser_signal.m_bSignalCSVFile = true;

		while (parser_signal.ReadRecord())
		{
			int up_node_id, node_id, dest_node_id;

			std::string parameter_key;


			int CycleLength = 0;
			int Offset = 0;

			parser_signal.GetValueByFieldName("node_id", node_id);

			if (g_NodeNametoIDMap.find(node_id) == g_NodeNametoIDMap.end())
				continue;  // skip this record


			parser_signal.GetValueByFieldName("key", parameter_key);

			if (parameter_key == "cycle_length")
			{
				parser_signal.GetValueByFieldName("value", CycleLength);
				g_NodeVector[g_NodeNametoIDMap[node_id]].m_CycleLength_In_Second = CycleLength;

			}

			if (parameter_key == "offset")
			{
				parser_signal.GetValueByFieldName("value", Offset);
				g_NodeVector[g_NodeNametoIDMap[node_id]].m_SignalOffset_In_Second = Offset;

			}

			if (parameter_key == "start_time" || parameter_key == "end_time")
			{

				for (int m = 0; m < _max_number_of_movements; m++)
				{
					float value = 0;
					if (parser_signal.GetValueByFieldName(g_movement_column_name[m], value, true) == true)
					{
						std::string strid;
						strid = g_NodeVector[g_NodeNametoIDMap[node_id]].m_Movement2LinkIDStringMap[g_movement_column_name[m]];

						// find link
						DTALink *pLink = NULL;

						if (strid.size() >= 1 && g_LinkMap.find(strid) != g_LinkMap.end())
						{
							pLink = g_LinkMap[strid];
							pLink->m_bSignalizedArterialType = true;

						}
						else
						{
							continue;   // output error message later
						}


						if (parameter_key == "start_time")
						{
							if (g_movement_column_name[m].find("T") != string::npos)
							{
								pLink->m_GreenStartTime_In_Second = value;  // movement start time;
							}
							if (g_movement_column_name[m].find("L") != string::npos)
							{
								pLink->m_LeftTurnGreenStartTime_In_Second = value;  // movement start time;
							}
						}

						if (parameter_key == "end_time")
						{
							if (g_movement_column_name[m].find("T") != string::npos)
							{
								pLink->m_EffectiveGreenTime_In_Second = max(0, value - pLink->m_GreenStartTime_In_Second);  // consider the lost time for permitted phases.
							}

							if (g_movement_column_name[m].find("L") != string::npos)
							{
								pLink->m_LeftTurn_EffectiveGreenTime_In_Second = max(0, value - pLink->m_GreenStartTime_In_Second);  // consider the lost time for permitted phases.
							}
						}


					}
				}
			}

		}
	}
}

	

