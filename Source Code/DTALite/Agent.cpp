#include "stdafx.h"
#include <stdlib.h>
#include <crtdbg.h>

#include "DTALite.h"


#include "Geometry.h"
#include "GlobalData.h"
#include "CSVParser.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

using namespace std;

int g_FindNodeNumberWithCoordinate(double x, double y, double min_distance = 0.1)
{
	int NodeNumber = -1;
	for (int i = 0; i < g_NodeVector.size(); i++)
	{

		double distance = sqrt((g_NodeVector[i].m_pt.x - x)*(g_NodeVector[i].m_pt.x - x) + (g_NodeVector[i].m_pt.y - y)*(g_NodeVector[i].m_pt.y - y));
		if (distance <  min_distance)
		{
			min_distance = distance;
			NodeNumber = g_NodeVector[i].m_NodeNumber;
		}
	}

	return NodeNumber;

}


void g_AddVehicleID2ListBasedonDepartureTime(DTAVehicle * pVehicle)
{

	int simulation_time_no = (int)(pVehicle->m_DepartureTime * 10);
	g_OriginalVehicleTDListMap[simulation_time_no].m_AgentIDVector.push_back(pVehicle->m_AgentID);


}
void g_AllocateDynamicArrayForVehicles()
{
	if (g_TDOVehicleArray == NULL)  // has not allocated memory yet
	{
		_proxy_ABM_log(0, "Allocate memory for %d zones and %d SP calculation intervals.\n",

			g_ZoneMap.size(), g_NumberOfSPCalculationPeriods);

		g_TDOVehicleArray = AllocateDynamicArray<VehicleArrayForOriginDepartrureTimeInterval>(g_ZoneMap.size(), g_NumberOfSPCalculationPeriods);

	}
}

vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}


vector<float> ParseLineToFloat(string line)
{
	vector<float> SeperatedValues;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedValues.push_back(integer);
	}
	return SeperatedValues;
}

bool AddPathToVehicle(DTAVehicle * pVehicle, std::vector<int> path_node_sequence, CString FileName)
{

	if (pVehicle->m_NodeSize >= 1 && pVehicle->m_LinkAry != NULL)
	{
		delete pVehicle->m_LinkAry;
	}

	pVehicle->m_NodeSize = path_node_sequence.size();

	if (pVehicle->m_NodeSize >= 1)  // in case reading error
	{
		pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];
		pVehicle->m_NodeNumberSum = 0;

		pVehicle->m_Distance = 0;  // reset distanace when there are new paths assigned. 
		for (int i = 0; i < pVehicle->m_NodeSize; i++)
		{

			int node_id;
			float event_time_stamp, travel_time, emissions;

			pVehicle->m_NodeNumberSum += path_node_sequence[i];

			if (i == 0)
				pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

			if (i == pVehicle->m_NodeSize - 1)
				pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

			if (i >= 1)
			{
				DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
				if (pLink == NULL && FileName.GetLength() > 0)
				{
					CString msg;
					msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], pVehicle->m_AgentID, FileName);
					cout << msg << endl;

					return false;
				}

				pVehicle->m_Distance += pLink->m_Length;

				pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
			}


		}

	}
	return true;
}


bool g_UseExternalPathDefinedInRoutingPolicy(DTAVehicle* pVehicle)
{

	if (g_ODPathSetVector == NULL)
		return false;

	int OrgZoneSequentialNo = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
	int DestZoneSequentialNo = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

	float random_value = pVehicle->GetRandomRatio();

	int information_type = pVehicle->m_InformationType;

	if (information_type >= 2) // for enroute and pretrip infor users, we do not have information yet, so we default their paths to the learning from the previous day
		information_type = 1;

	// loop through the path set
	if (g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size() >= 1)
	{
		int i = 0;
		for (i = 0; i < g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size(); i++)
		{

			if (random_value <= g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].CumulativeRatio)
			{
				break;
			}

		}

		if (i < 0)
			i = 0;

		if (i == g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size())
			i = g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size() - 1;

		_proxy_ABM_log(0, "--apply routing policy for agent %d from zone %d to zone %d information type %d\n", 
			pVehicle->m_AgentID ,
			pVehicle->m_OriginZoneID,
			pVehicle->m_DestinationZoneID,
			information_type);

		for (int n = 0; n < g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].m_NodeNumberArray.size(); n++)
		{
			_proxy_ABM_log(0, "node %d\n", g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].m_NodeNumberArray[n]);


		}
		AddPathToVehicle(pVehicle, g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].m_NodeNumberArray, NULL);

		return true;
	}
	return false;
}

bool g_ReadTripCSVFile(string file_name, bool bOutputLogFlag)
{
	int	LineCount = 0;

	bool bOutputDebugLogFile = true;

	bool bUpdatePath = false;

	g_AllocateDynamicArrayForVehicles();
	float start_time_value = -100;

	CCSVParser parser_agent;

	int dependency_agent_reading_percentage = g_GetPrivateProfileInt("ABM_integration", "dependency_agent_reading_percentage",0, g_DTASettingFileName);

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name, false))
	{

		if (bOutputLogFlag)
		{
			cout << "reading file " << file_name << endl;

		}
		if (bOutputDebugLogFile)
			fprintf(g_DebugLogFile, "reading file %s\n", file_name.c_str());
		_proxy_ABM_log(0, "**step 0: reading file %s\n", file_name.c_str());

		int line_no = 1;

		int i = 0;

		int count = 0;

		int count_for_sameOD = 0;
		int count_for_not_defined_zones = 0;
		int information_type = 0;
		while (parser_agent.ReadRecord())
		{
			//------------QU 10.30------------------//
			//---initializing for checking status---//
			bUpdatePath = false;
			start_time_value = -100;
			//--------------------------------------//

			if ((count + 1) % 1000 == 0)
			{
				cout << "reading " << count + 1 << " records..." << endl;

			}
			count++;

			int agent_id = 0;

			parser_agent.GetValueByFieldNameRequired("agent_id", agent_id);

			if (g_DemandCapacityScalingFactor < 0.999 && g_DemandCapacityScalingFactor >= 0.0001)  // valid ratio
			{
				double random_ratio = g_GetRandomRatio();

				if (random_ratio > g_DemandCapacityScalingFactor)
					continue; // skip reading 
			}

			if (agent_id == 561)
				TRACE("");

			_proxy_ABM_log(0, "--step 1: read agent id = %d \n", agent_id);

			DTAVehicle* pVehicle = 0;

			bool bCreateNewAgent = false;

			if (g_VehicleMap.find(agent_id) != g_VehicleMap.end())
			{
				_proxy_ABM_log(0, "--step 2: agent id =%d found in memory, will update data\n", agent_id);

				pVehicle = g_VehicleMap[agent_id];
				bCreateNewAgent = false;
			}
			else
			{

				pVehicle = new (std::nothrow) DTAVehicle;
				if (pVehicle == NULL)
				{
					cout << "Insufficient memory...";
					getchar();
					exit(0);

				}
				pVehicle->m_AgentID = agent_id;
				_proxy_ABM_log(0, "--step 2: new agent id = %d, the current size of agent vector= %d \n", agent_id, g_VehicleVector.size());

				bCreateNewAgent = true;

			}


			// additional error checking for updating agent data
			int ExternalTourID = 0;
			parser_agent.GetValueByFieldNameRequired("tour_id", ExternalTourID);

			_proxy_ABM_log(0, "--step 3: read tour_id id = %d \n", ExternalTourID);

			if (ExternalTourID >= 0)
			{
				pVehicle->m_ExternalTourID = ExternalTourID;
			}


			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			int from_zone_id = pVehicle->m_OriginZoneID;
			int to_zone_id = pVehicle->m_DestinationZoneID;

			parser_agent.GetValueByFieldNameRequired("from_zone_id", from_zone_id);
			parser_agent.GetValueByFieldNameRequired("to_zone_id", to_zone_id);

			int origin_node_id = -1;
			int origin_node_number = -1;

			parser_agent.GetValueByFieldNameRequired("from_origin_node_id", origin_node_number);

			int destination_node_id = -1;
			int destination_node_number = -1;
			parser_agent.GetValueByFieldNameRequired("to_destination_node_id", destination_node_number);


			//add special condition:
			if (from_zone_id == -1 && to_zone_id == -1 && origin_node_number == -1 && destination_node_number == -1)
			{
				double origin_node_x, origin_node_y, destination_node_x, destination_node_y;

				parser_agent.GetValueByFieldName("origin_node_x", origin_node_x);
				parser_agent.GetValueByFieldName("origin_node_y", origin_node_y);
				parser_agent.GetValueByFieldName("destination_node_x", destination_node_x);
				parser_agent.GetValueByFieldName("destination_node_y", destination_node_y);

				origin_node_number = g_FindNodeNumberWithCoordinate(origin_node_x, origin_node_y);
				destination_node_number = g_FindNodeNumberWithCoordinate(destination_node_x, destination_node_y);

				if (origin_node_number < 0 || destination_node_number < 0)
					continue;  //skip this record

				from_zone_id = origin_node_number;

				to_zone_id = destination_node_number;
			}


	


			if (pVehicle->m_OriginZoneID == -1)  // new vehicle
				pVehicle->m_OriginZoneID = from_zone_id;

			if (pVehicle->m_DestinationZoneID == -1) //new vehicle
				pVehicle->m_DestinationZoneID = to_zone_id;


			_proxy_ABM_log(0, "--step 4: read from_zone_id = %d, to_zone_id=%d \n", from_zone_id, to_zone_id);

			if (g_ZoneMap.find(from_zone_id) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;
				_proxy_ABM_log(0, "--step 4.1: from_zone_id = %d not defined, error.\n", from_zone_id);

				cout << "--step 4.1: from_zone_id =" << from_zone_id << "not defined, exit" << endl;


				continue;
			}

			if (g_ZoneMap.find(to_zone_id) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;
				_proxy_ABM_log(0, "--step 4.1: to_zone_id=%d not defined, error\n", to_zone_id);

				cout << "--step 4.1: to_zone_id =" << to_zone_id << "not defined, exit" << endl;

				continue;
			}
			// to do: update origin only when vehicle has not departed yet
			if (pVehicle->m_OriginZoneID != from_zone_id)
			{
				g_LogFile << " UPDATE Agent Data: origin zone =  " << pVehicle->m_OriginZoneID << "-> " << from_zone_id << endl;

				_proxy_ABM_log(0, "--step 4.2: update from_zone_id = %d->%d \n", pVehicle->m_OriginZoneID, from_zone_id);
				pVehicle->m_OriginZoneID = from_zone_id;
			}

			//to do: update destination only when vehicle has not reached the destination
			if (pVehicle->m_DestinationZoneID != to_zone_id)
			{
				g_LogFile << " UPDATE Agent Data: destination zone =  " << pVehicle->m_DestinationZoneID << "-> " << to_zone_id << endl;
				_proxy_ABM_log(0, "--step 4.2: update to_zone_id = %d->%d \n", pVehicle->m_DestinationZoneID, to_zone_id);
				pVehicle->m_DestinationZoneID = to_zone_id;
			}

			if (g_NodeNametoIDMap.find(origin_node_number) != g_NodeNametoIDMap.end())  // convert node number to internal node id
			{
				origin_node_id = g_NodeNametoIDMap[origin_node_number];
			}
			if (g_NodeNametoIDMap.find(destination_node_number) != g_NodeNametoIDMap.end()) // convert node number to internal node id
			{
				destination_node_id = g_NodeNametoIDMap[destination_node_number];
			}

			_proxy_ABM_log(0, "--step 5: read origin_node_id = %d, destination_node_id=%d \n",
				origin_node_number, destination_node_number);

			if (origin_node_id == -1)  // no default origin node value, re-generate origin node
			{
				origin_node_id = g_ZoneMap[pVehicle->m_OriginZoneID].GetRandomOriginNodeIDInZone((pVehicle->m_AgentID % 100) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily
			}
			if (destination_node_id == -1)// no default destination node value, re-destination origin node
				destination_node_id = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_AgentID % 100) / 100.0f);

			if (pVehicle->m_OriginNodeID != -1 && pVehicle->m_OriginNodeID != origin_node_id)
			{
				g_LogFile << " UPDATE Agent Data: origin node =  " << pVehicle->m_OriginNodeID << "-> " << origin_node_id << endl;


				_proxy_ABM_log(0, "--step 5.2 update origin_node_id = %d->%d \n",
					pVehicle->m_OriginNodeID, origin_node_id);

				bUpdatePath = true;
			}


			if (pVehicle->m_DestinationNodeID != -1 && pVehicle->m_DestinationNodeID != destination_node_id)
			{
				g_LogFile << " UPDATE Agent Data: destination node =  " << pVehicle->m_DestinationNodeID << "-> " << destination_node_id << endl;

				_proxy_ABM_log(0, "--step 5.2: update destination_node_id = %d->%d \n",
					pVehicle->m_DestinationNodeID, destination_node_id);

				bUpdatePath = true;

			}

			// for input or update data or not, we all reset the origin_node_id and destination_node_id
			pVehicle->m_OriginNodeID = origin_node_id;
			pVehicle->m_DestinationNodeID = destination_node_id;


			if (origin_node_id == destination_node_id)
			{  // do not simulate intra zone traffic
				_proxy_ABM_log(0, "--step 5.3: found intrazone traffic %d->%d, not simulated\n",
					origin_node_id, destination_node_id);

				count_for_sameOD++;
				continue;
			}

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}

			float departure_time = 0;
			if (parser_agent.GetValueByFieldNameRequired("departure_time_in_min", departure_time) == true)
			{
				_proxy_ABM_log(0, "--step 6: read departure_time = %.2f\n",
					departure_time);

				if (start_time_value < 0)  // set first value
					start_time_value = departure_time;
				else if (start_time_value > departure_time + 0.00001)  // check if the departure times are sequential
				{
					departure_time = start_time_value; // use a larger value 
					start_time_value = departure_time;
				}

				if (pVehicle->m_DepartureTime <= -1)
				{
					pVehicle->m_DepartureTime = departure_time;  // new vehicle

				}
				else if (fabs(pVehicle->m_DepartureTime - departure_time) > 0.5)
				{
					g_LogFile << " UPDATE Agent Data: departure time=  " << pVehicle->m_DepartureTime << "-> " << departure_time << endl;
					_proxy_ABM_log(0, "--step 6.2: update departure_time = %.2f->%.2f\n",
						pVehicle->m_DepartureTime, departure_time);

					bUpdatePath = true;

					//remove vehicle id for the old departure time slot

					int int_to_remove = pVehicle->m_AgentID;
					g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.erase(std::remove(g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.begin(), g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.end(), int_to_remove), g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.end());


					//add vehicle id for the new departure time slot
					g_VehicleTDListMap[departure_time * 10].m_AgentIDVector.push_back(pVehicle->m_AgentID);

					pVehicle->m_DepartureTime = departure_time;
					pVehicle->m_PreferredDepartureTime = departure_time;


				}

			}



			int beginning_departure_time = departure_time;

			ASSERT(pVehicle->m_DepartureTime < 4000);

			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: agent_id " << agent_id << " in file " << file_name << " has a start time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt";
				g_ProgramStop();
			}

			int demand_type = pVehicle->m_DemandType;
			if (parser_agent.GetValueByFieldName("demand_type", demand_type) == true)
			{
				if (pVehicle->m_DemandType != demand_type)
					pVehicle->m_DemandType = demand_type;

				_proxy_ABM_log(0, "--step 7: read demand_type = %d\n",
					demand_type);

				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationType, pVehicle->m_VOT, pVehicle->m_Age);

				// if there are values in the file, then update the related attributes; 
				int VOT = 0;
				int DemandType = 0;
				int VehicleType = 0;



				parser_agent.GetValueByFieldNameRequired("vehicle_type", VehicleType);

				_proxy_ABM_log(0, "--step 8: read vehicle_type = %d\n",
					VehicleType);

				if (VehicleType >= 1)
				{

					pVehicle->m_VehicleType = VehicleType;
				}

				information_type = pVehicle->m_InformationType;

				parser_agent.GetValueByFieldNameRequired("information_type", information_type); //default is 0;

				_proxy_ABM_log(0, "--step 8: read information_type = %d\n",
					information_type);
				if (information_type != pVehicle->m_InformationType)
				{
					_proxy_ABM_log(0, "--step 8.2: update information_type = %d->%d\n",
						pVehicle->m_InformationType, information_type);

					g_LogFile << " UPDATE Agent Data: information type =  " << pVehicle->m_InformationType << "-> " << information_type << endl;

				}

				if (pVehicle->m_InformationType == 3)  // enroute info
				{

					double time_to_start_information_retrieval = -1.0;
					parser_agent.GetValueByFieldName("time_to_start_information_retrieval", time_to_start_information_retrieval); //default is -1;

					if (time_to_start_information_retrieval >= 0)
					{
						pVehicle->m_TimeToRetrieveInfo = time_to_start_information_retrieval;
					}

					pVehicle->m_EnrouteInformationUpdatingTimeIntervalInMin = g_information_updating_interval_in_min;

					double information_updating_interval_in_min = -1.0;
					parser_agent.GetValueByFieldName("information_updating_interval_in_min", information_updating_interval_in_min); //default is -1;

					if (information_updating_interval_in_min >= 0)
					{
						pVehicle->m_EnrouteInformationUpdatingTimeIntervalInMin = information_updating_interval_in_min;
					}

				}

				parser_agent.GetValueByFieldNameRequired("value_of_time", VOT);

				_proxy_ABM_log(0, "--step 9: read value_of_time = %d\n", VOT);


				int dependency_agent_id = -1;

				int line_no_within_100 = line_no % 100 + 1;


				if( line_no_within_100 <= dependency_agent_reading_percentage)
				{

					parser_agent.GetValueByFieldName("dependency_agent_id", dependency_agent_id);
					float duration_in_min = -1;

					if (dependency_agent_id >= 0)
					{
						parser_agent.GetValueByFieldNameRequired("duration_in_min", duration_in_min);

						if (dependency_agent_id >= 0 && duration_in_min >= 0)
						{
							pVehicle->m_dependency_agent_id = dependency_agent_id;
							pVehicle->m_duration_in_min = duration_in_min;
						}
					}

				}
	



				if (VOT >= 1)  // only with valid value
					pVehicle->m_VOT = VOT;

				parser_agent.GetValueByFieldNameRequired("vehicle_age", pVehicle->m_Age);

				_proxy_ABM_log(0, "--step 10: read vehicle_age = %d\n", pVehicle->m_Age);

			}
			else
			{


			}

			int sub_path_switch_flag = 0;


			//-----QU 10.30-------------------------------------------------------------------------------------------//
			//====feasible path checking==========//
			std::vector<int> path_node_sequence;
			string path_node_sequence_str;
			parser_agent.GetValueByFieldNameRequired("path_node_sequence", path_node_sequence_str);

			path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

			if (!g_IsPathNodeSequenceAFeasiblePath(path_node_sequence))
			{
				cout << "Errors in " << file_name << ": the path node sequence of (agent_id = " << agent_id
					<< ", tour_id = " << ExternalTourID << ") is infeasible, please check!" << endl;
				g_ProgramStop();
			}

			if (path_node_sequence.size() >= 2)
			{
				if (origin_node_number != path_node_sequence[0] || destination_node_number != path_node_sequence[path_node_sequence.size() - 1])
				{
					cout << "Errors in " << file_name << ": the path node sequence of (agent_id = " << agent_id
						<< ", tour_id = " << ExternalTourID << ") is infeasible, because the origin/destination node does not equal to the first/last node in path_node_sequence, please check!" << endl;
					g_ProgramStop();
				}
			}
			//------------------------------------//
			//--------------------------------------------------------------------------------------------------------//

			if (bCreateNewAgent == true)
			{

				pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
				pVehicle->m_ArrivalTime = 0;
				pVehicle->m_bComplete = false;
				pVehicle->m_bLoaded = false;
				pVehicle->m_TollDollarCost = 0;
				pVehicle->m_Distance = 0;

				pVehicle->m_NodeSize = 0;

				pVehicle->m_NodeNumberSum = 0;
				pVehicle->m_Distance = 0;

				pVehicle->m_InformationType = information_type;

				//add vehicle data into memory
				g_VehicleVector.push_back(pVehicle);

				if (pVehicle->m_dependency_agent_id >= 0 && pVehicle->m_duration_in_min >= 0)
				{
					if (g_VehicleMap.find(pVehicle->m_dependency_agent_id) != g_VehicleMap.end())
					{
						g_VehicleMap[pVehicle->m_dependency_agent_id]->m_following_agent_id = pVehicle->m_AgentID;
					}
					else
					{
						cout << "dependency_agent_id =" << pVehicle->m_dependency_agent_id << "; duration_in_min = " << pVehicle->m_duration_in_min << ", which are incorrect in input agent files." << endl;
						g_ProgramStop();

					}

				}else
				{
					g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.push_back(pVehicle->m_AgentID);
				}
				
			

				g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

				g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
				int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);
				ASSERT(pVehicle->m_OriginZoneID <= g_ODZoneNumberSize);
				g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);


				_proxy_ABM_log(0, "--step 11: create new path\n");


				if (path_node_sequence.size() >= 2)
				{
					//-----QU 10.30-------------------------------------------------------------------------------------------//
					_proxy_ABM_log(0, "--step 11.1: read and add path_node_sequence = %s\n", path_node_sequence_str.c_str());
					AddPathToVehicle(pVehicle, path_node_sequence, file_name.c_str());
					//--------------------------------------------------------------------------------------------------------//

				}
				else
				{
					//-----QU 10.30------------to do list: information type and new path--------------------------------------//

					if (pVehicle->m_InformationType == info_hist_based_on_routing_policy || pVehicle->m_InformationType == learning_from_hist_travel_time)
					{
						g_UseExternalPathDefinedInRoutingPolicy(pVehicle);
					}
					else // pVehicle >=2
					{
						g_UpdateAgentPathBasedOnNewDestinationOrDepartureTime(pVehicle->m_AgentID);
					}

					_proxy_ABM_log(0, "--step 11.2: no routing policy, create shortest path based on prevailing traffic time\n");
					//--------------------------------------------------------------------------------------------------------//

				}
				int time_interval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

				if (g_ODEstimationFlag == 1) // having hist od only unde ODME mode
				{
					g_SystemDemand.AddValue(pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, time_interval, 1); // to store the initial table as hist database
				}
			}
			else if (bUpdatePath) //not new 
			{
				//-----QU 10.30-----------------------//
				if (path_node_sequence.size() >= 2)
				{
					AddPathToVehicle(pVehicle, path_node_sequence, file_name.c_str());
					_proxy_ABM_log(0, "--step 11.1: read and add path_node_sequence = %s\n", path_node_sequence_str.c_str());

					fprintf(g_DebugLogFile, "read and add path_node_sequence = %s for vehicle %d\n", path_node_sequence_str.c_str(), pVehicle->m_AgentID);
				}
				//-----------------------------------//
				else
				{
					//-----QU 10.30------------to do list: information type and new path--------------------------------------//

					//--for information type 0;
					//--for information type 1;
					//--for information type 2;

					//--for information type 3-------has been finished~~~//
					_proxy_ABM_log(0, "--step 12: update existing new path (which has not been used before the trip starts) if the destination or departure time is changed\n");
					
					if (parser_agent.GetValueByFieldName("path_switch_flag", sub_path_switch_flag))  // user defined
					{
						if (sub_path_switch_flag >= 1)  // if the user defines  path_switch_flag >=1
						{
							_proxy_ABM_log(0, "--step 13: path_switch_flag=1 for starting from current link\n");

							//first, check if m_alt_path_node_sequence has been defined in the memory
							std::vector<int> sub_path_node_sequence = pVehicle->m_alt_path_node_sequence;
							_proxy_ABM_log(0, "--step 13.1: size of in memory alternative path node sequence = %d\n", pVehicle->m_alt_path_node_sequence.size());

							string detour_node_sequence_str;
							if (parser_agent.GetValueByFieldName("alt_path_node_sequence", detour_node_sequence_str) == true)
							{
								//second, rewrite alt_path_node_sequence if the user defines the node sequence in the input agent text file, again

								sub_path_node_sequence = ParseLineToIntegers(detour_node_sequence_str);
								_proxy_ABM_log(0, "--step 13.2: size of alternative path node sequence in input agent file = %d\n", sub_path_node_sequence);
							}

							if (sub_path_node_sequence.size() > 0)
							{

								g_UpdateAgentPathBasedOnDetour(pVehicle->m_AgentID, sub_path_node_sequence);
								_proxy_ABM_log(0, "--step 13.3: switch to alternative path\n");
							}
						}
					}
					else // no user defined, use default settings 
					{					if (pVehicle->m_InformationType == info_hist_based_on_routing_policy || pVehicle->m_InformationType == learning_from_hist_travel_time)
										{
											g_UseExternalPathDefinedInRoutingPolicy(pVehicle);
										}
										else // pVehicle >=2
										{
											g_UpdateAgentPathBasedOnNewDestinationOrDepartureTime(pVehicle->m_AgentID);
										}
					}

					//--------------------------------------------------------------------------------------------------------//
				}

			}
			else if (parser_agent.GetValueByFieldName("path_switch_flag", sub_path_switch_flag))
			{
				if (sub_path_switch_flag >= 1)  // if the user defines  path_switch_flag >=1
				{
					_proxy_ABM_log(0, "--step 13: path_switch_flag=1 for starting from current link\n");

					//first, check if m_alt_path_node_sequence has been defined in the memory
					std::vector<int> sub_path_node_sequence = pVehicle->m_alt_path_node_sequence;
					_proxy_ABM_log(0, "--step 13.1: size of in memory alternative path node sequence = %d\n", pVehicle->m_alt_path_node_sequence.size());

					string detour_node_sequence_str;
					if (parser_agent.GetValueByFieldName("alt_path_node_sequence", detour_node_sequence_str) == true)
					{
						//second, rewrite alt_path_node_sequence if the user defines the node sequence in the input agent text file, again

						sub_path_node_sequence = ParseLineToIntegers(detour_node_sequence_str);
						_proxy_ABM_log(0, "--step 13.2: size of alternative path node sequence in input agent file = %d\n", sub_path_node_sequence);
					}

					if (sub_path_node_sequence.size() > 0)
					{

						g_UpdateAgentPathBasedOnDetour(pVehicle->m_AgentID, sub_path_node_sequence);
						_proxy_ABM_log(0, "--step 13.3: switch to alternative path\n");
					}
				}
			}

			int number_of_agents = 1;

			float ending_departure_time = 0;


			i++;
		}


		line_no++;



		if (bOutputLogFlag)
		{

			cout << count << " records have been read from file " << file_name << endl;

			cout << i << " agents have been read from file " << file_name << endl;

			if (count_for_sameOD >= 1)
				cout << "there are " << count_for_sameOD << " agents with the same from_zone_id and to_zone_id, which will not be simulated. " << endl;


			if (count_for_not_defined_zones >= 1)
				cout << "there are " << count_for_not_defined_zones << " agents with zones not being defined in input_zone.csv file, which will not be simulated. " << endl;

		}
		LineCount = count;
	}
	else
	{
		cout << "Waiting for file " << file_name << "... " << endl;

		return false;
	}

	return true;

}

bool g_ReadAgentBinFile(string file_name, bool b_with_updated_demand_type_info)
{

	cout << "Reading Agent Bin File..." << endl;
	g_VehicleLoadingMode = vehicle_binary_file_mode;

	g_DetermineDemandLoadingPeriod();

	int path_node_sequence[MAX_NODE_SIZE_IN_A_PATH];

	g_AllocateDynamicArrayForVehicles();

	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "rb");
	if (st != NULL)
	{
		struct_VehicleInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_VehicleInfo_Header), 1, st);

			if (header.vehicle_id < 0)
				break;

			if (header.vehicle_id == 28)
				TRACE("Vehicle ID = %d\n", header.vehicle_id);


			if (header.number_of_nodes != 12)
			{
				TRACE("");

			}

			if (result != 1)  // read end of file
				break;

			DTAVehicle* pVehicle = 0;
			//try
			//{
			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			//if(header.departure_time >= 420)
			//	break;

			////}
			////catch (std::bad_alloc& exc)
			////{
			////	cout << "Insufficient memory...";
			////	getchar();
			////	exit(0);

			////}

			pVehicle->m_AgentID = header.vehicle_id;
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			pVehicle->m_OriginZoneID = header.from_zone_id;
			pVehicle->m_DestinationZoneID = header.to_zone_id;

			g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
			g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;


			pVehicle->m_DepartureTime = header.departure_time;

			if (g_DemandLoadingEndTimeInMin < pVehicle->m_DepartureTime)
				g_DemandLoadingEndTimeInMin = pVehicle->m_DepartureTime;

			if (g_DemandLoadingStartTimeInMin > pVehicle->m_DepartureTime)
				g_DemandLoadingStartTimeInMin = pVehicle->m_DepartureTime;

			pVehicle->m_PreferredDepartureTime = header.departure_time;
			pVehicle->m_ArrivalTime = header.arrival_time;

			pVehicle->m_TripTime = header.trip_time;

			pVehicle->m_DemandType = header.demand_type;


			if (pVehicle->m_DemandType == 0) // unknown type
				pVehicle->m_DemandType = 1;

			pVehicle->m_VehicleType = header.vehicle_type;
			pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType - 1].PCE;
			pVehicle->m_InformationType = header.information_type;
			pVehicle->m_VOT = header.value_of_time;
			pVehicle->m_Age = header.age;


			//
			float vehicle_trip_multiplier_factor = 1.0f;

			if (b_with_updated_demand_type_info)
			{
				int demand_type = -1;
				double RandomPercentage = g_GetRandomRatio() * 100;

				double previous_cumulative_percentage = 0;

				for (int type_no = 0; type_no < g_DemandTypeVector.size(); type_no++)
				{


					double cumulative_percentage = g_DemandTypeVector[type_no].cumulative_demand_type_percentage;
					if (RandomPercentage >= previous_cumulative_percentage && RandomPercentage <= cumulative_percentage)
					{
						vehicle_trip_multiplier_factor = g_DemandTypeVector[type_no].vehicle_trip_multiplier_factor;
						demand_type = type_no;

						previous_cumulative_percentage = cumulative_percentage;

					}
				}

				//	TRACE("vehicle id = %d, demand type = %d\n ", header.vehicle_id, demand_type);

				if (demand_type == -1)
				{
					cout << "Error: demand_type = -1" << endl;
					g_ProgramStop();

				}

				pVehicle->m_DemandType = demand_type;

				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationType, pVehicle->m_VOT, pVehicle->m_Age);


			}



			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;

			pVehicle->m_NodeSize = header.number_of_nodes;

			if (header.number_of_nodes >= 1999)

			{
				cout << "Error in reading agent file: header.number_of_node = " << header.number_of_nodes << endl;
				g_ProgramStop();
			}
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Distance = 0;
			pVehicle->m_NodeNumberSum = 0;

			int time_interval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

			if (g_ODEstimationFlag == 1) // having hist od only unde ODME mode
			{
				g_SystemDemand.AddValue(pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, time_interval, 1); // to store the initial table as hist database
			}

			if (pVehicle->m_NodeSize >= 1)  // in case reading error
			{
				pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];

				pVehicle->m_NodeNumberSum = 0;
				int i;
				for (i = 0; i < pVehicle->m_NodeSize; i++)
				{

					int node_id;
					float event_time_stamp, travel_time, emissions;

					struct_Vehicle_Node node_element;
					fread(&node_element, sizeof(node_element), 1, st);

					path_node_sequence[i] = node_element.NodeName;
					pVehicle->m_NodeNumberSum += path_node_sequence[i];

					if (i == 0)
						pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

					if (i == pVehicle->m_NodeSize - 1)
						pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

					if (i >= 1)
					{
						DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
						if (pLink == NULL)
						{
							CString msg;
							msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], header.vehicle_id, file_name.c_str());
							cout << msg << endl;
							continue;
						}

						if (pLink->GetNumberOfLanes() < 0.01)  // this is a blocked link by work zone
						{
							pVehicle->m_bForcedSwitchAtFirstIteration = true;

						}

						pVehicle->m_Distance += pLink->m_Length;

						pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
					}


				}


				if (g_DemandCapacityScalingFactor < 0.999 && g_DemandCapacityScalingFactor >= 0.0001)  // valid ratio
				{
					double random_ratio = g_GetRandomRatio();

					if (random_ratio > g_DemandCapacityScalingFactor)
					{
						delete pVehicle;
						pVehicle = NULL;
						continue;  // do not proceed to the remaining steps

					}

				}




				if (vehicle_trip_multiplier_factor < 0.9999 && b_with_updated_demand_type_info == true)  // we have to run a random number to decide if the vehicles should be added into the simulation or not.
				{

					double RandomRatio = g_GetRandomRatio();
					if (RandomRatio < vehicle_trip_multiplier_factor)
					{

						delete pVehicle;
						pVehicle = NULL;
						continue;  // do not proceed to the remaining steps

					}

				}
				if (i >= pVehicle->m_NodeSize)
				{

					g_VehicleVector.push_back(pVehicle);
					g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
					g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

					int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

					g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);

					count++;
				}
				if (count % 10000 == 0)
					cout << "reading " << count / 1000 << "K agents from binary file " << file_name << endl;
			}
		}
		g_ResetVehicleAttributeUsingDemandType();



		fclose(st);
		return true;

	}
	else
	{
		cout << "File agent.bin cannot be found. Please check." << endl;
		g_ProgramStop();



	}
	return false;
}

void g_ResetVehicleType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;
		pVehicle->m_InformationType = info_hist_based_on_routing_policy;
		double RandomPercentage = g_GetRandomRatio() * 100;
		for (int in = 0; in < MAX_INFO_CLASS_SIZE; in++)
		{
			int demand_type_no = pVehicle->m_DemandType - 1;

			if (RandomPercentage >= g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[in - 1] &&
				RandomPercentage < g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[in])
				pVehicle->m_InformationType = in + 1; // return pretrip as 2 or enoute as 3
		}
	}

}

void g_ResetVehicleAttributeUsingDemandType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;

		g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationType, pVehicle->m_VOT, pVehicle->m_Age);

	}
}


class DTAPathNodeSequence{
public:

	std::vector<int> m_node_sequence;

};

void g_SkimMatrixGenerationForAllDemandTypes(string FileName, bool bTimeDependentFlag, double CurrentTime)
{

	CString file_name;
	file_name.Format(FileName.c_str());

	CString str_output_file_in_summary;
	str_output_file_in_summary.Format("Output file =,%s\n", file_name);
	g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);

	int time_dependent_ODMOE_distance_label = g_GetPrivateProfileInt("output", "time_dependent_ODMOE_distance_label", 1, g_DTASettingFileName);

	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int DemandLoadingStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int DemandLoadingEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		DemandLoadingStartTimeInMin = CurrentTime;
		DemandLoadingEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculate time interval " << DemandLoadingStartTimeInMin << " -> " << DemandLoadingEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;


	int total_demand_type = g_DemandTypeVector.size();
	//cout << "------00---------" << endl;
	int StatisticsIntervalSize = 1;

	int number_of_time_intervals_for_reporting = max(1, (DemandLoadingEndTimeInMin - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval);

	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float**** ODTravelTime = NULL;
	//ODTravelTime = Allocate4DDynamicArray<float>(1000, 1000, 1000, 1000);
	ODTravelTime = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDistance = NULL;
	ODDistance = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDollarCost = NULL;
	ODDollarCost = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);


	for (int departure_time_index = 0; departure_time_index < number_of_time_intervals_for_reporting; departure_time_index++)
	{

		int departure_time = DemandLoadingStartTimeInMin + departure_time_index* g_AggregationTimetInterval;

		int assignment_interval_index = departure_time / 15;

		if (departure_time_index > 0 && assignment_interval_index < 200 && g_AssignmentIntervalOutputFlag[assignment_interval_index] == 0)  // we output the data for sure: departure_time_index ==0
		{
			// skip the output of skim matrix 
			continue;

		}


		cout << "...calculating time interval " << departure_time << " min... " << endl;


		//cout << "------05---------" << endl;
		for (int d = 1; d <= total_demand_type; d++)
			for (int i = 0; i <= g_ODZoneIDSize; i++)
				for (int j = 0; j <= g_ODZoneIDSize; j++)
					for (int t = 0; t < StatisticsIntervalSize; t++)
					{
						if (i == j)
						{
							ODTravelTime[d][i][j][t] = 0.5;
							ODDistance[d][i][j][t] = 0.5;
							ODDollarCost[d][i][j][t] = 0.0;
						}
						else
						{
							ODTravelTime[d][i][j][t] = 0;
							ODDistance[d][i][j][t] = 0;
							ODDollarCost[d][i][j][t] = 0;
						}
					}


		if (bTimeDependentFlag)
			cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
		else
			cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


#pragma omp parallel for
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{

			// create network for shortest path calculation at this processor
			int	id = omp_get_thread_num();  // starting from 0

			//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

			if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
			{
				g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
			}

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 
					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{


						int starting_demand_type = 1;

						if (departure_time_index == 0) // starting time of all computing 
							starting_demand_type = 0;

						for (int demand_type = starting_demand_type; demand_type <= total_demand_type; demand_type++)
						{

							bDistanceCost = false;

							if (bTimeDependentFlag == false)  // real time version
							{
								if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
									continue;
							}
							g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, iterZone->first, departure_time, demand_type, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo

							// to each destination zone
							for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
							{

								int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
								if (dest_node_index >= 0 && (iterZone->first != iterZone2->first)) // convert node number to internal node id
								{

									int time_interval_no = 0;  // send data to 0 index element

									float TravelTime = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
									ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];

									ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];
									ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDollarCostAry[dest_node_index];

									if (ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] > 0.1)
									{
										TRACE("");
									}
								}

							} //for each destination zone
						}  // departure type
					}  // with origin node numbers 
				} // each demand type 
			} // current thread	

		}  // origin zone





		FILE* st = NULL;

		int hour = departure_time / 60;
		int min = (departure_time - hour * 60);

		CString time_str;
		time_str.Format("_%02dh%02dm", hour, min);

		CString final_file_name; 
		final_file_name = file_name;

		if (bTimeDependentFlag)
		{
			final_file_name += time_str;
			final_file_name += ".csv";

		}

		fopen_s(&st, final_file_name, "w");
		if (st != NULL)
		{
			//write header:
			fprintf(st, "from_zone_id,to_zone_id,departure_time_in_min,");
			CString str;

			for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
			{
				str.Format("DT%d_TT_in_min,", demand_type);
				fprintf(st, str);
			}

			if(time_dependent_ODMOE_distance_label==1)
			{ 
				for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
				{
					str.Format("DT%d_Distance,", demand_type);
					fprintf(st, str);
				}
			}

			for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
			{
				str.Format("DT%d_Toll_Cost,", demand_type);
				fprintf(st, str);
			}


			//str.Format("demand_type_%d_generalized_travel_time_diff,demand_type_%d_distance_diff,demand_type_%d_dollar_cost_diff,", total_demand_type, total_demand_type, total_demand_type);


			fprintf(st, "\n");

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{
				// to each destination zone
				for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
				{
					int origin_zone = iterZone->first;
					int destination_zone = iterZone2->first;


					if (origin_zone != destination_zone && origin_zone >= 1 && destination_zone >= 1)
					{

						int time_interval_no = 0; // send data to zero index element

						if (bTimeDependentFlag == false)  // real time version
						{
							if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
								continue;
						}

						bool new_data_flag = false;

						//check travel time
						int demand_type_0 = 0;
						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
						{
							if (fabs(ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
								ODTravelTime[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
							{
								new_data_flag = true;
								break;
							}
						}

						if (new_data_flag == false)
						{
							//check distance 
							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							{
								if (fabs(ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
									ODDistance[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
								{
									new_data_flag = true;
									break;
								}
							}

							if (new_data_flag == false)
							{

								//check ODDollarCost 
								for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								{
									if (fabs(ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
										ODDollarCost[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
									{
										new_data_flag = true;
										break;

									}
								}
							}
						}




						if (departure_time_index == 0 || new_data_flag == true || bTimeDependentFlag == true)
						{
							fprintf(st, "%d,%d,%d,",
								origin_zone,
								destination_zone,

								departure_time);

							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								fprintf(st, "%4.1f,", ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

							if (time_dependent_ODMOE_distance_label == 1)
							{ 
								for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								fprintf(st, "%4.2f,", ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);
							}
							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							{
								float value = ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no];

								if(value >=0.01)
									fprintf(st, "%4.1f,", value );
								else 
									fprintf(st, "0,");
							}

							fprintf(st, "\n");
						}
					}

				}  // each department type

			}
			fclose(st);
		}
		else
		{
			cout << "File " << file_name << " cannot be opened." << endl;
			getchar();
			exit(0);

		}
		

	}
}


void g_SkimMatrixGenerationForAllDemandTypesV2(string FileName, bool bTimeDependentFlag, double CurrentTime)
{

	CString file_name;
	file_name.Format(FileName.c_str());

	CString str_output_file_in_summary;
	str_output_file_in_summary.Format("Output file =,%s\n", file_name);
	g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);

	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int DemandLoadingStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int DemandLoadingEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		DemandLoadingStartTimeInMin = CurrentTime;
		DemandLoadingEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculate time interval " << DemandLoadingStartTimeInMin << " -> " << DemandLoadingEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;


	int total_demand_type = g_DemandTypeVector.size();
	//cout << "------00---------" << endl;
	int StatisticsIntervalSize = 1;

	int number_of_time_intervals_for_reporting = max(1, (DemandLoadingEndTimeInMin - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval);

	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float**** ODTravelTime = NULL;
	//ODTravelTime = Allocate4DDynamicArray<float>(1000, 1000, 1000, 1000);
	ODTravelTime = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDistance = NULL;
	ODDistance = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDollarCost = NULL;
	ODDollarCost = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);


	for (int departure_time_index = 0; departure_time_index < number_of_time_intervals_for_reporting; departure_time_index++)
	{

		int departure_time = DemandLoadingStartTimeInMin + departure_time_index* g_AggregationTimetInterval;

		int assignment_interval_index = departure_time / 15;

		if (departure_time_index > 0 && assignment_interval_index < 200 && g_AssignmentIntervalOutputFlag[assignment_interval_index] == 0)  // we output the data for sure: departure_time_index ==0
		{
			// skip the output of skim matrix 
			continue;

		}


		cout << "...calculating time interval " << departure_time << " min... " << endl;


		//cout << "------05---------" << endl;
		for (int d = 1; d <= total_demand_type; d++)
			for (int i = 0; i <= g_ODZoneIDSize; i++)
				for (int j = 0; j <= g_ODZoneIDSize; j++)
					for (int t = 0; t < StatisticsIntervalSize; t++)
					{
						if (i == j)
						{
							ODTravelTime[d][i][j][t] = 0.5;
							ODDistance[d][i][j][t] = 0.5;
							ODDollarCost[d][i][j][t] = 0.0;
						}
						else
						{
							ODTravelTime[d][i][j][t] = 0;
							ODDistance[d][i][j][t] = 0;
							ODDollarCost[d][i][j][t] = 0;
						}
					}


		if (bTimeDependentFlag)
			cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
		else
			cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


#pragma omp parallel for
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{

			// create network for shortest path calculation at this processor
			int	id = omp_get_thread_num();  // starting from 0


											//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

			if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
			{
				g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
			}

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 
					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{


						int starting_demand_type = 1;

						if (departure_time_index == 0) // starting time of all computing 
							starting_demand_type = 0;

						for (int demand_type = starting_demand_type; demand_type <= total_demand_type; demand_type++)
						{

							bDistanceCost = false;

							if (bTimeDependentFlag == false)  // real time version
							{
								if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
									continue;
							}
							g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, iterZone->first, departure_time, demand_type, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo

																																																   // to each destination zone
							for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
							{

								int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
								if (dest_node_index >= 0 && (iterZone->first != iterZone2->first)) // convert node number to internal node id
								{

									int time_interval_no = 0;  // send data to 0 index element

									float TravelTime = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
									ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];

									ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];
									ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDollarCostAry[dest_node_index];

									if (ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] > 0.1)
									{
										TRACE("");
									}
								}

							} //for each destination zone
						}  // departure type
					}  // with origin node numbers 
				} // each demand type 
			} // current thread	

		}  // origin zone





		FILE* st = NULL;

		int hour = departure_time / 60;
		int min = (departure_time - hour * 60);

		CString time_str;
		time_str.Format("_%02dh%02dm", hour, min);

		CString final_file_name;
		final_file_name = file_name;

		if (bTimeDependentFlag)
		{
			final_file_name += time_str;
			final_file_name += ".csv";

		}

		fopen_s(&st, final_file_name, "w");
		if (st != NULL)
		{
			//write header:
			fprintf(st, "from_zone_id,to_zone_id,departure_time_in_min,");
			CString str;

			for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
			{
				str.Format("DT%d_TT_in_min,", demand_type);
				fprintf(st, str);
			}

			for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
			{
				str.Format("DT%d_Distance,", demand_type);
				fprintf(st, str);
			}

			for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
			{
				str.Format("DT%d_Toll_Cost,", demand_type);
				fprintf(st, str);
			}


			//str.Format("demand_type_%d_generalized_travel_time_diff,demand_type_%d_distance_diff,demand_type_%d_dollar_cost_diff,", total_demand_type, total_demand_type, total_demand_type);


			fprintf(st, "\n");

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{
				// to each destination zone
				for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
				{
					int origin_zone = iterZone->first;
					int destination_zone = iterZone2->first;


					if (origin_zone != destination_zone && origin_zone >= 1 && destination_zone >= 1)
					{

						int time_interval_no = 0; // send data to zero index element

						if (bTimeDependentFlag == false)  // real time version
						{
							if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
								continue;
						}

						bool new_data_flag = false;

						//check travel time
						int demand_type_0 = 0;
						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
						{
							if (fabs(ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
								ODTravelTime[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
							{
								new_data_flag = true;
								break;
							}
						}

						if (new_data_flag == false)
						{
							//check distance 
							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							{
								if (fabs(ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
									ODDistance[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
								{
									new_data_flag = true;
									break;
								}
							}

							if (new_data_flag == false)
							{

								//check ODDollarCost 
								for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								{
									if (fabs(ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
										ODDollarCost[demand_type_0][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][0]) > 0.01)
									{
										new_data_flag = true;
										break;

									}
								}
							}
						}




						if (departure_time_index == 0 || new_data_flag == true || bTimeDependentFlag == true)
						{
							fprintf(st, "%d,%d,%d,",
								origin_zone,
								destination_zone,

								departure_time);

							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								fprintf(st, "%4.2f,", ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								fprintf(st, "%4.2f,", ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

							for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
								fprintf(st, "%4.2f,", ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

							fprintf(st, "\n");
						}
					}

				}  // each department type

			}
			fclose(st);
		}
		else
		{
			cout << "File " << file_name << " cannot be opened." << endl;
			getchar();
			exit(0);

		}


	}
}


