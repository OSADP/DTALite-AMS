//  Portions Copyright 2014 Xuesong Zhou, Taylor Li

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
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
#include "stdafx.h"
#include "DTALite.h"
#include "GlobalData.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <stdlib.h>  
#include <math.h>    

#include <iostream>
#include <conio.h>
#include <windows.h>

using namespace std;

int g_use_global_path_set_flag = 0;
extern ODPathSet*** g_ODPathSetVector;


void g_BuildGlobalPathSet()
{
	if (g_ODPathSetVector == NULL)
	{
		g_ODPathSetVector = Allocate3DDynamicArray<ODPathSet>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, _max_info_type);
	}

	// initialization 
		for (int i = 0; i<g_ODZoneIDSize; i++)
		{
			for (int j = 0; j<g_ODZoneIDSize; j++)
			{
				for (int it = 0; it < _max_info_type; it++)
				g_ODPathSetVector[i][j][it].PathSet.clear();
			}

		}

 // tally
		std::map<int, DTAVehicle*>::iterator iterVM;
		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
		{

			DTAVehicle* pVehicle = iterVM->second;

			int OrgZoneSequentialNo = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
			int DestZoneSequentialNo = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

			if (pVehicle->m_NodeSize >= 2 )
			{
				std::vector<PathStatistics> Vehicle_ODTPathSet;

				Vehicle_ODTPathSet = g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet;

				bool ExistPath_Flag = false;
				int PathNo = 0;
				for (std::vector<PathStatistics>::iterator IterPS = Vehicle_ODTPathSet.begin(); IterPS != Vehicle_ODTPathSet.end(); IterPS++)
				{
					if (pVehicle->m_NodeNumberSum == IterPS->NodeSums) //existing path
					{
						ExistPath_Flag = true;
						break;
					}
					else                                             //New Path;
						ExistPath_Flag = false;
					PathNo++;
				}

				if (ExistPath_Flag)				// if old path, add statistics
				{
					g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalVehicleSize += 1;
					g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalDistance += pVehicle->m_Distance;
					g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalFFTT += pVehicle->m_TripFFTT;
				}
				else				           // if new path, add
				{
					PathStatistics vehicle_PathStatistics;

					vehicle_PathStatistics.Origin_ZoneID = pVehicle->m_OriginZoneID;
					vehicle_PathStatistics.Destination_ZoneID = pVehicle->m_DestinationZoneID;

					vehicle_PathStatistics.TotalVehicleSize = 1;
					vehicle_PathStatistics.TotalDistance = pVehicle->m_Distance;
					vehicle_PathStatistics.TotalFFTT = pVehicle->m_TripFFTT;

					vehicle_PathStatistics.NodeSums = pVehicle->m_NodeNumberSum;

					// add node id along the path
					int NodeID = g_LinkVector[pVehicle->m_LinkAry[0].LinkNo]->m_FromNodeID;  // first node
					int NodeName = g_NodeVector[NodeID].m_NodeNumber;
					vehicle_PathStatistics.m_NodeNumberArray.push_back(NodeName);
					for (int j = 0; j< pVehicle->m_NodeSize - 1; j++)
					{
						int LinkID = pVehicle->m_LinkAry[j].LinkNo;
						NodeID = g_LinkVector[LinkID]->m_ToNodeID;
						NodeName = g_NodeVector[NodeID].m_NodeNumber;
						vehicle_PathStatistics.m_NodeNumberArray.push_back(NodeName);

						vehicle_PathStatistics.m_LinkIDArray.push_back(g_LinkVector[LinkID]->m_OrgLinkID);


					}
					g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet.push_back(vehicle_PathStatistics);

				}
			}
		}
}

void g_OutputCurrentGlobalPathSet(int SimulationTimeInMin)
{

	// initialization 
	for (int i = 0; i<g_ODZoneIDSize; i++)
	{
		for (int j = 0; j<g_ODZoneIDSize; j++)
		{
			for (int it = 0; it < _max_info_type; it++)
			{
			
			for (int k = 0; k < g_ODPathSetVector[i][j][it].PathSet.size(); k++)
			{
				g_ODPathSetVector[i][j][it].PathSet[k].TotalVehicleSize = 0;  // reset path volume
			}
			}

		}

	}

	// tally
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{

		DTAVehicle* pVehicle = iterVM->second;

		int OrgZoneSequentialNo = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
		int DestZoneSequentialNo = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

		if (pVehicle->m_NodeSize >= 2 && pVehicle->m_DepartureTime >= SimulationTimeInMin - 15 && pVehicle->m_DepartureTime < SimulationTimeInMin)
		{
			std::vector<PathStatistics> Vehicle_ODTPathSet;

			Vehicle_ODTPathSet = g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet;

			bool ExistPath_Flag = false;
			int PathNo = 0;
			for (std::vector<PathStatistics>::iterator IterPS = Vehicle_ODTPathSet.begin(); IterPS != Vehicle_ODTPathSet.end(); IterPS++)
			{
				if (pVehicle->m_NodeNumberSum == IterPS->NodeSums) //existing path
				{
					ExistPath_Flag = true;
					break;
				}
				else                                             //New Path;
					ExistPath_Flag = false;
				PathNo++;
			}

			if (ExistPath_Flag)				// if old path, add statistics
			{
				g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalVehicleSize += 1;
				g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalDistance += pVehicle->m_Distance;
				g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalFFTT += pVehicle->m_TripFFTT;
				g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalTravelTime += (pVehicle->m_ArrivalTime - pVehicle->m_DepartureTime);
				g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][0].PathSet[PathNo].TotalEmissions += pVehicle->CO2;
			}
			else				           // if new path, // possible error
			{
			}
		}
	}


}

void g_ExchangeVISSIM_RealTime_Link_Status(int meso_simulation_time_interval_no)
{
	CString file_name;

	file_name.Format("DEX_VIS_RT_SEC_%d.csv", (meso_simulation_time_interval_no+1)*6);  // 6 second

	CString file_name_ack;

	file_name_ack.Format("DEX_VIS_RT_SEC_%d_ACK.csv", (meso_simulation_time_interval_no + 1) * 6);  // 6 second

	while (1)
	{

		CCSVParser parser_link_status_attribute;
		if (parser_link_status_attribute.OpenCSVFile(CString2StdString(file_name), false))
		{

			int count = 0;
			while (parser_link_status_attribute.ReadRecord())
			{

				// updating file exists.
				int link_id = -1;
				int status = -1;

				if (parser_link_status_attribute.GetValueByFieldName("link_id", link_id) == false)
					break;

				if (parser_link_status_attribute.GetValueByFieldName("status", status) == false)
					break;

				if (g_LinkIDMap.find(link_id) != g_LinkIDMap.end())
				{

					if (g_LinkIDMap.find(link_id) != g_LinkIDMap.end())
					{

						DTALink* pLink = g_LinkIDMap[link_id];

							if (status == 0)
							{
								pLink->m_OutflowNumLanes = 0;
								pLink->m_InflowNumLanes = 0;

							}
							else  // ==1 
							{
								pLink->m_OutflowNumLanes = pLink->m_Orginal_OutflowNumLanes;
								pLink->m_InflowNumLanes = pLink->m_Orginal_InflowNumLane;

							}

							count++;

					}
					else
					{
						cout << "link_id " << link_id << " has not been defined. Please any key to continue." << endl;
						getchar();


					}
					}  //  a link is found
				}  // read record

				// close the received file
				parser_link_status_attribute.CloseCSVFile();

				// output the new ACK file to VISSIM to move forward 
				FILE* st = NULL;
				fopen_s(&st, file_name_ack, "w");
				if (st != NULL)
				{
					//write header:
					fprintf(st, "ACK.");
					fclose(st);
				}
				else
				{
					cout << "ACK_file " << file_name_ack << " cannot be opened. Please any key to continue." << endl;
					getchar();

				}

				cout << "File " << file_name << " has been read with " << count << " records." << endl;
				break;  // finished the file reading
		}
		else
		{

			cout << "wait for 5 seconds... " << endl;

			Sleep(1000); // wait for 1 second

		}
	}  //while loop

}