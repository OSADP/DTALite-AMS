//  Portions Copyright 2013 Xuesong Zhou, Jinjin Tang

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


// prototype 1: heterogeneous traveler
// prototype 2: departure time and mode options
// prototype 3: extention

// assignment module
// obtain simulation results, fetch shortest paths, assign vehicles to the shortest path according to gap function or MSA
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


DTANetworkForSP g_PrevailingTimeNetwork_MP[_MAX_NUMBER_OF_PROCESSORS]; //  network instance for single processor in multi-thread environment: no more than 8 threads/cores
ODPathSet*** g_ODPathSetVector = NULL;

void DTANetworkForSP::AgentBasedPathAdjustment(
	int DayNo, int zone,int departure_time_begin, double current_time)
 // for pre-trip and en-route information user classes, for each departure time interval
{
	if(g_ODEstimationFlag==1)
		return;
	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH]={-1};
	int CurrentPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };
	int TempPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };
	float AbsArrivalTimeOnDSNVector[MAX_NODE_SIZE_IN_A_PATH]={0};
	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;

	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);  // starting assignment interval


	// loop through the TDOVehicleArray to assign or update vehicle paths...
	for (int vi = 0; vi<g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size(); vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		DTAVehicle* pVeh  = g_VehicleMap[VehicleID];

		// if this is a pre-trip vehicle, and he has not obtained real-time information yet

		bool b_switch_flag = false;

		if(g_bVehicleAttributeUpdatingFlag ==true)  // if some vehicles' attribute might need updating
		{
		int current_time_in_min = (int)(current_time);

			if(pVeh->m_attribute_update_time_in_min <= current_time_in_min 
				&& pVeh->m_bEvacuationResponse == false
				&& pVeh->m_SimLinkSequenceNo < pVeh->m_NodeSize - 2) // if scheduled updating time is earlier than the current time, has not reached at destination yet 
			{
				pVeh->m_bEvacuationResponse  = true;
				// change destination zone number

				// check if the current link is still in evacuation zone
				int CurrentLinkID = pVeh->m_LinkAry[pVeh->m_SimLinkSequenceNo].LinkNo;
				
				if( g_LinkVector[CurrentLinkID]->IsInsideEvacuationZoneFlag(DayNo, current_time))
				{
				pVeh->m_DestinationZoneID  = pVeh->m_DestinationZoneID_Updated;
				pVeh->m_DestinationNodeID =  g_ZoneMap[pVeh->m_DestinationZoneID].GetRandomDestinationIDInZone ((pVeh->m_AgentID%100)/100.0f); 

				pVeh->m_InformationType = info_en_route_and_pre_trip; // 3, so all the vehicles will access real-time information 
				pVeh->m_TimeToRetrieveInfo = 99999; // no more update

				b_switch_flag = true;
				DTAEvacuationRespone response;
				response.LinkNo = CurrentLinkID;
				response.ResponseTime = current_time;
				}
			}
		
		}



		// pVeh->m_DepartureTime +1: assume we access pre trip information one min before the trip

		if(pVeh->m_InformationType == info_pre_trip 
			&& int(pVeh->m_DepartureTime +1) > current_time /* if the departure time rounds up to the current time (min by min), then perforom pre-trip routing  */
			&& int(pVeh->m_TimeToRetrieveInfo) <= current_time)
		{

						if(pVeh->m_AgentID ==86)
						{
						TRACE("");
						}
			b_switch_flag = true;

			pVeh->m_TimeToRetrieveInfo = 99999; // no more update
		}

		// if this is an enroute info vehicle, 
		if(pVeh->m_InformationType == info_en_route_and_pre_trip && pVeh->m_bLoaded == true &&
			int(pVeh->m_DepartureTime) < current_time &&
			(int)(pVeh->m_TimeToRetrieveInfo) <= current_time && pVeh ->m_bComplete == false)
		{

			pVeh->m_TimeToRetrieveInfo += 1; // next 1 min
			b_switch_flag = true;

			if(pVeh->m_AgentID == 66 && current_time >=120)
			{
			TRACE("Current Time %f, Node Size = %d\n", current_time, pVeh->m_NodeSize);
			}


		}

		if(b_switch_flag == true)
		{

		if(pVeh->m_AgentID == 68)
		{
		TRACE("");
		}
		float TotalCost = 0;
		bool bGeneralizedCostFlag = false;
		bool bDebugFlag = false;

		int StartingNodeID = pVeh->m_OriginNodeID;

		if(pVeh->m_InformationType == info_en_route_and_pre_trip  && pVeh->m_bLoaded == true )  // en route info, has not reached destination yet
		{
			int CurrentLinkID = pVeh->m_LinkAry[pVeh->m_SimLinkSequenceNo].LinkNo;
			StartingNodeID = g_LinkVector[CurrentLinkID]->m_ToNodeID;

			if(pVeh->m_SimLinkSequenceNo == pVeh->m_NodeSize - 2)
				continue;

			if(StartingNodeID == pVeh->m_DestinationNodeID )  // you will reach the destination (on the last link).
				continue;

			// copy existing link number
			for(int i = 0; i<= pVeh->m_SimLinkSequenceNo; i++)
			{
				CurrentPathLinkList[i] = 	pVeh->m_LinkAry[i].LinkNo;
				AbsArrivalTimeOnDSNVector[i]= pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN ;
			}
		}


		// use agent-specific information look up window to update the prevailing travel time conditions 
		// step 1: update m_prevailing_travel_time for each link
		// step 2: call function BuildPhysicalNetwork again to use m_prevailing_travel_time to overwrite link travel time data
		// step 3: use link travel time data in agent-based routing 


		NodeSize = FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, StartingNodeID , pVeh->m_DepartureTime , pVeh->m_DestinationZoneID , pVeh->m_DestinationNodeID, pVeh->m_DemandType , pVeh->m_VOT, PathLinkList, TotalCost,bGeneralizedCostFlag, bDebugFlag);


	if(pVeh->m_InformationType == info_en_route_and_pre_trip && pVeh->m_bLoaded == true )  // en route info
	{

		int count = pVeh->m_SimLinkSequenceNo;
		for(int i = 0; i<NodeSize-1; i++)
		{
			count  +=1;  // add link no from the new path starting from the current position 
			CurrentPathLinkList[count] = 	PathLinkList[i];;
		}

		NodeSize = count+2;

		for(int i = 0; i< NodeSize-1; i++)  // copy back
		{		
			PathLinkList[i] = CurrentPathLinkList[i];
		}
	
	
	}

	
		if(NodeSize<=1)
			continue;

			if( pVeh->m_LinkAry !=NULL)  // delete the old path
			{
				pVeh->StoreOriginalPath();

				delete pVeh->m_LinkAry;
			}

			pVeh->m_NodeSize = NodeSize;


			if(pVeh->m_NodeSize>=2)  // for feasible path
			{

				if(NodeSize>=MAX_NODE_SIZE_IN_A_PATH)
				{
					cout << "PATH Size is out of bound of " << MAX_NODE_SIZE_IN_A_PATH << ": "<< NodeSize;
					g_ProgramStop();
				}

				pVeh->m_LinkAry = new SVehicleLink[NodeSize];

				if(pVeh->m_LinkAry==NULL)
				{
					cout << "Insufficient memory for allocating vehicle arrays!";
					g_ProgramStop();
				}

				int NodeNumberSum =0;
				float Distance =0;

				for(int i = 0; i< NodeSize-1; i++)
				{
					pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN = AbsArrivalTimeOnDSNVector[i];

					pVeh->m_LinkAry[i].LinkNo = PathLinkList[i];
					NodeNumberSum += PathLinkList[i];

					if(g_LinkVector[pVeh->m_LinkAry [i].LinkNo]!=NULL)
					{
						DTALink* pLink = g_LinkVector[pVeh->m_LinkAry [i].LinkNo];
						TRACE("Prevailing Travel Time %f \n", pLink->m_prevailing_travel_time );

					}
					
					ASSERT(pVeh->m_LinkAry [i].LinkNo < g_LinkVector.size());
					Distance+= g_LinkVector[pVeh->m_LinkAry [i].LinkNo] ->m_Length ;

				}
				
					
				pVeh->m_Distance  = Distance;
				pVeh->m_NodeNumberSum = NodeNumberSum;


			}
		}
	} // for each vehicle on this OD pair

}

void g_ApplyExternalPathInput(int departure_time_begin)
 // for pre-trip and en-route information user classes, for each departure time interval
{
	if(g_ODEstimationFlag==1)
		return;
	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);  // starting assignment interval

	for(int origin_zone_index = 0; origin_zone_index <g_ODZoneIDSize; origin_zone_index++)
	{
		// loop through the TDOVehicleArray to assign or update vehicle paths...
		for (int vi = 0; vi<g_TDOVehicleArray[origin_zone_index][AssignmentInterval].VehicleArray.size(); vi++)
		{
			int VehicleID = g_TDOVehicleArray[origin_zone_index][AssignmentInterval].VehicleArray[vi];
			DTAVehicle* pVeh  = g_VehicleMap[VehicleID];
			
			if (pVeh->m_InformationType == info_hist_based_on_routing_policy || pVeh->m_InformationType == learning_from_hist_travel_time)
			{
				g_UseExternalPathDefinedInRoutingPolicy(pVeh);
			}
		} // for each vehicle on this OD pair
	}
}

int g_number_of_CPU_threads()
{
	int number_of_threads = omp_get_max_threads ( );

	int max_number_of_threads = g_GetPrivateProfileInt("computation", "max_number_of_threads_to_be_used", 8, g_DTASettingFileName);

	if(number_of_threads > max_number_of_threads)
		number_of_threads = max_number_of_threads ;

	if (g_simulation_log_level >= 1)
		number_of_threads = 1; // if we need to print out simulation log, we use sequential mode only

	if(number_of_threads > _MAX_NUMBER_OF_PROCESSORS)
	{ 
		cout<< "the number of threads is "<< number_of_threads << ", which is greater than _MAX_NUMBER_OF_PROCESSORS. Please contact developers!" << endl; 
		g_ProgramStop();
	}

	return number_of_threads;

}

int g_number_of_CPU_threads_for_real_time_routing()
{
	int number_of_threads = omp_get_max_threads();

	int max_number_of_threads = g_GetPrivateProfileInt("computation", "max_number_of_threads_for_real_time_routing", 1, g_DTASettingFileName);

	if (number_of_threads > max_number_of_threads)
		number_of_threads = max_number_of_threads;


	if (number_of_threads > _MAX_NUMBER_OF_PROCESSORS)
	{
		cout << "the number of threads is " << number_of_threads << ", which is greater than _MAX_NUMBER_OF_PROCESSORS. Please contact developers!" << endl;
		g_ProgramStop();
	}

	return number_of_threads;

}

void g_ShortestPathDataMemoryAllocation()  
// for VMS responsive vehicles
{


	if (g_use_routing_policy_from_external_input == 1 )
	{	
		g_ODPathSetVector = Allocate3DDynamicArray<ODPathSet>(g_ODZoneIDSize+1,g_ODZoneIDSize+1, _max_info_type);
	}

	int node_size = g_NodeVector.size() + 1+  g_ODZoneIDSize;
	int link_size  = g_LinkVector.size() + g_NodeVector.size(); // maximal number of links including connectors assuming all the nodes are destinations

	cout << "Allocate Memory: node_size= " << node_size << ",link_size =" << link_size << endl;

#ifdef _large_memory_usage_lr
	g_LogFile << " Allocate shortest path memory : node_size = " << node_size << ", link_size = " << link_size << g_GetUsedMemoryDataInMB() << endl;
#else
	g_LogFile << " Allocate time-dependent large shortest path memory : node_size = " << node_size << ", link_size = " << link_size << g_GetUsedMemoryDataInMB() << endl;

#endif 		

	// under zone based mode, we add connectors for each destination 
	int number_of_connectors = 0;
	std::map<int, DTAZone>::iterator iterZone;
	for (iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		DTAZone zone = iterZone->second;
		number_of_connectors += zone.m_DestinationActivityVector.size();
	}

	link_size += number_of_connectors;

	int number_of_threads_for_real_time_routing = g_number_of_CPU_threads_for_real_time_routing();
	for (int ProcessID = 0; ProcessID < number_of_threads_for_real_time_routing; ProcessID++)
	{
	g_PrevailingTimeNetwork_MP[ProcessID].Setup(node_size, link_size, g_PlanningHorizon,g_AdjLinkSize,g_DemandLoadingStartTimeInMin);
	//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors
	g_PrevailingTimeNetwork_MP[ProcessID].BuildPhysicalNetwork(0, 0, g_TrafficFlowModelFlag, true, g_DemandLoadingStartTimeInMin);
	}
	int number_of_threads = g_number_of_CPU_threads();

	
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{
		g_TimeDependentNetwork_MP[ProcessID].Setup(node_size, link_size, g_PlanningHorizon, g_AdjLinkSize, g_DemandLoadingStartTimeInMin, g_ODEstimationFlag);
	}

	
	cout << "------- Memory allocation completed.-------" << endl;


	g_LogFile << " ------- shortest path memory allocation completed.-------" << g_GetUsedMemoryDataInMB() << endl;

}
void g_UpdateRealTimeInformation(double CurrentTime)
{
	int number_of_threads_for_real_time_routing = g_number_of_CPU_threads_for_real_time_routing();
	for (int ProcessID = 0; ProcessID < number_of_threads_for_real_time_routing; ProcessID++)
	{
		g_PrevailingTimeNetwork_MP[ProcessID].UpdateCurrentTravelTime(0, CurrentTime);
	}
}
void g_AgentBasedPathAdjustmentWithRealTimeInfo(int ProcessID, int VehicleID , double current_time)
// for VMS resonsive information
{
	
	if(g_ODEstimationFlag==1)
		return;

	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH]={-1};   // the new path to be copied back
	int CurrentPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };
	int TempPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };


	std::vector<int> UpdatedLinkList;

	float AbsArrivalTimeOnDSNVector[MAX_NODE_SIZE_IN_A_PATH]={0};
	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int PathNodeSize;
	int TempNodeSize;



		DTAVehicle* pVeh  = g_VehicleMap[VehicleID];


		if( pVeh->m_NodeSize >= MAX_NODE_SIZE_IN_A_PATH - 1 )
		{
		cout << "pVeh->m_NodeSize > MAX_NODE_SIZE_IN_A_PATH" << endl;
		g_ProgramStop();
		}


		int StartingNodeID = pVeh->m_OriginNodeID;

		float TotalCost = 0;
		bool bGeneralizedCostFlag = false;
		bool bDebugFlag = false;
		int link_count = 0;
		int CurrentLinkID = 0;
		int node_id;
		if (pVeh->m_NodeSize >= 1) // with path
		{

		// if this is a pre-trip vehicle, and he has not obtained real-time information yet



			CurrentLinkID = pVeh->m_LinkAry[pVeh->m_SimLinkSequenceNo].LinkNo;
			StartingNodeID = g_LinkVector[CurrentLinkID]->m_ToNodeID;

			if(StartingNodeID == pVeh->m_DestinationNodeID )  // you will reach the destination (on the last link).
				return;

			// copy existing link number (and remaining links)

			for(int i = 0; i< pVeh->m_NodeSize -1; i++)
			{
				CurrentPathLinkList[i] = 	pVeh->m_LinkAry[i].LinkNo;
				AbsArrivalTimeOnDSNVector[i]= pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN ;
			}


			link_count = pVeh->m_SimLinkSequenceNo;


			for (int i = 0; i<= pVeh->m_SimLinkSequenceNo; i++)
			{
				UpdatedLinkList.push_back(CurrentPathLinkList[i]);  // first step
				
			}
			

			//if(is.Type == 2) // detour VMS
			//{
			//for(int ll = 0; ll < is.DetourLinkSize ; ll++)
			//{
			//PathLinkList[l++] = is.DetourLinkArray [ll];
			//CurrentTime+= g_LinkVector[is.DetourLinkArray [ll]]->m_FreeFlowTravelTime;  // add travel time along the path to get the starting time at the end of subpath
			//}

			//CurrentNodeID = g_LinkVector[is.DetourLinkArray [is.DetourLinkSize -1]]->m_ToNodeID;

			//}
		}
		if(pVeh->m_OriginNodeID < 0 || pVeh->m_DestinationNodeID <0)
		{

		cout << "m_OriginNodeID < 0 || m_DestinationNodeID <0" << endl;
		getchar();
		
		}
		 
		//calculate the shortest path based on prevailing travel time
		PathNodeSize = g_PrevailingTimeNetwork_MP[ProcessID].FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, StartingNodeID, pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);

		//count is the current node sequence number
		int bSwitchFlag = 0;
		for (int i = 0; i<PathNodeSize - 1; i++)
		{
			link_count += 1;  // add link no from the new path starting from the current position 

			if (CurrentPathLinkList[link_count] != PathLinkList[i])
				bSwitchFlag = 1;
				

			UpdatedLinkList.push_back(PathLinkList[i]);

		}


		if (bSwitchFlag == 1)  // logging
		{
			_proxy_ABM_log(0, "** enroute-info agent %d's with destination node %d, on link %d->%d, current path",
				pVeh->m_AgentID,
				g_NodeVector[pVeh->m_DestinationNodeID].m_NodeNumber,
				g_LinkVector[CurrentLinkID]->m_FromNodeNumber, g_LinkVector[CurrentLinkID]->m_ToNodeNumber);


			node_id = g_LinkVector[pVeh->m_LinkAry[0].LinkNo]->m_FromNodeID;  //first node
			_proxy_ABM_log(-1, "%d->", g_NodeVector[node_id].m_NodeNumber);

			for (int i = 0; i< pVeh->m_NodeSize - 1; i++)
			{
				node_id = g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_ToNodeID;

				_proxy_ABM_log(-1, "%d->", g_NodeVector[node_id].m_NodeNumber);

			}
			_proxy_ABM_log(-1, "\n");


			_proxy_ABM_log(0, "** enroute-info agent %d's with destination node %d, on link %d->%d, switch to new path",
				pVeh->m_AgentID,
				g_NodeVector[pVeh->m_DestinationNodeID].m_NodeNumber,
				g_LinkVector[CurrentLinkID]->m_FromNodeNumber, g_LinkVector[CurrentLinkID]->m_ToNodeNumber);
			node_id = g_LinkVector[PathLinkList[0]]->m_FromNodeID;

			_proxy_ABM_log(-1, "%d->", g_NodeVector[node_id].m_NodeNumber);

			for (int i = 0; i<PathNodeSize - 1; i++)
			{
				 node_id = g_LinkVector[PathLinkList[i]]->m_ToNodeID;

				_proxy_ABM_log(-1, "%d->", g_NodeVector[node_id].m_NodeNumber );

			}
			_proxy_ABM_log(-1, "\n");

		}


		if (PathNodeSize <= 1)
			return;


		if( bSwitchFlag==0)  // no switching
			return;

				
			if(pVeh->m_NodeSize>=2)  // for feasible path
			{
				//switching 
				if (pVeh->m_LinkAry != NULL)  // delete the old path
				{
					pVeh->StoreOriginalPath();
					delete pVeh->m_LinkAry;
				}

				pVeh->m_NodeSize = UpdatedLinkList.size() + 1;  //new # of nodes

				if (pVeh->m_NodeSize >= MAX_NODE_SIZE_IN_A_PATH)
				{
					cout << "PATH Size is out of bound: " << MAX_NODE_SIZE_IN_A_PATH << pVeh->m_NodeSize;
					g_ProgramStop();
				}

				pVeh->m_LinkAry = new SVehicleLink[pVeh->m_NodeSize];

				if(pVeh->m_LinkAry==NULL)
				{
					cout << "Insufficient memory for allocating vehicle arrays!";
					g_ProgramStop();
				}

				int NodeNumberSum =0;
				float Distance =0;
				_proxy_ABM_log(-1, " updated path: ");

				int node_id = g_LinkVector[UpdatedLinkList[0]]->m_FromNodeID;
				_proxy_ABM_log(-1, " %d->", g_NodeVector[node_id].m_NodeNumber);


				for (int i = 0; i< UpdatedLinkList.size(); i++)
				{
					pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN = AbsArrivalTimeOnDSNVector[i];

					pVeh->m_LinkAry[i].LinkNo = UpdatedLinkList[i];
					NodeNumberSum += UpdatedLinkList[i];


					node_id = g_LinkVector[UpdatedLinkList[i]]->m_ToNodeID;

					_proxy_ABM_log(-1, " %d->", g_NodeVector[node_id].m_NodeNumber);


					if(g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL)
					{
					cout << "Error: g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL", pVeh->m_LinkAry [i].LinkNo;
					getchar();
					exit(0);
					}
					
					ASSERT(pVeh->m_LinkAry [i].LinkNo < g_LinkVector.size());
					Distance+= g_LinkVector[pVeh->m_LinkAry [i].LinkNo] ->m_Length ;

				}
				
					
				pVeh->m_Distance  = Distance;
				pVeh->m_NodeNumberSum = NodeNumberSum;
			}
}

void g_OpenMPAgentBasedPathAdjustmentWithRealTimeInfo(int VehicleID, double current_time, int IS_id, MessageSign vms)
// for VMS resonsive information
{
	if (g_ODEstimationFlag == 1)
		return;

	int CurrentPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };
	int NewSubPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { -1 };
	float AbsArrivalTimeOnDSNVector[MAX_NODE_SIZE_IN_A_PATH] = { 0 };
	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;


	for (int i = 0; i < MAX_NODE_SIZE_IN_A_PATH; i++)
	{
		CurrentPathLinkList[i] = -1;
		NewSubPathLinkList[i] = -1;
	}

	DTAVehicle* pVeh = g_VehicleMap[VehicleID];


	if (pVeh->m_NodeSize >= MAX_NODE_SIZE_IN_A_PATH - 1)
	{
		cout << "pVeh->m_NodeSize > MAX_NODE_SIZE_IN_A_PATH" << endl;
		g_ProgramStop();
	}

	// if this is a pre-trip vehicle, and he has not obtained real-time information yet

	float TotalCost = 0;
	bool bGeneralizedCostFlag = false;
	bool bDebugFlag = false;

	int StartingNodeID = pVeh->m_OriginNodeID;

	int CurrentLinkID = pVeh->m_LinkAry[pVeh->m_SimLinkSequenceNo].LinkNo;
	StartingNodeID = g_LinkVector[CurrentLinkID]->m_ToNodeID;

	// copy existing link number (and remaining links)


	for (int i = 0; i < pVeh->m_NodeSize - 1; i++)
	{
		CurrentPathLinkList[i] = pVeh->m_LinkAry[i].LinkNo;
		AbsArrivalTimeOnDSNVector[i] = pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN;
	}


	int count = pVeh->m_SimLinkSequenceNo;

	bool isChange = false;

	if (IS_id >= 0)
	{
		count += 1; // for VMS
		double cumulative_percentage = 0;

		double random_ratio = pVeh->GetRandomRatio();
		for (int p = 0; p < vms.NumberOfSubpaths; p++)
		{
			cumulative_percentage += vms.DetourSwitchingRatio[p];

			if (random_ratio < cumulative_percentage)  // take the detour
			{
				isChange = true;
				_proxy_ABM_log(0, "** agent %d's path on link %d->%d, switch to detour path %d (swithcing ratio = %.2f)", pVeh->m_AgentID,

					g_LinkVector[CurrentLinkID]->m_FromNodeNumber, g_LinkVector[CurrentLinkID]->m_ToNodeNumber, p + 1, vms.DetourSwitchingRatio[p]);

				for (int n = 0; n < vms.detour_node_sequence[p].size(); n++)
				{
					_proxy_ABM_log(-1, "%d->", vms.detour_node_sequence[p][n]);

				}
				_proxy_ABM_log(0, "\n");


				g_UpdateAgentPathBasedOnDetour(pVeh->m_AgentID, vms.detour_node_sequence[p]);
				return;

			}
		}


	}



	if (pVeh->m_OriginNodeID < 0 || pVeh->m_DestinationNodeID < 0)
	{

		cout << "m_OriginNodeID < 0 || m_DestinationNodeID <0" << endl;
		getchar();

	}

	//int	processor_id = omp_get_thread_num( );  // starting from 0
	int	processor_id = 0;  // starting from 0

	NodeSize = g_PrevailingTimeNetwork_MP[processor_id].FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID,
		StartingNodeID, pVeh->m_DepartureTime,
		pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, 
		pVeh->m_DemandType, pVeh->m_VOT, NewSubPathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);


	// compare the newly calculated path and existing path

	int bSwitchFlag = 0;
	//for (int i = 0; i < NodeSize - 1; i++)
	//{

	//	if (CurrentPathLinkList[count] != NewSubPathLinkList[i])
	//		bSwitchFlag = 1;

	//	// copy the new subpath back to the current path, starting from the current location (downstream node of the current link)
	//	CurrentPathLinkList[count] = NewSubPathLinkList[i];
	//	count += 1;
	//}

	//===================MODIFIED BY QU=============================//
	//if the agent will reach destination, then he cannot enroute
	if (pVeh->m_SimLinkSequenceNo == pVeh->m_NodeSize - 2)
		return;

	int nowLinkCount = pVeh->m_SimLinkSequenceNo + 1;
	int firstDifferentIndexOfOldPath = -1;
	int firstDifferentIndexOfNewPath = -1;
	int linkCountOfNewPath = NodeSize - 1;
	//find no feasible path
	if (linkCountOfNewPath <= 0)
		return;

	for (int i = 0; i < NodeSize - 1; i++)
	{
		if (CurrentPathLinkList[nowLinkCount] != NewSubPathLinkList[i])
		{
			firstDifferentIndexOfOldPath = nowLinkCount;
			firstDifferentIndexOfNewPath = i;
			bSwitchFlag = 1;
			break;
		}
		nowLinkCount++;
	}

	//combine the two path
	int sizeOfDifferentSubPath = linkCountOfNewPath - firstDifferentIndexOfNewPath;
	if (bSwitchFlag)
	{
		for (int i = firstDifferentIndexOfOldPath; i < MAX_NODE_SIZE_IN_A_PATH; i++)
		{
			if (CurrentPathLinkList[i] == -1)
				break;
			else
			{
				CurrentPathLinkList[i] = -1;
			}
		}

		for (int i = 0; i < sizeOfDifferentSubPath; i++)
		{
			CurrentPathLinkList[firstDifferentIndexOfOldPath + i] = NewSubPathLinkList[firstDifferentIndexOfNewPath + i];
		}
	}

	NodeSize = firstDifferentIndexOfOldPath + sizeOfDifferentSubPath + 1;

	/*	for (int i = 0; i < NodeSize - 1; i++)
		{
		DTALink* pLink  = g_LinkIDMap[CurrentPathLinkList[i]];
		TRACE("%d ->%d\n", pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);

		}*/

	//if (NodeSize <= 1)
	//	return;

	//========================END=========================//


	//			// add link to VMS respons link
	//#ifdef  _large_memory_usage_lr
	//			bool bLinkAdded = false;
	//			for(int i =0; i< pVeh->m_VMSResponseVector.size(); i++)
	//			{
	//				if(pVeh->m_VMSResponseVector[i].LinkNo == CurrentLinkID)
	//				{
	//					bLinkAdded = true;
	//					break;
	//				}
	//
	//			}
	//
	//			if(!bLinkAdded)
	//			{
	//				DTAVMSRespone vms;
	//				vms.LinkNo = CurrentLinkID;
	//				vms.ResponseTime = current_time;
	//				vms.SwitchFlag = bSwitchFlag;
	//				pVeh->m_VMSResponseVector.push_back (vms);
	//
	//			}
	//#endif

	//the new path is too long!!!!! Don't switch to new path.
	if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH)
		return;
	//if (isChange == false)
	if (true)
	{
		//return;
	/*}
	else
	{*/
		if (bSwitchFlag == 0)
			return;

		if (pVeh->m_LinkAry != NULL)  // delete the old path
		{
			pVeh->StoreOriginalPath();
			delete pVeh->m_LinkAry;
		}




		pVeh->m_NodeSize = NodeSize;


		if (pVeh->m_NodeSize >= 2)  // for feasible path
		{

			if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH)
			{
				cout << "PATH Size is out of bound: " << MAX_NODE_SIZE_IN_A_PATH << NodeSize;
				g_ProgramStop();
			}

			pVeh->m_LinkAry = new SVehicleLink[NodeSize];

			if (pVeh->m_LinkAry == NULL)
			{
				cout << "Insufficient memory for allocating vehicle arrays!";
				g_ProgramStop();
			}

			int NodeNumberSum = 0;
			float Distance = 0;

			for (int i = 0; i < NodeSize - 1; i++)
			{
				//==============MODIFIED BY QU==================//
				if (AbsArrivalTimeOnDSNVector[i] == 0)
					AbsArrivalTimeOnDSNVector[i] = 99999.0;
				//==============================================//

				pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN = AbsArrivalTimeOnDSNVector[i];
				pVeh->m_LinkAry[i].LinkNo = CurrentPathLinkList[i];
				NodeNumberSum += CurrentPathLinkList[i];

				/*if(g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL)
				{
				cout << "Error: g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL", pVeh->m_LinkAry [i].LinkNo;
				getchar();
				exit(0);
				}
				*/
				ASSERT(pVeh->m_LinkAry[i].LinkNo < g_LinkVector.size());
				Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

			}


			pVeh->m_Distance = Distance;
			pVeh->m_NodeNumberSum = NodeNumberSum;
		}
	}
}

void g_UpdateAgentPathBasedOnDetour(int VehicleID, std::vector<int> detour_node_sequence)
{
	if (g_ODEstimationFlag == 1)
		return;

	int CurrentPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { 0 };
	int NewSubPathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { 0 };
	float AbsArrivalTimeOnDSNVector[MAX_NODE_SIZE_IN_A_PATH] = { 0 };
	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;


	DTAVehicle* pVeh = g_VehicleMap[VehicleID];


	if (pVeh->m_NodeSize >= MAX_NODE_SIZE_IN_A_PATH - 1)
	{
		cout << "pVeh->m_NodeSize > MAX_NODE_SIZE_IN_A_PATH" << endl;
		g_ProgramStop();
	}

	// if this is a pre-trip vehicle, and he has not obtained real-time information yet

	float TotalCost = 0;
	bool bGeneralizedCostFlag = false;
	bool bDebugFlag = false;

	int StartingNodeID = pVeh->m_OriginNodeID;

	int CurrentLinkID = pVeh->m_LinkAry[pVeh->m_SimLinkSequenceNo].LinkNo;
	StartingNodeID = g_LinkVector[CurrentLinkID]->m_ToNodeID;

	if (StartingNodeID == pVeh->m_DestinationNodeID)  // you will reach the destination (on the last link).
		return;

	// copy existing link number (and remaining links)


	for (int i = 0; i < pVeh->m_NodeSize - 1; i++)
	{
		CurrentPathLinkList[i] = pVeh->m_LinkAry[i].LinkNo;
		AbsArrivalTimeOnDSNVector[i] = pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN;
	}


	_proxy_ABM_log(0, "** original agent %d's path sequence = ", pVeh->m_AgentID);
	for (int i = 0; i < pVeh->m_NodeSize - 1; i++)
	{
		int link_no = pVeh->m_LinkAry[i].LinkNo;

		int from_node_number = g_LinkVector[link_no]->m_FromNodeNumber;

		_proxy_ABM_log(-1, "%d;", from_node_number);

		if (i == pVeh->m_NodeSize - 2)  // last link
		{
			int to_node_number = g_LinkVector[link_no]->m_ToNodeNumber;

			_proxy_ABM_log(-1, "%d;", to_node_number);

		}
	}

	_proxy_ABM_log(-1, "\n");


	int count = pVeh->m_SimLinkSequenceNo;

	count += 1; // for detour


	std::vector<int> detour_link_sequence;


	for (int i = 0; i < detour_node_sequence.size(); i++)
	{

		if (i >= 1)
		{
			if (g_LinkMap.find(GetLinkStringID(detour_node_sequence[i - 1], detour_node_sequence[i])) != g_LinkMap.end())
			{

				DTALink* pDetourLink = g_LinkMap[GetLinkStringID(detour_node_sequence[i - 1], detour_node_sequence[i])];
				if (pDetourLink != NULL)
				{

					detour_link_sequence.push_back(pDetourLink->m_LinkNo);
				}
			}
		}

	}


	for (int ll = 0; ll < detour_link_sequence.size(); ll++)
	{
		CurrentPathLinkList[count++] = detour_link_sequence[ll];
		//			CurrentTime+= g_LinkVector[is.DetourLinkArray [ll]]->m_FreeFlowTravelTime;  // add travel time along the path to get the starting time at the end of subpath
	}


	if (detour_link_sequence.size() >= 1)
		StartingNodeID = g_LinkVector[detour_link_sequence[detour_link_sequence.size() - 1]]->m_ToNodeID;
	else if (pVeh->m_SimLinkSequenceNo >= 0)
		StartingNodeID = g_LinkVector[pVeh->m_SimLinkSequenceNo]->m_ToNodeID;

	if (pVeh->m_OriginNodeID < 0 || pVeh->m_DestinationNodeID < 0)
	{

		cout << "m_OriginNodeID < 0 || m_DestinationNodeID <0" << endl;
		getchar();

	}

	//int	processor_id = omp_get_thread_num( );  // starting from 0
	int	processor_id = 0;  // starting from 0

	NodeSize = g_PrevailingTimeNetwork_MP[processor_id].FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, StartingNodeID, pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, NewSubPathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);


	// compare the newly calculated path and existing path
	int bSwitchFlag = 0;

	if (detour_node_sequence.size() > 0)  // with external given detour
		bSwitchFlag = 1;

	for (int i = 0; i < NodeSize - 1; i++)
	{

		if (CurrentPathLinkList[count] != NewSubPathLinkList[i])
			bSwitchFlag = 1;

		// copy the new subpath back to the current path, starting from the current location (downstream node of the current link)
		CurrentPathLinkList[count] = NewSubPathLinkList[i];
		count += 1;
	}
	
	NodeSize = count + 1;

	if (NodeSize <= 1)
		return;



	if (bSwitchFlag == 0)
		return;

	if (pVeh->m_LinkAry != NULL)  // delete the old path
	{
		pVeh->StoreOriginalPath();
		delete pVeh->m_LinkAry;
	}

	
	pVeh->m_NodeSize = NodeSize;


	if (pVeh->m_NodeSize >= 2)  // for feasible path
	{

		if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH)
		{
			cout << "PATH Size is out of bound: " << MAX_NODE_SIZE_IN_A_PATH << NodeSize;
			g_ProgramStop();
		}

		pVeh->m_LinkAry = new SVehicleLink[NodeSize];

		if (pVeh->m_LinkAry == NULL)
		{
			cout << "Insufficient memory for allocating vehicle arrays!";
			g_ProgramStop();
		}

		int NodeNumberSum = 0;
		float Distance = 0;

		for (int i = 0; i < NodeSize - 1; i++)
		{
			pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN = AbsArrivalTimeOnDSNVector[i];

			pVeh->m_LinkAry[i].LinkNo = CurrentPathLinkList[i];
			NodeNumberSum += CurrentPathLinkList[i];

			/*if(g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL)
			{
			cout << "Error: g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL", pVeh->m_LinkAry [i].LinkNo;
			getchar();
			exit(0);
			}
			*/
			ASSERT(pVeh->m_LinkAry[i].LinkNo < g_LinkVector.size());
			Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

		}

		_proxy_ABM_log(0, "** after path switch, updated agent %d's path sequence = ", pVeh->m_AgentID);
		for (int i = 0; i < pVeh->m_NodeSize - 1; i++)
		{
			int link_no = pVeh->m_LinkAry[i].LinkNo;

			int from_node_number = g_LinkVector[link_no]->m_FromNodeNumber;

			_proxy_ABM_log(-1, "%d;", from_node_number);

			if (i == pVeh->m_NodeSize - 2)  // last link
			{
				int to_node_number = g_LinkVector[link_no]->m_ToNodeNumber;

				_proxy_ABM_log(-1, "%d;", to_node_number);

			}
		}

		bool bOutputDebugLogFile = true;
		if (bOutputDebugLogFile)
		{
		
			fprintf(g_DebugLogFile, "switching path for agent %d; updated path sequence = ", pVeh->m_AgentID);


		for (int i = 0; i < pVeh->m_NodeSize - 1; i++)
		{
			int link_no = pVeh->m_LinkAry[i].LinkNo;
	/*		DTALink* pLink = g_LinkMap[GetLinkStringID(usn, dsn)]; */
			int from_node_number = g_LinkVector[link_no]->m_FromNodeNumber;

			fprintf(g_DebugLogFile, "%d;", from_node_number);

			if (i == pVeh->m_NodeSize - 2)  // last link
			{
				int to_node_number = g_LinkVector[link_no]->m_ToNodeNumber;

				fprintf(g_DebugLogFile, "%d;", to_node_number);

			}
		}

		fprintf(g_DebugLogFile, "\n");

		}

		_proxy_ABM_log(-1, "\n");
		pVeh->m_Distance = Distance;
		pVeh->m_NodeNumberSum = NodeNumberSum;
	}
}

void g_UpdateAgentPathBasedOnNewDestinationOrDepartureTime(int VehicleID)
{
	int	processor_id = 0;  // starting from 0
	DTAVehicle* pVeh = g_VehicleMap[VehicleID];

	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH] = { 0 };
	float TotalCost = 0;
	bool bGeneralizedCostFlag = false;
	bool bDebugFlag = false;

	int NodeSize = g_PrevailingTimeNetwork_MP[processor_id].FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, 
		pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, 
		pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);

	if (pVeh->m_LinkAry != NULL)  // delete the old path
	{
		pVeh->StoreOriginalPath();
		delete pVeh->m_LinkAry;
	}


	pVeh->m_NodeSize = NodeSize;


	if (pVeh->m_NodeSize >= 2)  // for feasible path
	{

		if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH)
		{
			cout << "PATH Size is out of bound: " << MAX_NODE_SIZE_IN_A_PATH << NodeSize;
			g_ProgramStop();
		}

		pVeh->m_LinkAry = new SVehicleLink[NodeSize];

		if (pVeh->m_LinkAry == NULL)
		{
			cout << "Insufficient memory for allocating vehicle arrays!";
			g_ProgramStop();
		}

		int NodeNumberSum = 0;
		float Distance = 0;

		for (int i = 0; i< NodeSize - 1; i++)
		{
			pVeh->m_LinkAry[i].AbsArrivalTimeOnDSN = 0;

			pVeh->m_LinkAry[i].LinkNo = PathLinkList[i];
			NodeNumberSum += PathLinkList[i];

			/*if(g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL)
			{
			cout << "Error: g_LinkVector[pVeh->m_LinkAry [i].LinkNo]==NULL", pVeh->m_LinkAry [i].LinkNo;
			getchar();
			exit(0);
			}
			*/
			ASSERT(pVeh->m_LinkAry[i].LinkNo < g_LinkVector.size());
			Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

		}


		pVeh->m_Distance = Distance;
		pVeh->m_NodeNumberSum = NodeNumberSum;
	}
}
void g_ReadRealTimeSimulationSettingsFile()
{

	// we enable information updating for real-time simulation mode
	g_bInformationUpdatingAndReroutingFlag = true;

	int record_count = 0;
	_proxy_ABM_log(0, "read configuration from DTASettings.ini.");

	std::string timestamp_in_string;
	std::string break_point_flag;
	int output_TD_start_time_in_min = 0;

	int output_MOE_aggregation_time_interval_in_min = 1;
	int update_attribute_aggregation_time_interval_in_min = 1;



	std::string output_TD_link_travel_time_file, output_TD_link_MOE_file,
		update_trip_file, update_TD_link_attribute_file;

	int day_no = 0;

	int start_time_in_second = g_DemandLoadingStartTimeInMin * 60;
	int end_time_in_second = g_DemandLoadingEndTimeInMin * 60;

	int RT_Input_Update_Agent = g_GetPrivateProfileInt("ABM_integration", "RT_Input_Agent_Frequency_in_Seconds", 0, g_DTASettingFileName);
	int RT_Input_LinkAttribute = RT_Input_Update_Agent;
	int RT_Input_Routing_Policy = g_GetPrivateProfileInt("ABM_integration", "RT_Input_Routing_Policy_Frequency_in_Seconds", 0, g_DTASettingFileName);


	int RT_Output_LinkMOE = g_GetPrivateProfileInt("ABM_integration", "RT_Output_LinkMOE_Frequency_in_Seconds", 0, g_DTASettingFileName);
	_proxy_ABM_log(0, "-- output link MOE every %d (min)\n", RT_Output_LinkMOE / 60);

	int RT_Output_PathMOE = 15 * 60;
	int RT_Output_ODMOE = g_GetPrivateProfileInt("ABM_integration", "RT_Output_TDODMOE_in_Seconds", 0, g_DTASettingFileName);
	int RT_Output_Complete_Agent = g_GetPrivateProfileInt("ABM_integration", "RT_Output_Complete_Agent_Frequency_in_Seconds", 0, g_DTASettingFileName);
	int RT_Output_Tagged_Agent = g_GetPrivateProfileInt("ABM_integration", "RT_Output_Tagged_Agent_Frequency_in_Seconds", 0, g_DTASettingFileName);;
	int RT_Input_DMS = -1;

	_proxy_ABM_log(0, "Real time link MOE output time period: %d->%d (sec),  %d->%d (min), for every %d sec \n",
		start_time_in_second, end_time_in_second, start_time_in_second / 60, end_time_in_second / 60, RT_Output_LinkMOE);

	_proxy_ABM_log(0, "Real time path MOE output for every %d sec (%d min) \n", RT_Output_PathMOE, RT_Output_PathMOE / 60);
	_proxy_ABM_log(0, "Real time OD MOE output for every %d sec  (%d min) \n", RT_Output_ODMOE, RT_Output_ODMOE / 60);
	_proxy_ABM_log(0, "Real time trip-complete agent output for every %d sec  (%d min) \n", RT_Output_Complete_Agent, RT_Output_Complete_Agent / 60);
	_proxy_ABM_log(0, "Real time tagged agent output (based on input_tag_settings.csv)  for every %d sec  (%d min) \n", RT_Output_Tagged_Agent, RT_Output_Tagged_Agent / 60);
	_proxy_ABM_log(0, "Real time link attribute input for every %d sec  (%d min) \n", RT_Input_LinkAttribute, RT_Input_LinkAttribute / 60);
	_proxy_ABM_log(0, "Real time link attribute input: Accepted Field: from_node_id,to_node_id,link_outflow_capacity,link_storage_capacity,speed_limit,demand_type1,demand_type2,demand_typeX\n");
	_proxy_ABM_log(0, "Real time agent input for every %d sec  (%d min) \n", RT_Input_Update_Agent, RT_Input_Update_Agent / 60);
	_proxy_ABM_log(0, "Real time DMS input for every %d sec  (%d min) \n", RT_Input_DMS, RT_Input_DMS / 60);
	_proxy_ABM_log(0, "Real time routing policy  input for every %d sec  (%d min) \n", RT_Input_Routing_Policy, RT_Input_Routing_Policy / 60);

	for (int timestamp_in_second = start_time_in_second; timestamp_in_second <= end_time_in_second; timestamp_in_second++)
	{
		g_RealTimeSimulationSettingsMap[timestamp_in_second].break_point_flag = true;

		if (break_point_flag.find("break_and_wait") != string::npos)
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].break_and_wait_flag = true;

		}

		int hour = timestamp_in_second / 3600;
		int min = (timestamp_in_second - hour * 3600) / 60;
		int sec = timestamp_in_second - hour * 3600 - min * 60;



		CString time_str;
		time_str.Format("_%02dh%02dm%02ds", hour, min, sec);

		string timestamp_str = CString2StdString(time_str);


		if (RT_Output_LinkMOE >= 1 && timestamp_in_second%RT_Output_LinkMOE == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].output_TD_link_MOE_file = "RT_Output_LinkMOE" + timestamp_str + ".csv";
			g_RealTimeSimulationSettingsMap[timestamp_in_second].output_TD_aggregation_time_in_min = RT_Output_LinkMOE / 60;
		}
		if (RT_Output_ODMOE >= 1 && timestamp_in_second%RT_Output_ODMOE == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].output_od_moe_file = "RT_Output_ODMOE" + timestamp_str + ".csv";
		}
		if (RT_Output_Complete_Agent >= 1 && timestamp_in_second%RT_Output_Complete_Agent == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].output_trip_file = "RT_Output_Complete_Agent" + timestamp_str + ".csv";
		}

		if (RT_Output_Tagged_Agent >= 1 && timestamp_in_second%RT_Output_Tagged_Agent == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].output_tag_agent_file = "RT_Output_Tagged_Agent" + timestamp_str + ".csv";
		}

		if (RT_Input_LinkAttribute >= 1 && timestamp_in_second%RT_Input_LinkAttribute == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].update_TD_link_attribute_file = "RT_Input_LinkAttribute" + timestamp_str + ".csv";
		}
		if (RT_Input_Update_Agent >= 1 && timestamp_in_second%RT_Input_Update_Agent == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].update_trip_file = "RT_Input_Agent" + timestamp_str + ".csv";
		}


		if (RT_Input_Routing_Policy >= 1 && timestamp_in_second%RT_Input_Routing_Policy == 0)  // time resolution
		{
			g_RealTimeSimulationSettingsMap[timestamp_in_second].update_routing_policy_file = "RT_Input_Routing_Policy" + timestamp_str + ".csv";
		}
		record_count++;
	}


	//cout << "File input_simulation_schedule.csv has " << record_count << " file records." << endl;
	//g_LogFile << "File input_simulation_schedule.csv has " << record_count << " file records." << endl;

}

bool g_ReadDMSScenarioFile(string FileName)
{

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->MessageSignVector.clear(); // remove all previouly read records

	}

	FILE* st = NULL;

	fopen_s(&st, FileName.c_str(), "r");
	if (st != NULL)
	{

		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;

		g_read_a_line(st);
		int count = 0;
		while (true)
		{
			int usn = g_read_integer(st);
			if (usn <= 0)
				break;

			int dsn = g_read_integer(st);
			if (g_LinkMap.find(GetLinkStringID(usn, dsn)) == g_LinkMap.end())
			{
				cout << "Link " << usn << "-> " << dsn << " at line " << count + 1 << " of file" << FileName << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}

			DTALink* pLink = g_LinkMap[GetLinkStringID(usn, dsn)];

			if (pLink != NULL)
			{
				MessageSign is;

				is.Type = 1;
				is.StartDayNo = 0;
				is.EndDayNo = 100;

				is.StartTime = 0;
				is.EndTime = 1440;
				is.NumberOfSubpaths = g_read_integer(st);
				_proxy_ABM_log(0, "reading input DMS file on link %d->%d with %d subtours\n", usn, dsn, is.NumberOfSubpaths);

				for (int path = 0; path < is.NumberOfSubpaths; path++)
				{
					int number_of_nodes = g_read_integer(st);

					_proxy_ABM_log(0, "reading input DMS file on subtour %d, number of nodes in subpath = %d\n", path + 1, number_of_nodes);

					for (int i = 0; i < number_of_nodes; i++)
					{
						int node_number = g_read_integer(st);
						is.detour_node_sequence[path].push_back(node_number);
						_proxy_ABM_log(0, "reading input DMS file: node %d \n", node_number);

					}

					is.DetourSwitchingRatio[path] = g_read_float(st);
					_proxy_ABM_log(0, "reading input DMS file: switching ratio = %f \n", is.DetourSwitchingRatio[path]);

				}
				

					g_bInformationUpdatingAndReroutingFlag = true;

					pLink->MessageSignVector.push_back(is);
					count++;
			}
		}

		fclose(st);
		return true;
	}

	return false;
}

void g_ExchangeRealTimeSimulationData(int day_no,int timestamp_in_second)
{



	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].output_TD_link_MOE_file.size() >= 1)
	{
		  OutputRealTimeLinkMOEData(
			  g_RealTimeSimulationSettingsMap[timestamp_in_second].output_TD_link_MOE_file,
		  timestamp_in_second/60,
			   g_RealTimeSimulationSettingsMap[timestamp_in_second].output_TD_aggregation_time_in_min,
				false);
	}



	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].output_trip_file.size() >= 1)
	{
		   // output complete agent  files;

		while (1)
		{
		char fname_trip[_MAX_PATH];

		sprintf(fname_trip, "%s", g_RealTimeSimulationSettingsMap[timestamp_in_second].output_trip_file.c_str());
		int output_mode = 0;
		bool FileOutput = OutputTripFile(fname_trip, output_mode);
		
			if(FileOutput)
			{
				break;
			}
			else 
			{
				 Sleep(5000); // wait for 5 second
		
			 cout << "wait for 5 seconds... " << endl;
		
			}
		}

	}


	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].output_tag_agent_file.size() >= 1)
	{
		// output complete agent  files;

		while (1)
		{
			char fname_trip[_MAX_PATH];

			sprintf(fname_trip, "%s", g_RealTimeSimulationSettingsMap[timestamp_in_second].output_tag_agent_file.c_str());
			g_ReadAgentTagSettings();

			int output_mode = 1;
			bool FileOutput = OutputTripFile(fname_trip, output_mode);

			if (FileOutput)
			{
				break;
			}
			else
			{
				Sleep(5000); // wait for 5 second

				cout << "wait for 5 seconds... " << endl;

			}
		}

	}

	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].output_od_moe_file.size() >= 1)
	{
		int timestamp_in_min = timestamp_in_second / 60;

		g_SkimMatrixGenerationForAllDemandTypes(g_RealTimeSimulationSettingsMap[timestamp_in_second].output_od_moe_file,
			false, timestamp_in_min);
	}

	
		  // with input_TD_travel_time_file


	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].update_trip_file.size() >= 1)
	{
		// wait for input_agent_updating_file;
		while (1)
		{
			int number_of_vehicles = 0;

			cout << "Reading input update agent file " << g_RealTimeSimulationSettingsMap[timestamp_in_second].update_trip_file<<endl;
			_proxy_ABM_log(0, "reading input update agent file %s\n", g_RealTimeSimulationSettingsMap[timestamp_in_second].update_trip_file.c_str());

			bool b_trip_file_ready = g_ReadTripCSVFile(g_RealTimeSimulationSettingsMap[timestamp_in_second].update_trip_file.c_str(), false);


			if (b_trip_file_ready)
			{

				g_UpdateRealTimeLinkAttributes();  // from scenario workzone

				int iteration = 0;

				bool bRebuildNetwork = false;

				//if(timestamp_in_min%15 ==0)  //rebuild the shoret path network every 15 min
				//	bRebuildNetwork = true;


				// use input link travel time 
				g_WithIterationPathBuildingForAllAgents(iteration, bRebuildNetwork, false, timestamp_in_second / 60, timestamp_in_second / 60 + 15);

				break;
			}
			else
			{
				Sleep(5000); // wait for 5 second
				cout << "wait for 5 seconds... " << endl;

			}


		}
	}

	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].update_DMS_file.size() >= 1)
	{
		// wait for input_agent_updating_file;
		while (1)
		{
			int number_of_vehicles = 0;

			cout << "Reading input DMS file " << g_RealTimeSimulationSettingsMap[timestamp_in_second].update_DMS_file;
			_proxy_ABM_log(0, "reading input DMS file %s\n", g_RealTimeSimulationSettingsMap[timestamp_in_second].update_DMS_file.c_str());

			bool b_trip_file_ready = g_ReadDMSScenarioFile(g_RealTimeSimulationSettingsMap[timestamp_in_second].update_DMS_file);

			if (b_trip_file_ready)
			{
				break;
			}
			else
			{
				Sleep(5000); // wait for 5 second
				cout << "wait for 5 seconds... " << endl;

			}


		}
	}


	if (g_RealTimeSimulationSettingsMap[timestamp_in_second].update_routing_policy_file.size() >= 1)
	{
		// wait for input_agent_updating_file;
		while (1)
		{
			int number_of_vehicles = 0;

			cout << "Reading input routing policy file " << g_RealTimeSimulationSettingsMap[timestamp_in_second].update_routing_policy_file;
			_proxy_ABM_log(0, "reading input routing policyfile %s\n", g_RealTimeSimulationSettingsMap[timestamp_in_second].update_routing_policy_file.c_str());

			// update routing policy in the simulator's memory 
			bool b_routing_policy_file_ready = g_ReadTimeDependentRoutingPolicyData(g_RealTimeSimulationSettingsMap[timestamp_in_second].update_routing_policy_file.c_str(), timestamp_in_second/60);

	
			if (b_routing_policy_file_ready)
			{
				break;
			}
			else
			{
				Sleep(5000); // wait for 5 second
				cout << "wait for 5 seconds... " << endl;

			}


		}
	}
}

void g_RealTimeDynamicToll(int day_no, int timestamp_in_min)
{
	// update timestamp using day no
	timestamp_in_min = day_no * 1440 + timestamp_in_min;

	// part I: determine dynamic tolling price 
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink * pLink = g_LinkVector[li];


		if (pLink->m_DynamicTollType == 1)
		{
			int DemandType = 1;
			float speed = pLink->m_Length / max(pLink->m_FreeFlowTravelTime, pLink->m_prevailing_travel_time) * 60;  // mile per hour
			float speed_limit_over_speed = pLink->m_SpeedLimit / max(0.01, speed);
			float voc = pow((speed_limit_over_speed - 1) / 0.15, 0.25);
			float min_rate = 0.25;
			float max_rate = 3;
			float dynamic_toll_rate = 1;
			dynamic_toll_rate = (max_rate - min_rate) * pow(voc - 0.15, 2) * pLink->pTollVector[0].TollRate[2];
			if (dynamic_toll_rate < min_rate)
				dynamic_toll_rate = min_rate;

			if (dynamic_toll_rate > max_rate)
				dynamic_toll_rate = max_rate;


			pLink->pTollVector[0].TollRate[DemandType] = dynamic_toll_rate;

			if (g_DebugLogFile != NULL)
			{
				fprintf(g_DebugLogFile, "\nDay NO: %d, Min: %d, Link %d->%d, speed: %f, speed_limit_over_speed: %f, voc: %f, rate: %f  ", day_no, timestamp_in_min, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber,
					speed, speed_limit_over_speed, voc, dynamic_toll_rate);
			}

		}


	}
}

void g_UpdateRealTimeLinkAttributes()
{
	// read scenario file
	g_ReadScenarioInputFiles(0);

	fprintf(g_DebugLogFile, "reading scenario file ...\n" );

}
