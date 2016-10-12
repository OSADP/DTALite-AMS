//  Portions Copyright 2010 Xuesong Zhou, Jason Lu

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


using namespace std;
extern CTime g_AppLastIterationStartTime;


std::map<CString, int> g_path_index_map;

bool IsWithinODMEIteration(int iteration)
{
	
	if (g_ODEstimationFlag == 1 && iteration >= g_ODEstimation_StartingIteration && iteration < g_ODEstimation_EndingIteration)
		return true;
	else
		return false;

}

float g_GetSwitchingRate(int iteration, float ExperiencedTravelCost, float ComputedShortestPathCost)
{
	float relative_gap = (max(0, ExperiencedTravelCost - ComputedShortestPathCost)) / max(0.1, ComputedShortestPathCost);

	float adjustment_factor = 1.0;

	float switching_rate = max(0.0001, 1.0f / (iteration + 1)*adjustment_factor);

	if (g_ODEstimationFlag == 1 && iteration >= g_ODEstimation_EndingIteration)
	{
		if (iteration <= g_ODEstimation_EndingIteration + 1)
			switching_rate = 0;
		else if(iteration >= g_ODEstimation_EndingIteration + 2)
			switching_rate = 0.05;
	}

	return switching_rate;
}


bool g_GetSequentialUEAdjustmentTimePeriod(int iteration, float DepartureTime)
{
	// 20 iterations for each hour adjustment period

	int intervier_with_in_a_cycle = max(0, (iteration - g_ODEstimation_StartingIteration)% g_NumberOfSPCalculationPeriods);

	if (g_FindAssignmentIntervalIndexFromTime(DepartureTime) == intervier_with_in_a_cycle )  //within time interval
		return true;
	else
		return false;

}

DTANetworkForSP g_TimeDependentNetwork_MP[_MAX_NUMBER_OF_PROCESSORS]; //  network instance for single processor in multi-thread environment: no more than 8 threads/cores

void g_WithIterationPathBuildingForAllAgents(int iteration, bool bRebuildNetwork, bool bOutputLog, int DemandLoadingStartTime, int DemandLoadingEndTime)
{
	if (IsWithinODMEIteration(iteration))  // perform path flow adjustment after at least 10 normal OD estimation
	{
		g_SystemDemand.StoreDemandParametersFromLastIteration();
	
	}



	// assign different zones to different processors
	int number_of_threads = omp_get_max_threads();

	if (bOutputLog)
	{
		cout << "# of Computer Processors = " << number_of_threads << endl;
	}


	number_of_threads = g_number_of_CPU_threads();

#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{
		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0

		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors
		if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
		{
			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(iteration, -1, g_TrafficFlowModelFlag);  // build network for this zone, because different zones have different connectors...
		}
		if (bOutputLog)
		{
			cout << "---- agent-based routing and assignment at processor " << ProcessID + 1 << endl;
		}
		for (int CurZoneID = 1; CurZoneID <= g_ODZoneNumberSize; CurZoneID++)
		{

			if (g_ZoneMap.find(CurZoneID) == g_ZoneMap.end())  // no such zone being defined
				continue;

			if ((CurZoneID%number_of_threads) == ProcessID)  // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 
			{

				// scan all possible departure times
				for (int departure_time_index = 0; departure_time_index < g_NumberOfSPCalculationPeriods; departure_time_index ++)
				{

					int departure_time = g_AssignmentIntervalStartTimeInMin[departure_time_index];
					int  departure_end_time = g_AssignmentIntervalEndTimeInMin[departure_time_index];

					int vehicle_size = g_TDOVehicleArray[g_ZoneMap[CurZoneID].m_ZoneSequentialNo][departure_time_index].VehicleArray.size();
					
					if (vehicle_size > 0)
					{
						float node_size = 1;
						if (IsWithinODMEIteration(iteration))  // perform path flow adjustment after at least 10 normal OD estimation
							g_TimeDependentNetwork_MP[id].ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval_ODEstimation(CurZoneID, departure_time_index, iteration);
						else
							node_size = g_TimeDependentNetwork_MP[id].AgentBasedPathFindingForEachZoneAndDepartureTimeInterval(CurZoneID, departure_time, departure_end_time, iteration);



						if (g_ODZoneNumberSize > 1000 && departure_time == g_DemandLoadingStartTimeInMin)  // only for large networks and zones with data
						{

							if (bOutputLog)
							{
								if (IsWithinODMEIteration(iteration))  // perform path flow adjustment after at least 10 normal OD estimation
									cout << "Processor " << id << " is adjusting OD demand table for zone " << CurZoneID << endl;
								else
									cout << "Processor " << id << " is calculating the shortest paths for zone " << CurZoneID << " with " << vehicle_size << " agents." << endl;

							}
						}
					}


				}  // for each departure time
			}
		}  // for each zone
	} // for each computer processor

	if (bOutputLog)
	{
		cout << ":: complete assignment " << g_GetAppRunningTime() << endl;
		g_LogFile << ":: complete assignment " << g_GetAppRunningTime() << endl;

	}
}
void g_AgentBasedDynamicTrafficAssignmentSimulation()  // this is an adaptation of OD trip based assignment, we now generate and assign path for each individual vehicle (as an agent with personalized value of time, value of reliability)
{
	// reset random number seeds
	int node_size = g_NodeVector.size() + 1 + g_ODZoneIDSize;
	int link_size = g_LinkVector.size() + g_NodeVector.size(); // maximal number of links including connectors assuming all the nodes are destinations

	cout << ":: start assignment " << g_GetAppRunningTime() << endl;
	g_LogFile << ":: start assignment " << g_GetAppRunningTime() << endl;


	int iteration = 0;
	bool NotConverged = true;
	int TotalNumOfVehiclesGenerated = 0;



	//cout << "------- Allocating memory for networks for " number_of_threads << " CPU threads" << endl;
	//cout << "Tips: If your computer encounters a memory allocation problem, please open file DTASettings.txt in the project folder " << endl;
	//cout << "find section [computation], set max_number_of_threads_to_be_used=1 or a small value to reduce memory usage. " << endl;
	//cout << "This modification could significantly increase the total runing time as a less number of CPU threads will be used. " << endl;

	//
	//" number_of_threads << " CPU threads" << endl;

	int number_of_threads = omp_get_max_threads();

	cout << "# of Computer Processors = " << number_of_threads << endl;


	number_of_threads = g_number_of_CPU_threads();

	// ----------* start of outer loop *----------
	for (iteration = 0; NotConverged && iteration <= g_NumberOfIterations; iteration++)  // we exit from the loop under two conditions (1) converged, (2) reach maximum number of iterations
	{
		cout << "------- Iteration = " << iteration + 1 << "--------" << endl;

		if (IsWithinODMEIteration(iteration))
			g_SystemDemand.ResetUpdatedValue(); // reset update hist table


		g_CurrentGapValue = 0.0;
		g_CurrentRelativeGapValue = 0.0;
		g_CurrentNumOfVehiclesForUEGapCalculation = 0;
		g_CurrentNumOfVehiclesSwitched = 0;
		g_NewPathWithSwitchedVehicles = 0;


		g_SetupTDTollValue(iteration);





		if (g_VehicleLoadingMode == vehicle_binary_file_mode && iteration == 0)
		{
			// do nothing	// we do not need to generate initial paths for vehicles for the first iteration of vehicle loading mode
		}
		else
		{
			g_WithIterationPathBuildingForAllAgents(iteration, true, true, g_DemandLoadingStartTimeInMin, g_DemandLoadingEndTimeInMin);
		}

		////


		if (IsWithinODMEIteration(iteration))  // re-generate vehicles based on global path set
		{
			_proxy_ODME_log(0, iteration, "--------------------------\n");

			for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
			{
				if (g_ZoneMap.find(z) == g_ZoneMap.end())
					continue;


				int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;

				int boundary_value = 0;
				if (g_SystemDemand.m_alpha[zone_index] <= g_SystemDemand.m_alpha_hist_value[zone_index] * 0.11)
					boundary_value = -1;

				if (g_SystemDemand.m_alpha[zone_index] >= g_SystemDemand.m_alpha_hist_value[zone_index] * 9.9)
					boundary_value = 1;
#pragma omp critical
				{
					float diff_value = g_SystemDemand.m_alpha[zone_index] - g_SystemDemand.m_alpha_hist_value[zone_index];

					if (boundary_value == 0)
						_proxy_ODME_log(0, iteration, "alpha[%d]= %.2f, %.1f\n", z, g_SystemDemand.m_alpha[zone_index], diff_value);
					else
						_proxy_ODME_log(0, iteration, "alpha[%d]= %.2f, %.1f, *%d\n", z, g_SystemDemand.m_alpha[zone_index], diff_value, boundary_value);
				}
			}
			g_GenerateVehicleData_ODEstimation();
		}
		cout << "---- Network Loading for Iteration " << iteration + 1 << "----" << endl;

		NetworkLoadingOutput SimuOutput;



		SimuOutput = g_NetworkLoading(g_TrafficFlowModelFlag, 0, iteration);
		g_GenerateSimulationSummary(iteration, NotConverged, TotalNumOfVehiclesGenerated, &SimuOutput);

	}  // for each assignment iteration

	cout << "Writing Vehicle Trajectory and MOE File... " << endl;

	if (iteration == g_NumberOfIterations)
	{
		iteration = g_NumberOfIterations - 1;  //roll back to the last iteration if the ending condition is triggered by "iteration < g_NumberOfIterations"
	}

	g_OutputMOEData(iteration);

}


float DTANetworkForSP::AgentBasedPathFindingForEachZoneAndDepartureTimeInterval(int zone, int departure_time_begin, int departure_time_end, int iteration)
// for vehicles starting from departure_time_begin to departure_time_end, assign them to shortest path using a proportion according to MSA or graident-based algorithms
{
	//	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH];
	int TempPathLinkList[MAX_NODE_SIZE_IN_A_PATH];

	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;

	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);

	int total_node_size = 0;
	// loop through the TDOVehicleArray to assign or update vehicle paths...

	int VehicleSize = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size();
	for (int vi = 0; vi < VehicleSize; vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		
		if (g_VehicleMap.find(VehicleID) == g_VehicleMap.end())
			continue;

		DTAVehicle* pVeh = g_VehicleMap[VehicleID];

		if (pVeh->m_AgentID == 17931 && pVeh->m_NodeSize == 0)
			TRACE("");

		pVeh->m_bConsiderToSwitch = false;
		pVeh->m_bSwitch = false;

		if (pVeh->m_InformationType == 0 && pVeh->m_NodeSize >= 2)  // skip updating the path from assignment 
			continue; 
		/// finding optimal path 
		bool bDebugFlag = false;

		float TotalCost;
		bool bGeneralizedCostFlag = false;

		// general settings

		int final_departuret_time_shift = 0;  // unit: min
		TempNodeSize = 0;
		float switch_threshold = 0;   // 2 min for day to day learning, - 100 min for  special case when using MSA, so 15% of agents will be selected for swtiching for sure. 

		// 2 min for day to day learning, 
		// -100 min for  special case when using MSA
		// +100 will disallow any departure switch, because the benchmark (experienced travel time - 100) is too low. 


		float switching_rate = g_GetSwitchingRate(iteration, pVeh->m_TripTime, TotalCost);   // default switching rate 



		if (pVeh->m_OriginZoneID == pVeh->m_DestinationZoneID)
		{  // do not simulate intra zone traffic
			continue;
		}

		if (g_use_routing_policy_from_external_input == 1 && pVeh->m_NodeSize >= 1 && iteration == 0)
		{
			// no need to re-assign the traffic path when loading vehicle data from routing policy at first iteration 
			continue;
		}

		double RandomNumber = pVeh->GetRandomRatio();  // vehicle-dependent random number generator, very safe for multi-thread applications			


		bool bSwitchFlag = false;

		float ExperiencedTravelTime = pVeh->m_TripTime;
		float ExperiencedGeneralizedTravelTime = pVeh->m_TripTime + pVeh->m_TollDollarCost / max(1, pVeh->m_VOT);  // unit: min


		if (pVeh->m_AgentID == 1001)
			TRACE("");

		if (iteration == 0)  //we always switch at the first iteration
		{
			if (pVeh->m_NodeSize == 0)  // without pre-loaded path
			{
				bSwitchFlag = true;

				pVeh->m_PreferredDepartureTime = pVeh->m_DepartureTime;  // set departure time to m_PreferredDepartureTime

				NodeSize = FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);
				total_node_size += NodeSize;


				pVeh->m_bSwitch = true;
			}
			else
			{

				bSwitchFlag = false;

			}


		}
		else
		{ // iteration >=1
			if (RandomNumber < switching_rate)  // g_Day2DayAgentLearningMethod==0: no learning, just switching 
			{

				if (pVeh->m_InformationType == info_eco_so)  // for speical information type
				{

					bGeneralizedCostFlag = true;

				}
						NodeSize = FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);
						bSwitchFlag = true;


	
			}
		}

		if (pVeh->m_bForcedSwitchAtFirstIteration == true)
		{
			// m_bForcedSwitchAtFirstIteration is true when the vehicle has to pass through a totally closed work zone link
			bSwitchFlag = true;
			pVeh->m_bForcedSwitchAtFirstIteration = false; // reset the flag to false in any case, as it is effective only for the first iteration

		}

		pVeh->m_gap_update = false;

		if (bSwitchFlag)  // for all vehicles that need to switch
		{

			//pVeh->m_DepartureTime = pVeh->m_PreferredDepartureTime + final_departuret_time_shift;

			pVeh->SetMinCost(TotalCost);

			/// get shortest path only when bSwitchFlag is true; no need to obtain shortest path for every vehicle


			pVeh->m_bConsiderToSwitch = true;

			if (pVeh->m_LinkAry != NULL && pVeh->m_NodeSize > 0)  // delete the old path
			{
				delete pVeh->m_LinkAry;
			}

			pVeh->m_NodeSize = NodeSize;


			if (pVeh->m_NodeSize >= 2)  // for feasible path
			{
				pVeh->m_bConsiderToSwitch = true;


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
					pVeh->m_LinkAry[i].LinkNo = PathLinkList[i];

					if (i==0)
						NodeNumberSum += g_LinkVector[PathLinkList[i]]->m_FromNodeNumber;

					NodeNumberSum += g_LinkVector[PathLinkList[i]]->m_ToNodeNumber;


					ASSERT(pVeh->m_LinkAry[i].LinkNo < g_LinkVector.size());
					Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

				}
				if (NodeNumberSum != pVeh->m_NodeNumberSum)
				{  //switch to different  paths 
					pVeh->m_bSwitch = true;
				}


				float m_gap = ExperiencedGeneralizedTravelTime - TotalCost;

				if (m_gap < 0)
					m_gap = 0.0;

				if (NodeNumberSum == pVeh->m_NodeNumberSum)  //same path
					m_gap = 0.0;


				pVeh->m_gap_update = true;
				pVeh->m_gap = m_gap;
#pragma omp critical
				{
					g_CurrentNumOfVehiclesSwitched += 1;
					g_CurrentGapValue += m_gap; // Jason : accumulate g_CurrentGapValue only when iteration >= 1
					g_CurrentRelativeGapValue += m_gap / max(0.1, ExperiencedGeneralizedTravelTime);
					g_CurrentNumOfVehiclesForUEGapCalculation += 1;
				}


				pVeh->m_Distance = Distance;
				pVeh->m_NodeNumberSum = NodeNumberSum;



				//cout << pVeh->m_AgentID <<  " Distance" << pVeh->m_Distance <<  endl;;

			}

		}  // switch
		else
		{  // not switch but we calculate gap function
			if (g_CalculateUEGapForAllAgents ==1 )
			{
				NodeSize = FindBestPathWithVOTForSingleAgent(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, pVeh->m_DepartureTime, pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, bDebugFlag);
				total_node_size += NodeSize;

				int NodeNumberSum = 0;

				for (int i = 0; i < NodeSize - 1; i++)
				{
					NodeNumberSum += PathLinkList[i];

				}

				float m_gap = ExperiencedGeneralizedTravelTime - TotalCost;

				if (m_gap < 0)
					m_gap = 0.0;

				if (NodeNumberSum == pVeh->m_NodeNumberSum)  //same path
					m_gap = 0.0;


				pVeh->m_gap_update = true;
				pVeh->m_gap = m_gap;
#pragma omp critical
				{
					g_CurrentGapValue += m_gap; // Jason : accumulate g_CurrentGapValue only when iteration >= 1
					g_CurrentRelativeGapValue += m_gap / max(0.1, ExperiencedGeneralizedTravelTime);
					g_CurrentNumOfVehiclesForUEGapCalculation += 1;

				}


			}


		}

	} // for each vehicle on this OD pair

	return total_node_size / max(1, VehicleSize);

}





void DTANetworkForSP::ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval(int zone, int departure_time_begin, int departure_time_end, int iteration, bool debug_flag = false)
// for vehicles starting from departure_time_begin to departure_time_end, assign them to shortest path using a proportion according to MSA or graident-based algorithms
{

	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int PredNode;
	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);  // starting assignment interval


	int vehicle_id_trace = -1;


	// loop through the TDOVehicleArray to assign or update vehicle paths...
	for (int vi = 0; vi < g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size(); vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		DTAVehicle* pVeh = g_VehicleMap[VehicleID];
		ASSERT(pVeh != NULL);


		int OriginCentriod = m_PhysicalNodeSize;

		int destination_zone_no = g_ZoneMap[pVeh->m_DestinationZoneID].m_ZoneSequentialNo;
		int DestinationCentriod = m_PhysicalNodeSize + 1+ destination_zone_no;  // map m_ZoneSequentialNo to DestinationCentriod

		if (pVeh->m_InformationType == 0  && pVeh->m_NodeSize >=2)  // skip updating the path from assignment 
			continue;

		double TotalCost =  MAX_SPLABEL;
		if (g_ShortestPathWithMovementDelayFlag)  // consider movement delay
		{
			//step 1: find the incoming link has the lowerest cost to the destinatin node
			int destination = DestinationCentriod;
			int link_id_with_min_cost = -1;
			int i;
			for (i = 0; i < m_InboundSizeAry[destination]; i++)
			{
				int incoming_link = m_InboundLinkAry[destination][i];
				if (LabelCostVectorPerType[pVeh->m_DemandType][incoming_link] < TotalCost && LinkPredVectorPerType[pVeh->m_DemandType][incoming_link] >= 0)
				{
					TotalCost = LabelCostVectorPerType[pVeh->m_DemandType][incoming_link];
					link_id_with_min_cost = incoming_link;
				}
			}

			if (TotalCost > MAX_SPLABEL - 10)
			{


				{
				cout << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. Please check warning.log for details. " << endl;
				//g_WarningFile << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. " << endl;
				pVeh->m_NodeSize = 0;
				continue;
				}
			}
		}
		else  // no movement delay
		{
			if (LabelCostVectorPerType[pVeh->m_DemandType][DestinationCentriod] > MAX_SPLABEL - 10)
			{

				{
					cout << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. Please check warning.log for details. " << endl;
					//g_WarningFile << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. " << endl;
					pVeh->m_NodeSize = 0;
					continue;
				}

			}
		
			TotalCost = LabelCostVectorPerType[pVeh->m_DemandType][DestinationCentriod];

		}


		bool bSwitchFlag = false;
		pVeh->m_bConsiderToSwitch = false;

		pVeh->m_bSwitch = false;
		pVeh->m_gap_update = false;

		if (iteration > 0 && pVeh->m_InformationType != 0) // update path assignments -> determine whether or not the vehicle will switch
		{


			pVeh->SetMinCost(TotalCost);


			float switching_rate = g_GetSwitchingRate(iteration, pVeh->m_TripTime, TotalCost);

			double RandomNumber = pVeh->GetRandomRatio();  // vehicle-dependent random number generator, very safe for multi-thread applications			

			if ((pVeh->m_bComplete == false && pVeh->m_NodeSize >= 2)) //for incomplete vehicles with feasible paths, need to switch at the next iteration
			{
				bSwitchFlag = true;
			}
			else
			{
				if (RandomNumber < switching_rate)
				{
					bSwitchFlag = true;
				}
			}
		}
		else	// iteration = 0;  at iteration 0, every vehicle needs a path for simulation, so every vehicle switches
		{
			if (g_use_routing_policy_from_external_input == 1 && pVeh->m_NodeSize >= 1)
			{
				// no need to re-assign the traffic path when loading vehicle data from routing policy at first iteration 
				continue;
			}


			if (pVeh->m_NodeSize == 0)  // no external path at the first iteration 
				bSwitchFlag = true;
			else
			{


			}
		}

		if (bSwitchFlag)
		{
			// get shortest path only when bSwitchFlag is true; no need to obtain shortest path for every vehicle

		if(g_ShortestPathWithMovementDelayFlag)  // consider movement delay
			{
				//step 1: find the incoming link has the lowerest cost to the destinatin node
				int destination = DestinationCentriod;
				float min_cost = MAX_SPLABEL;
				int link_id_with_min_cost = -1;
				int i;
				for (i = 0; i< m_InboundSizeAry[destination]; i++)
				{
					int incoming_link = m_InboundLinkAry[destination][i];
					if (LabelCostVectorPerType[pVeh->m_DemandType][incoming_link] < min_cost && LinkPredVectorPerType[pVeh->m_DemandType][incoming_link] >= 0)
					{
						min_cost = LabelCostVectorPerType[pVeh->m_DemandType][incoming_link];
						link_id_with_min_cost = incoming_link;
					}
				}


				TotalCost = min_cost;

				if (link_id_with_min_cost <0)
				{

					int node_number = g_NodeVector[destination].m_NodeNumber;
				//	cout << "Destination Node " << node_number << " cannot be reached from origin zone " << zone  << endl;
					g_LogFile << "Error: Destination Node " << node_number << " cannot be reached from origin zone  " << zone << endl;
							
					continue;
					
				}

				//step 2 trace the incoming link to the first link in origin node

				int LinkSize = 0;
				temp_reversed_PathLinkList[LinkSize] = link_id_with_min_cost;  // last link to destination  // skip the last link by not using LinkSize++
				int PrevLinkID = LinkPredVectorPerType[pVeh->m_DemandType ][link_id_with_min_cost];

				if (PrevLinkID == -1)
				{
					TRACE("Error!");

				}
				temp_reversed_PathLinkList[LinkSize++] = PrevLinkID;   //second last link

				while (PrevLinkID != -1 && LinkSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
				{
					ASSERT(LinkSize< MAX_NODE_SIZE_IN_A_PATH - 1);

					if (LinkPredVectorPerType[pVeh->m_DemandType][PrevLinkID] != -1)
					{
						temp_reversed_PathLinkList[LinkSize++] = LinkPredVectorPerType[pVeh->m_DemandType][PrevLinkID];
					}
					PrevLinkID = LinkPredVectorPerType[pVeh->m_DemandType][PrevLinkID];
				}

				int j = 0;
				
				LinkSize--;
				// using LinkSize - 2 to avoid the first connnector

 			for (i = LinkSize - 1; i >= 0; i--) 
				{
					PathLinkList[j++] = temp_reversed_PathLinkList[i];
					if (debug_flag)
					{
						TRACE("\n  link no. %d, %d, %d ->%d", i, temp_reversed_PathLinkList[i]
							, m_FromIDAry[temp_reversed_PathLinkList[i]], m_ToIDAry[temp_reversed_PathLinkList[i]]);
					}

				}

				
				NodeSize = LinkSize + 1; 

				int NodeNumberSum = 0;
				for (int i = 0; i < NodeSize - 1; i++) // NodeSize-1 is the number of links along the paths
				{
					NodeNumberSum += PathLinkList[i];
				}
			
				float m_gap;
				float ExperiencedTravelTime = pVeh->m_TripTime;


				m_gap = ExperiencedTravelTime - TotalCost;
				if (m_gap < 0) m_gap = 0.0;

				if (NodeNumberSum == pVeh->m_NodeNumberSum)  //same path
					m_gap = 0.0;

				pVeh->m_gap_update = true;
				pVeh->m_gap = m_gap;

#pragma omp critical
				{
					if (pVeh->m_bComplete)
					{
						if (m_gap>1000)
							TRACE("");

						g_CurrentGapValue += m_gap; // Jason : accumulate g_CurrentGapValue only when iteration >= 1
						g_CurrentRelativeGapValue += m_gap / max(0.1, ExperiencedTravelTime);
						g_CurrentNumOfVehiclesForUEGapCalculation += 1;
						// Jason : accumulate number of vehicles switching paths
						g_CurrentNumOfVehiclesSwitched += 1;
					}
				}
				pVeh->m_bConsiderToSwitch = true;
				pVeh->m_NodeSize = NodeSize;

				if (pVeh->m_LinkAry != NULL)
				{
					delete pVeh->m_LinkAry;
				}

				if (pVeh->m_NodeSize >= 2)
				{
					pVeh->m_bConsiderToSwitch = true;

					pVeh->m_LinkAry = new SVehicleLink[NodeSize];

					if (pVeh->m_LinkAry == NULL)
					{
						cout << "Insufficient memory for allocating vehicle arrays!";
						g_ProgramStop();
					}

					pVeh->m_NodeNumberSum = 0;
					pVeh->m_Distance = 0;

					for (int i = 0; i < NodeSize - 1; i++) // NodeSize-1 is the number of links along the paths
					{
						pVeh->m_LinkAry[i].LinkNo = PathLinkList[i];

						if (i==0)
							pVeh->m_NodeNumberSum += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_FromNodeNumber;

						pVeh->m_NodeNumberSum += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_ToNodeNumber;

						if (pVeh->m_LinkAry[i].LinkNo < g_LinkVector.size())
						{
						pVeh->m_Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

						}
					}

					pVeh->m_bSwitch = true;
				}
				else
				{
					pVeh->m_bLoaded = false;
					pVeh->m_bComplete = false;


					
					if (iteration == 0)
					{
						#pragma omp critical
						{
						g_WarningFile << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. Path Cost:" << TotalCost << endl;
						}
					}
				}
			}
			else  // without movement delay information
			{

				NodeSize = 0;
				PredNode = NodePredVectorPerType[pVeh->m_DemandType][DestinationCentriod];
				while (PredNode != OriginCentriod && PredNode != -1) // scan backward in the predessor array of the shortest path calculation results
				{
					if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH - 1)
					{
#pragma omp critical
						{

							g_LogFile << "error in path finding: too many nodes: OD pair: " << zone << " -> " << pVeh->m_DestinationZoneID << endl;

							for (int i = 0; i < NodeSize; i++)
							{
								cout << "error in path finding: too many nodes: OD pair: " << zone << " -> " << pVeh->m_DestinationZoneID << endl;
								cout << "Node sequence " << i << ":node " << g_NodeVector[temp_reversed_PathLinkList[i]].m_NodeNumber << ",cost: " << LabelCostVectorPerType[pVeh->m_DemandType][temp_reversed_PathLinkList[i]] << endl;


								g_LogFile << "Node sequence " << i << ":node " << g_NodeVector[temp_reversed_PathLinkList[i]].m_NodeNumber << ",cost: " << LabelCostVectorPerType[pVeh->m_DemandType][temp_reversed_PathLinkList[i]] << endl;

							}
						}

						break;
					}

					temp_reversed_PathLinkList[NodeSize++] = PredNode;  // node index 0 is the physical node, we do not add OriginCentriod into PathNodeList, so NodeSize contains all physical nodes.
					PredNode = NodePredVectorPerType[pVeh->m_DemandType][PredNode];
				}

				// the first node in the shortest path is the super zone center, should not be counted

				int j = 0;

				int NodeNumberSum = 0;
				for (int i = NodeSize - 1; i >= 0; i--)
				{
					PathNodeList[j++] = temp_reversed_PathLinkList[i];

					ASSERT(PathNodeList[j] < m_PhysicalNodeSize);
				}

				for (int i = 0; i < NodeSize - 1; i++) // NodeSize-1 is the number of links along the paths
				{
					NodeNumberSum += PathNodeList[i];
				}

				float m_gap;
				float ExperiencedTravelTime = pVeh->m_TripTime;


				m_gap = ExperiencedTravelTime - TotalCost;
				if (m_gap < 0) m_gap = 0.0;

				if (NodeNumberSum == pVeh->m_NodeNumberSum)  //same path
					m_gap = 0.0;

				pVeh->m_gap_update = true;
				pVeh->m_gap = m_gap;
#pragma omp critical
				{
					if (pVeh->m_bComplete)
					{
					
					g_CurrentGapValue += m_gap; // Jason : accumulate g_CurrentGapValue only when iteration >= 1
					g_CurrentRelativeGapValue += m_gap / max(0.1, ExperiencedTravelTime);
					g_CurrentNumOfVehiclesForUEGapCalculation += 1;
					// Jason : accumulate number of vehicles switching paths
					g_CurrentNumOfVehiclesSwitched += 1;
					}
				}
				pVeh->m_bConsiderToSwitch = true;
				pVeh->m_NodeSize = NodeSize;

				if (pVeh->m_LinkAry != NULL)
				{
					delete pVeh->m_LinkAry;
				}

				if (pVeh->m_NodeSize >= 2)
				{
					pVeh->m_bConsiderToSwitch = true;

					pVeh->m_LinkAry = new SVehicleLink[NodeSize];

					if (pVeh->m_LinkAry == NULL)
					{
						cout << "Insufficient memory for allocating vehicle arrays!";
						g_ProgramStop();
					}

					pVeh->m_NodeNumberSum = 0;
					pVeh->m_Distance = 0;

					for (int i = 0; i < NodeSize - 1; i++) // NodeSize-1 is the number of links along the paths
					{
						pVeh->m_LinkAry[i].LinkNo = GetLinkNoByNodeIndex(PathNodeList[i], PathNodeList[i + 1]);
						pVeh->m_NodeNumberSum += PathNodeList[i];
						pVeh->m_Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;
					}
					//cout << pVeh->m_AgentID <<  " Distance" << pVeh->m_Distance <<  endl;;

					//// check whether or not this is a new path
					//int PathIndex = 0;
					//for(int p=1; p<=PathArray[pVeh->m_DestinationZoneID].NumOfPaths; p++)
					//{
					//	if(pVeh->m_NodeNumberSum == PathArray[pVeh->m_DestinationZoneID].PathNodeSums[p])
					//	{
					//		PathIndex = p;
					//		break;
					//	}
					//}
					//if(PathIndex == 0) // a new path found
					//	g_NewPathWithSwitchedVehicles++;	
					pVeh->m_bSwitch = true;

				}
				else
				{
					pVeh->m_bLoaded = false;
					pVeh->m_bComplete = false;

					if (iteration == 0)
					{
						#pragma omp critical
						{

							g_WarningFile << "Warning: vehicle " << pVeh->m_AgentID << " from zone " << pVeh->m_OriginZoneID << " to zone " << pVeh->m_DestinationZoneID << " does not have a physical path. Path Cost:" << TotalCost << endl;
						}
						}
				}
			}//without movement
		} // if(bSwitchFlag)
	}


}

void g_AgentBasedShortestPathGeneration()
{

	// find unique origin nodegf
	// find unique destination node

	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	int line = 0;

	FILE* st_input = NULL;
	fopen_s(&st_input, "input_od_pairs.csv", "r");
	if (st_input != NULL)
	{
		char str[100];

		fscanf(st_input, "%[^\n]", str);  // read a line

		int origin_node_id, destination_node_id, record_id;

		while (!feof(st_input))
		{
			origin_node_id = 0;
			destination_node_id = 0;
			record_id = 0;

			int number_of_values = fscanf(st_input, "%d,%d,%d\n", &record_id, &origin_node_id, &destination_node_id);

			// the expected number of items = 3;
			if (number_of_values < 3)
				break;

			TRACE("%d;%d;%d\n", record_id, &origin_node_id, &destination_node_id);

			if (g_NodeNametoIDMap.find(origin_node_id) == g_NodeNametoIDMap.end())
			{
				//				cout<< "origin_node_id "  << origin_node_id << " in input_od_pairs.csv has not be defined in input_node.csv.  Please check line =" << line  << endl; 
				//				getchar();
				//				exit(0);
				continue;
			}

			if (g_NodeNametoIDMap.find(destination_node_id) == g_NodeNametoIDMap.end())
			{
				//				cout<< "destination_node_id "  << destination_node_id << " in input_od_pairs.csv has not be defined in input_node.csv. Please check line =" << line   << endl; 
				//				getchar();
				//				exit(0);
				continue;
			}

			int number_indx = g_NodeNametoIDMap[origin_node_id];
			int dest_node_index = g_NodeNametoIDMap[destination_node_id];

			DTADestination element;
			element.destination_number = destination_node_id;
			element.record_id = record_id;
			element.destination_node_index = dest_node_index;

			g_NodeVector[number_indx].m_DestinationVector.push_back(element);
			g_NodeVector[number_indx].m_bOriginFlag = true;

			if (line % 10000 == 0)
				cout << g_GetAppRunningTime() << " reading line " << line / 1000 << "k in input_od_pairs.csv." << endl;
			line++;
		}
		fclose(st_input);
	}
	else
	{
		cout << "File input_od_pairs.csv cannot be opened. Please check!" << endl;
		g_ProgramStop();
	}


	unsigned int i;
	int UniqueOriginSize = 0;
	int UniqueDestinationSize = 0;
	int number_of_threads = omp_get_max_threads();

	if (g_ParallelComputingMode == 0)
		number_of_threads = 1;

	for (i = 0; i < g_NodeVector.size(); i++)
	{
		if (g_NodeVector[i].m_bOriginFlag == true)
		{
			UniqueOriginSize += 1;
		}



		if (g_NodeVector[i].m_bDestinationFlag == true)
			UniqueDestinationSize += 1;

	}

	cout << "# of OD pairs = " << line << endl;
	cout << "# of unique origins = " << UniqueOriginSize << " with " << line / UniqueOriginSize << " nodes per origin" << endl;
	cout << "# of processors = " << number_of_threads << endl;

	g_LogFile << "# of OD pairs = " << line << endl;

	g_LogFile << "# of unique origins = " << UniqueOriginSize << " with " << line / UniqueOriginSize << " nodes per origin" << endl;
	g_LogFile << g_GetAppRunningTime() << "# of processors = " << number_of_threads << endl;



#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{
		// create network for shortest path calculation at this processor
		DTANetworkForSP g_TimeDependentNetwork_MP(node_size, link_size, 1, g_AdjLinkSize); //  network instance for single processor in multi-thread environment
		int	cpu_id = omp_get_thread_num();  // starting from 0
		g_TimeDependentNetwork_MP.BuildPhysicalNetwork(0, 0, g_TrafficFlowModelFlag);  // build network for this zone, because different zones have different connectors...

		for (int node_index = 0; node_index < node_size; node_index++)
		{
			if (node_index %number_of_threads == ProcessID && g_NodeVector[node_index].m_bOriginFlag)
			{
				if (node_index % 100 == cpu_id)
				{

					if (node_index % 100 == 0)  // only one CUP can output to log file
					{
						cout << g_GetAppRunningTime() << "processor " << cpu_id << " working on node  " << node_index << ", " << node_index*1.0f / node_size * 100 << "%" << endl;
						g_LogFile << g_GetAppRunningTime() << "processor " << cpu_id << " working on node  " << node_index << ", " << node_index*1.0f / node_size * 100 << "%" << endl;
					}
				}

				// this generate the one-to-all shortest path tree
				g_TimeDependentNetwork_MP.TDLabelCorrecting_DoubleQueue(node_index, 0, 0, 1, DEFAULT_VOT, true, true, false);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo

				for (int dest_no = 0; dest_no < g_NodeVector[node_index].m_DestinationVector.size(); dest_no++)
				{
					int dest_node_index = g_NodeVector[node_index].m_DestinationVector[dest_no].destination_node_index;
					g_NodeVector[node_index].m_DestinationVector[dest_no].destination_node_distance = g_TimeDependentNetwork_MP.LabelCostAry[dest_node_index];
					//					 TRACE("Label: %f: \n",g_NodeVector[node_index].m_DestinationVector[dest_no].destination_node_cost_label);

				}

			}
		}
	}


	FILE* st = NULL;
	fopen_s(&st, "output_shortest_path.txt", "w");
	if (st != NULL)
	{
		fprintf(st, "record_id,from_node_id,to_node_id,distance\n");
		for (int node_index = 0; node_index < node_size; node_index++)
		{
			if (node_index % 100000 == 0)
				cout << g_GetAppRunningTime() << " Computation engine is outputing results for node sequence " << node_index << endl;
			for (int dest_no = 0; dest_no < g_NodeVector[node_index].m_DestinationVector.size(); dest_no++)
			{
				int dest_node_index = g_NodeVector[node_index].m_DestinationVector[dest_no].destination_node_index;
				float label = g_NodeVector[node_index].m_DestinationVector[dest_no].destination_node_distance;

				fprintf(st, "%d,%d,%d,%4.2f\n", g_NodeVector[node_index].m_DestinationVector[dest_no].record_id,
					g_NodeVector[node_index].m_NodeNumber,
					g_NodeVector[node_index].m_DestinationVector[dest_no].destination_number, label);

			}

		}

		fclose(st);
	}
	else
	{
		cout << "File output_shortest_path.csv cannot be opened. Please check!" << endl;
		g_ProgramStop();

	}

	cout << g_GetAppRunningTime() << " Done!" << endl;

}




void g_GenerateSimulationSummary(int iteration, bool NotConverged, int TotalNumOfVehiclesGenerated, NetworkLoadingOutput* p_SimuOutput)
{

	cout << g_GetAppRunningTime() << "GenerateSimulationSummary()";
	
	if (g_AssignmentMOEVector.size() == 0)  // no assignment being involved
		return;

	if (g_EmissionDataOutputFlag == 2 || (g_EmissionDataOutputFlag >= 1 && iteration == g_NumberOfIterations))   //output emission for each iteration 
		g_CalculateEmissionMOE();

	TotalNumOfVehiclesGenerated = p_SimuOutput->NumberofVehiclesGenerated; // need this to compute avg gap

	g_AssignmentMOEVector[iteration] = (*p_SimuOutput);

	if (iteration >= 1) // Note: we output the gap for the last iteration, so "iteration-1"
	{
		//agent based, we record gaps only for vehicles switched (after they find the paths)
		p_SimuOutput->AvgUEGap = g_CurrentGapValue / max(1, g_CurrentNumOfVehiclesForUEGapCalculation);
		p_SimuOutput->AvgRelativeUEGap = g_CurrentRelativeGapValue * 100 / max(1, g_CurrentNumOfVehiclesForUEGapCalculation);
		g_PrevRelativeGapValue = p_SimuOutput->AvgRelativeUEGap;

	}

	float PercentageComplete = 0;

	if (p_SimuOutput->NumberofVehiclesGenerated >= 1)
		PercentageComplete = p_SimuOutput->NumberofVehiclesCompleteTrips*100.0f / max(1, p_SimuOutput->NumberofVehiclesGenerated);

	g_LogFile << g_GetAppRunningTime() << "Iteration: " << iteration << ", Average Trip Time: " << p_SimuOutput->AvgTravelTime << ", Travel Time Index: " << p_SimuOutput->AvgTTI << ", Average Distance: " << p_SimuOutput->AvgDistance << ", Switch %:" << p_SimuOutput->SwitchPercentage << ", Number of Vehicles Complete Their Trips: " << p_SimuOutput->NumberofVehiclesCompleteTrips << ", " << PercentageComplete << "%" << endl;
	cout << g_GetAppRunningTime() << "Iter: " << iteration << ", Avg Trip Time: " << p_SimuOutput->AvgTripTime << ", Avg Trip Time Index: " << p_SimuOutput->AvgTTI << ", Avg Dist: " << p_SimuOutput->AvgDistance << ", Switch %:" << p_SimuOutput->SwitchPercentage << ", # of veh Complete Trips: " << p_SimuOutput->NumberofVehiclesCompleteTrips << ", " << PercentageComplete << "%" << endl;

	g_AssignmentLogFile << g_GetAppRunningTime() << "," << iteration << "," << p_SimuOutput->AvgTravelTime << "," << p_SimuOutput->AvgTTI << "," << p_SimuOutput->AvgDistance << "," << p_SimuOutput->SwitchPercentage << "," << p_SimuOutput->NumberofVehiclesCompleteTrips << "," << PercentageComplete << "%,";

	g_AssignmentLogFile << p_SimuOutput->AvgUEGap << "," << p_SimuOutput->TotalDemandDeviation << "," << p_SimuOutput->LinkVolumeAvgAbsError << "," << /*p_SimuOutput->LinkVolumeRootMeanSquaredError <<*/ "," << p_SimuOutput->LinkVolumeAvgAbsPercentageError;

	g_AssignmentLogFile << endl;

	int time_interval = 15; // min
	int start_time = (int)(g_DemandLoadingStartTimeInMin / time_interval)*time_interval;

	if (iteration == 0)
	{

		g_SummaryStatFile.Reset();
		g_SummaryStatFile.SetFieldName("Iteration #");
		g_SummaryStatFile.SetFieldName("CPU Running Time");
		g_SummaryStatFile.SetFieldName("Per Iteration CPU Running Time");
		g_SummaryStatFile.SetFieldName("# of agents");
		g_SummaryStatFile.SetFieldName("Avg Travel Time (min)");
		g_SummaryStatFile.SetFieldName("Avg Waiting Time at Origin (min)");
		g_SummaryStatFile.SetFieldName("Avg Trip Time Index=(Mean TT/Free-flow TT)");
		g_SummaryStatFile.SetFieldName("Avg Distance");
		g_SummaryStatFile.SetFieldName("Avg Speed");
		g_SummaryStatFile.SetFieldName("Avg CO (g)");

		g_SummaryStatFile.SetFieldName("% considering to switch");
		g_SummaryStatFile.SetFieldName("% switched");
		g_SummaryStatFile.SetFieldName("% completing trips");
		g_SummaryStatFile.SetFieldName("network clearance time (in min)");
		g_SummaryStatFile.SetFieldName("Avg User Equilibirum (UE) gap (min)");
		g_SummaryStatFile.SetFieldName("Relative UE gap (%)");



		//		if(g_ODEstimationFlag == 1)
		//		{
		//			g_SummaryStatFile.SetFieldName ("Demand Deviation");
		g_SummaryStatFile.SetFieldName("ODME: number of data points");

		g_SummaryStatFile.SetFieldName("ODME: Absolute link count error");
		g_SummaryStatFile.SetFieldName("ODME: % link count error");
		g_SummaryStatFile.SetFieldName("ODME: slope =observed/estimated link count");
		g_SummaryStatFile.SetFieldName("ODME: r_squared link count");

		if (g_ObsTravelTimeAvailableFlag)
		{
			g_SummaryStatFile.SetFieldName("ODME: Absolute link speed error");
			g_SummaryStatFile.SetFieldName("ODME: % link speed error");
			g_SummaryStatFile.SetFieldName("ODME: slope =observed/estimated link speed");
			g_SummaryStatFile.SetFieldName("ODME: r_squared link speed");

		}
		if (g_ObsDensityAvailableFlag)
		{
			g_SummaryStatFile.SetFieldName("ODME: Absolute lane density error");
			g_SummaryStatFile.SetFieldName("ODME: % lane density error");
			g_SummaryStatFile.SetFieldName("ODME: slope =observed/estimated lane density");
			g_SummaryStatFile.SetFieldName("ODME: r_squared lane density");
		}

		//		g_SummaryStatFile.SetFieldName ("ODME: avg_simulated_to_avg_obs");
		//		}


		for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
		{

			std::string time_str = "travel_time_" + GetTimeClockString(time);
			g_SummaryStatFile.SetFieldName(time_str);

		}
		for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
		{

			std::string time_str = "Avg UE gap_" + GetTimeClockString(time);
			g_SummaryStatFile.SetFieldName(time_str);

		}

		for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
		{

			std::string time_str = "Relative UE gap_" + GetTimeClockString(time);
			g_SummaryStatFile.SetFieldName(time_str);

		}
		cout << "Avg Gap: " << p_SimuOutput->AvgUEGap;


		cout << endl;

		g_SummaryStatFile.WriteHeader(false, false);

	}

	int day_by_day_MOE_output = g_GetPrivateProfileInt("ABM_integration", "day_by_day_output", 0, g_DTASettingFileName);
	int day_by_day_MOE_output_starting_iteration = g_GetPrivateProfileInt("ABM_integration", "day_by_day_output_starting_iteration", 100, g_DTASettingFileName);

	if (day_by_day_MOE_output > 0 && iteration+1 > day_by_day_MOE_output_starting_iteration )
	{
		//
		std::string skim_file_name;
		int day = iteration + 1;

		cout << "outputing day by day time-dependent MOE ..." << endl;
		g_OutputMOEData(iteration, true);

	}



	int day_no = iteration + 1;

	g_SummaryStatFile.SetValueByFieldName("Iteration #", day_no);  // iteration from 0
	g_SummaryStatFile.SetValueByFieldName("CPU Running Time", g_GetAppRunningTime(false));
	g_SummaryStatFile.SetValueByFieldName("Per Iteration CPU Running Time", g_GetAppRunningTimePerIteration(false));

	g_AppLastIterationStartTime = CTime::GetCurrentTime();

	g_SummaryStatFile.SetValueByFieldName("# of agents", p_SimuOutput->NumberofVehiclesGenerated);
	g_SummaryStatFile.SetValueByFieldName("Avg Travel Time (min)", p_SimuOutput->AvgTravelTime);

	float buffer_waiting_time = max(0, p_SimuOutput->AvgTripTime - p_SimuOutput->AvgTravelTime);
	g_SummaryStatFile.SetValueByFieldName("Avg Waiting Time at Origin (min)", buffer_waiting_time);
	g_SummaryStatFile.SetValueByFieldName("Avg Trip Time Index=(Mean TT/Free-flow TT)", p_SimuOutput->AvgTTI);
	g_SummaryStatFile.SetValueByFieldName("Avg Distance", p_SimuOutput->AvgDistance);

	float avg_speed = p_SimuOutput->AvgDistance / max(0.1, p_SimuOutput->AvgTravelTime) * 60;

	g_SummaryStatFile.SetValueByFieldName("Avg Speed", avg_speed);
	g_SummaryStatFile.SetValueByFieldName("Avg CO (g)", p_SimuOutput->AvgCO);

	
	g_SummaryStatFile.SetValueByFieldName("% switched", p_SimuOutput->SwitchPercentage);
	g_SummaryStatFile.SetValueByFieldName("% considering to switch", p_SimuOutput->ConsideringSwitchPercentage);

	g_SummaryStatFile.SetValueByFieldName("network clearance time (in min)", p_SimuOutput->NetworkClearanceTimeStamp_in_Min);

	g_SummaryStatFile.SetValueByFieldName("% completing trips", PercentageComplete);

	g_PercentageCompleteTrips = PercentageComplete;

	if (IsWithinODMEIteration(iteration))
	{
		p_SimuOutput->AvgUEGap = 0;
		p_SimuOutput->AvgRelativeUEGap = 0;
	
		g_SummaryStatFile.SetValueByFieldName("ODME: number of data points", p_SimuOutput->ODME_result_link_count.data_size);

		g_SummaryStatFile.SetValueByFieldName("ODME: Absolute link count error", p_SimuOutput->LinkVolumeAvgAbsError);
		g_SummaryStatFile.SetValueByFieldName("ODME: % link count error", p_SimuOutput->LinkVolumeAvgAbsPercentageError);
		g_SummaryStatFile.SetValueByFieldName("ODME: slope =observed/estimated link count", p_SimuOutput->ODME_result_link_count.slope);
		g_SummaryStatFile.SetValueByFieldName("ODME: r_squared link count", p_SimuOutput->ODME_result_link_count.rsqr);

		if (g_ObsTravelTimeAvailableFlag)
		{
		
		g_SummaryStatFile.SetValueByFieldName("ODME: Absolute link speed error", p_SimuOutput->LinkSpeedAvgAbsError);
		g_SummaryStatFile.SetValueByFieldName("ODME: % link speed error", p_SimuOutput->LinkSpeedAvgAbsPercentageError);
		g_SummaryStatFile.SetValueByFieldName("ODME: slope =observed/estimated link speed", p_SimuOutput->ODME_result_link_speed.slope);
		g_SummaryStatFile.SetValueByFieldName("ODME: r_squared link speed", p_SimuOutput->ODME_result_link_speed.rsqr);
		}
		if (g_ObsDensityAvailableFlag)
		{
			g_SummaryStatFile.SetValueByFieldName("ODME: slope =observed/estimated lane density", p_SimuOutput->ODME_result_lane_density.slope);
			g_SummaryStatFile.SetValueByFieldName("ODME: r_squared lane density", p_SimuOutput->ODME_result_lane_density.rsqr);
			g_SummaryStatFile.SetValueByFieldName("ODME: Absolute lane density error", p_SimuOutput->ODME_result_lane_density.avg_absolute_error);
			g_SummaryStatFile.SetValueByFieldName("ODME: % lane density error", p_SimuOutput->ODME_result_lane_density.avg_percentage_error);
		}
		//		g_SummaryStatFile.SetValueByFieldName ("ODME: avg_simulated_to_avg_obs",p_SimuOutput->ODME_result_link_count.avg_y_to_x_ratio  );
	}
	if (IsWithinODMEIteration(iteration))
	{
		//ODME gap results
		float AvgUEGap = g_CurrentGapValue / max(1, p_SimuOutput->NumberofVehiclesGenerated);
		//		float AvgRelativeUEGap = g_CurrentRelativeGapValue / max(1, p_SimuOutput->NumberofVehiclesGenerated);
		float AvgRelativeUEGap = 0;
		g_SummaryStatFile.SetValueByFieldName("Avg User Equilibirum (UE) gap (min)", AvgUEGap);
		g_SummaryStatFile.SetValueByFieldName("Relative UE gap (%)", AvgRelativeUEGap);


	}
	else  // simulation gap
	{
		if (iteration >= 1)
		{
			g_SummaryStatFile.SetValueByFieldName("Avg User Equilibirum (UE) gap (min)", p_SimuOutput->AvgUEGap);
			g_SummaryStatFile.SetValueByFieldName("Relative UE gap (%)", p_SimuOutput->AvgRelativeUEGap);
		}

	}

	//-- time dependent MOE 

	for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
	{

		std::string time_str = "travel_time_" + GetTimeClockString(time);

		int time_interval_ = time / 15;
		double travel_time = p_SimuOutput->GetTimeDependentAvgTravelTime(time);

		g_SummaryStatFile.SetValueByFieldName(time_str, travel_time);

	}

	if (iteration >= 1)
	{

		for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
		{

			std::string time_str = "Avg UE gap_" + GetTimeClockString(time);
			double travel_time_gap = p_SimuOutput->GetTimeDependentAvgGap(time);

			g_SummaryStatFile.SetValueByFieldName(time_str, travel_time_gap);

		}

		for (int time = start_time; time < g_DemandLoadingEndTimeInMin; time += time_interval)
		{

			std::string time_str = "Relative UE gap_" + GetTimeClockString(time);

			double travel_relative_time_gap = p_SimuOutput->GetTimeDependentAvgRelativeGapInPercentage(time);

			g_SummaryStatFile.SetValueByFieldName(time_str, travel_relative_time_gap);

		}
	}

	g_SummaryStatFile.WriteRecord();
	g_WriteUserDefinedMOE(g_MultiScenarioSummaryStatFile, iteration + 1);

	////	if(0) //comment out day to day code
	//	{
	//
	//	unsigned li;
	//	for(li = 0; li< g_LinkVector.size(); li++)
	//	{
	//		DTALink* pLink = g_LinkVector[li];
	//
	//		Day2DayLinkMOE element;
	//		element.TotalFlowCount  = pLink->CFlowArrivalCount;
	//		element.AvgTravelTime= pLink->GetTravelTimeByMin(iteration,0, pLink->m_SimulationHorizon,g_TrafficFlowModelFlag);
	//		element.AvgSpeed = pLink->m_Length / max(0.00001,element.AvgTravelTime) *60;  // unit: mph
	//
	//		for(int i = 1; i < MAX_demand_type_SIZE; i++)
	//		{
	//			element.CumulativeArrivalCount_DemandType[i] = pLink->CFlowArrivalCount_DemandType[i];
	//		}
	//
	//		element.m_NumberOfCrashes =  pLink->m_NumberOfCrashes;
	//		element.m_NumberOfFatalAndInjuryCrashes = pLink->m_NumberOfFatalAndInjuryCrashes;
	//		element.m_NumberOfPDOCrashes = pLink->m_NumberOfPDOCrashes;
	//
	//		pLink->m_Day2DayLinkMOEVector .push_back (element);
	//	}
	//
	//	}

	//if(g_ODEstimationFlag == 1)
	//	cout << "Avg Gap: " << p_SimuOutput->AvgUEGap   << ", Demand Dev:"	<< p_SimuOutput->TotalDemandDeviation << ", Avg volume error: " << p_SimuOutput->LinkVolumeAvgAbsError << ", Avg % error: " << p_SimuOutput->LinkVolumeAvgAbsPercentageError << endl;

	p_SimuOutput->ResetStatistics();

	// with or without inner loop 
	// check outer loop convergence (without inner loop)
		if (g_UEAssignmentMethod <= 1) // MSA and day-to-day learning
		{
			if (g_CurrentNumOfVehiclesSwitched < g_ConvergenceThreshold_in_Num_Switch)
				NotConverged = false; // converged!
		}
		else // gap-based approaches
		{
			//if(g_RelativeGap < g_ConvergencyRelativeGapThreshold_in_perc && !g_ODEstimationFlag )
			//	NotConverged = false; // converged! 
		}
}


void g_ZoneBasedDynamicTrafficAssignmentSimulation()
{


	int node_size = g_NodeVector.size() + 1 + g_ODZoneIDSize;

	int connector_count = 0;

	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		connector_count += (iterZone->second.m_OriginActivityVector.size() + iterZone->second.m_DestinationActivityVector.size());  // only this origin zone has vehicles, then we build the network
	}

	int link_size = g_LinkVector.size() + connector_count; // maximal number of links including connectors assuming all the nodes are destinations

	g_LogFile << "Number of iterations = " << g_NumberOfIterations << endl;

	// Jason
	int iteration = 0;
	bool NotConverged = true;
	int TotalNumOfVehiclesGenerated = 0;
	int number_of_threads = g_number_of_CPU_threads();



	cout << "# of Computer Processors = " << number_of_threads << endl;

	if (g_ODEstimationFlag)
	{

		fprintf(g_ODME_result_file, "Iteration no, Alpha,");

		for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
		{
			if (g_ZoneMap.find(z) == g_ZoneMap.end())
				continue;
			fprintf(g_ODME_result_file,  "zone %d,", z);

		}
		fprintf(g_ODME_result_file, "\n,,");

		for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
		{
			if (g_ZoneMap.find(z) == g_ZoneMap.end())
				continue;

					int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;

				fprintf(g_ODME_result_file, "%.3f,", g_SystemDemand.m_alpha[zone_index]);

		}
		fprintf(g_ODME_result_file, "\n");


	}
		//	int Current_AggregationTimetInterval = g_AggregationTimetInterval;
	// ----------* start of outer loop *----------
	for (iteration = 0; NotConverged && iteration <= g_NumberOfIterations; iteration++)  // we exit from the loop under two conditions (1) converged, (2) reach maximum number of iterations
	{


		cout << "------- Iteration = " << iteration << "--------" << endl;

		if (IsWithinODMEIteration(iteration))  // perform path flow adjustment after at least 10 normal OD estimation
		{
			g_SystemDemand.StoreDemandParametersFromLastIteration();
		}

		// initialize for each iteration
		g_CurrentGapValue = 0.0;
		g_CurrentRelativeGapValue = 0.0;
		g_CurrentNumOfVehiclesForUEGapCalculation = 0;
		g_CurrentNumOfVehiclesSwitched = 0;
		g_NewPathWithSwitchedVehicles = 0;

		// initialize for OD estimation
		g_TotalDemandDeviation = 0;
		g_TotalMeasurementDeviation = 0;
		int Actual_ODZoneSize = g_ZoneMap.size();  // Actual_ODZoneSize can be < ODZoneSize after subarea cut with new zones


		if (!(g_VehicleLoadingMode == vehicle_binary_file_mode && iteration == 0))  // we do not need to generate initial paths for vehicles for the first iteration of vehicle loading mode
		{
			g_EstimationLogFile << "----- Iteration = " << iteration << " ------" << endl;

			if (IsWithinODMEIteration(iteration))
				g_SystemDemand.ResetUpdatedValue(); // reset update hist table

				g_SetupTDTollValue(iteration);

#pragma omp parallel for
			for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
			{
				// create network for shortest path calculation at this processor
				int	id = omp_get_thread_num();  // starting from 0

				cout << "Processor " << id << " is working on shortest path calculation..  " << endl;
				//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

				for (int CurZoneID = 1; CurZoneID <= g_ODZoneNumberSize; CurZoneID++)
				{
					if (g_ZoneMap.find(CurZoneID) == g_ZoneMap.end())  // no such zone being defined
						continue;

					if (g_ProgramStopFlag == 1)
						g_ProgramStop();



					if ((CurZoneID%number_of_threads) == ProcessID)  // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 
					{
						if (g_ZoneMap[CurZoneID].m_OriginVehicleSize > 0)  // only this origin zone has vehicles, then we build the network
						{
							// create network for shortest path calculation at this processor
						//	cout << "Processor " << id << " starts building network for zone centriod for zone " << CurZoneID << endl;

								g_TimeDependentNetwork_MP[ProcessID].BuildNetworkBasedOnZoneCentriod(iteration, CurZoneID);  // build network for this zone, because different zones have different connectors...
						//		cout << "Processor " << id << " end building network for zone centriod for zone " << CurZoneID << endl;

							if (Actual_ODZoneSize > 300)  // only for large networks
							{
								cout << "Processor " << id << " is calculating the zone based shortest paths for zone " << CurZoneID << endl;
							}



							// scan all possible departure times
							for (int departure_time_index = 0; departure_time_index < g_NumberOfSPCalculationPeriods; departure_time_index ++)
							{

								int departure_time = g_AssignmentIntervalStartTimeInMin[departure_time_index];
								int  departure_end_time = g_AssignmentIntervalEndTimeInMin[departure_time_index];


								if (g_TDOVehicleArray[g_ZoneMap[CurZoneID].m_ZoneSequentialNo][departure_time_index].VehicleArray.size() > 0)
								{

									bool debug_flag = false;

									if (IsWithinODMEIteration(iteration))  // perform path flow adjustment after at least 10 normal OD estimation
										g_TimeDependentNetwork_MP[ProcessID].ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval_ODEstimation(CurZoneID, departure_time_index, iteration);
									else
									{
										for (int demand_type = 1; demand_type <= g_DemandTypeVector.size(); demand_type++)  // from SOV, HOV, truck
										{
											if (g_SimulationResult.number_of_vehicles_DemandType[demand_type] >= 1)
											{
												g_TimeDependentNetwork_MP[ProcessID].TDLabelCorrecting_DoubleQueue_PerDemandType(CurZoneID, g_NodeVector.size(), departure_time, demand_type, g_DemandTypeVector[demand_type-1].Avg_VOT, false, debug_flag);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo
											}
										}

										g_TimeDependentNetwork_MP[ProcessID].ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval(CurZoneID, departure_time, departure_end_time, iteration, debug_flag);
										}

								}
							} // for each departure time

						} // for zone with volume
					} // for zone id assigned to the processor id
				}	// for each zone
			}  // for each processor
			//the OD estimation code below should be single thread

			if (IsWithinODMEIteration(iteration))  // re-generate vehicles based on global path set
			{

				_proxy_ODME_log(0, iteration, "--------------------------\n");

				for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
				{
					if (g_ZoneMap.find(z) == g_ZoneMap.end())
						continue;

					int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;
					_proxy_ODME_log(0, iteration, "alpha[%d]= %f\n", z, g_SystemDemand.m_alpha[zone_index]);

				}
				fprintf(g_ODME_result_file, "%d,,", iteration);

				for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
				{
					if (g_ZoneMap.find(z) == g_ZoneMap.end())
						continue;

					int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;

					fprintf(g_ODME_result_file, "%.3f,", g_SystemDemand.m_alpha[zone_index]);

				}
				fprintf(g_ODME_result_file, "\n");
				fflush(g_ODME_result_file);

				g_GenerateVehicleData_ODEstimation();
			}

			cout << "---- Network Loading for Iteration " << iteration+1 << "----" << endl;

			//	 DTANetworkForSP network(node_size, link_size, g_DemandLoadingHorizon);  // network instance for single-thread application

			NetworkLoadingOutput SimuOutput = g_NetworkLoading(g_TrafficFlowModelFlag, 0, iteration);
			g_GenerateSimulationSummary(iteration, NotConverged, TotalNumOfVehiclesGenerated, &SimuOutput);

		}	// end of outer loop


	} // for each assignment iteration

	if (g_ODEstimationFlag)  // re-generate vehicles based on global path set
	{
		FILE *final_ODME_result_file = fopen("ODME_final_result.csv", "w");

		if (final_ODME_result_file != NULL)
		{

			fprintf(final_ODME_result_file, "zone_id,ODME_ratio\n");

			for (int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
			{
				if (g_ZoneMap.find(z) == g_ZoneMap.end())
					continue;

				int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;

				float ODME_ratio = g_SystemDemand.m_alpha[zone_index] / max(1, g_SystemDemand.m_alpha_hist_value[zone_index]);
				if (g_SystemDemand.m_alpha_hist_value[zone_index] < 0.1)
					ODME_ratio = 1.0;

				fprintf(final_ODME_result_file, "%d,%.4f\n", z, ODME_ratio);

			}

			fclose(final_ODME_result_file);
		}
	}
	cout << "Writing Vehicle Trajectory and MOE File... " << endl;

	if (iteration == g_NumberOfIterations)
	{

		iteration = g_NumberOfIterations - 1;  //roll back to the last iteration if the ending condition is triggered by "iteration < g_NumberOfIterations"
	}

	g_OutputMOEData(iteration);

}

