//  Portions Copyright 2014 Xuesong Zhou; Lagrangian-relaxation based, agent-based time-depenedent network optimization 
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

int g_NetworkDesignOptimalLinkSize = 3;
float g_NetworkDesignTravelTimeBudget = 12;

float g_NetworkDesignGlobalUpperBound = 9999999;
float g_NetworkDesignGlobalLowerBound = -9999990;

// create this toll vector
std::vector<VehicleLinkPrice> g_NetworkDesignRoadConstructionVector;


float g_CalculateUpperBoundValue()
{
	int UpperBoundObjectiveFunctionValue = 0;
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{

		DTAVehicle* pVehicle = iterVM->second;

		if (pVehicle->m_bMeetTarget==false)
			UpperBoundObjectiveFunctionValue++;
	}

	return UpperBoundObjectiveFunctionValue;

}
void g_AgentBasedOptimization()  // this is an adaptation of OD trip based assignment, we now generate and assign path for each individual vehicle (as an agent with personalized value of time, value of reliability)
{

	// reset random number seeds
	int node_size = g_NodeVector.size() + 1 + g_ODZoneIDSize;
	int link_size = g_LinkVector.size() + g_NodeVector.size(); // maximal number of links including connectors assuming all the nodes are destinations

	cout << ":: start agent-based optimization " << g_GetAppRunningTime() << endl;
	g_LogFile << ":: start agent-based optimization " << g_GetAppRunningTime() << endl;
	g_NetworkDesignLogFile << ":: start agent-based optimization " << g_GetAppRunningTime() << endl;


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
	//	if(g_ODEstimationFlag==1)  // single thread mode for ODME 
	//		number_of_threads = 1;

	cout << "# of Computer Processors = " << number_of_threads << endl;


	number_of_threads = g_number_of_CPU_threads();

#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{

		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0
		g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(iteration, -1, g_TrafficFlowModelFlag);  // build network for this zone, because different zones have different connectors...
		g_TimeDependentNetwork_MP[id].InitializeTDLinkCost();
	}

	int find_best_upper_bound = g_GetPrivateProfileInt("network_design", "find_best_upper_bound", 0, g_DTASettingFileName);
	if (find_best_upper_bound ==1)
	{
	cout << "Finding best Upper Bound Solutions... " << endl;
	g_NetworkDesignEnemerateAllSolutions();
	}



	// ----------* start of outer loop *----------
	for (iteration = 0; NotConverged && iteration <= g_NumberOfIterations; iteration++)  // we exit from the loop under two conditions (1) converged, (2) reach maximum number of iterations
	{
		cout << "------- Iteration = " << iteration + 1 << "--------" << endl;
		g_NetworkDesignLogFile << "Iteration =," << iteration + 1;
		
		g_CurrentGapValue = 0.0;
		g_CurrentRelativeGapValue = 0.0;
		g_CurrentNumOfVehiclesForUEGapCalculation = 0;
		g_CurrentNumOfVehiclesSwitched = 0;
		g_NewPathWithSwitchedVehicles = 0;

		//subgraident 
		//sub problem 1: time-dependent least path finding for each agent
		g_OptimizePathsForAgents(iteration, true, true, g_DemandLoadingStartTimeInMin, g_DemandLoadingEndTimeInMin);

		//subproblem 2: knapsack problem for selecting links to be constructed
		g_NetworkDesignKnapsackProblem(iteration, true, true, g_DemandLoadingStartTimeInMin, g_DemandLoadingEndTimeInMin);

		////check upper_bound_solution: time-dependent least path finding for each agent with links to be built
	
		g_GenerateUpperBoundFeasibleSolutionForAgents(g_DemandLoadingStartTimeInMin, g_DemandLoadingEndTimeInMin);

		g_NetworkDesignLogFile << endl;
	}  // for each assignment iteration
	g_NetworkDesignLogFile << "--Final Solution--" << endl;
	g_NetworkDesignLogFile << "Link, Build Flag" << endl;

	for (int li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];
		if (pLink->m_NetworkDesignFlag >= 1)
		{ 
			CString str;
			if (pLink->m_NetworkDesignBuildCapacity >0.99)
				str.Format("%d -> %d,1\n", pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);
			else 
				str.Format("%d -> %d,0\n", pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);

			g_NetworkDesignLogFile << str;
		}
	}


	float  UpperBoundObjectiveFunctionValue = g_CalculateUpperBoundValue();
	g_NetworkDesignLogFile << endl;
	g_NetworkDesignLogFile << "Upper Bound Value = " << UpperBoundObjectiveFunctionValue << endl;



}

void g_NetworkDesignTestAllCombination()
{


}

void 	DTANetworkForSP::InitializeTDLinkCost()
{
	unsigned li;
	for (li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		int ti;
		//initialization
		for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
		{
			TD_LinkCostAry[pLink->m_LinkNo][ti] = 0.0;
		}
	}
}



void 	DTANetworkForSP::ResourcePricing_Subgraident(int iteration)
{
	// 

	float stepsize = 0.5;

	if (iteration < 20)   //MSA
		stepsize = 1 / (iteration + 1);
	else
		stepsize = 0.05;

	unsigned li;
	for (li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		int ti;
		//initialization
		for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
		{
			TD_LinkCostAry[pLink->m_LinkNo][ti] = 0.0;

			if (iteration == 0)
			{
				TD_LinkVolumeAry[pLink->m_LinkNo][ti] = 0.0;
			}
		}


		float subgradient = 0;

		// determien the price
		if (pLink->m_NetworkDesignFlag >= 1)
		{
			for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
			{

				subgradient = (TD_LinkVolumeAry[pLink->m_LinkNo][ti] - pLink->m_NetworkDesignBuildCapacity);

				TD_LinkCostAry[pLink->m_LinkNo][ti] += subgradient * stepsize;

				if (TD_LinkCostAry[pLink->m_LinkNo][ti] < 0)   // resource price must be positive or zero
					TD_LinkCostAry[pLink->m_LinkNo][ti] = 0;

			}

		}

		//initialization
		for (int ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
		{
			TD_LinkVolumeAry[pLink->m_LinkNo][ti] = 0.0;
		}

	}
}

void g_NetworkDesignKnapsackProblem(int iteration, bool bRebuildNetwork, bool bOutputLog, int DemandLoadingStartTime, int DemandLoadingEndTime)
{
#ifdef _large_memory_usage_lr

	// create this toll vector
	std::vector<VehicleLinkPrice> Global_RoadPriceVector;

	unsigned li;
	for (li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];
		if (pLink->m_NetworkDesignFlag >= 1)  // links to be built
		{
			VehicleLinkPrice element;
			element.LinkNo = pLink->m_LinkNo;
			element.RoadPrice = 0;
			element.RoadUsageFlag = 0;
			element.TotalTollColected = 0;

			Global_RoadPriceVector.push_back(element);

		}
	}
	

	float total_road_traveling_cost = 0;

	// collect the toll from the memory of all vehicles

	int UpperBoundObjectiveFunctionValue = 0;
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{

		DTAVehicle* pVehicle = iterVM->second;

		if (pVehicle->m_bMeetTarget==false)
		{
			UpperBoundObjectiveFunctionValue++;
			continue;
		}
		total_road_traveling_cost += pVehicle->m_MinCost;

		if (pVehicle->m_NodeSize >= 2)  // with physical path in the network
		{ 


			for (li = 0; li < pVehicle->m_PersonalizedRoadPriceVector.size(); li++)
			{
				if (pVehicle->m_PersonalizedRoadPriceVector[li].RoadPrice >0.01)
					TRACE("");
				Global_RoadPriceVector[li].RoadPrice  += pVehicle->m_PersonalizedRoadPriceVector[li].RoadPrice;

				Global_RoadPriceVector[li].TotalTollColected += pVehicle->m_PersonalizedRoadPriceVector[li].RoadPrice;
				Global_RoadPriceVector[li].RoadUsageFlag += pVehicle->m_PersonalizedRoadPriceVector[li].RoadUsageFlag;


			}

		}

	}



	std::sort(Global_RoadPriceVector.begin(), Global_RoadPriceVector.end());  //knapsack sorting 
	
	CString str;
	float dual_resource_price = 0;

	for (li = 0; li < Global_RoadPriceVector.size(); li++)
	{
		int LinkNo = Global_RoadPriceVector[li].LinkNo;
		DTALink* pLink = g_LinkVector[LinkNo];

		if (pLink->m_NetworkDesignFlag >= 1)  // links to be built
		{
			pLink->m_LROptimizationLinkPrice.RoadPrice = Global_RoadPriceVector[li].RoadPrice;
			pLink->m_LROptimizationLinkPrice.RoadUsageFlag = Global_RoadPriceVector[li].RoadUsageFlag;
			pLink->m_LROptimizationLinkPrice.TotalTollColected = Global_RoadPriceVector[li].TotalTollColected;

		}
	

	}
	for (li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];
		if (pLink->m_NetworkDesignFlag >= 1)  // links to be built
		{
			str.Format(",Link %d -> %d,LR Price =,%f,Volume =,%d,", 
				pLink->m_FromNodeNumber, pLink->m_ToNodeNumber, 
				pLink->m_LROptimizationLinkPrice.RoadPrice, 
				pLink->m_LROptimizationLinkPrice.RoadUsageFlag);
			g_NetworkDesignLogFile << str;
		}
	}
	int K = g_NetworkDesignOptimalLinkSize;
	for (li = 0; li < Global_RoadPriceVector.size(); li++)
	{
	
		int LinkNo = Global_RoadPriceVector[li].LinkNo;
		DTALink* pLink = g_LinkVector[LinkNo];

		TRACE("\nLink %d -> %d: Price = %f ", pLink->m_FromNodeNumber, pLink->m_ToNodeNumber, Global_RoadPriceVector[li].RoadPrice );
		str.Format(",Rank No.%d, Link %d -> %d,Total LR Price =,%f,", li+1, 
			pLink->m_FromNodeNumber, pLink->m_ToNodeNumber, 
			Global_RoadPriceVector[li].TotalTollColected);


		g_NetworkDesignLogFile << str;

		if (li < K)
		{
			pLink->m_NetworkDesignBuildCapacity = 1;
			dual_resource_price += Global_RoadPriceVector[li].RoadPrice;
		}
		else
		{
			pLink->m_NetworkDesignBuildCapacity = 0;
		}

	}


	g_NetworkDesignLogFile << "Solution = ,";

	for (li = 0; li < Global_RoadPriceVector.size(); li++)
	{

		int LinkNo = Global_RoadPriceVector[li].LinkNo;
		DTALink* pLink = g_LinkVector[LinkNo];

		if (pLink->m_NetworkDesignBuildCapacity >= 0.99)
		{
			str.Format(",Build Link, %d -> %d,", pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);
			g_NetworkDesignLogFile << str;
		}
	}





	float LR_relaxed_objective_function_value = total_road_traveling_cost - dual_resource_price;


	if (LR_relaxed_objective_function_value > g_NetworkDesignGlobalLowerBound)
		g_NetworkDesignGlobalLowerBound = LR_relaxed_objective_function_value;
	
	if (UpperBoundObjectiveFunctionValue < g_NetworkDesignGlobalUpperBound)
		g_NetworkDesignGlobalUpperBound = UpperBoundObjectiveFunctionValue;

	float gap_percentage = (g_NetworkDesignGlobalUpperBound - g_NetworkDesignGlobalLowerBound) * 100 / max(0.001,g_NetworkDesignGlobalUpperBound);

	g_NetworkDesignLogFile << "," << g_GetAppRunningTime() << ",total_road_traveling_cost_cx_pie_x=, " << total_road_traveling_cost << ", dual_resource_price_pie_y, " << dual_resource_price << ", LR_relaxed_objective_function_value, " << g_NetworkDesignGlobalLowerBound << ", UpperBound = , " << g_NetworkDesignGlobalUpperBound << ", relaitve gap = , " << gap_percentage << ", ";
	cout << "\niteration," << iteration << ",total_road_traveling_cost_cx_pie_x," << total_road_traveling_cost << ",dual_resource_price_pie_y," << dual_resource_price << ",LR_relaxed_objective_function_value," << LR_relaxed_objective_function_value << "," << endl;

	g_NetworkDesignLogFile << ",# of Links to be build=," << g_NetworkDesignOptimalLinkSize << ",";

#endif 
}

void g_OptimizePathsForAgents(int iteration, bool bRebuildNetwork, bool bOutputLog, int DemandLoadingStartTime, int DemandLoadingEndTime)
{
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
				for (int departure_time_index = 0; departure_time_index < g_NumberOfSPCalculationPeriods; departure_time_index++)
				{

					int departure_time = g_AssignmentIntervalStartTimeInMin[departure_time_index];
					int  departure_end_time = g_AssignmentIntervalEndTimeInMin[departure_time_index];
					int vehicle_size = g_TDOVehicleArray[g_ZoneMap[CurZoneID].m_ZoneSequentialNo][departure_time_index].VehicleArray.size();
					if (vehicle_size > 0)
					{

						int node_size = g_TimeDependentNetwork_MP[id].AgentBasedPathOptimization(CurZoneID, departure_time, departure_end_time, iteration);
							cout << "Processor " << id << " is calculating accessibility for zone " << CurZoneID << endl;
					}
				}  // for each departure time
			}
		}  // for each zone
	} // for each computer processor

	if (bOutputLog)
	{
		cout << ":: complete optimization " << g_GetAppRunningTime() << endl;
		g_LogFile << ":: complete optimization " << g_GetAppRunningTime() << endl;
		bool bStartWithEmptyFile = true;
		cout << "     outputing output_agent.csv... " << endl;

		bool Day2DayOutputFlag = false; 
		int iteration = 0;
		OutputVehicleTrajectoryData(g_CreateFileName("output_agent", Day2DayOutputFlag, iteration+1, true),
			g_CreateFileName("output_trip", Day2DayOutputFlag, iteration,true), iteration, true, true);

	}
}

void g_GenerateUpperBoundFeasibleSolutionForAgents(int DemandLoadingStartTime, int DemandLoadingEndTime)
{
	// assign different zones to different processors
	int number_of_threads = omp_get_max_threads();

	number_of_threads = g_number_of_CPU_threads();

#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{
		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0

		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors


		for (int CurZoneID = 1; CurZoneID <= g_ODZoneNumberSize; CurZoneID++)
		{

			if (g_ZoneMap.find(CurZoneID) == g_ZoneMap.end())  // no such zone being defined
				continue;

			if ((CurZoneID%number_of_threads) == ProcessID)  // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 
			{

				// scan all possible departure times
				for (int departure_time_index = 0; departure_time_index < g_NumberOfSPCalculationPeriods; departure_time_index++)
				{

					int departure_time = g_AssignmentIntervalStartTimeInMin[departure_time_index];
					int  departure_end_time = g_AssignmentIntervalEndTimeInMin[departure_time_index];
					int vehicle_size = g_TDOVehicleArray[g_ZoneMap[CurZoneID].m_ZoneSequentialNo][departure_time_index].VehicleArray.size();
					if (vehicle_size > 0)
					{

						int node_size = g_TimeDependentNetwork_MP[id].AgentBasedUpperBoundSolutionGeneration(CurZoneID, departure_time, departure_end_time, 0);
						cout << "Processor " << id << " is calculating feasible solution for zone " << CurZoneID << endl;
					}
				}  // for each departure time
			}
		}  // for each zone
	} // for each computer processor

		cout << ":: complete optimization " << g_GetAppRunningTime() << endl;
		g_LogFile << ":: complete optimization " << g_GetAppRunningTime() << endl;

}


float DTANetworkForSP::AgentBasedPathOptimization(int zone, int departure_time_begin, int departure_time_end, int iteration)
// for vehicles starting from departure_time_begin to departure_time_end, assign them to shortest path using a proportion according to MSA or graident-based algorithms
{
	int total_node_size = 0;
	
	int VehicleSize =  0;
#ifdef _large_memory_usage_lr
	//	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH];
	int TempPathLinkList[MAX_NODE_SIZE_IN_A_PATH];

	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;

	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);

	

	// loop through the TDOVehicleArray to assign or update vehicle paths...

	float const_stepsize = max(0.1,g_GetPrivateProfileFloat("network_design", "constant_stepsize", 0.5, g_DTASettingFileName));
	float stepsize = 0.5;

	if (iteration < 20)
		stepsize = 1.0 / (iteration + 1) * const_stepsize;
	else
		stepsize = 0.05 * const_stepsize;


	VehicleSize = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size();
	for (int vi = 0; vi < VehicleSize; vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		DTAVehicle* pVeh = g_VehicleMap[VehicleID];

		pVeh->m_bConsiderToSwitch = false;
		pVeh->m_bSwitch = false;


		/// finding optimal path 
		bool bDebugFlag = false;

		float TotalCost;
		bool bGeneralizedCostFlag = false;

		// general settings

		int final_departuret_time_shift = 0;  // unit: min
		TempNodeSize = 0;
		float switch_threshold = 0;   // 2 min for day to day learning, - 100 min for  special case when using MSA, so 15% of agents will be selected for swtiching for sure. 


		if (pVeh->m_OriginZoneID == pVeh->m_DestinationZoneID)
		{  // do not simulate intra zone traffic
			continue;
		}

		if (pVeh->m_AgentID == 1001)
			TRACE("");

		unsigned li;

		// iteration 0: Load initial road price vector
		if (iteration == 0)
		{
			for (li = 0; li < g_LinkVector.size(); li++)
			{
				DTALink* pLink = g_LinkVector[li];
				if (pLink->m_NetworkDesignFlag ==1)
				{
					VehicleLinkPrice element;
					element.LinkNo = pLink->m_LinkNo;
					element.RoadPrice = 0;
					element.RoadUsageFlag = 0; 

					pVeh->m_PersonalizedRoadPriceVector.push_back(element);

				}
			}
		}



		// we perform subgradient for each vehicle

		float subgradient = 0;
		for (li = 0; li < pVeh->m_PersonalizedRoadPriceVector.size(); li++)
		{

			DTALink* pLink = g_LinkVector[pVeh->m_PersonalizedRoadPriceVector[li].LinkNo];


			subgradient = pVeh->m_PersonalizedRoadPriceVector[li].RoadUsageFlag - pLink->m_NetworkDesignBuildCapacity;

			pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice += subgradient * stepsize;

			if (pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice >=0.01)
			{
				TRACE("");
			
			}

			if (pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice < 0)
				pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice = 0;


		}


		// copy road price for links to be built
		int ti;
		for (li = 0; li < pVeh->m_PersonalizedRoadPriceVector.size(); li++)
		{
			int LinkNo = pVeh->m_PersonalizedRoadPriceVector[li].LinkNo;


				for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
				{
					if (pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice > 0.0001)
					{
						TD_LinkCostAry[LinkNo][ti] = pVeh->m_PersonalizedRoadPriceVector[li].RoadPrice;
					}
					else
					{
						TD_LinkCostAry[LinkNo][ti] = 0;

					
					}
				}
			


		}

		// we find shortet path for each agent



		for (li = 0; li < pVeh->m_PersonalizedRoadPriceVector.size(); li++)
		{
			pVeh->m_PersonalizedRoadPriceVector[li].RoadUsageFlag = 0;

		}


		if (pVeh->m_NodeSize >=1 && pVeh->m_LinkAry != NULL)  // delete the old path
		{
			delete pVeh->m_LinkAry;
		}
		pVeh->m_NodeSize = 0;


		float TargeTravelTimeInMin = g_NetworkDesignTravelTimeBudget;
		float OptimialTravelTimeInMin = 0;

		NodeSize = FindOptimalNodePath_TDLabelCorrecting_DQ(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, pVeh->m_DepartureTime,
			pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, TargeTravelTimeInMin, OptimialTravelTimeInMin, bDebugFlag);

		float TargeTravelTimeInMin_global = TargeTravelTimeInMin;
		if (NodeSize >= 2 && OptimialTravelTimeInMin <= TargeTravelTimeInMin_global && TotalCost <= 1)   // a feasible path that meets the travel time target with lower road price of pie than 1
		{
		
			pVeh->SetMinCost(TotalCost);

			pVeh->m_NodeSize = NodeSize;

			pVeh->m_LinkAry = new SVehicleLink[NodeSize];

			if (pVeh->m_LinkAry == NULL)
			{
				cout << "Insufficient memory for allocating vehicle arrays!";
				g_ProgramStop();
			}
				int NodeNumberSum = 0;
				float Distance = 0;

				// scan the path
				for (int i = 0; i < NodeSize - 1; i++)
				{
					pVeh->m_LinkAry[i].LinkNo = PathLinkList[i];
					NodeNumberSum += PathLinkList[i];

					TRACE("\nlink %d ->%d", g_LinkVector[PathLinkList[i]]->m_FromNodeNumber, g_LinkVector[PathLinkList[i]]->m_ToNodeNumber);
		
					Distance += g_LinkVector[pVeh->m_LinkAry[i].LinkNo]->m_Length;

					if (g_LinkVector[PathLinkList[i]]->m_NetworkDesignFlag >= 1)  // candidate link
					{
						int NetworkDesignSequenceNo = g_LinkVector[PathLinkList[i]]->m_NetworkDesignSequenceNo;

						if (NetworkDesignSequenceNo >= 0 && NetworkDesignSequenceNo < pVeh->m_PersonalizedRoadPriceVector.size())
						{
						
						pVeh->m_PersonalizedRoadPriceVector[NetworkDesignSequenceNo].RoadUsageFlag = 1;
						}
					}


				}

				pVeh->m_gap_update = true;
				pVeh->m_gap = 1 - TotalCost;

				pVeh->m_Distance = Distance;
				pVeh->m_NodeNumberSum = NodeNumberSum;

		}
		else
		{
			pVeh->SetMinCost(1);  // use the alternative virtual traveling arc

		}

		} // for each vehicle on this OD pair
#endif 
	return total_node_size / max(1, VehicleSize);

}

int DTANetworkForSP::FindOptimalNodePath_TDLabelCorrecting_DQ(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, float TargetTravelTime, float &OptimialTravelTimeInMin, bool bDebugFlag)
// time -dependent label correcting algorithm with deque implementati
{

#ifndef _large_memory_usage_lr
	return 0;
#endif
	int BeginOfTDSPCalculationInterval = max(0, (departure_time - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin);
	int EndOfTDSPCalculationInterval = min(m_NumberOfTDSPCalculationIntervals, (departure_time + TargetTravelTime+1 - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin );
	int i;
	int debug_flag = 0;  // set 1 to debug the detail information
	if (debug_flag)
		TRACE("\nCompute shortest path from %d at time %d", origin, departure_time);

	bool bFeasiblePathFlag = false;

	if (m_OutboundSizeAry[origin] == 0)
		return false;

	for (i = 0; i <m_NodeSize; i++) // Initialization for all nodes
	{
		NodeStatusAry[i] = 0;

		for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
		{
			TD_LabelCostAry[i][t] = MAX_SPLABEL;
			TD_NodePredAry[i][t] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
			TD_TimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
		}

	}


	int l;

	for (l = 0; l <m_LinkSize; l++) // Initialization for all nodes
	{
		for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
		{

			int aggregated_time_interval = g_FindAssignmentIntervalIndexFromTime(m_StartTimeInMin + t / g_TDSPTimetIntervalSizeForMin);
			TD_LinkTimeIntervalAry[l][t] = max(1, (int)(m_LinkTDTimeAry[l][aggregated_time_interval] * g_TDSPTimetIntervalSizeForMin+0.5));

		
		}

	}

	// Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node
	TD_LabelCostAry[origin][BeginOfTDSPCalculationInterval] = 0;

	SEList_clear();
	SEList_push_front(origin);


	while (!SEList_empty())
	{
		int FromID = SEList_front();
		SEList_pop_front();  // remove current node FromID from the SE list


		NodeStatusAry[FromID] = 2;        //scaned

		//scan all outbound nodes of the current node
		for (i = 0; i<m_OutboundSizeAry[FromID]; i++)  // for each arc (i,j) belong A(i)
		{
			int LinkNo = m_OutboundLinkAry[FromID][i];
			int ToID = m_OutboundNodeAry[FromID][i];

			if (ToID == origin)  // remove possible loop back to the origin
				continue;

			int ToNodeNumber = g_NodeVector[ToID].m_NodeNumber;

			int ToNodeInScanListFlag = 0;

			if (m_LinkConnectorFlag[LinkNo] == 1)  // only check the following speical condition when a link is a connector
			{
				int OriginTAZ = m_OutboundConnectorOriginZoneIDAry[FromID][i];
				int DestinationTAZ = m_OutboundConnectorDestinationZoneIDAry[FromID][i];

				if (OriginTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ <= 0 && OriginTAZ != origin_zone)
					continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

				if (DestinationTAZ >= 1 /* TAZ >=1*/ && OriginTAZ <= 0 && DestinationTAZ != destination_zone)
					continue;  // special feature 2: skip connectors with destination TAZ that do not belong to this destination zone

				if (OriginTAZ >= 1 /* TAZ >=1*/ && OriginTAZ != origin_zone  && DestinationTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ != destination_zone)
					continue;  // special feature 3: skip connectors (with both TAZ at two ends) that do not belong to the origin/destination zones

				if (ToID == origin) // special feature 2: no detour at origin
					continue;
			}

			if (debug_flag)
				TRACE("\nScan from node %d to node %d", g_NodeVector[FromID].m_NodeNumber, g_NodeVector[ToID].m_NodeNumber);



			// for each time step, starting from the departure time
			for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
			{
				if (TD_LabelCostAry[FromID][t]<MAX_SPLABEL - 1)  // for feasible time-space point only
				{
			
					int time_stopped = 0;

					int NewToNodeArrivalTimeInterval = t + time_stopped + TD_LinkTimeIntervalAry[LinkNo][t];  // time-dependent travel times for different train type

					if (NewToNodeArrivalTimeInterval == t)
						NewToNodeArrivalTimeInterval = t + 1;

					NewToNodeArrivalTimeInterval = GetFeasibleTDSPTimeInterval(NewToNodeArrivalTimeInterval);
					int		link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(t);




					float NewCost = TD_LabelCostAry[FromID][t] + TD_LinkCostAry[LinkNo][t];
					// costs come from time-dependent resource price or road toll

					if (g_LinkTDCostAry[LinkNo][link_entering_time_interval].m_bMonetaryTollExist)
					{ // with VOT and toll
						float AdditionalCostInMin = g_LinkTDCostAry[LinkNo][link_entering_time_interval].TollValue[demand_type] / VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions
						NewCost += AdditionalCostInMin;
					}
					if (debug_flag && TD_LinkCostAry[LinkNo][t] > 0.0001)
					{
						TRACE("\n         Pay Price  %f, link cost %f from time %d to time %d", NewCost, TD_LinkCostAry[LinkNo][t], t, NewToNodeArrivalTimeInterval);

					}
					if (NewCost < TD_LabelCostAry[ToID][NewToNodeArrivalTimeInterval]) // we only compare cost at the downstream node ToID at the new arrival time t
					{

						if (ToID == destination)
							bFeasiblePathFlag = true;


						if (debug_flag)
							TRACE("\n         UPDATE to %f, link cost %f at time %d", NewCost, TD_LinkCostAry[LinkNo][t], NewToNodeArrivalTimeInterval);

						// update cost label and node/time predecessor

						TD_LabelCostAry[ToID][NewToNodeArrivalTimeInterval] = NewCost;
						TD_NodePredAry[ToID][NewToNodeArrivalTimeInterval] = FromID;  // pointer to previous NODE INDEX from the current label at current node and time
						TD_LinkPredAry[ToID][NewToNodeArrivalTimeInterval] = LinkNo;  // pointer to previous LinkNo INDEX from the current label at current node and time
						TD_TimePredAry[ToID][NewToNodeArrivalTimeInterval] = t;  // pointer to previous TIME INDEX from the current label at current node and time

				
						if (ToNodeInScanListFlag == 0)  // since we have many time steps, so we only need to mark the node is in the scan list once across different time steps
						{
							ToNodeInScanListFlag = 1;

							// Dequeue implementation
							if (NodeStatusAry[ToID] == 2) // in the SEList_TD before
							{
								SEList_push_front(ToID);
								NodeStatusAry[ToID] = 1;
							}
							if (NodeStatusAry[ToID] == 0)  // not be reached
							{
								SEList_push_back(ToID);
								NodeStatusAry[ToID] = 1;
							}
						}


					}
				}
				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
			}

			
			// waiting time step
			// starting from the departure time + waiting time step at the same node
			for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
			{
				if (TD_LabelCostAry[ToID][t]<MAX_SPLABEL - 1)  // for feasible time-space point only
				{

					int time_stopped = 0;

					int NewToNodeArrivalTimeInterval = t + 1;  // time-dependent waitin time arc

					NewToNodeArrivalTimeInterval = GetFeasibleTDSPTimeInterval(NewToNodeArrivalTimeInterval);


					float NewCost = TD_LabelCostAry[ToID][t] + TD_LinkCostAry[LinkNo][t];
					// costs come from time-dependent resource price or road toll


					if (debug_flag && TD_LinkCostAry[LinkNo][t] > 0.0001)
					{
						TRACE("\n         Pay Price  %f, link cost %f from time %d to time %d", NewCost, TD_LinkCostAry[LinkNo][t], t, NewToNodeArrivalTimeInterval);

					}
					if (NewCost < TD_LabelCostAry[ToID][NewToNodeArrivalTimeInterval]) // we only compare cost at the downstream node ToID at the new arrival time t
					{

						if (ToID == destination)
							bFeasiblePathFlag = true;


						if (debug_flag)
							TRACE("\n         UPDATE to %f, link cost %f at time %d", NewCost, TD_LinkCostAry[LinkNo][t], NewToNodeArrivalTimeInterval);

						// update cost label and node/time predecessor

						TD_LabelCostAry[ToID][NewToNodeArrivalTimeInterval] = NewCost;
						TD_NodePredAry[ToID][NewToNodeArrivalTimeInterval] = FromID;  // pointer to previous NODE INDEX from the current label at current node and time
						TD_LinkPredAry[ToID][NewToNodeArrivalTimeInterval] = LinkNo;  // pointer to previous LinkNo INDEX from the current label at current node and time
						TD_TimePredAry[ToID][NewToNodeArrivalTimeInterval] = t;  // pointer to previous TIME INDEX from the current label at current node and time

						if (ToNodeInScanListFlag == 0)  // since we have many time steps, so we only need to mark the node is in the scan list once across different time steps
						{
							ToNodeInScanListFlag = 1;

							// Dequeue implementation
							if (NodeStatusAry[ToID] == 2) // in the SEList_TD before
							{
								SEList_push_front(ToID);
								NodeStatusAry[ToID] = 1;
							}
							if (NodeStatusAry[ToID] == 0)  // not be reached
							{
								SEList_push_back(ToID);
								NodeStatusAry[ToID] = 1;
							}

						}
					}
				}
				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
			}


		}      // end of for each link

	}	// end of while

	if (bFeasiblePathFlag == false)
		return 0;

	return FindOptimalSolution(origin, departure_time, destination, PathLinkList, TargetTravelTime, TotalCost, OptimialTravelTimeInMin);
}
int DTANetworkForSP::FindOptimalSolution(int origin, int departure_time, int destination, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float TargetTravelTimeInMin, float &TotalCost, float &OptimialTravelTimeInMin)  // the last pointer is used to get the node array
{

	// step 1: scan all the time label at destination node, consider time cost
	// step 2: backtrace to the origin (based on node and time predecessors)
	// step 3: reverse the backward path
	// return final optimal solution

	// step 1: scan all the time label at destination node, consider time cost
	int tmp_AryTN[MAX_NODE_SIZE_IN_A_PATH]; //backward temporal solution
	int tmp_AryTLink[MAX_NODE_SIZE_IN_A_PATH]; //backward temporal link solution

	float min_cost = MAX_SPLABEL;
	int min_cost_time_index = -1;

	int BeginOfTDSPCalculationInterval = max(0, (departure_time - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin);
	int EndOfTDSPCalculationInterval = min(m_NumberOfTDSPCalculationIntervals, (departure_time + TargetTravelTimeInMin - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin);

	for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
	{
		if (TD_LabelCostAry[destination][t] < min_cost)
		{
			min_cost = TD_LabelCostAry[destination][t];
			min_cost_time_index = t;
		}

	}
	
	TotalCost = min_cost;

	if (min_cost_time_index < 0)
		return 0;
		// if min_cost_time_index ==-1, then no feasible path if founded

	// step 2: backtrace to the origin (based on node and time predecessors)

	int	NodeSize = 0;

	//record the first node backward, destination node
	tmp_AryTN[NodeSize] = destination;

	NodeSize++;

	int PredTime = TD_TimePredAry[destination][min_cost_time_index];
	int PredNode = TD_NodePredAry[destination][min_cost_time_index];
	int PredLink = TD_LinkPredAry[destination][min_cost_time_index];
	int LinkSize = 0;
	tmp_AryTLink[LinkSize++] = PredLink;

	while (PredNode != origin && PredNode != -1 && NodeSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
	{
		ASSERT(NodeSize< MAX_NODE_SIZE_IN_A_PATH - 1);

		tmp_AryTN[NodeSize] = PredNode;

		NodeSize++;

		//record current values of node and time predecessors, and update PredNode and PredTime
		int PredTime_cur = PredTime;
		int PredNode_cur = PredNode;

		//TRACE("\n PredTime_cur = %d, PredNode_cur = %d ", PredTime_cur, PredNode_cur);

		PredNode = TD_NodePredAry[PredNode_cur][PredTime_cur];
		PredLink = TD_LinkPredAry[PredNode_cur][PredTime_cur];
		PredTime = TD_TimePredAry[PredNode_cur][PredTime_cur];
		tmp_AryTLink[LinkSize++] = PredLink;

	}

	OptimialTravelTimeInMin = (min_cost_time_index - PredTime) / g_TDSPTimetIntervalSizeForMin;
	tmp_AryTN[NodeSize] = origin;

	NodeSize++;

	// step 3: reverse the backward solution

	int i;
	int j = 0;
	for (i = LinkSize - 1; i >= 0; i--)
	{
		PathLinkList[j++] = tmp_AryTLink[i];
	}

	return NodeSize;

}


int DTANetworkForSP::FindOptimalLinkPath_TDLabelCorrecting_DQ(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, float TargetTravelTime, float &OptimialTravelTimeInMin, bool bDebugFlag)
// time -dependent label correcting algorithm with deque implementati
{

#ifndef _large_memory_usage_lr
	return 0;
#endif

	int BeginOfTDSPCalculationInterval = max(0, (departure_time - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin);
	int EndOfTDSPCalculationInterval = min(m_NumberOfTDSPCalculationIntervals, (departure_time + TargetTravelTime+1 - m_StartTimeInMin)* g_TDSPTimetIntervalSizeForMin );
	int i;
	int debug_flag = 0;  // set 1 to debug the detail information
	if (debug_flag)
		TRACE("\nCompute shortest path from %d at time %d", origin, departure_time);

	bool bFeasiblePathFlag = false;

	if (m_OutboundSizeAry[origin] == 0)
		return false;

	for (i = 0; i <m_LinkSize; i++) // Initialization for all nodes
	{
		LinkStatusAry[i] = 0;

		for (int t = BeginOfTDSPCalculationInterval; t <EndOfTDSPCalculationInterval; t++)
		{
			TD_InflowLinkLabelCostAry[i][t] = MAX_SPLABEL;
			TD_InflowEntranceQueueLabelCostAry[i][t] = MAX_SPLABEL;
			TD_OutflowLinkLabelCostAry[i][t] = MAX_SPLABEL;
			TD_OutflowExitQueueLabelCostAry[i][t] = MAX_SPLABEL;

			TD_InflowLinkTimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current link and time
			TD_InflowEntranceQueueTimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current link and time
			TD_OutflowLinkTimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
			TD_OutlowExitQueueTimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and tim

			int aggregated_time_interval = g_FindAssignmentIntervalIndexFromTime(m_StartTimeInMin + t / g_TDSPTimetIntervalSizeForMin);
			TD_LinkTimeIntervalAry[i][t] = max(1, (int)(m_LinkTDTimeAry[i][aggregated_time_interval] * g_TDSPTimetIntervalSizeForMin + 0.5));

		}

	}


	float AdditionalCostInMin = 0;

	if (m_OutboundSizeAry[origin] == 0)
		return 0;

	if (m_InboundSizeAry[destination] == 0)
		return 0;

	// Initialization for origin node: for all outgoing links from origin node

	LinkBasedSEList_clear();
	for (i = 0; i< m_OutboundSizeAry[origin]; i++)
	{
		int LinkID = m_OutboundLinkAry[origin][i];

		int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(departure_time) ;
		LinkLabelTimeAry[LinkID] = departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];

		if (bGeneralizedCostFlag == false)
			LinkLabelCostAry[LinkID] = departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];
		else  // for generalized cost, we start from cost of zero, other than start time
			LinkLabelCostAry[LinkID] = 0 + g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[0];

		LinkBasedSEList_push_back(LinkID);

		if (m_ToIDAry[LinkID] == destination)  //reach destination on the first link
		{
			PathLinkList[0] = LinkID;
			return 2; // 2 nodes
		}
	}


	int FromLinkID, ToLinkID, NodeID;
	float CostUpperBound = MAX_SPLABEL;

	float NewTime, NewCost;
	while (!LinkBasedSEList_empty())
	{

		FromLinkID = LinkBasedSEList_front();
		LinkBasedSEList_pop_front();

		if (debug_flag)
		{
			TRACE("\nScan from link %d ->%d", g_NodeVector[m_FromIDAry[FromLinkID]].m_NodeNumber, g_NodeVector[m_ToIDAry[FromLinkID]].m_NodeNumber);
		}

		LinkStatusAry[FromLinkID] = 2;        //scaned

		for (i = 0; i<m_OutboundMovementSizeAry[FromLinkID]; i++)  // for each arc (i,j) belong to A(i)
		{
			ToLinkID = m_OutboundMovementAry[FromLinkID][i];
			int FromID = m_FromIDAry[FromLinkID];

			if (debug_flag)  // physical nodes
			{
				if (g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber == 80125)
				{
					TRACE("Trace!");
				}
				//			TRACE("\n   to link %d, from node: %d, downstream node %d ", ToLinkID, g_NodeVector[FromID].m_NodeNumber, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber );
			}
			// need to check here to make sure  LabelTimeAry[FromID] is feasible.


			if (m_LinkConnectorFlag[ToLinkID] == 1)  // only check the following speical condition when a link is a connector
			{
				int OriginTAZ = m_LinkConnectorOriginZoneIDAry[ToLinkID];
				int DestinationTAZ = m_LinkConnectorDestinationZoneIDAry[ToLinkID];

				if (OriginTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ <= 0 && OriginTAZ != origin_zone)
					continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

				if (DestinationTAZ >= 1 /* TAZ >=1*/ && OriginTAZ <= 0 && DestinationTAZ != destination_zone)
					continue;  // special feature 2: skip connectors with destination TAZ that do not belong to this destination zone

				if (OriginTAZ >= 1 /* TAZ >=1*/ && OriginTAZ != origin_zone  && DestinationTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ != destination_zone)
					continue;  // special feature 3: skip connectors (with both TAZ at two ends) that do not belong to the origin/destination zones

			}

			if (m_ToIDAry[ToLinkID] == origin) // special feature 2: no detour at origin
				continue;

			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LinkLabelTimeAry[FromLinkID]);

			//  original code				NewTime	= LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

			if (bGeneralizedCostFlag)
				NewTime = LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];
			else
			{// non- distance

					float preceived_travel_time =  m_LinkTDTimeAry[ToLinkID][link_entering_time_interval];

					NewTime = LinkLabelTimeAry[FromLinkID] + preceived_travel_time + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

					double movement_delay_in_min = m_OutboundMovementDelayAry[FromLinkID][i];
					if (movement_delay_in_min >= 90 && debug_flag)
					{
						TRACE("prohibited movement!");
					}

			}



			// original code			NewCost    = LinkLabelCostAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];


			//  road pricing module
			float toll_in_min = 0;			// special feature 5: road pricing
			if (VOT > 0.01 && g_LinkTDCostAry[ToLinkID][link_entering_time_interval].m_bMonetaryTollExist)
			{ // with VOT and toll
				toll_in_min = g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue[demand_type] / VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions
			}


			if (debug_flag == 1 && ToLinkID == 5 && link_entering_time_interval >= 29)
			{
				TRACE("problematic node!");

			}

			if (g_LinkTDCostAry[ToLinkID][link_entering_time_interval].m_bTravelTimeTollExist)
			{
				toll_in_min = g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue[demand_type];
				if (debug_flag)
					TRACE("AdditionalCostInMin = %f\n", toll_in_min);
			}
			// special feature 6: update cost
			if (bGeneralizedCostFlag)
				NewCost = LinkLabelCostAry[FromLinkID] + g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue[0];  // we only consider the genralized cost value (without actual travel time)
			else
			{  // non distance cost
					NewCost = LinkLabelCostAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i] + toll_in_min;       // costs come from time-dependent tolls, VMS, information provisions

			}


			if (NewCost < LinkLabelCostAry[ToLinkID] && NewCost < CostUpperBound) // special feature 7.1  we only compare cost not time
			{
				if (debug_flag)  // physical nodes
				{
					TRACE("\n         UPDATE to link %d, downstream node %d, cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber, NewCost, m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
				}

				if (NewTime > g_PlanningHorizon - 1)
					NewTime = float(g_PlanningHorizon - 1);

				LinkLabelTimeAry[ToLinkID] = NewTime;
				LinkLabelCostAry[ToLinkID] = NewCost;
				LinkPredAry[ToLinkID] = FromLinkID;

				if (m_ToIDAry[ToLinkID] == destination) // special feature 7.2  : update upper bound cost
				{
					CostUpperBound = LinkLabelCostAry[ToLinkID];
				}

				// Dequeue implementation
				//
				if (LinkStatusAry[ToLinkID] == 2) // in the SEList_TD before
				{
					LinkBasedSEList_push_front(ToLinkID);
					LinkStatusAry[ToLinkID] = 1;
				}
				if (LinkStatusAry[ToLinkID] == 0)  // not be reached
				{
					LinkBasedSEList_push_back(ToLinkID);
					LinkStatusAry[ToLinkID] = 1;
				}

				//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
			}
			else
			{

				if (debug_flag == 1 && LinkLabelCostAry[ToLinkID] <= 900)  // physical nodes
				{
					TRACE("\n        not-UPDATEd to link %d, downstream node %d, new cost %f, old cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber, NewCost, LinkLabelCostAry[ToLinkID], m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
				}

			}

		}      // end of for each link

	} // end of while


	// post processing: if destination >=0: physical node

	//step 1: find the incoming link has the lowerest cost to the destinatin node

	float min_cost = MAX_SPLABEL;
	int link_id_with_min_cost = -1;
	for (i = 0; i< m_InboundSizeAry[destination]; i++)
	{
		int incoming_link = m_InboundLinkAry[destination][i];
		if (LinkLabelCostAry[incoming_link] < min_cost && LinkPredAry[incoming_link] >= 0)
		{
			min_cost = LinkLabelCostAry[incoming_link];
			link_id_with_min_cost = incoming_link;
		}
	}


	TotalCost = min_cost - departure_time;

	if (link_id_with_min_cost <0)
	{

		int node_number = g_NodeVector[destination].m_NodeNumber;
		cout << "Destination Node " << node_number << " cannot be reached from Origin Node " << g_NodeVector[origin].m_NodeNumber << endl;
		g_LogFile << "Error: Destination Node " << node_number << " cannot be reached from Origin Node " << g_NodeVector[origin].m_NodeNumber << endl;


		/*	if(g_path_error==0)
		{

		cout << endl << "Please check file output_simulation.log. Please any key to continue... " <<endl;

		getchar();
		g_path_error ++;
		}*/


		return 0;
		//find shortest path without movement penality
		g_ShortestPathWithMovementDelayFlag = false;

		int number_of_nodes = FindBestPathWithVOTForSingleAgent(origin_zone, origin, departure_time, destination_zone, destination, demand_type,
			VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, debug_flag);
		// check if the link(s) have the predecesssor

		for (int ii = 0; ii<number_of_nodes - 1; ii++)
		{

			if (PathLinkList[ii] <0 || PathLinkList[ii]> g_LinkVector.size())
			{
				g_ProgramStop();

			}
			cout << "Link " << g_LinkVector[PathLinkList[ii]]->m_FromNodeNumber << "->" << g_LinkVector[PathLinkList[ii]]->m_ToNodeNumber << " with cost " << LinkLabelCostAry[PathLinkList[ii]] << endl;
		}


		getchar();
	}

	//step 2 trace the incoming link to the first link in origin node

	int LinkSize = 0;
	temp_reversed_PathLinkList[LinkSize++] = link_id_with_min_cost;  // last link to destination
	int PrevLinkID = LinkPredAry[link_id_with_min_cost];

	if (PrevLinkID == -1)
	{
		TRACE("Error!");

	}
	temp_reversed_PathLinkList[LinkSize++] = PrevLinkID;   //second last link

	while (PrevLinkID != -1 && LinkSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
	{
		ASSERT(LinkSize< MAX_NODE_SIZE_IN_A_PATH - 1);

		if (LinkPredAry[PrevLinkID] != -1)
		{
			temp_reversed_PathLinkList[LinkSize++] = LinkPredAry[PrevLinkID];
		}
		PrevLinkID = LinkPredAry[PrevLinkID];
	}

	int j = 0;
	for (i = LinkSize - 1; i >= 0; i--)
	{
		PathLinkList[j++] = temp_reversed_PathLinkList[i];
		if (debug_flag)
		{
			TRACE("\n  link no. %d, %d, %d ->%d", i, temp_reversed_PathLinkList[i]
				, m_FromIDAry[temp_reversed_PathLinkList[i]], m_ToIDAry[temp_reversed_PathLinkList[i]]);
		}

	}

	if (debug_flag)
	{
		TRACE("\nPath sequence end, cost = ..%f\n", min_cost);
	}

	return LinkSize + 1;  // as node size

	if (bFeasiblePathFlag == false)
		return 0;

	return FindOptimalSolution(origin, departure_time, destination, PathLinkList, TargetTravelTime, TotalCost, OptimialTravelTimeInMin);
}




float DTANetworkForSP::AgentBasedUpperBoundSolutionGeneration(int zone, int departure_time_begin, int departure_time_end, int iteration)
// for vehicles starting from departure_time_begin to departure_time_end, assign them to shortest path using a proportion according to MSA or graident-based algorithms
{
	//	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH];
	int TempPathLinkList[MAX_NODE_SIZE_IN_A_PATH];

	std::vector<DTAVehicle*>::iterator iterVehicle = g_VehicleVector.begin();
	int NodeSize;
	int TempNodeSize;

	int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(departure_time_begin);

	int VehicleSize = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size();
	for (int vi = 0; vi < VehicleSize; vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		DTAVehicle* pVeh = g_VehicleMap[VehicleID];

		/// finding optimal path 
		bool bDebugFlag = false;

		float TotalCost;
		bool bGeneralizedCostFlag = false;

		// general settings

		TempNodeSize = 0;

		if (pVeh->m_OriginZoneID == pVeh->m_DestinationZoneID)
		{  // do not simulate intra zone traffic
			continue;
		}


		if (pVeh->m_AgentID == 1001)
			TRACE("");

		unsigned li;
			
		// copy road price for links already built
		int ti;
		for (li = 0; li < g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];
			if (pLink->m_NetworkDesignFlag >= 1 && pLink->m_NetworkDesignBuildCapacity <= 0.1)
			{

				for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
				{

					TD_LinkCostAry[li][ti] = MAX_SPLABEL;
				}
			}
			else
			{
				for (ti = 0; ti < m_NumberOfTDSPCalculationIntervals; ti += 1)
				{

					TD_LinkCostAry[li][ti] = 0;
				}
			}


		}

		

		float TargeTravelTimeInMin = g_NetworkDesignTravelTimeBudget;
		float OptimialTravelTimeInMin = 0;

		NodeSize = FindOptimalNodePath_TDLabelCorrecting_DQ(pVeh->m_OriginZoneID, pVeh->m_OriginNodeID, pVeh->m_DepartureTime,
			pVeh->m_DestinationZoneID, pVeh->m_DestinationNodeID, pVeh->m_DemandType, pVeh->m_VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, TargeTravelTimeInMin, OptimialTravelTimeInMin, bDebugFlag);

		float TargeTravelTimeInMin_global = TargeTravelTimeInMin;
		if (NodeSize >= 2 && OptimialTravelTimeInMin <= TargeTravelTimeInMin_global && TotalCost <= 1)   // a feasible path that meets the travel time target with lower road price of pie than 1
		{
			pVeh->m_bMeetTarget = true;
		}
		else
		{
			pVeh->m_bMeetTarget = false;

		}
	} // for each vehicle on this OD pair

	return 0;

}



void  g_NetworkDesignEnemerateAllSolutions()
{
	std::vector<int> combination_array;

	unsigned li;
	for (li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];
		if (pLink->TollVector.size() > 0)
		{
			VehicleLinkPrice element;
			element.LinkNo = pLink->m_LinkNo;
			element.RoadPrice = 0;
			element.RoadUsageFlag = 0;
			element.TotalTollColected = 0;

			g_NetworkDesignRoadConstructionVector.push_back(element);

		}
	}


	int n = g_NetworkDesignRoadConstructionVector.size();
	int r = g_NetworkDesignOptimalLinkSize;
	float MinUpperBound = 9999999;
	std::vector<bool> v(n);
	std::fill(v.begin() + n - r, v.end(), true);

	int count = 0;
	do {

		combination_array.clear();

		for (int i = 0; i < g_NetworkDesignRoadConstructionVector.size(); i++)
		{
			int LinNo = g_NetworkDesignRoadConstructionVector[i].LinkNo;
			DTALink* pLink = g_LinkVector[LinNo];
			pLink->m_NetworkDesignBuildCapacity = 0;
		}
		

		for (int i = 0; i < n; ++i) {
			if (v[i])
			{
				combination_array.push_back(i);
			}
		}

		for (int ci = 0; ci < combination_array.size(); ci++)
		{
			TRACE("%d,", combination_array[ci]);

			g_NetworkDesignLogFile << combination_array[ci] << " ";

			int LinNo = g_NetworkDesignRoadConstructionVector[combination_array[ci]].LinkNo;
			DTALink* pLink = g_LinkVector[LinNo];
			pLink->m_NetworkDesignBuildCapacity = 1;

		}


		g_GenerateUpperBoundFeasibleSolutionForAgents(g_DemandLoadingStartTimeInMin, g_DemandLoadingEndTimeInMin);

		float  UpperBoundObjectiveFunctionValue = g_CalculateUpperBoundValue();
		g_NetworkDesignLogFile << "----- no." << count << ", UpperBoundObjectiveFunctionValue=," << UpperBoundObjectiveFunctionValue << endl;
		count++;
		if (MinUpperBound > UpperBoundObjectiveFunctionValue)
		{
			MinUpperBound = UpperBoundObjectiveFunctionValue;
			g_NetworkDesignLogFile << "Upper Bound Value = " << MinUpperBound << endl;
			for (int ci = 0; ci < combination_array.size(); ci++)
			{
				int LinNo = g_NetworkDesignRoadConstructionVector[combination_array[ci]].LinkNo;
				DTALink* pLink = g_LinkVector[LinNo];
				g_NetworkDesignLogFile << " link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber << "" ;

			}


			cout << "Upper Bound Value = " << UpperBoundObjectiveFunctionValue << endl;
		}
		TRACE("\n");


	} while (std::next_permutation(v.begin(), v.end()));

}


class CIntermodalTransitionArc
{
public:
	int to_route_no;
	int to_time_t;
	int to_node_j;
	float transportation_cost;
};
class CIntermodalTransitionArcSet
{
public:

	std::vector<CIntermodalTransitionArc> m_Vector;

};


float*** l_state_node_label_cost = NULL;



CIntermodalTransitionArcSet ***l_state_vertex_adjacent_set = NULL;

int*** l_state_node_predecessor = NULL;
int*** l_state_time_predecessor = NULL;
int*** l_state_mode_predecessor = NULL;
int g_number_of_time_intervals_for_intermodal_routing = 1440;

float*** l_state_node_transition = NULL;

#define _MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS 1440
#define _MAX_NUMBER_OF_INTERMODAL_LINKS 10000




void g_allocate_intemodal_memory()
{
	//int number_of_states = g_StateVector.size();
	//int number_of_nodes = g_NodeVector.size();
	//int number_of_time_intervals = g_number_of_time_intervals_for_intermodal_routing;

	//cout << "number of states = " << g_StateVector.size() << endl;

	//l_state_vertex_adjacent_set = Allocate3DDynamicArray<CIntermodalTransitionArcSet>(number_of_nodes, number_of_time_intervals, number_of_states);

	//l_state_node_label_cost = Allocate3DDynamicArray<float>(number_of_nodes, number_of_time_intervals, number_of_states);
	//l_state_node_predecessor = Allocate3DDynamicArray<int>(number_of_nodes, number_of_time_intervals, number_of_states);
	//l_state_time_predecessor = Allocate3DDynamicArray<int>(number_of_nodes, number_of_time_intervals, number_of_states);
	//l_state_mode_predecessor = Allocate3DDynamicArray<int>(number_of_nodes, number_of_time_intervals, number_of_states);

}

void g_free_intermodal_memory()
{
	//int number_of_states = g_StateVector.size();
	//int number_of_nodes = g_NodeVector.size();
	//int number_of_time_intervals = g_number_of_time_intervals_for_intermodal_routing ;

	//Deallocate3DDynamicArray<CIntermodalTransitionArcSet>(l_state_vertex_adjacent_set, number_of_nodes, number_of_time_intervals);

	//Deallocate3DDynamicArray<float>(l_state_node_label_cost, number_of_nodes, number_of_time_intervals);
	//Deallocate3DDynamicArray<int>(l_state_node_predecessor, number_of_nodes, number_of_time_intervals);
	//Deallocate3DDynamicArray<int>(l_state_time_predecessor, number_of_nodes, number_of_time_intervals);
	//Deallocate3DDynamicArray<int>(l_state_mode_predecessor, number_of_nodes, number_of_time_intervals);

}

//class CIntermodalState  //class for intermodal scheduling states
//{
//public:
//	int route_id;
//
//};
//
//std::vector<CIntermodalState> g_StateVector;
//
//
//
//class CIntermodalLink  //class for intermodal scheduling states
//{
//public:
//	int from_node_id;
//	int to_node_id;
//	int from_route_id;
//	int to_route_id;
//
//};
//
//std::vector<CIntermodalLink> g_IMLinkVector;
//
//int g_FindTravelTime(int from_node_id, int to_node_id, int from_route_id, int to_route_id, int starting_time_in_min)
//{
//	return 10;
//}
//
//
//
//float g_optimal_time_dependenet_dynamic_programming(
//	int vehicle_id,
//	float arc_cost[_MAX_NUMBER_OF_INTERMODAL_LINKS][_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
//	int &path_number_of_nodes,
//	int path_node_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	int path_link_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	int path_time_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	int path_state_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	float path_cost_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS],
//	int travel_time_calculation_flag,
//	int vehicle_capacity,
//	float &travel_time_return_value)
//	// time-dependent label correcting algorithm with double queue implementation
//{
//
//	if (arrival_time_ending > g_number_of_time_intervals_for_intermodal_routing)
//	{
//		TRACE("error");
//		return;
//	}
//
//	float total_cost = MAX_SPLABEL;
//	//if (g_outbound_node_size[origin_node] == 0)
//	//{
//	//	return MAX_SPLABEL;
//	//}
//
//	// step 1: Initialization for all nodes
//	for (int i = 0; i < g_NodeVector.size(); i++) //Initialization for all nodes
//	{
//		for (int t = 0; t < g_number_of_time_intervals_for_intermodal_routing; t++)
//		{
//
//			for (int w = 0; w < g_StateVector.size(); w++)
//			{
//				l_state_node_label_cost[i][t][w] = MAX_SPLABEL;
//				l_state_node_predecessor[i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
//				l_state_time_predecessor[i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
//				l_state_mode_predecessor[i][t][w] = -1;
//			}
//		}
//	}
//
//	//step 2: Initialization for origin node at the preferred departure time, at departure time
//
//	int w0 = 0;  // start fro empty
//
//	l_state_node_label_cost[origin_node][departure_time_beginning][w0] = 0;
//
//	// step 3: //dynamic programming
//	for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
//	{
//		if (t % 10 == 0)
//		{
//			cout << "vehicle " << vehicle_id << " is scanning time " << t << "..." << endl;
//		}
//		for (int from_node = 0; from_node < g_NodeVector.size(); from_node++)  // for each link (i,j)
//		{
//			for (int l = 0; l < g_StateVector.size(); l++)
//			{
//
//				if (l_state_node_label_cost[from_node][t][l] < MAX_SPLABEL - 1)  // for feasible time-space point only
//				{
//
//
//					for (int w2_index = 0; w2_index < l_state_vertex_adjacent_set[from_node][t][l].m_Vector.size(); w2_index++)
//					{
//						CIntermodalTransitionArc element = l_state_vertex_adjacent_set[from_node][t][l].m_Vector[w2_index];
//
//						// part 1: link based update
//						int new_to_node_arrival_time = min(element.to_time_t, g_number_of_time_intervals_for_intermodal_routing - 1);
//
//						float temporary_label_cost = l_state_node_label_cost[from_node][t][l] + element.transportation_cost;
//
//						int lp = element.to_route_no;
//						int to_node = element.to_node_j;
//
//						if (temporary_label_cost < l_state_node_label_cost[to_node][new_to_node_arrival_time][lp]) // we only compare cost at the downstream node ToID at the new arrival time t
//						{
//
//							//if (g_shortest_path_debugging_flag)
//							//{
//							//	fprintf(g_pFileDebugLog, "DP: updating node: %d from time %d to time %d, current cost: %.2f, from cost %.2f ->%.2f\n",
//							//		to_node, t, new_to_node_arrival_time,
//							//		l_state_node_label_cost[from_node][t][w2],
//							//		l_state_node_label_cost[to_node][new_to_node_arrival_time][w2], temporary_label_cost);
//							//}
//
//							// update cost label and node/time predecessor
//
//							l_state_node_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
//							l_state_node_predecessor[to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
//							l_state_time_predecessor[to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
//							l_state_mode_predecessor[to_node][new_to_node_arrival_time][w2] = w1;
//						}
//					}
//					// w2
//				}  // feasible vertex label cost
//			}  // for all states
//
//		} // for all link
//	} // for all time t
//
//
//	//if (g_shortest_path_debugging_flag)
//	//{
//	//	fprintf(g_pFileDebugLog, "--Node label cost matrix--\n");
//
//	//	for (int i = 1; i <= g_number_of_nodes; i++)
//	//	{
//
//	//		for (int t = 0; t <= arrival_time_ending; t++)
//	//		{
//	//			//	if (g_node_label_cost[i][t] < _MAX_LABEL_COST - 1) // feasible cost label
//	//			for (int w = 0; w < g_VRStateVector.size(); w++)
//	//			{
//	//				if (l_state_node_label_cost[i][t][w] < _MAX_LABEL_COST - 1)
//	//				{
//	//					fprintf(g_pFileDebugLog, "Node %d @ %d: w = %d, %4.2f, node pred = %d time pred t= %d\n",
//	//						i, t, w, l_state_node_label_cost[i][t][w], l_state_node_predecessor[i][t][w], l_state_time_predecessor[i][t][w]);
//	//				}
//
//	//			}
//	//		}
//	//	}
//	//	fprintf(g_pFileDebugLog, "--End of node label cost matrix--\n");
//
//
//	//}
//
//
//	total_cost = MAX_SPLABEL;
//
//	int min_cost_time_index = arrival_time_ending;
//
//	int reversed_path_node_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS];
//	int reversed_path_time_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS];
//	int reversed_path_state_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS];
//	float reversed_path_cost_sequence[_MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS];
//
//	int w = 0;
//	total_cost = l_state_node_label_cost[destination_node][min_cost_time_index][w];
//
//	// step 2: backtrack to the origin (based on node and time predecessors)
//	int	node_size = 0;
//	reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
//	reversed_path_time_sequence[node_size] = min_cost_time_index;
//	reversed_path_state_sequence[node_size] = w;
//	reversed_path_cost_sequence[node_size] = l_state_node_label_cost[destination_node][min_cost_time_index][w];
//
//
//	node_size++;
//
//	int pred_node = l_state_node_predecessor[destination_node][min_cost_time_index][w];
//	int pred_time = l_state_time_predecessor[destination_node][min_cost_time_index][w];
//	int pred_state = l_state_mode_predecessor[destination_node][min_cost_time_index][w];
//
//	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_INTERMODAL_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
//	{
//		reversed_path_node_sequence[node_size] = pred_node;
//		reversed_path_time_sequence[node_size] = pred_time;
//		reversed_path_state_sequence[node_size] = pred_state;
//		reversed_path_cost_sequence[node_size] = l_state_node_label_cost[pred_node][pred_time][pred_state];
//
//		node_size++;
//
//		//record current values of node and time predecessors, and update PredNode and PredTime
//
//		int pred_node_record = pred_node;
//		int pred_time_record = pred_time;
//		int pred_state_record = pred_state;
//
//		pred_node = l_state_node_predecessor[pred_node_record][pred_time_record][pred_state_record];
//		pred_time = l_state_time_predecessor[pred_node_record][pred_time_record][pred_state_record];
//		pred_state = l_state_mode_predecessor[pred_node_record][pred_time_record][pred_state_record];
//
//	}
//
//	//reverse the node sequence 
//
//	for (int n = 0; n < node_size; n++)
//	{
//		path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
//		path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
//		path_state_sequence[n] = reversed_path_state_sequence[node_size - n - 1];
//		path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
//	}
//
//	for (int i = 0; i < node_size - 1; i++)  // for each link, 
//	{
//
//		DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i], path_node_sequence[i + 1])];
//
//		if (pLink != NULL)
//			path_link_sequence[i] = pLink->m_LinkNo;
//	}
//
//	travel_time_return_value = path_time_sequence[node_size - 1] - path_time_sequence[0];
//
//	path_number_of_nodes = node_size;
//	return total_cost;
//
//}
