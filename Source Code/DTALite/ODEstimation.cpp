//  Portions Copyright 2010 Xuesong Zhou, Hao Lei

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
#include "CSVParser.h"
#include "DTALite.h"
#include "GlobalData.h"
#include <algorithm>

using namespace std;

std::vector<PathArrayForEachODTK> g_ODTKPathVector;
/***************
to do list:
completed: read historical demand file

0. read target OD demand

1. Read sensor data, assign data to each link. NEXTA format, speed/ travel time, link count or density

2. Perform traffic assignment, compare the error statistics based on vehicle trajectory
calculate the link MOE error(l,tau) = error term (obs MOE- simulated)

3. iterative adjustment at each iteration

3.1 for each link, for each observation time interval tau, identify the state and the end timestamp of a traffic state phase (FF-FF, FF-PC-FF, FF-PC-CC, CC-CC, CC-PC-FF)
min - by - min

3.2 gather all vehicle data to reconstruct a dynamic OD matrix,
calcuate each OD error term (target - simulated OD)

3.3 for each vehicle

path flow marginal = OD error term () + sub-gradient multiplier * (current cost - shortest path cost)

starting time (ARR arrival time) assuming in FF
walk through each link l in the path 

Find NTS for time ARR on link l

--- update vehicle marginal ---

--	FLOW MOE --
{
for the next timestamp (NTS) of the current phase
path flow marginal += link flow error(l,ARR)* +1
}

--	DENSITY MOE -- 
from t = ARR to NTS
{
FF-FF, FF-PC-FF:
path flow marginal += link DENSITY error(l,t)
}
-- Travel time MOE --
{
FF-FF, CC-CC: no change
FF--PC-FF or FF--PC-CC
path flow marginal += link travel time error(l,t)

}

FF: move to the next link. 
ARR = ARR+ FFTT(l)
l = l+1

FF-PC-FF:
ARR = NTS+ FFTT(l)
l = l+1

FF-PC-CC: 
move back to the previous link, l= l-1
ARR = NTS - FFTT(l) /// now l is "l-1" previous link of the current link"

CC-CC:
move back to the previous link, l= l-1
ARR = NTS - FFTT(l-1)

ensure (ARR is greater then the departure time and the end of simulation time interval)

ensure (l >=0, l <= number of links)

end of case


path flow marginal > 0 should reduce flow

1 (current flow unit) - step size * path flow marginal

run (- step size * path flow marginal)

endfor


*////////

float    g_ODEstimation_WeightOnHistODDemand = 0.1f;
float	 g_ODEstimation_WeightOnUEGap = 1.0f;

float    g_ODEstimation_Weight_Flow = 1.0f;
float    g_ODEstimation_Weight_NumberOfVehicles = 1.0f;
float    g_ODEstimation_Weight_TravelTime = 1.0f;
float    g_ODEstimation_StepSize  = 0.1f;

bool g_ObsDensityAvailableFlag = false;
bool g_ObsTravelTimeAvailableFlag = false;

int g_ODEstimationFlag = 0;
int g_SensorDataCount = 0;
int g_Agent_shortest_path_generation_flag = 0;
int g_ODEstimationMeasurementType = 0; // 0: flow, 1: density, 2, speed
int g_ODEstimation_StartingIteration = 2;
int g_ODEstimation_EndingIteration = 2;
float g_ODEstimation_max_ratio_deviation_wrt_hist_demand;

int g_ODEstimationNumberOfIterationsForSequentialAdjustment = 10;
int g_ODEstimationTimePeriodForSequentialAdjustment = 60; // min
int g_ODEstimationWeightOnTravelTimeDeviationFreeflow = 100;
int g_ODEstimationWeightOnTravelTimeDeviationCongested = 10;

int g_ODEstimation_alpha_based_adjustment = 1;


int g_ODDemandIntervalSize = 1;
std::map<long, long> g_LinkIDtoSensorIDMap;

void HistoricalDemand::UpdatedDemandParameters(int origin_zone_number, int destination_zone_number, int t, float flow_value, float flow_gradient)
{
	float alpha_step_size = g_ODEstimation_StepSize;
	float beta_step_size = 0.1;

	if (fabs(flow_gradient) < 0.0000001)
		return;
	int i = g_ZoneMap[origin_zone_number].m_ZoneSequentialNo;
	int j = g_ZoneMap[destination_zone_number].m_ZoneSequentialNo;



	m_alpha[i] -= alpha_step_size*flow_gradient* flow_value ;
#pragma omp critical

	{
	_proxy_ODME_log(1, 0, "flow_gradient = %f,flow_value = %f,m_alpha_last_value[%d] = %f,  updated m_alpha = %f, diff = %f\n",
		
		flow_gradient, flow_value, i, m_alpha_last_value[i], m_alpha[i], m_alpha[i] - m_alpha_last_value[i]);
	}

	if (m_alpha[i] <= m_alpha_hist_value[i] * (1 - g_ODEstimation_max_ratio_deviation_wrt_hist_demand))
		m_alpha[i] = m_alpha_hist_value[i] * (1 - g_ODEstimation_max_ratio_deviation_wrt_hist_demand);

	if (m_alpha[i] <= m_alpha_hist_value[i] * 0.1)  // non negative constraints
		m_alpha[i] = m_alpha_hist_value[i] * 0.1;



	if (m_alpha[i] >= m_alpha_hist_value[i] * (1 + g_ODEstimation_max_ratio_deviation_wrt_hist_demand))
		m_alpha[i] = m_alpha_hist_value[i] * (1 + g_ODEstimation_max_ratio_deviation_wrt_hist_demand);


	
}

bool g_GetSequentialEstimationTimePeriod(int iteration,float ArrivalTime)
{
	// 20 iterations for each hour adjustment period

	int ODME_iteration_no = iteration - g_ODEstimation_StartingIteration;

	int SequentialAdjustmentTimePeriodStart = g_ValidationDataStartTimeInMin + ODME_iteration_no/g_ODEstimationNumberOfIterationsForSequentialAdjustment*60; // 60 min

	if(SequentialAdjustmentTimePeriodStart>= g_ValidationDataEndTimeInMin) // restart from the beginning 
		SequentialAdjustmentTimePeriodStart = g_ValidationDataStartTimeInMin; 

	int SequentialAdjustmentTimePeriodEnd = SequentialAdjustmentTimePeriodStart +  g_ODEstimationTimePeriodForSequentialAdjustment; // 60 min

	if(ArrivalTime >= SequentialAdjustmentTimePeriodStart && ArrivalTime <=SequentialAdjustmentTimePeriodEnd)
		return true;
	else 
		return false;

}
bool g_ReadLinkMeasurementFile()
{
	CCSVParser parser;
	int count = 0;
	int error_count = 0;

	int count_sensor_count = 0;
	int speed_sensor_count = 0;

	int last_day_no = -1;

	int specific_dayno = -1;
	if(g_ODEstimationFlag==1 )  // do not change hist demand when creating vehicles in the middle of  ODME , called by  g_GenerateVehicleData_ODEstimation()
	{
		TCHAR ODMESettingFileName[_MAX_PATH] = _T("./DTASettings.txt");

		g_ODEstimation_WeightOnHistODDemand = g_GetPrivateProfileFloat("estimation", "weight_on_hist_oddemand", 1, ODMESettingFileName,true);
		g_ODEstimation_WeightOnUEGap = g_GetPrivateProfileFloat("estimation", "weight_on_ue_gap", 1, ODMESettingFileName,true);
		g_ODEstimationNumberOfIterationsForSequentialAdjustment = g_GetPrivateProfileInt("estimation", "number_of_iterations_per_sequential_adjustment", 10, ODMESettingFileName,true);	
		g_ODEstimationTimePeriodForSequentialAdjustment = g_GetPrivateProfileInt("estimation", "time_period_in_min_per_sequential_adjustment", 60, ODMESettingFileName,true);	
		g_ODEstimationWeightOnTravelTimeDeviationCongested = g_GetPrivateProfileInt("estimation", "weight_on_travel_time_deviation_congested", 10, ODMESettingFileName, true);
		g_ODEstimationWeightOnTravelTimeDeviationFreeflow = g_GetPrivateProfileInt("estimation", "weight_on_travel_time_deviation_freeflow", 100, ODMESettingFileName, true);
		g_ODEstimation_alpha_based_adjustment = g_GetPrivateProfileInt("estimation", "alpha_based_adjustment_mode", 0, ODMESettingFileName, true);
		specific_dayno = g_GetPrivateProfileInt("estimation", "specific_day_no", -1, ODMESettingFileName, true);
	}

	// sensor count data
	int early_start_time_in_min = 99999;
	int late_end_time_in_min = 0;

	if (parser.OpenCSVFile("sensor_count.csv", g_ODEstimationFlag == 1))
	{
		int sensor_count = 0;
		while (parser.ReadRecord())
		{

			std::string count_sensor_id;

			parser.GetValueByFieldName("count_sensor_id", count_sensor_id);

		

			DTALink* pLink = NULL;

			if (count_sensor_id.size() > 0)
			{
				if (g_CountSensorIDMap.find(count_sensor_id.c_str()) != g_CountSensorIDMap.end())
				{
					pLink = g_CountSensorIDMap[count_sensor_id.c_str()];
				}
				else
				{
					if (error_count <= 10)
					{
						cout << "count_sensor_id " << count_sensor_id << ": at line " << count + 1 << " of file sensor_count.csv  has not been defined in input_link.csv." << endl;
					}
					g_LogFile << "count_sensor_id " << count_sensor_id << ": at line " << count + 1 << " of file sensor_count.csv  has not been defined in input_link.csv." << endl;

					error_count++;
					continue;
				}
			}
			else  // empty string in count sensor id
			{

				int  from_node_id = 0;
				int to_node_id = 0;
				if (pLink == NULL)
				{
					parser.GetValueByFieldName("from_node_id", from_node_id);
					parser.GetValueByFieldName("to_node_id", to_node_id);

					if (g_LinkMap.find(GetLinkStringID(from_node_id, to_node_id)) != g_LinkMap.end())
					{
						pLink = g_LinkMap[GetLinkStringID(from_node_id, to_node_id)];
					}
					if (pLink == NULL)
					{

						cout << "link " << from_node_id << "->" << to_node_id << ": at line " << count + 1 << " of file sensor_count.csv  has not been defined in input_link.csv." << endl;
						error_count++;
						continue;


					}

				}
			}




			int start_time_in_min = 0;
			int end_time_in_min = 0;
			int volume_count = -1;

			parser.GetValueByFieldNameRequired("start_time_in_min", start_time_in_min);
			parser.GetValueByFieldNameRequired("end_time_in_min", end_time_in_min);

			if (start_time_in_min < early_start_time_in_min)
				early_start_time_in_min = start_time_in_min;

			if (end_time_in_min > late_end_time_in_min)
				late_end_time_in_min = end_time_in_min;

			int dest_node_id = -1;
			parser.GetValueByFieldNameRequired("link_count", volume_count);


			pLink->m_bSensorData = true;
			SLinkMeasurement element;

			element.StartTime = start_time_in_min;
			element.EndTime = end_time_in_min;

			if (volume_count >= 1)
			{
				element.ObsFlowCount = volume_count;
				count_sensor_count++;
			}

			//if (density >= 1)
			//{
			//	element.ObsDensity = density;
			//	g_ObsDensityAvailableFlag = true;
			//}

	
			std::string second_count_sensor_id;
			parser.GetValueByFieldName("second_count_sensor_id", second_count_sensor_id);


			DTALink* pLink2 = NULL;  // find the second link in the movement.

			if (second_count_sensor_id.size() > 0 && g_CountSensorIDMap.find(second_count_sensor_id.c_str()) != g_CountSensorIDMap.end())
			{
				pLink2 = g_CountSensorIDMap[second_count_sensor_id.c_str()];
				element.MovementDestinationNode = pLink2->m_ToNodeNumber;

			}
			_proxy_ODME_log(0, 0, "push %d th element [%d->%d, time %d->%d (min), link count %f]\n", 
				g_SensorDataCount + 1, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber,
				element.StartTime, element.EndTime, element.ObsFlowCount
				);

			pLink-> m_LinkMeasurementAry.push_back(element);
			g_SensorDataCount++;

			count++;


		}

		parser.CloseCSVFile();
	}
		// sensor speed data
	last_day_no = -1;

		if (parser.OpenCSVFile("sensor_speed.csv", false))
		{

			int sensor_count = 0;
			while (parser.ReadRecord())
			{

				std::string speed_sensor_id;

				parser.GetValueByFieldName("speed_sensor_id", speed_sensor_id);


				DTALink* pLink = NULL;

				

				if (speed_sensor_id.size() > 0)
				{
					if (g_SpeedSensorIDMap.find(speed_sensor_id.c_str()) != g_SpeedSensorIDMap.end())
					{
						pLink = g_SpeedSensorIDMap[speed_sensor_id.c_str()];
					}
					else
					{
						if (error_count <= 10)
						{

							cout << "speed_sensor_id " << speed_sensor_id << ": at line " << count + 1 << " of file sensor_speed.csv  has not been defined in input_link.csv." << endl;

						}
						
						g_LogFile << "speed_sensor_id " << speed_sensor_id << ": at line " << count + 1 << " of file sensor_speed.csv  has not been defined in input_link.csv." << endl;

						error_count++;
						continue;
					}
				}
				else
				{



					int  from_node_id = 0;
					int to_node_id = 0;
					if (pLink == NULL)
					{
						parser.GetValueByFieldName("from_node_id", from_node_id);
						parser.GetValueByFieldName("to_node_id", to_node_id);

						if (g_LinkMap.find(GetLinkStringID(from_node_id, to_node_id)) != g_LinkMap.end())
						{
							pLink = g_LinkMap[GetLinkStringID(from_node_id, to_node_id)];
						}


					}

					if (pLink == NULL)
					{

						cout << "link " << from_node_id << "->" << to_node_id << ": at line " << count + 1 << " of file sensor_speed.csv  has not been defined in input_link.csv." << endl;
						error_count++;
						continue;


					}
				}

				int start_time_in_min = 0;
				int end_time_in_min = 0;
				int volume_count = -1;

				parser.GetValueByFieldNameRequired("start_time_in_min", start_time_in_min);
				parser.GetValueByFieldNameRequired("end_time_in_min", end_time_in_min);

				if (start_time_in_min < early_start_time_in_min)
					early_start_time_in_min = start_time_in_min;

				if (end_time_in_min > late_end_time_in_min)
					late_end_time_in_min = end_time_in_min;

				float avg_speed = 0;
				parser.GetValueByFieldName("speed", avg_speed);

				pLink->m_bSensorData = true;
				SLinkMeasurement element;

				element.StartTime = start_time_in_min;
				element.EndTime = end_time_in_min;

				element.ObsTravelTime = pLink->m_Length / max(1, avg_speed) * 60;

				pLink->m_LinkMeasurementAry.push_back(element);
				g_ObsTravelTimeAvailableFlag = true;
				speed_sensor_count++;
				g_SensorDataCount++;

				count++;


			}

			parser.CloseCSVFile();

	}
	if (g_ODEstimationFlag == 1)
	{

		if (count == 0)
		{
		
		cout << "ODME mode is used, but file sensor_count.csv or sensor_speed.csv has 0 valid sensor record with min " << g_ValidationDataStartTimeInMin << " -> min " <<
			g_ValidationDataEndTimeInMin << ". Please check input_scenario_settings.csv and sensor_count.csv/sensor_speed.csv." << endl;

		if (g_ValidationDataEndTimeInMin < early_start_time_in_min || late_end_time_in_min < g_ValidationDataStartTimeInMin)
		{
			cout << "calibration_data_start_time_in_min = " << g_ValidationDataStartTimeInMin << endl;
			cout << "calibration_data_end_time_in_min = " << g_ValidationDataEndTimeInMin << endl;
			cout << "time period in input_scenario_settings = [" << early_start_time_in_min << "," << late_end_time_in_min << "]" << endl;

		}


		g_ProgramStop();
		}
		else
		{
			CString calibration_interval_period_str;
			calibration_interval_period_str.Format("%s-> %s", GetTimeClockString(g_ValidationDataEndTimeInMin).c_str(), GetTimeClockString(g_ValidationDataEndTimeInMin).c_str());
			g_SummaryStatFile.WriteParameterValue("ODME time period", calibration_interval_period_str);

			CString str;
			g_SummaryStatFile.WriteParameterValue("# of sensor count records for ODME", count_sensor_count);
			g_SummaryStatFile.WriteParameterValue("# of sensor speed records for ODME", speed_sensor_count);
		
		}

	}

	cout << "File sensor_count.csv and sensor_speed.csv has " << count << " valid sensor records." << endl;
	g_LogFile << "Reading file sensor_count.csv and sensor_speed.csv with " << count << " valid sensor records." << endl;

	return true;
}





void ConstructPathArrayForEachODT_ODEstimation(int iteration,std::vector<PathArrayForEachODT> PathArray, int origin_zone, int AssignmentInterval)
{  // this function has been enhanced for path flow adjustment
	// step 1: initialization

	int CriticalOD_origin = 1;
	int CriticalOD_destination = 19;

	int DestZoneNumber; 
	for(DestZoneNumber=1; DestZoneNumber <= g_ODZoneNumberSize; DestZoneNumber++) // initialization...
	{
		if(g_ZoneNumber2NoVector[DestZoneNumber]<0)  // no such zone number
			continue;

		int DestZoneNo = g_ZoneNumber2NoVector[DestZoneNumber];

		PathArray[DestZoneNo].NumOfPaths = 0;
		PathArray[DestZoneNo].NumOfVehicles = 0;
		PathArray[DestZoneNo].DeviationNumOfVehicles = 0;
		PathArray[DestZoneNo].BestPathIndex = 0;

		PathArray[DestZoneNo].ClearPathElements ();

	}

	//step 2: construct path array
	// Scan all vehicles and construct path array for each destination
	for (int vi = 0; vi<g_TDOVehicleArray[g_ZoneMap[origin_zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.size(); vi++)
	{
		int VehicleID = g_TDOVehicleArray[g_ZoneMap[origin_zone].m_ZoneSequentialNo][AssignmentInterval].VehicleArray[vi];
		DTAVehicle* pVeh  = g_VehicleMap[VehicleID];
		ASSERT(pVeh!=NULL);

		if(g_ZoneNumber2NoVector[pVeh->m_DestinationZoneID]<0)  // no such zone number
			continue;

		if(iteration >=1 && pVeh->m_bComplete == false)  // do not count incomplet trips/paths: ideally we want to complete all vehicle trips before ODME
			continue; 

		int VehicleDestZoneNumber = pVeh->m_DestinationZoneID;
		float TripTime = pVeh->m_TripTime;
		int NodeSum = pVeh->m_NodeNumberSum;
		int NodeSize = pVeh->m_NodeSize;

		int VehicleDestZoneNo  = g_ZoneNumber2NoVector[VehicleDestZoneNumber];

		PathArray[VehicleDestZoneNo].NumOfVehicles++;  // count the number of vehicles from o to d at assignment time interval tau


		if(PathArray[VehicleDestZoneNo].NumOfPaths == 0) // the first path for VehicleDestZoneNo
		{
			PathArray[VehicleDestZoneNo].AddPathElement ();  // 0 index
			PathArray[VehicleDestZoneNo].AddPathElement ();  // 1 index

			PathArray[VehicleDestZoneNo].NumOfPaths++;
			PathArray[VehicleDestZoneNo].NumOfVehsOnEachPath[PathArray[VehicleDestZoneNo].NumOfPaths]++;
			PathArray[VehicleDestZoneNo].PathNodeSums[PathArray[VehicleDestZoneNo].NumOfPaths] = NodeSum;
			PathArray[VehicleDestZoneNo].AvgPathTimes[PathArray[VehicleDestZoneNo].NumOfPaths] = TripTime;
			// obtain path link sequence from vehicle link sequence
			for(int i = 0; i< NodeSize-1; i++)
			{
				PathArray[VehicleDestZoneNo].PathLinkSequences[PathArray[VehicleDestZoneNo].NumOfPaths].LinkNoVector.push_back (pVeh->m_LinkAry[i].LinkNo); 

			}
			PathArray[VehicleDestZoneNo].PathSize[PathArray[VehicleDestZoneNo].NumOfPaths] = NodeSize;
		}
		else{
			int PathIndex = 0;
			for(int p=1; p<=PathArray[VehicleDestZoneNo].NumOfPaths; p++)
			{
				if(NodeSum == PathArray[VehicleDestZoneNo].PathNodeSums[p])
				{
					PathIndex = p; // this veh uses the p-th in the set
					break;
				}
			}


			if(PathIndex == 0) // a new path is found
			{
			
				PathArray[VehicleDestZoneNo].AddPathElement (); 

				PathArray[VehicleDestZoneNo].NumOfPaths++;

				PathArray[VehicleDestZoneNo].NumOfVehsOnEachPath[PathArray[VehicleDestZoneNo].NumOfPaths]++;
				PathArray[VehicleDestZoneNo].PathNodeSums[PathArray[VehicleDestZoneNo].NumOfPaths] = NodeSum;
				PathArray[VehicleDestZoneNo].AvgPathTimes[PathArray[VehicleDestZoneNo].NumOfPaths] = TripTime;

							// obtain path link sequence from vehicle link sequence
				PathArray[VehicleDestZoneNo].PathLinkSequences[PathArray[VehicleDestZoneNo].NumOfPaths].LinkNoVector.clear ();
				for(int i = 0; i< NodeSize-1; i++)
				{
					PathArray[VehicleDestZoneNo].PathLinkSequences[PathArray[VehicleDestZoneNo].NumOfPaths].LinkNoVector.push_back(pVeh->m_LinkAry[i].LinkNo); 
				}
				PathArray[VehicleDestZoneNo].PathSize[PathArray[VehicleDestZoneNo].NumOfPaths] = NodeSize;
			}
			else{ // an existing path found
				PathArray[VehicleDestZoneNo].AvgPathTimes[PathIndex] = (PathArray[VehicleDestZoneNo].AvgPathTimes[PathIndex] * PathArray[VehicleDestZoneNo].NumOfVehsOnEachPath[PathIndex] + TripTime) / (PathArray[VehicleDestZoneNo].NumOfVehsOnEachPath[PathIndex] + 1);
				PathArray[VehicleDestZoneNo].NumOfVehsOnEachPath[PathIndex]++;				
			}
		}
	}

	// step 3: identify the best path for each destination
	// calculate the path gap

	for(DestZoneNumber=1; DestZoneNumber <= g_ODZoneNumberSize; DestZoneNumber++)
	{

		if(g_ZoneNumber2NoVector[DestZoneNumber]<0)  // no such zone number
			continue;


		int DestZoneNo = g_ZoneNumber2NoVector[DestZoneNumber];


		if(PathArray[DestZoneNo].NumOfPaths > 0)
		{
			int BestPath = 0;
			float LeastTime = 999999.0;

			int p;
			for(p=1; p<=PathArray[DestZoneNo].NumOfPaths; p++)
			{
				if(PathArray[DestZoneNo].AvgPathTimes[p] < LeastTime)
				{
					LeastTime = PathArray[DestZoneNo].AvgPathTimes[p];
					BestPath = p;
				}
			}
			PathArray[DestZoneNo].BestPathIndex = BestPath;
			PathArray[DestZoneNo].LeastTravelTime = LeastTime;


			for(p=1; p<=PathArray[DestZoneNo].NumOfPaths; p++)
			{

				PathArray[DestZoneNo].AvgPathGap[p] = PathArray[DestZoneNo].AvgPathTimes[p] - LeastTime;

#pragma omp critical
				{
					g_CurrentGapValue += PathArray[DestZoneNo].NumOfVehsOnEachPath[p] * PathArray[DestZoneNo].AvgPathGap[p];
					g_CurrentRelativeGapValue += PathArray[DestZoneNo].NumOfVehsOnEachPath[p] * (PathArray[DestZoneNo].AvgPathGap[p] - max(0.1, PathArray[DestZoneNo].AvgPathTimes[p]));


					if (origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
					{
						g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " path " << p << " gap: " << PathArray[DestZoneNo].AvgPathGap[p] << endl;
					}
				}
				// we calculate the gap for the previous iteraiton, for each path, path travel time gap * path flow
				// comment out for omp				g_CurrentGapValue += (PathArray[DestZoneNo].AvgPathGap[p]* PathArray[DestZoneNo].NumOfVehsOnEachPath[p]);

			}

		}
	}

	// step 4: update OD demand flow 
	for(DestZoneNumber=1; DestZoneNumber <= g_ODZoneNumberSize; DestZoneNumber++)  //  for each OD pair
	{

		if(g_ZoneNumber2NoVector[DestZoneNumber]<0)  // no such Zone ID
			continue;
		int DestZoneNo = g_ZoneNumber2NoVector[DestZoneNumber];

		// step 4.1: calculate the vehicle size deviation for  each O D tau pair

		// if demand has not been assigned to this destination origin_zone yet,skip

		float hist_demand = g_SystemDemand.GetValue (origin_zone, DestZoneNumber, AssignmentInterval);

		if(hist_demand < 0.001)  // only perform OD estimation when hist_demand  is positive
			continue; 

		PathArray[DestZoneNo].DeviationNumOfVehicles = PathArray[DestZoneNo].NumOfVehicles - hist_demand;

		if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
		{
			g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber <<  " with "<< PathArray[DestZoneNo].NumOfPaths << " paths; Hist Demand =" << hist_demand << "; simu Demand = " << PathArray[DestZoneNo].NumOfVehicles  << ";Dev = " << PathArray[DestZoneNo].DeviationNumOfVehicles <<endl; 
		}


		for(int p=1; p<= PathArray[DestZoneNo].NumOfPaths; p++)  // recall that path index start from 1 in this module
		{
			// step 4.2: assign the demand deviation as the initial value for path marginal 
			PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] = 0;

			// step 4.3: walk through the link sequence to accumlate the link marginals

			float ArrivalTime = g_AssignmentIntervalStartTimeInMin[AssignmentInterval];
			float ArrivalTimeBasedonTravelTime = ArrivalTime;

			if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination && ArrivalTime>=990)
			{
				g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << "arrival time = "<< ArrivalTime << endl;
			}

			int number_of_links_with_extremely_heavy_congestion = 0;
			int l = 0;
			while(l>=0 && l< PathArray[DestZoneNo].PathSize[p]-1)  // for each link along the path
			{

				int MovementDestinationNode = -1;

				int LinkID = PathArray[DestZoneNo].PathLinkSequences[p].LinkNoVector [l];

				DTALink* pLink = g_LinkVector[LinkID];
				
				if(l< PathArray[DestZoneNo].PathSize[p]-2)  // mark destination node number for movement
				{
				int LinkID2 = PathArray[DestZoneNo].PathLinkSequences[p].LinkNoVector [l+1];
					DTALink* pLink2 = g_LinkVector[LinkID2];
					MovementDestinationNode = pLink->m_ToNodeNumber;

				}

				if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination && ArrivalTime>=990)
				{
					g_EstimationLogFile << "Link Index : " << l << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber << ", arrival time = "<< ArrivalTime << endl;
				}



				int ArrivalTime_int = (int)(ArrivalTime);

				if (ArrivalTime_int >= g_PlanningHorizon)  // time of interest exceeds the simulation horizon
					break;



				if(l==0) // first link: check loading buffer
				{

					// if loading buffer is congested at the current time, we need to move the the end of the congestion period (to start over)
					if( g_TrafficFlowModelFlag != tfm_BPR && pLink->m_LinkMOEAry[ArrivalTime_int].LoadingBuffer_QueueLength >=1)
					{
						ArrivalTime  = pLink->m_LinkMOEAry[ArrivalTime_int].LoadingBuffer_EndTimeOfCongestion;
						ArrivalTime_int = (int)(ArrivalTime);
					}
				}

				int arrival_time_in_hour = int(ArrivalTimeBasedonTravelTime/60);
				if (arrival_time_in_hour <= 24 && pLink->SimultedHourlySpeed[arrival_time_in_hour] < pLink->m_SpeedLimit *0.15) // 15% of speed limit
					number_of_links_with_extremely_heavy_congestion++;

				ArrivalTimeBasedonTravelTime += pLink->GetTravelTimeByMin(0, ArrivalTimeBasedonTravelTime, g_FindAssignmentIntervalLengthInMinFromTime(ArrivalTimeBasedonTravelTime), g_TrafficFlowModelFlag);  // update travel time 

				if(ArrivalTime >= pLink->m_LinkMOEAry.size() || ArrivalTime >= g_PlanningHorizon) // time of interest exceeds the simulation horizon
					break;

				if (pLink->m_FromNodeNumber == 10)
				{
					TRACE("");

				}
				if( g_TrafficFlowModelFlag == tfm_BPR || pLink->m_LinkMOEAry [(int)(ArrivalTime)].TrafficStateCode == 0) 					// if Arrival at Free-flow condition or using BPR function
				{



					if(g_GetSequentialEstimationTimePeriod(iteration,ArrivalTime) && pLink->ContainFlowCount(ArrivalTime)) // with flow measurement
					{
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]+= pLink->GetDeviationOfFlowCount(ArrivalTime);

						if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						{
							g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber 
								<< " Obs link flow: "<< pLink->GetObsFlowCount (ArrivalTime) <<", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount (ArrivalTime) << ", Measurement Error: " 
								<<  pLink->GetDeviationOfFlowCount(ArrivalTime)  << 
								", " <<   pLink->GetDeviationOfFlowCount(ArrivalTime)/max(1, pLink->GetObsFlowCount (ArrivalTime))*100 << " %, cumulative error: " <<  PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						}
					}


					if (g_GetSequentialEstimationTimePeriod(iteration, ArrivalTime) && pLink->ContainTravelTimeObservation(ArrivalTime)) // with travel time measurement
					{
						float marginal_dv = pLink->GetDeviationOfTravelTime(ArrivalTime);

						// flow rate gradient  // we assume the sensor is located upstream of the link, so we do not consider the departure flow rate increase at the downstream of the link
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]
							+= pLink->GetDeviationOfTravelTime(ArrivalTime)*g_ODEstimationWeightOnTravelTimeDeviationFreeflow;
						
						marginal_dv = PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p];

						TRACE("");
						//weight of 5: (0.5 miles/30mph*60 - 0.5miles/40mph*60) = 7.5, 100/7.5 = 13.3
						//prevent overadjustment
						//0.5 miles, avg link length

						//if (origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						//{
						//	g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber
						//		<< " Obs link flow: " << pLink->GetObsFlowCount(ArrivalTime) << ", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount(ArrivalTime) << ", Measurement Error: "
						//		<< pLink->GetDeviationOfFlowCount(ArrivalTime) <<
						//		", " << pLink->GetDeviationOfFlowCount(ArrivalTime) / max(1, pLink->GetObsFlowCount(ArrivalTime)) * 100 << " %, cumulative error: " << PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						//}

					}

					if(g_GetSequentialEstimationTimePeriod(iteration,ArrivalTime) && pLink->ContainMovementFlowCount(ArrivalTime,MovementDestinationNode)) // with flow measurement
					{
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]+= pLink->GetDeviationOfFlowCount(ArrivalTime,MovementDestinationNode);

						if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						{
							g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber 
								<< " Obs link flow: "<< pLink->GetObsFlowCount (ArrivalTime) <<", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount (ArrivalTime) << ", Measurement Error: " 
								<<  pLink->GetDeviationOfFlowCount(ArrivalTime)  << 
								", " <<   pLink->GetDeviationOfFlowCount(ArrivalTime)/max(1, pLink->GetObsFlowCount (ArrivalTime))*100 << " %, cumulative error: " <<  PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						}
					}

					if(g_TrafficFlowModelFlag != tfm_BPR)
					{
					ArrivalTime+= pLink->m_FreeFlowTravelTime ;
					
					}

					if (pLink->ContainObsDensity(ArrivalTime))  // to be fined tuned later  // free frlow
					{
						int temp_value = pLink->GetDeviationOfNumberOfVehicles(ArrivalTime, ArrivalTime);
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] += 
							pLink->GetDeviationOfNumberOfVehicles(ArrivalTime, ArrivalTime);
					}

					l+= 1;  // move forward to the next link

				}else 		// if ArrivalTime is partially congested or fully congested,
				{


					TRACE("traffic state = %d \n",pLink->m_LinkMOEAry [ArrivalTime_int].TrafficStateCode);
					int timestamp_end_of_congestion = min(g_DemandLoadingEndTimeInMin, pLink->m_LinkMOEAry[(int)(ArrivalTime)].EndTimeOfPartialCongestion);  // move to the end of congestion


					if(pLink->ContainFlowCount(ArrivalTime)) // with flow measurement
					{
						// flow rate gradient  // we assume the sensor is located upstream of the link, so we do not consider the departure flow rate increase at the downstream of the link
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]+= 
							pLink->GetDeviationOfFlowCount(ArrivalTime);

						//weight of 5: (0.5 miles/30mph*60 - 0.5miles/40mph*60) = 7.5, 100/7.5 = 13.3
						//prevent overadjustment
						//0.5 miles, avg link length

						if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						{
							g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber 
								<< " Obs link flow: "<< pLink->GetObsFlowCount (ArrivalTime) <<", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount (ArrivalTime) << ", Measurement Error: " 
								<<  pLink->GetDeviationOfFlowCount(ArrivalTime)  << 
								", " <<   pLink->GetDeviationOfFlowCount(ArrivalTime)/max(1, pLink->GetObsFlowCount (ArrivalTime))*100 
								<< " %, cumulative error: " <<  PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						}

					}
					if (pLink->ContainTravelTimeObservation(ArrivalTime)) // with travel time measurement
					{
						// flow rate gradient  // we assume the sensor is located upstream of the link, so we do not consider the departure flow rate increase at the downstream of the link
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] 
							+= pLink->GetDeviationOfTravelTime(ArrivalTime)*g_ODEstimationWeightOnTravelTimeDeviationCongested*max(1, (timestamp_end_of_congestion - ArrivalTime));

						//weight of 5: (0.5 miles/30mph*60 - 0.5miles/40mph*60) = 7.5, 100/7.5 = 13.3
						//prevent overadjustment
						//0.5 miles, avg link length

						//if (origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						//{
						//	g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber
						//		<< " Obs link flow: " << pLink->GetObsFlowCount(ArrivalTime) << ", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount(ArrivalTime) << ", Measurement Error: "
						//		<< pLink->GetDeviationOfFlowCount(ArrivalTime) <<
						//		", " << pLink->GetDeviationOfFlowCount(ArrivalTime) / max(1, pLink->GetObsFlowCount(ArrivalTime)) * 100 << " %, cumulative error: " << PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						//}

					}

					if(pLink->ContainMovementFlowCount(ArrivalTime,MovementDestinationNode)) // with movement flow measurement
					{
						// flow rate gradient  // we assume the sensor is located upstream of the link, so we do not consider the departure flow rate increase at the downstream of the link
						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]+= 
							pLink->GetDeviationOfFlowCount(ArrivalTime,MovementDestinationNode);


						//weight of 5: (0.5 miles/30mph*60 - 0.5miles/40mph*60) = 7.5, 100/7.5 = 13.3
						//prevent overadjustment
						//0.5 miles, avg link length

						if(origin_zone == CriticalOD_origin && DestZoneNumber == CriticalOD_destination)
						{
							g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneNumber << " passing link " << pLink->m_FromNodeNumber << "->" << pLink->m_ToNodeNumber 
								<< " Obs link flow: "<< pLink->GetObsFlowCount (ArrivalTime) <<", Simulated link flow: " << pLink->GetSimulatedFlowCountWithObsCount (ArrivalTime) << ", Measurement Error: " 
								<<  pLink->GetDeviationOfFlowCount(ArrivalTime)  << 
								", " <<   pLink->GetDeviationOfFlowCount(ArrivalTime)/max(1, pLink->GetObsFlowCount (ArrivalTime))*100 << " %, cumulative error: " <<  PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << endl;

						}

					}

					if(pLink->ContainObsDensity (ArrivalTime))  // to be fined tuned later
					{

						PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p]+= 
							pLink->GetDeviationOfNumberOfVehicles(ArrivalTime,timestamp_end_of_congestion)*max(1,timestamp_end_of_congestion-ArrivalTime);
					}

					// move to the end of congestion
					int OldArrivalTime = (int)(ArrivalTime);
					ArrivalTime = pLink->m_LinkMOEAry [(int)(ArrivalTime)].EndTimeOfPartialCongestion ;  
					int NewArrivalTime = (int)(ArrivalTime);

					// remark: EndTimeOfPartialCongestion is like the end time of the current episo. 
					// arrval time is the arrival time at the exit queue


					if(pLink->m_LinkMOEAry [(int)(ArrivalTime)].TrafficStateCode == 0) // freeflow
					{
						l+= 1;  // move forward to the next link
					}
					else // fully congested
					{
						// keep the same link, move to the next arrival time, end of the event duration

						TRACE("fully congested, move to new time");
						ArrivalTime+=1;

					}

				}


				//				TRACE("link index l = %d, arrival time = %5.1f \n",l,ArrivalTime);
				//				g_EstimationLogFile <<  "link index l = " << l << "arrival time = "<< ArrivalTime << endl;



			}

			// calculate the total demand deviation statistics only for the first path
			//if(p==1) 
			//{
			//	g_TotalDemandDeviation += PathArray[DestZoneNo].DeviationNumOfVehicles;
			//			if(origin_zone == CriticalOD_origin && DestZoneID == CriticalOD_destination)
			//			{
			//				g_EstimationLogFile << "Critical OD demand " << origin_zone << " -> " << DestZoneID <<
			//					" TotalDemandDeviation = " <<  PathArray[DestZoneNo].DeviationNumOfVehicles << endl;
			//			}
			//}

			//g_TotalMeasurementDeviation += PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p];

			// this is a very important path flow adjustment equation;

			float marginal = PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p];

			float FlowAdjustment = 
				g_ODEstimation_WeightOnHistODDemand * PathArray[DestZoneNo].DeviationNumOfVehicles /* gradient wrt target demand*/
				+ PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] /* gradient wrt measurements (link incoming flow, density and travel time*/
			+ g_ODEstimation_WeightOnUEGap*PathArray[DestZoneNo].AvgPathGap[p];


			if(FlowAdjustment<0.0001f) // negative flow (simu - observed) : simu < observed: we wanted to increase the flow at the next iteration
			{
				if(number_of_links_with_extremely_heavy_congestion >=2)
				{
					FlowAdjustment = 0; // reset the adjustment volume to zero if there are more than 2  heavily congested links
					//					cout << "ODME: reset adjustment flow to 0: avoid heavy congestion." << endl;
				}
			}


			// add constraints based on the deviation from historical demand 


			int abs_threashold = 0;
			int bound_for_applying_target_demand_constraint = 5;

			bool bConsiderBoundForAdjustment = false;

			if (bConsiderBoundForAdjustment == true)  // not used, this is old code
			{
			

			if ( hist_demand >= bound_for_applying_target_demand_constraint)  // ;OD demand with sufficiently large volumes
			{

				//upper bound and lower bound

				if (FlowAdjustment < hist_demand*(-1))
					FlowAdjustment = hist_demand*(-1);

				if (FlowAdjustment > hist_demand)
					FlowAdjustment = hist_demand;



				if(origin_zone == 1 && DestZoneNumber == 19 )
				{
					TRACE("");
				}
				float existing_demand = PathArray[DestZoneNo].NumOfVehicles;
				float new_demand  =  existing_demand - g_ODEstimation_StepSize*FlowAdjustment;

			float upper_bound = hist_demand * (1+ g_ODEstimation_max_ratio_deviation_wrt_hist_demand);
				float lower_bound = max(0, hist_demand * (1- g_ODEstimation_max_ratio_deviation_wrt_hist_demand));

				if( new_demand>upper_bound  + abs_threashold && FlowAdjustment < 0) 
				{ 
					FlowAdjustment  = -(upper_bound - existing_demand)/ g_ODEstimation_StepSize; 
				}

				if (  new_demand < lower_bound - abs_threashold && FlowAdjustment > 0) 
				{
					FlowAdjustment  = (existing_demand - lower_bound)/ g_ODEstimation_StepSize; 

				}
			}
			}
			

			float upper_bound_of_path_flow_adjustment = hist_demand * g_ODEstimation_max_ratio_deviation_wrt_hist_demand;

			int sign_of_flow_adjustment  = 1;

			if(FlowAdjustment < 0)
				sign_of_flow_adjustment = -1;

			// restrict the flow adjustment per iteration, no more than 25% of previous path flow at the previous iteration
			if( fabs(FlowAdjustment) > upper_bound_of_path_flow_adjustment)
				FlowAdjustment = upper_bound_of_path_flow_adjustment*sign_of_flow_adjustment;

			// we do not want to change path flow too much from one iteration to another iteration
			// if path flow is reset to zero, then no vehicles will be simulated along this path, and then we lose this path in the remaining iterations.
			// we just want to keep the set of efficient paths stable

			PathArray[DestZoneNo].NewNumberOfVehicles[p] = 	PathArray[DestZoneNo].NumOfVehsOnEachPath[p]   // this is the existing path flow volume
			- g_ODEstimation_StepSize*FlowAdjustment; /*gradient wrt gap function*/


			/*			g_EstimationLogFile << "OD " << origin_zone << " -> " << DestZoneID << " path =" << p << " @ "<< AssignmentInterval*g_AggregationTimetInterval << ": Measurement Dev=" << PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << 
			"; Demand Dev=" << PathArray[DestZoneNo].DeviationNumOfVehicles <<
			"; Gap Dev "<< g_ODEstimation_WeightOnUEGap*PathArray[DestZoneNo].AvgPathGap[p]  <<endl; 
			*/

			PathArrayForEachODTK element;
			int PathIndex  = g_TDOVehicleArray[g_ZoneMap[origin_zone].m_ZoneSequentialNo][AssignmentInterval].m_ODTKPathVector.size();


			element.Setup(PathIndex, origin_zone, DestZoneNumber, 1, g_AssignmentIntervalStartTimeInMin[AssignmentInterval], g_AssignmentIntervalEndTimeInMin[AssignmentInterval],

			PathArray[DestZoneNo].PathSize[p]-1, PathArray[DestZoneNo].PathLinkSequences[p].LinkNoVector ,PathArray[DestZoneNo].NewNumberOfVehicles[p],PathArray[DestZoneNo].PathNodeSums[p],AssignmentInterval );

			// copy flow adjustment for each O, D, tau and p
			element.FlowAdjustmentGradient = FlowAdjustment;
			
			//if (element.m_OriginZoneID == 109 && element.m_DestinationZoneID == 139 && false)
			//{

			//	cout << "setup OD:" << element.m_OriginZoneID << "->" << element.m_DestinationZoneID << ": "
			//		" AssignmentInterval = " << AssignmentInterval << " p = " << p << " hist_demand =  " << hist_demand << ": NumOfVehsOnEachPath[p] = " << PathArray[DestZoneNo].NumOfVehsOnEachPath[p]
			//		<< "m_VehicleSize = "
			//		<< element.m_VehicleSize << ":  FlowAdjustment = " << FlowAdjustment <<
			//		" NewNumberOfVehicles = " << PathArray[DestZoneNo].NewNumberOfVehicles[p] << " g_ODEstimation_StepSize = " << g_ODEstimation_StepSize << endl;
			//	getchar();
			//}

			g_SystemDemand.AddUpdatedValue(origin_zone,DestZoneNumber,AssignmentInterval, PathArray[DestZoneNo].NewNumberOfVehicles[p]); // to store the initial table as hist database
			g_SystemDemand.UpdatedDemandParameters(origin_zone, DestZoneNumber, AssignmentInterval, PathArray[DestZoneNo].NewNumberOfVehicles[p], element.FlowAdjustmentGradient); // to store the initial table as hist database


			
			g_TDOVehicleArray[g_ZoneMap[origin_zone].m_ZoneSequentialNo][AssignmentInterval].m_ODTKPathVector.push_back(element);


			//			g_AssignmentLogFile << "OED: O: " << origin_zone << ", D:" << DestZoneID << "Demand Dev: " << PathArray[DestZoneNo].DeviationNumOfVehicles << ", Path:" << p << ", marginal: " <<  PathArray[DestZoneNo].MeasurementDeviationPathMarginal[p] << ", AvgTT: "<< PathArray[DestZoneNo].AvgPathTimes[p]<< ", Gap : "<< PathArray[DestZoneNo].AvgPathGap[p]<<  ", VehicleSize:" << PathArray[DestZoneNo].NewNumberOfVehicles[p] << ", flow adjustment" << FlowAdjustment << endl;

			// for path p

		}

	}  //  for each OD pair

}



void DTANetworkForSP::ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval_ODEstimation(int origin_zone, int AssignmentInterval, int iteration)
// for vehicles starting from departure_time_begin to departure_time_end, assign them to shortest path using a proportion according to MSA or graident-based algorithms
{

	int PathNodeList[MAX_NODE_SIZE_IN_A_PATH]={-1};

	if(g_ZoneNumber2NoVector[origin_zone]<0)  // no such zone number
		return;
	
	ConstructPathArrayForEachODT_ODEstimation(iteration,m_PathArray, origin_zone, AssignmentInterval);

}

void g_GenerateVehicleData_ODEstimation()
{

	//	g_EstimationLogFile << "g_GenerateVehicleData_ODEstimation "  <<endl; 

	g_FreeMemoryForVehicleVector();

	cout << "Converting OD demand flow to vehicles..."<< endl;
	

	for (int di = 0; di < g_NumberOfSPCalculationPeriods; di++)  // for time index first, as we need to sort the vehicle vector according to departure time intervals, by doing so, we can speed up the sorting process
	{
		for(int z = 0; z <= g_ODZoneNumberSize; z++)  // for each zone 
		{

			if(g_ZoneNumber2NoVector[z]<0)  // no such Zone ID
				continue;
			
			g_TDOVehicleArray[g_ZoneMap[z].m_ZoneSequentialNo][di].VehicleArray.clear ();


			for(int pi = 0; pi< g_TDOVehicleArray[g_ZoneMap[z].m_ZoneSequentialNo][di].m_ODTKPathVector .size(); pi++)  // for each path
			{
				PathArrayForEachODTK element = g_TDOVehicleArray[g_ZoneMap[z].m_ZoneSequentialNo][di].m_ODTKPathVector[pi];
				//if (element.m_OriginZoneID == 109 && element.m_DestinationZoneID == 139 && false)
				//{

				//	cout << "OD:" << element.m_OriginZoneID << "->" << element.m_DestinationZoneID << ":" << element.m_VehicleSize << endl;
				//	getchar();
				//}
				// temp demand * updated path probability
				

				int OriginZoneNo = g_ZoneMap[element.m_OriginZoneID].m_ZoneSequentialNo;
				int DestinationZoneNo = g_ZoneMap[element.m_DestinationZoneID].m_ZoneSequentialNo;

				float adjustment_ratio = 1.0; 
				
				adjustment_ratio = g_SystemDemand.m_alpha[OriginZoneNo] / (max(0.1, g_SystemDemand.m_alpha_last_value[OriginZoneNo]));


				if (g_ODEstimation_alpha_based_adjustment == 1)  //enable alpha based adjustment mode
				{
				  element.m_VehicleSize = element.m_VehicleSize *adjustment_ratio;
				}
#pragma omp critical

				{
					_proxy_ODME_log(1, 0, "element.m_VehicleSize = %f with adjustment ratio = %f, temp demand = %f, old demand = %f\n",

						element.m_VehicleSize, adjustment_ratio,

						g_SystemDemand.m_TempDemand[OriginZoneNo][DestinationZoneNo][element.m_DepartureTimeIndex],
						g_SystemDemand.m_UpdatedDemand[OriginZoneNo][DestinationZoneNo][element.m_DepartureTimeIndex]
						);
				}


				CreateVehicles(element.m_OriginZoneID,element.m_DestinationZoneID ,
					element.m_VehicleSize ,
					element.m_DemandType,element.m_starting_time_in_min,element.m_ending_time_in_min,
					element.m_PathIndex,false, element.m_DepartureTimeIndex );
			}

		}
	}

	cout << g_simple_vector_vehicles.size() << " vehicles generated" << endl;



	// create vehicle heres...
	//		g_EstimationLogFile << " Converting demand flow to vehicles... "  <<endl; 


	cout << " Sort vehicles... "  <<endl; 
	std::sort(g_simple_vector_vehicles.begin(), g_simple_vector_vehicles.end());
	cout << " Sorting ends. Allocating memory for vehicles..."  <<endl; 

	std::vector<DTA_vhc_simple>::iterator kvhc =  g_simple_vector_vehicles.begin();

	DTAVehicle* pVehicle = 0;

	int i = 0;
	while(kvhc != g_simple_vector_vehicles.end())
	{
		if(kvhc->m_OriginZoneID!= kvhc->m_DestinationZoneID)    // only consider intra-origin_zone traffic
		{

			//try
			//{
			pVehicle = new DTAVehicle;
			if(pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			//}
			//catch (std::bad_alloc& exc)
			//{
			//	cout << "Insufficient memory...";
			//	getchar();
			//	exit(0);

			//}


			//if(i%10000==0)
			//cout << " Adding "<< i/10000<< "K vehicles... "  <<endl; 

			pVehicle->m_AgentID		= i;
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			pVehicle->m_OriginZoneID	= kvhc->m_OriginZoneID ;
			pVehicle->m_DestinationZoneID 	= kvhc->m_DestinationZoneID ;


			pVehicle->m_DepartureTime	= kvhc->m_DepartureTime;
			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_VOT = kvhc->m_VOT ;
			pVehicle->m_Age  = kvhc->m_Age;


			pVehicle->m_DemandType	= kvhc->m_DemandType;
			pVehicle->m_InformationType = kvhc->m_InformationType;
			pVehicle->m_DemandType 	= kvhc->m_DemandType ;
			pVehicle->m_VehicleType = kvhc->m_VehicleType;
			pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType-1].PCE;


			if( kvhc->m_PathIndex  >= g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][kvhc->m_DepartureTimeIndex ].m_ODTKPathVector.size())
			{
				cout << "kvhc->m_PathIndex  >= g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][kvhc->m_DepartureTimeIndex ].m_ODTKPathVector.size()" << endl;
				g_ProgramStop();
			}
			PathArrayForEachODTK element = g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][kvhc->m_DepartureTimeIndex ].m_ODTKPathVector[kvhc->m_PathIndex ];

			//				PathArrayForEachODTK element = g_ODTKPathVector[kvhc->m_PathIndex ];

			int NodeSize = element.m_LinkSize+1;
			pVehicle->m_NodeSize = NodeSize;
			pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];

			if(pVehicle->m_LinkAry==NULL)
			{
				cout << "Insufficient memory for allocating vehicle arrays!";
				g_ProgramStop();
			}

			pVehicle->m_NodeNumberSum =  element.m_NodeSum ;
			pVehicle->m_Distance =0;

			for(int j = 0; j< pVehicle->m_NodeSize-1; j++)
			{
				pVehicle->m_LinkAry[j].LinkNo = element.m_LinkNoArray[j];

				if(j==0)
				{
					pVehicle->m_OriginNodeID  = g_LinkVector[pVehicle->m_LinkAry [j].LinkNo] ->m_FromNodeID ;
				}

				if(j==pVehicle->m_NodeSize-2)
				{
					pVehicle->m_DestinationNodeID  = g_LinkVector[pVehicle->m_LinkAry [j].LinkNo] ->m_ToNodeID ;
				}

				pVehicle->m_Distance+= g_LinkVector[pVehicle->m_LinkAry [j].LinkNo] ->m_Length ;
			}
			g_VehicleVector.push_back(pVehicle);
			g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
			g_VehicleMap[i] = pVehicle;  // i is the vehicle id

			g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][kvhc->m_DepartureTimeIndex ].VehicleArray .push_back(i);


			i++;
		}
		kvhc++;

	}
	g_simple_vector_vehicles.clear ();

	for(int z = 0; z <=g_ODZoneNumberSize; z++)
	{
		if(g_ZoneNumber2NoVector[z]<0)
			continue;

		for (int di = 0; di < g_NumberOfSPCalculationPeriods; di++)
		{
			g_TDOVehicleArray[g_ZoneMap[z].m_ZoneSequentialNo][di].m_ODTKPathVector.clear();

		}
	}

	cout << " Memory allocation ends."  <<endl; 

}





void g_UpdateLinkMOEDeviation_ODEstimation(NetworkLoadingOutput& output, int Iteration)
{
	//MAPE Mean absolute percentage error 
	//RMSE  root mean sequared error

	//	g_EstimationLogFile << "--------------------------------- Iteration" << Iteration << " -----------------------------" << endl;
	float TotalMOESampleSize = 0;
	float TotalSpeedSampleSize = 0;
	float TotalFlowError = 0;
	float TotalFlowSequaredError = 0;

	float TotalMOEPercentageError = 0;
	float TotalMOEAbsError = 0;

	float TotalSpeedPercentageError = 0;
	float TotalSpeedAbsError = 0;

	float TotalDensityError = 0;

	int number_of_sensors = 0;

	std::vector <SensorDataPoint> SensorDataVector_link_count;
	std::vector <SensorDataPoint> SensorDataVector_link_speed;
	std::vector <SensorDataPoint> SensorDataVector_lane_density;



	g_EstimationLogFile << "Iteration," << Iteration << "," << "direction" << ","<< "link name,link_type," <<  "Link " << "from node" << ",->," << "to node" 
		<< ",time "<<  "start time in min" << "->" << "end time in min" <<  ",,"<< "observed link count" << "," << "simulated link count" <<", Error:, " << "Simulated flow count -  Obs flow count" << 
		"," << "Abosolute Percentage Error " << ",,Lane Flow Error /h=,Capacity ,obs voc,simu VOC," <<
		"observed lane density,simulated lane density,Error:,Abosolute Percentage Error "<< endl;


	for(unsigned li = 0; li< g_LinkVector.size(); li++)
	{	
		DTALink* pLink = g_LinkVector[li];
				// compare the error statistics
			for(unsigned int i = 0; i< pLink->m_LinkMeasurementAry.size(); i++)
			{

				if(pLink->m_FromNodeNumber == 56154)
				{

					TRACE("");
				}

				if (pLink->m_LinkMeasurementAry[i].StartTime  < g_ValidationDataStartTimeInMin)
					g_ValidationDataStartTimeInMin = pLink->m_LinkMeasurementAry[i].StartTime;

				if( pLink->m_LinkMeasurementAry[i].EndTime > g_ValidationDataEndTimeInMin)
					g_ValidationDataEndTimeInMin = pLink->m_LinkMeasurementAry[i].EndTime;

				// CumulativeArrivalCount is available after this time interval, so we calculate statistics on time time_index-1

				int EndTime = min(pLink->m_LinkMeasurementAry[i].EndTime,pLink->m_LinkMOEAry.size()-1);
				int StartTime = min(pLink->m_LinkMeasurementAry[i].StartTime,pLink->m_LinkMOEAry.size()-1);

				int SimulatedInFlowCount = pLink->m_LinkMOEAry[EndTime].CumulativeArrivalCount - pLink->m_LinkMOEAry[ StartTime ].CumulativeArrivalCount; 

				if(SimulatedInFlowCount>=0)
				{
					pLink->m_LinkMeasurementAry[i].SimuFlowCount = SimulatedInFlowCount;

					int ObsFlowCount =  pLink->m_LinkMeasurementAry[i].ObsFlowCount;

					SensorDataPoint element;
					element.x = ObsFlowCount;
					element.y = SimulatedInFlowCount;
					SensorDataVector_link_count.push_back (element);

					if (pLink->m_LinkMeasurementAry[i].ObsTravelTime >= 0.01)
					{
						element.x = pLink->m_Length / pLink->m_LinkMeasurementAry[i].ObsTravelTime * 60;  // mph
						element.y = pLink->m_Length / pLink->GetSimulatedTravelTime(StartTime) * 60;
						
						SensorDataVector_link_speed.push_back (element);
					}



					if(pLink->m_LinkMeasurementAry[i].ObsDensity >=0.01)
					{

					SensorDataPoint element_density;
					element_density.x = pLink->m_LinkMeasurementAry[i].ObsDensity;

					float simu_density = pLink->GetSimulateAvgDensityPerLane(pLink->m_LinkMeasurementAry[i].StartTime, pLink->m_LinkMeasurementAry[i].EndTime) ;

					element_density.y = simu_density;
					SensorDataVector_lane_density.push_back(element_density);
					}

					if( pLink->m_LinkMeasurementAry[i].MovementDestinationNode >=1)  //movement count measurement 
					{
						int SimulatedMovementCount = 0;

					int demand_type =  0;
					int vehicle_type = 0;
					int information_type = 0;
					int from_node_id = pLink->m_FromNodeNumber ;
					int mid_node_id	= pLink->m_ToNodeNumber ;
					int to_node_id	= pLink->m_LinkMeasurementAry[i].MovementDestinationNode ;
					int origin_zone_id	=0;
					int destination_zone_id	=0;
					int departure_starting_time	 = 0;
					int departure_ending_time= 1440;
					int entrance_starting_time	= pLink->m_LinkMeasurementAry[i].StartTime;
					int entrance_ending_time = pLink->m_LinkMeasurementAry[i].EndTime;
				

					float AvgTripTime, AvgDistance, AvgSpeed, AvgCost;
					EmissionStatisticsData emission_data;
					LinkMOEStatisticsData  link_data;
					SimulatedMovementCount = g_OutputSimulationMOESummary(AvgTripTime,AvgDistance, AvgSpeed,AvgCost, emission_data, link_data,
						demand_type,vehicle_type, information_type, origin_zone_id,destination_zone_id,
						from_node_id, mid_node_id, to_node_id,	
						departure_starting_time,departure_ending_time,entrance_starting_time,entrance_ending_time );

						pLink->m_LinkMeasurementAry[i].DeviationOfFlowCount = SimulatedMovementCount -  pLink->m_LinkMeasurementAry[i].ObsFlowCount ;
					}
					else if (ObsFlowCount >=1)	// link count measurment  
					{
						pLink->m_LinkMeasurementAry[i].DeviationOfFlowCount = SimulatedInFlowCount -  pLink->m_LinkMeasurementAry[i].ObsFlowCount ;
					}

					// travel time measurment
					if(pLink->m_LinkMeasurementAry[i].ObsTravelTime>=0.01)
					{
						pLink->m_LinkMeasurementAry[i].DeviationOfTravelTime  = pLink->GetSimulatedTravelTime(StartTime) - pLink->m_LinkMeasurementAry[i].ObsTravelTime; 
						
						float simu_speed = pLink->m_Length / max(0.01, pLink->GetSimulatedTravelTime(StartTime)) * 60;
						float obs_speed = pLink->m_Length / max(0.01, pLink->m_LinkMeasurementAry[i].ObsTravelTime) * 60;

						float SpeedError = simu_speed - obs_speed;
						float AbosolutePercentageError = SpeedError / max(1, obs_speed);
						g_EstimationLogFile << "Iteration," << Iteration << "," << pLink->m_LinkMeasurementAry[i].name << "," << pLink->m_LinkMeasurementAry[i].direction << "," << pLink->m_LinkTypeName << ",Link " << pLink->m_FromNodeNumber << ",->," << pLink->m_ToNodeNumber
							<< ",time " << g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].StartTime) << "->" << g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].EndTime) <<

							",Observed and simulated link speed," << obs_speed << "," << simu_speed << ", Error:, " << SpeedError << endl;

						TotalSpeedPercentageError += AbosolutePercentageError;
						TotalSpeedAbsError += fabs(SpeedError);
						TotalSpeedSampleSize++;
					}


					//density measurement
					if(pLink->m_LinkMeasurementAry[i].ObsDensity >=0.01)
					{
						pLink->m_LinkMeasurementAry[i].DeviationOfNumberOfVehicles  = pLink->GetSimulatedNumberOfVehicles(StartTime) - pLink->m_LinkMeasurementAry[i].ObsDensity * pLink->m_Length * pLink->m_OutflowNumLanes ; 
					}

					if(ObsFlowCount >= 1)  // flow count
					{
						int time_interval = pLink->m_LinkMeasurementAry[i].EndTime - pLink->m_LinkMeasurementAry[i].StartTime;
						float AbosolutePercentageError = abs((SimulatedInFlowCount -  ObsFlowCount)*1.0f/max(1,ObsFlowCount)*100);
						float LaneFlowError = (SimulatedInFlowCount -  ObsFlowCount)*60.0f/
							max(1,time_interval)/pLink->m_OutflowNumLanes;												

						float obs_v_over_c_ratio = 0;
						float simu_over_c_ratio = 0;
						float link_capacity = pLink->GetNumberOfLanes ()* pLink->m_LaneCapacity;

						if(link_capacity>1)
						{
							obs_v_over_c_ratio  = ObsFlowCount*60.0f/max(1,time_interval) /max(1,link_capacity);
							simu_over_c_ratio = SimulatedInFlowCount*60.0f/max(1,time_interval) /max(1,link_capacity);
						}
						g_EstimationLogFile << "Iteration," << Iteration << "," << pLink->m_LinkMeasurementAry[i].name << "," << pLink->m_LinkMeasurementAry[i].direction << "," << pLink->m_LinkTypeName << ",Link " << pLink->m_FromNodeNumber << ",->," << pLink->m_ToNodeNumber
							<< ",time " << g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].StartTime) << "->" << g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].EndTime) << ",Observed and simulated link count," << ObsFlowCount << "," << SimulatedInFlowCount << ", Error:, " << SimulatedInFlowCount - ObsFlowCount <<
							"," << AbosolutePercentageError << " %" << ",Lane Flow Error /h=, " << LaneFlowError << "," << pLink->GetNumberOfLanes()* pLink->m_LaneCapacity << "," << obs_v_over_c_ratio << "," << simu_over_c_ratio << endl;

						TotalMOEPercentageError +=AbosolutePercentageError ; 
						TotalMOEAbsError += fabs(LaneFlowError) ;
						TotalMOESampleSize ++;

					}
					if(pLink->m_LinkMeasurementAry[i].ObsDensity >= 1)  // with density data
					{

						float obs_density = pLink->m_LinkMeasurementAry[i].ObsDensity;
						float simu_density = pLink->GetSimulateAvgDensityPerLane(pLink->m_LinkMeasurementAry[i].StartTime, pLink->m_LinkMeasurementAry[i].EndTime) ;

						g_EstimationLogFile << obs_density << ",";
						g_EstimationLogFile << simu_density << ",";
						g_EstimationLogFile << obs_density - simu_density << ",";
						g_EstimationLogFile << ( obs_density - simu_density)/max(1,obs_density)*100  << ",";
					}



			}
		}
	}	

	output.ODME_result_link_count = LeastRegression(SensorDataVector_link_count, true);
	output.LinkVolumeAvgAbsPercentageError =TotalMOEPercentageError/max(1,TotalMOESampleSize);
	output.LinkVolumeAvgAbsError = TotalMOEAbsError /max(1,TotalMOESampleSize);
	//float mean_sqared_error = output.LinkVolumeRootMeanSquaredError/max(1,TotaMOESampleSize);
	//output.LinkVolumeRootMeanSquaredError = sqrt(mean_sqared_error);


	output.ODME_result_link_speed = LeastRegression(SensorDataVector_link_speed, true);
	output.LinkSpeedAvgAbsPercentageError = TotalSpeedPercentageError / max(1, TotalSpeedSampleSize);
	output.LinkSpeedAvgAbsError = TotalSpeedAbsError / max(1, TotalSpeedSampleSize);
	//float mean_sqared_error = output.LinkSpeedRootMeanSquaredError / max(1, TotaSpeedSampleSize);
	//output.LinkSpeedRootMeanSquaredError = sqrt(mean_sqared_error);


	if(g_ObsDensityAvailableFlag)
	{	
		output.ODME_result_lane_density = LeastRegression(SensorDataVector_lane_density, true);
	}

	if(TotalMOESampleSize > 0) 
		g_AssignmentLogFile << "Avg abs MOE error=, " << TotalMOEAbsError /max(1,TotalMOESampleSize)  << "Average Path flow Estimation MAPE =," << TotalMOEPercentageError / max(1,TotalMOESampleSize) << " %" << endl;

}

void g_OutputODMEResults()
{
	cout << "OutputODMEResults()" << endl;
	CCSVWriter validation_result_file;

	validation_result_file.Open ("debug_validation_results.csv");

	validation_result_file.SetFieldName ("name");
	validation_result_file.SetFieldName ("tag");
	validation_result_file.SetFieldName ("link_type");
	validation_result_file.SetFieldName ("from_node_id");
	validation_result_file.SetFieldName ("to_node_id");
	validation_result_file.SetFieldName ("link_id");
	validation_result_file.SetFieldName ("time_interval");
	validation_result_file.SetFieldName ("observed_link_count");
	validation_result_file.SetFieldName ("simulated_link_count");

	validation_result_file.SetFieldName ("simulated_over_observed_ratio");
	validation_result_file.SetFieldName ("simulated_vs_observed_percentage_error");
	validation_result_file.SetFieldName ("simulated_vs_observed_count_error");
	validation_result_file.SetFieldName ("simulated_voc_ratio");
	validation_result_file.SetFieldName ("observed_voc_ratio");

	validation_result_file.SetFieldName ("number_of_lanes");
	validation_result_file.SetFieldName ("lane_capacity_per_hour");
	validation_result_file.SetFieldName ("link_capacity_per_hour");

	validation_result_file.WriteHeader (false,false);

	std::vector <SensorDataPoint> SensorDataVector_link_count;

	//MAPE Mean absolute percentage error 
	//RMSE  root mean sequared error

	for(unsigned li = 0; li< g_LinkVector.size(); li++)
	{	
		DTALink* pLink = g_LinkVector[li];
			// compare the error statistics
			for(unsigned int i = 0; i< pLink->m_LinkMeasurementAry.size(); i++)
			{

				if(pLink->m_FromNodeNumber == 56154)
				{
					TRACE("");
				}


				if( g_ValidationDataStartTimeInMin<= pLink->m_LinkMeasurementAry[i].StartTime   &&  
					pLink->m_LinkMeasurementAry[i].EndTime  <= g_ValidationDataEndTimeInMin)
				{  // within the observation time window


					// CumulativeArrivalCount is available after this time interval, so we calculate statistics on time time_index-1

					int EndTime = min(pLink->m_LinkMeasurementAry[i].EndTime,pLink->m_LinkMOEAry.size()-1);
					int StartTime = min(pLink->m_LinkMeasurementAry[i].StartTime,pLink->m_LinkMOEAry.size()-1);

					int SimulatedInFlowCount = pLink->m_LinkMOEAry[EndTime].CumulativeArrivalCount 
						- pLink->m_LinkMOEAry[ StartTime ].CumulativeArrivalCount; 

					if(SimulatedInFlowCount>=0)
					{
						pLink->m_LinkMeasurementAry[i].SimuFlowCount = SimulatedInFlowCount;

						int ObsFlowCount =  pLink->m_LinkMeasurementAry[i].ObsFlowCount;

						SensorDataPoint element;
						element.x = ObsFlowCount;
						element.y = SimulatedInFlowCount;
						SensorDataVector_link_count.push_back (element);

						pLink->m_LinkMeasurementAry[i].DeviationOfFlowCount = SimulatedInFlowCount -  pLink->m_LinkMeasurementAry[i].ObsFlowCount ;

						if(pLink->m_LinkMeasurementAry[i].ObsTravelTime>=0.01)
						{
							pLink->m_LinkMeasurementAry[i].DeviationOfTravelTime  = pLink->GetSimulatedTravelTime(StartTime) - pLink->m_LinkMeasurementAry[i].ObsTravelTime; 
						}


						if(ObsFlowCount >= 1)  // flow count
						{
							int time_interval_in_min = pLink->m_LinkMeasurementAry[i].EndTime - pLink->m_LinkMeasurementAry[i].StartTime;
							float PercentageError = (SimulatedInFlowCount -  ObsFlowCount)*1.0f/max(1,ObsFlowCount)*100;
							float LaneFlowError = (SimulatedInFlowCount -  ObsFlowCount)*60.0f/
								max(1,time_interval_in_min)/pLink->m_OutflowNumLanes;												

							float obs_v_over_c_ratio = 0;
							float simu_over_c_ratio = 0;
							float link_capacity = pLink->GetNumberOfLanes ()* pLink->m_LaneCapacity;

							if(link_capacity>1)
							{
								obs_v_over_c_ratio  = ObsFlowCount*60.0f/max(1,time_interval_in_min) /link_capacity;  // 60.0f/max(1,time_interval_in_min) --> to hour
								simu_over_c_ratio = SimulatedInFlowCount*60.0f/max(1,time_interval_in_min) /link_capacity;
							}
							validation_result_file.SetValueByFieldName  ("name", pLink->m_LinkMeasurementAry[i].name);
							validation_result_file.SetValueByFieldName  ("tag", pLink->m_LinkMeasurementAry[i].tag);
							validation_result_file.SetValueByFieldName ("link_type", pLink->m_LinkTypeName );
							validation_result_file.SetValueByFieldName ("from_node_id", pLink->m_FromNodeNumber );
							validation_result_file.SetValueByFieldName ("to_node_id",  pLink->m_ToNodeNumber);

							CString link_id_str;
							link_id_str.Format("%d->%d", pLink->m_FromNodeNumber , pLink->m_ToNodeNumber);

							{
								CT2CA pszConvertedAnsiString (link_id_str);
								// construct a std::string using the LPCSTR input
								std::string strStd (pszConvertedAnsiString);

								validation_result_file.SetValueByFieldName ("link_id",  strStd);
							}

							CString time_interval_str;
							time_interval_str.Format("%s->%s", g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].StartTime), g_GetTimeStampString(pLink->m_LinkMeasurementAry[i].EndTime) );

							CT2CA pszConvertedAnsiString (time_interval_str);
							// construct a std::string using the LPCSTR input
							std::string strStd (pszConvertedAnsiString);

							validation_result_file.SetValueByFieldName ("time_interval",strStd);
							validation_result_file.SetValueByFieldName ("observed_link_count",ObsFlowCount );
							validation_result_file.SetValueByFieldName ("simulated_link_count",SimulatedInFlowCount);

							float simulated_over_observed_ratio = SimulatedInFlowCount*1.0f/max(1,ObsFlowCount);


							validation_result_file.SetValueByFieldName ("simulated_over_observed_ratio",simulated_over_observed_ratio);
							validation_result_file.SetValueByFieldName ("simulated_vs_observed_percentage_error",PercentageError);

							int simulated_vs_observed_count_error = SimulatedInFlowCount -  ObsFlowCount;
							validation_result_file.SetValueByFieldName ("simulated_vs_observed_count_error",simulated_vs_observed_count_error);
							validation_result_file.SetValueByFieldName ("simulated_voc_ratio",simu_over_c_ratio);
							validation_result_file.SetValueByFieldName ("observed_voc_ratio",obs_v_over_c_ratio);

							int number_of_lanes = pLink->GetNumberOfLanes ();
							validation_result_file.SetValueByFieldName ("number_of_lanes",number_of_lanes);
							validation_result_file.SetValueByFieldName ("lane_capacity_per_hour",pLink->m_LaneCapacity);
							int link_capacity_per_hour = number_of_lanes * pLink->m_LaneCapacity;
							validation_result_file.SetValueByFieldName ("link_capacity_per_hour",link_capacity_per_hour);

							validation_result_file.WriteRecord ();
						}
					}

				}
			}
	}	


}


