//  Portions Copyright 2010 Xuesong Zhou

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

/*************************************
How to build a simple DTA simulator

step 0: basic input functions


defintions of DTANode, DTALink and DTAVehicle
utility function g_read_integer() g_read_float()
ReadNetworkData() and ReadVehicleData()

step 1: dynamic memory management
FreeMemory() to deallocate dynamic memory for the whole simulation program
Allocate and deallocate dynamic arrays for DTANetworkForSP

step 2: network building for traffic assignment
DTANetworkForSP::BuildNetworkBasedOnZoneCentriod(int ZoneID)

step 3: convert time-dependent OD demand to vehicles data, sorted by departure time
CreateVehicles(int origin_zone, int destination_zone, float number_of_vehicles, int demand_type, float starting_time_in_min, float ending_time_in_min)


step 4: Scan eligiable list and shortest path algorithm
SEList functions in DTANetworkForSP
TDLabelCorrecting_DoubleQueue(int origin, int departure_time)

step 5: assign path to vehicles
GetLinkNoByNodeIndex(int usn_index, int dsn_index)
ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval(int zone,int departure_time_begin, int departure_time_end, int iteration)

step 6: integerate network building, shortest path algorithm and path assignment to dynamic traffic assignment

step 7: output vehicle trajectory file

step 8: NetworkLoading

step 9: VehicularSimulation

step 10: parallel computing: assign vehicles to arrays with different origins/departure times, to speed up the calculation

step 11: parallel computing for different zones
OpenMP

step 12: prepare dynamic arrays for within-simulation shortest path for pre-trp and enroute info vehicles
store the data for each destination node to global array as follows.
//int ***g_RTNodePredAry;  // vehicle type, destination, node available in real-time simulation
//unsigned char *** g_RTLabelCostAry; // vehicle type, destination, node available in real-time simulation


Step 13:
allow multiple centriods per zone in zone.csv, change BuildNetworkBasedOnZoneCentriod()

step 13: stochastic capacity with random numbers for different zones

step 14: UE, SO algorithm, day to day learning algorithm
updating, estimating variance and making prediction

make this as a convergency process

route choice -> compare threshold

Step 15: GetTimeDependentCapacityAtSignalizedIntersection
Add dynamical outflow capacity for signalized intersection

step 16: assign pre-trip info shortest path to vehicles

habitual paths vs. real time info
information provides


feature functions: link-based shortest path to consider turnning panalty

*************************************/

/**************************
menu -> project -> property -> configuraiton -> debugging -> setup working directory

***************************/


// The one and only application object
FILE *g_simulation_log_file = NULL;
int g_simulation_log_level = 0;

FILE *g_emission_log_file = NULL;
int g_emission_log_level = 0;


FILE *g_ODME_log_file = NULL;
FILE *g_ODME_result_file = NULL;

int g_ODME_log_level = 0;

FILE *g_ABM_log_file = NULL;
int g_ABM_log_level = 0;

TCHAR g_SupportedDemandFormat[200] = _T("column, matrix, full_matrix, agent_csv, dsp_vehicle_dat, agent_bin, dynasmart, emme_matrix,trip_csv,transims_trip_file");


CWinApp theApp;

TCHAR g_DTASettingFileName[_MAX_PATH] = _T("./DTASettings.txt");

GridNodeSet** g_GridMatrix = NULL;
double g_GridXStep = 1;
double g_GridYStep = 1;
GDRect g_GridRect;

double g_UnitMile = 1; // to do: calculate this measure based on total link distance 


C_RealTimeSimulationSettings g_RealTimeSimulationSettings;

NetworkSimulationResult g_SimulationResult;
std::vector<DTANode> g_NodeVector;
std::map<int, int> g_NodeNametoIDMap;

HistoricalDemand g_SystemDemand;
std::vector<DTALink*> g_LinkVector;
std::map<string, DTALink*> g_LinkMap;
std::map<int, DTALink*> g_LinkIDMap;




std::map<std::string, DTALink*> g_CountSensorIDMap;
std::map<std::string, DTALink*> g_LinkKeyMap;
std::map<std::string, DTALink*> g_SpeedSensorIDMap;


std::map<int, DTAZone> g_ZoneMap;
std::vector<int> g_ZoneNumber2NoVector;
std::vector<int> g_ZoneNo2NumberVector;

std::vector<DTAVehicleType> g_VehicleTypeVector;

std::vector<DTAVehicle*>		g_VehicleVector;

std::map<int, DTAVehicle*> g_VehicleMap;
std::map<int, DTAVehListPerTimeInterval> g_VehicleTDListMap;
std::map<int, DTAVehListPerTimeInterval> g_OriginalVehicleTDListMap;  // used to keep the copy of vehicle id list per simulation interval, we copy 


std::vector< DemandType> g_DemandTypeVector;


std::map<int, DTALinkType> g_LinkTypeMap;
std::map<int, string> g_NodeControlTypeMap;

// time inteval settings in assignment and simulation
double g_DTASimulationInterval = 0.10000; // min

double g_UnitOfMileOrKM = 1.0000;
double g_gain_factor_link_travel_time_from_external_input = 0.5;

double g_CarFollowingSimulationInterval = 1.0 / 600; // 1/ 600 min
int g_number_of_intervals_per_min = 10; // round to nearest integer
int g_number_of_car_following_intervals_per_min = 600; // round to nearest integer
int g_AggregationTimetInterval = 15; // min
int g_TDSPTimetIntervalSizeForMin = 1;
float g_DemandGlobalMultiplier = 1.0f;
int g_EmissionSmoothVehicleTrajectory = 1;
// maximal # of adjacent links of a node (including physical nodes and centriods( with connectors))
int g_AdjLinkSize = 100; // initial value of adjacent links

int g_ODZoneNumberSize = 0;
int g_ODZoneIDSize = 0;
int g_number_of_prohibited_movements = 0;
int g_StartIterationsForOutputPath = 2;
int g_EndIterationsForOutputPath = 2;

// assignment and simulation settings
int g_NumberOfIterations = 1;
int g_ParallelComputingMode = 1;
int g_AgentBasedAssignmentFlag = 1;
int g_ProhibitUTurnOnFeewayLinkFlag = 0;
float g_ConvergencyRelativeGapThreshold_in_perc;


e_demand_loading_mode g_VehicleLoadingMode = demand_matrix_file_mode; // not load from vehicle file by default, 1: load vehicle file
int g_PlanningHorizon = 120;  // short horizon for saving memory

int g_SimululationReadyToEnd = 120;

// assignment
e_analysis_method g_UEAssignmentMethod = analysis_fixed_percentage; // 0: MSA, 1: day-to-day learning, 2: GAP-based switching rule for UE, 3: Gap-based switching rule + MSA step size for UE

float g_DepartureTimeChoiceEarlyDelayPenalty = 1;
float g_DepartureTimeChoiceLateDelayPenalty = 1;
float g_CurrentGapValue = 0.0; // total network gap value in the current iteration
float g_CurrentRelativeGapValue = 0.0;
float g_PrevRelativeGapValue = 0.0;
float g_PercentageCompleteTrips = 100.0;
int g_CurrentNumOfVehiclesSwitched = 0; // total number of vehicles switching paths in the current iteration; for MSA, g_UEAssignmentMethod = 0
int g_CurrentNumOfVehiclesForUEGapCalculation = 0;
int g_PrevNumOfVehiclesSwitched = 0; // // total number of vehicles switching paths in last iteration; for MSA, g_UEAssignmentMethod = 0
int g_ConvergenceThreshold_in_Num_Switch; // the convergence threshold in terms of number of vehicles switching paths; for MSA, g_UEAssignmentMethod = 0
int g_VehicleExperiencedTimeGap = 1; // 1: Vehicle experienced time gap; 0: Avg experienced path time gap
int g_NewPathWithSwitchedVehicles = 0; // number of new paths with vehicles switched to them

float g_TotalDemandDeviation = 0;
float g_UpdatedDemandPrintOutThreshold = 5;
float g_TotalMeasurementDeviation = 0;

int g_output_OD_path_MOE_file = 1;
int g_output_OD_TD_path_MOE_file = 1;
int g_output_OD_path_MOE_cutoff_volume = 1;
float g_OverallPerceptionErrorRatio = 0;
float g_VMSPerceptionErrorRatio;

int g_information_updating_interval_in_min;
bool g_bInformationUpdatingAndReroutingFlag = false;
bool g_bVehicleAttributeUpdatingFlag = false;

int g_information_updating_interval_of_VMS_in_min = 60;


int g_LearningPercentage = 15;
float g_TravelTimeDifferenceForSwitching = 1.0;  // min
float g_RelativeTravelTimePercentageDifferenceForSwitching = 15;  // min


int g_RandomizedCapacityMode = 0;
double g_DemandCapacityScalingFactor = 1.0;
int g_StochasticCapacityMode = 0;
int g_UseRandomCapacityMode = 0;
float g_MinimumInFlowRatio = 0.1f;
float g_RelaxInFlowConstraintAfterDemandLoadingTime = 60;
float g_MaxDensityRatioForVehicleLoading = 0.8f;
float g_DefaultSaturationFlowRate_in_vehphpl;

std::vector<TimeDependentDemandProfile> g_TimeDependentDemandProfileVector;
int g_DemandLoadingStartTimeInMin = 0;
int g_DemandLoadingEndTimeInMin = 0;

int g_ValidationDataStartTimeInMin = 0;
int g_ValidationDataEndTimeInMin = 0;

int g_number_of_warnings = 0;  // use a global count to avoid warning messages when running multiple scenarioss

double g_number_of_intra_zone_trips = 0;

int g_Number_of_CompletedVehicles = 0;
int g_Number_of_CompletedVehiclesThatSwitch = 0;
int g_Number_of_GeneratedVehicles = 0;

int g_InfoTypeSize = 1;  // for shortest path with generalized costs depending on LOV, HOV, trucks or other vehicle classes.
int g_start_iteration_for_MOEoutput = 0;

// for fast data acessing
int g_LastLoadedVehicleID = 0; // scan vehicles to be loaded in a simulation interval
int g_use_routing_policy_from_external_input = 0;
int g_output_routing_policy_file = 0;

int g_SystemOptimalStartingTimeinMin = 450;

VehicleArrayForOriginDepartrureTimeInterval** g_TDOVehicleArray = NULL; // TDO for time-dependent origin

std::vector<DTA_vhc_simple>   g_simple_vector_vehicles;	// vector of DSP_Vehicle, not pointer!;

FILE* g_DebugLogFile = NULL;



ofstream g_LogFile;


CCSVWriter g_SummaryStatFile;
CCSVWriter g_MultiScenarioSummaryStatFile;

ofstream g_AssignmentLogFile;
ofstream g_NetworkDesignLogFile;
ofstream g_EstimationLogFile;
ofstream g_WarningFile;

e_traffic_flow_model g_TrafficFlowModelFlag = tfm_BPR;
e_signal_representation_model g_SignalRepresentationFlag = signal_model_continuous_flow;
float g_LearningPercVector[1000] = { 10 };

int g_ShortestPathWithMovementDelayFlag = 0;
int g_UseDefaultLaneCapacityFlag = 1;
int g_UseFreevalRampMergeModelFlag = 0;
int g_OutputLinkCapacityFlag = 0;
int g_OutputLinkCapacityStarting_Time = 0;
int g_OutputLinkCapacityEnding_Time = 300;
int g_CalculateUEGapForAllAgents = 0;
int g_EmissionDataOutputFlag = 0;
int g_VehiclePathOutputFlag = 1;
int g_TimeDependentODMOEOutputFlag = 0;
int g_OutputSecondBySecondEmissionData = 0;
float g_OutputSecondBySecondEmissionDataPercentage = 0.1f;
int g_start_departure_time_for_output_second_by_second_emission_data = 0;
int g_end_departure_time_for_output_second_by_second_emission_data = 0;
int g_OutputEmissionOperatingModeData = 0;
int g_TargetVehicleID_OutputSecondBySecondEmissionData = 0;

int g_TollingMethodFlag = 0;
float g_VMTTollingRate = 0;

int g_MergeNodeModelFlag = 1;
int g_FIFOConditionAcrossDifferentMovementFlag = 0;

std::vector<NetworkMOE>  g_NetworkMOEAry;
std::vector<NetworkLoadingOutput>  g_AssignmentMOEVector;
std::map<int, RealTimeSimulationSettings>  g_RealTimeSimulationSettingsMap;

DTALinkToll** g_LinkTDCostAry = NULL;

float g_ratio_mile_to_km = 1.60934;
DTASettings g_settings;  // global settings;


CTime g_AppStartTime;
CTime g_AppLastIterationStartTime;

int g_AssignmentIntervalIndex[MAX_TIME_INTERVAL_SIZE] = { 0 };   // internal index for memory block
int g_AssignmentIntervalOutputFlag[MAX_TIME_INTERVAL_SIZE] = { 0 };   // internal index for memory block


int g_AssignmentIntervalStartTimeInMin[MAX_TIME_INTERVAL_SIZE] = { 0 };
int g_AssignmentIntervalEndTimeInMin[MAX_TIME_INTERVAL_SIZE] = { 0 };

int g_NumberOfSPCalculationPeriods = 1;

unsigned int g_RandomSeed = 100;


using namespace std;

void _proxy_simulation_log(int level, int simulation_interval_no, double simulation_in_min, int debug_agent_id, const char *fmt, ...)
{

		va_list arg;

	/* Check if the message should be logged */
	if (level > g_simulation_log_level)
		return;
#pragma omp critical
	{
	/* Write the error message */
	va_start(arg, fmt);
	if (g_simulation_log_file != NULL)
	{
		fprintf(g_simulation_log_file, "sim clock=%d (%.2f min),", simulation_interval_no, simulation_in_min);

		if (debug_agent_id >= 0)
			fprintf(g_simulation_log_file, "debug_agent_id=%d,", debug_agent_id);

		vfprintf(g_simulation_log_file, fmt, arg);
	}
	va_end(arg);

#ifdef DEBUG
	fflush(g_simulation_log_file);
	//	fsync(fileno(g_simulation_log_file));
#endif
}
}

void _proxy_emission_log(int level, int debug_agent_id, const char *fmt, ...)
{

		va_list arg;

	/* Check if the message should be logged */
	if (level > g_emission_log_level)
		return;
#pragma omp critical
	{
	/* Write the error message */
	va_start(arg, fmt);
	if (g_emission_log_file != NULL)
	{
		if (debug_agent_id >= 0)
			fprintf(g_emission_log_file, "debug_agent_id=%d,", debug_agent_id);

		vfprintf(g_emission_log_file, fmt, arg);
	}
	va_end(arg);

#ifdef DEBUG
	fflush(g_emission_log_file);

#endif
}
}


void _proxy_ODME_log(int level, int iteration_no, const char *fmt, ...) 
{

		va_list arg;

	/* Check if the message should be logged */
	if (level > g_ODME_log_level)
		return;
#pragma omp critical
	{
	/* Write the error message */
	va_start(arg, fmt);
	if (g_ODME_log_file != NULL)
	{
		fprintf(g_ODME_log_file, "iteration no=%d,", iteration_no);

		vfprintf(g_ODME_log_file, fmt, arg);
	}
	va_end(arg);

#ifdef DEBUG
	fflush(g_ODME_log_file);
	//	fsync(fileno(g_simulation_log_file));
#endif
}
}


void _proxy_ABM_log(int level, const char *fmt, ...)
{
	return;

	va_list arg;

	/* Check if the message should be logged */
	if (level > g_ABM_log_level)
		return;
#pragma omp critical
	{
	/* Write the error message */
	va_start(arg, fmt);
	if (g_simulation_log_file != NULL)
	{
		int hour = g_simulation_time / 60;
		int min = g_simulation_time - hour * 60;
		int sec = (g_simulation_time - hour * 60 - min) * 60;

		if (level >= 0)
		{
		
		fprintf(g_ABM_log_file, "Simu Clock %02d:%02d:%02d--", hour, min, sec);
		}


		vfprintf(g_ABM_log_file, fmt, arg);
	}
	va_end(arg);

#ifdef DEBUG
	if (g_simulation_log_file != NULL)
	{
	fflush(g_ABM_log_file);
	}
	
	//	fsync(fileno(g_simulation_log_file));
#endif
	}
}

double g_simulation_time = 0;


void ReadNodeControlTypeCSVFile()
{

	g_NodeControlTypeMap[0] = "unknown_control";
	g_NodeControlTypeMap[1] = "no_control";
	g_NodeControlTypeMap[2] = "yield_sign";
	g_NodeControlTypeMap[3] = "2way_stop_sign";
	g_NodeControlTypeMap[4] = "4way_stop_sign";
	g_NodeControlTypeMap[5] = "pretimed_signal";
	g_NodeControlTypeMap[6] = "actuated_signal";
	g_NodeControlTypeMap[7] = "roundabout";


	CCSVParser parser;
	if (parser.OpenCSVFile("input_node_control_type.csv"))
	{
		int control_type_code;
		int i = 0;
		while (parser.ReadRecord())
		{
			control_type_code = 0;
			parser.GetValueByFieldName("unknown_control", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "unknown_control";

			control_type_code = 1;
			parser.GetValueByFieldName("no_control", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "no_control";

			control_type_code = 2;
			parser.GetValueByFieldName("yield_sign", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "yield_sign";

			control_type_code = 3;
			parser.GetValueByFieldName("2way_stop_sign", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "2way_stop_sign";

			control_type_code = 4;
			parser.GetValueByFieldName("4way_stop_sign", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "4way_stop_sign";

			control_type_code = 5;
			parser.GetValueByFieldName("pretimed_signal", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "pretimed_signal";

			control_type_code = 6;
			parser.GetValueByFieldName("actuated_signal", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "actuated_signal";

			control_type_code = 7;
			parser.GetValueByFieldName("roundabout", control_type_code);
			g_NodeControlTypeMap[control_type_code] = "roundabout";



			break;  // just one line
		}




	}

}

int FindNodeControlType(string control_type)
{
	for (std::map<int, string>::iterator iter_control_type = g_NodeControlTypeMap.begin(); iter_control_type != g_NodeControlTypeMap.end(); iter_control_type++)
	{
		if (iter_control_type->second.find(control_type) != string::npos)
		{
			return iter_control_type->first;
		}
	}

	return 0;
}

void g_ReadInputFiles()
{

	g_InitialFreeMemory = g_GetFreeMemoryDataInMB();
	// set random number seed
	// write version number (for agent.bin version control)

	//int current_revision_number = 2;
	//int previous_revision_number = g_GetPrivateProfileInt("version_control", "revision_number", 1, g_DTASettingFileName);
	//g_WritePrivateProfileInt("version_control", "revision_number", current_revision_number, g_DTASettingFileName);

	//if(previous_revision_number< current_revision_number)
	//{
	// cout << "The file revision number is outdated." << endl;
	// cout << "	The revision number in this data set = " << previous_revision_number << endl;
	// cout << " The latest revision number = " << current_revision_number << endl;
	// cout << "Please follow NEXTA installation folder\\default_data_folder\\version_control_log.csv to update data files accordingly." << endl;
	// g_ProgramStop();
	//}


	unsigned int state[16];

	for (int k = 0; k < 16; ++k)
	{
		state[k] = k + g_RandomSeed;
	}

	InitWELLRNG512a(state);

	int z;

	// step 0: check if output files are opened.

	//test if output files can be opened
	{

		//CCSVWriter File_output_ODMOE;
		//File_output_ODMOE.Open("output_ODMOE.csv");

		CCSVWriter File_output_Agent;
		File_output_Agent.Open("output_agent.csv");

		CCSVWriter File_output_ODTDMOE;
		File_output_ODTDMOE.Open("output_ODTDMOE.csv");


		CCSVWriter File_output_NetworkTDMOE;
		File_output_NetworkTDMOE.Open("output_NetworkTDMOE.csv");

		CCSVWriter File_output_LinkTDMOE;
		File_output_LinkTDMOE.Open("output_LinkTDMOE.csv");

		CCSVWriter File_output_LinkMOE;
		File_output_LinkMOE.Open("output_LinkMOE.csv");


	}


	 g_ReadAssignmentPeriodSettings();
	//*******************************
	// step 1: node input
	cout << "Step 1: Reading file input_node.csv..." << endl;
	g_LogFile << "Step 1: Reading file input_node.csv..." << g_GetUsedMemoryDataInMB() << endl;



	if (g_UEAssignmentMethod != analysis_accessibility_distance)  //  node control type is not required when calculating node-to-node distance 
	{
		ReadNodeControlTypeCSVFile();
	}

	if (FindNodeControlType("pretimed_signal") > 0)  // valid record
	{
		g_settings.pretimed_signal_control_type_code = FindNodeControlType("pretimed_signal");
	}

	if (FindNodeControlType("actuated_signal") > 0)  //valid record
	{
		g_settings.actuated_signal_control_type_code = FindNodeControlType("actuated_signal");
	}

	if (FindNodeControlType("no_control") > 0)  //valid record
	{
		g_settings.no_signal_control_type_code = FindNodeControlType("no_control");
	}

	int NodeControlTypeCount[10];

	for (int control_type_i = 0; control_type_i < 10; control_type_i++)
	{
		NodeControlTypeCount[control_type_i] = 0;
	}

	CCSVParser parser_node;
	if (parser_node.OpenCSVFile("input_node.csv"))
	{
		int i = 0;
		while (parser_node.ReadRecord())
		{
			int node_id;
			DTANode* pNode = 0;

			if (parser_node.GetValueByFieldNameRequired("node_id", node_id) == false)
				break;

			int control_type = 0;


			DTANode Node;
			parser_node.GetValueByFieldName("control_type", control_type);
		
			parser_node.GetValueByFieldName("transit_stop_id", Node.m_transit_stop_id);
			parser_node.GetValueByFieldName("transit_stop_type", Node.m_transit_stop_type);

			std::string demand_type_code;
			parser_node.GetValueByFieldName("transit_demand_type_code", Node.m_transit_demand_type_code);

			if (Node.m_transit_demand_type_code.size() > 0)
			{
				TRACE("");
			}
			parser_node.GetValueByFieldName("x", Node.m_pt.x);
			parser_node.GetValueByFieldName("y", Node.m_pt.y);

			if (control_type == g_settings.pretimed_signal_control_type_code ||
				control_type == g_settings.actuated_signal_control_type_code)
			{

				int cycle_length_in_second = 0;
				parser_node.GetValueByFieldName("cycle_length_in_second", cycle_length_in_second);
				Node.m_CycleLength_In_Second = cycle_length_in_second;
				int offset_in_second = 0;
				parser_node.GetValueByFieldName("offset_in_second", offset_in_second);
				Node.m_SignalOffset_In_Second = offset_in_second;

			}



			Node.m_NodeID = i;
			Node.m_ZoneID = 0;
			Node.m_NodeNumber = node_id;
			Node.m_ControlType = control_type;

			NodeControlTypeCount[control_type] += 1;
			g_NodeVector.push_back(Node);
			g_NodeNametoIDMap[node_id] = i;
			i++;
		}
	}
	else
	{
		cout << "Error: File input_node.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		g_ProgramStop();

	}

	g_SummaryStatFile.WriteParameterValue("# of Nodes", g_NodeVector.size());


	//*******************************
	// step 2: link type input

	cout << "Step 2: Reading file input_link_type.csv..." << endl;
	g_LogFile << "Step 2: Reading file input_link_type.csv.." << g_GetUsedMemoryDataInMB() << endl;

	if (g_UEAssignmentMethod != analysis_accessibility_distance)  //  link type is not required when calculating node-to-node distance 
	{
		CCSVParser parser_link_type;

		if (!parser_link_type.OpenCSVFile("input_link_type.csv"))
		{
			cout << "Error: File input_link_type.csv cannot be opened.\n Try to use default value." << endl;
			g_ProgramStop();


		}
		if (parser_link_type.inFile.is_open() || parser_link_type.OpenCSVFile("input_link_type.csv"))
		{

			std::map<int, int> link_type_map;
			int line_no = 0;

			while (parser_link_type.ReadRecord())
			{
				DTALinkType element;

				if (parser_link_type.GetValueByFieldName("link_type", element.link_type) == false)
				{
					if (line_no == 0)
					{
						cout << "Error: Field link_type cannot be found in file input_link_type.csv." << endl;
						g_ProgramStop();
					}
					else
					{  // read empty line
						break;
					}
				}

				if (link_type_map.find(element.link_type) != link_type_map.end())
				{
					cout << "Error: Field link_type " << element.link_type << " has been defined more than once in file input_link_type.csv." << endl;
					g_ProgramStop();

					break;
				}

				link_type_map[element.link_type] = 1;



				if (parser_link_type.GetValueByFieldName("link_type_name", element.link_type_name) == false)
				{
					g_LogFile << "Warning: value link_type_name for link type " << element.link_type << " cannot be found in file input_link_type.csv." << endl;

				}
				if (parser_link_type.GetValueByFieldName("type_code", element.type_code) == false)
				{
					cout << "Error: Field type_code for link type " << element.link_type << "cannot be found in file input_link_type.csv." << endl;
					cout << "The corresponding links will not be added in the network." << endl;
					cout << "Please check and press any key to continue!" << endl;

					element.type_code = "n";
					getchar();
				}


				if (element.type_code.find_first_not_of("afrhwctnbp") != string::npos)
				{
					cout << "Error: invalid type_code for link type " << element.link_type_name << " in input_link_type.csv = " << element.type_code << endl;
					g_ProgramStop();
				}

				parser_link_type.GetValueByFieldName("travel_time_bias_factor", element.link_type_bias_factor);


				if (element.link_type_bias_factor <= 0.01 || element.link_type_bias_factor >= 100)
				{
					cout << "Error: invalid link_type_bias_factor for link type " << element.link_type_name << " in input_link_type.csv = " << element.link_type_bias_factor << endl;
					g_ProgramStop();

				}

				parser_link_type.GetValueByFieldName("capacity_adjustment_factor", element.capacity_adjustment_factor);


				if (element.capacity_adjustment_factor <= 0.1 || element.capacity_adjustment_factor >= 10)
				{
					cout << "Error: invalid capacity_adjustment_factor for link type " << element.link_type_name << " in input_link_type.csv = " << element.capacity_adjustment_factor << endl;
					g_ProgramStop();

				}

				parser_link_type.GetValueByFieldName("approximate_cycle_length_in_second", element.approximate_cycle_length_in_second);


				if (element.approximate_cycle_length_in_second <= -2 || element.approximate_cycle_length_in_second >= 1000)
				{
					cout << "Error: invalid approximate_cycle_length_in_second for link type " << element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
					g_ProgramStop();

				}

				parser_link_type.GetValueByFieldName("saturation_flow_rate_in_vhc_per_hour_per_lane", element.saturation_flow_rate_in_vhc_per_hour_per_lane);


				if (element.saturation_flow_rate_in_vhc_per_hour_per_lane <= -2 || element.saturation_flow_rate_in_vhc_per_hour_per_lane >= 3000)
				{
					cout << "Error: invalid saturation_flow_rate_in_vhc_per_hour_per_lane for link type " << element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
					g_ProgramStop();

				}

				if (element.approximate_cycle_length_in_second >= 1)
				{
					if (element.saturation_flow_rate_in_vhc_per_hour_per_lane <= 1)
					{
						cout << "Error: invalid saturation_flow_rate_in_vhc_per_hour_per_lane for link type with positive approximate_cycle_length_in_second" << element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
						g_ProgramStop();

					}
				}


				if (g_UseDefaultLaneCapacityFlag == 1)
				{
					if (parser_link_type.GetValueByFieldName("default_lane_capacity", element.default_lane_capacity) == false)
					{
						cout << "Error: Field default_lane_capacity cannot be found in file input_link_type.csv." << endl;
						g_ProgramStop();
					}
				}


				g_LinkTypeMap[element.link_type] = element;

				line_no++;
			}
		}
		else
		{
			cout << "Error: File input_link_type.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;


		}

		g_SummaryStatFile.WriteParameterValue("# of Link Types", g_LinkTypeMap.size());

	}


	g_BuildGridSystem();
	//*******************************
	// step 3: link data input

	char InputLinkFileName[_MAX_PATH];

	GetPrivateProfileString("input_file", "link_data", "input_link.csv", InputLinkFileName, _MAX_PATH, g_DTASettingFileName);


	int AllowExtremelyLowCapacityFlag = 1; // g_GetPrivateProfileInt("input_checking", "allow_extremely_low_capacity", 1, g_DTASettingFileName);
	//	g_ProhibitUTurnOnFeewayLinkFlag = g_GetPrivateProfileInt("shortest_path", "prohibit_u_turn_on_freeway_link", 1, g_DTASettingFileName);	


	cout << "Step 3: Reading file input_link.csv... = " << endl;
	cout << "g_PlanningHorizon = " << g_PlanningHorizon << endl;

	
	g_LogFile << "Step 3: Reading file input_link.csv..." << g_GetUsedMemoryDataInMB() << endl;

	int  count_effective_green_time = 0;
	double total_effective_green_time = 0;
	double total_signal_link_capacity = 0;
	double total_signal_link_cycle_length = 0;
	int global_NetworkDesignSequenceNo = 0;
	int i = 0;

	int max_number_of_warnings_to_be_showed = 5;
	DTALink* pLink = 0;
	CCSVParser parser_link;
	int signal_reset_count = 0;
	int control_node_number = 0;
	int missing_node_error = 0;

	double total_link_length = 0;
	double total_link_coordinate_length = 0;
	if (parser_link.OpenCSVFile(InputLinkFileName))
	{
		bool bNodeNonExistError = false;
		while (parser_link.ReadRecord())
		{
			int from_node_name = 0;
			int to_node_name = 0;
			int direction = 1;
			double length_ = 1;
			int number_of_lanes = 1;
			int speed_limit_in_mph = 0;
			float speed_at_capacity = 50;
			float KCritical = 10;
			double capacity = 0;
			int type;
			string name, mode_code, demand_type_code;
			double K_jam, wave_speed_in_mph, AADT_conversion_factor;

			int org_link_id = 0;
			if (!parser_link.GetValueByFieldName("from_node_id", from_node_name))
			{
				if (i == 0)
				{
					cout << "Field from_node_id has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
				else
				{ // i>=1;
					break;  //read empty line.

				}
			}

			if (!parser_link.GetValueByFieldName("to_node_id", to_node_name))
			{
				cout << "Field to_node_id has not been defined in file input_link.csv. Please check.";
				getchar();
				exit(0);

			}

			if (!parser_link.GetValueByFieldName("mode_code", mode_code))
				mode_code = "";

			if (!parser_link.GetValueByFieldName("demand_type_code", demand_type_code))
				demand_type_code = "";

			if (mode_code.compare("w") == 0)   // do not model walking-only link in this version
				continue;

			if (mode_code.compare("t") == 0)   // do not model trainsit-only link in this version
				continue;

			if (mode_code.compare("b") == 0)   // do not model pedestran-only link in this version
				continue;

			if (mode_code.compare("n") == 0)   // do not model pedestran-only link in this version
				continue;

			if (!parser_link.GetValueByFieldName("link_type", type))
			{
				if (g_UEAssignmentMethod != analysis_accessibility_distance)
				{

					cout << "Field link_type has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}

			if (g_LinkTypeMap[type].type_code.find_first_of('nb') != string::npos)   //type_code: no: not included in the network, b: bike
				continue;



			if (g_NodeNametoIDMap.find(from_node_name) == g_NodeNametoIDMap.end())
			{

				fprintf(g_DebugLogFile, "from_node_id %d in input_link.csv has not be defined in input_node.csv.", from_node_name);

				if (missing_node_error <= 3)
				{
					cout << "from_node_id " << from_node_name << " in input_link.csv has not be defined in input_node.csv." << endl;
					cout << "associated link_type : " << type << endl;
					cout << "Please check error.log." << endl;
					getchar();
				}
				missing_node_error++;
				continue;

			}

			if (g_NodeNametoIDMap.find(to_node_name) == g_NodeNametoIDMap.end())
			{

				fprintf(g_DebugLogFile, "to_node_id %d in input_link.csv has not be defined in input_node.csv.", to_node_name);
				if (missing_node_error <= 3)
				{
					cout << "to_node_id " << to_node_name << " in input_link.csv has not be defined in input_node.csv. " << endl;
					cout << "associated from_node_id: " << from_node_name << " ; link_type : " << type << endl;
					cout << "Please check error.log." << endl;
					getchar();
				}
				missing_node_error++;

				continue;
			}

			parser_link.GetValueByFieldName("link_id", org_link_id);

			if (!parser_link.GetValueByFieldName("direction", direction))
				direction = 1;

			int netwrok_design_flag = 0;

			parser_link.GetValueByFieldName("network_design_flag", netwrok_design_flag);



			if (!parser_link.GetValueByFieldName("length", length_))
			{

				if (!parser_link.GetValueByFieldName("length_", length_))
				{

					cout << "Field length has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}

			if (length_ > 2000)
			{
				cout << "Link: " << from_node_name << "->" << to_node_name << " in input_link.csv has " << "length_ = " << length_ << " , which might be too long. Please check.";
				//				sleep(5);
			}

			if (!parser_link.GetValueByFieldName("number_of_lanes", number_of_lanes))
			{
				if (g_UEAssignmentMethod != analysis_accessibility_distance)
				{
					cout << "Field number_of_lanes has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}

			if (g_LinkTypeMap[type].type_code.find('c') != string::npos && number_of_lanes == 0)
			{
				number_of_lanes = 7; // reset # of lanes for connectors to a positive value
			}

			if (!parser_link.GetValueByFieldName("speed_limit", speed_limit_in_mph))
			{

				if (!parser_link.GetValueByFieldName("speed_limit_in_mph", speed_limit_in_mph))
				{
					cout << "Field speed_limit_in_mph has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}
			if (!parser_link.GetValueByFieldName("speed_at_capacity", speed_at_capacity))
			{
				speed_at_capacity = speed_limit_in_mph;
			}


			if (!parser_link.GetValueByFieldName("lane_cap", capacity))
			{
				
				if (!parser_link.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity))
				{
					cout << "Field lane_cap has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}

			}

			KCritical = capacity / max(10, speed_at_capacity); // e.g. 1500 vehicles/hour/lane / 30 mph = 50 vehicles /mile/lane


			if (g_DemandCapacityScalingFactor < 0.999 || g_DemandCapacityScalingFactor > 1.001)
				capacity *= g_DemandCapacityScalingFactor;


			int SaturationFlowRate;

			float BPR_Alpha = 0.15f;
			float BPR_Beta = 4.0f;
			float BPR_Distance = 0.0f;

			if (g_TrafficFlowModelFlag == tfm_BPR)
			{
				//if a static_traffic_analysis_mode is used, then the following term are required
				parser_link.GetValueByFieldNameRequired("BPR_alpha_term", BPR_Alpha);
				parser_link.GetValueByFieldNameRequired("BPR_beta_term", BPR_Beta);
				parser_link.GetValueByFieldNameRequired("BPR_distance_term", BPR_Distance);

			}


			int LeftTurnBayLengthInFeet = 0;
			int LeftTurnCapacity = 0;
			char link_direction;


			parser_link.GetValueByFieldName("from_approach", link_direction);





			if ((g_UEAssignmentMethod != analysis_accessibility_distance) && g_LinkTypeMap.find(type) == g_LinkTypeMap.end())
			{
				int round_down_type = int(type / 10) * 10;
				if (g_LinkTypeMap.find(round_down_type) != g_LinkTypeMap.end())  // round down type exists
				{
					g_LinkTypeMap[type].type_code = g_LinkTypeMap[round_down_type].type_code;
				}
				else
				{
					cout << "Link: " << from_node_name << "->" << to_node_name << " in input_link.csv has " << "link_type = " << type << ", which has not been defined in file input_link.csv. Please check. Use default link type: arterial.";
					g_LinkTypeMap[type].type_code = 'a';
					getchar();
				}

				//				if(g_UseDefaultLaneCapacityFlag ==1)
				capacity = g_LinkTypeMap[type].default_lane_capacity;
			}


			if (!parser_link.GetValueByFieldName("AADT_conversion_factor", AADT_conversion_factor))
				AADT_conversion_factor = 0.1;

			if (!parser_link.GetValueByFieldName("jam_density", K_jam))
			{
				if (!parser_link.GetValueByFieldName("jam_density_in_vhc_pmpl", K_jam))
				if (g_settings.use_mile_or_km_as_length_unit == 1)
					K_jam = 180;
				else
					K_jam = 180 / g_ratio_mile_to_km;
			}

			if (g_DemandCapacityScalingFactor < 0.999 || g_DemandCapacityScalingFactor > 1.001)
				K_jam *= g_DemandCapacityScalingFactor;

			if (!parser_link.GetValueByFieldName("wave_speed", wave_speed_in_mph))
			{
				if (!parser_link.GetValueByFieldName("wave_speed_in_mph", wave_speed_in_mph))
				{
					if (g_settings.use_mile_or_km_as_length_unit == 1)
						wave_speed_in_mph = 12;
					else
						wave_speed_in_mph = 12 / g_ratio_mile_to_km;

				}
			}


			int ProhibitedU_Turn = 0;

			parser_link.GetValueByFieldName("prohibited_u-turn,", ProhibitedU_Turn);


			if (from_node_name == 58987 && to_node_name == 54430)
			{
				TRACE(" ");
			}



			int link_code_start = 1;
			int link_code_end = 1;

			if (direction == -1) // reversed
			{
				link_code_start = 2; link_code_end = 2;
			}

			if (direction == 0) // two-directional link
			{
				link_code_start = 1; link_code_end = 2;
			}


			if (number_of_lanes == 0)  // skip this link 
			{
				g_WarningFile << "link with 0 lane, skip:" << from_node_name << " -> " << to_node_name << endl;
				continue;
			}

			if (capacity < 1)  // skip this link 
			{
				g_WarningFile << "link with capacity " << capacity << ", skip: " << from_node_name << " -> " << to_node_name << endl;
				continue;
			}

			if (speed_limit_in_mph < 1)  // skip this link 
			{
				g_WarningFile << "link with speed limit " << speed_limit_in_mph << ", skip: " << from_node_name << " -> " << to_node_name << endl;
				continue;
			}

			for (int link_code = link_code_start; link_code <= link_code_end; link_code++)
			{

				int FromID = from_node_name;
				int ToID = to_node_name;
				if (link_code == 1)  //AB link
				{
					FromID = from_node_name;
					ToID = to_node_name;
				}
				if (link_code == 2)  //BA link
				{
					FromID = to_node_name;
					ToID = from_node_name;
				}


				pLink = new DTALink(g_PlanningHorizon);
				if (pLink == NULL)
				{
					cout << "Allocating memory error at line " << i + 1 << endl;
					getchar();
					exit(0);

				}

				parser_link.GetValueByFieldName("name", pLink->m_Name, true);

				std::string link_key, count_sensor_id, speed_sensor_id;

				parser_link.GetValueByFieldName("link_key", link_key);
				parser_link.GetValueByFieldName("count_sensor_id", count_sensor_id);
				parser_link.GetValueByFieldName("speed_sensor_id", speed_sensor_id);

				if (link_key.size() > 0)
					g_LinkKeyMap[link_key] = pLink;

				if (count_sensor_id.size() > 0)
					g_CountSensorIDMap[count_sensor_id] = pLink;

				if (speed_sensor_id.size() > 0)
					g_SpeedSensorIDMap[speed_sensor_id] = pLink;


				pLink->m_LinkNo = i;
				pLink->m_RandomSeed = pLink->m_LinkNo; // assign a link specific random seed

				pLink->m_OrgLinkID = org_link_id;
				pLink->m_link_code = link_code;
				pLink->m_FromNodeNumber = FromID;
				pLink->m_ToNodeNumber = ToID;
				pLink->m_FromNodeID = g_NodeNametoIDMap[pLink->m_FromNodeNumber];
				pLink->m_ToNodeID = g_NodeNametoIDMap[pLink->m_ToNodeNumber];

				pLink->m_ProhibitedU_Turn = ProhibitedU_Turn;
				pLink->m_NetworkDesignFlag = netwrok_design_flag;

				if (netwrok_design_flag >= 1)
				{
					pLink->m_NetworkDesignSequenceNo = global_NetworkDesignSequenceNo++;
				}
				if (netwrok_design_flag == 1)
					TRACE("consider adding links %d->%d", from_node_name, to_node_name);

				if (ProhibitedU_Turn == 1)
				{
					g_ShortestPathWithMovementDelayFlag = true; // with movement input
					string movement_id = GetMovementStringID(FromID, ToID, FromID);
					int middle_node_id = g_NodeNametoIDMap[ToID];

					g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_from_node_id = FromID;
					g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_to_node_id = ToID;
					g_NodeVector[middle_node_id].m_MovementMap[movement_id].out_link_to_node_id = FromID;

					g_NodeVector[middle_node_id].m_MovementMap[movement_id].b_turning_prohibited = true;   // assign movement to individual node

					g_number_of_prohibited_movements++;
				}


				pLink->m_BPR_Alpha = BPR_Alpha;
				pLink->m_BPR_Beta = BPR_Beta;
				pLink->m_BPR_Distance = BPR_Distance;

				std::vector<CCoordinate> CoordinateVector;
				string geo_string;

				double link_coordinate_length = 0;
				if (parser_link.GetValueByFieldName("geometry", geo_string) == false)
				{
					// overwrite when the field "geometry" exists
					CGeometry geometry(geo_string);
					CoordinateVector = geometry.GetCoordinateList();
					for (int si = 0; si < CoordinateVector.size(); si++)
					{
						GDPoint	pt;
						pt.x = CoordinateVector[si].X;
						pt.y = CoordinateVector[si].Y;
						pLink->m_ShapePoints.push_back(pt);

						if (si >= 1)
						{
							link_coordinate_length += pow(pow(CoordinateVector[si].X - CoordinateVector[si - 1].X, 2) + pow(CoordinateVector[si].Y - CoordinateVector[si - 1].Y, 2), 0.5);

						}
					}

				}


				parser_link.GetValueByFieldName("geometry", pLink->m_geometry_string);
				parser_link.GetValueByFieldName("geometry", pLink->m_original_geometry_string);


				total_link_length += length_;
				total_link_coordinate_length += link_coordinate_length;


				pLink->m_SpeedLimit = speed_limit_in_mph;

				pLink->m_SpeedAtCapacity = speed_at_capacity;
				pLink->m_KCritical = KCritical;

				float	m_KCritical;
				float length = max(0.00001, length_);  // minimum link length 0.0001

				pLink->m_OutflowNumLanes = number_of_lanes;
				pLink->m_InflowNumLanes = number_of_lanes;

				pLink->m_Orginal_OutflowNumLanes = number_of_lanes;
				pLink->m_Orginal_InflowNumLane = number_of_lanes;



				pLink->m_Direction = link_direction;

				if (g_AgentBasedAssignmentFlag != analysis_accessibility_distance && g_TrafficFlowModelFlag != tfm_BPR)
					pLink->m_Length = max(length, pLink->m_SpeedLimit*0.1f / 60.0f);  // we do not impose the minimum distance in this version
				else
					pLink->m_Length = length;



				if (AllowExtremelyLowCapacityFlag == 0 && capacity < 10 && g_number_of_warnings < max_number_of_warnings_to_be_showed)
				{
					cout << "In file input_link.csv, line " << i + 1 << " has capacity <10" << capacity << ", which might not be realistic. Please correct the error." << endl;
					getchar();
					g_number_of_warnings++;
				}



				pLink->m_link_type = type;

				pLink->demand_type_code = demand_type_code;



				if (g_LinkTypeMap.find(type) == g_LinkTypeMap.end())
				{
					int round_down_type = int(type / 10) * 10;
					if (g_LinkTypeMap.find(round_down_type) != g_LinkTypeMap.end())
					{
						g_LinkTypeMap[type].type_code = g_LinkTypeMap[round_down_type].type_code;
					}
					else
					{
						cout << "In file input_link.csv, line " << i + 1 << " has link type " << type << ", which has not been defined in input_link_type.csv. Please correct. Use default link type: arterial street." << endl;
						g_LinkTypeMap[type].type_code = 'a';
						getchar();
					}
				}


				pLink->m_LaneCapacity = capacity * g_LinkTypeMap[type].capacity_adjustment_factor;
				pLink->m_BPRLaneCapacity = pLink->m_LaneCapacity;

				if (g_SignalRepresentationFlag == signal_model_link_effective_green_time )
				{
					if (g_NodeVector[pLink->m_ToNodeID].m_ControlType == g_settings.pretimed_signal_control_type_code ||
						g_NodeVector[pLink->m_ToNodeID].m_ControlType == g_settings.actuated_signal_control_type_code)
					{
						pLink->m_EffectiveGreenTime_In_Second = max(0.1, pLink->m_LaneCapacity / 1800) *  g_NodeVector[pLink->m_ToNodeID].m_CycleLength_In_Second;
						g_LogFile << "Link " << pLink->m_FromNodeNumber << " -> " << pLink->m_ToNodeNumber << " has an effective green time = " << pLink->m_EffectiveGreenTime_In_Second << endl;

						pLink->m_DownstreamCycleLength_In_Second = g_NodeVector[pLink->m_ToNodeID].m_CycleLength_In_Second;
						pLink->m_DownstreamNodeSignalOffset_In_Second = g_NodeVector[pLink->m_ToNodeID].m_SignalOffset_In_Second;

						count_effective_green_time++;
						total_effective_green_time += pLink->m_EffectiveGreenTime_In_Second;
						total_signal_link_capacity += pLink->m_LaneCapacity;
						total_signal_link_cycle_length += g_NodeVector[pLink->m_ToNodeID].m_CycleLength_In_Second;

					}
				}

				pLink->m_LinkTypeName = g_LinkTypeMap[type].link_type_name;


				pLink->m_bFreewayType = g_LinkTypeMap[type].IsFreeway();
				pLink->m_bArterialType = g_LinkTypeMap[type].IsArterial();




				pLink->m_KJam = K_jam;
				pLink->m_AADTConversionFactor = AADT_conversion_factor;
				pLink->m_BackwardWaveSpeed = wave_speed_in_mph;

				if (g_LinkTypeMap[type].IsConnector() == true && g_NodeVector[pLink->m_ToNodeID].m_ControlType != g_settings.no_signal_control_type_code)
				{
					//"no_control" for the downstream node of a connector
					//							g_NodeVector[pLink->m_ToNodeID]. m_ControlType = no_signal_control_type_code;
					if (control_node_number == 0)
						control_node_number = pLink->m_ToNodeNumber;

					//							signal_reset_count++;
				}

				pLink->m_VehicleSpaceCapacity = max(1, pLink->m_Length * pLink->m_OutflowNumLanes *K_jam);

				g_NodeVector[pLink->m_FromNodeID].m_TotalCapacity += (pLink->m_LaneCapacity* pLink->m_OutflowNumLanes);
				g_NodeVector[pLink->m_ToNodeID].m_IncomingLinkVector.push_back(i);
				g_NodeVector[pLink->m_FromNodeID].m_OutgoingLinkVector.push_back(i);

				if (g_NodeVector[pLink->m_FromNodeID].m_OutgoingLinkVector.size() > g_AdjLinkSize)
					g_AdjLinkSize = g_NodeVector[pLink->m_FromNodeID].m_OutgoingLinkVector.size();


				g_NodeVector[pLink->m_ToNodeID].m_IncomingLinkDelay.push_back(0);

				// prevent U turns on freeway links
				if (g_ProhibitUTurnOnFeewayLinkFlag == 1 && g_LinkTypeMap[pLink->m_link_type].IsFreeway() == true)
				{
					g_ShortestPathWithMovementDelayFlag = true; // with movement input

					string movement_id = GetMovementStringID(pLink->m_FromNodeNumber, pLink->m_ToNodeNumber, pLink->m_FromNodeNumber);
					int middle_node_id = g_NodeNametoIDMap[pLink->m_ToNodeNumber];

					g_NodeVector[middle_node_id].m_MovementMap[movement_id].turning_direction = "U-turn";
					g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_from_node_id = pLink->m_FromNodeNumber;
					g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_to_node_id = pLink->m_ToNodeNumber;
					g_NodeVector[middle_node_id].m_MovementMap[movement_id].out_link_to_node_id = pLink->m_FromNodeNumber;

					g_NodeVector[middle_node_id].m_MovementMap[movement_id].b_turning_prohibited = true;   // assign movement to individual node



				}


				pLink->CalculateShapePointRatios();
				pLink->SetupMOE();




				// tally statics for each link type
				g_LinkTypeMap[pLink->m_link_type].number_of_links++;
				g_LinkTypeMap[pLink->m_link_type].total_lane_capacity += pLink->m_LaneCapacity;
				g_LinkTypeMap[pLink->m_link_type].total_speed_limit += pLink->m_SpeedLimit;
				g_LinkTypeMap[pLink->m_link_type].total_number_of_lanes += pLink->m_OutflowNumLanes;
				g_LinkTypeMap[pLink->m_link_type].total_k_jam += pLink->m_KJam;
				g_LinkTypeMap[pLink->m_link_type].total_length += pLink->m_Length;

				g_LinkVector.push_back(pLink);
				string link_string_id = GetLinkStringID(FromID, ToID);
				g_LinkMap[link_string_id] = pLink;
				g_LinkIDMap[org_link_id] = pLink;




				i++;



				if (i % 1000 == 0)
				{
					cout << " loading " << i / 1000 << "K links..." << endl;
				}

				if (i == MAX_LINK_NO && g_AgentBasedAssignmentFlag != analysis_accessibility_distance) // g_AgentBasedAssignmentFlag == 2  -> no vehicle simulation
				{
					cout << "The network has more than " << MAX_LINK_NO << " links." << endl << "Please contact the developers for a new 64 bit version for this large-scale network." << endl;
					getchar();
					exit(0);

				}

			}
		}

		g_UnitOfMileOrKM = total_link_coordinate_length / max(0.1, total_link_length);



	}
	else
	{
		cout << "Error: File input_link.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		g_ProgramStop();
	}


	if (signal_reset_count >= 1)
	{
		cout << signal_reset_count << " nodes' control type is reset to 'no control', as they are connected to connectors. " << endl << "For example, node " << control_node_number << endl;
		getchar();
	}

	cout << " total # of links loaded = " << g_LinkVector.size() << endl;

	g_SummaryStatFile.WriteParameterValue("# of Links", g_LinkVector.size());

	if (g_SignalRepresentationFlag == signal_model_link_effective_green_time)
	{
		g_SummaryStatFile.WriteParameterValue("--Parameters for link effective green time mode --", "");

		g_SummaryStatFile.WriteParameterValue("# of Links with Effective Green Time", count_effective_green_time);

		float avg_effective_green_time = total_effective_green_time / max(1, count_effective_green_time);
		g_SummaryStatFile.WriteParameterValue("Avg Effective Green Time (sec)", avg_effective_green_time);

		float avg_lane_capacity = total_signal_link_capacity / max(1, count_effective_green_time);
		g_SummaryStatFile.WriteParameterValue("Avg Signalized Lane Capacity (vph)", avg_lane_capacity);

		float avg_cycle_length = total_signal_link_cycle_length / max(1, count_effective_green_time);
		g_SummaryStatFile.WriteParameterValue("Avg Cycle length (sec)", avg_cycle_length);

		g_SummaryStatFile.WriteParameterValue("---- ", "");

		cout << "# of Links with Effective Green Time = " << count_effective_green_time << endl;
		cout << "Avg Effective Green Time (sec) = " << avg_effective_green_time << endl;
	}
	// freeway, types


	if (g_UEAssignmentMethod == analysis_accessibility_distance)
	{
		g_AgentBasedShortestPathGeneration();
		exit(0);
	}



	// step 3.2 movement input

	g_ReadAMSMovementData();

	g_SummaryStatFile.WriteParameterValue("# of Prohibited Movements", g_number_of_prohibited_movements);
	g_SummaryStatFile.WriteParameterValue("# of Pretimed Signals", NodeControlTypeCount[g_settings.pretimed_signal_control_type_code]);
	g_SummaryStatFile.WriteParameterValue("# of Actuated Signals", NodeControlTypeCount[g_settings.actuated_signal_control_type_code]);



	//*******************************
	// step 4: zone input


	CCSVParser parser_zone;

	if (parser_zone.OpenCSVFile("input_zone.csv"))
	{

		int max_zone_number = 0;
		int i = 0;

		while (parser_zone.ReadRecord())
		{
			int zone_number;

			if (parser_zone.GetValueByFieldName("zone_id", zone_number) == false)
			{
				cout << "Error: Field zone_id does not exist in input_zone.csv" << endl;
				g_ProgramStop();
				break;
			}

			if (zone_number == 0)
			{
				cout << "Error: zone_id = 0 in input_zone.csv" << endl;
				g_ProgramStop();
			}

			if (max_zone_number < zone_number)
				max_zone_number = zone_number;

			DTAZone zone;
			std::vector<CCoordinate> CoordinateVector;
			string geo_string;
			if (parser_zone.GetValueByFieldName("geometry", geo_string, true))
			{
				// overwrite when the field "geometry" exists
				CGeometry geometry(geo_string);
				CoordinateVector = geometry.GetCoordinateList();
				for (int si = 0; si < CoordinateVector.size(); si++)
				{
					GDPoint	pt;
					pt.x = CoordinateVector[si].X;
					pt.y = CoordinateVector[si].Y;
					zone.m_ShapePoints.push_back(pt);

				}

			}

			float ODME_ratio = 1.0f;
			parser_zone.GetValueByFieldName("ODME_ratio", ODME_ratio);

			if (ODME_ratio <= 0.01 || ODME_ratio >= 100)
				ODME_ratio = 1.0;

			
			zone.m_ODME_ratio = ODME_ratio;
			zone.m_ZoneSequentialNo = i++;
			g_ZoneMap[zone_number] = zone;

		}

		if (max_zone_number == 0)
		{
			cout << "Error: 0 zone in input_zone.csv. Please check." << endl;
			g_ProgramStop();

		}

		g_ODZoneNumberSize = max_zone_number;


	}
	else
	{
		cout << "input_zone.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	cout << "   # of zones =" << g_ZoneMap.size() << endl;

	g_SummaryStatFile.WriteParameterValue("# of Zones", g_ZoneMap.size());

	// check how many centroids

	cout << "Step 5: Reading files input_activity_location.csv..." << endl;
	g_LogFile << "Step 5: Reading file input_activity_location.csv..." << g_GetUsedMemoryDataInMB() << endl;

	CCSVParser parser_activity_location;

	if (!parser_activity_location.OpenCSVFile("input_activity_location.csv"))
	{

		ofstream activity_location_file;
		activity_location_file.open("input_activity_location.csv");
		if (activity_location_file.is_open())
		{
			activity_location_file << "zone_id,node_id,external_OD_flag" << endl;

			// use each node as possible zone
			for (unsigned int i = 0; i < g_NodeVector.size(); i++)
			{
				activity_location_file << g_NodeVector[i].m_NodeNumber << "," << g_NodeVector[i].m_NodeNumber << ",0" << endl;
			}

			activity_location_file.close();
		}
	}

	int activity_location_count = 0;
	if (parser_activity_location.inFile.is_open() || parser_activity_location.OpenCSVFile("input_activity_location.csv"))
	{
		bool bMultipleZOneError = false;

		while (parser_activity_location.ReadRecord())
		{
			int zone_number;

			if (parser_activity_location.GetValueByFieldName("zone_id", zone_number) == false)
				break;

			if (zone_number == 18040)
			{
				TRACE("");
			}

			int node_number = 1;
			if (parser_activity_location.GetValueByFieldName("node_id", node_number) == false)
				break;

			g_LogFile << zone_number << " -> " << node_number << endl;

			if (g_NodeNametoIDMap.find(node_number) == g_NodeNametoIDMap.end())
			{
				cout << "node_id = " << node_number << " in file input_activity_location has not been defined in input_node.csv. Please check." << endl;
				g_ProgramStop();

			}

			int nodeid = g_NodeNametoIDMap[node_number];
			if (g_ODZoneNumberSize < zone_number)
				g_ODZoneNumberSize = zone_number;

			int external_od_flag;
			if (parser_activity_location.GetValueByFieldName("external_OD_flag", external_od_flag) == false)
				external_od_flag = 0;


			if (zone_number < 0)
			{
				cout << "zone_id = " << zone_number << " in file input_activity_location. Please check." << endl;
				g_ProgramStop();
			}

			if (external_od_flag >= 2 || external_od_flag <= -2)
			{
				cout << "Invalid external_od_flag = " << external_od_flag << " in file input_activity_location. Please check." << endl;
				g_ProgramStop();
			}

			if (g_NodeVector[g_NodeNametoIDMap[node_number]].m_ZoneID > 0)
			{
				g_WarningFile << "warning: node  " << node_number << " have been used by more than one zone in file input_activity_location. Please check." << endl;

				bMultipleZOneError = true;
			}

			g_NodeVector[g_NodeNametoIDMap[node_number]].m_ZoneID = zone_number;
			g_NodeVector[g_NodeNametoIDMap[node_number]].m_ABMZoneID = zone_number;
			

			DTANode ThisNode = g_NodeVector[g_NodeNametoIDMap[node_number]];
			if (external_od_flag != -1 && ThisNode.m_OutgoingLinkVector.size() >= 1) // not external destination
				g_ZoneMap[zone_number].m_OriginActivityVector.push_back(nodeid);
			else if (external_od_flag == 0)
			{
				g_WarningFile << "node_id " << node_number << " in input_activity_location.csv has no outgoing link, which should not defined as an origin activity node. ";
				g_WarningFile << "please set external_od_flag = -1 for node_id " << node_number << endl;

			}

			if (external_od_flag != 1 && ThisNode.m_IncomingLinkVector.size() >= 1) // not external origin
				g_ZoneMap[zone_number].m_DestinationActivityVector.push_back(nodeid);
			else if (external_od_flag == 0)
			{
				g_WarningFile << "node_id " << node_number << " in input_activity_location.csv has no incoming link, which should not defined as a destination activity node. ";
				g_WarningFile << "please set external_od_flag = 1 for node_id " << node_number << endl;

			}


			g_ZoneMap[zone_number].m_Capacity += g_NodeVector[nodeid].m_TotalCapacity;

			if (g_ZoneMap[zone_number].m_OriginActivityVector.size() > g_AdjLinkSize)
				g_AdjLinkSize = g_ZoneMap[zone_number].m_OriginActivityVector.size();

			activity_location_count++;
		}

	}
	else
	{
		cout << "input_activity_location.csv cannot be opened." << endl;
		g_ProgramStop();
	}



	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		if (iterZone->second.m_OriginActivityVector.size() > 30)
		{
			g_LogFile << "Zone " << iterZone->first << ": # of connectors:" << iterZone->second.m_OriginActivityVector.size() << endl;
		}
	}

	// after reading input_zone.csv and input_activity_location files

	int max_zone_number = 0;

	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		if (max_zone_number < iterZone->first)
			max_zone_number = iterZone->first;

		g_ZoneNo2NumberVector.push_back(iterZone->first);
	}

	g_ODZoneIDSize = g_ZoneMap.size();

	for (int i = 0; i <= max_zone_number; i++)
	{
		g_ZoneNumber2NoVector.push_back(-1);
	}


	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		iterZone->second.m_ZoneNumber = iterZone->first;
		g_ZoneNumber2NoVector[iterZone->first] = iterZone->second.m_ZoneSequentialNo;

	}


	g_SummaryStatFile.WriteParameterValue("# of Activity Locations", activity_location_count);



	//test if we have many activity locations per zone

	int max_connectors = 0;
	int max_connectors_zone_number = 0;


	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		if (iterZone->second.m_OriginActivityVector.size() > max_connectors)
		{
			max_connectors = iterZone->second.m_OriginActivityVector.size();
			max_connectors_zone_number = iterZone->first;
		}
	}

	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		if (iterZone->second.m_DestinationActivityVector.size() > max_connectors)
		{
			max_connectors = iterZone->second.m_DestinationActivityVector.size();
			max_connectors_zone_number = iterZone->first;
		}
	}


	g_LogFile << "Zone " << max_connectors_zone_number << " has a maximum of" << max_connectors << " connectors:" << endl;

	if (max_connectors_zone_number >= g_AdjLinkSize)
		g_AdjLinkSize = max_connectors_zone_number;




	//*******************************
	// step 5: vehicle type input

	cout << "Step 6: Reading file optional_vehicle_type.csv..." << endl;
	g_LogFile << "Step 6: Reading file optional_vehicle_type.csv..." << g_GetUsedMemoryDataInMB() <<endl;

	CCSVParser parser_vehicle_type;

	if (!parser_vehicle_type.OpenCSVFile("optional_vehicle_type.csv", false))
	{
		cout << "optional_vehicle_type.csv cannot be opened.  Use default values. " << endl;

		ofstream VehicleTypeFile;
		VehicleTypeFile.open("optional_vehicle_type.csv");
		if (VehicleTypeFile.is_open())
		{
			VehicleTypeFile << "vehicle_type,vehicle_type_name,percentage_of_age_0,percentage_of_age_5,percentage_of_age_10,percentage_of_age_15" << endl;
			VehicleTypeFile << "1,passenger car,25,25,25,25" << endl;
			VehicleTypeFile << "2,passenger truck,25,25,25,25" << endl;
			VehicleTypeFile << "3,light commercial truck,25,25,25,25" << endl;
			VehicleTypeFile << "4,single unit short-haul truck,25,25,25,25" << endl;
			VehicleTypeFile << "5,combination long-haul truck,25,25,25,25" << endl;
			VehicleTypeFile.close();
		}
	}

	if (parser_vehicle_type.inFile.is_open() || parser_vehicle_type.OpenCSVFile("optional_vehicle_type.csv",false))
	{
		g_VehicleTypeVector.clear();
		while (parser_vehicle_type.ReadRecord())
		{
			int vehicle_type = 0;
			if (parser_vehicle_type.GetValueByFieldName("vehicle_type", vehicle_type) == false)
				break;

			string vehicle_type_name;
			parser_vehicle_type.GetValueByFieldName("vehicle_type_name", vehicle_type_name);




			DTAVehicleType element;
			element.vehicle_type = vehicle_type;
			element.vehicle_type_name = vehicle_type_name;

			double PCE = 1.0;
			parser_vehicle_type.GetValueByFieldName("PCE", element.PCE);

			if (element.PCE < 0.01)
				element.PCE = 1; //default value: 1.0



			parser_vehicle_type.GetValueByFieldName("rolling_term_a", element.rollingTermA);
			parser_vehicle_type.GetValueByFieldName("rotating_term_b", element.rotatingTermB);
			parser_vehicle_type.GetValueByFieldName("drag_term_c", element.dragTermC);
			parser_vehicle_type.GetValueByFieldName("source_mass", element.sourceMass);


			float percentage_of_age = 0;

			int age = 0;

			// initialize age vector from 0 year to 30 year
			for (age = 0; age <= 30; age++)
			{
				element.percentage_age_vector.push_back(0);
			}

			for (age = 0; age <= 30; age++)
			{
				CString str_age;
				str_age.Format("percentage_of_age_%d", age);

				CT2CA pszConvertedAnsiString(str_age);
				// construct a std::string using the LPCSTR input
				std::string strStd(pszConvertedAnsiString);

				if (parser_vehicle_type.GetValueByFieldName(strStd, percentage_of_age) == true) // with data
				{
					element.percentage_age_vector[age] = percentage_of_age;

				}

			}


			g_VehicleTypeVector.push_back(element);

		}

	}
	else
	{
		if(g_EmissionDataOutputFlag==1)
		{
		cout << "optional_vehicle_type.csv cannot be opened. " << endl;
		g_ProgramStop();
		}
	}
	g_SummaryStatFile.WriteParameterValue("# of Vehicle Types", g_VehicleTypeVector.size());


	//*******************************
	// step 6: demand type input

	CCSVParser parser_demand_type;
	cout << "Step 7.1: Reading file input_demand_type.csv..." << endl;
	g_LogFile << "Step 7.1: Reading file input_demand_type.csv..." << g_GetUsedMemoryDataInMB() << endl;
	if (!parser_demand_type.OpenCSVFile("input_demand_type.csv"))
	{
		cout << "input_demand_type.csv cannot be opened. " << endl;
		g_ProgramStop();

	}

	if (parser_demand_type.inFile.is_open() || parser_demand_type.OpenCSVFile("input_demand_type.csv"))
	{
		g_DemandTypeVector.clear();
		while (parser_demand_type.ReadRecord())
		{
			int demand_type = 1;
			float average_VOT = 10;

			if (parser_demand_type.GetValueByFieldName("demand_type", demand_type) == false)
				break;

			if (demand_type != g_DemandTypeVector.size() + 1)
			{
				cout << "Error: demand_type = " << demand_type << " in file input_demand_type.csv should be a sequential number: " << g_DemandTypeVector.size() + 1 << endl << "Please check!";

				g_ProgramStop();
			}

			if (demand_type >= MAX_DEMAND_TYPE_SIZE)
			{
				cout << "Error: demand_type = " << demand_type << " in file input_demand_type.csv is too large. " << "MAX_DEMAND_TYPE_SIZE = " << MAX_DEMAND_TYPE_SIZE << "Please contact program developers!";

				g_ProgramStop();
			}


			float ratio_pretrip = 0;
			float ratio_enroute = 0;

			float ratio_person_info = 0;
			float ratio_ecoSO = 0;

			float Avg_VOT = 12;
			parser_demand_type.GetValueByFieldNameRequired("avg_VOT", Avg_VOT);


			parser_demand_type.GetValueByFieldName("percentage_of_pretrip_info", ratio_pretrip);
			parser_demand_type.GetValueByFieldName("percentage_of_enroute_info", ratio_enroute);

			parser_demand_type.GetValueByFieldName("percentage_of_personalized_info", ratio_person_info);
			parser_demand_type.GetValueByFieldName("percentage_of_ecoso_info", ratio_ecoSO);

			DemandType element;
			element.demand_type = demand_type;

			parser_demand_type.GetValueByFieldName("demand_type_name", element.demand_type_name);

			element.Avg_VOT = Avg_VOT;

			element.info_class_percentage[1] = 0;  //learning 
			element.info_class_percentage[2] = ratio_pretrip;
			element.info_class_percentage[3] = ratio_enroute;

			element.info_class_percentage[4] = ratio_person_info;
			element.info_class_percentage[5] = ratio_ecoSO;

			element.info_class_percentage[0] = 100 - ratio_enroute - ratio_pretrip - ratio_person_info - ratio_ecoSO;


			if (ratio_pretrip > 0 || ratio_enroute > 0)
			{
				if (g_NumberOfIterations >= 1)
				{
					cout << "Please use 1 iteration when pre-trip and en route information is enabled." << endl;
					g_ProgramStop();
				}
			}
			for (int ic = 1; ic < MAX_INFO_CLASS_SIZE; ic++)
			{
				element.cumulative_info_class_percentage[ic] = element.cumulative_info_class_percentage[ic - 1] + element.info_class_percentage[ic];
			}
			for (int i = 0; i < g_VehicleTypeVector.size() - 1; i++)  // the vehicle type here is dynamically determined from th rea
			{
				std::ostringstream  str_percentage_of_vehicle_type;
				str_percentage_of_vehicle_type << "percentage_of_vehicle_type" << i + 1;

				float percentage_vehicle_type = 0;
				if (parser_demand_type.GetValueByFieldName(str_percentage_of_vehicle_type.str(), percentage_vehicle_type) == false)
				{
					cout << "Error: Field percentage_of_vehicle_type " << i + 1 << " cannot be found in the input_demand_type.csv file.";
					cout << "In file optional_vehicle_type.csv, " << g_VehicleTypeVector.size() << " have been defined, so input_demand_type.csv should percentage_of_vehicle_type for all vehicle types.";


					g_ProgramStop();
					return;
				}
				else
				{
					element.vehicle_type_percentage[i + 1] = percentage_vehicle_type;

					element.cumulative_type_percentage[i + 1] = element.cumulative_type_percentage[i] + percentage_vehicle_type;

				}
			}

			g_DemandTypeVector.push_back(element);

		}

	}
	else
	{
		cout << "Error: File input_demand_type.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		g_ProgramStop();
	}

	g_SummaryStatFile.WriteParameterValue("# of Demand Types", g_DemandTypeVector.size());

	


	// done with zone.csv

	if (g_TrafficFlowModelFlag != tfm_BPR && g_TrafficFlowModelFlag != tfm_point_queue)
	{
		DTANetworkForSP PhysicalNetwork(g_NodeVector.size(), g_LinkVector.size(), g_PlanningHorizon, g_AdjLinkSize);  //  network instance for single processor in multi-thread environment
		PhysicalNetwork.Setup(g_NodeVector.size(), g_LinkVector.size(), g_PlanningHorizon, g_AdjLinkSize, g_DemandLoadingStartTimeInMin);
		PhysicalNetwork.BuildPhysicalNetwork(0, 0, g_TrafficFlowModelFlag, true);
		PhysicalNetwork.IdentifyBottlenecks(g_StochasticCapacityMode);
	}
	//	ConnectivityChecking(&PhysicalNetwork);

	g_ReadLinkMeasurementFile();
	g_SummaryStatFile.WriteParameterValue("# of Sensor Records", g_SensorDataCount);

	if (g_SensorDataCount == 0 && g_UEAssignmentMethod == analysis_OD_demand_estimation)
	{
		cout << "OD estimation mode but no sensor data is input. Please check." << endl;
		g_ProgramStop();
	}
	//*******************************
	// step 10: demand trip file input

	// initialize the demand loading range, later resized by CreateVehicles
	if (g_UEAssignmentMethod != analysis_real_time_simulation &&
		g_UEAssignmentMethod != analysis_accessibility_distance)
	{

		g_SummaryStatFile.WriteTextLabel("Demand Load Mode=,demand meta database");

		////////////////////////////////////// VOT

		cout << "Step 10: Reading files based on user settings in meta database file..." << endl;
		g_LogFile << "Step 10: Reading files  based on user settings in  meta database file..." << g_GetUsedMemoryDataInMB() << endl;
	}


	int LinkSizeForLinkCostArray = g_LinkVector.size() + g_NodeVector.size(); // double the size to account for artificial connectors
	g_LinkTDCostAry = AllocateDynamicArray<DTALinkToll>(LinkSizeForLinkCostArray, g_NumberOfSPCalculationPeriods);

	g_ShortestPathDataMemoryAllocation();

	if (g_UEAssignmentMethod != analysis_real_time_simulation)
	{
		if ( g_UEAssignmentMethod == analysis_accessibility_distance)
		{
			// do not load demand data
		}
		else
		{
			//------------MODIFIED BY QU 10.29----------------------------//
			//*******************************
			// step 12: transit trip file input
			cout << "Step 12.1: Reading transit trip file..." << endl;
			//------------------------------------------------------------//
			g_ReadTransitTripCSVFile(); // read the transit schedule first // later on for optimization 


			//------------MODIFIED BY QU 10.29----------------------------//
			//*******************************
			// step 12: reading/initialing agent file input
			cout << "Step 12.2: Reading/Initializing agent trip file..." << endl;
			//------------------------------------------------------------//
	/*		if (g_UEAssignmentMethod == analysis_vehicle_binary_file_based_scenario_evaluation)
			{
				g_ReadAgentBinFile("agent.bin", true);
			}
			else if (g_UEAssignmentMethod == analysis_system_optimal)
			{
				g_ReadAgentBinFile("agent.bin", true);
			}
			else if (g_UEAssignmentMethod == analysis_metro_sim)
			{
				g_ReadAgentBinFile("agent.bin", false);
			}
			else*/
			{  // this is the common mode for loading demand files using demand meta database. 
				g_ReadDemandFileBasedOnDemandFileList();
			}


		}


	}
	else
	{

		g_AllocateDynamicArrayForVehicles();
	}

	if (g_PlanningHorizon < g_DemandLoadingEndTimeInMin + 300)
	{
		//reset simulation horizon to make sure it is longer than the demand loading horizon
		g_PlanningHorizon = g_DemandLoadingEndTimeInMin + 300;

		for (unsigned link_index = 0; link_index < g_LinkVector.size(); link_index++)
		{
			DTALink* pLink = g_LinkVector[link_index];
			pLink->ResizeData(g_PlanningHorizon);
		}

	}

	//use the user defined travel time 
	//g_ReadInputLinkTravelTime();

	g_LogFile << " -- zone-specific demand data -- " << endl;
	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{

		if (iterZone->second.m_Demand > 0.01)
		{
			g_LogFile << "Zone " << iterZone->first << ",demand =" << iterZone->second.m_Demand <<
				", agent count =" << iterZone->second.m_OriginVehicleSize << endl;
		}
	}


	//*******************************
	// step 9: Crash Prediction input


	//	cout << "Global Loading Factor = "<< g_DemandGlobalMultiplier << endl;


	cout << "Number of Zones = " << g_ODZoneNumberSize << endl;
	cout << "Number of Nodes = " << g_NodeVector.size() << endl;
	cout << "Number of Links = " << g_LinkVector.size() << endl;

	cout << "Number of Vehicles to be Simulated = " << g_VehicleVector.size() << endl;
	cout << "Demand Loading Period = " << g_DemandLoadingStartTimeInMin << " min -> " << g_DemandLoadingEndTimeInMin << " min." << endl;

	g_SummaryStatFile.WriteParameterValue("\nDemand multiplier in input_scenario_settings.csv file", g_DemandGlobalMultiplier);

	g_SummaryStatFile.WriteParameterValue("# of Vehicles to be simulated", g_VehicleVector.size());
	g_SummaryStatFile.WriteParameterValue("# of Intra-zone Vehicles (not be simulated)", g_number_of_intra_zone_trips);


	if (g_UEAssignmentMethod == analysis_ABM_integration)
	{

		CCSVParser parser_RTSimulation_settings;
			_proxy_ABM_log(0, "There are %d agents in the memory.\n", g_VehicleVector.size());
	}



	g_SummaryStatFile.WriteTextLabel("Starting Time of Demand Loading (min)=,");
	g_SummaryStatFile.WriteNumber(g_DemandLoadingStartTimeInMin);

	g_SummaryStatFile.WriteTextLabel("Ending Time of Demand Loading (min)=,");
	g_SummaryStatFile.WriteNumber(g_DemandLoadingEndTimeInMin);

	g_SummaryStatFile.WriteTextLabel("Output Emission data=,");
	if (g_EmissionDataOutputFlag >= 1)
		g_SummaryStatFile.WriteTextLabel("YES\n");
	else
		g_SummaryStatFile.WriteTextLabel("NO\n");

	if (g_EmissionDataOutputFlag >= 1)
	{
		g_SummaryStatFile.WriteTextLabel("Output second by second speed data=,");
		if (g_OutputSecondBySecondEmissionData)
			g_SummaryStatFile.WriteTextLabel("YES\n");
		else
			g_SummaryStatFile.WriteTextLabel("NO\n");

		g_SummaryStatFile.WriteParameterValue("Vehicle ID with second by second speed data", g_TargetVehicleID_OutputSecondBySecondEmissionData);
	}




//	g_WritePrivateProfileInt("output", "simulation_data_horizon_in_min", g_PlanningHorizon, g_DTASettingFileName);
	// 
	g_NetworkMOEAry.clear();

	for (int time = 0; time <= g_PlanningHorizon; time++)
	{
		NetworkMOE element;
		g_NetworkMOEAry.push_back(element);
	}

	g_AssignmentMOEVector.clear();
	for (int iter = 0; iter <= g_NumberOfIterations; iter++)
	{
		NetworkLoadingOutput element;
		g_AssignmentMOEVector.push_back(element);
	}


	cout << "Number of Vehicle Types = " << g_VehicleTypeVector.size() << endl;
	cout << "Number of Demand Types = " << g_DemandTypeVector.size() << endl;

	g_LogFile << "Number of Zones = " << g_ODZoneNumberSize << endl;
	g_LogFile << "Number of Nodes = " << g_NodeVector.size() << endl;
	g_LogFile << "Number of Links = " << g_LinkVector.size() << endl;
	g_LogFile << "Number of Vehicles to be Simulated = " << g_VehicleVector.size() << endl;
	g_LogFile << "Demand Loading Period = " << g_DemandLoadingStartTimeInMin << " min -> " << g_DemandLoadingEndTimeInMin << " min." << endl;
	g_LogFile << "Number of Vehicle Types = " << g_VehicleTypeVector.size() << endl;
	g_LogFile << "Number of Demand Types = " << g_DemandTypeVector.size() << endl;

	// link type summary

	CString title_str;
	title_str.Format("\n--Link Type Statistics--");
	g_SummaryStatFile.WriteTextString(title_str);
	g_SummaryStatFile.Reset();

	g_SummaryStatFile.SetFieldName("link_type");
	g_SummaryStatFile.SetFieldName("link_type_name");
	g_SummaryStatFile.SetFieldName("link_type_code");
	g_SummaryStatFile.SetFieldName("number_of_links");
	g_SummaryStatFile.SetFieldName("avg_lane_capacity");
	g_SummaryStatFile.SetFieldName("avg_number_of_lanes");
	g_SummaryStatFile.SetFieldName("avg_speed_limit");
	g_SummaryStatFile.SetFieldName("avg_link_length");
	g_SummaryStatFile.SetFieldName("avg_K_jam");
	g_SummaryStatFile.SetFieldName("total_link_length");

	g_SummaryStatFile.WriteHeader(false, false);

	for (std::map<int, DTALinkType>::iterator itr = g_LinkTypeMap.begin(); itr != g_LinkTypeMap.end(); itr++)
	{
		std::replace(itr->second.link_type_name.begin(), itr->second.link_type_name.end(), ',', ' ');

		g_SummaryStatFile.SetValueByFieldName("link_type", itr->first);
		std::string link_type_name = itr->second.link_type_name.c_str();
		std::string type_code = itr->second.type_code.c_str();
		g_SummaryStatFile.SetValueByFieldName("link_type_name", link_type_name);
		g_SummaryStatFile.SetValueByFieldName("link_type_code", type_code);
		g_SummaryStatFile.SetValueByFieldName("number_of_links", itr->second.number_of_links);

		float avg_lane_capacity = itr->second.total_lane_capacity / max(1, itr->second.number_of_links);
		float avg_number_of_lanes = itr->second.total_number_of_lanes / max(1, itr->second.number_of_links);
		float avg_speed_limit = itr->second.total_speed_limit / max(1, itr->second.number_of_links);
		float avg_K_jam = itr->second.total_k_jam / max(1, itr->second.number_of_links);
		float avg_link_length = itr->second.total_length / max(1, itr->second.number_of_links);
		float total_link_length = avg_link_length * itr->second.number_of_links;

		g_SummaryStatFile.SetValueByFieldName("avg_lane_capacity", avg_lane_capacity);
		g_SummaryStatFile.SetValueByFieldName("avg_number_of_lanes", avg_number_of_lanes);
		g_SummaryStatFile.SetValueByFieldName("avg_speed_limit", avg_speed_limit);
		g_SummaryStatFile.SetValueByFieldName("avg_K_jam", avg_K_jam);
		g_SummaryStatFile.SetValueByFieldName("avg_link_length", avg_link_length);
		g_SummaryStatFile.SetValueByFieldName("total_link_length", total_link_length);

		g_SummaryStatFile.WriteRecord();
	}

	int use_link_travel_time_from_external_input = g_GetPrivateProfileInt("assignment", "use_link_travel_time_from_external_input", 0, g_DTASettingFileName);
	float g_gain_factor_link_travel_time_from_external_input = g_GetPrivateProfileFloat("assignment", "use_link_travel_time_from_external_input_gain_factor", 0.5, g_DTASettingFileName);

	//if (use_link_travel_time_from_external_input)
	//{
	//	// read input link travel times
	//	g_ReadInputLinkTravelTime_Parser();
	//}

	if (g_TrafficFlowModelFlag != tfm_BPR)  // if this is not BRP model, we require emission files 
	{
		// Xuesong Zhou to do: 
		//if( g_EmissionDataOutputFlag == 1)
		//{ 
		//	ReadInputEmissionRateFile();

		//}

		//ReadInputCycleAverageEmissionFactors();

		//ReadFractionOfOperatingModeForBaseCycle();
		//SetupOperatingModeVector();
	}


}
// debug:


int CreateVehicles(int origin_zone, int destination_zone, float number_of_vehicles, int demand_type, float starting_time_in_min, float ending_time_in_min, int PathIndex, bool bChangeHistDemandTable, int DepartureTimeIndex)
{
	if (origin_zone == destination_zone)  // do not simulate intra-zone traffic
	{
		g_number_of_intra_zone_trips += number_of_vehicles;
		return 0;
	}
	// reset the range of demand loading interval
	
	//apply ODME factor
	number_of_vehicles *= g_ZoneMap[origin_zone].m_ODME_ratio;
	

	int number_of_vehicles_generated = g_GetRandomInteger_SingleProcessorMode(number_of_vehicles);
	//g_LogFile << "r," << number_of_vehicles << "," << number_of_vehicles_generated << ",";


	for (int i = 0; i < number_of_vehicles_generated; i++)
	{

		if (g_DemandCapacityScalingFactor < 0.999 && g_DemandCapacityScalingFactor >= 0.0001)  // valid ratio global
		{
			double random_ratio = g_GetRandomRatio();

			if (random_ratio > g_DemandCapacityScalingFactor)
				continue; // skip reading 
		}


		DTA_vhc_simple vhc;
		vhc.m_OriginZoneID = origin_zone;
		vhc.m_DestinationZoneID = destination_zone;
		vhc.m_PathIndex = PathIndex;
		vhc.m_DepartureTimeIndex = DepartureTimeIndex;

		g_ZoneMap[origin_zone].m_OriginVehicleSize++;

		// generate the random departure time during the interval
		float RandomRatio = 0;
		if (number_of_vehicles_generated < 10) // small demand volume
		{
			RandomRatio = g_GetRandomRatioForVehicleGeneration(); // use random number twice to determine exact departure time of vehicles
		}
		else
		{
			RandomRatio = (i + 1)*1.0f / (number_of_vehicles_generated + 1); // uniform distribution
		}

		vhc.m_DepartureTime = starting_time_in_min + RandomRatio*(ending_time_in_min - starting_time_in_min);

		if (vhc.m_DepartureTime >= 1000)
			TRACE("");

		// important notes: to save memory and for simplicity, DTALite simulation still uses interval clock starts from as 0
		// when we output the all statistics, we need to 
		vhc.m_DemandType = demand_type;

		if (g_ODEstimationFlag == 1)  // do not change hist demand when creating vehicles in the middle of  ODME , called by  g_GenerateVehicleData_ODEstimation()
		{
			int time_interval = g_FindAssignmentIntervalIndexFromTime(vhc.m_DepartureTime);

			if (bChangeHistDemandTable == true)
				g_SystemDemand.AddValue(origin_zone, destination_zone, time_interval, 1); // to store the initial table as hist database
		}

		g_GetVehicleAttributes(vhc.m_DemandType, vhc.m_VehicleType, vhc.m_InformationType, vhc.m_VOT, vhc.m_Age);


		g_SimulationResult.number_of_vehicles_DemandType[vhc.m_DemandType]++;

		g_simple_vector_vehicles.push_back(vhc);
	}

	return number_of_vehicles_generated;
}

void g_print_out_zone_activity_location(int ZoneID)
{

	cout << "Zone " << ZoneID << " has the following OriginActivityVector. " << endl;
	for (int i = 0; i < g_ZoneMap[ZoneID].m_OriginActivityVector.size(); i++)
	{
		cout << g_NodeVector[g_ZoneMap[ZoneID].m_OriginActivityVector[i]].m_NodeNumber << endl;

	}
	cout << "Zone " << ZoneID << " has the following DestinationActivityVector. " << endl;
	for (int i = 0; i < g_ZoneMap[ZoneID].m_DestinationActivityVector.size(); i++)
	{
		cout << g_NodeVector[g_ZoneMap[ZoneID].m_DestinationActivityVector[i]].m_NodeNumber << endl;

	}
}


void g_ConvertDemandToVehicles()
{
	int SkipDemandError = 0;
	std::map<int, int> g_NoActivityZoneMap;
	cout << "Total number of vehicles to be simulated = " << g_simple_vector_vehicles.size() << endl;
	g_LogFile << "Total number of vehicles to be simulated = " << g_simple_vector_vehicles.size() << endl;

	std::sort(g_simple_vector_vehicles.begin(), g_simple_vector_vehicles.end());

	std::vector<DTA_vhc_simple>::iterator kvhc = g_simple_vector_vehicles.begin();

	DTAVehicle* pVehicle = 0;

	int i = 0;

	int max_agent_id = 0;

	for (int v = 0; v < g_VehicleVector.size(); v++)
	{
		if (max_agent_id < g_VehicleVector[v]->m_AgentID + 1)
			max_agent_id = g_VehicleVector[v]->m_AgentID + 1;
	}


	while (kvhc != g_simple_vector_vehicles.end())
	{
		if (kvhc->m_OriginZoneID != kvhc->m_DestinationZoneID)    // only consider intra-zone traffic
		{

			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			pVehicle->m_AgentID = max_agent_id+ i;
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			if (pVehicle->m_AgentID >= 21)
			{
				TRACE("");
			}


			pVehicle->m_OriginZoneID = kvhc->m_OriginZoneID;
			pVehicle->m_DestinationZoneID = kvhc->m_DestinationZoneID;

			if (g_ZoneMap[kvhc->m_OriginZoneID].m_OriginActivityVector.size() == 0)
			{
				g_NoActivityZoneMap[kvhc->m_OriginZoneID] = 1;
				kvhc++;
				delete pVehicle;
				pVehicle = NULL;
				continue;
			}

			if (g_ZoneMap[kvhc->m_DestinationZoneID].m_DestinationActivityVector.size() == 0)
			{
				g_NoActivityZoneMap[kvhc->m_DestinationZoneID] = 1;
				kvhc++;

				delete pVehicle;
				pVehicle = NULL;
				continue;
			}

			pVehicle->m_OriginNodeID = g_ZoneMap[kvhc->m_OriginZoneID].GetRandomOriginNodeIDInZone((pVehicle->m_AgentID % 100) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily
			pVehicle->m_DestinationNodeID = g_ZoneMap[kvhc->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_AgentID % 100) / 100.0f);


			if (pVehicle->m_OriginNodeID == -1)
			{

				g_NoActivityZoneMap[kvhc->m_OriginZoneID] = 1;
				kvhc++;
				cout << "Origin zone " << kvhc->m_OriginZoneID << " has no activity location defined in input_activity_location.csv. vehicle id = " << pVehicle->m_AgentID << endl;
				g_print_out_zone_activity_location(kvhc->m_OriginZoneID);
				getchar();

				continue;
			}

			if (pVehicle->m_DestinationNodeID == -1)
			{

				g_NoActivityZoneMap[kvhc->m_DestinationZoneID] = 1;
				cout << "Destination zone " << kvhc->m_DestinationZoneID << " has no activity location defined in input_activity_location.csv. vehicle id = " << pVehicle->m_AgentID << endl;
				g_print_out_zone_activity_location(kvhc->m_DestinationZoneID);
				getchar();

				kvhc++;

				continue;
			}

			pVehicle->m_DepartureTime = kvhc->m_DepartureTime;
			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;

			pVehicle->m_DemandType = kvhc->m_DemandType;
			pVehicle->m_DemandType = kvhc->m_DemandType;
			pVehicle->m_VehicleType = kvhc->m_VehicleType;

			pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType - 1].PCE;

			pVehicle->m_InformationType = kvhc->m_InformationType;
			pVehicle->m_VOT = kvhc->m_VOT;
			pVehicle->m_Age = kvhc->m_Age;

			pVehicle->m_NodeSize = 0;  // initialize NodeSize as o
			g_VehicleVector.push_back(pVehicle);
			g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
			g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

			int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);
			g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);
			i++;
		}
		kvhc++;

		if (i % 10000 == 0 && i > 0)
			cout << "Generating " << i << " agents... " << endl;

	}

	if (g_NoActivityZoneMap.size() >= 1)
	{
		int b_ignore_demand_without_activity_location = g_GetPrivateProfileInt("assignment", "ignore_demand_without_activity_location", 1, g_DTASettingFileName);

		if (b_ignore_demand_without_activity_location == 0)
		{
			cout << "The following zones have no activity location but have demand being defined in OD demand files." << endl;

		}
		fprintf(g_DebugLogFile, "The following zones have no activity location but have demand being defined in OD demand files.\n");
		for (std::map<int, int>::iterator iterZone = g_NoActivityZoneMap.begin(); iterZone != g_NoActivityZoneMap.end(); iterZone++)
		{
			if (b_ignore_demand_without_activity_location == 0)
			{
				cout << iterZone->first << endl;
			}
			fprintf(g_DebugLogFile, "%d\n", iterZone->first);

		}

		if (b_ignore_demand_without_activity_location == 0)
		{
			cout << "Please check input_activity_location.csv and demand files. You might need to manually prepare zone-to-node mapping in file input_activity_location file to ensure each zone has at least one activity node. " << endl;
			cout << "Please any key to continue the simulation without consider vehicles associated with those zones. " << endl << " Please check error.log for more information." << endl;
			getchar();
		}
		fprintf(g_DebugLogFile, "Please check input_activity_location.csv and demand files.You might need to manually prepare zone - to - node mapping in file input_activity_location file to ensure each zone has at least one activity node. ");

		fprintf(g_DebugLogFile, "Please any key to continue the simulation without consider vehicles associated with those zones. ");

	}


	cout << "Total number of vehicles to be simulated = " << g_simple_vector_vehicles.size() << endl;
	g_simple_vector_vehicles.clear();
}


void g_ReadInputLinkTravelTime_Parser()
{
	int line_no = 0;

	CCSVParser parser_link_travel_time;

	if (parser_link_travel_time.OpenCSVFile("internal_link_travel_time.csv", false))
	{

		while (parser_link_travel_time.ReadRecord())
		{
			int from_node_id, to_node_id;

			int starting_time_in_min = 0;
			int ending_time_in_min = 1440;
			float travel_time_in_min = 0;

			if (parser_link_travel_time.GetValueByFieldName("from_node_id", from_node_id) == false)
				break;
			if (parser_link_travel_time.GetValueByFieldName("to_node_id", to_node_id) == false)
				break;


			if (parser_link_travel_time.GetValueByFieldName("starting_time_in_min", starting_time_in_min) == false)
				break;
			if (parser_link_travel_time.GetValueByFieldName("ending_time_in_min", ending_time_in_min) == false)
				break;

			if (parser_link_travel_time.GetValueByFieldName("travel_time_in_min", travel_time_in_min) == false)
				break;

			if (g_LinkMap.find(GetLinkStringID(from_node_id, to_node_id)) == g_LinkMap.end())
			{
				cout << "Link " << from_node_id << "-> " << to_node_id << " at line " << line_no + 1 << " of file input_link_travel_time.csv" << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}

			DTALink* pLink = g_LinkMap[GetLinkStringID(from_node_id, to_node_id)];

			if (pLink != NULL)
			{

				if (starting_time_in_min >= 0 && starting_time_in_min < ending_time_in_min && ending_time_in_min < pLink->m_LinkMOEAry.size()
					&& travel_time_in_min >= 0.01)
				{
					for (int t = starting_time_in_min; t < ending_time_in_min; t++)
					{
						pLink->m_LinkMOEAry[t].UserDefinedTravelTime_in_min = travel_time_in_min;
					}
				}
			}

			if (line_no % 100000 == 0)
			{
				cout << g_GetAppRunningTime() << "Reading " << line_no / 1000 << "K lines..." << endl;
			}
			line_no++;

		}
	}
	cout << " total # of link travel time records loaded = " << line_no << endl;
	g_SummaryStatFile.WriteParameterValue("# of Input Link Travel Time Records", line_no);

}

void g_CreateLinkTollVector()
{

	// transfer toll set to dynamic toll vector (needed for multi-core computation)
	int count = 0;
	for (unsigned li = 0; li< g_LinkVector.size(); li++)
	{

		if (g_LinkVector[li]->pTollVector != NULL)
			delete g_LinkVector[li]->pTollVector;

		if (g_LinkVector[li]->TollVector.size() >0)
		{
			g_LinkVector[li]->m_TollSize = g_LinkVector[li]->TollVector.size();
			g_LinkVector[li]->pTollVector = new Toll[g_LinkVector[li]->m_TollSize];

			for (int s = 0; s < g_LinkVector[li]->m_TollSize; s++)
			{
				g_LinkVector[li]->pTollVector[s] = g_LinkVector[li]->TollVector[s];
			}


			count++;
		}

	}
}

void ReadIncidentScenarioFile(string FileName, int scenario_no)
{

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->IncidentCapacityReductionVector.clear(); // remove all previouly read records

	}


	FILE* st = NULL;

	fopen_s(&st, FileName.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << FileName << " ..." << endl;
		g_LogFile << "Reading file " << FileName << endl;
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
				int local_scenario_no = g_read_integer(st);
				CapacityReduction cs;
				cs.StartDayNo = g_read_integer(st) - 1; // start from zero
				cs.EndDayNo = cs.StartDayNo;

				if (cs.StartDayNo < 0) cs.StartDayNo = 0;
				if (cs.EndDayNo < 0) cs.EndDayNo = 0;

				if (cs.StartDayNo < 0)
				{
					cout << "Start day = " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Incident.csv. Please ensure Start Day >=0." << endl;
					g_ProgramStop();
				}

				cs.StartTime = g_read_integer(st);
				cs.EndTime = g_read_integer(st);

				if (cs.StartTime >= cs.EndTime)
				{
					cout << "End Time = " << cs.EndTime << " < Start Time " << cs.StartTime << " in line " << count + 1 << "in file Scenario_Incident.csv. Please ensure End Time >= Start Time." << endl;
					g_ProgramStop();
				}


				float percentage = g_read_float(st);

				cs.LaneClosureRatio = percentage / 100.f; // percentage -> to ratio


				if (cs.LaneClosureRatio > 1.0)
					cs.LaneClosureRatio = 1.0;
				if (cs.LaneClosureRatio < 0.0)
					cs.LaneClosureRatio = 0.0;
				cs.SpeedLimit = g_read_float(st);

				if (cs.SpeedLimit < -0.1f)
				{
					cout << "Speed Limit = " << cs.SpeedLimit << " % in line " << count + 1 << " in file Scenario_Incident.csv. Please check!" << endl;
					g_ProgramStop();

				}


					pLink->IncidentCapacityReductionVector.push_back(cs);
					count++;

			}
		}

		g_LogFile << "incident records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("\n# of incident records=");
		g_SummaryStatFile.WriteNumber(count);

		fclose(st);
	}
	

}

void ReadWorkZoneScenarioFile(string FileName, int scenario_no)
{
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->CapacityReductionVector.clear(); // remove all previouly read records

	}

	FILE* st = NULL;
	fopen_s(&st, FileName.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;
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
				if (pLink->m_FromNodeNumber == 12730 && pLink->m_ToNodeNumber == 12742)
				{
					TRACE("");
				}

				int local_scenario_no = g_read_integer(st);

				CapacityReduction cs;
				cs.StartDayNo = g_read_integer(st) - 1; // start from zero
				cs.EndDayNo = g_read_integer(st) - 1;

				if (cs.StartDayNo < 0) cs.StartDayNo = 0;
				if (cs.EndDayNo < 0) cs.EndDayNo = 0;

				if (cs.EndDayNo < cs.StartDayNo)
				{
					cout << "End day = " << cs.EndDayNo << " < Start day " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Day >= Start Day." << endl;
					g_ProgramStop();
				}


				cs.StartTime = g_read_float(st);
				cs.EndTime = g_read_float(st);

				if (cs.StartTime >= cs.EndTime)
				{
					cout << "End Time = " << cs.EndTime << " < Start Time " << cs.StartTime << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Time >= Start Time." << endl;
					g_ProgramStop();
				}

				float percentage = g_read_float(st);

				if (percentage < -0.1f)
				{
					cout << "Capacity Reduction Percentage = " << percentage << " % in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please check!" << endl;

					g_ProgramStop();

				}


				cs.LaneClosureRatio = percentage / 100.0f; // percentage -> to ratio
				if (cs.LaneClosureRatio > 1.0)
					cs.LaneClosureRatio = 1.0;
				if (cs.LaneClosureRatio < 0.0)
					cs.LaneClosureRatio = 0.0;
				cs.SpeedLimit = g_read_float(st);

				if (cs.SpeedLimit < -0.1f)
				{
					cout << "Speed Limit = " << cs.SpeedLimit << " % in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please check!" << endl;

					g_ProgramStop();

				}

					pLink->CapacityReductionVector.push_back(cs);
					count++;

			}
		}

		g_LogFile << "work zone records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of work zone records=");
		g_SummaryStatFile.WriteNumber(count);


		fclose(st);
	}
	

}
void ReadGenericTrafficControlScenarioFile(string FileName, int scenario_no)
{


	FILE* st = NULL;
	fopen_s(&st, FileName.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;
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
				if (pLink->m_FromNodeNumber == 12730 && pLink->m_ToNodeNumber == 12742)
				{
					TRACE("");
				}

				int local_scenario_no = g_read_integer(st);

				CapacityReduction cs;
				cs.StartDayNo = g_read_integer(st) - 1; // start from zero
				cs.EndDayNo = g_read_integer(st) - 1;

				if (cs.StartDayNo < 0) cs.StartDayNo = 0;
				if (cs.EndDayNo < 0) cs.EndDayNo = 0;

				if (cs.EndDayNo < cs.StartDayNo)
				{
					cout << "End day = " << cs.EndDayNo << " < Start day " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Ramp_Meter.csv. Please ensure End Day >= Start Day." << endl;
					g_ProgramStop();
				}

				int hour1= g_read_integer(st);
				int min1 = g_read_integer(st);
				int second1 = g_read_integer(st);


				cs.StartTime = hour1 * 60 + min1 + second1 / 60.0;

				int hour2 = g_read_integer(st);
				int min2 = g_read_integer(st);
				int second2 = g_read_integer(st);

				cs.EndTime = hour2 * 60 + min2 + second2 / 60.0;


				if (cs.StartTime >= cs.EndTime)
				{
					cout << "End Time = " << cs.EndTime << " < Start Time " << cs.StartTime << " in line " << count + 1 << "in file Scenario_Ramp_Meter.csv. Please ensure End Time >= Start Time." << endl;
					g_ProgramStop();
				}

				float service_rate = g_read_float(st);

				if (service_rate < 1.1f || service_rate > 4000)
				{
					cout << "Lane Capacity (vph) = " << service_rate << " % in line " << count + 1 << "in file Scenario_Ramp_Meter.csv. Please check!" << endl;

					g_ProgramStop();

				}


				cs.LaneClosureRatio = (1 - service_rate / pLink->m_LaneCapacity);

				cs.SpeedLimit = g_read_float(st);

				if (cs.SpeedLimit < -0.1f)
				{
					cout << "Speed Limit = " << cs.SpeedLimit << " % in line " << count + 1 << "in file Scenario_Ramp_Meter.csv. Please check!" << endl;

					g_ProgramStop();

				}

					pLink->CapacityReductionVector.push_back(cs);
					count++;

			}
		}

		g_LogFile << "generic traffic control records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of generic traffic control records=");
		g_SummaryStatFile.WriteNumber(count);


		fclose(st);
	}
	

}

void ReadWeatherScenarioFile(string FileName, int scenario_no)
{


	FILE* st = NULL;
	fopen_s(&st, FileName.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;
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
				if (pLink->m_FromNodeNumber == 12730 && pLink->m_ToNodeNumber == 12742)
				{
					TRACE("");
				}

				int local_scenario_no = g_read_integer(st);

				CapacityReduction cs;
				cs.StartDayNo = g_read_integer(st) - 1; // start from zero
				cs.EndDayNo = g_read_integer(st) - 1;

				if (cs.StartDayNo < 0) cs.StartDayNo = 0;
				if (cs.EndDayNo < 0) cs.EndDayNo = 0;

				if (cs.StartDayNo < 0)
				{
					cout << "Start day = " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure Start Day >=0." << endl;
					g_ProgramStop();
				}

				if (cs.EndDayNo < 0)
				{
					cout << "End day = " << cs.EndDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Day >=1." << endl;
					g_ProgramStop();
				}

				if (cs.EndDayNo < cs.StartDayNo)
				{
					cout << "End day = " << cs.EndDayNo << " < Start day " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Day >= Start Day." << endl;
					g_ProgramStop();
				}


				cs.StartTime = g_read_integer(st);
				cs.EndTime = g_read_integer(st);

				if (cs.StartTime >= cs.EndTime)
				{
					cout << "End Time = " << cs.EndTime << " < Start Time " << cs.StartTime << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Time >= Start Time." << endl;
					g_ProgramStop();
				}

				float percentage = g_read_float(st);

				if (percentage < -0.1f)
				{
					cout << "Capacity Reduction Percentage = " << percentage << " % in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please check!" << endl;

					g_ProgramStop();

				}


				cs.LaneClosureRatio = percentage / 100.0f; // percentage -> to ratio
				if (cs.LaneClosureRatio > 1.0)
					cs.LaneClosureRatio = 1.0;
				if (cs.LaneClosureRatio < 0.0)
					cs.LaneClosureRatio = 0.0;
				cs.SpeedLimit = g_read_float(st);

				if (cs.SpeedLimit < -0.1f)
				{
					cout << "Speed Limit = " << cs.SpeedLimit << " % in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please check!" << endl;

					g_ProgramStop();

				}

					pLink->CapacityReductionVector.push_back(cs);
					count++;

			}
		}

		g_LogFile << "weather records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of weather records=");
		g_SummaryStatFile.WriteNumber(count);


		fclose(st);
	}
	else
	{

		cout << "Weather records = 0 " << endl;

		if (FileName.size() > 0 && FileName.compare("Scenario_Weather.csv") != 0)
		{
			cout << "File " << FileName << " cannot be opened. Please check!" << endl;
			g_ProgramStop();
		}
	}

}
void ReadEvacuationScenarioFile(string FileName, int scenario_no)
{
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->EvacuationImpactVector.clear(); // remove all previouly read records

	}

	FILE* st = NULL;
	fopen_s(&st, FileName.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;
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
				if (pLink->m_FromNodeNumber == 12730 && pLink->m_ToNodeNumber == 12742)
				{
					TRACE("");
				}

				int local_scenario_no = g_read_integer(st);

				CapacityReduction cs;
				cs.StartDayNo = g_read_integer(st) - 1; // start from zero
				cs.EndDayNo = g_read_integer(st) - 1;

				if (cs.StartDayNo < 0) cs.StartDayNo = 0;
				if (cs.EndDayNo < 0) cs.EndDayNo = 0;

				if (cs.StartDayNo < 0)
				{
					cout << "Start day = " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure Start Day >=0." << endl;
					g_ProgramStop();
				}

				if (cs.EndDayNo < 0)
				{
					cout << "End day = " << cs.EndDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Day >=1." << endl;
					g_ProgramStop();
				}

				if (cs.EndDayNo < cs.StartDayNo)
				{
					cout << "End day = " << cs.EndDayNo << " < Start day " << cs.StartDayNo << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Day >= Start Day." << endl;
					g_ProgramStop();
				}


				cs.StartTime = g_read_integer(st);
				cs.EndTime = g_read_integer(st);

				if (cs.StartTime >= cs.EndTime)
				{
					cout << "End Time = " << cs.EndTime << " < Start Time " << cs.StartTime << " in line " << count + 1 << "in file Scenario_Work_Zone.csv. Please ensure End Time >= Start Time." << endl;
					g_ProgramStop();
				}

				float percentage = g_read_float(st);

				cs.LaneClosureRatio = percentage / 100.0f; // percentage -> to ratio
				if (cs.LaneClosureRatio > 1.0)
					cs.LaneClosureRatio = 1.0;
				if (cs.LaneClosureRatio < 0.0)
					cs.LaneClosureRatio = 0.0;
				cs.SpeedLimit = g_read_float(st);

				if (cs.SpeedLimit < -0.1f)
				{
					cout << "Speed Limit = " << cs.SpeedLimit << " % in line " << count + 1 << " in file Scenario_Work_Zone.csv. Please check!" << endl;

					g_ProgramStop();

				}
					//evacuation data record goes to both capacity reduction and evacuation vectors. the latter is used to inform drivers to switch
					pLink->EvacuationImpactVector.push_back(cs);
					pLink->CapacityReductionVector.push_back(cs);
					count++;

			}
		}

		g_LogFile << "evacuation link records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of evacuation link records=");
		g_SummaryStatFile.WriteNumber(count);


		fclose(st);
	}
	else
	{

		cout << "evacuation linkrecords = 0 " << endl;

		if (FileName.size() > 0 && FileName.compare("Scenario_Evacuation_Zone.csv") != 0)
		{
			cout << "File " << FileName << " cannot be opened. Please check!" << endl;
			g_ProgramStop();
		}
	}

}

void ReadMovementScenarioFile(string FileName, int scenario_no)
{

	CCSVParser parser_movement;
	int count = 0;

	if (parser_movement.OpenCSVFile(FileName, false))
	{
		int i = 1;

		while (parser_movement.ReadRecord())
		{
			int record_scenario_no = 0;
			DTANodeMovement element;
			parser_movement.GetValueByFieldNameWithPrintOut("scenario_no", record_scenario_no);

			parser_movement.GetValueByFieldNameWithPrintOut("node_id", element.in_link_to_node_id);

			parser_movement.GetValueByFieldNameWithPrintOut("incoming_link_from_node_id", element.in_link_from_node_id);

			if (g_LinkMap.find(GetLinkStringID(element.in_link_from_node_id, element.in_link_to_node_id)) == g_LinkMap.end())
			{
				cout << "Link " << element.in_link_from_node_id << "-> " << element.in_link_to_node_id << " at line " << i + 1 << " of file input_movement.csv" << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}


			parser_movement.GetValueByFieldNameWithPrintOut("outgoing_link_to_node_id", element.out_link_to_node_id);
			if (g_LinkMap.find(GetLinkStringID(element.in_link_to_node_id, element.out_link_to_node_id)) == g_LinkMap.end())
			{
				cout << "Link " << element.in_link_to_node_id << "-> " << element.out_link_to_node_id << " at line " << i + 1 << " of file input_movement.csv" << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}

			parser_movement.GetValueByFieldNameWithPrintOut("effective_green_time_in_min", element.movement_effective_green_time_in_min);

			parser_movement.GetValueByFieldName("turning_direction", element.turning_direction);



			if (element.movement_hourly_capacity >= 0)
			{

				string movement_id = GetMovementStringID(element.in_link_from_node_id, element.in_link_to_node_id, element.out_link_to_node_id);
				int middle_node_id = g_NodeNametoIDMap[element.in_link_to_node_id];

					g_NodeVector[middle_node_id].m_MovementMap[movement_id] = element;  // assign movement to individual node
					count++;

			}// skip -1 as no data available

			i++;
		}
	}

	g_SummaryStatFile.WriteTextLabel("# of movement records=");
	g_SummaryStatFile.WriteNumber(count);
}
void ReadVMSScenarioFile(string FileName, int scenario_no)
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
				int local_scenario_no = g_read_integer(st);

				MessageSign is;

				is.Type = 1;
				is.StartDayNo = g_read_integer(st) - 1;  // start from day zero
				is.EndDayNo = g_read_integer(st) - 1;  // start from day zero

				if (is.StartDayNo < 0) is.StartDayNo = 0;
				if (is.EndDayNo < 0) is.EndDayNo = 0;

				is.StartTime = g_read_integer(st);
				is.EndTime = g_read_integer(st);
				is.NumberOfSubpaths = g_read_integer(st);

				if (is.NumberOfSubpaths == -1)
				{
					cout << "DMS has missing number of subpaths "<< endl;
					g_ProgramStop();

				}

				if (is.NumberOfSubpaths >=10)
				{
					cout << "DMS only support no more than 10 subpaths " << endl;
					g_ProgramStop();

				}

				for (int path = 0; path < min(10, is.NumberOfSubpaths); path++)  //at maximum sub paths
				{
					int number_of_nodes = g_read_integer(st);

					if (number_of_nodes == -1)
					{
						cout << "DMS has missing number of nodes " << endl;
						g_ProgramStop();

					}



					for (int i = 0; i < number_of_nodes; i++)
					{
						int node_number = g_read_integer(st);  // node number

						if (node_number == -1)
						{
							cout << "DMS has missing node_number " << endl;
							g_ProgramStop();

						}

						if (i==0 /*first node*/ && node_number != dsn)
						{
							cout << "DMS on link " << usn << "->" << dsn << " detour no. " << path +1 << " should start from node " << dsn << endl;
							g_ProgramStop();

						}
						is.detour_node_sequence[path].push_back(node_number);

					}

					is.DetourSwitchingRatio[path] = g_read_float(st);  // switching rate
					if (is.DetourSwitchingRatio[path] == -1)
					{
						cout << "DMS has mising switching rate " << endl;
						g_ProgramStop();

					}

				}


					g_bInformationUpdatingAndReroutingFlag = true;

					pLink->MessageSignVector.push_back(is);
					count++;
			}
		}

		g_LogFile << "VMS records = " << count << endl;
		cout << "VMS records = " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of VMS records=");
		g_SummaryStatFile.WriteNumber(count);


		fclose(st);
	}
	

}

void ReadLinkTollScenarioFile(string FileName, int scenario_no)
{

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->TollVector.clear(); // remove all previouly read records

	}


	// generate toll based on demand type code in input_link.csv file
	int demand_mode_type_count = 0;

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		if (g_LinkVector[li]->demand_type_code.size() >= 1)
		{  // with data string

			Toll tc;  // toll collection
			tc.StartDayNo = 0;
			tc.EndDayNo = 999;

			tc.StartTime = 0;
			tc.EndTime = 99999;

			std::string demand_type_code = g_LinkVector[li]->demand_type_code;


			for (int pt = 1; pt <= g_DemandTypeVector.size(); pt++)
			{
				CString number;
				number.Format("%d", pt);

				std::string str_number = CString2StdString(number);
				if (demand_type_code.find(str_number) == std::string::npos)   // do not find this number
				{
					tc.TollRate[pt] = 999;
					demand_mode_type_count++;
				}

			}  //end of pt
			g_LinkVector[li]->TollVector.push_back(tc);

		}

	}

	g_SummaryStatFile.WriteTextLabel("# of demand type code based toll records=");
	g_SummaryStatFile.WriteNumber(demand_mode_type_count);

	FILE* st = NULL;

	fopen_s(&st, FileName.c_str(), "r");
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;


		char  str_line[2000]; // input string
		int str_line_size = 1000;
		g_read_a_line(st, str_line, str_line_size); //  skip the first line  // with HOV 2 and 3

		float TotalTollValue[MAX_DEMAND_TYPE_SIZE] = { 0 };

		int count = 0;

		while (true)
		{
			int usn = g_read_integer(st, false);  // from node	

			if (usn <= 0)
				break;

			int dsn = g_read_integer(st, false);  // to node
			if (g_LinkMap.find(GetLinkStringID(usn, dsn)) == g_LinkMap.end())
			{
				cout << "Link " << usn << "-> " << dsn << " at line " << count + 1 << " of file" << FileName << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}

			DTALink* pLink = g_LinkMap[GetLinkStringID(usn, dsn)];

			if (pLink != NULL)
			{
				int local_scenario_no = g_read_integer(st);

				Toll tc;  // toll collection
				tc.StartDayNo = g_read_integer(st, false) - 1;  // start from zero
				tc.EndDayNo = g_read_integer(st, false) - 1;  // start from zero

				tc.StartTime = g_read_integer(st, false);
				tc.EndTime = g_read_integer(st, false);

				for (int pt = 1; pt <= g_DemandTypeVector.size(); pt++) // for each demand type
				{

					float toll_value = g_read_float(st);
					tc.TollRate[pt] = toll_value;

				}

					count++;
					pLink->TollVector.push_back(tc);

			}
		}

		cout << "Number of link-based toll records =  " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of link-based toll records=");
		g_SummaryStatFile.WriteNumber(count);

		//for (int pt = 1; pt< MAX_DEMAND_TYPE_SIZE; pt++)  // last one is transit fare
		//{
		//	CString label;
		//	label.Format("total toll value for pricing type %d= ", pt );
		//	g_SummaryStatFile.WriteTextLabel(label);
		//	g_SummaryStatFile.WriteNumber(TotalTollValue[pt]);

		//}


		fclose(st);

	}
	

	g_CreateLinkTollVector();
}

void ReadRadioMessageScenarioFile(string FileName, int scenario_no)
{

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{

		g_LinkVector[li]->m_RadioMessageVector.clear(); // remove all previouly read records

	}

	FILE* st = NULL;

	fopen_s(&st, FileName.c_str(), "r");
	if (st != NULL)
	{
		cout << "Reading file " << FileName << endl;
		g_LogFile << "Reading file " << FileName << endl;


		int count = 0;

		while (true)
		{
			int usn = g_read_integer(st, false);

			if (usn <= 0)
				break;

			int dsn = g_read_integer(st, false);
			if (g_LinkMap.find(GetLinkStringID(usn, dsn)) == g_LinkMap.end())
			{
				cout << "Link " << usn << "-> " << dsn << " at line " << count + 1 << " of file" << FileName << " has not been defined in input_link.csv. Please check.";
				g_ProgramStop();
			}

			DTALink* pLink = g_LinkMap[GetLinkStringID(usn, dsn)];

			if (pLink != NULL)
			{
				int local_scenario_no = g_read_integer(st);

				RadioMessage rm;
				rm.StartDayNo = g_read_integer(st, false) - 1;  // start from zero
				rm.EndDayNo = g_read_integer(st, false) - 1;  // start from zero

				rm.StartTime = g_read_integer(st, false);
				rm.EndTime = g_read_integer(st, false);

				rm.ResponsePercentage = g_read_float(st);
				rm.DelayPenaltyInMin = g_read_float(st);

				pLink->m_RadioMessageVector.push_back(rm);


			}
		}

		cout << "Number of link-based toll records =  " << count << endl;
		g_SummaryStatFile.WriteTextLabel("# of link-based toll records=");
		g_SummaryStatFile.WriteNumber(count);

		fclose(st);

	}
	


	g_CreateLinkTollVector();
}
void g_ReadScenarioInputFiles(int scenario_no)
{
	ReadLinkTollScenarioFile("Scenario_Link_Based_Toll.csv", scenario_no);
	ReadWorkZoneScenarioFile("Scenario_Work_Zone.csv", scenario_no);

	if (g_ODEstimationFlag == false)
	{
		ReadVMSScenarioFile("Scenario_Dynamic_Message_Sign.csv", scenario_no);
	}

}
void FreeMemory()
{

}



void OutputLinkMOEData(std::string fname, int Iteration, bool bStartWithEmpty)
{
	FILE* st = NULL;
	FILE* st_struct = NULL;

	int TDLinkMOE_aggregation_time_interval = g_GetPrivateProfileInt("output", "TDLinkMOE_aggregation_time_interval_in_min", 1, g_DTASettingFileName);

	if (TDLinkMOE_aggregation_time_interval < 1)
		TDLinkMOE_aggregation_time_interval = 1;

	if (bStartWithEmpty)
		fopen_s(&st, fname.c_str(), "w");
	else
		fopen_s(&st, fname.c_str(), "a");

	fopen_s(&st_struct, "output_LinkTDMOE.bin", "wb");

	std::set<DTALink*>::iterator iterLink;

	typedef struct
	{
		int from_node_id;
		int to_node_id;
		int timestamp_in_min;
		float travel_time_in_min;
		float delay_in_min;
		float link_volume_in_veh_per_hour_per_lane;
		float link_volume_in_veh_per_hour_for_all_lanes;
		float density_in_veh_per_mile_per_lane;
		float speed_in_mph;
		float exit_queue_length;  // in ratio
		int cumulative_arrival_count;
		int cumulative_departure_count;

		int time_dependent_left_arrival_count;
		int time_dependent_left_departure_count;
		int number_of_queued_vehicles;
		int number_of_left_queued_vehicles;

		int cumulative_SOV_revenue;
		int cumulative_HOV_revenue;
		int cumulative_truck_revenue;
		int day_no;
		float Energy;
		float CO2;
		float NOX;
		float CO;
		float HC;


	} struct_TDMOE;

	if (st != NULL)
	{
		if (bStartWithEmpty)
		{
			fprintf(st, "from_node_id,to_node_id,link_id_from_to,day_no,timestamp_in_min,travel_time_in_min,delay_in_min,link_in_volume_number_of_veh,link_out_volume_number_of_veh,link_volume_in_veh_per_hour_per_lane,link_volume_in_veh_per_hour_for_all_lanes,density_in_veh_per_distance_per_lane,speed,queue_length_percentage,number_of_queued_vehicles,cumulative_arrival_count,cumulative_departure_count,total_energy,total_CO2,total_NOX,total_CO,total_HC,total_PM,total_PM2.5\n");
		}

		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{

			DTALink* pLink = g_LinkVector[li];
			for (int time = g_DemandLoadingStartTimeInMin; time <= g_SimululationReadyToEnd; time += TDLinkMOE_aggregation_time_interval)
			{

				//				if((pLink->m_LinkMOEAry[time].CumulativeArrivalCount - pLink->m_LinkMOEAry[time].CumulativeDepartureCount) > 0) // there are vehicles on the link
				if (time < pLink->m_LinkMOEAry.size())
				{
					float LinkInFlow = float(pLink->GetArrivalFlow(time));
					float LinkOutFlow = float(pLink->GetDepartureFlow(time));
					float travel_time = pLink->GetTravelTimeByMin(Iteration, time, TDLinkMOE_aggregation_time_interval, g_TrafficFlowModelFlag);

					struct_TDMOE tdmoe_element;

					tdmoe_element.Energy = 0.0;
					tdmoe_element.CO2 = 0.0;
					tdmoe_element.NOX = 0.0;
					tdmoe_element.HC = 0.0;
					tdmoe_element.CO = 0.0;


					float queue_length_ratio = 0;


					queue_length_ratio = pLink->m_LinkMOEAry[time].ExitQueueLength / (pLink->m_KJam * pLink->m_Length * pLink->m_InflowNumLanes); /* in ratio*/

					int day_no = Iteration;

					fprintf(st, "%d,%d,%d->%d,%d,%d,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%3.2f,%3.2f,%d,%d,%d",
						g_NodeVector[pLink->m_FromNodeID].m_NodeNumber,
						g_NodeVector[pLink->m_ToNodeID].m_NodeNumber,
						g_NodeVector[pLink->m_FromNodeID].m_NodeNumber,
						g_NodeVector[pLink->m_ToNodeID].m_NodeNumber,
						day_no, time,
						travel_time, travel_time - pLink->m_FreeFlowTravelTime,
						LinkInFlow, LinkOutFlow, LinkInFlow*60.0 / max(1, pLink->m_InflowNumLanes), LinkInFlow*60.0,
						(pLink->m_LinkMOEAry[time].CumulativeArrivalCount - pLink->m_LinkMOEAry[time].CumulativeDepartureCount)*1.0 / pLink->m_Length / max(1, pLink->m_InflowNumLanes),
						pLink->GetSpeed(time),
						queue_length_ratio * 100,
						pLink->m_LinkMOEAry[time].ExitQueueLength,
						pLink->m_LinkMOEAry[time].CumulativeArrivalCount,
						pLink->m_LinkMOEAry[time].CumulativeDepartureCount);

					tdmoe_element.day_no = day_no;
					tdmoe_element.from_node_id = g_NodeVector[pLink->m_FromNodeID].m_NodeNumber;
					tdmoe_element.to_node_id = g_NodeVector[pLink->m_ToNodeID].m_NodeNumber;
					tdmoe_element.timestamp_in_min = time;
					tdmoe_element.travel_time_in_min = travel_time;
					tdmoe_element.delay_in_min = travel_time - pLink->m_FreeFlowTravelTime;

					//	count total node delay for each inbound link
					g_NodeVector[pLink->m_ToNodeID].m_TotalDelay += tdmoe_element.delay_in_min;


					g_NodeVector[pLink->m_ToNodeID].AddIncomingLinkDelay(li, tdmoe_element.delay_in_min);



					tdmoe_element.link_volume_in_veh_per_hour_per_lane = LinkInFlow*60.0 / pLink->m_InflowNumLanes;
					tdmoe_element.link_volume_in_veh_per_hour_for_all_lanes = LinkInFlow*60.0;
					tdmoe_element.density_in_veh_per_mile_per_lane = (pLink->m_LinkMOEAry[time].CumulativeArrivalCount - pLink->m_LinkMOEAry[time].CumulativeDepartureCount) / pLink->m_Length / max(1, pLink->m_InflowNumLanes);
					tdmoe_element.speed_in_mph = pLink->GetSpeed(time);
					tdmoe_element.exit_queue_length = queue_length_ratio; /* in ratio*/;
					tdmoe_element.cumulative_arrival_count = pLink->m_LinkMOEAry[time].CumulativeArrivalCount;
					tdmoe_element.cumulative_departure_count = pLink->m_LinkMOEAry[time].CumulativeDepartureCount;

					tdmoe_element.time_dependent_left_arrival_count = pLink->m_LinkMOEAry[time].IntervalLeftArrivalCount;
					tdmoe_element.time_dependent_left_departure_count = pLink->m_LinkMOEAry[time].IntervalLeftDepartureCount;
					tdmoe_element.number_of_queued_vehicles = pLink->m_LinkMOEAry[time].ExitQueueLength;
					tdmoe_element.number_of_left_queued_vehicles = pLink->m_LinkMOEAry[time].LeftExit_QueueLength;


					//tdmoe_element.cumulative_SOV_revenue = pLink->m_LinkMOEAry[time].CumulativeRevenue_DemandType[1];
					//tdmoe_element.cumulative_HOV_revenue = pLink->m_LinkMOEAry[time].CumulativeRevenue_DemandType[2];
					//tdmoe_element.cumulative_truck_revenue = pLink->m_LinkMOEAry[time].CumulativeRevenue_DemandType[3];


					if (LinkInFlow >= 1)
					{
#ifdef _high_level_memory_usage
						float Energy = pLink->m_LinkMOEAry[time].Energy;

						if (Energy >= 1)
						{
							tdmoe_element.Energy = pLink->m_Length / max(0.00000001, Energy / 1000 / 121.7 / LinkInFlow);
							tdmoe_element.CO2 = pLink->m_LinkMOEAry[time].CO2 / 1000 / LinkInFlow / pLink->m_Length;
							tdmoe_element.NOX = pLink->m_LinkMOEAry[time].NOX / LinkInFlow / pLink->m_Length;
							tdmoe_element.CO = pLink->m_LinkMOEAry[time].CO / LinkInFlow / pLink->m_Length;
							tdmoe_element.HC = pLink->m_LinkMOEAry[time].HC / LinkInFlow / pLink->m_Length;
						}
#endif
					}

					
					int pt;



#ifdef _high_level_memory_usage
					fprintf(st, ",%f,", pLink->m_LinkMOEAry[time].Energy); // total
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].CO2 / 1000);
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].NOX);
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].CO);
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].HC);
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].PM);
					fprintf(st, "%f,", pLink->m_LinkMOEAry[time].PM2_5);

#endif
			
					fprintf(st, "\n");
					fwrite(&tdmoe_element, sizeof(tdmoe_element), 1, st_struct);

				}

			}

		}
		fclose(st);
		fclose(st_struct);
	}
	else
	{
		fprintf(g_DebugLogFile, "File output_LinkTDMOE.csv cannot be opened. It might be currently used and locked by EXCEL.");
		cout << "Error: File output_LinkTDMOE.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		cin.get();  // pause
	}



}

void OutputRealTimeLinkMOEData(std::string fname, int current_time_in_min, int ouput_MOE_aggregation_time_interval_in_min, bool bTravelTimeOnly)
{

	FILE* st = NULL;

	fopen_s(&st, fname.c_str(), "w");

	_proxy_ABM_log(0,  "writing link MOE file %s...\n", fname.c_str());

	std::set<DTALink*>::iterator iterLink;

	if (st != NULL)
	{

		g_LogFile << " current time: " << current_time_in_min << "min, output link moe data to file " << fname << endl;

		if (bTravelTimeOnly)
		{
			fprintf(st, "from_node_id,to_node_id,timestamp_in_min,travel_time_in_min\n");
		}
		else
		{
			fprintf(st, "from_node_id,to_node_id,timestamp_in_min,travel_time_in_min,delay_in_min,link_volume_in_veh_per_hour_per_lane,link_volume_in_veh_per_hour_for_all_lanes,density_in_veh_per_distance_per_lane,speed,exit_queue_length,cumulative_arrive_count,cumulative_departure_count\n");
		}

		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{

			DTALink* pLink = g_LinkVector[li];
			for (int time = max(0, current_time_in_min - ouput_MOE_aggregation_time_interval_in_min); time < current_time_in_min; time++)
			{

				float LinkInFlow = float(pLink->GetArrivalFlow(time, ouput_MOE_aggregation_time_interval_in_min));
				float travel_time = max(pLink->GetTravelTimeByMin(0, time, ouput_MOE_aggregation_time_interval_in_min, g_TrafficFlowModelFlag), pLink->m_prevailing_travel_time);
				
				float queue_length_ratio = 0;

				queue_length_ratio = pLink->m_LinkMOEAry[time].ExitQueueLength;

				if (bTravelTimeOnly)
				{
					fprintf(st, "%d,%d,%d,%6.2f",
						g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber, time,
						travel_time);

					fprintf(st, "\n");


				}
				else
				{

					fprintf(st, "%d,%d,%d,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%3.2f,%d,%d",
						g_NodeVector[pLink->m_FromNodeID].m_NodeNumber,
						g_NodeVector[pLink->m_ToNodeID].m_NodeNumber,
						time,
						travel_time,
						travel_time - pLink->m_FreeFlowTravelTime,
						LinkInFlow*60.0 / max(1,pLink->m_OutflowNumLanes),  /*per lane*/
						LinkInFlow*60.0,  /*per link hourly */
						(pLink->m_LinkMOEAry[time].CumulativeArrivalCount - pLink->m_LinkMOEAry[time].CumulativeDepartureCount) / pLink->m_Length / max(1, pLink->m_OutflowNumLanes),  /*density*/
						pLink->m_Length / max(0.1, travel_time)*60.0,
						queue_length_ratio,
						pLink->m_LinkMOEAry[time].CumulativeArrivalCount,
						pLink->m_LinkMOEAry[time].CumulativeDepartureCount);

					fprintf(st, "\n");
				}

			}



		}  //all links

		fclose(st);
	}
	else
	{
		cout << "Error: File " << fname << " cannot be opened.\n It might be currently used and locked. Please any key to continue." << endl;
		cin.get();  // pause
	}

}


void OutputNetworkMOEData(ofstream &output_NetworkTDMOE_file)
{
	output_NetworkTDMOE_file << "time_stamp_in_min,cumulative_in_flow_count,cumulative_out_flow_count,number_of_vehicles_in_network,flow_in_a_min,avg_trip_time_in_min" << endl;

	for (int time = g_DemandLoadingStartTimeInMin; time< min(g_NetworkMOEAry.size(), g_PlanningHorizon); time++)
	{
		if (g_NetworkMOEAry[time].Flow_in_a_min > 0)
			g_NetworkMOEAry[time].AvgTripTime = g_NetworkMOEAry[time].AbsArrivalTimeOnDSN_in_a_min / g_NetworkMOEAry[time].Flow_in_a_min;

		if (time >= 1)
		{
			if (g_NetworkMOEAry[time].CumulativeInFlow < g_NetworkMOEAry[time - 1].CumulativeInFlow)
				g_NetworkMOEAry[time].CumulativeInFlow = g_NetworkMOEAry[time - 1].CumulativeInFlow;

			if (g_NetworkMOEAry[time].CumulativeOutFlow < g_NetworkMOEAry[time - 1].CumulativeOutFlow)
				g_NetworkMOEAry[time].CumulativeOutFlow = g_NetworkMOEAry[time - 1].CumulativeOutFlow;
		}

		output_NetworkTDMOE_file << time << "," <<
			g_NetworkMOEAry[time].CumulativeInFlow << "," <<
			g_NetworkMOEAry[time].CumulativeOutFlow << "," <<
			g_NetworkMOEAry[time].CumulativeInFlow - g_NetworkMOEAry[time].CumulativeOutFlow << "," <<
			g_NetworkMOEAry[time].Flow_in_a_min << "," <<
			g_NetworkMOEAry[time].AvgTripTime << "," << endl;

		if (time > g_DemandLoadingStartTimeInMin + 10 && ((g_NetworkMOEAry[time].CumulativeInFlow - g_NetworkMOEAry[time].CumulativeOutFlow) == 0))
			break; // stop, as all vehicles leave. 
	}

}






void 	RT_ShortestPath_Thread(int id, int nthreads, int node_size, int link_size, int departure_time)
{

}

int g_InitializeLogFiles()
{
	g_AppStartTime = CTime::GetCurrentTime();
	g_AppLastIterationStartTime = g_AppStartTime;

	//g_simulation_log_file = fopen("debug_simulation.log","w");
	//g_simulation_log_level = g_GetPrivateProfileInt("debug", "simulation_log_level", 0, g_DTASettingFileName);

	//g_emission_log_file = fopen("debug_emission.log", "w");
	//g_emission_log_level = g_GetPrivateProfileInt("debug", "emission_log_level", 0, g_DTASettingFileName);

	//g_ODME_log_file = fopen("debug_ODME.log", "w");
	//g_ODME_log_level = g_GetPrivateProfileInt("debug", "ODME_log_level", 0, g_DTASettingFileName);


	g_ODME_result_file = fopen("ODME_zone_based_log.csv", "w");
	//g_ABM_log_file = fopen("debug_ABM.log", "w");
	//g_ABM_log_level = g_GetPrivateProfileInt("debug", "ABM_log_level", 0, g_DTASettingFileName);

	

	//g_LogFile.open("debug_simulation.log", ios::out);
	//if (g_LogFile.is_open())
	//{
	//	g_LogFile.width(12);
	//	g_LogFile.precision(3);
	//	g_LogFile.setf(ios::fixed);
	//}
	//else
	//{
	//	cout << "File output_simulation.log cannot be opened, and it might be locked by another program or the target data folder is read-only." << endl;
	//	cin.get();  // pause
	//	return 0;
	//}


	//g_NetworkDesignLogFile.open("debug_network_design.csv", ios::out);
	//if (g_NetworkDesignLogFile.is_open())
	//{
	//	g_NetworkDesignLogFile.width(12);
	//	g_NetworkDesignLogFile.precision(3);
	//	g_NetworkDesignLogFile.setf(ios::fixed);

	//	g_NetworkDesignLogFile << endl;

	//}
	//else
	//{
	//	cout << "File output_network_design_log.csv cannot be opened, and it might be locked by another program!" << endl;
	//	cin.get();  // pause
	//	return 0;
	//}



	g_EstimationLogFile.open("ODME_link_based_log.csv", ios::out);
	if (g_EstimationLogFile.is_open())
	{
		g_EstimationLogFile.width(12);
		g_EstimationLogFile.precision(3);
		g_EstimationLogFile.setf(ios::fixed);
	}
	else
	{
		cout << "File ODME_link_based_log.csv cannot be opened, and it might be locked by another program!" << endl;
		cin.get();  // pause
		return 0;
	}


	g_WarningFile.open("warning.log", ios::out);
	if (g_WarningFile.is_open())
	{
		g_WarningFile.width(12);
		g_WarningFile.precision(3);
		g_WarningFile.setf(ios::fixed);
	}
	else
	{
		cout << "File warning.log cannot be opened, and it might be locked by another program!" << endl;
		cin.get();  // pause
		return 0;
	}

	DWORD dwReturn;                                    //Declare needed variables
	char szBuffer[MAX_PATH];                           //Allocate (easily) a buffer
	//using MAX_PATH equate
	dwReturn = GetCurrentDirectory(MAX_PATH, szBuffer); //Call Api Function

	cout << "--- Current Directory: " << szBuffer << " ---" << endl << endl;
	g_LogFile << "--- Current Directory: " << szBuffer << " ---" << endl << endl;


	CCSVParser parser_MOE_settings;
	if (!parser_MOE_settings.OpenCSVFile("optional_MOE_settings.csv", false))
	{
		// use default files
		CCSVWriter csv_output("optional_MOE_settings.csv");
		csv_output.WriteTextString("moe_type,moe_group,notes,moe_category_label,cut_off_volume,demand_type,vehicle_type,information_type,from_node_id,mid_node_id,to_node_id,origin_zone_id,destination_zone_id,exclude_link_no,link_list_string,departure_starting_time,departure_ending_time,entrance_starting_time,entrance_ending_time,impact_type");
		csv_output.WriteTextString("network,1,,network");
		csv_output.WriteTextString("network,2,,sov,,1,");
		csv_output.WriteTextString("network,2,,hov,,2,");
		csv_output.WriteTextString("network,2,,truck,,3,");
		csv_output.WriteTextString("network,2,,intermodal,,4,");

		csv_output.WriteTextString("network,3,,passenger_car,,,1");
		csv_output.WriteTextString("network,3,,passenger_truck,,,2");
		csv_output.WriteTextString("network,3,,light_commercial,,,3");
		csv_output.WriteTextString("network,3,,single_unit_long_truck,,,4");
		csv_output.WriteTextString("network,3,,combination_long_truck,,,5");

		csv_output.WriteTextString("od,4,,OD1,,,,,,,,1,2");
		csv_output.WriteTextString("od,4,,OD2,,,,,,,,2,1");

		csv_output.WriteTextString("link,5,,link_48_41_ NB freeway,,,,,48,,41,,,,,,,,,");
		csv_output.WriteTextString("link,5,,link_39_44_NB freeway,,,,,30,,25,,,,,,,,,");

		csv_output.WriteTextString("path_3point,6,,'Path1_Major freeway route, which carries a lot of the damand',,,,,200,41,198,,,,,,,,,");

		csv_output.WriteTextString("network_time_dependent,7,,for all demand types,,,,,120,91,162");

		csv_output.WriteTextString("link_critical,8,,critical link list,1000");
		csv_output.WriteTextString("od_critical,9,,critical OD list,50");

	}

	

	fopen_s(&g_DebugLogFile, "debug.log", "w");
	if (g_DebugLogFile == NULL)
	{
		cout << "Cannot open file debug.log! Another instance of DTALite might be running on the same folder." << endl;
		cin.get();  // pause
		return 0;
	}

	//		g_RunStaticExcel();
	return 1;
}

void g_ReadDTALiteSettings()
{

	// if  ./DTASettings.ini does not exist, then we should print out all the default settings for user to change
	//
	g_AdjLinkSize = g_GetPrivateProfileInt("shortest_path", "max_size_of_adjacent_links", 30, g_DTASettingFileName);

	if (g_AdjLinkSize < 30)
		g_AdjLinkSize = 30;

	g_AggregationTimetInterval = 15; // g_GetPrivateProfileInt("shortest_path", "travel_time_aggregation_interval_in_min", 15, g_DTASettingFileName);

	if (g_AggregationTimetInterval < 1)
		g_AggregationTimetInterval = 1;

	g_settings.AdditionalYellowTimeForSignals = 4; // g_GetPrivateProfileInt("simulation", "additional_amber_time_per_link_per_cycle", 4, g_DTASettingFileName);
	//	g_settings.IteraitonNoStartSignalOptimization = g_GetPrivateProfileInt("signal_optimization", "starting_iteration_no", 1, g_DTASettingFileName);
	//	g_settings.IteraitonStepSizeSignalOptimization = g_GetPrivateProfileInt("signal_optimization", "number_of_iterations_per_optimization", 5, g_DTASettingFileName);
	//	g_settings.DefaultCycleTimeSignalOptimization = g_GetPrivateProfileInt("signal_optimization", "default_cycle_time_in_sec", 60, g_DTASettingFileName);

	g_UseDefaultLaneCapacityFlag = 0; // g_GetPrivateProfileInt("simulation", "use_default_lane_capacity", 0, g_DTASettingFileName);
	//	g_UseFreevalRampMergeModelFlag = g_GetPrivateProfileInt("simulation", "use_freeval_merge_model", 0, g_DTASettingFileName);	
	g_OutputLinkCapacityFlag = 0;  //g_GetPrivateProfileInt("simulation", "output_link_capacity_file", 0, g_DTASettingFileName);
	g_OutputLinkCapacityStarting_Time = 0; // g_GetPrivateProfileInt("simulation", "output_link_capacity_start_time_in_min", 0, g_DTASettingFileName);
	g_OutputLinkCapacityEnding_Time = 0; // g_GetPrivateProfileInt("simulation", "output_link_capacity_end_time_in_min", 300, g_DTASettingFileName);
	g_settings.use_point_queue_model_for_on_ramps = g_GetPrivateProfileInt("simulation", "use_point_queue_for_on_ramps", 1, g_DTASettingFileName);
	g_settings.use_point_queue_model_for_off_ramps = g_GetPrivateProfileInt("simulation", "use_point_queue_for_off_ramps", 1, g_DTASettingFileName);

	g_settings.use_mile_or_km_as_length_unit = 1; // g_GetPrivateProfileInt("simulation", "use_mile_or_km_as_length_unit", 1, g_DTASettingFileName);

	g_EmissionDataOutputFlag = g_GetPrivateProfileInt("emission", "output_emission_data", 1, g_DTASettingFileName);

	g_OutputEmissionOperatingModeData = 0; // g_GetPrivateProfileInt("emission", "output_opreating_mode_data", 0, g_DTASettingFileName);
	g_use_routing_policy_from_external_input = 0; // g_GetPrivateProfileInt("assignment", "use_routing_policy_from_external_input", 0, g_DTASettingFileName);

	if (g_use_routing_policy_from_external_input == 1)
		g_output_routing_policy_file = 1;

	if (!g_OutputEmissionOperatingModeData)
		g_OutputSecondBySecondEmissionData = 0;
	else
	{
		g_OutputSecondBySecondEmissionData = g_GetPrivateProfileInt("emission", "output_second_by_second_emission_data", 0, g_DTASettingFileName);
		g_OutputSecondBySecondEmissionDataPercentage = g_GetPrivateProfileFloat("emission", "sampling_percentange_for_outputting_second_by_second_emission_data", 1, g_DTASettingFileName);
		g_EmissionSmoothVehicleTrajectory = g_GetPrivateProfileFloat("emission", "smooth_vehicle_trajectory", 1, g_DTASettingFileName);
		g_start_departure_time_for_output_second_by_second_emission_data = g_GetPrivateProfileInt("emission", "start_departure_time_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);
		g_end_departure_time_for_output_second_by_second_emission_data = g_GetPrivateProfileInt("emission", "end_departure_time_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);
	}

	g_TargetVehicleID_OutputSecondBySecondEmissionData = g_GetPrivateProfileInt("emission", "target_vehicle_id_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);

	//		g_TollingMethodFlag = g_GetPrivateProfileInt("tolling", "method_flag", 0, g_DTASettingFileName);	
	//		g_VMTTollingRate = g_GetPrivateProfileFloat("tolling", "VMTRate", 0, g_DTASettingFileName);
	g_VehicleLoadingMode = demand_matrix_file_mode;
	//	g_ParallelComputingMode = g_GetPrivateProfileInt("assignment", "parallel_computing", 1, g_DTASettingFileName);
	g_ParallelComputingMode = 1;


	g_RandomizedCapacityMode = 0; // g_GetPrivateProfileInt("simulation", "ramdomized_capacity", 0, g_DTASettingFileName);
	g_DemandCapacityScalingFactor = g_GetPrivateProfileFloat("simulation", "demand_capcaity_scaling_factor", 1.0, g_DTASettingFileName);

	g_StochasticCapacityMode = 0; // g_GetPrivateProfileInt("simulation", "stochatic_capacity_mode", 1, g_DTASettingFileName);

	g_MergeNodeModelFlag = 0;
	//	g_MergeNodeModelFlag = g_GetPrivateProfileInt("simulation", "merge_node_model", 0, g_DTASettingFileName);	
	g_FIFOConditionAcrossDifferentMovementFlag = 0; // g_GetPrivateProfileInt("simulation", "first_in_first_out_condition_across_different_movements", 0, g_DTASettingFileName);
	g_MinimumInFlowRatio = 0; // g_GetPrivateProfileFloat("simulation", "minimum_link_in_flow_ratio", 0.00f, g_DTASettingFileName);
	g_RelaxInFlowConstraintAfterDemandLoadingTime = 60; // g_GetPrivateProfileFloat("simulation", "use_point_queue_model_x_min_after_demand_loading_period", 60.0f, g_DTASettingFileName);
	g_MaxDensityRatioForVehicleLoading = 0.8; // g_GetPrivateProfileFloat("simulation", "max_density_ratio_for_loading_vehicles", 0.8f, g_DTASettingFileName);
	g_DefaultSaturationFlowRate_in_vehphpl = g_GetPrivateProfileFloat("simulation", "default_saturation_flow_rate_in_vehphpl", 1800, g_DTASettingFileName);

	g_AgentBasedAssignmentFlag =  g_GetPrivateProfileInt("assignment", "agent_based_assignment", 0, g_DTASettingFileName);
	g_AggregationTimetInterval = g_GetPrivateProfileInt("assignment", "aggregation_time_interval_in_min", 15, g_DTASettingFileName);

	g_ConvergencyRelativeGapThreshold_in_perc = g_GetPrivateProfileFloat("assignment", "convergency_relative_gap_threshold_percentage", 5, g_DTASettingFileName);
	g_UpdatedDemandPrintOutThreshold = 5; // g_GetPrivateProfileFloat("estimation", "updated_demand_print_out_threshold", 5, g_DTASettingFileName);

	//g_StartIterationsForOutputPath = g_GetPrivateProfileInt("output", "start_iteration_output_path", g_NumberOfIterations, g_DTASettingFileName);	
	//g_EndIterationsForOutputPath = g_GetPrivateProfileInt("output", "end_iteration_output_path", g_NumberOfIterations, g_DTASettingFileName);	

	//if( g_StartIterationsForOutputPath  < 0) 
	//	g_StartIterationsForOutputPath = 0;

	//if(g_EndIterationsForOutputPath > g_NumberOfIterations)
	//	g_EndIterationsForOutputPath = g_NumberOfIterations;

	g_StartIterationsForOutputPath = g_EndIterationsForOutputPath = g_NumberOfIterations - 1;

	g_DepartureTimeChoiceEarlyDelayPenalty = 0; // g_GetPrivateProfileFloat("assignment", "departure_time_choice_early_delay_penalty", 0.969387755f, g_DTASettingFileName); // default is non learning
	g_DepartureTimeChoiceLateDelayPenalty = 0; // g_GetPrivateProfileFloat("assignment", "departure_time_choice_late_delay_penalty", 1.306122449f, g_DTASettingFileName); // default is non learning

	g_TravelTimeDifferenceForSwitching = g_GetPrivateProfileFloat("assignment", "travel_time_difference_for_switching_in_min", 1, g_DTASettingFileName);
	g_RelativeTravelTimePercentageDifferenceForSwitching = g_GetPrivateProfileFloat("assignment",
		"relative_travel_time_difference_in_percentage_for_switching_in_percentage", 15, g_DTASettingFileName);

	if (g_DemandLoadingEndTimeInMin >= 2000)
	{
		cout << "Error: g_DemandLoadingEndTimeInMin >=2000";
		g_ProgramStop();
	}

	g_DetermineDemandLoadingPeriod();

	g_start_iteration_for_MOEoutput = -1; // g_GetPrivateProfileInt("output", "start_iteration_for_MOE", -1, g_DTASettingFileName);




	g_SystemOptimalStartingTimeinMin = 0; // g_GetPrivateProfileInt("system_optimal_assignment", "re_routing_start_time_in_min", 0, g_DTASettingFileName);

	g_VMSPerceptionErrorRatio = 0.05;// g_GetPrivateProfileFloat("traveler_information", "coefficient_of_variation_of_VMS_perception_error", 0.05f, g_DTASettingFileName);
	g_information_updating_interval_in_min = g_GetPrivateProfileInt("traveler_information", "information_updating_interval_in_min", 1, g_DTASettingFileName);

	g_output_OD_path_MOE_file = 0; // = g_GetPrivateProfileInt("output", "OD_path_MOE_file", 0, g_DTASettingFileName);
	g_output_OD_TD_path_MOE_file = 0;  // g_GetPrivateProfileInt("output", "OD_path_MOE_file", 0, g_DTASettingFileName);

	g_output_OD_path_MOE_cutoff_volume = 1;// g_GetPrivateProfileInt("output", "OD_path_MOE_cutoff_volume", 1, g_DTASettingFileName);

	g_NetworkDesignOptimalLinkSize = 0;// g_GetPrivateProfileInt("network_design", "number_of_links_to_be_built", 1, g_DTASettingFileName);
	g_NetworkDesignTravelTimeBudget = 0; // g_GetPrivateProfileInt("network_design", "travel_time_budget_in_min", 12, g_DTASettingFileName);

	if (g_UEAssignmentMethod == analysis_LR_agent_based_system_optimization)  // 12
	{
		g_AssignmentLogFile << "[network_design]," << "number_of_links_to_be_built=," << g_NetworkDesignOptimalLinkSize << endl;
		g_AssignmentLogFile << "[network_design]," << "target_travel_time_in_min=," << g_NetworkDesignTravelTimeBudget << endl;
	}

	if (g_UEAssignmentMethod == analysis_accessibility_distance || g_UEAssignmentMethod == analysis_accessibility_travel_time)
		g_PlanningHorizon = 1;

	srand(g_RandomSeed);
	g_LogFile << "Simulation Horizon (min) = " << g_PlanningHorizon << endl;
	g_LogFile << "Departure Time Interval (min) = " << g_AggregationTimetInterval << endl;
	g_LogFile << "Number of Iterations = " << g_NumberOfIterations << endl;


	if (g_VehicleLoadingMode == vehicle_binary_file_mode)
	{
		g_LogFile << "Load vehicles from the trajectory file agent.bin " << endl;
	}



	if (g_start_iteration_for_MOEoutput == -1)  // no value specified
	{

		g_start_iteration_for_MOEoutput = g_NumberOfIterations;
	}


}


void g_FreeMemory(bool exit_flag = true)
{
	cout << "Free memory... " << endl;
	return;
	// Free pointers
	// ask the operating system to free the memory  after the program complete
	exit(0);

	int LinkSizeForLinkCostArray = g_LinkVector.size() + g_NodeVector.size(); // double the size to account for artificial connectors

	if (g_LinkTDCostAry != NULL)
		DeallocateDynamicArray<DTALinkToll>(g_LinkTDCostAry, LinkSizeForLinkCostArray, g_NumberOfSPCalculationPeriods);

	if (g_GridMatrix)
		DeallocateDynamicArray<GridNodeSet>(g_GridMatrix, _MAX_TRANSIT_GRID_SIZE, _MAX_TRANSIT_GRID_SIZE);


	try
	{
		cout << "Free link set... " << endl;
		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];

			if (pLink != NULL && pLink->m_OutflowNumLanes >= 1)
			{
				delete pLink;
			}

		}

	}
	catch (const std::exception& e)
	{
		std::cout << "Got exception: " << e.what() << std::endl;

	}


	cout << "Free node set... " << endl;
	g_NodeVector.clear();

	g_LinkVector.clear();


	g_VehicleTypeVector.clear();
	g_NodeNametoIDMap.clear();
	g_DemandTypeVector.clear();
	g_TimeDependentDemandProfileVector.clear();
	g_FreeMemoryForVehicleVector();
	g_NetworkMOEAry.clear();

	g_AssignmentMOEVector.clear();
	g_ODTKPathVector.clear();
	g_simple_vector_vehicles.clear();

	if (g_TDOVehicleArray != NULL && g_ODZoneNumberSize > 0);
	{
		DeallocateDynamicArray<VehicleArrayForOriginDepartrureTimeInterval>(g_TDOVehicleArray, g_ZoneMap.size(), g_NumberOfSPCalculationPeriods);  // +1 is because the zone numbers start from 1 not from 0
		g_TDOVehicleArray = NULL;
		g_NumberOfSPCalculationPeriods = -1;
	}

	//free zone map after g_TDOVehicleArray as g_ZoneMap is still used in DeallocateDynamicArray for g_TDOVehicleArray
	g_ZoneMap.clear();

	int number_of_threads = omp_get_max_threads();

	if (g_ODEstimationFlag == 0)
	{

		if ((g_use_routing_policy_from_external_input == 1) && g_ODPathSetVector != NULL)
			Deallocate3DDynamicArray<ODPathSet>(g_ODPathSetVector, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

	}
}

void g_CloseFiles()
{

	if (g_DebugLogFile);
	fclose(g_DebugLogFile);

	if (g_DebugLogFile);
	fclose(g_DebugLogFile);


	if (g_simulation_log_file != NULL)
		fclose(g_simulation_log_file);
	
	if (g_emission_log_file != NULL)
		fclose(g_emission_log_file);

	if (g_ODME_log_file != NULL)
		fclose(g_ODME_log_file);

	if (g_ODME_result_file != NULL)
		fclose(g_ODME_result_file);


	
	if (g_ABM_log_file != NULL)
		fclose(g_ABM_log_file);
	
	g_LogFile.close();
	g_AssignmentLogFile.close();
	g_NetworkDesignLogFile.close();
	g_WarningFile.close();


	cout << "Assignment-Simulation Completed. " << g_GetAppRunningTime() << endl;
	g_LogFile << "Assignment-Simulation Completed. " << g_GetAppRunningTime() << endl;

}

void g_TrafficAssignmentSimulation()
{

	g_ReadInputFiles();

	cout << "Start Traffic Assignment/Simulation... " << endl;

	// to simplfy the computational process, we only perform agent-based assignment
	if (g_AgentBasedAssignmentFlag == 0)
	{
		cout << "OD Demand based dynamic traffic assignment... " << endl;
		g_ZoneBasedDynamicTrafficAssignmentSimulation(); // multi-iteration dynamic traffic assignment
	}
	else
	{
		cout << "Agent based dynamic traffic assignment... " << endl;
		g_AgentBasedDynamicTrafficAssignmentSimulation();  // agent-based assignment
	}

}


int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{

	int nRetCode = 0;

	// initialize MFC and print and error on failure
	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: MFC initialization failed\n"));
		nRetCode = 1;

		return 0;
	}

	/**********************************************/
	//below is the main traffic assignment-simulation code



	// if  ./DTASettings.ini does not exist, then we should print out all the default settings for user to change
	//
	g_AppStartTime = CTime::GetCurrentTime();
	g_AppLastIterationStartTime = g_AppStartTime;

	//Read DTALite Settings first
	g_ReadDTALiteSettings();

	g_DTALiteMultiScenarioMain();

	//	exit(0);   // rely on operating system to release all memory
	return nRetCode;
}

void DTANetworkForSP::IdentifyBottlenecks(int StochasticCapacityFlag)
{

	g_LogFile << "The following freeway/highway bottlenecks are identified." << endl;

	// ! there is an freeway or highway downstream with less number of lanes
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		if (g_LinkTypeMap[g_LinkVector[li]->m_link_type].IsFreeway() && m_OutboundSizeAry[g_LinkVector[li]->m_ToNodeID] == 1)  // freeway or highway
		{
			int FromID = g_LinkVector[li]->m_FromNodeID;
			int ToID = g_LinkVector[li]->m_ToNodeID;

			for (int i = 0; i < m_OutboundSizeAry[ToID]; i++)
			{
				DTALink* pNextLink = g_LinkVector[m_OutboundLinkAry[ToID][i]];
				if (g_LinkTypeMap[pNextLink->m_link_type].IsFreeway() && pNextLink->m_OutflowNumLanes < g_LinkVector[li]->m_OutflowNumLanes && pNextLink->m_ToNodeID != FromID)
				{
					//					g_LinkVector[li]->m_StochaticCapcityFlag = StochasticCapacityFlag;  //lane drop from current link to next link
					g_LogFile << "lane drop:" << g_NodeVector[g_LinkVector[li]->m_FromNodeID].m_NodeNumber << " ->" << g_NodeVector[g_LinkVector[li]->m_ToNodeID].m_NodeNumber << endl;
				}

			}

		}
	}


	// merge: one outgoing link, two more incoming links with at least freeway link

	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		int incoming_link_freeway_and_ramp_count = 0;
		bool no_arterial_incoming_link = true;
		for (int incoming_link = 0; incoming_link < m_InboundSizeAry[g_LinkVector[li]->m_FromNodeID]; incoming_link++) // one outgoing link without considering u-turn
		{
			int incoming_link_id = m_InboundLinkAry[g_LinkVector[li]->m_FromNodeID][incoming_link];

			if ((g_LinkVector[incoming_link_id]->m_FromNodeID != g_LinkVector[li]->m_ToNodeID)) // non-uturn link
			{
				if (g_LinkTypeMap[g_LinkVector[incoming_link_id]->m_link_type].IsFreeway() //freeway link
					|| g_LinkTypeMap[g_LinkVector[incoming_link_id]->m_link_type].IsRamp())
				{
					incoming_link_freeway_and_ramp_count++;

				}
				else
				{
					no_arterial_incoming_link = false;

				}
			}

		}
		if (incoming_link_freeway_and_ramp_count >= 2 && no_arterial_incoming_link)
		{

			if (m_OutboundSizeAry[g_LinkVector[li]->m_FromNodeID] == 1)
			{
				TRACE("\nMerge link: %d->%d", g_LinkVector[li]->m_FromNodeNumber, g_LinkVector[li]->m_ToNodeNumber);
				g_LinkVector[li]->m_bMergeFlag = 1;

			}

		}

	}

	// first count # of incoming freeway, highway or ramp links to each freeway/highway link
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		int FromID = g_LinkVector[li]->m_FromNodeID;
		if (g_LinkVector[li]->m_bMergeFlag == 1 && m_InboundSizeAry[FromID] == 2)  // is a merge bottlebeck link with two incoming links
		{
			int il;
			bool bRampExistFlag = false;
			bool bFreewayExistFlag = false;

			for (il = 0; il < m_InboundSizeAry[FromID]; il++)
			{
				if (g_LinkTypeMap[g_LinkVector[m_InboundLinkAry[FromID][il]]->m_link_type].IsRamp())  // on ramp as incoming link
				{
					bRampExistFlag = true;

					g_LinkVector[m_InboundLinkAry[FromID][il]]->m_bOnRampType = true;
					g_LinkVector[li]->m_MergeOnrampLinkID = m_InboundLinkAry[FromID][il];
				}
				if (g_LinkTypeMap[g_LinkVector[m_InboundLinkAry[FromID][il]]->m_link_type].IsFreeway() ||
					g_LinkTypeMap[g_LinkVector[m_InboundLinkAry[FromID][il]]->m_link_type].IsHighway())  // freeway or highway
				{
					bFreewayExistFlag = true;
					g_LinkVector[li]->m_MergeMainlineLinkID = m_InboundLinkAry[FromID][il];
				}
				if (bRampExistFlag && bFreewayExistFlag)
				{
					g_LinkVector[li]->m_bMergeFlag = 2; // merge with ramp and mainline street
					g_LogFile << "merge with ramp:" << g_NodeVector[g_LinkVector[li]->m_FromNodeID].m_NodeNumber << " ->" << g_NodeVector[g_LinkVector[li]->m_ToNodeID].m_NodeNumber;
					g_LogFile << " with onramp:" << g_NodeVector[g_LinkVector[g_LinkVector[li]->m_MergeOnrampLinkID]->m_FromNodeID].m_NodeNumber << " ->" << g_NodeVector[g_LinkVector[g_LinkVector[li]->m_MergeOnrampLinkID]->m_ToNodeID].m_NodeNumber;
					g_LogFile << " and freeway mainline:" << g_NodeVector[g_LinkVector[g_LinkVector[li]->m_MergeMainlineLinkID]->m_FromNodeID].m_NodeNumber << " ->" << g_NodeVector[g_LinkVector[g_LinkVector[li]->m_MergeMainlineLinkID]->m_ToNodeID].m_NodeNumber << endl;
					break;
				}

			}


		}

		if (g_LinkVector[li]->m_bMergeFlag == 1 || g_LinkVector[li]->m_bMergeFlag == 2)
		{
			// merge with several merging ramps
			int ij;
			int TotalNumberOfLanes = 0;
			for (ij = 0; ij < m_InboundSizeAry[FromID]; ij++)
			{
				TotalNumberOfLanes += g_LinkVector[m_InboundLinkAry[FromID][ij]]->m_OutflowNumLanes;
			}

			for (ij = 0; ij < m_InboundSizeAry[FromID]; ij++)
			{
				MergeIncomingLink mil;
				mil.m_LinkNo = m_InboundLinkAry[FromID][ij];
				mil.m_link_type = g_LinkVector[mil.m_LinkNo]->m_link_type;
				mil.m_OutflowNumLanes = g_LinkVector[mil.m_LinkNo]->m_OutflowNumLanes;
				mil.m_LinkInCapacityRatio = (float)(mil.m_OutflowNumLanes) / TotalNumberOfLanes;
				g_LinkVector[li]->MergeIncomingLinkVector.push_back(mil);
				g_LogFile << "merge into freeway with multiple freeway/ramps:" << "No." << ij << " " << g_NodeVector[g_LinkVector[mil.m_LinkNo]->m_FromNodeID].m_NodeNumber << " -> " << g_NodeVector[g_LinkVector[mil.m_LinkNo]->m_ToNodeID].m_NodeNumber << " with " << g_LinkVector[mil.m_LinkNo]->m_OutflowNumLanes << " lanes and in flow capacity split " << mil.m_LinkInCapacityRatio << endl;
			}

		}

	}

	// determine offramp
	for (unsigned li = 0; li < g_LinkVector.size(); li++)
	{
		if (g_LinkTypeMap[g_LinkVector[li]->m_link_type].IsRamp())
		{

			if (g_LinkVector[li]->m_bOnRampType == false)
			{

				g_LinkVector[li]->m_bOffRampType = true;

				g_LogFile << "Offramp:" << g_NodeVector[g_LinkVector[li]->m_FromNodeID].m_NodeNumber << " -> " << g_NodeVector[g_LinkVector[li]->m_ToNodeID].m_NodeNumber << endl;

				if (g_settings.use_point_queue_model_for_off_ramps)
				{
					g_LinkVector[li]->m_VehicleSpaceCapacity = 99999; // change to point queue model for offramp
				}
			}
			else
			{
				if (g_settings.use_point_queue_model_for_on_ramps)
				{
					g_LinkVector[li]->m_VehicleSpaceCapacity = 99999; // change to point queue model for offramp
				}

			}


		}

	}
}

void g_OutputSummaryKML(Traffic_MOE moe_mode)
{

	float base_rate = 1;

	FILE* st;
	if (moe_mode == MOE_crashes)
	{
		fopen_s(&st, "simulated_crashes.kml", "w");
		base_rate = 1;
	}
	if (moe_mode == MOE_CO2)
	{
		fopen_s(&st, "simulated_CO2.kml", "w");

		float total_CO2 = 0;
		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];
			total_CO2 += pLink->TotalCO2;
		}

		base_rate = total_CO2 / 5000;  // generate abound 5000 place marks per area
	}
	if (moe_mode == MOE_total_energy)
	{
		fopen_s(&st, "simulated_total_energy.kml", "w");

		float total_energy = 0;
		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];
			total_energy += pLink->TotalEnergy;
		}

		base_rate = total_energy / 5000;  // generate abound 5000 place marks per area
	}


	if (st != NULL)
	{
		fprintf(st, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(st, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
		fprintf(st, "<Document>\n");

		if (moe_mode == MOE_crashes)
			fprintf(st, "<name>simulated_crashes</name>\n");
		if (moe_mode == MOE_CO2)
			fprintf(st, "<name>simulated_emission_CO2_event: unit: %.3f (g//hr)</name>\n", base_rate);
		if (moe_mode == MOE_total_energy)
			fprintf(st, "<name>simulated_t_event: unit: %.3f (J//hr)</name>\n", base_rate);


		int total_count = 0;

		for (unsigned li = 0; li < g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];

			int count = 0;

			float value = 0;

			switch (moe_mode)
			{
			case MOE_CO2:
				value = pLink->TotalCO2 / base_rate; break;
			case MOE_total_energy:
				value = pLink->TotalEnergy / base_rate; break;
			default:
				value = 0;
			}



			count = (int)(value);
			float residual = value - count;
			if (g_GetRandomRatio() < residual)
				count++;


			for (int i = 0; i < count; i++)
			{
				float random_ratio = g_GetRandomRatio();

				float x, y;
				if (pLink->m_ShapePoints.size() == 2)
				{
					x = random_ratio * pLink->m_ShapePoints[0].x + (1 - random_ratio)* pLink->m_ShapePoints[1].x;
					y = random_ratio * pLink->m_ShapePoints[0].y + (1 - random_ratio)* pLink->m_ShapePoints[1].y;
				}
				else if (pLink->m_ShapePoints.size() >0)
				{
					int shape_point_id = pLink->m_ShapePoints.size()*random_ratio;
					x = pLink->m_ShapePoints[shape_point_id].x;
					y = pLink->m_ShapePoints[shape_point_id].y;
				}


				fprintf(st, "\t<Placemark>\n");
				fprintf(st, "\t <name>%d</name>\n", total_count);

	
				if (moe_mode == MOE_CO2 || moe_mode == MOE_total_energy)
				{
					fprintf(st, "\t <description>%d-%d</description>\n", g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber);
				}


				fprintf(st, "\t <Point><coordinates>%f,%f,0</coordinates></Point>\n", x, y);
				fprintf(st, "\t</Placemark>\n");

				total_count++;
			}  // for each count
		} // for each link   
		fprintf(st, "</Document>\n");
		fprintf(st, "</kml>\n");
		fclose(st);
	}
}



void g_SetLinkAttributes(int usn, int dsn, int NumOfLanes)
{
	DTANetworkForSP PhysicalNetwork(g_NodeVector.size(), g_LinkVector.size(), g_PlanningHorizon, g_AdjLinkSize);  //  network instance for single processor in multi-thread environment
	PhysicalNetwork.BuildPhysicalNetwork(0, 0, g_TrafficFlowModelFlag);
	int LinkID = PhysicalNetwork.GetLinkNoByNodeIndex(g_NodeNametoIDMap[usn], g_NodeNametoIDMap[dsn]);

	DTALink* pLink = g_LinkVector[LinkID];
	pLink->m_OutflowNumLanes = NumOfLanes;

}



//void g_OutputDay2DayVehiclePathData(char fname[_MAX_PATH],int StartIteration,int EndIteration)
//{
//
//	ofstream output_path_file;
//	output_path_file.open(fname);
//	if(output_path_file.is_open ())
//	{
//		output_path_file << "from_zone_id,to_zone_id,departure_time,vehicle_id,day_no,node_sum,number_of_nodes,path_sequence" << endl;
//
//		std::map<int, DTAVehicle*>::iterator iterVM;
//		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
//		{
//			DTAVehicle* pVeh = iterVM->second;
//			for(int DayNo = g_StartIterationsForOutputPath; DayNo<= g_EndIterationsForOutputPath; DayNo++)
//			{
//				DTAPath element = pVeh->Day2DayPathMap [DayNo];
//				output_path_file << pVeh->m_OriginZoneID << "," << pVeh->m_DestinationZoneID << "," << pVeh->m_DepartureTime  << ",v" << pVeh->m_AgentID  << "," << DayNo << "," <<
//					element.NodeSum << "," << (int)(element.LinkSequence .size()) << ",{";
//
//				if(element.LinkSequence.size()>=1)
//				{
//					int LinkID_0 = element.LinkSequence [0];
//					output_path_file << g_LinkVector[LinkID_0]->m_FromNodeNumber << ";";
//
//					for(int j = 0; j< element.LinkSequence.size()-1; j++)
//					{
//						int LinkID = element.LinkSequence[j];
//						output_path_file << g_LinkVector[LinkID]->m_ToNodeNumber<< ";";
//					}
//
//				}
//
//
//				output_path_file << "}" <<endl;
//			} // for each day
//		} // for each vehicle
//
//
//		output_path_file.close();
//	}
//}



void g_ErrorMesssage_for_0_demand(std::string file_name, std::string format_type)
{

	if (g_UEAssignmentMethod != analysis_ABM_integration)
	{
		cout << "Error: File " << file_name << " contain 0 trips." << endl;
		cout << "Please check if format_type = " << format_type << " is correct." << endl;
		cout << "DTALite supports the following format: " << g_SupportedDemandFormat << endl;
		g_ProgramStop();
	}
}

void g_DetermineDemandLoadingPeriod()
{

	g_DemandLoadingStartTimeInMin = 1440;
	g_DemandLoadingEndTimeInMin = 0;

	CCSVParser parser0;
	if (parser0.OpenCSVFile("input_demand_file_list.csv"))
	{
		int i = 0;

		while (parser0.ReadRecord())
		{

			int file_sequence_no = -1;
			string file_name;
			string format_type;
			int number_of_lines_to_be_skipped = 0;
			int subtotal_in_last_column = 0;
			int demand_type_in_3rd_column = 0;
			float loading_multiplier = 1;
			int start_time_in_min = -1;
			int end_time_in_min = -1;
			int number_of_demand_types = 0;
			float local_demand_loading_multiplier = 1;
			char demand_type_field_name[20];
			int demand_type_code[20] = { 0 };

			int demand_format_flag = 0;

			parser0.GetValueByFieldNameWithPrintOut("file_sequence_no", file_sequence_no);

			if (file_sequence_no <= -1)  // skip negative sequence no 
				break;

			parser0.GetValueByFieldNameWithPrintOut("file_name", file_name);
			if (file_name.length() == 0)  // no file name input
			{
				break;
			}

			parser0.GetValueByFieldNameWithPrintOut("start_time_in_min", start_time_in_min);
			parser0.GetValueByFieldNameWithPrintOut("end_time_in_min", end_time_in_min);

			if (start_time_in_min == -1)  // skip negative sequence no 
			{
				cout << "Please provide start_time_in_min in file input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}
			if (end_time_in_min == -1)  // skip negative sequence no 
			{
				cout << "Please provide end_time_in_min in file input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}

			if (end_time_in_min > 2880)
			{
				cout << "end_time_in_min should be less than 2880 min in input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}

			if (start_time_in_min < 0)
			{
				cout << "start_time_in_min should be greater than 0 min in input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}

			// set g_DemandLoadingStartTimeInMin according the start time and end time of each record
			if (g_DemandLoadingStartTimeInMin > start_time_in_min)
				g_DemandLoadingStartTimeInMin = start_time_in_min;

		if (g_DemandLoadingEndTimeInMin < end_time_in_min)
				g_DemandLoadingEndTimeInMin = end_time_in_min;

		}

	}  //determine loading horizon

	if (g_DemandLoadingStartTimeInMin > g_DemandLoadingEndTimeInMin)
	{  // reset
		g_DemandLoadingStartTimeInMin = 0;
		g_DemandLoadingEndTimeInMin = 1400;
		g_PlanningHorizon = 1440;

	}

	cout << "demand loading starting time = " << g_DemandLoadingStartTimeInMin << " (min)" << endl;
	cout << "demand loading ending time = " << g_DemandLoadingEndTimeInMin << " (min)" << endl;

	if (g_PlanningHorizon < g_DemandLoadingEndTimeInMin + 300)
	{
		//reset simulation horizon to make sure it is longer than the demand loading horizon
		g_PlanningHorizon = g_DemandLoadingEndTimeInMin + 300;
	}
}
void g_ReadDemandFileBasedOnDemandFileList()
{

	float total_demand_in_demand_file = 0;
	float total_number_of_vehicles_to_be_generated = 0;

	int max_line_number_for_logging = 10;

	g_DetermineDemandLoadingPeriod();



	////step 2:
	if (g_ODEstimationFlag == 1)
	{
		g_SystemDemand.Initialize();
	}

	CCSVParser parser;


	//step 3:
	if (parser.OpenCSVFile("input_demand_file_list.csv"))
	{
		int i = 0;

		while (parser.ReadRecord())
		{

			int file_sequence_no = 1;
			string file_name;
			string format_type = "null";
			int number_of_lines_to_be_skipped = 0;
			int subtotal_in_last_column = 0;
			int demand_type_in_3rd_column = 0;

			int start_time_in_min = 0;
			int end_time_in_min = 1440;
			int number_of_demand_types = 0;
			float local_demand_loading_multiplier = 1;
			char demand_type_field_name[20];
			int demand_type_code[20] = { 0 };

			int demand_format_flag = 0;

			if (parser.GetValueByFieldNameWithPrintOut("file_sequence_no", file_sequence_no) == false)
				break;

			if (file_sequence_no <= -1)  // skip negative sequence no 
				continue;

			parser.GetValueByFieldNameWithPrintOut("file_name", file_name);



			parser.GetValueByFieldNameWithPrintOut("start_time_in_min", start_time_in_min);
			parser.GetValueByFieldNameWithPrintOut("end_time_in_min", end_time_in_min);

			if (end_time_in_min > 2880)
			{
				cout << "end_time_in_min should be less than 2880 min in input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}

			if (start_time_in_min < 0)
			{
				cout << "start_time_in_min should be greater than 0 min in input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}
			if (file_name.length() == 0)  // no file name input
			{
				break;
			}

			parser.GetValueByFieldNameWithPrintOut("format_type", format_type);
			if (format_type.find("null") != string::npos)  // skip negative sequence no 
			{
				cout << "Please provide format_type in file input_demand_file_list.csv" << endl;
				g_ProgramStop();
			}

			{ // error checking

				if (file_name.find("AMS_OD_table.csv") != string::npos && format_type.find("column") == string::npos)
				{
					cout << "Please specify column format for demand file AMS_OD_table.csv, other than " << format_type << endl;
				}

				if (file_name.find("demand.dat") != string::npos && format_type.find("dynasmart") == string::npos)
				{
					cout << "Please specify dynasmart format for demand file demand.dat, other than " << format_type << endl;
				}

				if (file_name.find("demand_HOV.dat") != string::npos && format_type.find("dynasmart") == string::npos)
				{
					cout << "Please specify dynasmart format for demand file demand_HOV.dat, other than " << format_type << endl;
				}

				if (file_name.find("demand_truck.dat") != string::npos && format_type.find("dynasmart") == string::npos)
				{
					cout << "Please specify dynasmart format for demand_truck file demand.dat, other than " << format_type << endl;
				}
				if (file_name.compare("agent.bin") == 0 && format_type.find("agent_bin") == string::npos)
				{
					cout << "Please specify agent_bin format for agent binary file , other than " << format_type << endl;
				}

			}

			parser.GetValueByFieldNameWithPrintOut("number_of_lines_to_be_skipped", number_of_lines_to_be_skipped);
			parser.GetValueByFieldNameWithPrintOut("subtotal_in_last_column", subtotal_in_last_column);
			parser.GetValueByFieldName("demand_type_in_3rd_column", demand_type_in_3rd_column);

			int apply_additional_time_dependent_profile = 0;
			parser.GetValueByFieldNameWithPrintOut("apply_additional_time_dependent_profile", apply_additional_time_dependent_profile);
			parser.GetValueByFieldNameWithPrintOut("loading_multiplier", local_demand_loading_multiplier);


			double time_dependent_ratio[MAX_TIME_INTERVAL_SIZE] = { 0 };

			double total_ratio = 0;
			if (apply_additional_time_dependent_profile == 1)
			{
				for (int time_index = start_time_in_min / 15; time_index < end_time_in_min / 15; time_index++)  // / 15 converts min to 15-min interval for demand patterns
				{
					std::string time_stamp_str = g_GetTimeStampStrFromIntervalNo(time_index);

					time_dependent_ratio[time_index] = 0;
					parser.GetValueByFieldNameWithPrintOut(time_stamp_str, time_dependent_ratio[time_index]);
					total_ratio += time_dependent_ratio[time_index];
				}


				if (total_ratio < 0.001)
				{
					cout << "Error: apply_additional_time_dependent_profile = 1, but the total temporal ratio read from file input_temporal_demand_profile.csv is 0, which means no demand will be loaded. " << endl;
					g_ProgramStop();
				}

			}

			parser.GetValueByFieldNameWithPrintOut("number_of_demand_types", number_of_demand_types);

			if (demand_type_in_3rd_column == 1)
			{
				if (number_of_demand_types != 1)
				{
					cout << "Error: number_of_demand_types should be 1 when demand_type_in_3rd_column is set to 1. The current value is " << number_of_demand_types << endl;
					g_ProgramStop();

				}

			}


			for (int type = 1; type <= number_of_demand_types; type++)
			{
				sprintf(demand_type_field_name, "demand_type_%d", type);
				int demand_type = -1;
				parser.GetValueByFieldNameWithPrintOut(demand_type_field_name, demand_type);

				if (demand_type <= 0)
				{
					cout << "Missing input: no value has been specified for field " << demand_type_field_name << " in file " << file_name << " in demand meta file input_demand_file_list.csv. " << endl;
					g_ProgramStop();

				}


				demand_type_code[type] = demand_type;
			}






			if (format_type.find("column") != string::npos)  // or muliti-column
			{

				if (number_of_demand_types == 0)
				{
					cout << "number_of_demand_types = 0 in file input_demand_file_list.csv. Please check." << endl;
					g_ProgramStop();
				}

				bool bFileReady = false;
				int i;

				FILE* st;

				//test # of numerical values per line
				fopen_s(&st, file_name.c_str(), "r");
				if (st != NULL)
				{

					char  str_line[_MAX_STRING_LINE]; // input string
					int str_line_size = _MAX_STRING_LINE;

					// skip lines
					for (int line_skip = 0; line_skip < number_of_lines_to_be_skipped; line_skip++)
					{
						str_line_size = _MAX_STRING_LINE;
						g_read_a_line(st, str_line, str_line_size); //  skip the first line
						cout << str_line << endl;
					}

					str_line_size = _MAX_STRING_LINE;
					g_read_a_line(st, str_line, str_line_size); //  skip the first line
					int number_of_values = g_read_number_of_numerical_values(str_line, str_line_size);

					if (demand_type_in_3rd_column == 1)
					{

						if (number_of_values != 4)
						{
							cout << "demand_type_in_3rd_column = 1, please make sure there are 4 values per line" << endl;
							g_ProgramStop();

						}

					}

					if (number_of_values != 2 + number_of_demand_types + demand_type_in_3rd_column + subtotal_in_last_column) // 2: origin, destination (demand type),  values for each demand type, subtotal
					{
						cout << "There are " << number_of_values << " values() per line in file " << file_name << "," << endl << "but " << number_of_demand_types << " demand type(s) are defined in file input_demand_file_list.csv. " << endl << "Please check file input_demand_file_list.csv." << endl;
						g_ProgramStop();

					}
					fclose(st);
				}

				// read the file formaly after the test. 

				fopen_s(&st, file_name.c_str(), "r");
				if (st != NULL)
				{
					char  str_line[2000]; // input string
					int str_line_size = _MAX_STRING_LINE;

					// skip lines
					for (int line_skip = 0; line_skip < number_of_lines_to_be_skipped; line_skip++)
					{
						str_line_size = _MAX_STRING_LINE;
						g_read_a_line(st, str_line, str_line_size); //  skip the first line
						cout << str_line << endl;
					}

					bFileReady = true;
					int line_no = number_of_lines_to_be_skipped;

					while (true)
					{
						int origin_zone = g_read_integer(st);

						if (origin_zone <= 0)
						{

							if (line_no == 1 && !feof(st))  // read only one line, but has not reached the end of the line
							{
								cout << endl << "Error: Only one line has been read from file. Are there multiple columns of demand type in file " << file_name << " per line?" << endl;
								g_ProgramStop();

							}
							break;
						}

						if (origin_zone > g_ODZoneNumberSize)
						{
							cout << endl << "Error: Line " << line_no << " origin zone = " << origin_zone << ", which is greater than the maximum zone number. Please check." << endl;
							g_ProgramStop();
						}

						int destination_zone = g_read_integer(st);

						if (destination_zone > g_ODZoneNumberSize)
						{
							cout << endl << "Error: Line " << line_no << " destination zone = " << destination_zone << ", which is greater than the maximum zone number of " << g_ODZoneNumberSize << " in input_zone.csv. Please check." << endl;
							g_ProgramStop();
						}


						if (demand_type_in_3rd_column == 1)
						{
							demand_type_code[1] = g_read_integer(st);  // read the user specified demand type per row

						}

						float number_of_vehicles;




						for (int type = 1; type <= number_of_demand_types; type++)
						{

							float demand_value = g_read_float_from_a_line(st);

							if (demand_value < -99) // encounter return 
							{
								break;
							}

							number_of_vehicles = demand_value*g_DemandGlobalMultiplier*local_demand_loading_multiplier;

							if (demand_type_code[type] >= 1)  // load this demand type
							{

								total_demand_in_demand_file += number_of_vehicles;


								// we generate vehicles here for each OD data line
								if (line_no <= 5)  // read only one line, but has not reached the end of the line
									cout << "origin:" << origin_zone << ", destination: " << destination_zone << ", value = " << number_of_vehicles << endl;

								if (g_ZoneMap.find(origin_zone) != g_ZoneMap.end())
								{
									g_ZoneMap[origin_zone].m_Demand += number_of_vehicles;

									if (apply_additional_time_dependent_profile == 1)  // use time-dependent profile
									{
										for (int time_interval = start_time_in_min / 15; time_interval < end_time_in_min / 15; time_interval++)
										{
											if (time_dependent_ratio[time_interval] > 0.000001) // this is the last one applicable
											{
												// reset the time interval, create vehicles with the same origin, destination, changed # of vehicles, and time interval
												double number_of_vehicles_to_be_loaded = time_dependent_ratio[time_interval] * number_of_vehicles;
												CreateVehicles(origin_zone, destination_zone, number_of_vehicles_to_be_loaded, demand_type_code[type], time_interval * 15, (time_interval + 1) * 15);
											}
										}
									}
									else // do not use time-dependent profile
									{
										CreateVehicles(origin_zone, destination_zone, number_of_vehicles, demand_type_code[type], start_time_in_min, end_time_in_min);

									}
								}
								//  given the number of OD demand voluem to be created. we need to apply time-dependent profile for each data block , 


								if (line_no % 100000 == 0)
								{
									cout << g_GetAppRunningTime() << "Reading file no." << file_sequence_no << ": " << file_name << " at " << line_no / 1000 << "K lines..." << endl;
								}


							}
							else if (type != 0)  // if demand type == 0 then we will skip this value. By doing so, we can reading only one demand type per record with demand-type specific departure time loading profile. e.g. truck vs. HOV
							{
								cout << "demand type " << type << " in file input_demand_file_list has not been defined. Please check." << endl;
								g_ProgramStop();

							}
						}  // for each demand type

						if (subtotal_in_last_column == 1)
							g_read_float_from_a_line(st);  // so skip this last number

						//if(line_no >= max_line_number)
						//break;
						line_no++;
					}  // scan lines

					if (total_demand_in_demand_file < 0.5f)
					{
						g_ErrorMesssage_for_0_demand(file_name, format_type);

					}

					fclose(st);
				}
				else  //open file
				{
					cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
					g_ProgramStop();

				}

				CString demand_file_str;

				demand_file_str.Format("Reading demand file,%s,cumulative # of agents=,%d,\n", file_name.c_str(), g_simple_vector_vehicles.size());
				g_SummaryStatFile.WriteTextLabel(demand_file_str);


			}
			else if (format_type.compare("emme_matrix") == 0)
			{

				if (number_of_demand_types == 0)
				{
					cout << "number_of_demand_types = 0 in file input_demand_file_list.csv. Please check." << endl;
					g_ProgramStop();
				}

				bool bFileReady = false;
				int i;

				FILE* st;

				//test # of numerical values per line
				fopen_s(&st, file_name.c_str(), "r");
				if (st != NULL)
				{

					char  str_line[_MAX_STRING_LINE]; // input string
					int str_line_size = _MAX_STRING_LINE;
					int line_no = 1;

					// skip lines
					for (int line_skip = 0; line_skip < number_of_lines_to_be_skipped; line_skip++)
					{
						str_line_size = _MAX_STRING_LINE;
						g_read_a_line(st, str_line, str_line_size); //  skip the first line
						cout << str_line << endl; \

							line_no++;
					}

					int type = 1;


					while (true)
					{
						std::vector<float> ValueVector;
						int end_of_flag_flag = g_read_numbers_from_a_line(st, ValueVector);

						if (end_of_flag_flag < 0)  // end of file
							break;

						if (ValueVector.size() >= 3)  // origin, destination, value
						{

							int origin_zone = ValueVector[0];

							if (origin_zone <= 0)
							{

								if (line_no == 1 && !feof(st))  // read only one line, but has not reached the end of the line
								{
									cout << endl << "Error: Only one line has been read from file. Are there multiple columns of demand type in file " << file_name << " per line?" << endl;
									g_ProgramStop();

								}
								break;
							}



							if (origin_zone > g_ODZoneNumberSize)
							{
								cout << endl << "Error: Line " << line_no << " origin zone = " << origin_zone << ", which is greater than the maximum zone number. Please check." << endl;
								g_ProgramStop();
							}

							int number_of_OD_pairs = (ValueVector.size() - 1) / 2;
							for (int n = 0; n< number_of_OD_pairs; n++)
							{
								int destination_zone = ValueVector[2 * n + 1];

								if (destination_zone > g_ODZoneNumberSize)
								{
									cout << endl << "Error: Line " << line_no << " destination zone = " << destination_zone << ", which is greater than the maximum zone number of " << g_ODZoneNumberSize << " in input_zone.csv. Please check." << endl;
									g_ProgramStop();
								}



								float number_of_vehicles = 0;


								float demand_value = ValueVector[2 * n + 2];

								if (demand_value < -99) // encounter return 
								{
									break;
								}

								number_of_vehicles = demand_value*g_DemandGlobalMultiplier*local_demand_loading_multiplier;

								if (demand_type_code[type] >= 1)  // load this demand type
								{

									total_demand_in_demand_file += number_of_vehicles;

									// we generate vehicles here for each OD data line
									if (line_no <= 5)  // read only one line, but has not reached the end of the line
										cout << "origin:" << origin_zone << ", destination: " << destination_zone << ", value = " << number_of_vehicles << endl;

									if (g_ZoneMap.find(origin_zone) != g_ZoneMap.end())
									{
										g_ZoneMap[origin_zone].m_Demand += number_of_vehicles;

										if (apply_additional_time_dependent_profile == 1)  // use time-dependent profile
										{
											for (int time_interval = start_time_in_min / 15; time_interval < end_time_in_min / 15; time_interval++)
											{
												if (time_dependent_ratio[time_interval] > 0.000001) // this is the last one applicable
												{
													// reset the time interval, create vehicles with the same origin, destination, changed # of vehicles, and time interval
													double number_of_vehicles_to_be_loaded = time_dependent_ratio[time_interval] * number_of_vehicles;
													CreateVehicles(origin_zone, destination_zone, number_of_vehicles_to_be_loaded, demand_type_code[type], time_interval * 15, (time_interval + 1) * 15);
												}
											}
										}
										else // do not use time-dependent profile
										{
											float number_of_time_dependent_intervals = max(1, (end_time_in_min - start_time_in_min) / g_AggregationTimetInterval);
											CreateVehicles(origin_zone, destination_zone, number_of_vehicles, demand_type_code[type], start_time_in_min, end_time_in_min);

										}
									}
									//  given the number of OD demand voluem to be created. we need to apply time-dependent profile for each data block , 


									if (line_no % 100000 == 0)
									{
										cout << g_GetAppRunningTime() << "Reading file no." << file_sequence_no << ": " << file_name << " at " << line_no / 1000 << "K lines..." << endl;
									}


								}
								else if (type != 0)  // if demand type == 0 then we will skip this value. By doing so, we can reading only one demand type per record with demand-type specific departure time loading profile. e.g. truck vs. HOV
								{
									cout << "demand type " << type << " in file input_demand_file_list has not been defined. Please check." << endl;
									g_ProgramStop();

								}

							}  // for each OD pair
						}   // end if 
						//if(line_no >= max_line_number)
						//break;
						line_no++;
					}  // scan lines

					if (total_demand_in_demand_file < 0.5f)
					{
						g_ErrorMesssage_for_0_demand(file_name, format_type);

					}

					fclose(st);
				}
				else  //open file
				{
					cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
					g_ProgramStop();

				}

				CString demand_file_str;

				demand_file_str.Format("Reading demand file,%s,cumulative # of agents=,%d,\n", file_name.c_str(), g_simple_vector_vehicles.size());
				g_SummaryStatFile.WriteTextLabel(demand_file_str);


			}
			else if (format_type.compare("matrix") == 0)
			{

				if (g_detect_if_a_file_is_column_format(file_name.c_str()) == true)
				{
					CString str;
					str.Format("Demand input file %s looks to be based on column format,\nbut format_type=matrix in input_demand_file_list.csv.\nPlease check the demand file format, and change format_type=column in input_demand_file_list.cv.\nClick any key to exit.", file_name.c_str());
					cout << str;

					getchar();
					exit(0);

				}
				vector<int> LineIntegerVector;

				CCSVParser parser;
				parser.IsFirstLineHeader = false;
				if (parser.OpenCSVFile(file_name))
				{
					int control_type_code;
					int i = 0;
					if (parser.ReadRecord())
					{
						parser.ConvertLineStringValueToIntegers();
						LineIntegerVector = parser.LineIntegerVector;
					}

				}

				int number_of_zones = LineIntegerVector.size();


				bool bFileReady = false;
				int i;

				FILE* st;
				fopen_s(&st, file_name.c_str(), "r");
				if (st != NULL)
				{
					// read the first line
					g_read_a_line(st);

					cout << "number of zones to be read = " << number_of_zones << endl;

					//test if a zone has been defined. 
					for (int destination_zone_index = 0; destination_zone_index < number_of_zones; destination_zone_index++)
					{
						int zone = LineIntegerVector[destination_zone_index];

						if (g_ZoneMap.find(zone) == g_ZoneMap.end())
						{
							cout << "Zone " << zone << " (no." << destination_zone_index + 1 << " in the first line) of file " << file_name << " has not been defined in input_zone.csv. Please check." << endl;
							g_ProgramStop();

						}
					}



					int line_no = 0;
					for (int origin_zone_index = 0; origin_zone_index < number_of_zones; origin_zone_index++)
					{
						int origin_zone = g_read_integer(st); // read the origin zone number


						cout << "Reading file no." << file_sequence_no << " " << file_name << " at zone " << origin_zone << " ... " << endl;

						for (int destination_zone_index = 0; destination_zone_index < number_of_zones; destination_zone_index++)
						{
							int destination_zone = LineIntegerVector[destination_zone_index];
							float value = g_read_float(st);

							float number_of_vehicles = value*g_DemandGlobalMultiplier*local_demand_loading_multiplier;  // read the value

							if (line_no <= 5)  // read only one line, but has not reached the end of the line
								cout << "origin:" << origin_zone << ", destination: " << destination_zone << ", value = " << number_of_vehicles << endl;

							line_no++;
							int type = 1;  // first demand type definition
							if (demand_type_code[type] >= 1)  // feasible demand type
							{
								total_demand_in_demand_file += number_of_vehicles;

								g_ZoneMap[origin_zone].m_Demand += number_of_vehicles;
								// condition 1: without time-dependent profile 

								if (apply_additional_time_dependent_profile == 1)  // use time-dependent profile
								{
									for (int time_interval = start_time_in_min / 15; time_interval < end_time_in_min / 15; time_interval++)
									{
										if (time_dependent_ratio[time_interval] > 0.000001) // this is the last one applicable
										{
											// reset the time interval, create vehicles with the same origin, destination, changed # of vehicles, and time interval
											double number_of_vehicles_to_be_loaded = time_dependent_ratio[time_interval] * number_of_vehicles;

											CreateVehicles(origin_zone, destination_zone, number_of_vehicles_to_be_loaded, demand_type_code[type], time_interval * 15, (time_interval + 1) * 15);
										}
									}
								}
								else // do not use time-dependent profile
								{

									float number_of_time_dependent_intervals = max(1, (end_time_in_min - start_time_in_min) / 15);

									CreateVehicles(origin_zone, destination_zone, number_of_vehicles, demand_type_code[type], start_time_in_min, end_time_in_min);

								}


							}

						}
						//
						if (subtotal_in_last_column == 1)
							g_read_float(st); //read sub total value

					}

					if (total_demand_in_demand_file < 0.5f)
					{
						g_ErrorMesssage_for_0_demand(file_name, format_type);
					}

					fclose(st);
				}
				else  //open file
				{
					cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
					g_ProgramStop();

				}

			}
			else if (format_type.compare("full_matrix") == 0)
			{
				vector<int> LineIntegerVector;

				CCSVParser parser;
				parser.IsFirstLineHeader = false;
				if (parser.OpenCSVFile(file_name))
				{
					int control_type_code;
					int i = 0;
					if (parser.ReadRecord())
					{
						parser.ConvertLineStringValueToIntegers();
						LineIntegerVector = parser.LineIntegerVector;
					}

				}

				int number_of_zones = LineIntegerVector.size();


				bool bFileReady = false;
				int i;

				FILE* st;
				fopen_s(&st, file_name.c_str(), "r");
				if (st != NULL)
				{
					// read the first line
					g_read_a_line(st);

					cout << "number of zones to be read = " << number_of_zones << endl;

					int line_no = 0;


					for (int origin_zone_index = 0; origin_zone_index < number_of_zones; origin_zone_index++)
					{
						int origin_zone = g_read_integer(st); // read the origin zone number

						if(origin_zone == -1)
							break;
						if (g_ZoneMap.find(origin_zone) == g_ZoneMap.end())
						{
							continue; // origin zone  has not been defined, skipped. 
						}

						int total_demand = (int)(total_demand_in_demand_file);
						if (origin_zone % 100 == 0)
							cout << "Reading file no." << file_sequence_no << ": " << file_name << " at zone " << origin_zone << ",total demand = " << total_demand << "..." << endl;

						for (int destination_zone_index = 0; destination_zone_index < number_of_zones; destination_zone_index++)
						{
							int destination_zone = LineIntegerVector[destination_zone_index];
							float value = g_read_float(st);


							float number_of_vehicles = value*g_DemandGlobalMultiplier*local_demand_loading_multiplier;  // read the value

							if (line_no <= 5)  // read only one line, but has not reached the end of the line
								cout << "origin:" << origin_zone << ", destination: " << destination_zone << ", value = " << number_of_vehicles << endl;

							line_no++;
							int type = 1;  // first demand type definition
							if (demand_type_code[type] >= 1)  // feasible demand type
							{
								total_demand_in_demand_file += number_of_vehicles;

								g_ZoneMap[origin_zone].m_Demand += number_of_vehicles;
								// condition 1: without time-dependent profile 

								if (apply_additional_time_dependent_profile == 1)  // use time-dependent profile
								{
									for (int time_interval = start_time_in_min / 15; time_interval < end_time_in_min / 15; time_interval++)
									{
										if (time_dependent_ratio[time_interval] > 0.000001) // this is the last one applicable
										{
											// reset the time interval, create vehicles with the same origin, destination, changed # of vehicles, and time interval
											double number_of_vehicles_to_be_loaded = time_dependent_ratio[time_interval] * number_of_vehicles;

											CreateVehicles(origin_zone, destination_zone, number_of_vehicles_to_be_loaded, demand_type_code[type], time_interval * 15, (time_interval + 1) * 15);
										}
									}
								}
								else // do not use time-dependent profile
								{

									float number_of_time_dependent_intervals = max(1, (end_time_in_min - start_time_in_min) / 15);

									CreateVehicles(origin_zone, destination_zone, number_of_vehicles, demand_type_code[type], start_time_in_min, end_time_in_min);

								}


							}

						}
						//
						if (subtotal_in_last_column == 1)
							g_read_float(st); //read sub total value

					}

					if (total_demand_in_demand_file < 0.5f)
					{
						g_ErrorMesssage_for_0_demand(file_name, format_type);
					}

					fclose(st);
				}
				else  //open file
				{
					cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
					g_ProgramStop();

				}


			}
			else if (format_type.find("agent_csv") != string::npos)
			{
				g_AgentBasedAssignmentFlag = 1;
				if (g_ReadTripCSVFile(file_name, false) == false)
				{
					cout << "input agent file " << file_name << " is missing. Please check." << endl;
					g_ProgramStop();
				}
			}
			else if (format_type.find("agent_bin") != string::npos)
			{
				g_AgentBasedAssignmentFlag = 1;
				g_ReadAgentBinFile(file_name, false);
				return;
			}
			else if (format_type.find("dynasmart") != string::npos)
			{

				int type = 1;  // first demand type definition
				if (demand_type_code[type] >= 1)  // feasible demand type
				{

					bool bFileReady = false;
					int i;

					FILE* st;
					fopen_s(&st, file_name.c_str(), "r");
					if (st != NULL)
					{
						int num_zones = g_ZoneMap.size();
						int num_matrices = 0;

						num_matrices = g_read_integer(st);
						float demand_factor = g_read_float(st);

						std::vector<int> TimeIntevalVector;
						// Start times
						int i;
						for (i = 0; i < num_matrices; i++)
						{
							int start_time = g_read_float(st);
							TimeIntevalVector.push_back(start_time);

						}

						int time_interval = 60; // min

						if (TimeIntevalVector.size() >= 2)
							time_interval = TimeIntevalVector[1] - TimeIntevalVector[0];

						// read the last value
						int end_of_simulation_horizon = g_read_float(st);

						TimeIntevalVector.push_back(end_of_simulation_horizon);

						long line_no = 2;
						float total_demand = 0;
						for (i = 0; i < num_matrices; i++)
						{
							// Find a line with non-blank values to start
							// Origins
							double DSP_start_time = g_read_float(st) + start_time_in_min; // start time
							double DSP_end_time = DSP_start_time + time_interval; // end time

							for (int origin_zone = 1; origin_zone <= num_zones; origin_zone++)
							for (int destination_zone = 1; destination_zone <= num_zones; destination_zone++)
							{
								float value = g_read_float(st);
								float number_of_vehicles = value* demand_factor*local_demand_loading_multiplier * g_DemandGlobalMultiplier;
								total_demand_in_demand_file += number_of_vehicles;

								// obtain demand table type
								float number_of_time_dependent_intervals = max(1, (DSP_end_time - DSP_start_time) / 15);


								g_ZoneMap[origin_zone].m_Demand += number_of_vehicles;


								if (origin_zone != destination_zone)
								{
									CreateVehicles(origin_zone, destination_zone, number_of_vehicles, demand_type_code[type], DSP_start_time, DSP_end_time);
									line_no++;

									if ((line_no) % 6000 == 0 && line_no > 0)
									{
										cout << g_GetAppRunningTime() << "Reading file " << file_name << " at " << line_no / 6000 << "K lines..." << endl;
									}

									line_no++;
								}

							}

						} // time-dependent matrix

						fclose(st);
					}
					else
					{

						cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
						g_ProgramStop();

						//
					}

					if (total_demand_in_demand_file < 0.5f)
					{
						g_ErrorMesssage_for_0_demand(file_name, format_type);
					}


				}
				else  //open file
				{
					cout << "Error: File " << file_name << " cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
					g_ProgramStop();

				}

			}
			else if (format_type.find("trip_csv") != string::npos)
			{
				int number_of_vehicles = 0;
				cout << "Reading file no." << file_sequence_no << ": " << file_name << " ... " << endl;

				if (g_ReadTripCSVFile(file_name, false) == false)
				{
					cout << "input agent file " << file_name << " is missing. Please check." << endl;
					g_ProgramStop();

				}
				total_demand_in_demand_file += number_of_vehicles;
			}
			else
			{
				cout << "Error: format_type = " << format_type << " is not supported. Currently DTALite supports multi_column, matrix, full_matrix, dynasmart, agent_csv, agent_bin, trip_csv,transims_trip_file." << endl;
				g_ProgramStop();

			}
		}
	}


	g_SummaryStatFile.WriteRecord();
	g_LogFile << "Total demand volume = " << total_demand_in_demand_file << g_GetUsedMemoryDataInMB() << endl;

	cout << "Total demand volume = " << total_demand_in_demand_file << endl;

	// create vehicle heres...
	cout << "Step 11: Converting demand flow to vehicles..." << endl;

	// round the demand loading horizon using g_AggregationTimetInterval as time unit
	g_AllocateDynamicArrayForVehicles();

	for (int z = 1; z <= g_ODZoneNumberSize; z++)  // for each zone 
	{
		if (g_ZoneMap.find(z) != g_ZoneMap.end())
		{
			int zone_index = g_ZoneMap[z].m_ZoneSequentialNo;

			if (g_SystemDemand.m_alpha != NULL && zone_index >= 0)
			{

				_proxy_ODME_log(0, 0, "initial alpha[%d]= %f\n", z, g_SystemDemand.m_alpha[zone_index]);
			}
		}
	}

	g_ConvertDemandToVehicles();
	g_LogFile << "After creating memory for vehicle set = " << total_demand_in_demand_file << g_GetUsedMemoryDataInMB() << endl;

}
void g_ReadAssignmentPeriodSettings()
{
	g_SummaryStatFile.WriteTextLabel("\n");
	g_NumberOfSPCalculationPeriods = min(MAX_TIME_INTERVAL_SIZE - 1, max(1, (g_PlanningHorizon - g_DemandLoadingStartTimeInMin) / g_AggregationTimetInterval));

	for (int t = 0; t < MAX_TIME_INTERVAL_SIZE; t++)
	{
		g_AssignmentIntervalIndex[t] = 0;   // if we have file input, then we need to set the interval index to 0 for all intervals
		g_AssignmentIntervalOutputFlag[t] = 0;
	}

	for (int index = 0; index < g_NumberOfSPCalculationPeriods; index++)
	{
		g_AssignmentIntervalIndex[g_DemandLoadingStartTimeInMin / g_AggregationTimetInterval + index] = index;   // internal index for memory block, natural order of 15 min in case we do not have file input
		g_AssignmentIntervalStartTimeInMin[index] = g_DemandLoadingStartTimeInMin + index * g_AggregationTimetInterval;
		g_AssignmentIntervalEndTimeInMin[index] = g_DemandLoadingStartTimeInMin + (index + 1) * g_AggregationTimetInterval;
	}


		for (int t = 0; t < MAX_TIME_INTERVAL_SIZE; t++)
		{
			g_AssignmentIntervalIndex[t] = 0;   // if we have file input, then we need to set the interval index to 0 for all intervals
		}

		g_AssignmentIntervalIndex[int(g_DemandLoadingStartTimeInMin / 15)] = 0;
		g_AssignmentIntervalOutputFlag[int(g_DemandLoadingStartTimeInMin / 15)] = 1;
		int index_counter = 0;

		g_AssignmentIntervalStartTimeInMin[0] = g_DemandLoadingStartTimeInMin;


			int index = 0;


			for (int i = 0; i < MAX_TIME_INTERVAL_SIZE; i++)
			{

				if (i * 15 >= g_DemandLoadingStartTimeInMin && i * 15 < g_PlanningHorizon)
				{ 
					//CString time_string;

					//int hour = i / 4;
					//int min = (i - hour * 4) * 15;

					//if (hour<10)
					//	time_string.Format("'0%d:%02d", hour, min);
					//else
					//	time_string.Format("'%2d:%02d", hour, min);

					int period_start_flag = 1; //g_GetPrivateProfileInt("skim_output", time_string, 1, g_DTASettingFileName);
					g_AssignmentIntervalOutputFlag[i] = period_start_flag;
					if (period_start_flag == 1 && i * 15 != g_DemandLoadingStartTimeInMin)   // if see 1, increaese the index counter by 1, if we are at the boundary of interval then there is no need to swtich to next time interval
					{
						index_counter++;

						g_AssignmentIntervalEndTimeInMin[index_counter - 1] = i * 15;
						g_AssignmentIntervalStartTimeInMin[index_counter] = i * 15;
					}

					g_AssignmentIntervalIndex[i] = index_counter;

				}


			}

		g_AssignmentIntervalEndTimeInMin[index_counter] = g_DemandLoadingEndTimeInMin;  // close the last time interval with the end time 

		g_NumberOfSPCalculationPeriods = index_counter + 1;  // final update from assignment time interval file

	//	g_SummaryStatFile.WriteParameterValue("User Defined Assignment Intervals=", g_NumberOfSPCalculationPeriods);


		//for (int ti = 0; ti < g_NumberOfSPCalculationPeriods; ti++)
	//{
	//	CString analysis_interval_str;
	//	analysis_interval_str.Format("No.%d", ti + 1);

	//	CString analysis_interval_period_str;
	//	analysis_interval_period_str.Format("%s-> %s", GetTimeClockString(g_AssignmentIntervalStartTimeInMin[ti]).c_str(), GetTimeClockString(g_AssignmentIntervalEndTimeInMin[ti]).c_str());
	//	g_SummaryStatFile.WriteParameterValue(analysis_interval_str, analysis_interval_period_str);
	//}
	g_SummaryStatFile.WriteTextLabel("\n");
}


void g_BuildGridSystem()
{

	//build grid network 
	g_GridMatrix = AllocateDynamicArray<GridNodeSet>(_MAX_TRANSIT_GRID_SIZE, _MAX_TRANSIT_GRID_SIZE);

	bool bRectInitialized = false;
	for (int i = 0; i < g_NodeVector.size(); i++)
	{
		if (!bRectInitialized)
		{
			g_GridRect.left = g_NodeVector[i].m_pt.x;
			g_GridRect.right = g_NodeVector[i].m_pt.x;
			g_GridRect.top = g_NodeVector[i].m_pt.y;
			g_GridRect.bottom = g_NodeVector[i].m_pt.y;
			bRectInitialized = true;
		}

		g_GridRect.Expand(g_NodeVector[i].m_pt);
	}

	g_GridXStep = max(0.0001, g_GridRect.Width() / _MAX_TRANSIT_GRID_SIZE);

	g_GridYStep = max(0.0001, g_GridRect.Height() / _MAX_TRANSIT_GRID_SIZE);


	for (int i = 0; i < g_NodeVector.size(); i++)
	{

		int x_key = (g_NodeVector[i].m_pt.x - g_GridRect.left) / g_GridXStep;
		int y_key = (g_NodeVector[i].m_pt.y - g_GridRect.bottom) / g_GridYStep;

		//feasible region
		x_key = max(0, x_key);
		x_key = min(_MAX_TRANSIT_GRID_SIZE - 1, x_key);

		y_key = max(0, y_key);
		y_key = min(_MAX_TRANSIT_GRID_SIZE - 1, y_key);

		g_GridMatrix[x_key][y_key].m_NodeVector.push_back(g_NodeVector[i].m_NodeID);
		g_GridMatrix[x_key][y_key].m_NodeX.push_back(g_NodeVector[i].m_pt.x);
		g_GridMatrix[x_key][y_key].m_NodeY.push_back(g_NodeVector[i].m_pt.y);
	}

}

int g_FindClosestNode(double x, double y, double min_distance, int step_size)
{

	step_size = int(min_distance / g_GridXStep + 1);

	int x_key = (x - g_GridRect.left) / g_GridXStep;
	int y_key = (y - g_GridRect.bottom) / g_GridYStep;

	//feasible region
	x_key = max(0, x_key);
	x_key = min(99, x_key);

	y_key = max(0, y_key);
	y_key = min(99, y_key);

	int NodeId = -1;


	for (int x_i = max(0, x_key - step_size); x_i <= min(99, x_key + step_size); x_i++)
	for (int y_i = max(0, y_key - step_size); y_i <= min(99, y_key + step_size); y_i++)
	{

		GridNodeSet element = g_GridMatrix[x_i][y_i];

		for (unsigned int i = 0; i < element.m_NodeVector.size(); i++)
		{

			double distance = sqrt((x - element.m_NodeX[i])*(x - element.m_NodeX[i]) + (y - element.m_NodeY[i])*(y - element.m_NodeY[i])) / g_UnitMile;

			if (distance < min_distance)
			{

				min_distance = distance;

				NodeId = element.m_NodeVector[i];

			}

			//// this is for GPS map matching

			//double distance_ = distance / g_UnitMile;
			//if (distance_ < m_NodeNoMap[element.m_NodeVector[i]]->m_min_distance_from_GPS_point)
			//{
			//	m_NodeNoMap[element.m_NodeVector[i]]->m_min_distance_from_GPS_point = distance_;
			//	m_NodeNoMap[element.m_NodeVector[i]]->m_GPS_arrival_time = time_stamp_in_min;

			//	m_NodeNoMap[element.m_NodeVector[i]]->m_DistanceToRoot = distance_;

			//}


		}	// per node in a grid cell

	} // for nearby cell

	return NodeId;
}

void g_MatchODForAgents()
{

	std::map<int, DTAVehicle*>::iterator iterVM;

	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{

		DTAVehicle* pVehicle = iterVM->second;
		int StartFlag = 0; // 0: not start yet, 1: start node chain, 2: end node chain

		int origin_node = -1;
		int destination_node = -1;

		for (int i = 0; i < pVehicle->m_ShapePoints.size(); i++)
		{
			GDPoint pt = pVehicle->m_ShapePoints[i];
			int node_id = g_FindClosestNode(pt.x, pt.y, 0.1, 0);

			if (node_id >= 0) // the node exists in subarea 
			{

				if (StartFlag == 1)
				{
					destination_node = node_id;  // update destination_node
				}


				if (StartFlag == 0)
				{
					origin_node = node_id;
					StartFlag = 1; // start node chain
				}


			}
			else  // node does not exist 
			{
				if (StartFlag == 1) // node chain has been started 
				{
					StartFlag = 2;  // terminate
					break;
				}
				else
				{
					//continue // StartFlag ==0
				}
			}
		}



	}
}

void g_ReadInputLinkTravelTime()
{

	CCSVParser parser;


	//step 3:
	if (parser.OpenCSVFile("input_link_travel_time.csv"))
	{
		int count = 0;

		while (parser.ReadRecord())
		{
			int usn = 0;
			int dsn = 0;

			parser.GetValueByFieldName("from_node_id", usn);
			parser.GetValueByFieldName("to_node_id", dsn);

			if (g_LinkMap.find(GetLinkStringID(usn, dsn)) == g_LinkMap.end())
			{
				cout << "Link " << usn << "-> " << dsn << " at line " << count + 1 << " of file input_link_travel_time.csv has not been defined in input_link.csv. Please check.";
			}
			else
			{

				DTALink* pLink = g_LinkMap[GetLinkStringID(usn, dsn)];

				for (int time_index = g_DemandLoadingStartTimeInMin / 15; time_index < g_DemandLoadingEndTimeInMin / 15; time_index++)  // / 15 converts min to 15-min interval for demand patterns
				{
					std::string time_stamp_str = g_GetTimeStampStrFromIntervalNo(time_index);

					float travel_time = -1;
					parser.GetValueByFieldName(time_stamp_str, travel_time);

					if (travel_time > 0)
					{
						int start_time_in_min = time_index * 15;
						int time_step_in_min = (time_index + 1) * 15;
						pLink->UseUserDefinedAttribute(start_time_in_min, time_step_in_min, travel_time);
					}
				}
			}
			count++;

		}

		parser.CloseCSVFile();
	}
}


bool g_ReadTransitTripCSVFile()
{

	CCSVParser parser_transit_trip;

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_transit_trip.OpenCSVFile("input_transit_trip.csv", false))
	{
		int count = 0;

		int line_no = 1;

		int i = 0;
		while (parser_transit_trip.ReadRecord())
		{

			string trip_id, trip_type;

			parser_transit_trip.GetValueByFieldName("trip_id", trip_id);
			parser_transit_trip.GetValueByFieldName("trip_type", trip_type);

			int origin_node_id = -1;
			int origin_node_number = -1;


			std::vector<CCoordinate> CoordinateVector;
			string geo_string;

			int day_no;
			parser_transit_trip.GetValueByFieldName("day_no", day_no);

			int number_of_nodes = 0;

			std::vector<int> path_node_sequence;

			//--------MODIFIED BY QU 2015.10.29-----------------------------------//
			string m_vehicleID;
			int m_routeID;
			float m_capacity;
			parser_transit_trip.GetValueByFieldName("vehicle_id", m_vehicleID);
			parser_transit_trip.GetValueByFieldName("route_id", m_routeID);
			parser_transit_trip.GetValueByFieldName("vehicle_capacity", m_capacity);


			//-------transit vehicle feasible check------------------------------//
			if (m_capacity < 0)
			{
				cout << "The capacity of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") is lower than 0, please check!"<<endl;
				g_ProgramStop();
			}

			string path_node_sequence_str;
			if (parser_transit_trip.GetValueByFieldName("path_node_sequence", path_node_sequence_str) == true)
			{

				path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

				if (g_IsPathNodeSequenceAFeasiblePath(path_node_sequence))
				{
					number_of_nodes = path_node_sequence.size();
				}
				else
				{
					cout << "The path_node_sequence of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") is an infeasible path, please check!"<<endl;
					g_ProgramStop();
				}
			}
			else
			{
				cout << "The path_node_sequence of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") is not given, please check!"<<endl;
				g_ProgramStop();
			}

			//--------------------------------------------------------------------//

			DTAVehicle* pVehicle = 0;
			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			//--------MODIFIED BY QU 2015.10.29---add attributions to vehicle-----//
			pVehicle->m_vehicleID = m_vehicleID;
			pVehicle->m_routeID = m_routeID;
			pVehicle->m_capacity = m_capacity;
			//--------------------------------------------------------------------//

			pVehicle->m_AgentID = g_VehicleVector.size();
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			pVehicle->m_OriginZoneID = 1;
			pVehicle->m_DestinationZoneID = 1;

			parser_transit_trip.GetValueByFieldName("from_zone_id", pVehicle->m_OriginZoneID);
			parser_transit_trip.GetValueByFieldName("to_zone_id", pVehicle->m_DestinationZoneID);
			parser_transit_trip.GetValueByFieldName("departure_time", pVehicle->m_DepartureTime);

			if (g_DemandLoadingStartTimeInMin > pVehicle->m_DepartureTime)
				g_DemandLoadingStartTimeInMin = pVehicle->m_DepartureTime;


			//if (path_node_sequence.size() >= 2)
			//{

			//	/*if (g_NodeNametoIDMap.find(path_node_sequence[0]) != g_NodeNametoIDMap.end())
			//	{
			//		pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];
			//	}
			//	if (g_NodeNametoIDMap.find(path_node_sequence[path_node_sequence.size() - 1]) != g_NodeNametoIDMap.end())
			//	{
			//		pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[path_node_sequence.size() - 1]];

			//	}*/
			//}
			//else
			//{

				origin_node_id = -1;
				int destination_node_id = -1;

				parser_transit_trip.GetValueByFieldName("origin_node_id", origin_node_id);
				parser_transit_trip.GetValueByFieldName("destination_node_id", destination_node_id);

				pVehicle->m_OriginNodeID = g_NodeNametoIDMap[origin_node_id];
				pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[destination_node_id];


			//}

			//----------QU 10.30---transit path_node_sequence feasible checking-------//
			if (pVehicle->m_OriginNodeID != g_NodeNametoIDMap[path_node_sequence[0]] ||
				pVehicle->m_DestinationNodeID != g_NodeNametoIDMap[path_node_sequence[path_node_sequence.size() - 1]])
			{
				cout << "The first/last node in 'path_node_sequence' of transit (vehicle_id = " << m_vehicleID
					<< ", route_id =" << m_routeID << ") is not same as the origin/destination node, please check!" << endl;
				g_ProgramStop();
			}
			//------------------------------------------------------------------------//


			string schedule_node_sequence_str;
			if (parser_transit_trip.GetValueByFieldName("schedule_node_sequence", schedule_node_sequence_str) == true)
			{
				std::vector<int> schedule_node_sequence;
				schedule_node_sequence = ParseLineToIntegers(schedule_node_sequence_str);

				//--------MODIFIED BY QU 2015.10.29---feasible check for service node-----//
				if (!g_IsScheduleNodeAsServiceNode(schedule_node_sequence))
				{
					cout << "Some nodes in 'schedule_node_sequence' of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") are not service node, please check!"<<endl;
					/*getchar();
					exit(0);*/
					g_ProgramStop();
				}
				if (!g_IsServiceNodeInPathNode(path_node_sequence, schedule_node_sequence))
				{
					cout << "Some nodes in 'schedule_node_sequence' of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") are not in 'path_node_sequence', please check!" << endl;
					/*getchar();
					exit(0);*/
					g_ProgramStop();
				}
				//--------------------------------------------------------------------//

				for (int i = 0; i < schedule_node_sequence.size(); i++)
				{
					int node_id = g_NodeNametoIDMap[schedule_node_sequence[i]];
					pVehicle->m_StopTimeMap[node_id].Init();
				}
			}
			else
			{
				cout << "The schedule_node_sequence of transit (vehicle_id = " << m_vehicleID << ", route_id =" << m_routeID << ") is not given, please check!" << endl;
				g_ProgramStop();
			}


			pVehicle->m_transit_service_flag = true;
			pVehicle->m_DemandType = -1;
			pVehicle->m_VehicleType = 6; //BUS

			if (pVehicle->m_VehicleType <= g_VehicleTypeVector.size())
				pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType - 1].PCE;
			else
				pVehicle->m_PCE = 1;

			pVehicle->m_InformationType = 0;

			pVehicle->m_VOT = 10;
			pVehicle->m_Age = 5;


			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Distance = 0;

			pVehicle->m_NodeSize = number_of_nodes;

			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;


			g_VehicleVector.push_back(pVehicle);
			g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
			g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

			pVehicle->m_LinkAry = new SVehicleLink[number_of_nodes];
			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;

			for (int i = 0; i < number_of_nodes - 1; i++) // NodeSize-1 is the number of links along the paths
			{

				DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i], path_node_sequence[i + 1])];

				if (pLink != NULL)
				{
					pVehicle->m_LinkAry[i].LinkNo = pLink->m_LinkNo;


					if (pVehicle->m_LinkAry[i].LinkNo < g_LinkVector.size())
					{
						pVehicle->m_Distance += pLink->m_Length;

					}

				}

				if (i == 0)
					pVehicle->m_NodeNumberSum += path_node_sequence[i];

				pVehicle->m_NodeNumberSum += path_node_sequence[i + 1];

			}
			pVehicle->m_bSwitch = false; // no need to go through assignment stage 
			count++;

		}
		CString str_summary;
		str_summary.Format("Number of transit trips =,%d\n", count);
		g_SummaryStatFile.WriteTextLabel(str_summary);
	}


	return true;
}





//---------------------MODIFIED BY QU-------------------//
bool g_IsPathNodeSequenceAFeasiblePath(std::vector<int> path_node_sequence)
{
	bool isFeasible = true;
	if (path_node_sequence.size() == 0)
		return true;
	if (path_node_sequence.size() == 1)
		return false;

	for (int i = 0; i < path_node_sequence.size()-1; i++)
	{
		int from_node = path_node_sequence[i];
		int to_node = path_node_sequence[i + 1];

		string link_str = GetLinkStringID(from_node, to_node);;
		if (g_LinkMap.find(link_str) == g_LinkMap.end())
		{
			isFeasible = false;
			break;
		}

	}
	return isFeasible;
}
bool g_IsServiceNodeInPathNode(std::vector<int> path_node_sequence, std::vector<int> schedule_node_sequence)
{
	bool isSubSet = true;
	for (int i = 0; i < schedule_node_sequence.size(); i++)
	{
		int sche_node = schedule_node_sequence[i];

		bool isIn = false;

		for (int j = 0; j < path_node_sequence.size(); j++)
		{
			int path_node = path_node_sequence[j];
			if (path_node == sche_node)
			{
				isIn = true;
				break;
			}
		}

		if (!isIn)
		{
			isSubSet = false;
			break;
		}
	}
	return isSubSet;
}
bool g_IsScheduleNodeAsServiceNode(std::vector<int> schedule_node_sequence)
{
	bool isSubSet = true;

	for (int i = 0; i < schedule_node_sequence.size(); i++)
	{
		if (g_NodeNametoIDMap.find(schedule_node_sequence[i]) == g_NodeNametoIDMap.end())
		{
			isSubSet = false;
			break;
		}
		else
		{
			int node_index = g_NodeNametoIDMap[schedule_node_sequence[i]];
			DTANode node = g_NodeVector[node_index];
			if (node.m_transit_demand_type_code.size() == 0)
			{
				isSubSet = false;
				break;
			}
		}
	}

	return isSubSet;
}

void g_ProhibitMovement(int up_node_id, int node_id, int dest_node_id)
{
	string movement_id = GetMovementStringID(up_node_id, node_id, dest_node_id);

	if (g_NodeNametoIDMap.find(node_id) == g_NodeNametoIDMap.end())
		return;
	int node_no = g_NodeNametoIDMap[node_id];
	g_NodeVector[node_no].m_MovementMap[movement_id].in_link_from_node_id = up_node_id;
	g_NodeVector[node_no].m_MovementMap[movement_id].in_link_to_node_id = node_id;
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




//------------------------------------------------------//