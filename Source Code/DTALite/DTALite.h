//  Portions Copyright 2010 Xuesong Zhou

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

#pragma once

#pragma warning(disable:4244)  // stop warning: "conversion from 'int' to 'float', possible loss of data"
#pragma warning(disable:4996)  // stop warning: 'MBCS_Support_Deprecated_In_MFC': MBCS support in MFC is deprecated 
#include "resource.h"

//#define _large_memory_usage_lr
//#define _large_memory_usage_emission
#define _large_memory_usage

#define _high_level_memory_usage
#include <math.h>
#include <deque>
#include <map>
#include <set>
#include <iostream>
#include <vector>
#include <list>
#include "CSVParser.h"
using namespace std;
#define _MAX_NUMBER_OF_PROCESSORS  64
#define PI 3.1415626
#define _MAX_STRING_LINE 30000
#define _MAX_TRANSIT_GRID_SIZE 100


enum e_traffic_information_class { info_hist_based_on_routing_policy = 0, learning_from_hist_travel_time, info_pre_trip, info_en_route_and_pre_trip, info_personalized_info, info_eco_so, _max_info_type};
enum e_traffic_flow_model { tfm_BPR =0, tfm_point_queue, tfm_newells_model, tfm_spatial_queue, trm_car_following};
enum e_demand_loading_mode { demand_matrix_file_mode = 0, vehicle_binary_file_mode, real_time_demand_matrix_file_mode, accessibility_demand_mode};
enum e_signal_representation_model {signal_model_continuous_flow = 0,  signal_model_link_effective_green_time, signal_model_spatial_queue };

enum e_analysis_method { 
analysis_MSA =0, 
analysis_trip_generation,
analysis_trip_distribution,
analysis_OD_demand_estimation,
analysis_ABM_integration,
analysis_fixed_percentage,
analysis_day_to_day_learning_threshold_route_choice,
analysis_accessibility_distance,
analysis_accessibility_travel_time, 
analysis_vehicle_binary_file_based_scenario_evaluation,
analysis_real_time_simulation,  /*8 for ABM+DTA integration*/
analysis_integration_with_AGENT_PLUS,
analysis_system_optimal,
analysis_gap_function_MSA_step_size,
analysis_LR_agent_based_system_optimization,

};

extern e_traffic_flow_model g_TrafficFlowModelFlag;
extern e_signal_representation_model g_SignalRepresentationFlag;

// extention for multi-day equilibirum
#define MAX_FIFO_QUEUESIZE 5000
#define MAX_DAY_SIZE 1
#define MAX_PATH_LINK_SIZE 300
#define MAX_MEASUREMENT_INTERVAL 15 

#define MAX_INFO_CLASS_SIZE 7
#define MAX_VEHICLE_TYPE_SIZE 15
#define MAX_DEMAND_TYPE_SIZE  40 // because C starts from 0 
#define MAX_DEMAND_TIME_SIZE  96 // because C starts from 0 
#define MAX_TIME_INTERVAL_SIZE 300

#define MAX_SIZE_INFO_USERS 5 
#define MAX_VOT_RANGE 101
#define DEFAULT_VOT 12
#define _MAX_ODT_PATH_SIZE_4_ODME 50
#define _MAX_PATH_NODE_SIZE_4_ODME 300

extern int g_ODZoneNumberSize;
extern int g_ODZoneIDSize;
extern int g_number_of_prohibited_movements;

enum SPEED_BIN {VSP_0_25mph=0, VSP_25_50mph, VSP_GT50mph, MAX_SPEED_BIN};
enum VSP_BIN {VSP_LT0=0,VSP_0_3, VSP_3_6, VSP_6_9, VSP_9_12, VSP_12_18, VSP_18_24, VSP_24_30, VSP_GT30, MAX_VSP_BIN};
enum SENSOR_TYPE {sensor_type_time_dependent_link_count=0,sensor_type_static_link_count,sensor_type_time_dependent_movement_count,sensor_type_static_movement_count};


enum Traffic_State {FreeFlow,PartiallyCongested,FullyCongested};

enum Traffic_MOE {MOE_crashes,MOE_CO2, MOE_total_energy};

enum Tolling_Method {no_toll,time_dependent_toll,VMT_toll,SO_toll};
extern double g_DTASimulationInterval;


extern int g_CalculateUEGapForAllAgents;

extern double g_CarFollowingSimulationInterval;

#define	MAX_SPLABEL 9999.0f  // this value cannot be further increase as an extremely large value could lead to overflow
#define MAX_TIME_INTERVAL_ADCURVE 300  // 300 simulation intervals of data are stored to keep tract Cumulative flow counts of each link
extern int g_AggregationTimetInterval;
extern int g_TDSPTimetIntervalSizeForMin;

extern float g_MinimumInFlowRatio;
extern float g_RelaxInFlowConstraintAfterDemandLoadingTime;
extern float g_MaxDensityRatioForVehicleLoading;
extern float g_DefaultSaturationFlowRate_in_vehphpl;


#ifdef _WIN64
#define MAX_LINK_NO 99999999
#endif 

#ifndef _WIN64
#define MAX_LINK_NO 65530
#endif

#define MAX_NODE_SIZE_IN_A_PATH 3000
#define MAX_LINK_SIZE_IN_VMS 20

#define MAX_CPU_SIZE 20
// Linear congruential generator 
#define LCG_a 17364
#define LCG_c 0
#define LCG_M 65521  // it should be 2^32, but we use a small 16-bit number to save memory

void g_ProgramStop();
extern int g_ProgramStopFlag;
void g_ProgramTrace(CString str);
float g_RNNOF();
bool g_GetVehicleAttributes(int demand_type, int &VehicleType, int &InformationClass, float &VOT, int &Age);

string GetLinkStringID(int FromNodeName, int ToNodeName);

extern int g_AssignmentIntervalIndex[MAX_TIME_INTERVAL_SIZE];   // internal index for memory block
extern int g_AssignmentIntervalOutputFlag[MAX_TIME_INTERVAL_SIZE];   
extern int g_AssignmentIntervalStartTimeInMin[MAX_TIME_INTERVAL_SIZE];
extern int g_AssignmentIntervalEndTimeInMin[MAX_TIME_INTERVAL_SIZE];
extern int g_NumberOfSPCalculationPeriods;

extern int g_FindAssignmentIntervalIndexFromTime(float time_in_min);

extern int g_FindAssignmentIntervalLengthInMinFromTime(float time_in_min);


template <typename T>
T **AllocateDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();

	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}


template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}



template <typename T>
T ****Allocate4DDynamicArray(int nM, int nX, int nY, int nZ)
{
	T ****dynamicArray;

	dynamicArray = new (std::nothrow) T***[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		dynamicArray[m] = new (std::nothrow) T**[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T*[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				delete[] dArray[m][x][y];
			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

class DemandType
{
public:
	int demand_type;
	double demand_type_percentage;
	double vehicle_trip_multiplier_factor;

	double cumulative_demand_type_percentage;
	double vehicle_type_percentage[MAX_VEHICLE_TYPE_SIZE];
	double cumulative_type_percentage[MAX_VEHICLE_TYPE_SIZE];

	double info_class_percentage[MAX_INFO_CLASS_SIZE];
	double cumulative_info_class_percentage[MAX_INFO_CLASS_SIZE];
	double Avg_VOT;
	string demand_type_name;

	DemandType()
	{
		Avg_VOT = 12;
		vehicle_trip_multiplier_factor = 1;
		cumulative_demand_type_percentage = 0;
		demand_type_percentage = 0;
		for (int vehicle_type = 0; vehicle_type < MAX_VEHICLE_TYPE_SIZE; vehicle_type++)
		{
			vehicle_type_percentage[vehicle_type] = 0;
			cumulative_type_percentage[vehicle_type] = 0;
		}
		for (int info_class = 0; info_class < MAX_INFO_CLASS_SIZE; info_class++)
		{
			info_class_percentage[info_class] = 0;
			cumulative_info_class_percentage[info_class] = 0;
		}

	}

};

extern std::vector <DemandType> g_DemandTypeVector;
class C_RealTimeSimulationSettings
{

public:

	int synchronization_sleep_time_interval_in_second;
	int input_link_attribute_generated_from_external_program;
	int input_link_attribute_updating_time_interval_in_second;

	int input_trip_generated_from_external_program;
	int input_trip_updating_time_interval_in_min;
	
	int input_routing_policy_generated_from_external_program;
	int input_routing_policy_updating_time_interval_in_min;


	int output_trip_generated_to_external_program;
	int output_trip_updating_time_interval_in_min;

	int output_routing_policy_generated_to_external_program;
	int output_routing_policy_updating_time_interval_in_min;

	int output_link_performance_generated_to_external_program;
	int output_link_performance_updating_time_interval_in_min;

	int output_travel_cost_skim_generated_to_external_program;
	int output_travel_cost_skim_updating_time_interval_in_min;


	C_RealTimeSimulationSettings()
	{
		 synchronization_sleep_time_interval_in_second = 1;
		 input_link_attribute_generated_from_external_program = 0;
		 input_link_attribute_updating_time_interval_in_second = 60;

		 input_trip_generated_from_external_program = 0;
		 input_trip_updating_time_interval_in_min = 1;

		 input_routing_policy_generated_from_external_program = 0;
		 input_routing_policy_updating_time_interval_in_min = 1;


		 output_trip_generated_to_external_program = 0;
		 output_trip_updating_time_interval_in_min = 1;

		 output_routing_policy_generated_to_external_program = 0;
		 output_routing_policy_updating_time_interval_in_min = 1;

		 output_link_performance_generated_to_external_program = 0;
		 output_link_performance_updating_time_interval_in_min = 1;

		 output_travel_cost_skim_generated_to_external_program = 0;
		 output_travel_cost_skim_updating_time_interval_in_min = 15;

	
	}


};
///////////////////////////
// linear regression
//////////////////


struct struc_VehicleLocationRecord
{
	double within_link_distance;
	double time_stamp_in_second;
	int vehicle_id;
	int link_id;

};

struct SensorDataPoint
{

	double x;
	double y;
};

class struc_LinearRegressionResult
{
public: 
	double avg_y_to_x_ratio;
	double slope;
	double y_intercept;
	double rsqr;
	double average_residue;
	
	double avg_absolute_error;
	double avg_percentage_error;
	

	int data_size;



	struc_LinearRegressionResult()
	{
	avg_y_to_x_ratio = 0;
	slope = 0;
	y_intercept = 0;
	rsqr = 0;
	average_residue = 0;
	data_size = 0;
	
	avg_absolute_error = 0;
	avg_percentage_error = 0;

	}

};

//////////////////////

struct GDPoint
{
	double x;
	double y;
};


struct GDRect
{
	double left, right, top, bottom;

	double Height() { return top - bottom; }
	double Width()  { return right - left; }

	bool PtInRect(GDPoint& pt)
	{
		return left <= pt.x && pt.x <= right && bottom <= pt.y && pt.y <= top;
	}

	GDPoint Center(){
		GDPoint pt;
		pt.x = left + (right - left) / 2;
		pt.y = bottom + (top - bottom) / 2;
		return pt;
	}

	void Expand(GDPoint& pt)  // Inflate by a point
	{
		left = min(left, pt.x);
		top = max(top, pt.y);
		right = max(right, pt.x);
		bottom = min(bottom, pt.y);
	}

	void Expand(GDRect& rect)  // Inflate by another Rectangle
	{
		left = min(left, rect.left);
		top = max(top, rect.top);
		right = max(right, rect.right);
		bottom = min(bottom, rect.bottom);
	}

};



struct VehicleCFData
{
	int   VehicleID;
	int   LaneNo;
	int   SequentialLinkNo;
	float FreeflowDistance_per_SimulationInterval;  
	float CriticalSpacing_in_meter;

	int StartTime_in_SimulationInterval; // in time interval, LinkStartTime, so it should be sorted
	int EndTime_in_SimulationInterval; // in time interval
	int TimeLag_in_SimulationInterval;
};
class LinkMOEStatisticsData
{

public:

	int SOV_volume, HOV_volume, Truck_volume,Intermodal_volume;
	float number_of_crashes_per_year, number_of_fatal_and_injury_crashes_per_year,number_of_property_damage_only_crashes_per_year;
	char level_of_service;
	LinkMOEStatisticsData()
	{
		Init();
	}
	void Init()
	{
		level_of_service = 'A';
		SOV_volume = HOV_volume =  Truck_volume = Intermodal_volume = 0;
		number_of_crashes_per_year = number_of_fatal_and_injury_crashes_per_year = number_of_property_damage_only_crashes_per_year = 0;

	}

};

class EmissionStatisticsData
{
public: 

	float TotalEnergy;
	float TotalCO2;
	float TotalNOX;
	float TotalCO;
	float TotalHC;
	float TotalPM;
	float TotalPM2_5;

	float AvgEnergy;
	float AvgCO2;
	float AvgNOX;
	float AvgCO;
	float AvgHC;
	float AvgPM;
	float AvgPM2_5;
	float TotalMilesPerGallon;
	float AvgMilesPerGallon;

	float TotalMiles;
	float TotalGasolineGallon;


	EmissionStatisticsData()
	{
		Init();
	}
	void Init()
	{
		TotalEnergy = 0;
		TotalCO2 = 0;
		TotalNOX = 0;
		TotalCO = 0;
		TotalHC = 0;
		TotalPM = 0;
		TotalPM2_5 = 0;

		AvgEnergy = 0;
		AvgCO2  = 0;
		AvgNOX  = 0;
		AvgCO = 0;
		AvgHC = 0;
		AvgPM = 0;
		AvgPM2_5 = 0;
		TotalMilesPerGallon = 0;
		AvgMilesPerGallon = 0;

		TotalMiles = 0;
		TotalGasolineGallon = 0;
	}
};


class EmissionLaneData
{
public: 

	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;
	float PM;
	float PM2_5;
	EmissionLaneData()
	{
		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
		PM = 0;
		PM2_5 = 0;
	}

};
class LaneVehicleCFData
{

public:
	LaneVehicleCFData(int SimulationHorizonInMin)
	{
		m_LaneEmissionVector.resize(SimulationHorizonInMin);
	};
	std::vector<EmissionLaneData> m_LaneEmissionVector;
	std::vector<VehicleCFData> LaneData;
};

class VOTDistribution
{
public:
	int demand_type;
	int VOT;
	float percentage;
	float cumulative_percentage_LB;
	float cumulative_percentage_UB;

};

class DTAActivityLocation
{ public: 
DTAActivityLocation()
{
	ExternalODFlag = 0;
}

int ZoneID;
int NodeID;
int ExternalODFlag;

};



class DTAZone
{ 
public:
	int m_ZoneNumber;
	int m_ZoneSequentialNo;
	float m_ODME_ratio; 
	int m_OriginVehicleSize;  // number of vehicles from this origin, for fast acessing
	std::vector<int> m_OriginActivityVector;
	std::vector<int> m_DestinationActivityVector;

	std::vector<GDPoint> m_ShapePoints;

	float m_DemandGenerationRatio ;


	int GetRandomOriginNodeIDInZone(float random_ratio)
	{
		int ArraySize  = m_OriginActivityVector.size();

		if(ArraySize ==0)
			return -1;
		if (ArraySize == 1)
			return  m_OriginActivityVector[0];

		int node_index = int(random_ratio*ArraySize);

		if (node_index <  0)
			node_index = 0;

		if(node_index >= ArraySize)
			node_index = ArraySize -1;

		return m_OriginActivityVector[node_index];
	}

	int GetRandomDestinationIDInZone(float random_ratio)
	{
		int ArraySize  = m_DestinationActivityVector.size();
		if(ArraySize ==0)
			return -1;
		if (ArraySize == 1)
			return m_DestinationActivityVector[0];

		int node_index = int(random_ratio*ArraySize);

		if (node_index <  0)
			node_index = 0;

		if(node_index >= ArraySize)
			node_index = ArraySize -1;

			return m_DestinationActivityVector[node_index];
	}

	DTAZone()
	{
		m_ZoneNumber = -1;

		m_DemandGenerationRatio = 1.0f;
		m_ZoneSequentialNo = 0;
		m_Capacity  =0;
		m_Demand = 0;
		m_OriginVehicleSize = 0;
		m_ODME_ratio = 1.0f;

	}
	~DTAZone()
	{
		m_OriginActivityVector.clear();
		m_DestinationActivityVector.clear();
	}

	float m_Capacity;
	float m_Demand;

};

class DTADestination
{
public:
	int from_TAZ;
	int to_TAZ;
	int record_id;
	int destination_number;
	int destination_node_index;
	float destination_node_distance;
	std::vector<float> destination_node_travel_time_vector;
	std::vector<float> destination_node_travel_distance_vector;

	DTADestination()
	{
		from_TAZ = 0;
		to_TAZ = 0;
		record_id = 0;
		destination_number = 0;
		destination_node_index= 0;
		destination_node_distance = 0;

	}
};

class DTANodeMovement
{
public:
	DTANodeMovement()
	{

		starting_time_in_min = 0;
		ending_time_in_min = 1440;
		turnning_percentage = 0;
		b_turning_prohibited = false;
		signal_control_no = 0;
		signal_group_no = 0;
		phase_index = 0;
		turn_volume = 10;
		total_vehicle_count = 0;
		total_vehicle_delay = 0;
		movement_capacity_per_simulation_interval = 0;
		movement_vehicle_counter = 0 ;
		movement_hourly_capacity = 100;
		movement_effective_green_time_in_min = 0;
	}

	float GetAvgDelay_In_Min()
	{
		if(b_turning_prohibited == true)  // movement is prohibited. 
			return 99;  // this is still lower than 99999 as maximal cost

		float avg_delay = total_vehicle_delay/ max(1, total_vehicle_count );

		if(movement_hourly_capacity<=0.1)
			avg_delay = 99;

		return avg_delay;
	}


		
	int in_link_from_node_id;
	int in_link_to_node_id;  // this equals to the current node number
	int out_link_to_node_id;

	string turning_direction;

	int total_vehicle_count;
	float total_vehicle_delay;

	float movement_effective_green_time_in_min;

	float movement_hourly_capacity;
	float movement_capacity_per_simulation_interval;
	int movement_vehicle_counter;
	int starting_time_in_min;
	int ending_time_in_min;
	float turnning_percentage;
	bool b_turning_prohibited;
	int phase_index;
	int signal_control_no;  // for meso-scopic, link -based
	int signal_group_no;  // for meso-scopic, link -based
	int turn_volume;
};
// event structure in this "event-based" traffic simulation
typedef struct{
	int veh_id;
	int veh_next_node_number; // next link's downstream node number
	float event_time_stamp;

}struc_vehicle_item;

class DTAPickupQueue
{
public: 
std::list<struc_vehicle_item> PickUpQueue;  //pick up queue  of each service node / stop

};
class DTANode
{
public:
	DTANode()
	{
		m_CycleLength_In_Second = 60;
		m_SignalOffset_In_Second = 0;
		m_ControlType = 0;
		m_ZoneID = 0;
		m_ABMZoneID = 0; // for ABM output
		m_TotalCapacity = 0;

		m_bOriginFlag = false;
		m_bDestinationFlag = false;
		m_TotalDelay = 0;

	};
	~DTANode()
	{
		m_DestinationVector.clear();
	};

	std::string m_transit_stop_id;
	std::string m_transit_stop_type;
	std::string m_transit_demand_type_code;

	std::list<int> m_TransitVehicleQueue;   //service vehicle queue

	std::map<int, DTAPickupQueue> m_PickupQueuePerRoute;

	std::list<struc_vehicle_item> DropOffQueue;   //drop off queue


	GDPoint m_pt;
	int m_NodeID;
	int m_NodeNumber;
	int m_ZoneID;  // If ZoneID > 0 --> centriod,  otherwise a physical node.
	int m_ABMZoneID;
	int m_ControlType; // Type:
	int m_SignalOffset_In_Second;
	int m_CycleLength_In_Second;
	std::map<std::string, std::string> m_Movement2LinkIDStringMap;
	float m_TotalCapacity;

	std::vector<DTADestination> m_DestinationVector;
	std::vector<int> m_IncomingLinkVector;
	std::vector<int> m_OutgoingLinkVector;

	std::vector<double> m_IncomingLinkDelay;

	void AddIncomingLinkDelay(int LinkID, double Delay)
	{
		for(int i = 0; i< m_IncomingLinkVector.size(); i++)
		{
		if(m_IncomingLinkVector[i]== LinkID)
			m_IncomingLinkDelay[i]+= Delay;
		
		}
	
	}




	std::map<string, DTANodeMovement> m_MovementMap;


	bool m_bOriginFlag;
	bool m_bDestinationFlag;

	double m_TotalDelay;
	void QuickSignalOptimization();

};


class Day2DayLinkMOE
{
public:
	float m_NumberOfCrashes;
	float m_NumberOfFatalAndInjuryCrashes;
	float m_NumberOfPDOCrashes;
	float AvgSpeed;
	float AvgTravelTime;
	int TotalFlowCount;

	int CumulativeArrivalCount_DemandType[MAX_DEMAND_TYPE_SIZE];

	float CumulativeRevenue_DemandType[MAX_DEMAND_TYPE_SIZE];

	Day2DayLinkMOE()
	{
		m_NumberOfCrashes = 0;
		m_NumberOfFatalAndInjuryCrashes = 0;
		m_NumberOfPDOCrashes = 0;
		AvgSpeed = 0;
		AvgTravelTime = 0;
		TotalFlowCount = 0;

		for(int i = 1; i < MAX_DEMAND_TYPE_SIZE; i++)
		{
			CumulativeArrivalCount_DemandType[i] = 0;
			CumulativeRevenue_DemandType[i] = 0;
		}


	}

};

class SLinkMOE  // time-dependent link MOE
{
public:


#ifdef _high_level_memory_usage
	float SystemOptimalMarginalCost;

	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;
	float PM;
	float PM2_5;

#endif 
	float UserDefinedTravelTime_in_min;
	float TotalTravelTime;   // cumulative travel time for vehicles departing at this time interval
	int TotalFlowCount;

	
	//int CumulativeArrivalCount_DemandType[MAX_DEMAND_TYPE_SIZE];
	//float CumulativeRevenue_DemandType[MAX_DEMAND_TYPE_SIZE];

	//int CumulativeArrivalCount_VehicleType[MAX_VEHICLE_TYPE_SIZE];

	int CumulativeArrivalCount; 
	int CumulativeDepartureCount;
	int ExitQueueLength;
	int LeftExit_QueueLength;

	int IntervalLeftArrivalCount; 
	int IntervalLeftDepartureCount;


	int EndTimeOfPartialCongestion;  // time in min to the end of partial congestion
	int TrafficStateCode;  // 0: free-flow: 1: partial congested: 2: fully congested

	int LoadingBuffer_QueueLength;
	int LoadingBuffer_EndTimeOfCongestion;  // time in min to the end of partial congestion
	int LoadingBuffer_TrafficStateCode;  // 0: free-flow: 1: partial congested: 2: fully congested

	//   Density can be derived from CumulativeArrivalCount and CumulativeDepartureCount
	//   Flow can be derived from CumulativeDepartureCount
	//   AvgTravel time can be derived from CumulativeArrivalCount and TotalTravelTime

	SLinkMOE()
	{
#ifdef		_high_level_memory_usage
		SystemOptimalMarginalCost = 0;
		UserDefinedTravelTime_in_min = -1;
		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
		PM = 0;
		PM2_5 = 0;
#endif
		TotalTravelTime = 0;
		TotalFlowCount = 0;
		CumulativeArrivalCount  = 0;
		CumulativeDepartureCount = 0;
		ExitQueueLength = 0;
		LeftExit_QueueLength = 0;

		IntervalLeftArrivalCount  = 0;
		IntervalLeftDepartureCount = 0;

		EndTimeOfPartialCongestion = 0;
		TrafficStateCode = 0;  // free-flow

		//for(int i = 1; i < MAX_DEMAND_TYPE_SIZE; i++)
		//{
		//	CumulativeArrivalCount_DemandType[i] = 0;
		//	CumulativeRevenue_DemandType[i] = 0;
		//}

		//for (int i = 0; i < MAX_VEHICLE_TYPE_SIZE; i++)
		//{
		//	CumulativeArrivalCount_VehicleType[i] = 0;

		//}

	};


	void SetupMOE(float FreeflowTravelTime)
	{
		
#ifdef		_high_level_memory_usage
		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
#endif
		TotalTravelTime = 0;
		TotalFlowCount = 0;
		CumulativeArrivalCount  = 0;
		CumulativeDepartureCount = 0;
		ExitQueueLength = 0;
		LeftExit_QueueLength = 0;

		EndTimeOfPartialCongestion = 0;
		TrafficStateCode = 0;  // free-flow
		//for(int i = 1; i < MAX_DEMAND_TYPE_SIZE; i++)
		//{
		//	CumulativeArrivalCount_DemandType[i] = 0;
		//	CumulativeRevenue_DemandType[i] = 0;
		//}

		//for (int i = 0; i < MAX_VEHICLE_TYPE_SIZE; i++)
		//{
		//	CumulativeArrivalCount_VehicleType[i] = 0;

		//}

	}

} ;

class SLinkMeasurement  // time-dependent link measurement
{
public:


	int MovementDestinationNode;
	int StartTime;
	int EndTime;
	SENSOR_TYPE m_SensorType;
	string sensor_type;
	string name;
	string tag;
	string direction;
	// observed
	float ObsFlowCount;
	float ObsNumberOfVehicles;  // converted from density
	float ObsTravelTime;   // converted from speed
	float ObsDensity;  

	// simulated
	float SimuFlowCount;
	float SimuNumberOfVehicles;  // density
	float SimuTravelTime;   // departure time based
	float SimuDensity;  

	// Deviation
	float DeviationOfFlowCount;
	float DeviationOfNumberOfVehicles;  // density
	float DeviationOfTravelTime;   // departure time based

	SLinkMeasurement()
	{
		ObsDensity = 0;
		SimuDensity = 0;

		MovementDestinationNode = -1;
		ObsFlowCount = 0;
		ObsNumberOfVehicles = 0;
		ObsTravelTime = -1;

		SimuFlowCount = 0;
		SimuNumberOfVehicles = 0;
		SimuTravelTime =0;

		// error
		DeviationOfFlowCount = 0;
		DeviationOfNumberOfVehicles = 0;
		DeviationOfTravelTime = 0;

	}
};


class MergeIncomingLink
{
public:
	MergeIncomingLink()
	{
		m_LinkInCapacityRatio = 0;
		m_LinkInRemainingCapaityPerSimuInterval = 0;
	};
	int m_LinkNo;
	int m_link_type;
	int m_OutflowNumLanes;
	float m_LinkInCapacityRatio;
	int m_LinkInRemainingCapaityPerSimuInterval;  // derived from other incoming  demand
};


class CapacityReduction
{
public:
	int StartDayNo;
	int EndDayNo;
	float StartTime;  
	float EndTime;    
	float LaneClosureRatio;
	float SpeedLimit;
};

class EvacuationImpact
{
public:
	int StartDayNo;
	int EndDayNo;
	int StartTime;  // use integer
	int EndTime;    // use integer
	float LaneClosureRatio;
	float SpeedLimit;
};

class MessageSign
{
public:
	int StartDayNo;
	int EndDayNo;
	int VMS_no;
	float StartTime;
	float EndTime;
	float ResponsePercentage;
	int	  Type;  // Type 1: warning. Type 2: Detour
	int   BestPathFlag;

	int NumberOfSubpaths;
	std::vector<int>   detour_node_sequence[10];
	float DetourSwitchingRatio[10];

	int NodeSize;
	int DownstreamNodeID;
	void UpdateNodePredAry(int DayNo, int CurTime);

	void Initialize(int node_size, int node_pred_ary[],int link_no_ary[])
	{
	}

	MessageSign()
	{
		NumberOfSubpaths = 0;
		VMS_no = 0;

		for (int path = 0; path < 10; path++)
		{
			DetourSwitchingRatio[path] = 0;
		}

	}

	~MessageSign()
	{
	}

};


class Toll
{
public:
	int StartDayNo;
	int EndDayNo;
	float StartTime;
	float EndTime;
	float TollRate[MAX_DEMAND_TYPE_SIZE];  // 4 is 3_+1 , as pricing 

	Toll()
	{
		for(int vt = 1; vt<MAX_DEMAND_TYPE_SIZE; vt++)
			TollRate[vt]=0;
	}
};

class RadioMessage
{
public:
	int StartDayNo;
	int EndDayNo;
	float StartTime;
	float EndTime;
	float ResponsePercentage;
	float DelayPenaltyInMin;

	RadioMessage()
	{
		ResponsePercentage = 0;
		DelayPenaltyInMin = 0;
	}
};

class DTALinkOutCapacity
{
public:
	DTALinkOutCapacity(double event_time_stamp, float out_capacity, int queue_size, int i_blocking_count, int i_blocking_node_number,
	float i_out_left_capacity_in_vehicle_number,
	int i_link_left_queue_size
		)	
	{
		time_stamp_in_min = event_time_stamp;
		out_capacity_in_vehicle_number = out_capacity;
		link_queue_size = queue_size;
		blocking_count = i_blocking_count;
		blocking_node_number = i_blocking_node_number;

		out_left_capacity_in_vehicle_number = i_out_left_capacity_in_vehicle_number;
		link_left_queue_size = i_link_left_queue_size;
	}
	double time_stamp_in_min;
	float out_capacity_in_vehicle_number;
	int link_queue_size;
	int blocking_count;
	int blocking_node_number;

	float out_left_capacity_in_vehicle_number;
	int link_left_queue_size;


};
class VehicleLinkPrice
{
public:
	int LinkNo;
	float RoadPrice;    // pie variable 
	int RoadUsageFlag;  // X variable 
	float TotalTollColected;

	VehicleLinkPrice()
	{
		RoadPrice = 0;
		RoadUsageFlag = 0;
		TotalTollColected = 0;

	}

	bool operator<(const VehicleLinkPrice &other) const
	{
		return RoadPrice > other.RoadPrice;   // reverse sorting, from large to small
	}
};

class DTALink
{
public:
	DTALink(int TimeSize)  // TimeSize's unit: per min
	{
		m_External_link_outflow_capacity = -1;  // no value
		m_NetworkDesignSequenceNo = -1;
		m_NetworkDesignFlag = 0;
		m_NetworkDesignBuildCapacity = 0;
		m_NetworkDesignBuildToll = 0;

		m_ProhibitedU_Turn = 0;
		m_LeftTurn_DestNodeNumber = -1;
		m_LeftTurn_NumberOfLanes = 0; 
		m_LeftTurn_EffectiveGreenTime_In_Second = 0;
		m_LeftTurn_SaturationFlowRate_In_vhc_per_hour_per_lane = 1900;
		m_LeftTurnGreenStartTime_In_Second = 0;

		m_CumulativeOutCapacityCount = 0.0f;
		m_CumulativeOutCapacityCountAtPreviousInterval =0;
		m_CumulativeInCapacityCountAtPreviousInterval =0 ;
		
		m_CumulativeLeftOutCapacityCount = 0.0f;
		m_CumulativeMergeOutCapacityCount = 0.0f;
		m_CumulativeInCapacityCount = 0.0f;
		m_Direction;

		m_bOnRampType  = false;
		m_bOffRampType = false;
		m_EffectiveGreenTime_In_Second = 0;
		m_DownstreamCycleLength_In_Second = 120;
		m_DownstreamNodeSignalOffset_In_Second = 0;
		m_bFreewayType = false;
		m_bArterialType = false;
		m_bSignalizedArterialType = false;

		TotalEnergy  = 0;
		TotalCO2  = 0;
		TotalNOX  = 0;
		TotalCO  = 0;
		TotalHC  = 0;
		TotalPM = 0;
		TotalPM2_5 = 0;


		CurrentSequenceNoForVechileDistanceAry = 0;
		CycleSizeForVechileDistanceAry = 0;

		m_ObservedFlowVolume = 0;
		m_FlowMeasurementError = 0;
		m_AADT = 0;
		m_bSensorData = false;

		m_BPR_Distance =0;

		m_SimulationHorizon	= TimeSize;
		m_LinkMOEAry.resize(m_SimulationHorizon+1);

		m_CumuArrivalFlow.resize(MAX_TIME_INTERVAL_ADCURVE+1);         // for Cumulative flow counts: unit is per simulation time interval. e.g. 6 seconds 
		m_CumuDeparturelFlow.resize(MAX_TIME_INTERVAL_ADCURVE+1);      // TimeSize  (unit: min), TimeSize*10 = 0.1 min: number of simulation time intervals


		m_BPRLinkVolumeVector.resize(g_NumberOfSPCalculationPeriods);
		m_BPRLinkTravelTimeVector.resize(g_NumberOfSPCalculationPeriods);
		m_BPRLinkTravelCostVector.resize(g_NumberOfSPCalculationPeriods);

		m_StochaticCapcityFlag = 0;

		for (int p = 0; p < g_NumberOfSPCalculationPeriods; p++)
		{
			m_BPRLinkVolumeVector[p] = 0;
			m_BPRLinkTravelTimeVector[p] = 0;
			m_BPRLinkTravelCostVector[p] = 0;
		}

		m_bMergeFlag = 0;
		m_MergeOnrampLinkID = -1;
		m_MergeMainlineLinkID = -1;
		m_TollSize = 0;
		pTollVector = NULL;

		m_LoadingBufferWaitingTime = 0;

		m_BPR_Alpha = 0.15f;
		m_BPR_Beta = 4.0f;

		m_GreenStartTime_In_Second = 0;
		m_SaturationFlowRate_In_vhc_per_hour_per_lane = 1800;



	};
	int m_FromNodeNumber;
	int m_ToNodeNumber;

	VehicleLinkPrice m_LROptimizationLinkPrice; 

	void	ComputeVSP_FastMethod();

	void ResizeData(int TimeSize)  // TimeSize's unit: per min
	{
		m_SimulationHorizon	= TimeSize;
		m_LinkMOEAry.resize(m_SimulationHorizon+1);
		//		m_LinkMeasurementAry.clear();  do not remove measurement data for ODME 

	}

	bool 	GetImpactedFlag(int DayNo=0, int DepartureTime = -1)
	{
		for(unsigned int il = 0; il< CapacityReductionVector.size(); il++)
		{
			if((CapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= CapacityReductionVector[il].EndDayNo ) && (DepartureTime >= CapacityReductionVector[il].StartTime && DepartureTime<=CapacityReductionVector[il].EndTime + 60))  // 60 impacted after capacity reduction
			{
				return true;
			}
		}

		for(unsigned int il = 0; il< IncidentCapacityReductionVector.size(); il++)
		{
			if((IncidentCapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= IncidentCapacityReductionVector[il].EndDayNo ) && (DepartureTime >= IncidentCapacityReductionVector[il].StartTime && DepartureTime<=IncidentCapacityReductionVector[il].EndTime + 60))  // 60 impacted after capacity reduction
			{
				return true;
			}
		}

		return false;
	}

	bool 	IsInsideEvacuationZoneFlag(int DayNo=0, int CurrentTime = -1)
	{
		for(unsigned int il = 0; il< EvacuationImpactVector.size(); il++)
		{
			if((EvacuationImpactVector[il].StartDayNo  <=DayNo 
				&& DayNo <= EvacuationImpactVector[il].EndDayNo ) 
				&& (CurrentTime >= EvacuationImpactVector[il].StartTime 
				&& CurrentTime<=EvacuationImpactVector[il].EndTime)) 
			{
				return true;
			}
		}

		return false;
	}

	float GetHourlyPerLaneCapacity(int Time=-1)
	{
		return m_LaneCapacity;
	}

	float GetInflowNumberOfLanes(int DayNo = 0, int Time = 0, bool OutputFlowFlag = false, bool bConsiderIncident = true)  // with lane closure
	{

		int NumLanes = m_InflowNumLanes;

		for (unsigned int il = 0; il< CapacityReductionVector.size(); il++)
		{
			if ((CapacityReductionVector[il].StartDayNo <= DayNo && DayNo <= CapacityReductionVector[il].EndDayNo) && (Time >= CapacityReductionVector[il].StartTime && Time <= CapacityReductionVector[il].EndTime))
			{
				return (1 - CapacityReductionVector[il].LaneClosureRatio)*NumLanes;
			}
		}


		if (bConsiderIncident == true)
		{

			for (unsigned int il = 0; il< IncidentCapacityReductionVector.size(); il++)
			{
				if ((IncidentCapacityReductionVector[il].StartDayNo <= DayNo && DayNo <= IncidentCapacityReductionVector[il].EndDayNo) && (Time >= IncidentCapacityReductionVector[il].StartTime && Time <= IncidentCapacityReductionVector[il].EndTime))
				{
					return (1 - IncidentCapacityReductionVector[il].LaneClosureRatio)*NumLanes;
				}
			}

		}

		return (float)NumLanes;

	}
	float GetNumberOfLanes(int DayNo=0, double Time=0, bool OutputFlowFlag = false, bool bConsiderIncident = true)  // with lane closure
	{

		int NumLanes = m_OutflowNumLanes;

		for(unsigned int il = 0; il< CapacityReductionVector.size(); il++)
		{
			if( (CapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= CapacityReductionVector[il].EndDayNo ) && (Time>= CapacityReductionVector[il].StartTime && Time<=CapacityReductionVector[il].EndTime))
			{
				return (1-CapacityReductionVector[il].LaneClosureRatio)*NumLanes;
			}
		}


		if(bConsiderIncident == true)
		{
	
			for(unsigned int il = 0; il< IncidentCapacityReductionVector.size(); il++)
			{
				if( (IncidentCapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= IncidentCapacityReductionVector[il].EndDayNo ) && (Time>= IncidentCapacityReductionVector[il].StartTime && Time<=IncidentCapacityReductionVector[il].EndTime))
				{
					return (1-IncidentCapacityReductionVector[il].LaneClosureRatio)*NumLanes;
				}
			}

		}

		return (float)NumLanes;

	}

	float GetNumberOfLanes_ImpactedByWorkZoneConditions(int DayNo=0, double Time=-1, bool OutputFlowFlag = false)  // with lane closure
	{

		int NumLanes = m_OutflowNumLanes;

		for(unsigned int il = 0; il< CapacityReductionVector.size(); il++)
		{
			if( (CapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= CapacityReductionVector[il].EndDayNo ) && (Time>= CapacityReductionVector[il].StartTime && Time<=CapacityReductionVector[il].EndTime))
			{
				return (1-CapacityReductionVector[il].LaneClosureRatio)*NumLanes;
			}
		}

		return (float)NumLanes;

	}

	int GetInformationResponseID(int DayNo=0, float Time=-1)  // from information signs
	{
		for(unsigned int il = 0; il< MessageSignVector.size(); il++)
		{
			if((MessageSignVector[il].StartDayNo  <=DayNo && DayNo <= MessageSignVector[il].EndDayNo )
				&&(Time>=MessageSignVector[il].StartTime && Time<=MessageSignVector[il].EndTime))
			{
				return il;
			}
		}

		return -1;

	}

	float GetTollRateInDollar(int DayNo=0, float Time=0, int DemandType=1)  
	{
		for(int il = 0; il< m_TollSize; il++)
		{
			if(( pTollVector[il].StartDayNo  <= DayNo &&  DayNo <= pTollVector[il].EndDayNo) && (Time >= pTollVector[il].StartTime && Time<=pTollVector[il].EndTime))
			{
				return pTollVector[il].TollRate[DemandType];
			}
		}
		return 0;
	}

	unsigned int m_RandomSeed; 
	float GetRandomRatio()  // get link_specific random seed
	{
		m_RandomSeed = (LCG_a * m_RandomSeed + LCG_c) % LCG_M;  //m_RandomSeed is automatically updated.

		return float(m_RandomSeed)/LCG_M;
	}


		
	std::vector<struc_VehicleLocationRecord> m_VehicleLocationVector;  // used for microscopic traffic simulation
	void AddVehicleLocation(int vehicle_id,  int link_id, double time_stamp_in_second, double within_link_distance)
	{
		struc_VehicleLocationRecord element;
		element.link_id = link_id;
		element.vehicle_id = vehicle_id;
		element.time_stamp_in_second = time_stamp_in_second;
		element.within_link_distance = within_link_distance;
		m_VehicleLocationVector.push_back(element);

	}

	GDPoint GetRelativePosition(float ratio)
	{

		GDPoint Pt;
		Pt.x = 0;
		Pt.y = 0;

		if (m_ShapePoints.size() == 0)  // no shape point data
			return Pt;

		Pt.x= (m_ShapePoints[0].x+ m_ShapePoints[m_ShapePoints .size()-1].x)/2;
		Pt.y= (m_ShapePoints[0].y+ m_ShapePoints[m_ShapePoints .size()-1].y)/2;

		unsigned	int si;

		if(m_ShapePointRatios.size() == m_ShapePoints.size())
		{

			for(si = 0; si < m_ShapePoints .size()-1; si++)
			{

				if(ratio > m_ShapePointRatios[si] && ratio < m_ShapePointRatios[si+1])
				{

					float SectionRatio = m_ShapePointRatios[si+1] - m_ShapePointRatios[si];

					float RelateveRatio = 0;
					if(SectionRatio >0)
						RelateveRatio = (ratio - m_ShapePointRatios[si])/SectionRatio;

					Pt.x = m_ShapePoints[si].x + RelateveRatio*(m_ShapePoints[si+1].x - m_ShapePoints[si].x);
					Pt.y = m_ShapePoints[si].y + RelateveRatio*(m_ShapePoints[si+1].y - m_ShapePoints[si].y);

					return Pt;
				}
			}

		}
		return Pt;
	}

// end of microsimulation 

	float m_CumulativeOutCapacityCount; 
	int m_CumulativeOutCapacityCountAtPreviousInterval; 
	int m_CumulativeInCapacityCountAtPreviousInterval; 

	float m_CumulativeLeftOutCapacityCount; 
	float m_CumulativeMergeOutCapacityCount; 

	int m_ProhibitedU_Turn;

	float m_CumulativeInCapacityCount;



	float	m_Length;  // in miles
	float   m_VehicleSpaceCapacity; // in vehicles
	int	m_OutflowNumLanes;

	float m_External_link_outflow_capacity;

	int m_Orginal_OutflowNumLanes;
	int m_Orginal_InflowNumLane;

	int m_InflowNumLanes;
	float	m_SpeedLimit;
	float	m_SpeedAtCapacity;
	float	m_KCritical;

	float m_KJam;
	float m_AADTConversionFactor;
	float m_BackwardWaveSpeed; // unit: mile/ hour
	float	m_LaneCapacity;  //Capacity used in BPR for each link, reduced due to link type and other factors.
	float m_LoadingBufferWaitingTime;


	char m_Direction;

	std::string m_geometry_string, m_original_geometry_string;
	bool m_bOnRampType;
	bool m_bOffRampType;
	string m_LinkTypeName;

	std::map<int, int> m_OperatingModeCount;
	std::vector<GDPoint> m_ShapePoints;
	std::vector<float> m_ShapePointRatios;

double GetPoint2Point_Distance(GDPoint p1, GDPoint p2)
{
return pow(((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)),0.5);
}
	void CalculateShapePointRatios()
	{

		m_ShapePointRatios.clear();

		float total_distance = 0; 
		unsigned int si;

		if(m_ShapePoints.size()==0)
			return;

		for(si = 0; si < m_ShapePoints .size()-1; si++)
		{
			total_distance += GetPoint2Point_Distance(m_ShapePoints[si],m_ShapePoints[si+1]); 
		}

		if(total_distance < 0.0000001f)
			total_distance = 0.0000001f;

		float distance_ratio = 0;
		float P2Origin_distance = 0;
		m_ShapePointRatios.push_back(0.0f);
		for(si = 0; si < m_ShapePoints .size()-1; si++)
		{
			P2Origin_distance += GetPoint2Point_Distance(m_ShapePoints[si],m_ShapePoints[si+1]);
			m_ShapePointRatios.push_back(P2Origin_distance/total_distance);
		}
	}

	std::vector <SLinkMOE> m_LinkMOEAry;

	std::vector<DTALinkOutCapacity> m_OutCapacityVector;

	float TotalEnergy;
	float TotalCO2;
	float TotalNOX;
	float TotalCO;
	float TotalHC;
	float TotalPM;
	float TotalPM2_5;

	std::vector<LaneVehicleCFData> m_VehicleDataVector;   

	void ThreeDetectorVehicleTrajectorySimulation();

	void ComputeVSP();
	//	void ComputeVSP_FastMethod();
	std::vector <SLinkMeasurement> m_LinkMeasurementAry;

	float SimultedHourlySpeed[25];


	bool ContainObsDensity(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsDensity >=1 )
			{

				return true;
			}

		}

		return false;
	}

	float GetDeviationOfNumberOfVehicles(float timestamp)
	{
	
			for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsDensity >=1 )
			{

				return m_LinkMeasurementAry[i].DeviationOfNumberOfVehicles ;
			}

		}

			return 0;
	}

	float GetDeviationOfNumberOfVehicles(int start_time, int end_time)
	{
	
		float total_deviation = 0;
		for(int t = start_time; t <= end_time; t++)
		{
				total_deviation+= GetDeviationOfNumberOfVehicles(t);
		}
			return total_deviation;
	}

	bool ContainTravelTimeObservation(float timestamp)
	{
		for (unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if (m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsTravelTime  >= 0.01)
			{

				return true;
			}

		}

		return false;
	}
	bool ContainFlowCount(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 )
			{

				return true;
			}

		}

		return false;
	}

	bool ContainMovementFlowCount(float timestamp,  int DestinationNode)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 &&
				m_LinkMeasurementAry[i].MovementDestinationNode == DestinationNode )
			{

				return true;
			}

		}

		return false;
	}
	int GetDeviationOfFlowCount(float timestamp, int DestinationNode = -1)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(DestinationNode == -1)
			{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 )
			{

				return m_LinkMeasurementAry[i].DeviationOfFlowCount ;

			}
			
			}else
			{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 
				&&m_LinkMeasurementAry[i].MovementDestinationNode == DestinationNode )
			{

				return m_LinkMeasurementAry[i].DeviationOfFlowCount ;

			}
			
			
		}

	}
		return 0;

	}

	float GetDeviationOfTravelTime(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime 
				&&  m_LinkMeasurementAry[i].ObsTravelTime>=0.001 )
			{

				return m_LinkMeasurementAry[i].DeviationOfTravelTime  ;

			}

		}
		return 0;

	}




	int GetObsFlowCount(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 )
			{

				return m_LinkMeasurementAry[i].ObsFlowCount; 
			}

		}
		return 0;

	}

	int GetSimulateAvgDensityPerLane(int start_timestamp, int end_timestamp)
	{

		if(start_timestamp < end_timestamp && start_timestamp >= 0 && start_timestamp < m_LinkMOEAry.size() &&  end_timestamp < m_LinkMOEAry.size()  )
			{
			int timestamp = (start_timestamp + end_timestamp) / 2;
			int TotalSimulatedNumberOfVehicles = m_LinkMOEAry[timestamp].CumulativeArrivalCount - m_LinkMOEAry[timestamp].CumulativeDepartureCount;
			// we can do a more complicated summation here but we perfer to use a simpler calculation for a single time stamp
			return TotalSimulatedNumberOfVehicles / max(1, this->m_OutflowNumLanes) / max(0.01, m_Length);

			}
		return 0;

	}

	int GetSimulatedNumberOfVehicles(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsDensity  >=1 )
			{

				int SimulatedNumberOfVehicles = m_LinkMOEAry[m_LinkMeasurementAry[i].StartTime].CumulativeArrivalCount  - m_LinkMOEAry[m_LinkMeasurementAry[i].StartTime ].CumulativeDepartureCount; 

				return SimulatedNumberOfVehicles; 
			}

		}
		return 0;

	}
	int GetSimulatedFlowCountWithObsCount(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime && m_LinkMeasurementAry[i].ObsFlowCount >=1 )
			{

				int SimulatedFlowCount = m_LinkMOEAry[m_LinkMeasurementAry[i].EndTime].CumulativeArrivalCount - m_LinkMOEAry[m_LinkMeasurementAry[i].StartTime ].CumulativeArrivalCount; 

				return SimulatedFlowCount; 
			}

		}
		return 0;

	}

	bool UpdateSpeedMeasurement(float speed, int timestamp)
	{

		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime )
			{


				m_LinkMeasurementAry[i].ObsTravelTime = this->m_Length / max(1,speed)*60 ;

				return 1;
			}

		}
		return 0;

	}

	float GetSimulatedTravelTime(float timestamp)
	{
		for(unsigned i = 0; i< m_LinkMeasurementAry.size(); i++)
		{
			if(m_LinkMeasurementAry[i].StartTime <= timestamp && timestamp <= m_LinkMeasurementAry[i].EndTime )
			{

				return GetTravelTimeByMin(-1,timestamp,g_AggregationTimetInterval,g_TrafficFlowModelFlag); 
			}

		}
		return m_FreeFlowTravelTime;

	}
	std::vector<Day2DayLinkMOE> m_Day2DayLinkMOEVector;
	std::vector <int> m_CumuArrivalFlow;
	std::vector <int> m_CumuDeparturelFlow;

	std::vector<CapacityReduction> CapacityReductionVector;
	std::vector<CapacityReduction> IncidentCapacityReductionVector;

	std::vector<CapacityReduction> EvacuationImpactVector;


	std::vector<MessageSign> MessageSignVector;
	std::vector<Toll> TollVector;




	float m_NetworkDesignBuildCapacity;
	float m_NetworkDesignBuildToll;
	int   m_NetworkDesignFlag;
	int   m_NetworkDesignSequenceNo;

	int  m_DynamicTollType;  // 0: static toll as default: 1: dynamic toll: 


	int m_TollSize;
	Toll *pTollVector;  // not using SLT here to avoid issues with OpenMP

	std::vector<RadioMessage> m_RadioMessageVector;

	int m_bMergeFlag;  // 1: freeway and freeway merge, 2: freeway and ramp merge
	std::vector<MergeIncomingLink> MergeIncomingLinkVector;
	int m_MergeOnrampLinkID;
	int m_MergeMainlineLinkID;

	std::list<struc_vehicle_item> LoadingBuffer;  //loading buffer of each link, to prevent grid lock

	std::list<struc_vehicle_item> EntranceQueue;  //link-in queue  of each link
	std::list<struc_vehicle_item> ExitQueue;      // link-out queue of each link

	void WriteDataFromBufferToDisk(FILE * pFile)
	{
		fprintf(pFile, "%d,%d,%d,%d,%d,", this->m_FromNodeNumber, this->m_ToNodeNumber, LoadingBuffer.size(), EntranceQueue.size(), ExitQueue.size());
		for (std::list<struc_vehicle_item>::iterator i = LoadingBuffer.begin(); i != LoadingBuffer.end(); ++i)
		{
			struc_vehicle_item vi = (*i);

			fprintf(pFile, "%d;%d;%.1f;",
				vi.veh_id,
				vi.veh_next_node_number,
				vi.event_time_stamp);
		}

		fprintf(pFile, ",");

		for (std::list<struc_vehicle_item>::iterator i = EntranceQueue.begin(); i != EntranceQueue.end(); ++i)
		{
			struc_vehicle_item vi = (*i);

			fprintf(pFile, "%d;%d;%.1f;",
				vi.veh_id,
				vi.veh_next_node_number,
				vi.event_time_stamp);
		}

		fprintf(pFile, ",");

		for (std::list<struc_vehicle_item>::iterator i = ExitQueue.begin(); i != ExitQueue.end(); ++i)
		{
			struc_vehicle_item vi = (*i);

			fprintf(pFile, "%d;%d;%.1f;",
				vi.veh_id,
				vi.veh_next_node_number,
				vi.event_time_stamp);
		}

		fprintf(pFile, ",");

		fprintf(pFile, "\n");
	}
	// for left turn queues
	std::list<struc_vehicle_item> LeftEntrance_Queue;  // left-turn in queue  of each link
	std::list<struc_vehicle_item> LeftExit_Queue;      // left-turn out  queue of each link
//

	int CurrentSequenceNoForVechileDistanceAry;  // start from 0, 
	int CycleSizeForVechileDistanceAry; // cycle size


	int m_LinkNo;
	int m_OrgLinkID;    //original link id from input_link.csv file
	int m_FromNodeID;  // index starting from 0
	int m_ToNodeID;    // index starting from 0
	int m_link_code; //1: AB, 2: BA

	int  m_StochaticCapcityFlag;  // 0: deterministic cacpty, 1: lane drop. 2: merge, 3: weaving
	// optional for display only
	int	m_link_type;

	string demand_type_code;

	bool m_bFreewayType;  //created to store the freeway type, used in simulation
	bool m_bArterialType;
	bool m_bSignalizedArterialType;
	int m_DownstreamCycleLength_In_Second;
	int m_DownstreamNodeSignalOffset_In_Second;

	int m_GreenStartTime_In_Second;

	float m_EffectiveGreenTime_In_Second;
	float m_SaturationFlowRate_In_vhc_per_hour_per_lane;


	int m_LeftTurn_DestNodeNumber; 
	int m_LeftTurn_NumberOfLanes; 
	int m_LeftTurn_EffectiveGreenTime_In_Second;
	int m_LeftTurn_SaturationFlowRate_In_vhc_per_hour_per_lane; 
	int m_LeftTurnGreenStartTime_In_Second;


	// for MOE data array
	int m_SimulationHorizon;


	// traffic flow propagation
	float m_FreeFlowTravelTime; // min
	float m_MinimMovingTravelTime; // min
	float m_BPR_Alpha;
	float m_BPR_Beta;
	float m_BPR_Distance;
	float m_BPR_MaxTravelTime;

	int m_BackwardWaveTimeInSimulationInterval; // simulation time interval

	vector<int> m_BPRLinkVolumeVector;
	
	vector<float> m_BPRLinkTravelTimeVector;
	vector<float> m_BPRLinkTravelCostVector;


	//  multi-day equilibirum: travel time for stochastic capacity
	float m_BPRLaneCapacity;
	float m_DayDependentCapacity[MAX_DAY_SIZE];



	~DTALink()
	{

		if(LoadingBufferVector!=NULL && LoadingBufferSize>=1)
			delete LoadingBufferVector;

		if(pTollVector)
			delete pTollVector;

		LoadingBuffer.clear();
		EntranceQueue.clear();
		ExitQueue.clear();

		LeftEntrance_Queue.clear();
		LeftExit_Queue.clear();

		MergeIncomingLinkVector.clear();

		m_ShapePoints.clear();
		m_LinkMOEAry.clear();
		m_LinkMeasurementAry.clear();
		m_VehicleDataVector.clear();
		m_OutCapacityVector.clear();

		m_CumuArrivalFlow.clear();
		m_CumuDeparturelFlow.clear();

		CapacityReductionVector.clear();
		IncidentCapacityReductionVector.clear();
		MessageSignVector.clear();
		TollVector.clear();


		m_Day2DayLinkMOEVector.clear();
	};

	float GetFreeMovingTravelTime(int TrafficModelFlag = 2, int DayNo=0, float Time = -1, int analysis_period_no = 0)
	{
		if(TrafficModelFlag == 0) // BRP model
			return m_BPRLinkTravelTimeVector[analysis_period_no];  // we use time, not the cost here
		else 
		{
			for(unsigned int il = 0; il< CapacityReductionVector.size(); il++)
			{
				if((CapacityReductionVector[il].StartDayNo  <=DayNo && DayNo <= CapacityReductionVector[il].EndDayNo ) &&(Time>=CapacityReductionVector[il].StartTime && Time<=CapacityReductionVector[il].EndTime))
				{
					return m_Length/max(1,CapacityReductionVector[il].SpeedLimit)*60.0f;  // convert from hour to min;
				}
			}
			for(unsigned int il = 0; il< IncidentCapacityReductionVector.size(); il++)
			{
				if(IncidentCapacityReductionVector[il].StartDayNo   <=DayNo && Time>=IncidentCapacityReductionVector[il].StartTime && Time<=IncidentCapacityReductionVector[il].EndTime)
				{
					return m_Length/max(1,IncidentCapacityReductionVector[il].SpeedLimit)*60.0f;  // convert from hour to min;
				}
			}


				return m_FreeFlowTravelTime;

		}
	}

	float GetMinimumMovingTravelTime()
	{
		return m_MinimMovingTravelTime;
	}
	void UpdateMinimumMovingTravelTime(float gain_factor = 0.5)
	{
			float density = (CFlowArrivalCount - CFlowDepartureCount) / max(0.001, m_Length*m_OutflowNumLanes); 
			float current_speed_above_capacity = max(m_SpeedAtCapacity, m_SpeedLimit - density* (m_SpeedLimit - m_SpeedAtCapacity) / m_KCritical);
			float current_moving_travel_time = m_Length / max(0.1, current_speed_above_capacity)*60;  // unit: min

			m_MinimMovingTravelTime = (1 - gain_factor) * m_MinimMovingTravelTime + gain_factor* current_moving_travel_time;

	}

	//if (TrafficModelFlag == 4)  // modified Newell's model
	//{
	//	float density = (CFlowArrivalCount - CFlowDepartureCount) / max(0.001, m_Length*m_OutflowNumLanes); 
	//	float current_speed_above_capacity = max(m_SpeedAtCapacity, m_SpeedLimit - density* (m_SpeedLimit - m_SpeedAtCapacity) / m_KCritical);
	//	return m_Length / max(0.1, current_speed_above_capacity)*60;  // unit: min
	//}
	//else
	//{

	float GetHistoricalTravelTime(int Time = -1)
	{	// default value for now
		return m_BPRLinkTravelTimeVector[0];
	}

	void SetupMOE()
	{
		departure_count = 0;
		total_departure_based_travel_time = 0;
		m_CumulativeOutCapacityCount = 0;
		m_CumulativeOutCapacityCountAtPreviousInterval = 0;
		m_CumulativeInCapacityCountAtPreviousInterval =0 ;
		m_CumulativeLeftOutCapacityCount = 0;
		m_CumulativeMergeOutCapacityCount = 0;
		m_CumulativeInCapacityCount = 0;
		m_JamTimeStamp = (float) m_SimulationHorizon;
		m_FreeFlowTravelTime = m_Length/m_SpeedLimit*60.0f;  // convert from hour to min
		m_MinimMovingTravelTime = m_FreeFlowTravelTime;
		total_departure_based_travel_time = m_FreeFlowTravelTime;
		m_prevailing_travel_time = m_FreeFlowTravelTime;

		for (int p = 0; p < m_BPRLinkVolumeVector.size(); p++)
		{
			m_BPRLinkVolumeVector[p] = 0;
			m_BPRLinkTravelTimeVector[p] = m_FreeFlowTravelTime;
			m_BPRLinkTravelCostVector[p] = m_FreeFlowTravelTime;
		}
		m_FFTT_simulation_interval = int(m_FreeFlowTravelTime/g_DTASimulationInterval);
		LoadingBufferVector = NULL;

		MicroSimulationLastVehiclePassingTimeStamp = 0;

		LoadingBufferSize = 0;

		m_BackwardWaveTimeInSimulationInterval = int(m_Length/m_BackwardWaveSpeed*60/g_DTASimulationInterval); // assume backwave speed is 20 mph, 600 conversts hour to simulation intervals

		CFlowArrivalCount = 0;
		CFlowImpactedCount = 0;

		for(int pt = 1; pt < MAX_DEMAND_TYPE_SIZE; pt++)
		{
			CFlowArrivalCount_DemandType[pt] = 0;
			CFlowArrivalRevenue_DemandType[pt] = 0;
		}

		for (int i = 0; i < MAX_VEHICLE_TYPE_SIZE; i++)
		{
			CFlowArrivalCount_VehicleType[i] = 0;
		}

		for(int t=0; t<MAX_TIME_INTERVAL_ADCURVE; t++)
		{
			A[t] = 0;
			D[t] = 0;
		}

		StartIndexOfLoadingBuffer = 0;
		LoadingBufferVector = NULL;

		CFlowDepartureCount = 0;

		LinkOutCapacity = 0;
		LinkLeftOutCapacity = 0;
		LinkInCapacity = 0;

		VehicleCount = 0;

		int t;


		for(t=0; t< m_CumuArrivalFlow.size(); t++)
		{
			m_CumuArrivalFlow[t] = 0;
			m_CumuDeparturelFlow[t] = 0;

		}

		for(t=0; t<=m_SimulationHorizon; t++)
		{
			m_LinkMOEAry[t].SetupMOE(m_FreeFlowTravelTime);
		}
		for(int hour = 0; hour <= 24; hour++)  // used for ODME
			SimultedHourlySpeed[hour] = m_SpeedLimit;

		LoadingBuffer.clear();
		EntranceQueue.clear();
		ExitQueue.clear();

		LeftEntrance_Queue.clear();
		LeftExit_Queue.clear();

		TotalEnergy = 0;
		TotalCO2 = 0;
		TotalNOX = 0;
		TotalCO = 0;
		TotalHC = 0;
		TotalPM = 0;
		TotalPM2_5 = 0;
	}

	void ResetUserDefinedTravelTime()
	{
			for(int t=0; t<m_LinkMOEAry.size(); t++)
		{
			m_LinkMOEAry[t] .UserDefinedTravelTime_in_min = -1;
		}
	}
	void UseUserDefinedAttribute(int start_time_in_min, int time_step_in_min, float travel_time = -1, float toll_value_in_dollar = -1, float lane_capacity = -1) 
	{
		for(int t = max(0,start_time_in_min); t<= min(start_time_in_min + time_step_in_min, m_SimulationHorizon); t++)
		{
			if(travel_time >=0.1)  // minimum simulation time  = 0.1 min
			{
			m_LinkMOEAry[t].UserDefinedTravelTime_in_min  = travel_time;
			}
		}
	
	}
	void InitializeDayDependentCapacity()
	{
		for(int day = 0; day < MAX_DAY_SIZE; day ++)
		{
			float TotalCapacity =0;
			int number_of_days = 1;
			for(int t = 0; t< number_of_days; t++)
			{
				float Normal = g_RNNOF(); // call random number anyway, used in with queue and without queue cases
				float Shift  = 1.5f;
				float XM    = -0.97f;
				float S     = 0.68f;

				float Headway = exp(Normal*S+XM)+Shift;

				if(Headway < 1.5f)
					Headway = 1.5f;

				TotalCapacity += 3600.0f / Headway;

			}

			float hourly_capacity = TotalCapacity/number_of_days;
			TRACE("Hourly Capacity %f\n", hourly_capacity);

			m_DayDependentCapacity[day] = hourly_capacity/1870*m_LaneCapacity;
			//1820.31 is the mean capacity derived from mean headway

		}
	}


	float m_DayDependentTravelTime[MAX_DAY_SIZE];
	float m_AverageTravelTime;

	float m_JamTimeStamp;


	int CFlowArrivalCount_DemandType[MAX_DEMAND_TYPE_SIZE];
	int CFlowArrivalCount_VehicleType[MAX_VEHICLE_TYPE_SIZE];
	float CFlowArrivalRevenue_DemandType[MAX_DEMAND_TYPE_SIZE];

	int CFlowArrivalCount;
	int CFlowDepartureCount;

	int CFlowImpactedCount;

	int  A[MAX_TIME_INTERVAL_ADCURVE];
	int  D[MAX_TIME_INTERVAL_ADCURVE];
	int  m_FFTT_simulation_interval;  // integral value of  FFTT, in terms of simulation  time interval

	int* LoadingBufferVector;
	int  LoadingBufferSize;


	int  StartIndexOfLoadingBuffer;

	std::string m_Name;

	float m_ObservedFlowVolume;
	float m_FlowMeasurementError ;
	float m_AADT;
	bool m_bSensorData;

	int LinkOutCapacity;  // unit: number of vehiles
	int LinkLeftOutCapacity;  // unit: number of vehiles
	int LinkInCapacity;   // unit: number of vehiles

	double MicroSimulationLastVehiclePassingTimeStamp;
	int VehicleCount;

	int departure_count;
	float total_departure_based_travel_time;
	float m_prevailing_travel_time;

	float GetSpeed(int time, int aggregation_time_interval=1)
	{
		return m_Length / max(0.1, GetTravelTimeByMin(-1, time, aggregation_time_interval, g_TrafficFlowModelFlag))*60.0f;  // 60.0f converts min to hour, unit of speed: mph
	}

	int GetArrivalFlow(int time, int aggregation_time_interval=1)
	{

		time = max(0, time);
		if (time + aggregation_time_interval< m_SimulationHorizon - 1)  // if time = m_SimulationHorizon-1, time+1 = m_SimulationHorizon  --> no data available
			return max(0,m_LinkMOEAry[time + aggregation_time_interval].CumulativeArrivalCount - m_LinkMOEAry[time].CumulativeArrivalCount);
		else
			return 0;

	};

	int GetDepartureFlow(int time, int aggregation_time_interval=1)
	{
		time = max(0, time);
		if (time + aggregation_time_interval < m_SimulationHorizon - 1)  // if time = m_SimulationHorizon-1, time+1 = m_SimulationHorizon  --> no data available
			return max(0,m_LinkMOEAry[time + aggregation_time_interval].CumulativeDepartureCount - m_LinkMOEAry[time].CumulativeDepartureCount);
		else
			return 0;

	};


	float GetRadioMessageResponsePercentage(int DayNo,int CurrentTime)
	{
		for(int i = 0; i < m_RadioMessageVector.size(); i++)
		{
			if(DayNo >=  m_RadioMessageVector[i].StartDayNo && DayNo <=  m_RadioMessageVector[i].EndDayNo && 
			   CurrentTime >= m_RadioMessageVector[i].StartTime && CurrentTime <= m_RadioMessageVector[i].EndTime )
			{
				return m_RadioMessageVector[i].ResponsePercentage;
			}
		}
	
		return -1; // default non-response percentage: // not using 0, as it is difficult to check if a float point value is >0.000

	}

	float GetPrevailingTravelTime(int DayNo,int CurrentTime)
	{  // used by real time information users
		bool bConsiderIncident = false;

		if(GetNumberOfLanes(DayNo,CurrentTime,false,bConsiderIncident)<=0.001)   // road blockage by work zone, less than 0.1 lanes, or about 2000 capacity
			return 1440; // unit min

		for(int i = 0; i < m_RadioMessageVector.size(); i++)
		{
			if(DayNo >=  m_RadioMessageVector[i].StartDayNo && DayNo <=  m_RadioMessageVector[i].EndDayNo && 
			   CurrentTime >= m_RadioMessageVector[i].StartTime && CurrentTime <= m_RadioMessageVector[i].EndTime )
			{

			
			m_prevailing_travel_time+= m_RadioMessageVector[i].DelayPenaltyInMin; 
			}
		}


		return m_prevailing_travel_time;


	};

	float GetTrafficVolumeByMin(int DayNo,int starting_time, int time_interval)  // DayNo =-1: unknown day
	{

		int t;
		int total_flow =0;
		int time_end = min(starting_time+time_interval, m_SimulationHorizon);
		for(t=starting_time; t< time_end; t++)
		{
			total_flow +=  m_LinkMOEAry[t].TotalFlowCount ;
		}

		return total_flow;

	};


	float GetTravelTimeByMin(int DayNo,int starting_time, int time_interval, e_traffic_flow_model TrafficModelFlag)  // DayNo =-1: unknown day
	{
		float travel_time  = 0.0f;
		starting_time = max(0, starting_time);

		//// condition 1:road blockage caused by work zones
		//if(GetNumberOfLanes_ImpactedByWorkZoneConditions(DayNo,starting_time)<=0.001)   // 
		//	return 9999; // unit min

		ASSERT(m_SimulationHorizon < m_LinkMOEAry.size());

		// condition 2:  BRF travel time

		if (TrafficModelFlag == tfm_BPR) // BPR model
		{
			int period_no = 0;
			period_no = g_FindAssignmentIntervalIndexFromTime(starting_time);
			return m_BPRLinkTravelCostVector[period_no];  // used in shortest path
		}



		// condition 3: traffic flow model to determine time-dependent travel time for point queue, spatial queue or Newell's model
		int t;
		float total_travel_time = 0;
		int total_flow =0;
		int time_end = min(starting_time+time_interval, m_SimulationHorizon);

		float time_dependent_free_flow_traveltime = GetFreeMovingTravelTime(2, 0, starting_time);
		for(t=starting_time; t< time_end; t++)
		{
			total_travel_time += m_LinkMOEAry[t].TotalTravelTime;
			total_flow +=  m_LinkMOEAry[t].TotalFlowCount ;
		}

		if(total_flow >= 1)
		{
			travel_time =  total_travel_time/total_flow;
			if(travel_time < time_dependent_free_flow_traveltime)
				travel_time = time_dependent_free_flow_traveltime; // minimum travel time constraint for shortest path calculation
		}
		else
			travel_time =  time_dependent_free_flow_traveltime;

		if(travel_time > 99999)
		{
			TRACE("Error, link %d", this ->m_LinkNo );
			for(t=starting_time; t< time_end; t++)
			{
				TRACE("t = %d,%f\n",t,m_LinkMOEAry[t].TotalTravelTime);
			}

		}

		return travel_time;

	};


};

// link element of a vehicle path

class SVehicleLink
{  public:

#ifdef _WIN64
unsigned long  LinkNo;  // range: 4294967295
#else
unsigned short  LinkNo;  // range:  65535
#endif

float AbsArrivalTimeOnDSN;     // absolute arrvial time at downstream node of a link: 0 for the departure time, including delay/stop time
//   float LinkWaitingTime;   // unit: 0.1 seconds
SVehicleLink()
{
	LinkNo = MAX_LINK_NO;
	AbsArrivalTimeOnDSN = 99999;
	//		LinkWaitingTime = 0;
}

};

struct VehicleTimestampSpeed
{
	int timestamp_in_second;
	float speed;

};

class TimeDependentDemandProfile
{
public:
	int from_zone_id;
	int to_zone_id;
	int demand_type;
	double time_dependent_ratio[MAX_TIME_INTERVAL_SIZE];

	TimeDependentDemandProfile()
	{
		for(int interval =0; interval < MAX_TIME_INTERVAL_SIZE; interval++)
		{
			time_dependent_ratio[interval] = 0;
		}

	}

};

class DTAVehicleType
{
public:
	DTAVehicleType()
	{
		vehicle_type = 1;
		rollingTermA = 0.156461f;
		rotatingTermB = 0.00200193f;
		dragTermC = 0.000492646f;
		sourceMass = 1.4788f;
		PCE = 1;

	}

	float PCE; 

	float rollingTermA;
	float rotatingTermB;
	float dragTermC;
	float sourceMass;

	int vehicle_type;
	string vehicle_type_name;
	std::vector<float> percentage_age_vector;
};


class DTALinkType
{
public:
	float default_lane_capacity;
	int link_type;
	string link_type_name;
	string type_code;

	int safety_prediction_model_id;
	float link_type_bias_factor;
	float capacity_adjustment_factor;
	int approximate_cycle_length_in_second;
	float saturation_flow_rate_in_vhc_per_hour_per_lane;

	int number_of_links;

	float total_lane_capacity;
	float total_speed_limit;
	float total_number_of_lanes;
	float total_k_jam;
	float total_length;

	DTALinkType()
	{
		number_of_links = 0;
		capacity_adjustment_factor = 1.0;
		saturation_flow_rate_in_vhc_per_hour_per_lane = 1800;
		approximate_cycle_length_in_second = 0;
		link_type_bias_factor = 1.0;
		default_lane_capacity = 1000;

		total_lane_capacity = 0;
		total_speed_limit = 0;
		total_number_of_lanes =0 ;
		total_k_jam = 0;
		total_length = 0;

	}

	bool IsParking()
	{
		if (type_code.find('p') != string::npos)
			return true;
		else
			return false;
	}
	bool IsFreeway()
	{
		if(type_code.find('f')!= string::npos)
			return true;
		else
			return false;
	}

	bool IsHighway()
	{
		if(type_code.find('h')!= string::npos)
			return true;
		else
			return false;
	}

	bool IsArterial()
	{
		if(type_code.find('a')!= string::npos)
			return true;
		else
			return false;
	}

	bool IsRamp()
	{
		if(type_code.find('r')!= string::npos)
			return true;
		else
			return false;
	}

	bool IsConnector()
	{
		if(type_code.find('c')!= string::npos)
			return true;
		else
			return false;
	}

	/*bool IsTransit()
	{
		if(type_code.find('t')!= string::npos)
			return true;
		else
			return false;
	}
	bool IsWalking()
	{
		if(type_code.find('w')!= string::npos)
			return true;
		else
			return false;
	}*/
};

class DTAPath
{
public:
	std::vector<int> LinkSequence;
	int NodeSum;
	float Distance;
	float TravelTime;
	bool bDiverted;

	DTAPath ()
	{ 
		bDiverted = false;
		Distance = 0;
		TravelTime = 0;

	}
};

class VehicleSpeedProfileData
{
public:
	VehicleSpeedProfileData()
	{
	Energy = CO2 =  NOX =  CO =  HC =  PM = PM2_5 = 0;
	};
	int FromNodeNumber;
	int ToNodeNumber;
	float Speed;
	float Acceleration;
	int OperationMode;
	float VSP;
	VSP_BIN VSPBinNo;
	SPEED_BIN SpeedBinNo;
	float Energy, CO2, NOX, CO, HC, PM, PM2_5;
	int TimeOnThisLinkInSecond;

	CString GetSpeedBinNoString()
	{
		CString str;
		switch(SpeedBinNo)
		{
		case VSP_0_25mph: str.Format ("0_25 mph"); break;
		case VSP_25_50mph: str.Format ("25_50 mph"); break;
		case VSP_GT50mph: str.Format (">=50 mph"); break;
		default: str.Format("Unknown"); break;
		}
		return str;

	}

	CString GetVSPBinNoString()
	{
		CString str;
		switch(VSPBinNo)
		{
		case VSP_LT0: str.Format("VSP_<=0"); break;
		case VSP_0_3: str.Format("VSP_0_3"); break;
		case VSP_3_6: str.Format("VSP_3_6"); break;
		case VSP_6_9: str.Format("VSP_6_9"); break;
		case VSP_9_12: str.Format("VSP_9_12"); break;
		case VSP_12_18: str.Format("VSP_12_18"); break;
		case VSP_18_24: str.Format("VSP_18_24"); break;
		case VSP_24_30: str.Format("VSP_24_30"); break;
		case VSP_GT30: str.Format("VSP_>=30"); break;
		default: str.Format("Unknown"); break;
		}
		return str;

	}

};
class DTADecisionAlternative
{
public:

	DTADecisionAlternative()
	{
		total_cost = 999999;  // initial value
		node_size = 0;
		final_departuret_time_shift = 0;
	}

	float total_cost;
	int final_departuret_time_shift;
	int node_size;
	int path_link_list[MAX_NODE_SIZE_IN_A_PATH];

	void UpdateForLowerAlternativeCost(float temp_total_cost, int departure_time_shift, int temp_node_size, int temp_path_link_list[MAX_NODE_SIZE_IN_A_PATH])
	{  // by using this updating funciton, we easily add many alternatives (for mode, departure time choices)

		if(temp_total_cost < total_cost)
		{
			total_cost = temp_total_cost;
			final_departuret_time_shift = departure_time_shift;
			node_size = temp_node_size;

			for(int n = 0; n < MAX_NODE_SIZE_IN_A_PATH; n++)
				path_link_list[n] = temp_path_link_list[n];
		}

	}

};
class DTAVMSRespone
{
public:
	int LinkNo;
	int ResponseTime;
	int SwitchFlag;

	DTAVMSRespone()
	{
	ResponseTime = 0;
	SwitchFlag = 0;
	}

};

class DTAEvacuationRespone
{
public:
	int LinkNo;
	int ResponseTime;
	int SwitchFlag;

	DTAEvacuationRespone()
	{
	ResponseTime = 0;
	SwitchFlag = 0;
	}

};


class DTAVehListPerTimeInterval
{
public: 
	std::vector<int> m_AgentIDVector;

};

class DTAStopTime
{

public: 
	float ArrivalTimeInMin;
	float DepartureTimeInMin;
	
	void Init()
	{
		ArrivalTimeInMin = -1;
		DepartureTimeInMin = -1;
	
	}
	DTAStopTime ()
	{
		ArrivalTimeInMin = -1;
		DepartureTimeInMin = -1;
	
	}


};
class DTAVehicle
{
public:

	std::map<int, DTAStopTime> m_StopTimeMap;  // key is the node id

	float m_PCE;

	double within_link_driving_distance;  // unit: mile
	double within_link_driving_speed_per_hour;  // unit: mile per hour

	bool b_already_output_flag;
	bool b_od_already_output_flag;

	std::list<int> m_CurrentPassengerList;  // for transit vehicle

	int m_EvacuationTime_in_min;
	int m_EvacuationDestinationZone;
	bool m_bEvacuationMode;
	bool m_bEvacuationResponse;
	//std::map<int,DTAPath> Day2DayPathMap;
	//// multi-day equilibrium
	//bool m_bETTFlag;
	//int m_DayDependentLinkSize[MAX_DAY_SIZE];
	//std::map<int, int> m_DayDependentAryLink;  // first key: day*MAX_PATH_LINK_SIZE + index, second element; return link index
	//float m_DayDependentTripTime[MAX_DAY_SIZE];
	//int m_DayDependentNodeNumberSum[MAX_DAY_SIZE];  // used for comparing two paths
	//float m_DayDependentGap[MAX_DAY_SIZE];  // used for gap analysis
	//float m_AvgDayTravelTime;
	//float m_DayTravelTimeSTD;


	int m_NodeSize;
	int m_NodeNumberSum;  // used for comparing two paths
	SVehicleLink *m_LinkAry; // link list arrary of a vehicle path  // to do list, change this to a STL vector for better readability

	SVehicleLink *m_OriginalLinkAry;
	int m_OrgNodeSize;

	void StoreOriginalPath()
	{
		if (m_OriginalLinkAry == NULL)  // we have not set the original path
		{
			m_OriginalLinkAry = new SVehicleLink[m_NodeSize];

			for (int j = 0; j< m_NodeSize - 1; j++)  // for all nodes
			{
				m_OriginalLinkAry[j].LinkNo = m_LinkAry[j].LinkNo;
			}

			m_OrgNodeSize = m_NodeSize;
		}

	}

	float m_PrevSpeed;

	unsigned int m_RandomSeed;
	int m_AgentID;
	int m_ExternalTourID;
	int m_OriginZoneID;  
	int m_DestinationZoneID; 

	int m_OriginNodeID;
	int m_DestinationNodeID;

	int m_DemandType;     // 1: passenger,  2, HOV, 3, truck, 3: bus
	int m_VehicleType;    // for emissions analysis
	int m_InformationType;
	 
	bool m_transit_service_flag;  // = 1 for transit vehicle
	// 0: historical path (no change), --> fixed route flag
	// 1: historical learning using time-dependent travel time from the previous day,
	// 2: pre-trip, 
	// 3: en-route; 
	// 4: personalized info; 
	// 5: eco so info

	int m_SimLinkSequenceNo; //  range 0, 65535

	bool  m_bImpacted;

	double m_TimeToRetrieveInfo;
	double m_EnrouteInformationUpdatingTimeIntervalInMin;
	int m_following_agent_id;
	int m_dependency_agent_id;
	float m_duration_in_min;

	bool m_bRadioMessageResponseFlag;

	float m_DepartureTime;
	float m_LeavingTimeFromLoadingBuffer;

	float m_PreferredDepartureTime;
	float m_Distance;

	float m_ArrivalTime;
	float m_TripTime;
	float m_TripFFTT;
	float m_BufferWaitingTime;
	float m_TravelTime;
	float m_EstimatedTravelTime;
	float m_Delay;

	bool m_bForcedSwitchAtFirstIteration; // used by agent model, if there is a newly added work zone, then we have to force the vehicles to switch (in order to avoid bloced links)
	bool m_bSwitch;  // switch route in assignment
	float m_gap;
	bool m_bMeetTarget;
	bool m_gap_update;
	bool m_bConsiderToSwitch;  //consider to switch route in assignment

	bool m_bTag;
	// used for simulation
	bool m_bLoaded; // be loaded into the physical network or not
	bool m_bComplete;

	bool m_bDetailedEmissionOutput;
	float Energy, CO2, NOX, CO, HC, PM, PM2_5;

	int m_DestinationZoneID_Updated; 
	int m_attribute_update_time_in_min;

	int m_Age;
	float m_VOT;        // range 0 to 255
	float m_TollDollarCost;

	float m_MinCost;
	float m_MeanTravelTime;
	float m_TravelTimeVariance;
	unsigned short m_NumberOfSamples;  // when switch a new path, the number of samples starts with 0

	std::vector<GDPoint> m_ShapePoints;

	std::vector<int> m_alt_path_node_sequence;

	//-------------MODIFIED BY QU----2015.10.29-------//
	string m_vehicleID;
	string m_routeID;
	float m_capacity;
	//------------------------------------------------//

	float GetLinkTravelTime(int link_sequence_no)
	{
		
		if (link_sequence_no >= this->m_NodeSize - 1)
			return 0;

		float TravelTime = 0;
		if (link_sequence_no >= 1)
		{
			TravelTime =m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN -m_LinkAry[link_sequence_no - 1].AbsArrivalTimeOnDSN;
		}
		else if (link_sequence_no == 0)
		{
			TravelTime = m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN - m_DepartureTime;
		
		}
	return TravelTime;
	}

	//void StorePath(int DayNo)
	//{

	//	Day2DayPathMap[DayNo].NodeSum = m_NodeNumberSum;

	//	for(int i = 0; i < m_NodeSize; i++)
	//		Day2DayPathMap[DayNo].LinkSequence.push_back (m_LinkAry[i].LinkNo );

	//};

	void PostTripUpdate(float TripTime)   
	{
		float GainFactor = 0.2f;  // will use formula from Kalman Filtering, eventually

		m_MeanTravelTime = (1-GainFactor)*m_MeanTravelTime + GainFactor*TripTime;
		m_NumberOfSamples +=1;
	};



#ifdef _large_memory_usage_lr
	std::vector<VehicleLinkPrice> m_PersonalizedRoadPriceVector;
#endif 


#ifdef _large_memory_usage
	std::vector<DTAVMSRespone> m_VMSResponseVector;
	std::map<int, int> m_OperatingModeCount;
	std::map<int, int> m_SpeedCount;
	std::map<int, VehicleSpeedProfileData> m_SpeedProfile;
#endif 




	DTAVehicle()
	{
		m_following_agent_id = -1;
		m_dependency_agent_id = -1;
		m_duration_in_min = -1;

		m_ExternalTourID = -1;
		m_transit_service_flag = false;
		m_PCE = 1;
		m_bTag = false;
		m_DepartureTime = -1;
		m_bMeetTarget = false;
		m_NodeNumberSum = 0;
		m_OriginNodeID = -1;
		m_DestinationNodeID = -1;


		b_already_output_flag = false;
		b_od_already_output_flag = false;

		m_AgentID = 0;
		m_gap = 0;
		m_gap_update = false;
		m_bForcedSwitchAtFirstIteration  = false;
		m_bRadioMessageResponseFlag = false;

		m_DestinationZoneID_Updated = 0;
		m_attribute_update_time_in_min = -1;

		m_bEvacuationMode = false;
		m_bEvacuationResponse = false;
		m_EvacuationTime_in_min = 0;
		m_EvacuationDestinationZone = 0;

		m_Age = 0;
		Energy = CO2 = NOX = CO = HC = PM = PM2_5 = 0;
		m_PrevSpeed = 0;
		m_TimeToRetrieveInfo = -1;
		m_EnrouteInformationUpdatingTimeIntervalInMin = -1;  // default value 
		m_SimLinkSequenceNo = 0;

		m_NumberOfSamples =0;
		m_VOT = DEFAULT_VOT;
		m_TollDollarCost = 0;

		m_MinCost = 0;

		m_LinkAry = NULL;
		m_OriginalLinkAry = NULL;
		m_NodeSize	= 0;
		m_OrgNodeSize = 0;
		m_bImpacted = false; 
		m_InformationType = learning_from_hist_travel_time;
		m_DemandType = 1;
		m_VehicleType = 1;
		m_DemandType = 0;
		m_ArrivalTime = 0;
		//      m_FinalArrivalTime = 0;
		m_bLoaded = false;
		m_bSwitch = false;
		m_bConsiderToSwitch = false;
		m_bComplete = false;
		m_TripTime = 900;  // default: for incomplete vehicles, they have an extremey long trip time
		m_TravelTime = 900;
		m_Distance =0;
		m_Delay = 0;

		m_OriginZoneID = -1;
		m_DestinationZoneID = -1;

		m_OriginNodeID = -1;
		m_DestinationNodeID = -1;

		m_bDetailedEmissionOutput = false;

	};
	~DTAVehicle()
	{
		if(m_LinkAry != NULL && m_NodeSize > 0)
			delete m_LinkAry;

	};

	void PreTripReset()
	{
		if(m_DemandType!=4)
		{// non transit
			m_ArrivalTime = 0;
			m_TripTime = 0;
			m_TripFFTT = 0;
			m_BufferWaitingTime = 0;
			m_TollDollarCost = 0;

			Energy = 0;
			CO2 = 0;
			NOX = 0;
			CO = 0;
			HC = 0;
		}

		m_bLoaded = false;
		m_bComplete = false;
		m_Delay = 0;


	}

	float GetRandomRatio()
	{
		m_RandomSeed = (LCG_a * m_RandomSeed + LCG_c) % LCG_M;  //m_RandomSeed is automatically updated.

		return float(m_RandomSeed)/LCG_M;
	}

	void SetMinCost(float MinCost)
	{
		m_MinCost = MinCost;
	};

	float GetMinCost()
	{
		return m_MinCost;

	};

};

class DTA_vhc_simple // used in STL sorting only
{
public:

	DTA_vhc_simple()
	{
	m_DepartureTime = 0;
	m_DemandType = 1;
	m_VehicleType = 1;
	m_DemandType = 1;
	m_InformationType = info_hist_based_on_routing_policy;
	m_Age = 0;
	m_TimeToRetrieveInfo = 0;

	
	m_VOT = 10;
	}

	int m_OriginZoneID;
	int m_DestinationZoneID;

	int m_DemandType;
	int m_DepartureTimeIndex;
	int m_VehicleType;
	int m_InformationType;
	int m_Age;

	float    m_DepartureTime;
	int m_TimeToRetrieveInfo;
	float m_VOT;
	int m_PathIndex;  // for OD estimation


	bool operator<(const DTA_vhc_simple &other) const
	{
		return m_DepartureTime < other.m_DepartureTime;
	}

};



class DemandProfile
{
public:
	int from_zone_id;
	double time_dependent_ratio[MAX_TIME_INTERVAL_SIZE];
	int demand_type;
	CString series_name;
	DemandProfile()
	{
		for(int interval =0; interval < MAX_TIME_INTERVAL_SIZE; interval++)
		{
			time_dependent_ratio[interval] = 0;
		}

	}

};



class NetworkMOE
{
public:
	int CumulativeInFlow;
	int CumulativeOutFlow;
	int Flow_in_a_min;
	float AbsArrivalTimeOnDSN_in_a_min;
	float TotalFreeFlowTravelTime;
	float AvgTripTime;
	float TotalDistance;
	float TotalBufferWaitingTime;


	NetworkMOE()
	{
		TotalFreeFlowTravelTime = 0;
		CumulativeInFlow = 0;
		CumulativeOutFlow = 0;
		Flow_in_a_min = 0;
		AbsArrivalTimeOnDSN_in_a_min = 0;
		AvgTripTime = 0;
		TotalDistance = 0;
		TotalBufferWaitingTime = 0;

	}
};


template <typename M> void FreeClearMap( M & amap ) 
{
	for ( typename M::iterator it = amap.begin(); it != amap.end(); ++it ) {
		delete it->second;
	}
	amap.clear();
}

class DTALinkToll
{
public:

	bool m_bMonetaryTollExist ;
	bool m_bTravelTimeTollExist; // unit: generalized travel time in min 

	float TollValue[MAX_DEMAND_TYPE_SIZE];

	DTALinkToll()
	{
		m_bMonetaryTollExist = false;
		m_bTravelTimeTollExist = false;
		for(int i=0; i< MAX_DEMAND_TYPE_SIZE; i++)
			TollValue[i] = 0;

	}
};

struct PathLinkStruct
{
	std::vector<int> LinkNoVector;

};


class PathArrayForEachODT // Jason : store the path set for each OD pair and each departure time interval
{
public:
	//  // for path flow adjustment
	int   NumOfVehicles;
	float   DeviationNumOfVehicles; 
	int  LeastTravelTime;

	//
	int   NumOfPaths;

	std::vector<float> AvgPathGap; 
	std::vector<float> NewNumberOfVehicles; 
	std::vector<int>   PathNodeSums;            // max 100 path for each ODT
	std::vector<int>   NumOfVehsOnEachPath; 	
	std::vector<int>   PathSize;				// number of nodes on each path
	std::vector<float>   MeasurementDeviationPathMarginal;            // max 100 path for each ODT
	std::vector<float> AvgPathTimes; 	       // average path travel time across different vehicles on the same path with the same departure time

	std::vector<PathLinkStruct>   PathLinkSequences;	// max 300 links on each path

void AddPathElement()
{
	AvgPathGap.push_back(0);
	NewNumberOfVehicles.push_back(0);
	PathNodeSums.push_back(0);
	NumOfVehsOnEachPath.push_back(0);
	PathSize.push_back(0);
	PathNodeSums.push_back(0);
	MeasurementDeviationPathMarginal.push_back(0);
	AvgPathTimes.push_back(0);
	PathLinkStruct pl;
	PathLinkSequences.push_back(pl);

}
void ClearPathElements()
{
	AvgPathGap.clear();
	NewNumberOfVehicles.clear();
	PathNodeSums.clear();
	NumOfVehsOnEachPath.clear();
	PathSize.clear();
	PathNodeSums.clear();
	MeasurementDeviationPathMarginal.clear();
	AvgPathTimes.clear();
	PathLinkSequences.clear();
}


	int   BestPathIndex;				// index of the best (i.e., least experienced time) path for each ODT

};

class PathArrayForEachODTK // Xuesong: store path set for each OD, tau and k set.
{
public: 
	int m_PathIndex; 
	int m_OriginZoneID;  //range 0, 65535
	int m_DestinationZoneID;  // range 0, 65535
	int m_DemandType;     // 1: passenger,  2, HOV, 2, truck, 3: bus

	int m_DepartureTimeIndex;
	int m_NodeSum; 

	float m_starting_time_in_min;
	float m_ending_time_in_min;

	int m_LinkSize;
	std::vector<int> m_LinkNoArray;
	float m_VehicleSize;
	float FlowAdjustmentGradient;  // gradient of objective function with respect to path flow (o,d, tau, p)  

	PathArrayForEachODTK()
	{
		FlowAdjustmentGradient = 0;
		m_LinkSize = 0;
		m_VehicleSize = 0;
		m_DepartureTimeIndex = 0;

	}

	void Setup(int PathIndex,int OriginZoneID, int DestinationZoneID, int DemandType, int starting_time_in_min, int ending_time_in_min, int LinkSize, std::vector<int> PathLinkSequences, float VehicleSize, int NodeSum, int DepartureTimeIndex)
	{
		m_PathIndex  = PathIndex;
		m_DepartureTimeIndex = DepartureTimeIndex;

		m_OriginZoneID = OriginZoneID;
		m_DestinationZoneID = DestinationZoneID;
		m_DemandType = DemandType;

		m_starting_time_in_min = starting_time_in_min;
		m_ending_time_in_min = ending_time_in_min;


		m_LinkSize = LinkSize;

		m_VehicleSize = VehicleSize;
		for(int i = 0; i< LinkSize; i++)
		{
			m_LinkNoArray.push_back (PathLinkSequences[i]);
		}


		m_NodeSum = NodeSum;
	}
	~ PathArrayForEachODTK()
	{

		m_LinkNoArray.clear();

	}
};

extern DTALinkToll** g_LinkTDCostAry;

class DTANetworkForSP  // mainly for shortest path calculation, not just physical network
	// for shortest path calculation between zone centroids, for origin zone, there are only outgoing connectors, for destination zone, only incoming connectors
	// different shortest path calculations have different network structures, depending on their origions/destinations
{
public:
	

	std::vector<PathArrayForEachODT> m_PathArray;

	int m_NumberOfSPCalculationIntervals;
	int m_NumberOfTDSPCalculationIntervals;

	int m_PlanningHorizonInMin;
	int m_StartTimeInMin;

	int m_NodeSize;
	int m_PhysicalNodeSize;
	int m_LinkSize;

// the following is for node-based scan eligible list
	int m_NodeBasedSEListFront;
	int m_NodeBasedSEListTail;
	int* m_NodeBasedSEList;  // dimension: number of nodes

// the following is for link-based scan eligible list
	int m_LinkBasedSEListFront;
	int m_LinkBasedSEListTail;
	int* m_LinkBasedSEList;  // dimension: number of links


	int** m_OutboundNodeAry; //Outbound node array
	int** m_OutboundLinkAry; //Outbound link array
	
	int** m_OutboundConnectorOriginZoneIDAry; //Outbound connector array
	int** m_OutboundConnectorDestinationZoneIDAry; //Outbound connector array (destination zone)
	
	int* m_LinkConnectorFlag; //Outbound connector array
	int* m_LinkConnectorOriginZoneIDAry; 
	int* m_LinkConnectorDestinationZoneIDAry; 

	int* m_OutboundSizeAry;  //Number of outbound links

	int** m_OutboundMovementAry; //Outbound link movement array: for each link
	int* m_OutboundMovementSizeAry;  //Number of outbound movement for each link
	float** m_OutboundMovementDelayAry; //Outbound link movement array: for each link

	int** m_InboundLinkAry; //inbound link array
	int* m_InboundSizeAry;  //Number of inbound links

	int* m_FromIDAry;
	int* m_ToIDAry;

	float** m_LinkTDTimeAry;

	float** m_LinkTDTransitTimeAry;

	float*  m_LinkTDDistanceAry;
	float*  m_LinkFFTTAry;


	int* NodeStatusAry;                // Node status array used in KSP;
	float* LabelTimeAry;               // label - time
	int* NodePredAry;  

	int** NodePredVectorPerType;  
	int** LinkPredVectorPerType;
	float** LabelCostVectorPerType;


	float* LabelCostAry;
	float* LabelDollarCostAry;

	float* LabelDistanceAry;

	int* LinkNoAry;  //record link no according to NodePredAry

	// movement calculation
	int* LinkStatusAry;                // Node status array used in KSP;
	float* LinkLabelTimeAry;               // label - time
	int* LinkPredAry;  
	float* LinkLabelCostAry;



	int m_Number_of_CompletedVehicles;
	int m_AdjLinkSize;

	//below are time-dependent cost label and predecessor arrays
	float** TD_LabelCostAry;
	int** TD_NodePredAry;  // pointer to previous NODE INDEX from the current label at current node and time
	int** TD_LinkPredAry;  // pointer to previous Link INDEX from the current label at current node and time
	int** TD_TimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time
	float** TD_LinkCostAry;

	int** TD_LinkTimeIntervalAry;
	float** TD_LinkVolumeAry;  // to keep track of link volume for marginal cost calculation

	float** TD_InflowLinkLabelCostAry;
	int** TD_InflowLinkTimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time

	float** TD_InflowEntranceQueueLabelCostAry;
	int** TD_InflowEntranceQueueTimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time

	float** TD_OutflowLinkLabelCostAry;
	int** TD_OutflowLinkTimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time

	float** TD_OutflowExitQueueLabelCostAry;
	int** TD_OutlowExitQueueTimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time


	//	std::list<int> m_ScanLinkList;  // used for movement-based scanning process, use a std implementation for simplicity

	int PathLinkList[MAX_NODE_SIZE_IN_A_PATH];
	int CurrentPathLinkList[MAX_NODE_SIZE_IN_A_PATH];
	int PathNodeList[MAX_NODE_SIZE_IN_A_PATH];

	int temp_reversed_PathLinkList[MAX_NODE_SIZE_IN_A_PATH];  // tempory reversed path node list

	DTANetworkForSP()
	{
		m_NodeSize = 0;
	};

	void Setup(int NodeSize, int LinkSize, int PlanningHorizonInMin,int AdjLinkSize, int StartTimeInMin=0, bool bODMEFlag=false)
	{
		m_NodeSize = NodeSize;
		m_LinkSize = LinkSize;

		m_PlanningHorizonInMin = PlanningHorizonInMin;
		m_StartTimeInMin = StartTimeInMin;

	//		cout <<"start to allocate network memory, size  = " << size << endl;
		m_NumberOfSPCalculationIntervals = g_NumberOfSPCalculationPeriods;  // make sure it is not zero

		m_NumberOfTDSPCalculationIntervals = int(m_PlanningHorizonInMin - m_StartTimeInMin) * g_TDSPTimetIntervalSizeForMin + 1;  // unit: 0.1 min
		
		m_AdjLinkSize = AdjLinkSize;

		m_OutboundSizeAry = new int[m_NodeSize];
		m_InboundSizeAry = new int[m_NodeSize];


		m_OutboundNodeAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize+1);
		m_OutboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize+1);
		m_OutboundConnectorOriginZoneIDAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize+1);
		m_OutboundConnectorDestinationZoneIDAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize+1);
		m_LinkConnectorFlag = new int[m_LinkSize];
		m_LinkConnectorOriginZoneIDAry = new int[m_LinkSize];
		m_LinkConnectorDestinationZoneIDAry = new int[m_LinkSize];



		m_InboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize+1);

		//movement-specific array
		m_OutboundMovementAry = AllocateDynamicArray<int>(m_LinkSize,m_AdjLinkSize+1);
		m_OutboundMovementDelayAry = AllocateDynamicArray<float>(m_LinkSize,m_AdjLinkSize+1);
		m_OutboundMovementSizeAry = new int[m_LinkSize];

		m_NodeBasedSEList = new int[m_NodeSize];
		m_LinkBasedSEList = new int[m_LinkSize];

		m_LinkTDDistanceAry = new float[m_LinkSize];
		m_LinkFFTTAry  = new float[m_LinkSize];


	//	cout <<"start to allocate time-dependent network memory " << endl;



		m_LinkTDTransitTimeAry = NULL;
		m_LinkTDTimeAry = NULL;

		m_LinkTDTimeAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfSPCalculationIntervals);

		//m_LinkTDTransitTimeAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfSPCalculationIntervals);

		TD_LabelCostAry = NULL;
		TD_TimePredAry = NULL;

#ifdef _large_memory_usage_lr
		TD_LabelCostAry = AllocateDynamicArray<float>(m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		TD_NodePredAry = AllocateDynamicArray<int>(m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		TD_LinkPredAry = AllocateDynamicArray<int>(m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		TD_TimePredAry = AllocateDynamicArray<int>(m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		TD_LinkCostAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_LinkTimeIntervalAry = AllocateDynamicArray<int>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_LinkVolumeAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);


		TD_InflowLinkLabelCostAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_InflowEntranceQueueLabelCostAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_OutflowLinkLabelCostAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_OutflowExitQueueLabelCostAry = AllocateDynamicArray<float>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);

		TD_InflowLinkTimePredAry = AllocateDynamicArray<int>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_InflowEntranceQueueTimePredAry = AllocateDynamicArray<int>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_OutflowLinkTimePredAry = AllocateDynamicArray<int>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		TD_OutlowExitQueueTimePredAry = AllocateDynamicArray<int>(m_LinkSize, m_NumberOfTDSPCalculationIntervals);

#endif

		m_FromIDAry = new int[m_LinkSize];

		m_ToIDAry = new int[m_LinkSize];

		NodeStatusAry = new int[m_NodeSize];                    // Node status array used in KSP;

		NodePredVectorPerType   =  AllocateDynamicArray<int>(g_DemandTypeVector.size()+1,m_NodeSize);
		LinkPredVectorPerType = AllocateDynamicArray<int>(g_DemandTypeVector.size() + 1, max(m_NodeSize, m_LinkSize));
		LabelCostVectorPerType = AllocateDynamicArray<float>(g_DemandTypeVector.size() + 1, max(m_NodeSize, m_LinkSize));

		NodePredAry = new int[m_NodeSize];
		LinkNoAry = new int[m_NodeSize];
		LabelTimeAry = new float[m_NodeSize];                     // label - time
		LabelCostAry = new float[m_NodeSize];                     // label - cost
		LabelDistanceAry = new float[m_NodeSize];                     // label - distance
		LabelDollarCostAry = new float[m_NodeSize];                  //lable dolloar cost

		LinkStatusAry = new int[m_LinkSize];                    // Node status array used in KSP;
		LinkPredAry = new int[m_LinkSize];
		LinkLabelTimeAry = new float[m_LinkSize];                     // label - time
		LinkLabelCostAry = new float[m_LinkSize];                     // label - cost

		if(bODMEFlag)
		{
			PathArrayForEachODT element;

			for(int z = 0; z < g_ODZoneIDSize; z++)
				m_PathArray.push_back (element);
		}
		cout <<"end of network memory allocation. " << endl;

	};
	DTANetworkForSP(int NodeSize, int LinkSize, int PlanningHorizonInMin,int AdjLinkSize, int StartTimeInMin=0)
	{

		Setup(NodeSize, LinkSize, PlanningHorizonInMin,AdjLinkSize,StartTimeInMin=0);
	};

	float GetTDTravelCost(int LinkNo, int time_interval_no)
	{
		return 0; 
	}

	float GetTDTransitTime(int LinkNo, int time_interval_no)
	{
		return 0;
	}

	void Clean()
	{
		if(m_OutboundSizeAry && m_NodeSize>=1)  delete m_OutboundSizeAry;
		if(m_InboundSizeAry && m_NodeSize>=1)  delete m_InboundSizeAry;

		DeallocateDynamicArray<int>(m_OutboundNodeAry,m_NodeSize, m_AdjLinkSize+1);
		DeallocateDynamicArray<int>(m_OutboundLinkAry,m_NodeSize, m_AdjLinkSize+1);
		DeallocateDynamicArray<int>(m_OutboundConnectorOriginZoneIDAry,m_NodeSize, m_AdjLinkSize+1);
		DeallocateDynamicArray<int>(m_OutboundConnectorDestinationZoneIDAry,m_NodeSize, m_AdjLinkSize+1);

		if(m_LinkSize>=1)
		{
			delete m_LinkConnectorFlag;
			delete m_LinkConnectorOriginZoneIDAry; 
			delete m_LinkConnectorDestinationZoneIDAry; 
		}
		DeallocateDynamicArray<int>(m_InboundLinkAry,m_NodeSize, m_AdjLinkSize+1);

		// delete movement array
		if(m_OutboundMovementSizeAry)  delete m_OutboundMovementSizeAry;
		DeallocateDynamicArray<int>(m_OutboundMovementAry,m_LinkSize, m_AdjLinkSize+1);
		DeallocateDynamicArray<float>(m_OutboundMovementDelayAry,m_LinkSize, m_AdjLinkSize+1);


		if(m_NodeBasedSEList) delete m_NodeBasedSEList;
		if(m_LinkBasedSEList) delete m_LinkBasedSEList;


		DeallocateDynamicArray<float>(m_LinkTDTimeAry, m_LinkSize, m_NumberOfSPCalculationIntervals);

		if (m_LinkTDTransitTimeAry!=NULL)
		DeallocateDynamicArray<float>(m_LinkTDTransitTimeAry, m_LinkSize, m_NumberOfSPCalculationIntervals);





		if(m_LinkTDDistanceAry) delete m_LinkTDDistanceAry;
		if(m_LinkFFTTAry) delete m_LinkFFTTAry;


		



#ifdef _large_memory_usage_lr
		DeallocateDynamicArray<float>(TD_LabelCostAry, m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_NodePredAry, m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_LinkPredAry, m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_TimePredAry, m_NodeSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<float>(TD_LinkCostAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_LinkTimeIntervalAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<float>(TD_LinkVolumeAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);


		DeallocateDynamicArray<float>(TD_InflowLinkLabelCostAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<float>(TD_InflowEntranceQueueLabelCostAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<float>(TD_OutflowLinkLabelCostAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<float>(TD_OutflowExitQueueLabelCostAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);

		DeallocateDynamicArray<int>(TD_InflowLinkTimePredAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_InflowEntranceQueueTimePredAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_OutflowLinkTimePredAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);
		DeallocateDynamicArray<int>(TD_OutlowExitQueueTimePredAry, m_LinkSize, m_NumberOfTDSPCalculationIntervals);

#endif 
		

		if(m_FromIDAry)		delete m_FromIDAry;
		if(m_ToIDAry)	delete m_ToIDAry;

		if(NodeStatusAry) delete NodeStatusAry;                 // Node status array used in KSP;
		if(NodePredAry) delete NodePredAry;


		DeallocateDynamicArray<int>(NodePredVectorPerType,g_DemandTypeVector.size()+1,m_NodeSize);
		DeallocateDynamicArray<int>(LinkPredVectorPerType, g_DemandTypeVector.size() + 1, max(m_NodeSize, m_LinkSize));
		DeallocateDynamicArray<float>(LabelCostVectorPerType, g_DemandTypeVector.size() + 1, max(m_NodeSize, m_LinkSize));

		if(LinkNoAry) delete LinkNoAry;
		if(LabelTimeAry) delete LabelTimeAry;
		if(LabelCostAry) delete LabelCostAry;
		if(LabelDistanceAry) delete LabelDistanceAry;
		if (LabelDollarCostAry) delete LabelDollarCostAry;

		if(LinkStatusAry) delete LinkStatusAry;                 // Node status array used in KSP;
		if(LinkPredAry) delete LinkPredAry;
		if(LinkLabelTimeAry) delete LinkLabelTimeAry;
		if(LinkLabelCostAry) delete LinkLabelCostAry;

	}

	~DTANetworkForSP()
	{
		if(m_NodeSize==0)
			return;

		// do not free memory now
		// Clean();


	};


	int GetFeasibleTDSPTimeInterval(int time_interval)
	{
		if (time_interval < 0)
			time_interval = 0;
		if (time_interval >= m_NumberOfTDSPCalculationIntervals)
			time_interval = m_NumberOfTDSPCalculationIntervals - 1;

		return time_interval;

	}


	float GetTollRateInMin(int LinkID, float Time, int DemandType);  // built-in function for each network_SP to avoid conflicts with OpenMP parallel computing

	void ResourcePricing_Subgraident(int iteration);


	void BuildNetworkBasedOnZoneCentriod(int DayNo,int ZoneID);
	void BuildTravelerInfoNetwork(int DayNo,int CurrentTime, float Perception_error_ratio);
	void BuildPhysicalNetwork(int DayNo, int CurZoneID, e_traffic_flow_model TraffcModelFlag, bool bUseCurrentInformation = false, double CurrentTime = 0);
	void InitializeTDLinkCost();

	void UpdateCurrentTravelTime(int DayNo, double CurrentTime = 0);
	void IdentifyBottlenecks(int StochasticCapacityFlag);

	bool TDLabelCorrecting_DoubleQueue(int origin, int origin_zone, int departure_time, int demand_type, float VOT, bool bDistanceCost, bool debug_flag, bool bDistanceCostOutput);   // Pointer to previous node (node)
	bool TDLabelCorrecting_DoubleQueue_PerDemandType(int CurZoneID, int origin, int departure_time, int demand_type, float VOT, bool bDistanceCost, bool debug_flag);   // Pointer to previous node (node)
	bool TDLabelCorrecting_DoubleQueue_PerDemandType_Movement(int CurZoneID, int origin, int departure_time, int demand_type, float VOT, bool bDistanceCost, bool debug_flag);   // Pointer to previous node (node)

	//movement based shortest path
	int FindBestPathWithVOTForSingleAgent_Movement(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool debug_flag, float PerceptionErrorRatio = 0);

	int FindOptimalNodePath_TDLabelCorrecting_DQ(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, float TargetTravelTime, float &OptimialTravelTimeInMin, bool bDebugFlag = false);
	int FindOptimalLinkPath_TDLabelCorrecting_DQ(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, float TargetTravelTime, float &OptimialTravelTimeInMin, bool bDebugFlag = false);
	int  FindOptimalSolution(int origin, int departure_time, int destination, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float TargetTravelTime, float &TotalCost, float &OptimialTravelTimeInMin);  // the last pointer is used to get the node array;
	int  FindBestPathWithVOTForSingleAgent(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool bDebugFlag = false, float PerceptionErrorRatio = 0);
	int  FindBestSystemOptimalPathWithVOT(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool ResponseToRadioMessage = false, bool bDebugFlag = false);
	int  FindOptimalLinkPathSolution(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool ResponseToRadioMessage = false, bool bDebugFlag = false);


	void ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval(int zone,int departure_time_begin, int departure_time_end, int iteration,bool debug_flag);
	float AgentBasedPathFindingForEachZoneAndDepartureTimeInterval(int zone,int departure_time_begin, int departure_time_end, int iteration);
	
	float AgentBasedPathOptimization(int zone, int departure_time_begin, int departure_time_end, int iteration);
	float AgentBasedUpperBoundSolutionGeneration(int zone, int departure_time_begin, int departure_time_end, int iteration);

	
	
	void ZoneBasedPathFindingForEachZoneAndDepartureTimeInterval_ODEstimation(int zone, int AssignmentInterval, int iteration);
	void AgentBasedVMSPathAdjustment(int VehicleID , double current_time);

	void AgentBasedPathAdjustment(int DayNo, int zone,int departure_time_begin, double current_time);
	void AgentBasedRadioMessageResponse(int DayNo, int zone,int departure_time_begin, double current_time);
	// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_NodeBasedSEListFront= -1;
		m_NodeBasedSEListTail= -1;
	}

	void SEList_push_front(int node)
	{
		if(m_NodeBasedSEListFront == -1)  // start from empty
		{
			m_NodeBasedSEList[node] = -1;
			m_NodeBasedSEListFront  = node;
			m_NodeBasedSEListTail  = node;
		}
		else
		{
			m_NodeBasedSEList[node] = m_NodeBasedSEListFront;
			m_NodeBasedSEListFront  = node;
		}

	}
	void SEList_push_back(int node)
	{
		if(m_NodeBasedSEListFront == -1)  // start from empty
		{
			m_NodeBasedSEListFront = node;
			m_NodeBasedSEListTail  = node;
			m_NodeBasedSEList[node] = -1;
		}
		else
		{
			m_NodeBasedSEList[m_NodeBasedSEListTail] = node;
			m_NodeBasedSEList[node] = -1;
			m_NodeBasedSEListTail  = node;
		}
	}

	bool SEList_empty()
	{
		return(m_NodeBasedSEListFront== -1);
	}

	int SEList_front()
	{
		return m_NodeBasedSEListFront;
	}

	void SEList_pop_front()
	{
		int tempFront = m_NodeBasedSEListFront;
		m_NodeBasedSEListFront = m_NodeBasedSEList[m_NodeBasedSEListFront];
		m_NodeBasedSEList[tempFront] = -1;
	}

	////////// link based SE List

		// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void LinkBasedSEList_clear()
	{
		m_LinkBasedSEListFront= -1;
		m_LinkBasedSEListTail= -1;
	}

	void LinkBasedSEList_push_front(int link)
	{
		if(m_LinkBasedSEListFront == -1)  // start from empty
		{
			m_LinkBasedSEList[link] = -1;
			m_LinkBasedSEListFront  = link;
			m_LinkBasedSEListTail  = link;
		}
		else
		{
			m_LinkBasedSEList[link] = m_LinkBasedSEListFront;
			m_LinkBasedSEListFront  = link;
		}

	}
	void LinkBasedSEList_push_back(int link)
	{
		if(m_LinkBasedSEListFront == -1)  // start from empty
		{
			m_LinkBasedSEListFront = link;
			m_LinkBasedSEListTail  = link;
			m_LinkBasedSEList[link] = -1;
		}
		else
		{
			m_LinkBasedSEList[m_LinkBasedSEListTail] = link;
			m_LinkBasedSEList[link] = -1;
			m_LinkBasedSEListTail  = link;
		}
	}

	bool LinkBasedSEList_empty()
	{
		return(m_LinkBasedSEListFront== -1);
	}

	int LinkBasedSEList_front()
	{
		return m_LinkBasedSEListFront;
	}

	void LinkBasedSEList_pop_front()
	{
		int tempFront = m_LinkBasedSEListFront;
		m_LinkBasedSEListFront = m_LinkBasedSEList[m_LinkBasedSEListFront];
		m_LinkBasedSEList[tempFront] = -1;
	}

	/////////////// end of link-based SE list
	int  GetLinkNoByNodeIndex(int usn_index, int dsn_index);

};



int g_read_integer(FILE* f, bool special_char_handling = true);
int g_read_integer_with_char_O(FILE* f);
bool g_detect_if_a_file_is_column_format(LPCTSTR lpszFileName);
int read_multiple_integers_from_a_string(CString str, std::vector<int> &vector);


float g_read_float(FILE *f);

float g_read_float_from_a_line(FILE *f);

int g_read_number_of_numerical_values(char* line_string, int length);
int g_read_numbers_from_a_line(FILE* f, std::vector<float> &ValueVector);

void ReadNetworkTables();
int CreateVehicles(int originput_zone, int destination_zone, float number_of_vehicles, int demand_type, float starting_time_in_min, float ending_time_in_min,int PathIndex = -1, bool bChangeHistDemandTable=true, int departure_time_index = 0);

void Assignment_MP(int id, int nthreads, int node_size, int link_size, int iteration);

struct NetworkLoadingTimeDepedentMOE
{
public: 

	NetworkLoadingTimeDepedentMOE()
	{
		 TotalTripTime = 0;
		 TotalTripFFTT = 0;
		 TotalTravelTime = 0;
		 TotalDelay = 0;
		 TotalDistance  = 0;
		 VehicleSizeComplete = 0;
		 AvgTTI = 1;

		AvgTripTime = 0;
		AvgTravelTime = 0;
		AvgDelay = 0;
		AvgTripTimeIndex = 1;
		AvgDistance = 0;
		NumberofVehiclesCompleteTrips = 0;
		NumberofVehiclesGenerated = 0;
		SwitchPercentage = 0;
		ConsideringSwitchPercentage = 0;
		AvgUEGap = 0;
		AvgRelativeUEGap = 0;
	
	}

	double TotalTripTime; 
	double TotalTripFFTT;
	double TotalTravelTime; 
	double TotalDelay;
	double TotalDistance ;
	double VehicleSizeComplete;

	double AvgTripTime;
	double AvgTravelTime;
	double AvgTripTimeIndex;
	double AvgDelay;
	double AvgTTI;
	double AvgDistance;
	int   NumberofVehiclesCompleteTrips;
	int   NumberofVehiclesGenerated;
	double SwitchPercentage;
	double ConsideringSwitchPercentage; 
	double AvgUEGap;
	double AvgRelativeUEGap;

};	

struct NetworkLoadingTimeDepedentGap
{
public:
	NetworkLoadingTimeDepedentGap()
	{
	
	total_gap = 0;
	total_relative_gap = 0;
	NumberofVehiclesWithGapUpdate = 0;

	}
	double total_gap;
	double total_relative_gap;
	int NumberofVehiclesWithGapUpdate;

};

struct NetworkLoadingOutput
{
public:

	struc_LinearRegressionResult ODME_result_link_speed;
	struc_LinearRegressionResult ODME_result_link_count;  
	struc_LinearRegressionResult ODME_result_lane_density;

	NetworkLoadingOutput()
	{
		ResetStatistics();
	}

	void ResetStatistics ()
	{
		AvgUEGap = 0;
		AvgRelativeUEGap = 0;
		TotalDemandDeviation = 0;
		LinkVolumeAvgAbsError  =0 ;
		LinkVolumeAvgAbsPercentageError  =0 ;
		LinkVolumeRootMeanSquaredError = 0;
		CorrelationBetweenObservedAndSimulatedLinkVolume = 0;

		LinkSpeedAvgAbsError = 0;
		LinkSpeedAvgAbsPercentageError = 0;
		LinkSpeedRootMeanSquaredError = 0;
		CorrelationBetweenObservedAndSimulatedLinkSpeed = 0;

		AvgTripTime = 0;
		AvgTravelTime = 0;
		AvgDelay = 0;
		AvgTTI = 0;
		AvgDistance = 0;
		AvgCO = 0;
		NumberofVehiclesCompleteTrips = 0;
		NumberofVehiclesGenerated = 0;
		SwitchPercentage = 0;
		ConsideringSwitchPercentage = 0;
		NetworkClearanceTimeStamp_in_Min = 1440;
		NetworkClearanceTimePeriod_in_Min = 1440;

		TimeDedepentMOEMap.clear();
		TimeDedepentMOEMap.clear();
	}

	int   NetworkClearanceTimeStamp_in_Min;
	int  NetworkClearanceTimePeriod_in_Min;

	double AvgTripTime;
	double AvgTravelTime;
	double AvgDelay;
	double AvgTTI;
	double AvgDistance;

	double AvgCO;

	int   NumberofVehiclesCompleteTrips;
	int   NumberofVehiclesGenerated;
	double SwitchPercentage;
	double ConsideringSwitchPercentage; 
	double AvgUEGap;
	double AvgRelativeUEGap;

	std::map<int,NetworkLoadingTimeDepedentMOE> TimeDedepentMOEMap;
	std::map<int,NetworkLoadingTimeDepedentGap> TimeDedepentGapMap;


	double GetTimeDependentAvgTravelTime(int time)
	{
	int time_interval_no = time / 15;

		if(TimeDedepentMOEMap.find(time_interval_no)!= TimeDedepentMOEMap.end())
		{
			return TimeDedepentMOEMap[time_interval_no].AvgTravelTime;
			
		}else 
		{
			return 0;

		}
	}


	double GetTimeDependentAvgTravelDistance(int time)
	{
	int time_interval_no = time / 15;

		if(TimeDedepentMOEMap.find(time_interval_no)!= TimeDedepentMOEMap.end())
		{
			return TimeDedepentMOEMap[time_interval_no].AvgDistance ;
			
		}else 
		{
			return 0;

		}
	}

		double GetTimeDependentAvgGap(int time)
	{
	int time_interval_no = time / 15;

		if(TimeDedepentGapMap.find(time_interval_no)!= TimeDedepentGapMap.end())
		{
		 return TimeDedepentGapMap[time_interval_no].total_gap /max(1,TimeDedepentGapMap[time_interval_no].NumberofVehiclesWithGapUpdate);
			
		}else 
		{
			return 0;

		}
	}


		double GetTimeDependentAvgRelativeGapInPercentage(int time)
	{
	int time_interval_no = time / 15;

		if(TimeDedepentGapMap.find(time_interval_no)!= TimeDedepentGapMap.end())
		{
		 return TimeDedepentGapMap[time_interval_no].total_relative_gap *100 /max(1,TimeDedepentGapMap[time_interval_no].NumberofVehiclesWithGapUpdate);
			
		}else 
		{
			return 0;

		}
	}
	double TotalDemandDeviation;

	double LinkVolumeAvgAbsError;
	double LinkVolumeAvgAbsPercentageError;
	double LinkVolumeRootMeanSquaredError;
	double CorrelationBetweenObservedAndSimulatedLinkVolume;

	double LinkSpeedAvgAbsError;
	double LinkSpeedAvgAbsPercentageError;
	double LinkSpeedRootMeanSquaredError;
	double CorrelationBetweenObservedAndSimulatedLinkSpeed;

};

NetworkLoadingOutput g_NetworkLoading(e_traffic_flow_model TrafficFlowModelFlag, int SimulationMode, int Iteration);  // NetworkLoadingFlag = 0: static traffic assignment, 1: vertical queue, 2: spatial queue, 3: Newell's model, 




class NetworkSimulationResult
{
public: 

	int number_of_vehicles;
	int number_of_vehicles_DemandType[MAX_DEMAND_TYPE_SIZE];

	float avg_travel_time_in_min, avg_distance_s, avg_speed,avg_trip_time_in_min;


	float Energy, CO2, NOX, CO, HC, PM, PM2_5;

	NetworkSimulationResult()
	{

		number_of_vehicles = 0;
		for(int p = 0; p < MAX_DEMAND_TYPE_SIZE; p++)
		{
			number_of_vehicles_DemandType [p] = 0;
		}
		avg_trip_time_in_min = 0;
		avg_travel_time_in_min = 0;
		avg_distance_s = 0;

		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
		PM = 0;
		PM2_5 = 0;
	}
};


class ODStatistics
{
public: 

	ODStatistics()
	{
		TotalVehicleSize = 0;
		TotalCompleteVehicleSize = 0;
		TotalTravelTime = 0;
		TotalDistance = 0;
		TotalCost = 0;
		TotalEmissions = 0;

		Diverted_TotalVehicleSize = 0;
		Diverted_TotalTravelTime = 0;
		Diverted_TotalDistance = 0;

		OriginZoneNumber = 0;
		DestinationZoneNumber = 0;

	}

	int OriginZoneNumber;
	int DestinationZoneNumber;

	int   TotalVehicleSize;
	int   TotalCompleteVehicleSize;
	float TotalTravelTime;
	float TotalDistance;
	float TotalCost;
	float TotalEmissions;

	int   Diverted_TotalVehicleSize;
	float Diverted_TotalTravelTime;
	float Diverted_TotalDistance;

};



class PathStatistics
{
public: 
	PathStatistics()
	{
		TotalVehicleSize = 0;
		TotalTravelTime = 0;
		TotalDistance = 0;
		TotalCost = 0;
		TotalEmissions = 0;
		TotalFFTT = 0;
		Ratio = 0;
		CumulativeRatio = 0;
		NodeSums = 0;
		m_LinkSize = 0;
	}
	int Origin_ZoneID;
	int Destination_ZoneID;

	int   NodeSums;
	float Ratio; 
	float CumulativeRatio;

	int m_LinkSize;
	std::vector<int> m_LinkIDArray;
	std::vector<int> m_NodeNumberArray;

	int   TotalVehicleSize;
	float TotalTravelTime;
	float TotalFFTT;
	float TotalDistance;
	float TotalCost;
	float TotalEmissions;

};

class ODPathSet
{
public: 
	std::vector<PathStatistics> PathSet;
	int   TotalVehicleSize;
	float TotalTravelTime;
	float TotalDistance;
	float TotalCost;
	float TotalEmissionsEnergy;
	float TotalEmissionsCO2;
	float TotalEmissionsCO;
	float TotalEmissionsNOX;
	float TotalEmissionsHC;

	ODPathSet()
	{
		TotalVehicleSize = 0;
		TotalTravelTime = 0;
		TotalDistance = 0;
		TotalCost = 0;
		TotalEmissionsEnergy = TotalEmissionsCO2 = TotalEmissionsNOX  = TotalEmissionsHC = TotalEmissionsCO =0;
	}


};


typedef struct  
{
	int vehicle_id;
	int from_zone_id;
	int to_zone_id;
	float departure_time;
	float arrival_time;
	int complete_flag;
	float trip_time;
	int demand_type;
	int agent_type;
	int vehicle_type;
	int information_type;
	float value_of_time;
	float toll_cost_in_dollar;
	float PM;
	float distance_;
	int number_of_nodes;
	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;

	int age;
	int version_no;

	int day_no;
	float PM2_5;
	int number_of_VMS_response_links;

} struct_VehicleInfo_Header;

	typedef  struct  
	{
		int NodeName;
		float AbsArrivalTimeOnDSN;
	} struct_Vehicle_Node;

class VehicleArrayForOriginDepartrureTimeInterval
{
public:

	VehicleArrayForOriginDepartrureTimeInterval()
	{
		m_DemandGenerationRatio = 1.0f;
	}

	float m_DemandGenerationRatio;
	std::vector<int> VehicleArray;
	std::vector<PathArrayForEachODTK> m_ODTKPathVector;  // for ODME

	~VehicleArrayForOriginDepartrureTimeInterval()
	{
		VehicleArray.clear();
		m_ODTKPathVector.clear();
	}
};

class DTASettings
{
public:
	int use_mile_or_km_as_length_unit;
	int AdditionalYellowTimeForSignals;
	int IteraitonNoStartSignalOptimization;
	int IteraitonStepSizeSignalOptimization;
	int DefaultCycleTimeSignalOptimization;

	int pretimed_signal_control_type_code;
	int actuated_signal_control_type_code;

	int no_signal_control_type_code;

	int use_point_queue_model_for_on_ramps;  // no queue spillback
	int use_point_queue_model_for_off_ramps; //
	DTASettings()
	{

		use_mile_or_km_as_length_unit = 1;
		AdditionalYellowTimeForSignals = 0;
		IteraitonNoStartSignalOptimization = 10000;
		IteraitonStepSizeSignalOptimization = 5;
		DefaultCycleTimeSignalOptimization = 60;

		pretimed_signal_control_type_code = 5;
		actuated_signal_control_type_code = 6;
		no_signal_control_type_code = 1;
		use_point_queue_model_for_on_ramps = 1;
		use_point_queue_model_for_off_ramps = 1;

	}

};

typedef struct{
	int veh_id;
	double current_time_stamp;

}struc_real_time_path_computation_element;

class GridNodeSet
{
public:
	double x;
	double y;

	int x_int;
	int y_int;

	std::vector<int> m_NodeVector;
	std::vector<int> m_LinkNoVector;

	std::vector<float> m_NodeX;
	std::vector<float> m_NodeY;

	//for transit
	std::vector<int> m_TripIDVector;
	std::vector<int> m_StopIDVector;

};


extern double g_GridXStep;
extern double g_GridYStep;

extern GridNodeSet** g_GridMatrix;



extern DTASettings g_settings;
void Assignment_MP(int id, int nthreads, int node_size, int link_size, int iteration);

void g_OutputMOEData(int iteration, bool Day2DayOutputFlag=false);

void OutputLinkMOEDataHybridFromat(char fname[_MAX_PATH], int Iteration,bool bStartWithEmpty);

void OutputLinkMOEData(std::string fname, int Iteration, bool bStartWithEmpty);

void OutputRealTimeLinkMOEData(std::string fname,int current_time_in_min, int output_MOE_aggregation_time_interval_in_min, bool bTravelTimeOnly = true);

void g_UpdateRealTimeLinkAttributes();

bool g_ReadRealTimeLinkAttributeData(int current_time_in_second);
bool g_ReadRealTimeTripData(int current_time_in_minute, bool b_InitialLoadingFlag);

void OutputNetworkMOEData(ofstream &output_NetworkTDMOE_file);
void OutputVehicleTrajectoryData(std::string fname_agent, std::string  fname_trip, int Iteration, bool bStartWithEmpty, bool bIncremental);
bool OutputTripFile(char fname_trip[_MAX_PATH], int output_mode);
void OutputODMOEData(ofstream &output_ODMOE_file,int cut_off_volume = 1, int arrival_time_window_begin_time_in_min =0 );
void OutputTimeDependentODMOEData(ofstream &output_ODMOE_file,int department_time_intreval = 60, int end_time_in_min = 1440, int cut_off_volume = 1 );
void OutputEmissionData();
void g_OutputTimeDependentRoutingPolicyData(ofstream &output_PathMOE_file,int DemandLoadingStartTimeInMin, int DemandLoadingEndTimeInMin, int cut_off_volume = 50);
void OutputAssignmentMOEData(char fname[_MAX_PATH], int Iteration,bool bStartWithEmpty);


float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename,bool print_out=false);
int g_WritePrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename) ;
int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename, bool print_out=false);

float GetStochasticCapacity(bool bQueueFlag, float CurrentCapacity);

float GetTimeDependentCapacityAtSignalizedIntersection(int CycleLength_in_second, int EffectiveGreenTime_in_second, int GreenStartTime_in_second, int offset_in_second, double CurrentTime, float SaturationFlowRate);

void InitWELLRNG512a (unsigned int *init);
double WELLRNG512a (void);

double g_GetRandomRatio();
int g_GetRandomInteger_SingleProcessorMode(float Value);
int g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(float Value, int LinkID);

void g_ReadDTALiteAgentBinFile(string file_name);
bool g_ReadAgentBinFile(string file_name, bool b_with_updated_demand_type_info = false);
void g_ReadInputLinkTravelTime();
bool g_ReadTripCSVFile(string file_name, bool bOutputLogFlag);

void g_ReadDemandFile();
void g_ReadDemandFileBasedOnUserSettings();

void g_ZoneBasedDynamicTrafficAssignmentSimulation();
void g_ZoneBasedPeriodBasedDynamicTrafficAssignment();
void g_AgentBasedDynamicTrafficAssignmentSimulation();
void g_AgentBasedOptimization();

void g_ShortestPathDataMemoryAllocation() ;
void g_AgentBasedPathAdjustmentWithRealTimeInfo(int ProcessID, int VehicleID, double current_time);
void g_UpdateRealTimeInformation(double CurrentTime);
void g_OpenMPAgentBasedPathAdjustmentWithRealTimeInfo(int VehicleID, double current_time, int IS_id, MessageSign vms);
void g_MultiDayTrafficAssisnment();
void OutputMultipleDaysVehicleTrajectoryData(char fname[_MAX_PATH]);
int g_OutputSimulationSummary(float& AvgTravelTime, float& AvgDistance, float& AvgSpeed,float& AvgCost, EmissionStatisticsData &emission_data,
							  int InformationClass, int DemandType, int VehicleType, int DepartureTimeInterval);

void g_OutputLinkMOESummary(ofstream &LinkMOESummaryFile, int cut_off_volume=0);
void g_ExportLink3DLayerToKMLFiles(CString file_name, CString GISTypeString, int ColorCode, bool no_curve_flag, float height_ratio = 1);

void g_OutputLinkOutCapacitySummary();
void g_Output2WayLinkMOESummary(ofstream &LinkMOESummaryFile, int cut_off_volume=0);
void g_OutputSummaryKML(Traffic_MOE moe_mode);
extern CString g_GetAppRunningTime(bool with_title = true);
extern CString g_GetAppRunningTimePerIteration(bool with_title = true);



extern int g_output_OD_path_MOE_file;
extern int g_output_OD_TD_path_MOE_file;
extern int g_output_OD_path_MOE_cutoff_volume;
extern float g_OverallPerceptionErrorRatio;


extern float g_VMSPerceptionErrorRatio;
extern int g_information_updating_interval_in_min;
extern bool g_bInformationUpdatingAndReroutingFlag;
extern bool g_bVehicleAttributeUpdatingFlag;

extern int g_information_updating_interval_of_VMS_in_min;


extern void ConstructPathArrayForEachODT_ODEstimation(int,std::vector<PathArrayForEachODT> PathArray, int, int); // construct path array for each ODT
extern void g_UpdateLinkMOEDeviation_ODEstimation(NetworkLoadingOutput& output, int Iteration);
extern void g_OutputODMEResults();
extern void g_GenerateVehicleData_ODEstimation();

extern char g_GetLevelOfService(int PercentageOfSpeedLimit);
extern bool g_read_a_line(FILE* f, char* aline, int & size);
extern bool g_read_a_line(FILE* f);

std::string g_GetTimeStampStrFromIntervalNo(int time_interval);
extern CString g_GetTimeStampString(int time_stamp_in_mine);


extern void g_FreeMemoryForVehicleVector();

void g_AgentBasedShortestPathGeneration();
void g_SkimMatrixGenerationForAllDemandTypes(string file_name, bool bTimeDependentFlag, double CurrentTime);

extern bool g_ReadLinkMeasurementFile();
//extern void g_ReadObservedLinkMOEData(DTANetworkForSP* pPhysicalNetwork);

// for OD estimation
extern float    g_ODEstimation_WeightOnHistODDemand;
extern bool g_ObsDensityAvailableFlag;
extern bool g_ObsTravelTimeAvailableFlag;

extern float    g_ODEstimation_Weight_Flow;
extern float    g_ODEstimation_Weight_NumberOfVehicles;
extern float    g_ODEstimation_Weight_TravelTime;
extern float    g_ODEstimation_WeightOnUEGap;
extern float    g_ODEstimation_StepSize;

extern int g_ODEstimationFlag;
extern int g_SensorDataCount;
extern int g_ODEstimationMeasurementType;
extern int g_ODEstimation_StartingIteration;
extern int g_ODEstimation_EndingIteration;

extern bool IsWithinODMEIteration(int iteration);

extern float g_ODEstimation_max_ratio_deviation_wrt_hist_demand;

extern VehicleArrayForOriginDepartrureTimeInterval** g_TDOVehicleArray; // TDO for time-dependent origin;
extern std::vector<NetworkLoadingOutput>  g_AssignmentMOEVector;
extern std::vector<DTA_vhc_simple>   g_simple_vector_vehicles;

// for fast data acessing
extern int g_LastLoadedVehicleID; // scan vehicles to be loaded in a simulation interval

extern FILE* g_ErrorFile;
extern FILE* g_DebugLogFile;
extern ofstream g_LogFile;
extern CCSVWriter g_SummaryStatFile;
extern CCSVWriter g_MultiScenarioSummaryStatFile;
extern ofstream g_AssignmentLogFile;
extern ofstream g_NetworkDesignLogFile;
extern ofstream g_EstimationLogFile;
extern float g_LearningPercVector[1000];
void g_DTALiteMultiScenarioMain();

extern int g_InitializeLogFiles();
extern void g_ReadDTALiteSettings();
extern int g_AgentBasedAssignmentFlag;
extern float g_DemandGlobalMultiplier;
extern int g_ProhibitUTurnOnFeewayLinkFlag;

extern void g_TrafficAssignmentSimulation();
extern void g_OutputSimulationStatistics(int Iteration);
extern void g_FreeMemory(bool exit_flag);
extern void g_CloseFiles();

extern NetworkSimulationResult g_SimulationResult;
extern void g_RunStaticExcel();
extern TCHAR g_DTASettingFileName[_MAX_PATH];
extern void g_SetLinkAttributes(int usn, int dsn, int NumOfLanes);
extern void g_ReadInputFiles();
extern void g_ReadScenarioInputFiles(int scenario_no);
void  ReadIncidentScenarioFile(string FileName,int scenario_no=0);
void ReadVMSScenarioFile(string FileName,int scenario_no=0);
void ReadLinkTollScenarioFile(string FileName,int scenario_no=0);
void ReadRadioMessageScenarioFile(string FileName,int scenario_no=0);
void ReadWorkZoneScenarioFile(string FileName,int scenario_no=0);
void ReadGenericTrafficControlScenarioFile(string FileName,int scenario_no=0);


void ReadEvacuationScenarioFile(string FileName,int scenario_no=0);
void ReadWeatherScenarioFile(string FileName,int scenario_no=0);


void g_AgentBasedPathAdjustment(int DayNo, double CurrentTime );


void ReadLinkCapacityScenarioFile(string FileName,int scenario_no=0);
void ReadMovementCapacityScenarioFile(string FileName,int scenario_no=0);

extern void g_CreateLinkTollVector();
extern void g_ReadInputLinkTravelTime_Parser();
extern bool g_ReadTransitTripCSVFile();
extern void g_OutputDay2DayVehiclePathData(char fname[_MAX_PATH],int StartIteration,int EndIteration);

extern	int g_OutputSimulationMOESummary(float& AvgTravelTime, float& AvgDistance, float& AvgSpeed, float & AvgCost, EmissionStatisticsData &emission_data, LinkMOEStatisticsData &link_data,
										 int DemandType=0,int VehicleType = 0, int InformationClass=0, int origin_zone_id = 0, int destination_zone_id = 0,
										 int from_node_id = 0, int mid_node_id	=0, int to_node_id	=0,	
										 int departure_starting_time	 = 0,int departure_ending_time= 1440, int entrance_starting_time=0,int inentrance_ending_time = 1440);

extern std::vector<PathArrayForEachODTK> g_ODTKPathVector;
extern double g_UnitOfMileOrKM;
extern double g_gain_factor_link_travel_time_from_external_input;
extern void g_ConvertDemandToVehicles() ;

extern std::string CString2StdString(CString str);

extern void g_ResetVehicleAttributeUsingDemandType();


extern int g_number_of_CPU_threads();
extern int g_number_of_CPU_threads_for_real_time_routing();
extern void g_AllocateDynamicArrayForVehicles();
extern C_RealTimeSimulationSettings g_RealTimeSimulationSettings;
extern bool AddPathToVehicle(DTAVehicle * pVehicle, std::vector<int> path_node_sequence, CString FileName);

extern int g_NetworkDesignOptimalLinkSize;
extern float g_NetworkDesignTravelTimeBudget;

extern void g_SetupTDTollValue(int DayNo);

extern void g_BuildGridSystem();

//////////////////log file //////////////
extern FILE *g_simulation_log_file;
extern int g_simulation_log_level;

extern FILE *g_emission_log_file;
extern int g_emission_log_level;

void _proxy_simulation_log(int level, int simulation_interval_no, double simulation_in_min, int debug_agent_id, const char *fmt, ...);
void _proxy_emission_log(int level, int debug_agent_id, const char *fmt, ...);

void _proxy_ODME_log(int level, int iteration_no, const char *fmt, ...);
extern double g_simulation_time;
void _proxy_ABM_log(int level, const char *fmt, ...);
extern FILE* g_ODME_result_file;
extern int g_GetFreeMemoryDataInMB();
extern CString g_GetUsedMemoryDataInMB();
extern int g_InitialFreeMemory;
////////////////////////////////

//---------MODIFIED BY QU---------//
bool g_IsPathNodeSequenceAFeasiblePath(std::vector<int> path_node_sequence);
bool g_IsScheduleNodeAsServiceNode(std::vector<int> schedule_node_sequence);
bool g_IsServiceNodeInPathNode(std::vector<int> path_node_sequence, std::vector<int> schedule_node_sequence);
//--------------------------------//