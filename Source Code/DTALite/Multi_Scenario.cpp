//  Portions Copyright 2012 Xuesong Zhou

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
#include "Geometry.h"
#include "GlobalData.h"
#include "CSVParser.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
TCHAR g_real_time_simulation_setting_FileName[_MAX_PATH] = _T("./real_time_data_exchange/real_time_simulation_settings.txt");
using namespace std;

void g_WriteUserDefinedMOE(CCSVWriter  &g_MultiScenarioSummaryStatFile, int day_no = -1)
{
	
	if (day_no >= 1)
	{
		g_MultiScenarioSummaryStatFile.SetValueByFieldName("day_no", day_no);
	}
	else
	{
		g_MultiScenarioSummaryStatFile.SetValueByFieldName("day_no","");
	}


	CCSVParser parser_MOE_settings;
	if (parser_MOE_settings.OpenCSVFile("optional_MOE_settings.csv", false))
	{

		

		while (parser_MOE_settings.ReadRecord())
		{
			string moe_type, moe_category_label;

			int demand_type = 0;
			int vehicle_type = 0;
			int information_type = 0;
			int from_node_id = 0;
			int mid_node_id = 0;
			int to_node_id = 0;
			int origin_zone_id = 0;
			int destination_zone_id = 0;
			int departure_starting_time = 0;
			int departure_ending_time = 1440;
			int entrance_starting_time = 0;
			int entrance_ending_time = 1440;


			parser_MOE_settings.GetValueByFieldName("moe_type", moe_type);
			parser_MOE_settings.GetValueByFieldName("moe_category_label", moe_category_label);

			cout << " outputing MOE type " << moe_type << ", " << moe_category_label << endl;
			parser_MOE_settings.GetValueByFieldName("demand_type", demand_type);
			parser_MOE_settings.GetValueByFieldName("vehicle_type", vehicle_type);
			parser_MOE_settings.GetValueByFieldName("information_type", information_type);
			parser_MOE_settings.GetValueByFieldName("from_node_id", from_node_id);
			parser_MOE_settings.GetValueByFieldName("mid_node_id", mid_node_id);
			parser_MOE_settings.GetValueByFieldName("to_node_id", to_node_id);
			parser_MOE_settings.GetValueByFieldName("origin_zone_id", origin_zone_id);
			parser_MOE_settings.GetValueByFieldName("destination_zone_id", destination_zone_id);
			parser_MOE_settings.GetValueByFieldName("departure_starting_time_in_min", departure_starting_time);
			parser_MOE_settings.GetValueByFieldName("departure_ending_time_in_min", departure_ending_time);
			parser_MOE_settings.GetValueByFieldName("entrance_starting_time_in_min", entrance_starting_time);
			parser_MOE_settings.GetValueByFieldName("entrance_ending_time_in_min", entrance_ending_time);


			int Count = 0;
			float AvgTripTime, AvgDistance, AvgSpeed, AvgCost;
			EmissionStatisticsData emission_data;
			LinkMOEStatisticsData  link_data;
			Count = g_OutputSimulationMOESummary(AvgTripTime, AvgDistance, AvgSpeed, AvgCost, emission_data, link_data,
				demand_type, vehicle_type, information_type, origin_zone_id, destination_zone_id,
				from_node_id, mid_node_id, to_node_id,
				departure_starting_time, departure_ending_time, entrance_starting_time, entrance_ending_time);

			float percentage = Count*100.0f / max(1, g_SimulationResult.number_of_vehicles);

			int demand_type_no = demand_type - 1;

			if (demand_type_no < g_DemandTypeVector.size())
			{
			
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("percentage_" + g_DemandTypeVector[demand_type_no].demand_type_name, percentage);
			}
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("#_of_vehicles_" + moe_category_label, Count);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("percentage_" + moe_category_label, percentage);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_travel_time(min)_" + moe_category_label, AvgTripTime);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_distance_" + moe_category_label, AvgDistance);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_speed_" + moe_category_label, AvgSpeed);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_toll_cost_" + moe_category_label, AvgCost);


			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_energy_" + moe_category_label, emission_data.AvgEnergy);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_CO2_" + moe_category_label, emission_data.AvgCO2);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_NOX_" + moe_category_label, emission_data.AvgNOX);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_CO_" + moe_category_label, emission_data.AvgCO);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_HC_" + moe_category_label, emission_data.AvgHC);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_PM_" + moe_category_label, emission_data.AvgPM);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_PM2_5_" + moe_category_label, emission_data.AvgPM2_5);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("avg_MilesPerGallon_" + moe_category_label, emission_data.AvgMilesPerGallon);


			std::transform(moe_type.begin(), moe_type.end(), moe_type.begin(), ::tolower);

			if (moe_type.find("link") != string::npos) // Link MOE
			{
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("SOV_volume" + moe_category_label, link_data.SOV_volume);
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("HOV_volume" + moe_category_label, link_data.HOV_volume);
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("Truck_volume" + moe_category_label, link_data.Truck_volume);
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("Intermodal_volume" + moe_category_label, link_data.Intermodal_volume);

				g_MultiScenarioSummaryStatFile.SetValueByFieldName("number_of_crashes_per_year" + moe_category_label, link_data.number_of_crashes_per_year);
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("number_of_fatal_and_injury_crashes_per_year" + moe_category_label, link_data.number_of_fatal_and_injury_crashes_per_year);
				g_MultiScenarioSummaryStatFile.SetValueByFieldName("number_of_property_damage_only_crashes_per_year" + moe_category_label, link_data.number_of_property_damage_only_crashes_per_year);

			}


		}

	}  // input MOE settings

	g_MultiScenarioSummaryStatFile.WriteRecord();

}
void g_MultiScenarioTrafficAssignment()
{

	if (g_InitializeLogFiles() == 0)
		return;


	g_SummaryStatFile.Open("output_summary.csv");
	g_SummaryStatFile.WriteTextLabel("DTALite:\nA Fast Open Source DTA Engine\n");
	g_SummaryStatFile.WriteTextLabel("Software Version =,1.1.0\nRelease Date=,");
	g_SummaryStatFile.WriteTextLabel(__DATE__);
	g_SummaryStatFile.WriteTextLabel("\n");



	cout << "DTALite: A Fast Open-Source DTA Simulation Engine" << endl;
	cout << "Version 1.1.0, Release Date " << __DATE__ << "." << endl;

	g_LogFile << "---DTALite: A Fast Open-Source DTA Simulation Engine---" << endl;
	g_LogFile << "Version 1.1.0, Release Date " << __DATE__ << "." << endl;


	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	CString time_str;
	//`January', `February', `March', `April', `May', `June', `July', `August', `September', `October', `November', `December'. 
	time_str.Format("Simulation Date =,year: %d,month:%02d,day:%d,hour:%2d,min:%2d\n", now->tm_year + 1900, (now->tm_mon + 1), now->tm_mday, now->tm_hour, now->tm_min);

	g_SummaryStatFile.WriteTextLabel(time_str);

	int scenario_no;
	string scenario_name;
	int TotalUEIterationNumber = 5;

	int TestDemandLevel = 80;
	int TestFromNode = 0;
	int TestToNode = 0;
	float TestNumberOfLanes = 2;
	float LOVCost = 0;
	float HOVCost = 0;
	float TruckCost = 0;

	int NumberOfCriticalLinks = 3;
	int NumberOfCriticalODPairs = 3;

	g_MultiScenarioSummaryStatFile.Open("output_day_to_day_MOE.csv");
	g_MultiScenarioSummaryStatFile.WriteTextString("Unit of output:");
	g_MultiScenarioSummaryStatFile.WriteTextString(",,distance=,miles");
	g_MultiScenarioSummaryStatFile.WriteTextString(",,speed=,mph");
	g_MultiScenarioSummaryStatFile.WriteTextString(",,energy=,1000 joule");
	g_MultiScenarioSummaryStatFile.WriteTextString(",,CO2,NOX,CO,HC,PM,PM2.5=,g");

	int cl;

	g_MultiScenarioSummaryStatFile.SetFieldName("scenario_no");
	g_MultiScenarioSummaryStatFile.SetFieldName("scenario_name");
	g_MultiScenarioSummaryStatFile.SetFieldName("scenario_data");
	g_MultiScenarioSummaryStatFile.SetFieldName("number_of_iterations");
	g_MultiScenarioSummaryStatFile.SetFieldName("day_no");
	g_MultiScenarioSummaryStatFile.SetFieldName("traffic_flow_model");


	CCSVParser parser_MOE_settings;
	if (parser_MOE_settings.OpenCSVFile("optional_MOE_settings.csv", false))
	{
		while (parser_MOE_settings.ReadRecord())
		{
			string moe_type, moe_category_label;

			int demand_type = 0;
			int vehicle_type = 0;
			int information_type = 0;
			int from_node_id = 0;
			int mid_node_id = 0;
			int to_node_id = 0;
			int origin_zone_id = 0;
			int destination_zone_id = 0;
			int departure_starting_time = 0;
			int departure_ending_time = 1440;
			int entrance_starting_time = 0;
			int entrance_ending_time = 1440;


			parser_MOE_settings.GetValueByFieldName("moe_type", moe_type);
			parser_MOE_settings.GetValueByFieldName("moe_category_label", moe_category_label);

			TRACE("%s\n", moe_category_label.c_str());
			if (moe_category_label.find(",") != string::npos)
			{
				moe_category_label = '"' + moe_category_label + '"';
			}


			g_MultiScenarioSummaryStatFile.SetFieldNameWithCategoryName("#_of_vehicles_" + moe_category_label, moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("percentage_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_distance_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_travel_time(min)_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_speed_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_toll_cost_" + moe_category_label);

			g_MultiScenarioSummaryStatFile.SetFieldName("avg_energy_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_CO2_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_NOX_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_CO_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_HC_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_PM_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_PM2_5_" + moe_category_label);
			g_MultiScenarioSummaryStatFile.SetFieldName("avg_MilesPerGallon_" + moe_category_label);

			std::transform(moe_type.begin(), moe_type.end(), moe_type.begin(), ::tolower);

			if (moe_type.find("link") != string::npos)  // Link MOE
			{
				//			g_MultiScenarioSummaryStatFile.SetFieldName("level_of_service"+moe_category_label);

				g_MultiScenarioSummaryStatFile.SetFieldName("SOV_volume" + moe_category_label);
				g_MultiScenarioSummaryStatFile.SetFieldName("HOV_volume" + moe_category_label);
				g_MultiScenarioSummaryStatFile.SetFieldName("Truck_volume" + moe_category_label);
				g_MultiScenarioSummaryStatFile.SetFieldName("Intermodal_volume" + moe_category_label);

				g_MultiScenarioSummaryStatFile.SetFieldName("number_of_crashes_per_year" + moe_category_label);
				g_MultiScenarioSummaryStatFile.SetFieldName("number_of_fatal_and_injury_crashes_per_year" + moe_category_label);
				g_MultiScenarioSummaryStatFile.SetFieldName("number_of_property_damage_only_crashes_per_year" + moe_category_label);

			}

		}


		parser_MOE_settings.CloseCSVFile();
	}


	g_MultiScenarioSummaryStatFile.WriteHeader(true, false);


	int total_scenarios = 0;
	{
		CCSVParser parser_sce;
		if (parser_sce.OpenCSVFile("input_scenario_settings.csv"))
		{
			while (parser_sce.ReadRecord())
			{
				total_scenarios++;
			}
		}
	}
	CCSVParser parser_scenario;
	int line_no = 1;

	if (parser_scenario.OpenCSVFile("input_scenario_settings.csv"))
	{

		for (int day = 0; day < 1000; day++)
		{

			g_LearningPercVector[day] = g_LearningPercentage;
		}


		g_ODEstimationFlag = 0; 			// no OD estimation

		int max_scenarios = 50000;

		while (parser_scenario.ReadRecord())
		{
			if (line_no >= max_scenarios)
				break;
			if (parser_scenario.GetValueByFieldNameWithPrintOut("scenario_no", scenario_no) == false)
			{
				cout << "Field scenario_no cannot be found in file input_scenario_settings.csv. Please check." << endl;
				g_ProgramStop();
			}

			if (parser_scenario.GetValueByFieldNameWithPrintOut("scenario_name", scenario_name) == false)
			{
				cout << "Field scenario_name cannot be found in file input_scenario_settings.csv. Please check." << endl;
				g_ProgramStop();
			}

			parser_scenario.GetValueByFieldNameWithPrintOut("random_seed", g_RandomSeed);


			g_SummaryStatFile.WriteTextLabel("----------------------");
			g_SummaryStatFile.WriteTextLabel(scenario_name.c_str());
			g_SummaryStatFile.WriteTextLabel("----------------------\n");

			g_VehicleLoadingMode = demand_matrix_file_mode;  // default meta data mode

			if (parser_scenario.GetValueByFieldName("number_of_assignment_days", TotalUEIterationNumber) == false)
			{
				if (parser_scenario.GetValueByFieldName("number_of_iterations", TotalUEIterationNumber) == false)
				{
					cout << "Field number_of_iterations cannot be found in file input_scenario_settings.csv. Please check." << endl;
					g_ProgramStop();
				}
			}

			g_NumberOfIterations = TotalUEIterationNumber - 1;			// 0+1 iterations

			int traffic_flow_model = 1;
			if (parser_scenario.GetValueByFieldNameWithPrintOut("traffic_flow_model", traffic_flow_model) == false)
			{
				cout << "Field traffic_flow_model cannot be found in file input_scenario_settings.csv. Please check." << endl;
				g_ProgramStop();
			}

			int SignalRepresentationFlag = 0;

			parser_scenario.GetValueByFieldName("signal_representation_model", SignalRepresentationFlag);

			g_SignalRepresentationFlag = (e_signal_representation_model)SignalRepresentationFlag;

			g_SummaryStatFile.WriteTextLabel("Signal Control Representation =,");

			switch (g_SignalRepresentationFlag)
			{

			case signal_model_continuous_flow: 		g_LogFile << "Continuous Flow with Link Capacity Constrai" << endl;
				g_SummaryStatFile.WriteTextString("Continuous Flow with Link Capacity Constraint");
				break;

			case signal_model_link_effective_green_time: 		g_LogFile << "Cycle Length + Link-based Effective Green Time" << endl;
				g_SummaryStatFile.WriteTextString("Cycle Length + Link-based Effective Green Time");
				break;


				break;


			default: 		g_LogFile << "No Valid Model is Selected" << endl;
				g_SummaryStatFile.WriteTextString("Invalid Model");
				break;
			}



			g_TrafficFlowModelFlag = (e_traffic_flow_model)traffic_flow_model;

			g_UEAssignmentMethod = analysis_day_to_day_learning_threshold_route_choice;
			int UEAssignmentMethod = 0;
			if (parser_scenario.GetValueByFieldName("traffic_analysis_method", UEAssignmentMethod) == false)
			{
				if (parser_scenario.GetValueByFieldName("traffic_assignment_method", UEAssignmentMethod) == false)
				{

					cout << "Field traffic_analysis_method has not been specified in file input_scenario_settings.csv. A default method of day-to-day learning is used." << endl;
					getchar();
				}
			}


			g_UEAssignmentMethod = (e_analysis_method)UEAssignmentMethod;

			g_CalculateUEGapForAllAgents = 1;

			g_LogFile << "Traffic Flow Model =  ";
			g_SummaryStatFile.WriteTextLabel("Traffic Flow Model =,");

			switch (g_TrafficFlowModelFlag)
			{
			case tfm_BPR: 		g_LogFile << "BPR Function" << endl;
				g_SummaryStatFile.WriteTextString("BPR Function");
				break;

			case tfm_point_queue: 		g_LogFile << "Point Queue Model" << endl;
				g_SummaryStatFile.WriteTextString("Point Queue Model");
				break;

			case tfm_spatial_queue: 		g_LogFile << "Spatial Queue Model" << endl;
				g_SummaryStatFile.WriteTextString("Spatial Queue Model");
				break;

			case tfm_newells_model: 		g_LogFile << "Newell's Cumulative Flow Count Model" << endl;
				g_SummaryStatFile.WriteTextString("Newell's Cumulative Flow Count Model");
				break;

				traffic_flow_model = tfm_newells_model;  // newell's model
				g_EmissionDataOutputFlag = 2;  // with emission data

				break;


			default: 		g_LogFile << "No Valid Model is Selected" << endl;
				g_SummaryStatFile.WriteTextString("Invalid Model");
				break;
			}


			if (g_UEAssignmentMethod == analysis_day_to_day_learning_threshold_route_choice)
			{
				//if (parser_scenario.GetValueByFieldName("switching_percentage_iterations_1", g_LearningPercVector[1]) == false)
				//	g_LearningPercVector[1] = g_LearningPercentage;

				//if (TotalUEIterationNumber >= 1000)
				//{
				//	cout << "Too many iterations/days. Please contact the developer at xzhou99@gmail.com." << endl;
				//	g_ProgramStop();
				//}

				//for (int day = 2; day <= TotalUEIterationNumber; day++)
				//{
				//	CString str_learning;
				//	str_learning.Format("iteration_%d", day);

				//	string str = CString2StdString(str_learning);
				//	if (parser_scenario.GetValueByFieldName(str, g_LearningPercVector[day]) == false)  // no data
				//		g_LearningPercVector[day] = g_LearningPercentage;

				//	if (g_LearningPercVector[day] > 100)
				//		g_LearningPercVector[day] = 100;

				//	if (g_LearningPercVector[day] < 0)
				//		g_LearningPercVector[day] = 0;
				//}



			}

			g_ODEstimationFlag = 0;

			CCSVParser parser_RTSimulation_settings;

			switch (g_UEAssignmentMethod)
			{
			case analysis_MSA:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "MSA");
				break;
			case analysis_ABM_integration:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "ABM+DTA integration");
				break;
			case analysis_gap_function_MSA_step_size:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "Gap-funciton with step size based adjustment");
				break;

			case analysis_accessibility_distance:
				g_SummaryStatFile.WriteParameterValue("Routing method", "Assessibility based on distance");
				break;

			case analysis_accessibility_travel_time:
				g_SummaryStatFile.WriteParameterValue("Routing method", "Assessibility based on travel time");
				break;

			case analysis_OD_demand_estimation:
				g_SummaryStatFile.WriteParameterValue("Routing method", "OD demand estimation");
				g_AgentBasedAssignmentFlag = 0; // zone based mode only
				//g_UEAssignmentMethod = analysis_MSA; // default assignment mode to MSA, use ODME_step_size after running ODME
				g_ODEstimationFlag = 1; 
				break;

			case analysis_vehicle_binary_file_based_scenario_evaluation:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "Load binary agent file with demand type definition from scenarios files: Scenario_Demand_Type.csv, Scenario_Vehicle_Type.csv, Scenario_VOT.csv.");

				g_VehicleLoadingMode = vehicle_binary_file_mode;
				break;


			case analysis_real_time_simulation:


				if (g_use_routing_policy_from_external_input == 1)
				{
					g_SummaryStatFile.WriteParameterValue("Assignment method", "Real time simulation with automatically generated routing policy");
				}
				else
				{
					g_SummaryStatFile.WriteParameterValue("Assignment method", "Real time simulation without with automatically generated routing policy");
				}


				if (g_UEAssignmentMethod == analysis_real_time_simulation)
				{
					g_DemandLoadingStartTimeInMin = 0;
					g_DemandLoadingEndTimeInMin = 1440;
					g_PlanningHorizon = 1440;
				}

				break;

			case analysis_integration_with_AGENT_PLUS:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "Perform real time simulation with Agent + model. Required files: ____");
				break;

			case analysis_system_optimal:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "System Optimal for all agents: Based on agent binary file.");

				g_VehicleLoadingMode = vehicle_binary_file_mode;
				break;

			case analysis_LR_agent_based_system_optimization:
				g_SummaryStatFile.WriteParameterValue("Assignment method", "Lagrangian-relaxation based system optimization.");
				break;
		default:
				g_UEAssignmentMethod = analysis_MSA; 

			}



			g_ReadRealTimeSimulationSettingsFile();


			if (g_UEAssignmentMethod == analysis_real_time_simulation)
			{


				g_RealTimeSimulationSettings.synchronization_sleep_time_interval_in_second
					= g_GetPrivateProfileInt("synchronization", "sleep_time_interval_in_second ", 1, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.input_link_attribute_generated_from_external_program
					= g_GetPrivateProfileInt("input_link_attribute", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.input_link_attribute_updating_time_interval_in_second
					= g_GetPrivateProfileInt("input_link_attribute", "updating_time_interval_in_second", 6, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.input_trip_generated_from_external_program
					= g_GetPrivateProfileInt("input_trip", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.input_trip_updating_time_interval_in_min
					= g_GetPrivateProfileInt("input_trip", "updating_time_interval_in_min", 1, g_real_time_simulation_setting_FileName);


				g_RealTimeSimulationSettings.output_link_performance_generated_to_external_program
					= g_GetPrivateProfileInt("output_link_performance", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_link_performance_updating_time_interval_in_min
					= g_GetPrivateProfileInt("output_link_performance", "updating_time_interval_in_min", 1, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_trip_generated_to_external_program
					= g_GetPrivateProfileInt("output_trip", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_trip_updating_time_interval_in_min
					= g_GetPrivateProfileInt("output_trip", "updating_time_interval_in_min", 1, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_travel_cost_skim_generated_to_external_program
					= g_GetPrivateProfileInt("output_travel_cost_skim", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_travel_cost_skim_updating_time_interval_in_min
					= g_GetPrivateProfileInt("output_travel_cost_skim", "updating_time_interval_in_min", 1, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_routing_policy_generated_to_external_program
					= g_GetPrivateProfileInt("output_trip", "generated_from_external_program", 0, g_real_time_simulation_setting_FileName);

				g_RealTimeSimulationSettings.output_routing_policy_updating_time_interval_in_min
					= g_GetPrivateProfileInt("output_trip", "updating_time_interval_in_min", 1, g_real_time_simulation_setting_FileName);

			}



			g_SummaryStatFile.WriteTextString(" ");


			g_ValidationDataStartTimeInMin = 11440;
			g_ValidationDataEndTimeInMin = 0;
			g_ODEstimation_StartingIteration = 1000;
			g_ODEstimation_EndingIteration = 1000;
			g_ODEstimation_max_ratio_deviation_wrt_hist_demand = 0.20f;


			if (g_ODEstimationFlag == 1)
			{
				if (parser_scenario.GetValueByFieldName("ODME_start_iteration", g_ODEstimation_StartingIteration) == false)
				{
					g_ODEstimation_StartingIteration = 20;
					cout << "Field ODME_start_iteration has not been specified in file input_scenario_settings.csv. A default factor of 20 is used." << endl;
					getchar();
				}

				if (parser_scenario.GetValueByFieldName("ODME_end_iteration", g_ODEstimation_EndingIteration) == false)
				{
					g_ODEstimation_StartingIteration = g_ODEstimation_StartingIteration+1000;
				}

				if (g_ODEstimation_EndingIteration <= g_ODEstimation_StartingIteration )
				{
					g_ODEstimation_EndingIteration = TotalUEIterationNumber;
				}

				float ODEstimation_max_percentage_deviation_wrt_hist_demand = 20;
				if (parser_scenario.GetValueByFieldName("ODME_max_percentage_deviation_wrt_hist_demand", ODEstimation_max_percentage_deviation_wrt_hist_demand) == false)
				{


					cout << "Field ODME_max_percentage_deviation_wrt_hist_demand has not been specified in file input_scenario_settings.csv. A default value of 30 (%) is used." << endl;
					getchar();
				}

				g_ODEstimation_max_ratio_deviation_wrt_hist_demand = ODEstimation_max_percentage_deviation_wrt_hist_demand / 100.0;
				g_ODEstimation_StepSize = 0.05f;
				if (parser_scenario.GetValueByFieldName("ODME_step_size", g_ODEstimation_StepSize) == false)
				{
					cout << "Field ODME_step_size has not been specified in file input_scenario_settings.csv. A default value of 0.05 is used." << endl;
					getchar();
				}

				if (g_ODEstimation_StepSize < 0 || g_ODEstimation_StepSize >= 1)
				{
					cout << "Field ODME_step_size =" << g_ODEstimation_StepSize << ", which should be between 0 and 1" << endl;
					getchar();
					g_ODEstimation_StepSize = 0.2f;

				}

			}



			string File_Link_Based_Toll, File_Incident, File_MessageSign, File_WorkZone;

			if (line_no == 1)  // read it once
			{
				g_ReadInputFiles();


			}

			g_ReadScenarioInputFiles(scenario_no);

			cout << "Start Traffic Assignment/Simulation... " << endl;

			cout << "Agent based dynamic traffic assignment... " << endl;


			if (g_UEAssignmentMethod == analysis_LR_agent_based_system_optimization)  // 14
			{
				g_SummaryStatFile.WriteParameterValue("Routing method", "Lagrangian relaxation based, agent-based routing");
				g_AgentBasedOptimization();

			}
			else
			{

				if (g_AgentBasedAssignmentFlag == 1)
				{
					g_SummaryStatFile.WriteParameterValue("Routing method", "Individual agent-based routing");
					g_AgentBasedDynamicTrafficAssignmentSimulation();  // agent-based assignment
				}
				else
				{
					g_SummaryStatFile.WriteParameterValue("Assignment method", "Zone-based routing");
					g_ZoneBasedDynamicTrafficAssignmentSimulation(); // multi-iteration dynamic traffic assignment

				}


			}

			g_SummaryStatFile.WriteTextLabel("\n")
				;
			g_OutputSimulationStatistics(g_NumberOfIterations);

			g_MultiScenarioSummaryStatFile.SetValueByFieldName("scenario_no", scenario_no);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("scenario_name", scenario_name);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("number_of_iterations", TotalUEIterationNumber);
			g_MultiScenarioSummaryStatFile.SetValueByFieldName("demand_multiplier", g_DemandGlobalMultiplier);

#ifdef _large_memory_usage_lr
			g_SummaryStatFile.WriteParameterValue("Memory use mode", "large memory for demand-dependent data");
#endif 		

			g_MultiScenarioSummaryStatFile.SetValueByFieldName("traffic_flow_model", traffic_flow_model);



			///
			g_WriteUserDefinedMOE(g_MultiScenarioSummaryStatFile);
			/// 

			line_no++;
		}  // for each scenario

		g_MultiScenarioSummaryStatFile.WriteTextLabel(g_GetAppRunningTime());
		parser_MOE_settings.CloseCSVFile();

	}
	else
	{

		cout << "File input_scenario_settings.csv cannot be found. Please check." << endl;
		g_ProgramStop();

	}

	g_FreeMemory(line_no == total_scenarios);  // free memory at the end of multiple scenarios
	exit(0);
}

void g_DTALiteMultiScenarioMain()
{

	g_MultiScenarioTrafficAssignment();

}
