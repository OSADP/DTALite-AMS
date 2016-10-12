// Portions Copyright 2010 Xuesong Zhou

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

//shortest path calculation

// note that the current implementation is only suitable for time-dependent minimum time shortest path on FIFO network, rather than time-dependent minimum cost shortest path
// the key reference (1) Shortest Path Algorithms in Transportation Models http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.51.5192
// (2) most efficient time-dependent minimum cost shortest path algorithm for all departure times
// Time-dependent, shortest-path algorithm for real-time intelligent vehicle highway system applications&quot;, Transportation Research Record 1408 ?Ziliaskopoulos, Mahmassani - 1993

#include "stdafx.h"
#include "DTALite.h"
#include "GlobalData.h"
int g_path_error = 0;

int g_FindAssignmentIntervalIndexFromTime(float time_in_min)
{

	int return_value = 0;
	if (time_in_min < g_DemandLoadingStartTimeInMin)
		return 0;
	else if (time_in_min < g_PlanningHorizon)
	{
		int interval_with_15_min = max(0, int(time_in_min+0.1) / 15);
	
		return_value = g_AssignmentIntervalIndex[interval_with_15_min];
	}
	else
	{
		return_value =  max(0, g_NumberOfSPCalculationPeriods - 1);
	}

	return min(return_value, g_NumberOfSPCalculationPeriods - 1);
}

int g_FindAssignmentIntervalLengthInMinFromTime(float time_in_min)
{
	int time_interval_no = g_FindAssignmentIntervalIndexFromTime(time_in_min);

	int time_interval_length_in_min = (int)(g_AssignmentIntervalEndTimeInMin[time_interval_no] - g_AssignmentIntervalStartTimeInMin[time_interval_no]);

	return max(15, time_interval_length_in_min);

}
void g_SetupTDTollValue(int DayNo)
{

	unsigned li;
	for (li = 0; li< g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		// ECOSO 2016
		for (int t = g_DemandLoadingStartTimeInMin; t < g_PlanningHorizon; t += g_AggregationTimetInterval)
		{
			int		link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(t);
			int LinkInFlow = pLink->GetArrivalFlow(t);

			if(g_LinkTDCostAry!=NULL)
			{
			g_LinkTDCostAry[pLink->m_LinkNo][link_entering_time_interval].TollValue[0] = pLink->m_LinkMOEAry[t].SystemOptimalMarginalCost;  // this is average energy per vehicle for using this link at time t
			 }
		}


		if (pLink->TollVector.size() == 0)
			continue;

		for (int t = g_DemandLoadingStartTimeInMin; t < g_PlanningHorizon; t += g_AggregationTimetInterval)
		{
			int		link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(t);

			// copy pricing type dependent link toll values
			for (int itoll = 0; itoll < pLink->TollVector.size(); itoll++)
			{


				if ((DayNo >= pLink->TollVector[itoll].StartDayNo && DayNo <= pLink->TollVector[itoll].EndDayNo)
					&& t >= pLink->TollVector[itoll].StartTime && t <= pLink->TollVector[itoll].EndTime)
				{
					g_LinkTDCostAry[pLink->m_LinkNo][link_entering_time_interval].m_bMonetaryTollExist = true;


					for (int demand_type = 1; demand_type <=g_DemandTypeVector.size(); demand_type++)
					{
						if (pLink->TollVector[itoll].TollRate[demand_type] >= 0.00001)
						{
						
						g_LinkTDCostAry[pLink->m_LinkNo][link_entering_time_interval].TollValue[demand_type] = pLink->TollVector[itoll].TollRate[demand_type];
						}
					}
				}
			}
		}


	
		}

}
void DTANetworkForSP::BuildNetworkBasedOnZoneCentriod(int DayNo,int CurZoneID)  // build the network for shortest path calculation and fetch travel time and cost data from simulator
{
	// build a network from the current zone centriod (1 centriod here) to all the other zones' centriods (all the zones)
	std::set<DTANode*>::iterator iterNode;
	std::set<DTALink*>::iterator iterLink;

	m_PhysicalNodeSize = g_NodeVector.size();

	int IntervalLinkID=0;
	int FromID, ToID;

	unsigned int i;
	int t;

	for (i = 0; i< m_PhysicalNodeSize + g_ODZoneIDSize + 1; i++)
	{
		m_OutboundSizeAry[i] = 0;
		m_InboundSizeAry[i] =0;
	}
	// step 1: build physical network: 
//	cout << " step 1: build physical network " << endl;
	BuildPhysicalNetwork(DayNo, CurZoneID, g_TrafficFlowModelFlag);

	int LinkID = g_LinkVector.size();
//	cout << "  step 2: add outgoing connectors from origin zone center(m_PhysicalNodeSize)  " << endl;

	// step 2: add outgoing connectors from origin zone center(m_PhysicalNodeSize) to zone centriods
	for(i = 0; i< g_ZoneMap[CurZoneID].m_OriginActivityVector.size(); i++)
	{
		FromID = m_PhysicalNodeSize; // m_PhysicalNodeSize is the centriod number for CurZoneNo
		ToID = g_ZoneMap[CurZoneID].m_OriginActivityVector[i];
		// add outcoming connector from the centriod corresponding to the current zone: node ID = m_PhysicalNodeSize, to this physical node's ID

		//         TRACE("destination node of current zone %d: %d\n",CurZoneID, g_NodeVector[ToID]);

		m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
		m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = LinkID;

		//m_OutboundConnectorOriginZoneIDAry[FromID][m_OutboundSizeAry[FromID]] = LinkID;
		m_OutboundSizeAry[FromID] +=1;

		m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = LinkID ;
		m_InboundSizeAry[ToID] +=1;

		m_FromIDAry[LinkID] = FromID;
		m_ToIDAry[LinkID] = ToID;


		ASSERT(g_AdjLinkSize >= m_OutboundSizeAry[FromID]);

		for(int ti= 0; ti < m_NumberOfSPCalculationIntervals; ti+=1)
		{
			m_LinkTDTimeAry[LinkID][ti] = 0.01;

			// copy pricing type dependent link toll values

		}


		// construct outbound movement vector
		for(int movement = 0; movement < m_OutboundSizeAry[ToID]; movement++)
		{
			int outbound_link = m_OutboundLinkAry[ToID][movement];

			m_OutboundMovementAry[LinkID][movement] = outbound_link;
			m_OutboundMovementDelayAry[LinkID][movement] = 0;   // we need to add time-dependent movement delay here
			m_OutboundMovementSizeAry[LinkID]++;

		}

		//end of  constructing outbound movement vector
		LinkID++;
	}


//	cout << " step 3: add incoming connector to destination zone " << endl;

	// step 3: add incoming connector to destination zone which is not CurZoneNo connector from the centriod corresponding to the current zone: node ID = m_PhysicalNodeSize, to this physical node's ID
	std::map<int, DTAZone>::iterator iterZone;
	for (iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	{
		DTAZone zone = iterZone->second ;

		if(iterZone->first != CurZoneID)  // only this origin zone has vehicles, then we build the network
		{
			for(i = 0; i< zone.m_DestinationActivityVector .size(); i++)
			{
				FromID = zone.m_DestinationActivityVector[i];; // m_PhysicalNodeSize is the centriod number for CurZoneNo
				ToID = m_PhysicalNodeSize + 1+  zone.m_ZoneSequentialNo; // m_PhysicalNodeSize is the centriod number for CurZoneNo, note that  .m_ZoneSequentialNo start from 0

				m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
				m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = LinkID;
				m_OutboundSizeAry[FromID] +=1;
				
				
				m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = LinkID ;
				m_InboundSizeAry[ToID] +=1;


				ASSERT(g_AdjLinkSize >  m_OutboundSizeAry[FromID]);

				m_LinkTDDistanceAry[LinkID] = 0.01;
				m_LinkFFTTAry[LinkID] = 0.01;

				m_FromIDAry[LinkID] = FromID;
				m_ToIDAry[LinkID] = ToID;


				// for each incoming link at from node id, add outbound link of the connector to its outbound movement 

				for (int k = 0; k < m_InboundSizeAry[FromID]; k++)
				{
					int incoming_link_id = m_InboundLinkAry[FromID][k];

					m_OutboundMovementAry[incoming_link_id][m_OutboundMovementSizeAry[incoming_link_id]] = LinkID;
					m_OutboundMovementDelayAry[incoming_link_id][m_OutboundMovementSizeAry[incoming_link_id]] = 0;

					m_OutboundMovementSizeAry[incoming_link_id]++;
		

				}


				if (m_LinkTDTimeAry != NULL)
				{
				
				for( int ti= 0; ti <m_NumberOfSPCalculationIntervals; ti+=1)
				{
					m_LinkTDTimeAry[LinkID][ti] = 0.01;
				}
				}
				if (m_LinkTDTransitTimeAry != NULL)
				{
				
				for (int ti = 0; ti <m_NumberOfSPCalculationIntervals; ti += 1)
				{
					m_LinkTDTransitTimeAry[LinkID][ti] = 0;
				}
				}

				// construct outbound movement vector: no outgoing movement 
				m_OutboundMovementSizeAry[LinkID] = 0;
				LinkID++;
			}
		}
	}
	m_NodeSize = m_PhysicalNodeSize + 1 + g_ODZoneIDSize;
	m_LinkSize = LinkID;
}

void DTANetworkForSP::BuildPhysicalNetwork(int DayNo, int CurrentZoneNo, e_traffic_flow_model TraffcModelFlag, bool bUseCurrentInformation , double CurrentTime)  // for agent based 
{

	//CurrentZoneNo >=0: called by zone based assignment 

	if(m_OutboundSizeAry ==NULL)
		return;

	bool bDebug = true;

	std::set<DTANode*>::iterator iterNode;
	std::set<DTALink*>::iterator iterLink;

	m_NodeSize = g_NodeVector.size();

	int IntervalLinkID=0;
	int FromID, ToID;

	int i,t;

	for(i=0; i< m_NodeSize; i++)
	{
		m_OutboundSizeAry[i] = 0;
		m_InboundSizeAry[i] = 0;
	}

	for(i=0; i< m_LinkSize; i++)
	{
		m_OutboundMovementSizeAry[i] = 0;
	}

	// add physical links
	//cout << "add physical links " << endl;

		unsigned li;
		for(li = 0; li< g_LinkVector.size(); li++)
		{
			DTALink* pLink = g_LinkVector[li];
		FromID = pLink->m_FromNodeID;
		ToID   = pLink->m_ToNodeID;


	//	cout << "add link no." << li <<  endl;

			if( pLink->m_FromNodeNumber == 6 && pLink->m_ToNodeNumber == 7)
			{
			TRACE("");
			}

		m_FromIDAry[pLink->m_LinkNo] = FromID;
		m_ToIDAry[pLink->m_LinkNo]   = ToID;

		m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
		m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = pLink->m_LinkNo ;

		int link_id = pLink->m_LinkNo ;


		if(g_LinkTypeMap[g_LinkVector[link_id]->m_link_type].IsConnector())
		{
			m_LinkConnectorFlag[link_id] = 1 ;
			m_OutboundConnectorOriginZoneIDAry[FromID][m_OutboundSizeAry[FromID]] = g_NodeVector[g_LinkVector[link_id]->m_FromNodeID ].m_ZoneID ;
			m_OutboundConnectorDestinationZoneIDAry[FromID][m_OutboundSizeAry[FromID]] = g_NodeVector[g_LinkVector[link_id]->m_ToNodeID ].m_ZoneID ;
			m_LinkConnectorOriginZoneIDAry[link_id] = g_NodeVector[g_LinkVector[link_id]->m_FromNodeID ].m_ZoneID ;
			m_LinkConnectorDestinationZoneIDAry [link_id] = g_NodeVector[g_LinkVector[link_id]->m_ToNodeID ].m_ZoneID ;

	
		}else
		{
			m_LinkConnectorFlag[link_id] = 0 ;
			m_OutboundConnectorOriginZoneIDAry[FromID][m_OutboundSizeAry[FromID]]  = -1; // default values
			m_OutboundConnectorDestinationZoneIDAry[FromID][m_OutboundSizeAry[FromID]]  = -1; // default values
			m_LinkConnectorOriginZoneIDAry[link_id] = -1;
			m_LinkConnectorDestinationZoneIDAry [link_id] = -1 ;
		
		}

		m_OutboundSizeAry[FromID] +=1;

		m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = pLink->m_LinkNo  ;
		m_InboundSizeAry[ToID] +=1;

		if(g_AdjLinkSize <= m_OutboundSizeAry[FromID])
		{
			cout << "node " <<  g_NodeVector[FromID].m_NodeNumber  << " have more than " << m_OutboundSizeAry[FromID] << " outbound links. Please check." << endl;
	
			g_ProgramStop();
		}

		m_LinkTDDistanceAry[pLink->m_LinkNo] = pLink->m_Length ;
		m_LinkFFTTAry[pLink->m_LinkNo] = pLink->m_FreeFlowTravelTime  ; 


		// set time-dependent link travel time, cost, transit values 
		int link_entering_time_interval;

		//initialization
		for(int ti = 0; ti < m_NumberOfSPCalculationIntervals; ti += 1)
		{
		m_LinkTDTimeAry[pLink->m_LinkNo][ti]=0.01;
		}
		

		for (t = m_StartTimeInMin; t < m_PlanningHorizonInMin; t += g_FindAssignmentIntervalLengthInMinFromTime(t))
		{
			link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(t);
			int aggregation_time_interval_in_min = g_FindAssignmentIntervalLengthInMinFromTime(t);

			// we obtain simulated time-dependent travel time measurments from simulator, use that for time-dependent shortest path calculation
			float AvgTravelTime = pLink->GetTravelTimeByMin(DayNo, t, aggregation_time_interval_in_min, TraffcModelFlag);

			if(bUseCurrentInformation == true) // use travel information
			{
				AvgTravelTime = pLink->GetPrevailingTravelTime (DayNo, CurrentTime);
			
			}


			// use predicted travel time from user definded data file
			//

			if (pLink->m_LinkMOEAry[t].UserDefinedTravelTime_in_min >= 0.1 && DayNo==0)  // with valid data
			{
			
				AvgTravelTime = pLink->m_LinkMOEAry[t].UserDefinedTravelTime_in_min;
			}

			AvgTravelTime*=g_LinkTypeMap[pLink->m_link_type ].link_type_bias_factor;

			if(AvgTravelTime < 0.01f)  // to avoid possible loops
				AvgTravelTime = 0.01f ;

			ASSERT(AvgTravelTime < 99999);

			if(bDebug )
			{
				if(g_NodeVector[FromID].m_NodeNumber == 31 && g_NodeVector[ToID].m_NodeNumber ==32  ) 
				TRACE("FromID %d -> ToID %d, time: %d, %f\n", g_NodeVector[FromID].m_NodeNumber, g_NodeVector[ToID].m_NodeNumber, t,AvgTravelTime );
			}

			m_LinkTDTimeAry[pLink->m_LinkNo][link_entering_time_interval] = AvgTravelTime;


		}

	}

//	cout << "construct outbound movement vector " << endl;

	// construct outbound movement vector
	for(li = 0; li< g_LinkVector.size(); li++)
	{
		// if CurrentZoneNo ==- 1, we do not run the following line (to skip zone outgoing connectors)
		if( CurrentZoneNo >=0 && g_LinkTypeMap[g_LinkVector[li]->m_link_type].IsConnector() 
			&& g_NodeVector[g_LinkVector[li]->m_FromNodeID ].m_ZoneID != CurrentZoneNo) // only for non connector-links
			continue;

		//find downstream node
		ToID   = g_LinkVector[li]->m_ToNodeID;
		if(g_NodeVector[ToID].m_MovementMap.size()==0)  //without movement capacity input
		{
			ASSERT(m_AdjLinkSize >= m_OutboundSizeAry[ToID]);

			for(int movement = 0; movement < m_OutboundSizeAry[ToID]; movement++)
			{

				int outbound_link = m_OutboundLinkAry[ToID][movement];
				m_OutboundMovementAry[li][movement] = outbound_link;
				m_OutboundMovementDelayAry[li][movement] = 0;   // we need to add time-dependent movement delay here
				m_OutboundMovementSizeAry[li]++;
			}

		}else //with movement capacity input	
		{
			for(int movement = 0; movement < m_OutboundSizeAry[ToID]; movement++)
			{
				int outbound_link = m_OutboundLinkAry[ToID][movement];
				m_OutboundMovementAry[li][movement] = outbound_link;

			
				int from_node = g_LinkVector[li]->m_FromNodeNumber ;
				int to_node = g_LinkVector[li]->m_ToNodeNumber ;
				int dest_node =  g_LinkVector[outbound_link]->m_ToNodeNumber ;

				if(to_node == 14854)
					TRACE("");


				string movement_id = GetMovementStringID(from_node, to_node,dest_node);
				if(g_NodeVector[ToID].m_MovementMap.find(movement_id) != g_NodeVector[ToID].m_MovementMap.end()) // the capacity for this movement has been defined
				{
						DTANodeMovement movement_element = g_NodeVector[ToID].m_MovementMap[movement_id];
						m_OutboundMovementDelayAry[li][movement] = movement_element.GetAvgDelay_In_Min();
				}else
				{
					m_OutboundMovementDelayAry[li][movement] = 0;   // we need to add time-dependent movement delay here
				}

				m_OutboundMovementSizeAry[li]++;
			}

		}

			//end of  constructing outbound movement vector
	}

	m_LinkSize = g_LinkVector.size();

	int iteration = 1;
	g_SetupTDTollValue(iteration);

}
void DTANetworkForSP::UpdateCurrentTravelTime(int DayNo, double CurrentTime)  // for agent based 
{

	if (m_OutboundSizeAry == NULL)
		return;
	for (unsigned int li = 0; li < g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		for (int t = CurrentTime; t < m_PlanningHorizonInMin; t += g_FindAssignmentIntervalLengthInMinFromTime(t))
		{
			// we obtain simulated time-dependent travel time measurments from simulator, use that for time-dependent shortest path calculation
			float AvgTravelTime = pLink->GetPrevailingTravelTime(DayNo, CurrentTime);

			AvgTravelTime *= g_LinkTypeMap[pLink->m_link_type].link_type_bias_factor;

			if (AvgTravelTime < 0.01f)  // to avoid possible loops
				AvgTravelTime = 0.01f;

			ASSERT(AvgTravelTime < 99999);

			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(t);

			if (pLink->m_FromNodeNumber == 4 && pLink->m_ToNodeNumber == 5)
			{
				TRACE("");
			}
			m_LinkTDTimeAry[pLink->m_LinkNo][link_entering_time_interval] = AvgTravelTime;

			// copy pricing type dependent link toll values
			for (int itoll = 0; itoll < pLink->TollVector.size(); itoll++)
			{

				if ((DayNo >= pLink->TollVector[itoll].StartDayNo && DayNo <= pLink->TollVector[itoll].EndDayNo) && t >= pLink->TollVector[itoll].StartTime && t <= pLink->TollVector[itoll].EndTime)
				{
					g_LinkTDCostAry[pLink->m_LinkNo][link_entering_time_interval].m_bMonetaryTollExist = true;

					float speed = pLink->m_Length / AvgTravelTime * 60;


					for (int demand_type = 1; demand_type <= g_DemandTypeVector.size(); demand_type++)
					{
						// toll value from user external input: 
						if (pLink->TollVector[itoll].TollRate[demand_type] >= 0.00001)
						{
						
						g_LinkTDCostAry[pLink->m_LinkNo][link_entering_time_interval].TollValue[demand_type] = pLink->TollVector[itoll].TollRate[demand_type];
						}
					}
				}
			}
				
		}

	}

}



void DTANetworkForSP::BuildTravelerInfoNetwork(int DayNo, int CurrentTime, float Perception_error_ratio)  // build the network for shortest path calculation and fetch travel time and cost real-time data from simulator
{

	std::set<DTANode*>::iterator iterNode;
	std::set<DTALink*>::iterator iterLink;

	int IntervalLinkID=0;
	int FromID, ToID;

	int i;



	for(unsigned li = 0; li< g_LinkVector.size(); li++)
	{

		if(g_LinkTypeMap[g_LinkVector[li]->m_link_type].IsConnector() && g_NodeVector[g_LinkVector[li]->m_FromNodeID ].m_ZoneID >= 0 ) // connector from centroid and not the starting link
			continue;

		FromID = g_LinkVector[li]->m_FromNodeID;
		ToID   = g_LinkVector[li]->m_ToNodeID;

		m_FromIDAry[g_LinkVector[li]->m_LinkNo] = FromID;
		m_ToIDAry[g_LinkVector[li]->m_LinkNo]   = ToID;

		//      TRACE("FromID %d -> ToID %d \n", FromID, ToID);
		m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
		m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = g_LinkVector[li]->m_LinkNo ;
		m_OutboundSizeAry[FromID] +=1;

		m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = g_LinkVector[li]->m_LinkNo ;
		m_InboundSizeAry[ToID] +=1;

		ASSERT(g_AdjLinkSize > m_OutboundSizeAry[FromID]);


		float AvgTripTime = g_LinkVector[li]->GetPrevailingTravelTime(DayNo,CurrentTime);
		//			TRACE("\n%d -> %d, time %d, TT: %f", g_NodeVector[g_LinkVector[li]->m_FromNodeID], g_NodeVector[g_LinkVector[li]->m_ToNodeID],CurrentTime,AvgTripTime);

		float Normal_random_value = g_RNNOF() * Perception_error_ratio*AvgTripTime;

		float travel_time  = AvgTripTime + Normal_random_value;
		if(travel_time < g_LinkVector[li]->m_FreeFlowTravelTime )
			travel_time = g_LinkVector[li]->m_FreeFlowTravelTime;

		m_LinkTDTimeAry[g_LinkVector[li]->m_LinkNo][0] = travel_time;

	}
	m_NodeSize = m_PhysicalNodeSize;
}


bool DTANetworkForSP::TDLabelCorrecting_DoubleQueue(
	int origin, int origin_zone, int departure_time, 
	int demand_type=1, float VOT = 10, bool distance_cost_flag = false, 
	bool debug_flag = false, bool bDistanceCostByProductOutput = true)
// time -dependent label correcting algorithm with deque implementation
{
	debug_flag = false;
	// this is the standard shortest path algorithm
	int i;
	float AdditionalCostInMin = 0;

	if(m_OutboundSizeAry[origin]== 0)
		return false;

	for(i=0; i <m_NodeSize; i++) // Initialization for all nodes
	{
		NodePredAry[i]  = -1;
		LinkNoAry[i] = -1;

		NodeStatusAry[i] = 0;

		LabelTimeAry[i] = MAX_SPLABEL;
		LabelCostAry[i] = MAX_SPLABEL;

	}


	// Initialization for origin node
	LabelTimeAry[origin] = float(departure_time);
	LabelCostAry[origin] = 0;
	
	if (bDistanceCostByProductOutput)  // used mainly for accessibility calculation 
	{

		for (i = 0; i <m_NodeSize; i++) // Initialization for all nodes
		{
			LabelDistanceAry[i] = MAX_SPLABEL;
			LabelDollarCostAry[i] = 0;
		}

		LabelDistanceAry[origin] = 0;
		LabelDollarCostAry[origin] = 0;

	
	}
	

	SEList_clear();
	SEList_push_front(origin);

	int FromID, LinkID, ToID;


	float NewTime, NewCost, NewDistance,NewDollarCost;
	float DollarCost = 0;
	while(!SEList_empty())
	{
		FromID  = SEList_front();
		SEList_pop_front();

		if(debug_flag && FromID < m_PhysicalNodeSize)  // physical nodes
		{
			TRACE("\nScan from node %d",g_NodeVector[FromID].m_NodeNumber);
		}

		NodeStatusAry[FromID] = 2;        //scaned

		for(i=0; i<m_OutboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(j)
		{
			LinkID = m_OutboundLinkAry[FromID][i];
			ToID = m_OutboundNodeAry[FromID][i];

			if(ToID == origin)
				continue;


			if (m_LinkConnectorFlag[LinkID] == 1)  // only check the following speical condition when a link is a connector
			{
				int OriginTAZ = m_OutboundConnectorOriginZoneIDAry[FromID][i];
				int DestinationTAZ = m_OutboundConnectorDestinationZoneIDAry[FromID][i];

				if (OriginTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ <= 0 && OriginTAZ != origin_zone)
					continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

				//if (DestinationTAZ >= 1 /* TAZ >=1*/ && OriginTAZ <= 0 && DestinationTAZ != destination_zone)
				//	continue;  // special feature 2: skip connectors with destination TAZ that do not belong to this destination zone

				//if (OriginTAZ >= 1 /* TAZ >=1*/ && OriginTAZ != origin_zone  && DestinationTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ != destination_zone)
				//	continue;  // special feature 3: skip connectors (with both TAZ at two ends) that do not belong to the origin/destination zones

				if (ToID == origin) // special feature 2: no detour at origin
					continue;
			}

			if(debug_flag )  // physical nodes
			{
				TRACE("\n   to node %d", ToID);
			}
			// need to check here to make sure  LabelTimeAry[FromID] is feasible.


			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LabelTimeAry[FromID]);

			link_entering_time_interval = min(link_entering_time_interval, m_NumberOfSPCalculationIntervals);

			NewDistance    = LabelDistanceAry[FromID] + m_LinkTDDistanceAry[LinkID];


			if(distance_cost_flag)
				NewTime	= LabelTimeAry[FromID];
			else // distance
				NewTime	= LabelTimeAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval];  // time-dependent travel times come from simulator

			if(distance_cost_flag)
				NewCost    = LabelCostAry[FromID] + m_LinkTDDistanceAry[LinkID];
			else
				NewCost    = LabelCostAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval] ;

			DollarCost = 0;
			if(VOT > 0.01 && g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bMonetaryTollExist) 
			{ // with VOT and toll
				AdditionalCostInMin = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue [demand_type]/VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions
				NewCost += AdditionalCostInMin;
				DollarCost = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type];
			}

			if (g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bTravelTimeTollExist)
			{ 
				demand_type = 1; // travel time SO 
				AdditionalCostInMin = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type];
				if (debug_flag)
					TRACE("AdditionalCostInMin = %f\n", AdditionalCostInMin);

				NewCost += AdditionalCostInMin;
			}


			NewDollarCost = LabelDollarCostAry[FromID] + DollarCost;

			if(NewCost < LabelCostAry[ToID] ) // be careful here: we only compare cost not time
			{
				if(debug_flag )  // physical nodes
				{
					TRACE("\n         UPDATE to node %d, cost: %f, link travel time %f", ToID, NewCost, m_LinkTDTimeAry[LinkID][link_entering_time_interval]);
				}

				if(NewTime > g_PlanningHorizon -1)
					NewTime = float(g_PlanningHorizon-1);

				LabelTimeAry[ToID] = NewTime;
				LabelCostAry[ToID] = NewCost;


				if (bDistanceCostByProductOutput)
				{
					LabelDistanceAry[ToID] = NewDistance;
					LabelDollarCostAry[ToID] = NewDollarCost;

				
				}

				
				
				NodePredAry[ToID]   = FromID;
				LinkNoAry[ToID] = LinkID;


				// Dequeue implementation
				//
				if(NodeStatusAry[ToID]==2) // in the SEList_TD before
				{
					SEList_push_front(ToID);
					NodeStatusAry[ToID] = 1;
				}
				if(NodeStatusAry[ToID]==0)  // not be reached
				{
					SEList_push_back(ToID);
					NodeStatusAry[ToID] = 1;
				}

				//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
			}

		}      // end of for each link

	} // end of while


	//if (bDistanceCostByProductOutput)  // used mainly for accessibility calculation 
	//{

	//	for (i = 0; i < m_NodeSize; i++) // Initialization for all nodes
	//	{
	//		TRACE("Node %d, tt = %f, dist= %f,cost = %f\n", g_NodeVector[i].m_NodeNumber, LabelCostAry[i], LabelDistanceAry[i], LabelDollarCostAry[i]);
	//	}
	//}
	return true;
}



int DTANetworkForSP::FindBestPathWithVOTForSingleAgent(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool debug_flag, float PerceptionErrorRatio)   // Pointer to previous node (node)
// time-dependent label correcting algorithm with deque implementation
{

	if (VOT <= 0.01)  // overwrite small VOT;
		VOT = 1;

	TotalCost  = 0;
	debug_flag = false;

	if(origin == destination ) // origin and destination nodes are the same
		return 0;


	if(g_ShortestPathWithMovementDelayFlag)
		return FindBestPathWithVOTForSingleAgent_Movement(origin_zone, origin, departure_time, destination_zone, destination, demand_type, VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, debug_flag, PerceptionErrorRatio);

	if(demand_type == 0) // unknown type
		demand_type = 1; 


	if(debug_flag)
	{

			TRACE("\nScan from root node %d,",g_NodeVector[origin].m_NodeNumber);
			TRACE("\ndestination node %d,",g_NodeVector[destination].m_NodeNumber);
	}

	// checking boundary condition for departure time changes
	if(departure_time < g_DemandLoadingStartTimeInMin)
		departure_time = g_DemandLoadingStartTimeInMin;

	if(departure_time > g_DemandLoadingEndTimeInMin)
		departure_time = g_DemandLoadingEndTimeInMin;


	int i;
	if(m_OutboundSizeAry[origin]== 0)
		return 0;  // no outgoing link from the origin

	for(i=0; i <m_NodeSize; i++) // Initialization for all nodes
	{
		NodePredAry[i]  = -1;
		NodeStatusAry[i] = 0;

		LabelTimeAry[i] = MAX_SPLABEL;
		LabelCostAry[i] = MAX_SPLABEL;
		LabelDistanceAry[i] = MAX_SPLABEL;

	}

	
	// Initialization for origin node
	LabelTimeAry[origin] = float(departure_time);
	

	if (bGeneralizedCostFlag)
		LabelCostAry[origin] = 0;
	else
		LabelCostAry[origin] = departure_time;
	
	LabelDistanceAry[origin] = 0;

	SEList_clear();
	SEList_push_front(origin);

	int FromID, LinkID, ToID;
	float CostUpperBound = MAX_SPLABEL;

	float NewTime, NewCost, NewDistance;
	while(!SEList_empty())
	{
		FromID  = SEList_front();
		SEList_pop_front();


		if(debug_flag)
			TRACE("\nScan from node %d,",g_NodeVector[FromID].m_NodeNumber);

		NodeStatusAry[FromID] = 2;        //scaned

		for(i=0; i< m_OutboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(i)
		{
			LinkID = m_OutboundLinkAry[FromID][i];
			ToID = m_OutboundNodeAry[FromID][i];

			int ToNodeNumber = g_NodeVector[ToID].m_NodeNumber;

			if(m_LinkConnectorFlag[LinkID] ==1 )  // only check the following speical condition when a link is a connector
			{
			int OriginTAZ  = m_OutboundConnectorOriginZoneIDAry[FromID][i];
			int DestinationTAZ  = m_OutboundConnectorDestinationZoneIDAry[FromID][i];

			if( OriginTAZ >=1 /* TAZ >=1*/  && DestinationTAZ <=0 && OriginTAZ != origin_zone)
			continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

			if( DestinationTAZ >=1 /* TAZ >=1*/ && OriginTAZ <=0 && DestinationTAZ != destination_zone )
			continue;  // special feature 2: skip connectors with destination TAZ that do not belong to this destination zone

			if( OriginTAZ >=1 /* TAZ >=1*/ && OriginTAZ != origin_zone  && DestinationTAZ >=1 /* TAZ >=1*/ && DestinationTAZ != destination_zone)
			continue;  // special feature 3: skip connectors (with both TAZ at two ends) that do not belong to the origin/destination zones

			if(ToID == origin) // special feature 2: no detour at origin
			continue;
			}
			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LabelTimeAry[FromID]);

			if(bGeneralizedCostFlag)
				NewTime = LabelTimeAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval];
			else 
			{// non- distance
					float preceived_travel_time  =  m_LinkTDTimeAry[LinkID][link_entering_time_interval];


					NewTime	= LabelTimeAry[FromID] + preceived_travel_time;  // time-dependent travel times come from simulator
			}

			//  road pricing module
			float toll_in_min = 0;			// special feature 5: road pricing

			if(VOT > 0.01 && g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bMonetaryTollExist) 
			{ // with VOT and toll
				toll_in_min = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue [demand_type]/VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions

				if(debug_flag)
					TRACE("\ntoll in min = %f",toll_in_min);

			}

			if (g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bTravelTimeTollExist)
			{
				toll_in_min = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type];
				if (debug_flag)
					TRACE("AdditionalCostInMin = %f\n", toll_in_min);
			}
			// end of road pricing module
		    // special feature 6: update cost

				NewDistance    = LabelDistanceAry[FromID] + m_LinkTDDistanceAry[LinkID];  // do not take into account toll value

				if(bGeneralizedCostFlag)
					NewCost = LabelCostAry[FromID] + g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[0];  // consider generalized cost only without actual travel time, because it is difficult to convert trave time to generalized cost values 
				else 
				{  // non distance cost
						NewCost    = LabelCostAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval] + toll_in_min;       // costs come from time-dependent tolls, VMS, information provisions

				}

				if( g_floating_point_value_less_than(NewCost, LabelCostAry[ToID])  && NewCost < CostUpperBound) // special feature 7.1  we only compare cost not time
				{
					if(debug_flag && ( ToID== 9))
						TRACE("\n         UPDATE to node %d, cost: %f, link travel time %f", g_NodeVector[ToID].m_NodeNumber, NewCost, m_LinkTDTimeAry[LinkID][link_entering_time_interval]);

					if(NewTime > m_PlanningHorizonInMin -1)
						NewTime = m_PlanningHorizonInMin-1;

					LabelTimeAry[ToID] = NewTime;
					LabelCostAry[ToID] = NewCost;
					LabelDistanceAry[ToID] = NewDistance;
					NodePredAry[ToID]   = FromID;
					LinkNoAry[ToID] = LinkID;

					if (ToID == destination) // special feature 7.2: update upper bound cost
					{
						CostUpperBound = LabelCostAry[ToID];
					}

					// Dequeue implementation
					//
					if(NodeStatusAry[ToID]==2) // in the SEList_TD before
					{
						SEList_push_front(ToID);
						NodeStatusAry[ToID] = 1;
					}
					if(NodeStatusAry[ToID]==0)  // not be reached
					{
						SEList_push_back(ToID);
						NodeStatusAry[ToID] = 1;
					}

					//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
				}

			}      // end of for each link



		} // end of while

		int LinkSize = 0;
		int PredNode = NodePredAry[destination];	
		temp_reversed_PathLinkList[LinkSize++] = LinkNoAry[destination];

		while(PredNode != origin && PredNode!=-1 && LinkSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
		{
			ASSERT(LinkSize< MAX_NODE_SIZE_IN_A_PATH-1);
			temp_reversed_PathLinkList[LinkSize++] = LinkNoAry[PredNode];

			PredNode = NodePredAry[PredNode];

			if(debug_flag)
			{
			TRACE("\nTrace from node %d,",g_NodeVector[PredNode].m_NodeNumber);
			}

		}

		int j = 0;
		for(i = LinkSize-1; i>=0; i--)
		{
			PathLinkList[j++] = temp_reversed_PathLinkList[i];
		}

		TotalCost = LabelCostAry[destination]-departure_time;

		if(debug_flag)
		{
			TRACE("\nnode based: Path sequence end, cost = ..%f\n",TotalCost);
		}

		if(TotalCost > MAX_SPLABEL-10)
		{
			//ASSERT(false);
			return 0;
		}




//		ASSERT(node_size_in_path == LinkSize+1);
//
//		for(int i=0; i< LinkSize; i++)
//		{
////		TRACE("node based: linkId : %d vs %d\n",PathLinkList[i], tempPathLinkList[i]);
//		ASSERT(PathLinkList[i]==tempPathLinkList[i]);
//		}

		return LinkSize+1; // as NodeSize
	}


	int DTANetworkForSP::FindBestSystemOptimalPathWithVOT(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool ResponseToRadioMessage, bool debug_flag)   // Pointer to previous node (node)
		// time-dependent label correcting algorithm with deque implementation
	{

		if (VOT <= 0.01)  // overwrite small VOT;
			VOT = 1;

		TotalCost = 0;
		debug_flag = false;

		if (origin == destination) // origin and destination nodes are the same
			return 0;

		// comment it out as we do not need to consider movement delay at SO mode: too compplex. 
		//if (g_ShortestPathWithMovementDelayFlag)
		//	return FindBestPathWithVOTForSingleAgent_Movement(origin_zone, origin, departure_time, destination_zone, destination, demand_type, VOT, PathLinkList, TotalCost, bGeneralizedCostFlag, debug_flag);

		if (demand_type == 0) // unknown type
			demand_type = 1;


		if (debug_flag)
		{

			TRACE("\nScan from root node %d,", g_NodeVector[origin].m_NodeNumber);
			TRACE("\ndestination node %d,", g_NodeVector[destination].m_NodeNumber);
		}

		// checking boundary condition for departure time changes
		if (departure_time < g_DemandLoadingStartTimeInMin)
			departure_time = g_DemandLoadingStartTimeInMin;

		if (departure_time > g_DemandLoadingEndTimeInMin)
			departure_time = g_DemandLoadingEndTimeInMin;


		int i;
		if (m_OutboundSizeAry[origin] == 0)
			return 0;  // no outgoing link from the origin

		for (i = 0; i <m_NodeSize; i++) // Initialization for all nodes
		{
			NodePredAry[i] = -1;
			NodeStatusAry[i] = 0;

			LabelTimeAry[i] = MAX_SPLABEL;
			LabelCostAry[i] = MAX_SPLABEL;
			LabelDistanceAry[i] = MAX_SPLABEL;

		}

		// Initialization for origin node
		LabelTimeAry[origin] = float(departure_time);
		LabelCostAry[origin] = 0;
		LabelDistanceAry[origin] = 0;

		SEList_clear();
		SEList_push_front(origin);

		int FromID, LinkID, ToID;
		float CostUpperBound = MAX_SPLABEL;

		float NewTime, NewCost, NewDistance;
		while (!SEList_empty())
		{
			FromID = SEList_front();
			SEList_pop_front();


			if (debug_flag && g_NodeVector[FromID].m_NodeNumber == 4217)
				TRACE("\nScan from node %d,", g_NodeVector[FromID].m_NodeNumber);

			NodeStatusAry[FromID] = 2;        //scaned

			for (i = 0; i< m_OutboundSizeAry[FromID]; i++)  // for each arc (i,j) belong A(i)
			{
				LinkID = m_OutboundLinkAry[FromID][i];
				ToID = m_OutboundNodeAry[FromID][i];

				int ToNodeNumber = g_NodeVector[ToID].m_NodeNumber;

				if (m_LinkConnectorFlag[LinkID] == 1)  // only check the following speical condition when a link is a connector
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
				int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LabelTimeAry[FromID]);
				if (bGeneralizedCostFlag)
					NewTime = LabelTimeAry[FromID];
				else
				{// non- distance
					float preceived_travel_time = m_LinkTDTimeAry[LinkID][link_entering_time_interval];


						NewTime = LabelTimeAry[FromID] + preceived_travel_time;  // time-dependent travel times come from simulator
				}

				//  road pricing module
				float toll_in_min = 0;			// special feature 5: road pricing

				if (VOT > 0.01 && g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bMonetaryTollExist)
				{ // with VOT and toll
					toll_in_min = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type] / VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions

					if (debug_flag)
						TRACE("\ntoll in min = %f", toll_in_min);

				}

				if (g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bTravelTimeTollExist)
				{
					toll_in_min = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type];
					if (debug_flag)
						TRACE("AdditionalCostInMin = %f\n", toll_in_min);
				}

				// end of road pricing module
				// special feature 6: update cost

				NewDistance = LabelDistanceAry[FromID] + m_LinkTDDistanceAry[LinkID];  // do not take into account toll value

				if (bGeneralizedCostFlag)
					NewCost = LabelCostAry[FromID] + m_LinkTDDistanceAry[LinkID];  // do not take into account toll value
				else
				{  // non distance cost
						NewCost = LabelCostAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval] + toll_in_min;       // costs come from time-dependent tolls, VMS, information provisions

				}

				if (g_floating_point_value_less_than(NewCost, LabelCostAry[ToID]) && NewCost < CostUpperBound) // special feature 7.1  we only compare cost not time
				{
					if (debug_flag && (ToID == 9))
						TRACE("\n         UPDATE to node %d, cost: %f, link travel time %f", g_NodeVector[ToID].m_NodeNumber, NewCost, m_LinkTDTimeAry[LinkID][link_entering_time_interval]);

					if (NewTime > m_PlanningHorizonInMin - 1)
						NewTime = m_PlanningHorizonInMin - 1;

					LabelTimeAry[ToID] = NewTime;
					LabelCostAry[ToID] = NewCost;
					LabelDistanceAry[ToID] = NewDistance;
					NodePredAry[ToID] = FromID;
					LinkNoAry[ToID] = LinkID;

					if (ToID == destination) // special feature 7.2: update upper bound cost
					{
						CostUpperBound = LabelCostAry[ToID];
					}

					// Dequeue implementation
					//
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

					//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
				}

			}      // end of for each link



		} // end of while

		int LinkSize = 0;
		int PredNode = NodePredAry[destination];
		temp_reversed_PathLinkList[LinkSize++] = LinkNoAry[destination];

		while (PredNode != origin && PredNode != -1 && LinkSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
		{
			ASSERT(LinkSize< MAX_NODE_SIZE_IN_A_PATH - 1);
			temp_reversed_PathLinkList[LinkSize++] = LinkNoAry[PredNode];

			PredNode = NodePredAry[PredNode];

			if (debug_flag)
			{
				TRACE("\nTrace from node %d,", g_NodeVector[PredNode].m_NodeNumber);
			}

		}

		int j = 0;
		for (i = LinkSize - 1; i >= 0; i--)
		{
			PathLinkList[j++] = temp_reversed_PathLinkList[i];
		}

		TotalCost = LabelCostAry[destination] - departure_time;

		if (debug_flag)
		{
			TRACE("\nnode based: Path sequence end, cost = ..%f\n", TotalCost);
		}

		if (TotalCost > MAX_SPLABEL - 10)
		{
			//ASSERT(false);
			return 0;
		}




		//		ASSERT(node_size_in_path == LinkSize+1);
		//
		//		for(int i=0; i< LinkSize; i++)
		//		{
		////		TRACE("node based: linkId : %d vs %d\n",PathLinkList[i], tempPathLinkList[i]);
		//		ASSERT(PathLinkList[i]==tempPathLinkList[i]);
		//		}

		return LinkSize + 1; // as NodeSize
	}


	int DTANetworkForSP::FindBestPathWithVOTForSingleAgent_Movement(int origin_zone, int origin, int departure_time, int destination_zone, int destination, int demand_type, float VOT, int PathLinkList[MAX_NODE_SIZE_IN_A_PATH], float &TotalCost, bool bGeneralizedCostFlag, bool debug_flag = false, float PerceptionErrorRatio)
		// time -dependent label correcting algorithm with deque implementation
	{

		TotalCost = 0;
		debug_flag = false;

	// checking boundary condition for departure time changes
	if(departure_time < g_DemandLoadingStartTimeInMin)
		departure_time = g_DemandLoadingStartTimeInMin;

	if(departure_time > g_DemandLoadingEndTimeInMin)
		departure_time = g_DemandLoadingEndTimeInMin;


		int i;
		float AdditionalCostInMin = 0;

		if(m_OutboundSizeAry[origin]== 0)
			return 0;

		if(m_InboundSizeAry[destination]== 0)
			return 0;

		for(i=0; i <m_LinkSize; i++) // Initialization for all links
		{
			LinkPredAry[i]  = -1;
			LinkStatusAry[i] = 0;

			LinkLabelTimeAry[i] = MAX_SPLABEL;
			LinkLabelCostAry[i] = MAX_SPLABEL;


		}

		// Initialization for origin node: for all outgoing links from origin node

		LinkBasedSEList_clear();
		for(i=0; i< m_OutboundSizeAry[origin];i++)
		{
			int LinkID = m_OutboundLinkAry[origin][i];

			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(departure_time);

			LinkLabelTimeAry[LinkID] =  departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];

			if (bGeneralizedCostFlag == false)
				LinkLabelCostAry[LinkID] =  departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];
			else  // for generalized cost, we start from cost of zero, other than start time
				LinkLabelCostAry[LinkID] = 0 + g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[0];

			LinkBasedSEList_push_back (LinkID);

			if(m_ToIDAry[LinkID] == destination)  //reach destination on the first link
			{
				PathLinkList[0]= LinkID;
				return 2; // 2 nodes
			}
		}


		int FromLinkID, ToLinkID, NodeID;
		float CostUpperBound = MAX_SPLABEL;

		float NewTime, NewCost;
		while(!LinkBasedSEList_empty())
		{

			FromLinkID  = LinkBasedSEList_front();
			LinkBasedSEList_pop_front();

			if(debug_flag) 
			{
				TRACE("\nScan from link %d ->%d",g_NodeVector[m_FromIDAry[FromLinkID]].m_NodeNumber, g_NodeVector[m_ToIDAry[FromLinkID]].m_NodeNumber);
			}

			LinkStatusAry[FromLinkID] = 2;        //scaned

			for(i=0; i<m_OutboundMovementSizeAry[FromLinkID];  i++)  // for each arc (i,j) belong to A(i)
			{
				ToLinkID = m_OutboundMovementAry[FromLinkID][i];
			int FromID = m_FromIDAry[FromLinkID];

				if(debug_flag )  // physical nodes
				{
					if(g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber ==80125)
					{
					TRACE("Trace!");
					}
		//			TRACE("\n   to link %d, from node: %d, downstream node %d ", ToLinkID, g_NodeVector[FromID].m_NodeNumber, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber );
				}
				// need to check here to make sure  LabelTimeAry[FromID] is feasible.


			if(m_LinkConnectorFlag[ToLinkID] ==1)  // only check the following speical condition when a link is a connector
			{
			int OriginTAZ  = m_LinkConnectorOriginZoneIDAry[ToLinkID];
			int DestinationTAZ  = m_LinkConnectorDestinationZoneIDAry[ToLinkID];

			if( OriginTAZ >=1 /* TAZ >=1*/  && DestinationTAZ <=0 && OriginTAZ != origin_zone)
			continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

			if( DestinationTAZ >=1 /* TAZ >=1*/ && OriginTAZ <=0 && DestinationTAZ != destination_zone )
			continue;  // special feature 2: skip connectors with destination TAZ that do not belong to this destination zone

			if( OriginTAZ >=1 /* TAZ >=1*/ && OriginTAZ != origin_zone  && DestinationTAZ >=1 /* TAZ >=1*/ && DestinationTAZ != destination_zone)
			continue;  // special feature 3: skip connectors (with both TAZ at two ends) that do not belong to the origin/destination zones

			}

		if(m_ToIDAry[ToLinkID] == origin) // special feature 2: no detour at origin
			continue;

		int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LinkLabelTimeAry[FromLinkID]);

				

//  original code				NewTime	= LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

			if(bGeneralizedCostFlag)
				NewTime = LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] +m_OutboundMovementDelayAry[FromLinkID][i];
			else 
			{// non- distance

					float preceived_travel_time  = m_LinkTDTimeAry[ToLinkID][link_entering_time_interval];

					NewTime	= LinkLabelTimeAry[FromLinkID] +  preceived_travel_time + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

					double movement_delay_in_min  = m_OutboundMovementDelayAry[FromLinkID][i];
					if(movement_delay_in_min >=90 && debug_flag)
					{
					TRACE("prohibited movement!");
					}
			}



// original code			NewCost    = LinkLabelCostAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];


			//  road pricing module
			float toll_in_min = 0;			// special feature 5: road pricing
			if(VOT > 0.01 && g_LinkTDCostAry[ToLinkID][link_entering_time_interval].m_bMonetaryTollExist)	
			{ // with VOT and toll
					toll_in_min = g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue [demand_type]/VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions
			}


			if (debug_flag == 1 && ToLinkID == 5 && link_entering_time_interval>=29)
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
				if(bGeneralizedCostFlag)
					NewCost = LinkLabelCostAry[FromLinkID] + g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue[0];  // we only consider the genralized cost value (without actual travel time)
				else 
				{  // non distance cost
						NewCost    = LinkLabelCostAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i] + toll_in_min;       // costs come from time-dependent tolls, VMS, information provisions

				}


				if(NewCost < LinkLabelCostAry[ToLinkID] &&  NewCost < CostUpperBound ) // special feature 7.1  we only compare cost not time
				{
					if(debug_flag )  // physical nodes
					{
						TRACE("\n         UPDATE to link %d, downstream node %d, cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber,NewCost, m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
					}

					if(NewTime > g_PlanningHorizon -1)
						NewTime = float(g_PlanningHorizon-1);

					LinkLabelTimeAry[ToLinkID] = NewTime;
					LinkLabelCostAry[ToLinkID] = NewCost;
					LinkPredAry[ToLinkID]   = FromLinkID;

					if (m_ToIDAry[ToLinkID] == destination) // special feature 7.2  : update upper bound cost
					{
						CostUpperBound = LinkLabelCostAry[ToLinkID];
					}

					// Dequeue implementation
					//
					if(LinkStatusAry[ToLinkID]==2) // in the SEList_TD before
					{
						LinkBasedSEList_push_front (ToLinkID);
						LinkStatusAry[ToLinkID] = 1;
					}
					if(LinkStatusAry[ToLinkID]==0)  // not be reached
					{
						LinkBasedSEList_push_back (ToLinkID);
						LinkStatusAry[ToLinkID] = 1;
					}

					//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
				}else
				{
				
					if(debug_flag ==1 && LinkLabelCostAry[ToLinkID] <=900)  // physical nodes
					{
						TRACE("\n        not-UPDATEd to link %d, downstream node %d, new cost %f, old cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber,NewCost, LinkLabelCostAry[ToLinkID] , m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
					}
				
				}

			}      // end of for each link

		} // end of while


		// post processing: if destination >=0: physical node

		//step 1: find the incoming link has the lowerest cost to the destinatin node

		float min_cost = MAX_SPLABEL;
		int link_id_with_min_cost = -1;
		for(i =0; i< m_InboundSizeAry[destination]; i++)
		{ 
			int incoming_link = m_InboundLinkAry[destination][i];
			if(LinkLabelCostAry[incoming_link] < min_cost && LinkPredAry[incoming_link] >=0)
			{
				min_cost = LinkLabelCostAry[incoming_link];
				link_id_with_min_cost = incoming_link;
			}
		}


		TotalCost = min_cost - departure_time;

		if(link_id_with_min_cost <0)
		{

			int node_number = g_NodeVector[destination].m_NodeNumber ;
			cout << "Destination Node " << node_number << " cannot be reached from Origin Node " <<  g_NodeVector[origin].m_NodeNumber  << endl;
    		g_LogFile << "Error: Destination Node " << node_number << " cannot be reached from Origin Node " <<  g_NodeVector[origin].m_NodeNumber  << endl;


		/*	if(g_path_error==0)
			{

					cout << endl << "Please check file output_simulation.log. Please any key to continue... " <<endl;

				getchar();
				g_path_error ++;
			}*/


			return 0;
				//find shortest path without movement penality
			g_ShortestPathWithMovementDelayFlag = false;

			int number_of_nodes = FindBestPathWithVOTForSingleAgent(origin_zone, origin, departure_time,  destination_zone, destination, demand_type, 
				VOT,PathLinkList,TotalCost, bGeneralizedCostFlag, debug_flag);
			// check if the link(s) have the predecesssor

			for(int ii=0; ii<number_of_nodes -1; ii++)
			{
			
				if(PathLinkList[ii] <0 || PathLinkList[ii]> g_LinkVector.size())
				{
				g_ProgramStop();

				}
				cout << "Link " << g_LinkVector[PathLinkList[ii]]->m_FromNodeNumber  << "->" << g_LinkVector[PathLinkList[ii]]->m_ToNodeNumber  << " with cost " << LinkLabelCostAry[PathLinkList[ii]] << endl;
			}

			
			getchar();
		}

		//step 2 trace the incoming link to the first link in origin node

		int LinkSize = 0;
		temp_reversed_PathLinkList[LinkSize++] = link_id_with_min_cost;  // last link to destination
		int PrevLinkID = LinkPredAry[link_id_with_min_cost];	

		if(PrevLinkID==-1)
		{
		TRACE("Error!");
		
		}
		temp_reversed_PathLinkList[LinkSize++] = PrevLinkID;   //second last link

		while(PrevLinkID!=-1 && LinkSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
		{
			ASSERT(LinkSize< MAX_NODE_SIZE_IN_A_PATH-1);

			if( LinkPredAry[PrevLinkID]!= -1)
			{
				temp_reversed_PathLinkList[LinkSize++] = LinkPredAry[PrevLinkID];
			}
			PrevLinkID = LinkPredAry[PrevLinkID];
		}

		int j = 0;
		for(i = LinkSize-1; i>=0; i--)
		{
			PathLinkList[j++] = temp_reversed_PathLinkList[i];
			if(debug_flag )  
			{
				TRACE("\n  link no. %d, %d, %d ->%d",i, temp_reversed_PathLinkList[i]
				,m_FromIDAry[temp_reversed_PathLinkList[i]], m_ToIDAry[temp_reversed_PathLinkList[i]]);
			}

		}

		if(debug_flag)
		{
			TRACE("\nPath sequence end, cost = ..%f\n",min_cost);
		}

		return LinkSize+1;  // as node size

	}




	bool DTANetworkForSP::TDLabelCorrecting_DoubleQueue_PerDemandType_Movement(int CurZoneID, int origin, int departure_time, int demand_type, float VOT, bool bDistanceCost, bool debug_flag)
		// time -dependent label correcting algorithm with deque implementation
	{

		bool bGeneralizedCostFlag = false;

		float TotalCost = 0;
		debug_flag = false;
		
		// checking boundary condition for departure time changes
		if (departure_time < g_DemandLoadingStartTimeInMin)
			departure_time = g_DemandLoadingStartTimeInMin;

		if (departure_time > g_DemandLoadingEndTimeInMin)
			departure_time = g_DemandLoadingEndTimeInMin;


		int i;
		float AdditionalCostInMin = 0;

		if (m_OutboundSizeAry[origin] == 0)
			return 0;


		for (i = 0; i <m_LinkSize; i++) // Initialization for all links
		{
			LinkPredVectorPerType[demand_type][i] = -1;
			LinkStatusAry[i] = 0;

			LinkLabelTimeAry[i] = MAX_SPLABEL;
			LabelCostVectorPerType[demand_type][i] = MAX_SPLABEL;

		}

		// Initialization for origin node: for all outgoing links from origin node

		LinkBasedSEList_clear();
		for (i = 0; i< m_OutboundSizeAry[origin]; i++)
		{
			int LinkID = m_OutboundLinkAry[origin][i];

			int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(departure_time);
			LinkLabelTimeAry[LinkID] = departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];

			if (bGeneralizedCostFlag == false)
				LabelCostVectorPerType[demand_type][LinkID] = departure_time + m_LinkTDTimeAry[LinkID][link_entering_time_interval];
			else  // for generalized cost, we start from cost of zero, other than start time
				LabelCostVectorPerType[demand_type][LinkID] = 0 + g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[0];

			LinkBasedSEList_push_back(LinkID);

		}


		int FromLinkID, ToLinkID, NodeID;
		float CostUpperBound = MAX_SPLABEL;

		float NewTime, NewCost;
		while (!LinkBasedSEList_empty())
		{

			FromLinkID = LinkBasedSEList_front();
			LinkBasedSEList_pop_front();

			if (debug_flag && m_LinkConnectorFlag[FromLinkID] == 0)
			{
				TRACE("\nScan from link %d ->%d", g_NodeVector[m_FromIDAry[FromLinkID]].m_NodeNumber, g_NodeVector[m_ToIDAry[FromLinkID]].m_NodeNumber);

				if (g_NodeVector[m_FromIDAry[FromLinkID]].m_NodeNumber == 12 && g_NodeVector[m_ToIDAry[FromLinkID]].m_NodeNumber == 2)
				{
					TRACE("");
				}


			}

			if (m_LinkConnectorFlag[FromLinkID] == 0 && g_NodeVector[m_FromIDAry[FromLinkID]].m_NodeNumber == 34 && g_NodeVector[m_ToIDAry[FromLinkID]].m_NodeNumber == 2)
			{
			
				TRACE("");
			
			}

			LinkStatusAry[FromLinkID] = 2;        //scaned

			for (i = 0; i<m_OutboundMovementSizeAry[FromLinkID]; i++)  // for each arc (i,j) belong to A(i)
			{
				ToLinkID = m_OutboundMovementAry[FromLinkID][i];


				if (ToLinkID == 72)
				{
					TRACE("");

				}
				int FromID = m_FromIDAry[FromLinkID];

				if (debug_flag && m_LinkConnectorFlag[ToLinkID] == 0)  // physical nodes
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

					if (OriginTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ <= 0 && OriginTAZ != CurZoneID)
						continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone


				}

				if (m_ToIDAry[ToLinkID] == origin) // special feature 2: no detour at origin
					continue;

				int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LinkLabelTimeAry[FromLinkID]);
				if (g_TrafficFlowModelFlag == tfm_BPR)
				{  // for BPR function, the time index we look up has to be the departure time based, regardless the experienced time along the route
					link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(departure_time);

				}

				//  original code				NewTime	= LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

				if (bGeneralizedCostFlag)
					NewTime = LinkLabelTimeAry[FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];
				else
				{// non- distance

						float preceived_travel_time = m_LinkTDTimeAry[ToLinkID][link_entering_time_interval];

						NewTime = LinkLabelTimeAry[FromLinkID] + preceived_travel_time + m_OutboundMovementDelayAry[FromLinkID][i];  // time-dependent travel times come from simulator

						double movement_delay_in_min = m_OutboundMovementDelayAry[FromLinkID][i];
						if (movement_delay_in_min >= 90 && debug_flag)
						{
							TRACE("prohibited movement!");
						}
				}



				// original code			NewCost    = LabelCostVectorPerType[demand_type][FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i];


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
					NewCost = LabelCostVectorPerType[demand_type][FromLinkID] + g_LinkTDCostAry[ToLinkID][link_entering_time_interval].TollValue[0];  // we only consider the genralized cost value (without actual travel time)
				else
				{  // non distance cost
						NewCost = LabelCostVectorPerType[demand_type][FromLinkID] + m_LinkTDTimeAry[ToLinkID][link_entering_time_interval] + m_OutboundMovementDelayAry[FromLinkID][i] + toll_in_min;       // costs come from time-dependent tolls, VMS, information provisions

				}


				if (NewCost < LabelCostVectorPerType[demand_type][ToLinkID] && NewCost < CostUpperBound) // special feature 7.1  we only compare cost not time
				{
					if (debug_flag )  // physical nodes
					{

						if (m_LinkConnectorFlag[ToLinkID] == 0)
						{

							TRACE("\n         UPDATE to link %d, downstream node %d, cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber, NewCost, m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
						}
						else
						{
							TRACE("\n         UPDATE to connector link %d, cost: %f, link travel time %f", ToLinkID, NewCost, m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
						}

					}

					if (NewTime > g_PlanningHorizon - 1)
						NewTime = float(g_PlanningHorizon - 1);

					LinkLabelTimeAry[ToLinkID] = NewTime;
					LabelCostVectorPerType[demand_type][ToLinkID] = NewCost;

					ASSERT(NewCost >= -0.00001);
					LinkPredVectorPerType[demand_type][ToLinkID] = FromLinkID;


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

					if (debug_flag == 1 && m_LinkConnectorFlag[ToLinkID] == 0  && LabelCostVectorPerType[demand_type][ToLinkID] <= 900)  // physical nodes
					{
						TRACE("\n        not-UPDATEd to link %d, downstream node %d, new cost %f, old cost: %f, link travel time %f", ToLinkID, g_NodeVector[m_ToIDAry[ToLinkID]].m_NodeNumber, NewCost, LabelCostVectorPerType[demand_type][ToLinkID], m_LinkTDTimeAry[ToLinkID][link_entering_time_interval]);
					}

				}

			}      // end of for each link

		} // end of while


	
		return true;
	}


	bool DTANetworkForSP::TDLabelCorrecting_DoubleQueue_PerDemandType(int CurZoneID, int origin, int departure_time, int demand_type = 1, float VOT = 10, bool distance_cost_flag = false, bool debug_flag = false)
		// time -dependent label correcting algorithm with deque implementation
	{

		distance_cost_flag = false;

		debug_flag = true;
		int i;
		float AdditionalCostInMin = 0;

		if (m_OutboundSizeAry[origin] == 0)
			return false;
		if (g_ShortestPathWithMovementDelayFlag)
			return TDLabelCorrecting_DoubleQueue_PerDemandType_Movement(CurZoneID, origin, departure_time, demand_type, VOT, distance_cost_flag, debug_flag);


		for (i = 0; i <m_NodeSize; i++) // Initialization for all nodes
		{

			NodePredVectorPerType[demand_type][i] = -1;

			NodeStatusAry[i] = 0;

			LabelTimeAry[i] = MAX_SPLABEL;
			LabelCostVectorPerType[demand_type][i] = MAX_SPLABEL;

		}

		// Initialization for origin node
		LabelTimeAry[origin] = float(departure_time);
		LabelCostVectorPerType[demand_type][origin] = 0;

		SEList_clear();
		SEList_push_front(origin);

		int FromID, LinkID, ToID;


		float NewTime, NewCost;
		while (!SEList_empty())
		{
			FromID = SEList_front();
			SEList_pop_front();

			if (debug_flag)
			{
				if (FromID < m_PhysicalNodeSize)  // physical nodes
				{
					TRACE("\nScan from node %d", g_NodeVector[FromID].m_NodeNumber);
					if (g_NodeVector[FromID].m_NodeNumber == 11)
					{
						TRACE("dest!");
					}

				}
				else
					TRACE("\nScan from node_index %d", FromID);

			}

			NodeStatusAry[FromID] = 2;        //scaned

			for (i = 0; i<m_OutboundSizeAry[FromID]; i++)  // for each arc (i,j) belong A(j)
			{
				LinkID = m_OutboundLinkAry[FromID][i];
				ToID = m_OutboundNodeAry[FromID][i];

				if (i >= g_AdjLinkSize || LinkID >= m_LinkSize)
				{
					cout << "LinkID >= m_LinkSize" << endl;
					cout << LinkID << ">=" << m_LinkSize << "FromID = " << g_NodeVector[FromID].m_NodeNumber << "i=" << i << endl;
					getchar();
				}
				if (ToID == origin)
					continue;

				if (m_LinkConnectorFlag[LinkID] == 1)  // only check the following speical condition when a link is a connector
				{
					int OriginTAZ = m_OutboundConnectorOriginZoneIDAry[FromID][i];
					int DestinationTAZ = m_OutboundConnectorDestinationZoneIDAry[FromID][i];

					if (OriginTAZ >= 1 /* TAZ >=1*/ && DestinationTAZ <= 0 && OriginTAZ != CurZoneID)
						continue;  // special feature 1: skip connectors with origin TAZ only and do not belong to this origin zone

					if (ToID == origin) // special feature 2: no detour at origin
						continue;
				}

				if (debug_flag)
				{
					if (ToID < m_PhysicalNodeSize)  // physical nodes
						TRACE("\n   to node %d", g_NodeVector[ToID].m_NodeNumber);
					else
						TRACE("\n   to node_index %d", ToID);

				}
				// need to check here to make sure  LabelTimeAry[FromID] is feasible.


				int link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(LabelTimeAry[FromID]);


				if (g_TrafficFlowModelFlag == tfm_BPR)
				{  // for BPR function, the time index we look up has to be the departure time based, regardless the experienced time along the route
					link_entering_time_interval = g_FindAssignmentIntervalIndexFromTime(departure_time);

				}
				if (distance_cost_flag)
					NewTime = LabelTimeAry[FromID];
				else // distance
					NewTime = LabelTimeAry[FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval];  // time-dependent travel times come from simulator

				if (distance_cost_flag)
					NewCost = LabelCostVectorPerType[demand_type][FromID] + m_LinkTDDistanceAry[LinkID];
				else
					NewCost = LabelCostVectorPerType[demand_type][FromID] + m_LinkTDTimeAry[LinkID][link_entering_time_interval];

				bool with_toll_flag = true;

				if (with_toll_flag)
				{
				
					if (VOT > 0.01 && g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bMonetaryTollExist)
					{ // with VOT and toll
						AdditionalCostInMin = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type] / VOT * 60.0f;       // 60.0f for 60 min per hour, costs come from time-dependent tolls, VMS, information provisions
						if (debug_flag)
							TRACE("AdditionalCostInMin = %f\n", AdditionalCostInMin);

						NewCost += AdditionalCostInMin;
					}

					if (g_LinkTDCostAry[LinkID][link_entering_time_interval].m_bTravelTimeTollExist)
					{ // with VOT and toll
						AdditionalCostInMin = g_LinkTDCostAry[LinkID][link_entering_time_interval].TollValue[demand_type];
						if (debug_flag)
							TRACE("AdditionalCostInMin = %f\n", AdditionalCostInMin);

						NewCost += AdditionalCostInMin;
					}
				}

				if (NewCost < LabelCostVectorPerType[demand_type][ToID]) // be careful here: we only compare cost not time
				{
					if (debug_flag  && ToID < m_PhysicalNodeSize)  // physical nodes
					{
						TRACE("\n         UPDATE to node %d, cost: %f, link travel time %f", g_NodeVector[ToID].m_NodeNumber, NewCost, m_LinkTDTimeAry[LinkID][link_entering_time_interval]);
					}

					if (NewTime > g_PlanningHorizon - 1)
						NewTime = float(g_PlanningHorizon - 1);

					LabelTimeAry[ToID] = NewTime;

					LabelCostVectorPerType[demand_type][ToID] = NewCost;

					NodePredVectorPerType[demand_type][ToID] = FromID;

					// Dequeue implementation
					//
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

					//another condition: in the SEList now: there is no need to put this node to the SEList, since it is already there.
				}

			}      // end of for each link

		} // end of while

		return true;
	}

