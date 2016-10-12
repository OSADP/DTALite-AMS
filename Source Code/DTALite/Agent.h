//  Portions Copyright 2012

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
#include "resource.h"

#include <math.h>
#include <deque>
#include <map>
#include <set>
#include <iostream>
#include <vector>
#include <list>
using namespace std;

class DTATrip
{
	public:
	int OriginActivityLocation;
	int DestinationActivityLocation;
	int DepartureTimeInMin;
	int ArrivalTimeInMin;
	int VehicleID;
};

class DTAAgent
{
public:
	int m_AgentID;
	std::vector<DTATrip> TripVector;

	int FindVehicleID(int TripPurpose, int ArrivalTimeInMin, )
	{
	// given Duration:

	}
}

void VehicleToAgentAssignment()
{
// given OD demand table
// 
}