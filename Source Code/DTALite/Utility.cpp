
// Becasese the functions below might relate to file interfaces with other proprietary software packages, no copyright or GPL statement is made here.

// Utility.cpp : Utility functions used for reading and outputing

#include "stdafx.h"
#include "math.h"
#include "DTALite.h"
#include "GlobalData.h"

#include <algorithm>
#include <functional>
#include<windows.h>
#include<stdio.h>   
#include<tchar.h>

// Use to convert bytes to MB
#define DIV 1048576

// Use to convert bytes to MB
//#define DIV 1024

// Specify the width of the field in which to print the numbers. 
// The asterisk in the format specifier "%*I64d" takes an integer 
// argument and uses it to pad and right justify the number.

#define WIDTH 7
int g_InitialFreeMemory = 0;

int g_GetFreeMemoryDataInMB()
{
	MEMORYSTATUSEX statex;

	statex.dwLength = sizeof (statex);

	GlobalMemoryStatusEx(&statex);
	return statex.ullAvailPhys / DIV;
}

CString g_GetUsedMemoryDataInMB()
{
	MEMORYSTATUSEX statex;

	statex.dwLength = sizeof (statex);

	GlobalMemoryStatusEx(&statex);
	int used_memory = g_InitialFreeMemory - statex.ullAvailPhys / DIV;
	CString str;
	str.Format("(Used RAM %d MB)", used_memory);
	return str;
}

using namespace std;

extern CTime g_AppStartTime;
extern CTime g_AppLastIterationStartTime;
// polar form of the Box-Muller transformation to get two random numbers that follow a standard normal distribution 
unsigned int g_RandomSeedForVehicleGeneration = 101;
// Linear congruential generator 
#define g_LCG_a 17364
#define g_LCG_c 0
#define g_LCG_M 65521  

long g_precision_constant=100L;
long g_precision_constant2=100L;
extern float g_DemandGlobalMultiplier;
int g_ProgramStopFlag = 0;

#include<iostream>
#include<cmath>
using namespace std;



//******************************************************************************
// linear regression code is modified based on
// http://codesam.blogspot.com/2011/06/least-square-linear-regression-of-data.html
//********************************************************************************/


struc_LinearRegressionResult LeastRegression(std::vector <SensorDataPoint> &DataVector, bool SetYInterceptTo0)
{

	struc_LinearRegressionResult result;
	result.data_size = DataVector.size();
	result.average_residue = 0;
	result.rsqr = 1;
	result.slope = 1;
	result.y_intercept = 0;
	result.avg_y_to_x_ratio = 0;

	if(DataVector.size()<1 && g_ODEstimationFlag ==1)
	{
		//cout << "OD demand estimation mode: No sensor data are available for the simulation time period." << endl;
		//cout << " Please check if sensor_count.csv has the correct data in fields start_time_in_min, and end_time_in_min." << endl;
		//cout << " Please check if input_scenario_settings.csv has the correct calibration_data_start_time_in_min, and calibration_data_start_time_in_min which should at least cover a certain time period of sensor data." << endl;
	

		//g_ProgramStop();
		return result;
	}

	double sum_x = 0;     //sum of x values
	double sum_y = 0;     //sum of y values
	double sum_xy = 0;    //sum of x * y
	double sum_xx = 0;    //sum of x^2
	double sum_residue = 0;   //sum of squared residue
	double res = 0;      //residue squared
	double slope = 0;    //slope of regression line
	double y_intercept = 0; //y intercept of regression line
	double sum_y_res = 0; //sum of squared of the discrepancies
	double average_y = 0;     //mean of y
	double average_x = 0;     //mean of x
	double y_residual = 0;     //squared of the discrepancies
	double Rsqr = 0;     //coefficient of determination

	//calculate various sum_s 
	for (int i = 0; i < DataVector.size(); i++)
	{
		//sum of x
		sum_x = sum_x + DataVector[i].x;
		//sum of y
		sum_y = sum_y + DataVector[i].y;
		//sum of squared x*y
		sum_xy = sum_xy + DataVector[i].x * DataVector[i].y;
		//sum of squared x
		sum_xx = sum_xx + DataVector[i].x * DataVector[i].x;
	}

	//calculate the means of x and y
	int dataSize = DataVector.size();
	average_y = sum_y / dataSize;
	average_x = sum_x / dataSize;

	//slope or a1
	if(dataSize==1)
		slope = 0;
	else
		slope = (dataSize * sum_xy - sum_x * sum_y) / max(0.00001,(dataSize * sum_xx - sum_x*sum_x));

	//y itercept or a0
	y_intercept = average_y - slope * average_x;

	double total_absolute_error = 0;
	double total_percentage_error = 0;

	//calculate squared residues, their sum_ etc.
	for (int i = 0; i <  DataVector.size(); i++) 
	{
		//current (y_i - a0 - a1 * x_i)^2
		y_residual = pow((DataVector[i].y - y_intercept - (slope * DataVector[i].x)), 2);

		//sum of (y_i - a0 - a1 * x_i)^2
		sum_y_res += y_residual;

		//current residue squared (y_i - average_y)^2
		res = pow(DataVector[i].y - average_y, 2);

		//sum of squared residues
		sum_residue += res;

		total_absolute_error += fabs(DataVector[i].y - DataVector[i].x);
		total_percentage_error +=  fabs(DataVector[i].y - DataVector[i].x)/max(0.1,DataVector[i].x)*100;

	}

	//calculate r^2 coefficient of determination
	Rsqr = (sum_residue - sum_y_res) / max(0.00001,sum_residue);

	Rsqr = 1 - (sum_y_res / max(0.000001,sum_residue));

	result.average_residue = sum_y_res/max(1,DataVector.size());
	result.rsqr = Rsqr;
	result.slope = slope; 
	result.y_intercept = y_intercept;
	result.avg_y_to_x_ratio = average_y / max(0.0001,average_x);  // directly calculate bias slope 

	result.avg_absolute_error = total_absolute_error / max(1,DataVector.size());
	result.avg_percentage_error = total_percentage_error / max(1,DataVector.size());


	return result;

}

std::string CString2StdString(CString str)
{	 // Convert a TCHAR string to a LPCSTR
	CT2CA pszConvertedAnsiString (str);

	// construct a std::string using the LPCSTR input
	std::string strStd (pszConvertedAnsiString);

	return strStd;
}

std::string GetTimeClockString(int time)
{
	int hour = ((int)(time))/60;
	int min = time - hour*60;

	CString time_str;
	time_str.Format("%2d:%02d",hour, min);
	return (CString2StdString(time_str));
}

bool g_floating_point_value_less_than_or_eq_comparison(double value1, double value2)
{
	long lValue1 = (long) (value1*g_precision_constant);
	long lValue2 = (long) (value2*g_precision_constant);

	if( lValue1 < lValue2 + 2) // we take 2/100 sec accuracy
		return true;
	else 
		return false;

}

bool g_floating_point_value_less_than(double value1, double value2)
{
	long lValue1 = (long) (value1*g_precision_constant2);
	long lValue2 = (long) (value2*g_precision_constant2);


	if(lValue1<lValue2)
		return true;
	else 
		return false;

}

bool g_Compare_Vehicle_Item(struc_vehicle_item item1, struc_vehicle_item item2)
{
	if(item1.event_time_stamp < item2.event_time_stamp)  // item1 is earlier than item2
		return true;
	else 
		return false;
}

double g_GetRandomRatio()
{
	//	g_RandomSeed = (g_LCG_a * g_RandomSeed + g_LCG_c) % g_LCG_M;  //m_RandomSeed is automatically updated.
	//	return float(g_RandomSeed)/g_LCG_M;

	return WELLRNG512a();
}

int g_GetRandomInteger_SingleProcessorMode(float Value)
{
	int int_value = int(Value);
	double Residual = Value - int_value;
	double RandomNumber = g_GetRandomRatio(); //between 0 and 1
	if(RandomNumber < Residual)
	{
		return int_value+1;
	}
	else
	{
		return int_value;
	}
}

int g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(float Value, int LinkID)
{ // we have to use this random number generator in parallel computing mode, as the above version uses the "shared" seed
	float Residual = Value - int(Value);
	float RandomNumber = g_LinkVector[LinkID]->GetRandomRatio();
	if(RandomNumber < Residual)
	{
		return int(Value)+1;
	}
	else
	{
		return int(Value);
	}

}

float g_GetRandomRatioForVehicleGeneration()
{
	g_RandomSeedForVehicleGeneration = (g_LCG_a * g_RandomSeedForVehicleGeneration + g_LCG_c) % g_LCG_M;  //m_RandomSeed is automatically updated.

	return float(g_RandomSeedForVehicleGeneration)/g_LCG_M;

}

string GetLinkStringID(int FromNodeName, int ToNodeName)
{
	ostringstream ss;
	ss << FromNodeName << ":" << ToNodeName;
	return ss.str();
}

string GetMovementStringID(int FromNodeName, int ToNodeName, int DestNodeName)
{
	ostringstream string_movement;
	string_movement << FromNodeName << ":" << ToNodeName <<  ":" << DestNodeName;
	return string_movement.str();
}

float g_RNNOF()
{
	float x1, x2, w, y1, y2;

	do {
		x1 = 2.0f * g_GetRandomRatio() - 1.0f;
		x2 = 2.0f * g_GetRandomRatio()- 1.0f;
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0f );

	w = sqrt( (-2.0f * log( w ) ) / w );
	y1 = x1 * w;
	y2 = x2 * w;

	return y1;  // we only use one random number
}


bool g_GetVehicleAttributes(int demand_type, int &VehicleType, int &InformationClass, float &VOT, int &Age)
{
	int demand_type_no = demand_type - 1;
	if (demand_type_no >=  g_DemandTypeVector.size())
	{
		cout << "Error: The demand file has demand_type = " << demand_type << ", which has not been defined in input_demand_type.csv."<< endl;
		g_ProgramStop();
	}

	float RandomPercentage= g_GetRandomRatio() * 100; 

	// step 1. vehicle type
	VehicleType = 1;
	// default to a single value
	int i;




	//step 2: information type
	// default to historical info as class 1
	InformationClass = 1;
	RandomPercentage= g_GetRandomRatio() * 100; 
	for(i= 1; i< MAX_INFO_CLASS_SIZE; i++)
	{
		if (RandomPercentage >= g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[i - 1] && RandomPercentage < g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[i])

		{
			InformationClass = i; // return pretrip as 2 or enoute as 3
			if (InformationClass == 2)
			{
				int cc = 1;
			}
		}
	}


	Age = 0; // default value
	if(VehicleType>=1) 
	{
		RandomPercentage= g_GetRandomRatio() * 100;

		//if (VehicleType == 5)
		//{
		//	static int n = 0;
		//	printf("n = %d, RandomPercentage = %f\n",n,RandomPercentage);
		//	n++;
		//}

		float prev_cumulative_percentage = 0;
		float cumulative_percentage = 0;

		if(g_VehicleTypeVector.size() ==0)
		{
		
			cout << "No vehicle type data. Please check file optional_vehicle_type.csv." << endl;
			g_ProgramStop();
		
		}

		for(i=0; i< g_VehicleTypeVector[VehicleType-1].percentage_age_vector.size(); i++)
		{
			cumulative_percentage+=g_VehicleTypeVector[VehicleType-1].percentage_age_vector[i];

			if(RandomPercentage > prev_cumulative_percentage && RandomPercentage <= cumulative_percentage)
			{
				Age = i;
				break;
			}

			prev_cumulative_percentage = cumulative_percentage;

		}



	}

	RandomPercentage= g_GetRandomRatio() * 100; 

	VOT = g_DemandTypeVector[demand_type_no].Avg_VOT;

	if(VOT < 1)  // enforcing minimum travel time
		VOT = 1;
	return true;
}

bool g_detect_if_a_file_is_column_format(LPCTSTR lpszFileName)
{
	FILE* st;
	fopen_s(&st, lpszFileName, "r");
	if (st != NULL)
	{
		char  str_line[_MAX_STRING_LINE]; // input string
		int str_line_size = _MAX_STRING_LINE;
		g_read_a_line(st, str_line, str_line_size);

		fclose(st);

		if (strstr(str_line, "number_of_trips_demand_type1") != NULL)
			return true;
		else
			return false;

	}
	return false;
}
int g_read_integer_with_char_O(FILE* f)
// read an integer from the current pointer of the file, skip all spaces, if read "O", return 0;
{
	char ch, buf[ 32 ];
	int i = 0;
	int flag = 1;
	/* returns -1 if end of file is reached */

	while(true)
	{
		ch = getc( f );
		if( ch == EOF ) return -1;
		if( ch == 'O' ) return 0;  // special handling

		if (isdigit(ch))
			break;
		if (ch == '-')
			flag = -1;
		else
			flag = 1;
	};
	if( ch == EOF ) return -1;
	while( isdigit( ch )) {
		buf[ i++ ] = ch;
		ch = fgetc( f );
	}
	buf[ i ] = 0;


	return atoi( buf ) * flag;

}


int read_multiple_integers_from_a_string(CString str, std::vector<int> &vector)
// read an integer from the current pointer of the file, skip all spaces
{

	if(str.GetLength () ==0 )
		return 0;

	char string_line[1000];

	int string_lenghth  = str.GetLength();
	ASSERT(str.GetLength() < 100);

	sprintf(string_line,"%s\n",str);

	char ch, buf[ 32 ];
	int i = 0;
	int buffer_i = 0;
	int flag = 1;
	/* returns -1 if end of file is reached */

	for(int i_try  =0 ; i_try < 200; i_try++)  // maximal 200 numbers
	{
		buffer_i = 0;
	while(true)
	{
		ch = string_line[i++];
		if( ch=='\n' || i > string_lenghth)
		{
			return -1; // * and $ are special characters for comments
		}
		if (isdigit(ch))
			break;
		if (ch == '-')
			flag = -1;
		else
			flag = 1;
	};
	if( ch == '\n' )
	{
		return -1;
	}
	
	while( isdigit( ch ))
	{
		buf[ buffer_i++ ] = ch;
		ch =  string_line[i++];
	}
	buf[ buffer_i ] = 0;

	int value = atoi( buf ) * flag;

	vector.push_back (value);
	}

	return 0;
}

void g_ProgramStop()
{
	if(g_ProgramStopFlag ==0)
		g_ProgramStopFlag = 1;
	cout << "DTALite Program stops. Press any key to terminate. Thanks!" <<endl;
	getchar();
	exit(0);
};

void g_ProgramTrace(CString str)
{
	//	cout << str << endl;
	//	getchar();

};
int g_read_integer(FILE* f, bool speicial_char_handling )
// read an integer from the current pointer of the file, skip all spaces
{
	char ch, buf[ 32 ];
	int i = 0;
	int flag = 1;
	/* returns -1 if end of file is reached */

	while(true)
	{
		ch = getc( f );
		if( ch == EOF || (speicial_char_handling && (ch == '*' || ch == '$')))
			return -1; // * and $ are special characters for comments
		if (isdigit(ch))
			break;
		if (ch == '-')
			flag = -1;
		else
			flag = 1;
	};
	if( ch == EOF ) return -1;


	while( isdigit( ch )) {
		buf[ i++ ] = ch;
		ch = fgetc( f );
	}
	buf[ i ] = 0;


	return atoi( buf ) * flag;

}


float g_read_float_from_a_line(FILE *f)
//read a floating point number from the current pointer of the file,
//skip all spaces

{
	char ch, buf[ 32 ];
	int i = 0;
	int flag = 1;

	/* returns -100 if end of line is reached */

	while(true)
	{
		ch = getc( f );
		if( ch == EOF || ch == '*' || ch == '$' ) 
			return -1;

		if( ch == '\n' ) 
			return -100;

		if (isdigit(ch))
			break;

		if (ch == '-')
			flag = -1;
		else
			flag = 1;

	};
	if( ch == EOF ) return -1;
	while( isdigit( ch ) || ch == '.' ) {
		buf[ i++ ] = ch;
		ch = fgetc( f );

	}
	buf[ i ] = 0;

	/* atof function converts a character string (char *) into a doubleing
	pointer equivalent, and if the string is not a floting point number,
	a zero will be return.
	*/

	return (float)(atof( buf ) * flag);

}

float g_read_float(FILE *f)
//read a floating point number from the current pointer of the file,
//skip all spaces

{
	char ch, buf[ 32 ];
	int i = 0;
	int flag = 1;

	/* returns -1 if end of file is reached */

	while(true)
	{
		ch = getc( f );
		if( ch == EOF || ch == '*' || ch == '$' ) return -1;
		if (isdigit(ch))
			break;

		if (ch == '-')
			flag = -1;
		else
			flag = 1;

	};
	if( ch == EOF ) return -1;
	while( isdigit( ch ) || ch == '.' ) {
		buf[ i++ ] = ch;
		ch = fgetc( f );

	}
	buf[ i ] = 0;

	/* atof function converts a character string (char *) into a doubleing
	pointer equivalent, and if the string is not a floting point number,
	a zero will be return.
	*/

	return (float)(atof( buf ) * flag);

}


int g_read_number_of_numerical_values(char* line_string, int length)
//read a floating point number from the current pointer of the file,
//skip all spaces

{
	char ch, buf[ 32 ];

	int number_count= 0;
	int string_index = 0;

	/* returns -1 if end of file is reached */
	while(string_index<length)
	{

		int i = 0;
		int flag = 1;

		while(true)
		{
			if(string_index==length)
			{
				break;
			}


			ch = line_string[string_index++];
			if( ch == EOF ) return number_count;
			if (isdigit(ch))
				break;

			if (ch == '-')
				flag = -1;
			else
				flag = 1;

		};
		if( ch == EOF ) return number_count;
		while( isdigit( ch ) || ch == '.' ) {
			buf[ i++ ] = ch;
			ch = line_string[string_index++];

		}
		buf[ i ] = 0;

		double value = atof( buf );
		if(value>-0.0000001)  // positive values
		{

			number_count++;
		}
	}

	/* atof function converts a character string (char *) into a doubleing
	pointer equivalent, and if the string is not a floting point number,
	a zero will be return.
	*/

	return number_count;

}

int g_read_numbers_from_a_line(FILE *f, std::vector<float> &ValueVector)
//read a floating point number from the current pointer of the file,
//skip all spaces

{
	char ch, buf[32];

	int string_index = 0;

	ValueVector.clear();

	/* returns -1 if end of file is reached */
	while (1)
	{

		int i = 0;
		int flag = 1;

		while (true)
		{
			ch = getc(f);
			if (ch == EOF)
				return -100;
			if (isdigit(ch))
				break;

			if (ch == '\n')
				return ValueVector.size();

			if (ch == '-')
				flag = -1;
			else
				flag = 1;

		};
		if (ch == EOF)
			return -100;

		while (isdigit(ch) || ch == '.') {
			buf[i++] = ch;
			ch = getc(f);

		}
		buf[i] = 0;

		double value = atof(buf);
		if (value>-0.0000001)  // positive values
		{
			ValueVector.push_back(value);

		}

		if (ch == '\n')
			return ValueVector.size();

	}

	/* atof function converts a character string (char *) into a doubleing
	pointer equivalent, and if the string is not a floting point number,
	a zero will be return.
	*/

	return ValueVector.size();

}

int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename,bool print_out) 
{
	char lpbuffer[64];
	int value = def_value;
	if(GetPrivateProfileString(section,key,"",lpbuffer,sizeof(lpbuffer),filename)) 
	{
		value =  atoi(lpbuffer); 
	}

	if(value == def_value)  //  the parameter might not exist
	{
		sprintf_s(lpbuffer,"%d",def_value);
		WritePrivateProfileString(section,key,lpbuffer,filename);
	}

	if(print_out)
		cout << "section <" << section << ">: " <<  key << " = "  << value << endl;

	return value; 
}

int g_WritePrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename) 
{
	char lpbuffer[64];
	int value = def_value;
	sprintf_s(lpbuffer,"%d",def_value);
	WritePrivateProfileString(section,key,lpbuffer,filename);
	return value; 
}

float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename,bool print_out) 
{ 
	char lpbuffer[64];
	float value = def_value;
	if(GetPrivateProfileString(section,key,"",lpbuffer,sizeof(lpbuffer),filename)) 
	{
		value =  (float)(atof(lpbuffer)); 
	}

	if(value == def_value)  //  the parameter might not exist
	{
		sprintf_s(lpbuffer,"%5.2f",def_value);
		WritePrivateProfileString(section,key,lpbuffer,filename);
	}
	if(print_out)
		cout << "section <" << section << ">: " <<  key << " = "  << value << endl;

	return value; 
} 

struct entity_deleter
{
	void operator()(DTAVehicle*& e) // important to take pointer by reference!
	{ 
		delete e;
		e = NULL;
	}
};

void g_FreeMemoryForVehicleVector()
{
	cout << "Free memory for vehicle set... " << endl;
	for_each(g_VehicleVector.begin(), g_VehicleVector.end(), entity_deleter());

	//std::vector<DTAVehicle*>::iterator iterVehicle;		//this part of code needs to be carelfully reviewed, as it tries to delete pointers within STL					
	//for (iterVehicle = g_VehicleVector.begin(); iterVehicle != g_VehicleVector.end();iterVehicle++)
	//{
	//	delete *iterVehicle;
	//	iterVehicle = g_VehicleVector.erase(iterVehicle);
	//}

	g_VehicleVector.clear();
	g_VehicleMap.clear();
	g_VehicleTDListMap.clear();
	g_OriginalVehicleTDListMap.clear();
	cout << "Complete. " << endl;
}
CString g_GetTimeStampString(int time_stamp_in_min)
{
	CString str;
	int hour = time_stamp_in_min/60;
	int min = (time_stamp_in_min - hour*60);

	if(hour<10)
		str.Format ("0%d:%02d",hour,min);
	else
		str.Format ("%2d:%02d",hour,min);

	// Convert a TCHAR string to a LPCSTR
	return str;
}

std::string g_GetTimeStampStrFromIntervalNo(int time_interval)
{
	CString str;
	int hour = time_interval/4;
	int min = (time_interval - hour*4)*15;

	if(hour<10)
		str.Format ("'0%d:%02d",hour,min);
	else
		str.Format ("'%2d:%02d",hour,min);

	// Convert a TCHAR string to a LPCSTR
	CT2CA pszConvertedAnsiString (str);

	// construct a std::string using the LPCSTR input
	std::string  strStd (pszConvertedAnsiString);


	return strStd;
}


CString g_GetAppRunningTime(bool with_title)
{
	CString str;
	CTime EndTime = CTime::GetCurrentTime();
	CTimeSpan ts = EndTime  - g_AppStartTime;

	if(with_title)
		str = ts.Format( "CPU Clock: %H:%M:%S --" );
	else
		str = ts.Format( "%H:%M:%S" );
	return str;
}

CString g_GetAppRunningTimePerIteration(bool with_title)
{
	CString str;
	CTime EndTime = CTime::GetCurrentTime();
	CTimeSpan ts = EndTime - g_AppLastIterationStartTime;

	if (with_title)
		str = ts.Format("CPU Clock: %H:%M:%S --");
	else
		str = ts.Format("%H:%M:%S");
	return str;
}


char g_GetLevelOfService(int PercentageOfSpeedLimit)
{
	if(PercentageOfSpeedLimit >= 90)
		return 'A';
	else if (PercentageOfSpeedLimit >= 70)
		return 'B';
	else if (PercentageOfSpeedLimit >= 50)
		return 'C';
	else if (PercentageOfSpeedLimit >= 40)
		return 'D';
	else if (PercentageOfSpeedLimit >= 33)
		return 'E';
	else 
		return 'F';
}

bool g_read_a_line(FILE* f)
/* read a line from the current line from the file */
{

	char ch;

	while( 1 ) {
		ch = getc( f );
		if( ch != 13 && ch != 10 && ch != EOF )
		{
			// do nothing
		}
		else { /* terminate if it's end of line or end of file */
			{
				// do nothing
			}
			if( ch == EOF )
				return false;

			return true;
		}
	}
}


bool g_read_a_line(FILE* f, char* aline, int & size)
/* read a line from the current line from the file */
{
	int max_size = size;
	char ch;
	size = 0;

	while (size < max_size) {
		ch = getc( f );
		if (ch != 13 && ch != 10 && ch != EOF)
			aline[ size++ ] = ch;
		else { /* terminate if it's end of line or end of file */
			aline[ size ] = 0;
			if( ch == EOF )
				return false;

			return true;
		}
	}
	return false;
}


int  DTANetworkForSP:: GetLinkNoByNodeIndex(int usn_index, int dsn_index)
{
	int LinkNo = -1;
	for(int i=0; i < m_OutboundSizeAry[usn_index]; i++)
	{

		if(m_OutboundNodeAry[usn_index][i] == dsn_index)
		{
			LinkNo = m_OutboundLinkAry[usn_index][i];
			return LinkNo;
		}
	}

	cout << " Error in GetLinkNoByNodeIndex " << g_NodeVector[usn_index].m_NodeNumber  << "-> " << g_NodeVector[dsn_index].m_NodeNumber ;

	g_ProgramStop();


	return MAX_LINK_NO;


}

void ConnectivityChecking(DTANetworkForSP* pPhysicalNetwork)
{
	// network connectivity checking

	unsigned int i;
	int OriginForTesting=0;
	for(i=0; i< g_NodeVector.size(); i++)
	{
		if(pPhysicalNetwork->m_OutboundSizeAry [i] >0)
		{
			OriginForTesting = i;
			break;
		}
	}

	// starting with first node with origin nodes;
	pPhysicalNetwork->BuildPhysicalNetwork(0,-1,g_TrafficFlowModelFlag);

	pPhysicalNetwork->TDLabelCorrecting_DoubleQueue(OriginForTesting,0, 0,1,DEFAULT_VOT,false,false,false);  // CurNodeID is the node ID
	// assign shortest path calculation results to label array


	int count = 0;
	int centroid_count = 0;

	/*
	for(i=0; i< g_NodeVector.size(); i++)
	{
	if(pPhysicalNetwork->LabelCostAry[i] > MAX_SPLABEL-100)
	{

	if(g_NodeVector[i].m_ZoneID > 0)
	{
	cout << "Centroid "<<  g_NodeVector[i].m_NodeNumber  << " of zone " << g_NodeVector[i].m_ZoneID << " is not connected to node " << g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
	g_WarningFile << "Centroid "<<  g_NodeVector[i].m_NodeNumber  << " of zone " << g_NodeVector[i].m_ZoneID << " is not connected to node " << g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
	centroid_count ++;
	}else
	{
	cout << "Node "<<  g_NodeVector[i].m_NodeNumber  << " is not connected to node " << g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
	g_WarningFile << "Node "<<  g_NodeVector[i].m_NodeNumber  << " is not connected to node " << g_NodeVector[OriginForTesting].m_NodeNumber  <<", Cost: "  << endl;
	}
	count++;

	}

	}
	*/
	//	for(i=0; i< g_NodeVector.size(); i++)
	//	{
	//		g_WarningFile << "Node "<<  g_NodeVector[i] << " Cost: " << pPhysicalNetwork->LabelCostAry[i] << endl;
	//	}
	if(count > 0)
	{
		cout << count << " nodes are not connected to "<< g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
		g_WarningFile << count << " nodes are not connected to "<< g_NodeVector[OriginForTesting].m_NodeNumber  << endl;

		if(centroid_count > 0 )
		{
			cout << centroid_count << " controids are not connected to "<< g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
			g_WarningFile << centroid_count << " controids are not connected to "<< g_NodeVector[OriginForTesting].m_NodeNumber  << endl;
			//			cout << "Please check file warning.log later. Press any key to continue..."<< endl;
			//getchar();
		}
	}

}



/* ***************************************************************************** */
/* Copyright:      Francois Panneton and Pierre L'Ecuyer, University of Montreal */
/*                 Makoto Matsumoto, Hiroshima University                        */
/* Notice:         This code can be used freely for personal, academic,          */
/*                 or non-commercial purposes. For commercial purposes,          */
/*                 please contact P. L'Ecuyer at: lecuyer@iro.UMontreal.ca       */
/* ***************************************************************************** */
#define W 32
#define R 16
#define P 0
#define M1 13
#define M2 9
#define M3 5

#define MAT0POS(t,v) (v^(v>>t))
#define MAT0NEG(t,v) (v^(v<<(-(t))))
#define MAT3NEG(t,v) (v<<(-(t)))
#define MAT4NEG(t,b,v) (v ^ ((v<<(-(t))) & b))

#define V0            STATE[state_i                   ]
#define VM1           STATE[(state_i+M1) & 0x0000000fU]
#define VM2           STATE[(state_i+M2) & 0x0000000fU]
#define VM3           STATE[(state_i+M3) & 0x0000000fU]
#define VRm1          STATE[(state_i+15) & 0x0000000fU]
#define VRm2          STATE[(state_i+14) & 0x0000000fU]
#define newV0         STATE[(state_i+15) & 0x0000000fU]
#define newV1         STATE[state_i                 ]
#define newVRm1       STATE[(state_i+14) & 0x0000000fU]

#define FACT 2.32830643653869628906e-10

static unsigned int state_i = 0;
static unsigned int STATE[R];
static unsigned int z0, z1, z2;

void InitWELLRNG512a (unsigned int *init){
	int j;
	state_i = 0;
	for (j = 0; j < R; j++)
		STATE[j] = init[j];
}

double WELLRNG512a (void){
	z0    = VRm1;
	z1    = MAT0NEG (-16,V0)    ^ MAT0NEG (-15, VM1);
	z2    = MAT0POS (11, VM2)  ;
	newV1 = z1                  ^ z2; 
	newV0 = MAT0NEG (-2,z0)     ^ MAT0NEG(-18,z1)    ^ MAT3NEG(-28,z2) ^ MAT4NEG(-5,0xda442d24U,newV1) ;
	state_i = (state_i + 15) & 0x0000000fU;
	return ((double) STATE[state_i]) * FACT;
}

