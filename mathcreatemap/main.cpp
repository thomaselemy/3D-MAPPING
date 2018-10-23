#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include "tz.h"

using namespace std;

const double PI = 3.141592653589793238463;

int LineCount(ifstream&);
double ConvertToRadians(double);
double ConvertToDegrees(double);
void print(string);

int main() {

    using namespace std::chrono;
    using namespace date;

    //double y_offset = -40 * PI / 180;

    //opens the files to read/write
    ifstream lidarIFS("lidarData.txt");
    ifstream imuIFS("IMU.txt");
    ofstream test("testData.txt");
    ofstream ptCloudOFS("trial_.txt");
    //ofstream altFormat("imu_and_lidar_alt_format.txt");

    print("***Counting lines from input files and creating matrices");

    //Counts how many lines are in the file, skips every 13th line(which is the timestamp) and
    int nLidarLines = LineCount(lidarIFS);
    nLidarLines = (nLidarLines - floor(nLidarLines / 13)) * 2;	//multiplies it by two because each line in the text file takes up two lines in the matrix.
    int nGpsLines = (nLidarLines - (12 * floor(nLidarLines / 13)));
    int nImuLines = LineCount(imuIFS);

    //Resets the the file stream pointer to the beginning of the file.
    lidarIFS.clear();
    lidarIFS.seekg(0);
    imuIFS.clear();
    imuIFS.seekg(0);

#pragma region ARRAY DECLARATION

    double** lidarData = new double*[nLidarLines];
    for (int i = 0; i < nLidarLines; i++)
    {
        lidarData[i] = new double[50];
    }

    string** lidarGPS = new string*[nGpsLines];
    for (int i = 0; i < nGpsLines; i++)
    {
        lidarGPS[i] = new string[13];
    }

    double** imuData = new double*[nImuLines];
    for (int i = 0; i < nImuLines; i++)
    {
        imuData[i] = new double[11];
    }

    print("DONE");


    //Angle matrix initialization and conversion to radians.
    //Angles of the 16 individual lasers are provided by Velodyne documentation.
    //double laserAngle[16] = { 105, 89, 103, 93, 101, 85, 99, 83, 97, 81, 95, 79, 93, 77, 91, 75 };
    double laserAngle[16] = { 15, -1, 13, -3, 11, -5, 9, -7, 7, -9, 5, -11, 3, -13, 1, -15 };
    for (int ctr = 0; ctr < 16; ctr++)
    {
        laserAngle[ctr] = laserAngle[ctr] * PI / 180; //conversion to radians
    }
#pragma endregion

#pragma region VARIABLES FOR DATA INPUT
    const string ANGLE_DET = "angle=";	//"angle="
    const string TIME_DET = "time=";	//"time="
    const string GPS_DET = "GPS=";		//"GPS="

    string cur;			//Stores a line from the LIDAR text file. It is replaced with the following line during every looping of the while loop.
    int row = 0;		//Row value for the lidarData two-dimensional array.
    int col = 0;		//Column value "										".
    int gRow = 0;		//Row value for the lidarGPS two-dimensional array.
    int charPos = 0;	//Modified throughout the program to know where to start reading data.
    int curTime = 0;	//Stores the value of the most recently encountered LIDAR time value.
    int gpsTime = 0;	//Stores the value of the most recently encountered GPS time stamp, as identified by the VLP-16 documentation.
#pragma endregion

#pragma region VARIABLES FOR GEOREFERENCING MATH
    double lat0 = 0;		//will store the initial latitude value
    double lon0 = 0;		//will store the initial longitude value
    double alt0 = 0;		//will store the initial altitude value
    double roll0 = 0;
    double pitch0 = 0;
    double yaw0 = 0;
    double imuTime0 = 0;	//will store the initial IMU time stamp
    double lidarTime0 = 0;	//will store the initial LIDAR time stamp

    double lat = 0;
    double lon = 0;
    double alt = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    double latLength = 0;
    double lonLength = 0;

    double latOffset = 0;
    double lonOffset = 0;

    double imuTimeA = 0;	//IMU time stamp A for time stamp synchronization
    double imuTimeB = 0;	//IMU time stamp B for time stamp synchronization
    long long lidarTime = 0;	//LIDAR time stamp for time stamp synchronization
    int imuRowSelect = 0;	//will store the row number for which IMU data values to do the georef math with
    int lRow = 0;			//for traversing LIDAR matrix rows
    int lCol = 3;			//for traversing LIDAR matrix columns

    double alpha = 0;
    double distance = 0;
    double timeStamp = 0;
    double omega = 0;
    double X = 0;
    double Y = 0;
    double Z = 0;
    int lzr = 0;
#pragma endregion

    print("***Processing LIDAR data");

    while (getline(lidarIFS, cur))
    {
        //Seeks angle_det at the beginning of a line, stores angle value and the following distance and reflectivity points.
        //Interpolates missing angle values, as described in the VLP-16 documentation
        if (cur.substr(0, 6) == ANGLE_DET)
        {
            lidarData[row][col] = stod(cur.substr(6, 11)); //getting the angle value

            int cursor = 0;

            for (int i = 1; i < 96; i++)
            {
                col++;

                if (i == 49) //this indicates that it is at the end of one sequence of lazer firings
                {
                    row++;	//go down one row for the next set of distance+reflectivity values
                    col = 1;	//we will not go to 0 because that is where the interpolated angle value will be stored

                    //azimuth interpolation
                    if (row - 3 >= 0)	//checking to avoid access violation error or index out of bound error
                    {
                        double azi1 = lidarData[row - 3][0];
                        double azi3 = lidarData[row - 1][0];

                        if (azi3 < azi1)
                        {
                            azi3 = azi3 + 36000;
                        }

                        double azi2 = (azi1 + azi3) / 2;

                        if (azi2 > 35999)	//accounting for rollover. values are not to exceed 35999.
                        {
                            azi2 = azi2 - 36000;
                        }

                        lidarData[row - 2][0] = azi2; //assign the missing angle value with the interpolated one
                    }
                }

                if (i % 3 != 0) //This is to avoid any column that is reserved for time stamps. See the lidarData Matrix Organization spreadsheet
                {
                    charPos = 18 + (11 * (cursor));	//to move through the text file value by value, which are 11 characters apart
                    lidarData[row][col] = stod(cur.substr(charPos, 11)); //getting distance value
                    cursor++;
                }
            }
            row++;
            col = 0; //reset this to 0. when it reads an angle value next, col will be set to the first column
        }

        //Seeks time_det at the beginning of a line, stores the time value and calculates the exact time for each data point, as
        //described in the VLP-16 documentation
        if (cur.substr(0, 5) == TIME_DET)
        {
            //cout << "time detected" << endl;
            curTime = stod(cur.substr(5, 11));
            //cout << curTime << endl;

            for (int i = 23; i > -1; i--)
            {
                lidarData[row - 24 + i][49] = curTime;

                for (int j = 1; j < 17; j++)
                {
                    int sequence_index = i;
                    int data_pt_index = j - 1;

                    double exact_time = curTime + (55.296 * sequence_index) + (2.304 * data_pt_index);
                    lidarData[row - 24 + i][j * 3] = exact_time;
                }
            }
        }

        //Seeks GPS_DET at the beginning of a line, stores the entire GPS sentence in a string matrix with each row being it's
        //own sentence. Details are in the VLP-16 documentation and Matrix Organization spreadsheet
        if (cur.substr(0, 4) == GPS_DET)
        {
            if (cur.substr(0, 8) != "GPS= $GP") //this conditional is to avoid an exception from being thrown when the lidar capture code has a typo in the GPS line
            {
                //TODO: have this continue to gather the GPS data after the system typo
                //TODO: turn this into a try-catch block
                print("GPS ERROR");
                break;
            }
            gpsTime = stod(cur.substr(12, 6));

            lidarGPS[gRow][0] = cur.substr(12, 6);	//GPS time
            lidarGPS[gRow][1] = cur.substr(19, 1);	//Validity, A or V
            lidarGPS[gRow][2] = cur.substr(21, 9);	//Current Latitude
            lidarGPS[gRow][3] = cur.substr(31, 1);	//N or S
            lidarGPS[gRow][4] = cur.substr(33, 10);	//Current Longitude
            lidarGPS[gRow][5] = cur.substr(44, 1);	//E or W
            lidarGPS[gRow][6] = cur.substr(46, 5);	//Speed in knots
            lidarGPS[gRow][7] = cur.substr(52, 5);	//True course
            lidarGPS[gRow][8] = cur.substr(58, 6);	//Date Stamp
            lidarGPS[gRow][9] = cur.substr(65, 5);	//Variation
            lidarGPS[gRow][10] = cur.substr(71, 1);	//E or W
            lidarGPS[gRow][11] = cur.substr(73, 4);	//checksum
            lidarGPS[gRow][12] = curTime;			//timestamp from LIDAR

            gRow++;

        }

    }

    print("DONE");

    //reset for the next while loop that takes in the IMU data
    row = 0;
    col = 0;

    print("Processing IMU data");

    while (getline(imuIFS, cur))
    {
        //cout << "gathering IMU data into matrix" << endl;
        imuData[row][0] = stod(cur.substr(0, 15));	//latitude
        imuData[row][1] = stod(cur.substr(16, 15));	//longitude
        imuData[row][2] = stod(cur.substr(31, 15));	//altitude
        imuData[row][3] = stod(cur.substr(46, 15)); //w
        imuData[row][4] = stod(cur.substr(61, 15)); //x
        imuData[row][5] = stod(cur.substr(76, 15)); //y
        imuData[row][6] = stod(cur.substr(91, 15)); //z
        imuData[row][7] = stod(cur.substr(106, 15)); //roll
        imuData[row][8] = stod(cur.substr(121, 15)); //pitch
        imuData[row][9] = stod(cur.substr(136, 15)); //yaw
        imuData[row][10] = stod(cur.substr(151, 21)); //time stamp

        //cout << "imu time: " << fixed << imuData[row][10] << endl;

        row++;
    }

    print("DONE");

#pragma region GEOREF MATH
    print("***Start math");

    bool timeFlag;
    int testTime = 500;
    double testAngle = 30;

    //roll0 = imuData[0][7];
    //pitch0 = imuData[0][8];
    //yaw0 = imuData[0][9];
    //imuTime0 = imuData[0][10];				//Initial time value from the start of the IMU data capture. The unit is milliseconds since the Epoch.
    //lidarTime0 = lidarData[0][3];			//Initial time value from the start of the LIDAR data capture. The unit is microseconds past the hour.

    for (int imuRow = 0; imuRow < nImuLines; imuRow++)
    {
        if (imuRow + 1 >= nImuLines || lRow + 1 >= nLidarLines) { print("IOOB SAVE"); break; } //prevents loop from throwing an index oob error

#pragma region "LET'S GET THOSE TIMESTAMPS"


        //get data from array
        imuTimeA = imuData[imuRow][10];
        imuTimeB = imuData[imuRow + 1][10];
        lidarTime = lidarData[lRow][lCol] + 20000000; //might be 20 seconds behind imu data

        //put the values on a comparable scale
        using ms = duration<double, milli>;
        sys_time<milliseconds> imuTA{ round<milliseconds>(ms{imuTimeA}) };
        sys_time<milliseconds> imuTB{ round<milliseconds>(ms{imuTimeB}) };

        //auto loc_s = make_zoned(current_zone(), utc_ms).get_local_time();
        auto imuTA_msPH = imuTA - floor<hours>(imuTA);
        auto imuTB_msPH = imuTB - floor<hours>(imuTB);

        while (microseconds(lidarTime) < imuTA_msPH) //go to next lidarTime until it's greater than imuTimeA
        {
            lCol = lCol + 3;	//The next data point's timestamp is three columns away. Refer to the Matrix organization document

            if (lCol > 48) //lCol has reached the end of the row
            {
                lRow++;
                lCol = 3;
            }

            lidarTime = lidarData[lRow][lCol];	//update lidarTime
        }

        while (microseconds(lidarTime) >= imuTA_msPH && microseconds(lidarTime) < imuTB_msPH)		//while the lidarTime is between the two imu ts, keep incrementing through lidarTime
        {
            timeFlag = false;

            //test << imuTA_msPH << "  |   ";
            //test << microseconds(lidarTime) << "  |   ";
            //test << imuTB_msPH;

            if (abs(imuTA_msPH - microseconds(lidarTime)) <= abs(imuTB_msPH - microseconds(lidarTime))) { //lidarTime is closer to imuA than imuB

                //test << "            <<<   " << abs(imuTA_msPH - microseconds(lidarTime));
                imuRowSelect = imuRow; //use imuTimeA

                if (abs(imuTA_msPH - microseconds(lidarTime)) < (microseconds(testTime))) {
                    timeFlag = true;
                    //cout << abs(imuTA_msPH - microseconds(lidarTime)) << endl;
                }

            }
            else {											//lidarTime is closer to imuB than imuA

                //test << "            >>>   " << abs(imuTB_msPH - microseconds(lidarTime));
                imuRowSelect = imuRow + 1;	//use imuTimeB

                if (abs(imuTB_msPH - microseconds(lidarTime)) < (microseconds(testTime))) {
                    timeFlag = true;
                    //cout << abs(imuTB_msPH - microseconds(lidarTime)) << endl;
                }

            }


            //system("PAUSE");

            if (timeFlag == true) {

                //begin pt cloud math
                //lat = imuData[imuRowSelect][0];
                //lon = imuData[imuRowSelect][1];
                //alt = imuData[imuRowSelect][2];
                //roll = ConvertToRadians(imuData[imuRowSelect][7]);
                //pitch = ConvertToRadians(imuData[imuRowSelect][8]);
                yaw = ConvertToRadians(imuData[imuRowSelect][9]);
                //cout << imuData[imuRowSelect][9] << " ............ " << ConvertToRadians(imuData[imuRowSelect][9]) << endl;

                //test << "     " << ConvertToDegrees(yaw) << endl;

                //lat = lat - lat0;
                //lon = lon - lon0;
                //alt = alt - alt0;

                //lat = lat / 60;
                //lon = lon / 60 * -1;
                //alt = alt / 1000; //convert m to mm

                //latLength = 111321.5432;
                //lonLength = 111321.5432 * cos(lat * PI / 180);

                //latOffset = lat * latLength * 1000; //why did I do  *1000?
                //lonOffset = lon * lonLength * 1000;

                alpha = lidarData[lRow][0] / 100;
                //alpha = 90 - alpha;				 		//this is the math I found in Michael's code
                alpha = ConvertToRadians(alpha);
                distance = lidarData[lRow][lCol - 2];
                timeStamp = lidarData[lRow][lCol];
                lzr = (lCol / 3) - 1;
                omega = laserAngle[lzr];

                if (distance == 0) {	//skipping the data point if the distance is zero
                    lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                    if (lCol > 48) { lRow++; lCol = 3; }

                    lidarTime = lidarData[lRow][lCol];

                    //test << endl;

                    continue;
                }

                X = distance * sin(alpha) * cos(omega);
                Y = distance * cos(omega) * cos(alpha);
                Z = -distance * sin(omega);


                //ALTERNATE FORMAT PRINTING
                //altFormat << setw(15) << setprecision(6) << fixed << right << X << " " << setw(15) << setprecision(6) << fixed << right << Y << " " << setw(15) << setprecision(6) << fixed << right << Z << " "
                //	<< setw(15) << setprecision(6) << fixed << right << roll << " " << setw(15) << setprecision(6) << fixed << right << pitch << " " << setw(15) << setprecision(6) << fixed << right << yaw << " "
                //	<< setw(15) << setprecision(9) << fixed << right << lat << " " << setw(15) << setprecision(9) << fixed << right << lon << " " << setw(15) << setprecision(9) << fixed << right << alt << endl;

                ////X transform (pitch + y_offset)
                //Y = Y * cos(pitch + y_offset) - Z * sin(pitch + y_offset);
                //Z = Y * sin(pitch + y_offset) + Z * cos(pitch + y_offset);

                //Y transform (roll)
                //X = X * cos(roll) - Z * sin(roll);
                //Z = -X * sin(roll) + Z * cos(roll);

                //test << "X: " << X << "   Y: " << Y << "   YAW: " << ConvertToDegrees(yaw);

                ////Z transform ( yaw )
                double X1 = X * cos(yaw) - Y * sin(yaw)/* + lonOffset*/;
                double Y1 = X * sin(yaw) + Y * cos(yaw)/* - latOffset*/;
                //double Z1 = Z /*+ alt*/;

                //test << "   X1: " << X1 << "   Y1: " << Y1 << endl;

                //FOR SCREEN SHOTS ONLY
                int y_offset;
                int altOffset;
                double X, Y, Z, Z1;

                //X transform (pitch + y_offset)
                X1 = X;
                Y1 = Y * cos(pitch) - Z * sin(pitch);
                Z1 = Y * sin(pitch) + Z * cos(pitch);

                //Y transform (roll)
                X = X1 * cos(roll) - Z1 * sin(roll);
                Y = Y1;
                Z = -X1 * sin(roll) + Z1 * cos(roll);

                //Z transform (yaw)
                X1 = X * cos(yaw) - Y * sin(yaw);
                Y1 = X * sin(yaw) + Y * cos(yaw);
                Z1 = Z;

                //Position offset
                X1 = X1 + lonOffset;
                Y1 = Y1 - latOffset;
                Z1 = Z1 + altOffset;

                //END FOR SCREEN SHOTS

                //test << "USED";
                //test << "   " << setw(13) << right << setprecision(8) << fixed << X << "    " << setw(13) << right << setprecision(8) << fixed << Y << "    " << setw(13) << right << setprecision(8) << fixed << Z << " " << yaw / PI * 180 << endl ;

                /*if (X > 1250) {
                    double x = X * cos(-yaw) - Y * sin(-yaw);
                    double y = X * sin(-yaw) + Y * cos(-yaw);

                    test << "X:   " << setprecision(8) << fixed << x << "    " << setprecision(8) << fixed << y << "    " << setprecision(8) << fixed << Z << " ";
                    test << "   yaw: " << yaw / PI * 180 << endl;
                }

                if (Y < -1000) {
                    double x = X * cos(-yaw) - Y * sin(-yaw);
                    double y = X * sin(-yaw) + Y * cos(-yaw);

                    test << "Y:   " << setprecision(8) << fixed << x << "    " << setprecision(8) << fixed << y << "    " << setprecision(8) << fixed << Z << " ";
                    test << "   yaw: " << yaw / PI * 180 << endl;
                }*/

                if (ConvertToDegrees(yaw) > testAngle) {
                    ptCloudOFS << setw(12) << right << setprecision(5) << fixed << X1 << " " << setw(12) << right << setprecision(5) << fixed << Y1 << " " << setw(12) << right << setprecision(5) << fixed << Z << " " << setw(12) << right << setprecision(3) << 100 << endl;
                }
                else {
                    ptCloudOFS << setw(12) << right << setprecision(5) << fixed << X1 << " " << setw(12) << right << setprecision(5) << fixed << Y1 << " " << setw(12) << right << setprecision(5) << fixed << Z << " " << setw(12) << right << setprecision(3) << 0 << endl;

                }
                //end pt cloud math


                //increment lidarTime here
                lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                lidarTime = lidarTime;

            }
            else {
                //increment lidarTime here
                lCol = lCol + 3;	//the next data point's timestamp is three columns away. Refer to the Matrix organization document
                if (lCol > 48) { lRow++; lCol = 3; }

                lidarTime = lidarData[lRow][lCol];
                lidarTime = lidarTime;
                printf("lidartime: %d", lidarTime);
                test << endl;
            }
        }
    }

#pragma endregion
}


int LineCount(ifstream& file)
{
    int dot_count = 1;
    int n = 0;
    string s;

    while (getline(file, s))
    {
        n++;
    }

    cout << "Lines counted: " << n << endl;

    return n;
}

double ConvertToRadians(double angle)
{
    return angle * PI / 180;
}

double ConvertToDegrees(double angle) {
    return angle / PI * 180;
}

void print(string s) {
    cout << s << endl;
}