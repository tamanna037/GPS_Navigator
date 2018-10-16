/*
 * gps.h
 *
 * Created: 6/5/2017 10:14:22 AM
 *  Author: DELL
 */


#ifndef GPS_H_
#define GPS_H_

#include <math.h>
#include "path.h"

#define true 1
#define false 0
#define pi 3.14159265358979323846
#define DEGRAD pi/180



char* getLatLong(char* gpsStr, int pos, char flag);
int getCurrlocation(char *gpsStr, TrackPoint* curr,TrackPoint* prev);
char getDir(double angle);
double deg2rad(double deg);
double rad2deg(double rad);
double ddmmToDecimal(char* data);
char getTurn(double theta);
double getAngle(double prevLat,double prevLon, double currLat,double currLon, double wpLat,double wpLon);

/************************************************************************/
/* finds angle given three point using vector Cross multiplication                                                                     */
/************************************************************************/
double getAngle(double prevLat,double prevLon, double currLat,double currLon, double wpLat,double wpLon){
	double a2 = currLat - prevLat;
	double a1 = currLon - prevLon;
	double b2 = wpLat - currLat;
	double b1 = wpLon - currLon;
	

	double aCrossb = a1*b2-a2*b1;
	double a = sqrt(a1*a1+a2*a2);
	double b = sqrt(b1*b1+b2*b2);
	
	double theta = asin(aCrossb/(a*b));
	
	return theta*180/pi;
}

/************************************************************************/
/* calculates Turn from angle found from vector cross multiplication                                                                     */
/************************************************************************/
char getTurn(double theta){
	if(theta < -30 && theta > -180)
	return RIGHT;
	else if(theta > 30 && theta <= 180)
	return LEFT;
	else
	return STRAIGHT;
}


/************************************************************************/
/* find distance between two point using radius of earth                                                                     */
/************************************************************************/
uint16_t distance(double lat1, double lon1, double lat2, double lon2, char unit) {
	double x = (lon2-lon1)*DEGRAD*cos((lat1+lat2)*DEGRAD/2);
	double y = (lat2-lat1)*DEGRAD;
	
	return (uint16_t)(sqrt(x*x + y*y)*ERADIUS);
}

double deg2rad(double deg) {
    return (deg * pi / 180);
}

double rad2deg(double rad) {
    return (rad * 180 / pi);
}

/************************************************************************/
/* converts lat long ddmm:mm format to dd format                                                                     */
/************************************************************************/
double ddmmToDecimal(char* data){
	double f = atof(data);
	double x = f-(int)f;
	double min = ((int)f)%100;
	//printf("%d\n",min);
	min += x;
	int dd = f/100;
	double result = dd+(min/60);
	return result;
}

/************************************************************************/
/* calculate true bearing of two points                                                                     */
/************************************************************************/
double calculateBearing(double lat, double lon, double next_lat, double next_lon)
{
    double dLong = next_lon-lon;
	double dy = sin(dLong*DEGRAD)*cos(next_lat*DEGRAD);
	double dx = cos(lat*DEGRAD)*sin(next_lat*DEGRAD)-
		sin(lat*DEGRAD)*cos(next_lat*DEGRAD)*cos(dLong*DEGRAD);
	double result = (atan2(dy,dx)*(180/pi));	// 57.3 = 180/pi

	if (result > 0)
		return result;
	else
		return 360+result;
}

/************************************************************************/
/* parses the NMEA string and get Lattitude and Longitude from it                                                                     */
/************************************************************************/

int getCurrlocation(char *gpsStr, TrackPoint* curr,TrackPoint* prev)
{
    char latdir,londir;
	int length = strlen(gpsStr);
	if(length < 26)
		return false;
	if(!(gpsStr[0]=='G' && gpsStr[1]=='P'))
		return false;
	int i=0;
	while(gpsStr[i])
	{
		if(gpsStr[i]=='N' || gpsStr[i]=='S')
		{
		    latdir = gpsStr[i];
			char* latittude = getLatLong(gpsStr,i,'N');
			if(latittude == NULL)
				return false;
			i++;
			while(gpsStr[i]!='E' && gpsStr[i]!='W'){
				if(gpsStr[i]==0)
					return false;
				i++;
			}
			londir = gpsStr[i];
			char* longitude = getLatLong(gpsStr,i,'E');
			if(longitude==NULL)
				return false;

            copyTrackPoint(curr,prev);
            printf("lat: %s lon: %s\n",latittude,longitude);
            curr->lat = ddmmToDecimal(latittude);
			curr->lon = ddmmToDecimal(longitude);
			if(latdir=='S'){
                curr->lat = - curr->lat;
			}
			if(londir == 'W'){
                curr->lon = - curr->lon;
			}
			return true;
		}
		else if (gpsStr[i]==0)
		{
			return false;
		}

		i++;
	}
	return false;

}

char* getLatLong(char* gpsStr, int pos, char flag){
	int i=pos;
	char* location = (char*)malloc(12*sizeof(char));
	i--;
	if(gpsStr[i]==',')
	{
		i--;
		if(flag == 'N'){
			while(gpsStr[i]!=','&& gpsStr[i]!='P'){
				if(i==0)
					return NULL;
				i--;
			}
		}else{
			while(gpsStr[i]!=','){
				if(i==1)
				return NULL;
				i--;
			}
		}
		i++;
		int count = 0;
		char dotFlag=0;
		while(gpsStr[i]!=',')
		{
			location[count]=gpsStr[i];
			if(gpsStr[i]=='.'){
				if(dotFlag == 1)
					return NULL;
				dotFlag = 1;
			}

			count++;
			i++;
			if(count==12)
				return NULL;
		}

		location[count]=0;
		int len=strlen(location);
		if(len<9 || len >11)
			return NULL;
		return location;
	}
	else
		return NULL;
}


/*
 * Finds the direction to turn to reach the active waypoint if off-track or
 * transitioning to the next waypoint. Only gives an indication to turn if
 * the user's bearing is more than 60 degrees from the expected bearing
 *
 * actual: the actual bearing of the user in integer degrees (0-360)
 * expected: the bearing from the user's current position to the active
 * 		waypoint in integer degrees (0-360)
 * next: an integer (0 or 1) to indicate if transitioning to a new waypoint
 *
 * return: an integer giving the direction to turn (if any) or an error
 * 		-2 = input out of bounds, -1 = turn left, 0 = straight, 1 = turn right
 * 		2 = do nothing (on track)
 */
int giveDirection(double actual, double expected,char next)
{
    	if (actual > 360 || expected > 360 || actual < 0 || expected < 0)
		return ERROR;	// Error (input must be in range [0,360])

        double diff = expected - actual;
        double absD = (diff < 0) ? -diff : diff;

        if (absD > TURN_IND || next == 1) {
            if (absD >= 0 && absD<31)
                return STRAIGHT;
            else if (absD < 180)
                return (diff < 0) ? LEFT : RIGHT;
            else if (absD == 180)
                return LEFT;	// Arbitrarily chose turn left
            else if (expected > actual) {
                if (((actual+360)-expected) > TURN_IND)
                    return LEFT;
            }
            else {
                if (((expected+360)-actual) > TURN_IND)
                    return RIGHT;
            }
        }

        return NO_CHANGE;
}

#endif /* GPS_H_ */
