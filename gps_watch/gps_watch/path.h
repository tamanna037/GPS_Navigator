/*
 * path.h
 *
 * Created: 6/6/2017 9:31:35 AM
 *  Author: DELL
 */


#ifndef PATH_H_
#define PATH_H_

#define NEAR_DIST 20 //METER
#define INIT_DIST 25
#define LEFT 2
#define RIGHT 1
#define STRAIGHT 0
#define NO_CHANGE 3
#define TURN_IND 60
#define ERROR 4

#define ERADIUS 6371000	// Approximate Earth radius in meters


typedef struct tp{
	double lat;
	double lon;
}TrackPoint;

typedef struct{
    TrackPoint* tp;
    uint16_t distToWp; //meter
}CurrentLocation;

void copyTrackPoint(TrackPoint* from, TrackPoint* to ){
    to->lat = from->lat;
    to->lon = from->lon;
}

void initTrackPoint(TrackPoint* tp){
    tp->lat = 0;
    tp->lon = 0;
}

void printTrackPoint(TrackPoint* tp){
    printf("lat: %lf lon: %lf\n",tp->lat,tp->lon);
}

/************************************************************************/
/* route selected by runner                                                                     */
/************************************************************************/

double path[5][2]=
{

	{23.726511  ,90.388312},

	{23.726520	,90.388575},

	{23.726545	,90.388946},

	{23.7263368	,90.389059},

	{23.726212	,90.389231}
};



#endif /* PATH_H_ */
