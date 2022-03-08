/*=======================================================================
		Accuracy Test Sample code for LC898124
                                             	by Rex.Tang
                                                2016.03.27
========================================================================*/

// Checking radius
//#define		RADIUS 		75							// 75um

// Parameter define
//#define		DEGSTEP		3							// Degree of one step (3Åã)
//#define		ACCURACY	3.0F						// Accuracy (Å}3.0um)
//#define		WAIT_MSEC	10							// Each step wait time(msec)
#define		LOOPTIME	3							// Read times at each step

// Constants
#define		PI			3.14159		// ÉŒ


typedef struct tag_Dual_Axis
{
	float xpos;
	float xhall;
	float ypos;
	float yhall;
}Dual_Axis_t;

