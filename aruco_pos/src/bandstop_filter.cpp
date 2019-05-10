#include "aruco_dec.h"

static float xv[11][2], yv[11][2];


void bandstop_filter(Vector3d& inputdata, Vector3d& outputdata)
{
	
	for(int i = 0;i<2;i++)
	{
			xv[0][i] = xv[1][i]; xv[1][i] = xv[2][i]; xv[2][i] = xv[3][i]; xv[3][i] = xv[4][i]; xv[4][i] = xv[5][i]; xv[5][i] = xv[6][i]; xv[6][i] = xv[7][i]; xv[7][i] = xv[8][i]; xv[8][i] = xv[9][i]; xv[9][i] = xv[10][i]; 
       			xv[10][i] = inputdata[i] / 2.588682783e+05;
        		yv[0][i] = yv[1][i]; yv[1][i] = yv[2][i]; yv[2][i] = yv[3][i]; yv[3][i] = yv[4][i]; yv[4][i] = yv[5][i]; yv[5][i] = yv[6][i]; yv[6][i] = yv[7][i]; yv[7][i] = yv[8][i]; yv[8][i] = yv[9][i]; yv[9][i] = yv[10][i]; 
        		yv[10][i] =   (xv[0][i] + xv[10][i]) -   0.0000000004 * (xv[1][i] + xv[9][i]) + 5 * (xv[2][i] + xv[8][i])
                     -   0.0000000014 * (xv[3][i] + xv[7][i]) + 10 * (xv[4][i] + xv[6][i]) -   0.0000000022 * xv[5][i]
                     + (  0.5679688348 * yv[0][i]) + (  0.0000000000 * yv[1][i])
                     + ( -3.1605155086 * yv[2][i]) + ( -0.0000000001 * yv[3][i])
                     + (  7.0547556806 * yv[4][i]) + (  0.0000000001 * yv[5][i])
                     + ( -7.8977394350 * yv[6][i]) + ( -0.0000000001 * yv[7][i])
                     + (  4.4354068132 * yv[8][i]) + (  0.0000000000 * yv[9][i]);
        outputdata[i] = yv[10][i];
	outputdata[2] = inputdata[2];
	}
}
