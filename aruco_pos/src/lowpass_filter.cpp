#include "aruco_dec.h"
 /* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 5 -a 1.6666666667e-02 0.0000000000e+00 -l */


static float xv_lowpass[5+1][2] = {0}, yv_lowpass[5+1][2] = {0};

void lowpass_filter(Vector3d& inputdata, Vector3d& outputdata)
{
	for(int i = 0;i<3;i++)
	{
	xv_lowpass[0][i] = xv_lowpass[1][i]; xv_lowpass[1][i] = xv_lowpass[2][i]; xv_lowpass[2][i] = xv_lowpass[3][i]; xv_lowpass[3][i] = xv_lowpass[4][i]; xv_lowpass[4][i] = xv_lowpass[5][i]; 
        xv_lowpass[5][i] = inputdata[i] / 2.996744537e+06;
        yv_lowpass[0][i] = yv_lowpass[1][i]; yv_lowpass[1][i] = yv_lowpass[2][i]; yv_lowpass[2][i] = yv_lowpass[3][i]; yv_lowpass[3][i] = yv_lowpass[4][i]; yv_lowpass[4][i] = yv_lowpass[5][i]; 
        yv_lowpass[5][i] =   (xv_lowpass[0][i] + xv_lowpass[5][i]) + 5 * (xv_lowpass[1][i] + xv_lowpass[4][i]) + 10 * (xv_lowpass[2][i] + xv_lowpass[3][i])
                     + (  0.7124312835 * yv_lowpass[0][i]) + ( -3.8035532257 * yv_lowpass[1][i])
                     + (  8.1313017248 * yv_lowpass[2][i]) + ( -8.7013552442 * yv_lowpass[3][i])
                     + (  4.6611647833 * yv_lowpass[4][i]);
        outputdata[i] = yv_lowpass[5][i];
	}
}
