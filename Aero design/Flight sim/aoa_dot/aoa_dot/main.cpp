#include <iostream>
#include <iomanip>

using namespace std;

double Sa = 0.1;
double rho = 1.225;
double m = 2;
double dCDdaoa = 0;
double dCLdaoa = 3.14;
double C_L0 = 0;
double C_D0 = 0.01;

double* linspace(double nStart,double nEnd,int n)
{
	// Initialize a pointer to an address with length n
	double* arr = new double[n];

	double iter = (nEnd - nStart) / (n-1);
	for (int ii = 0; ii < n; ++ii) {
		arr[ii] = nStart + (iter * ii);
	}
	return arr;
}

double aoa_dot(double vx,double vy,double th,double w,double aoa)
{
	// parent equation
	double f = w-(((rho*Sa)/(2*m))*(vx*((dCLdaoa*aoa+C_L0)*cos(th-aoa))-((dCDdaoa*aoa+C_D0)*sin(th-aoa))-vy*((dCLdaoa*aoa+C_L0)*sin(th-aoa))-((dCDdaoa*aoa+C_D0)*cos(th-aoa)))/(vx*vx));
	return f;
}

int main()
{
	const int n_vx =  15;
	const int n_vy =  15;
	const int n_th =  15;
	const int n_w =   15;
	const int n_aoa = 15;

	double* bound_vx = linspace( 0.1, 50, n_vx);
	double* bound_vy = linspace( - 2, 2, n_vy);
	double* bound_th = linspace(-0.087266, 0.1745, n_th);
	double* bound_w = linspace( -0.2618, 0.2618, n_w);
	double* bound_aoa = linspace( -0.087266, 0.1745, n_aoa);

	//do things here
	double C_point[5] = {bound_vx[(n_vx-1)/2],bound_vy[(n_vy - 1) / 2],bound_th[(n_th - 1) / 2],bound_w[(n_w - 1) / 2],bound_aoa[(n_aoa - 1) / 2]};

	//equation goes here
	float numOfCalcs = n_vx * n_vy;
	float n = 0;

	double aoadot[n_vx][n_vy][n_th][n_w][n_aoa];

	cout << setprecision(3);

	for (int ii = 0; ii < n_vx; ++ii) { //x velocity
		for (int jj = 0; jj < n_vy; ++jj) { //y velocity

			//Loading update
			float perc = 100 * n / numOfCalcs;
			++n;
			cout << "Loading... " << perc << "%\n";

			for (int kk = 0; kk < n_th; ++kk) { //theta radians
				for (int ll = 0; ll < n_w; ++ll) { //omega radians/second
					for (int mm = 0; mm < n_aoa; ++mm) { //angle of attack radians
						aoadot[ii][jj][kk][ll][mm] = aoa_dot(bound_vx[ii], bound_vy[jj], bound_th[kk], bound_w[ll], bound_aoa[mm]);
					}
				}
			}
		}
	}

	//stop doing things here

	delete aoadot;
	delete bound_vx;
	delete bound_vy;
	delete bound_th;
	delete bound_w;
	delete bound_aoa;

	return 0;
}
