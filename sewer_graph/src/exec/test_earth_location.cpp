// This file loads a flight plan in relative coordinates and converts it to Earth coordinates

#include "sewer_graph/earthlocation.h"
#include <functions/functions.h>
#include <iostream>
#ifdef USE_KML
#include "kml/engine.h"
#include "kml/base/file.h"
#endif
#include <math.h>
#include <stdio.h>
#include <functions/ArgumentData.h>

using namespace std;
using namespace functions;
using sewer_graph::EarthLocation;

void fancy_test();

void rotate_plan(vector<double> &plan, double rad, double x_c, double y_c);
void shift_plan(vector<double> &plan, double x_shift, double y_shift);
double get_min_y(const vector<double> &plan);

int main(int argc, char **argv) {
	fancy_test();
	 
	return 0;
}

void fancy_test()
{
	EarthLocation loc1(37.14677, -5.78727 );
	
        cout.precision(15);
        
	cout << "Departure location: " << loc1.toString() << endl;
	
	EarthLocation loc2(loc1);
	loc2.shift(700, 0.0);
	
	cout << "Center location: " << loc2.toString() << endl;
	cout << "Center location (decimal degrees): " << loc2.getLatitude() << ", " ;
	cout << loc2.getLongitude()<< endl;
	
	cout << "Distance between two points: " << loc1.distance(loc2) << endl;
}

void rotate_plan(vector<double> &plan, double rad, double x_c, double y_c) {
	shift_plan(plan, -x_c, -y_c);
	
// 	cout << "Shifted flight plan: " << aux::printVector(plan) << endl;
	
	for (int i = 0; i < plan.size() - 1; i += 2) {
		double x_old = plan.at(i);
		double y_old = plan.at(i + 1);
		
// 		cout << "x_old = " << x_old << "\t y_old = " << y_old << "\t";
		
		plan[i] = x_old * cos(rad) - y_old * sin(rad);
		plan[i + 1] = x_old * sin(rad) + y_old * cos(rad);
		
// 		cout << "x_new = " << plan[i] << "\t y_new = " << plan[i+1] << endl;
	}
	
// 	cout << "Rotated 1 flight plan: " << aux::printVector(plan) << endl;
	
	shift_plan(plan, x_c, y_c);
}

void shift_plan(vector<double> &plan, double x_shift, double y_shift) {
	for (int i = 0; i < plan.size() - 1; i += 2) {
		plan[i] += x_shift;
		plan[i + 1] += y_shift;
	}
	
}

double get_min_y(const vector<double> &plan) {
	double min = 1e30;
	
	for (int i = 0; i < plan.size() - 1; i += 2) {
		min = functions::minimum(plan.at(i + 1), min);
	}
	
	return min;
}
