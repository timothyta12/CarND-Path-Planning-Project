#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Helper {
	vector<double> JMT(vector< double> start, vector <double> end, double T);
    
    double evaluateJMT(vector<double> coefficents, double T);
};

#endif