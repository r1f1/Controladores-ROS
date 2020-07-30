

#include <iostream>
#include <armadillo>
#include <iomanip>
#include <math.h>
#include <vector>
#include "def_pos_pkg/Trayectoria2.hpp"

using namespace std;
using namespace arma;


mat Tray::jtraj(float q0, float q1, float tf, int N){
	
	vec range(N);
    float delta = tf/float(N-1);

    for(float i=0; i<N; i++) {
        range.at(i) = i*delta;
    }

    rowvec tProv(N);
    float tscal = tf;
	for (float i = 0; i <= N-1; i++){
		tProv(i) = i/(N-1);
	}
	mat t = tProv.t();
	
    //cout << "pepe" << endl;
    float qd0 = 0;
    float qd1 = 0;
    
    float A = 6*(q1 - q0) - 3*(qd1+qd0)*tscal;
    float B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
    float C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
    float E = qd0*tscal;
    float F = q0;
    
    mat tt = join_horiz(pow(t,5), pow(t,4), pow(t,3), pow(t,2));
    tt = join_horiz(tt, t, ones<vec>(size(t)));
    mat c = {A, B, C, 0, E, F};
    
    mat qt = tt*c.t();

    mat c2 = {0, 5*A, 4*B, 3*C, 0, E};
    mat qdt = (tt*c2.t())/tscal;

    mat c3 = {0, 0, 20*A, 12*B, 6*C, 0};
    mat qddt = (tt*c3.t())/pow(tscal,2);

    mat pos = join_horiz(qt, qdt, qddt);
    
    vector<vector<float>> posVec;

    posVec.resize(200, std::vector<float>(18));

    for (float i = 0; i < 200; i++){
        for (float j = 0; j < 18; j++){
            posVec[i][j] = pos.at(i,j);
        }
    }

    return pos;
}