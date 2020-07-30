
#include <iostream>
#include <armadillo>
#include <iomanip>
#include <math.h>
#include <vector>

using namespace std;
using namespace arma;

class Tray
{
public:
	Tray() = default;
	mat jtraj(float q0, float q1, float tf, int N);
};