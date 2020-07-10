/*

	July 9, He Zhang, hzhang8@vcu.edu  

*/

#include <map>
#include "sfm.h"

using namespace std; 

pair<double, double> mean_std(vector<double>& vdpt)
{
	if(vdpt.size() > 1)
	{
		double sum = std::accumulate(std::begin(vdpt), std::end(vdpt), 0.0);
		double m =  sum / vdpt.size();

		double accum = 0.0;
		std::for_each (std::begin(vdpt), std::end(vdpt), [&](const double d) {
		    accum += (d - m) * (d - m);
		});

		double stdev = sqrt(accum / (vdpt.size()-1));
		return make_pair(m, stdev); 
	}
	return make_pair(0,0);
}

