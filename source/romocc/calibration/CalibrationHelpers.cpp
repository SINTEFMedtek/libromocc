#include "CalibrationHelpers.h"

namespace romocc
{

std::vector<Transform3d> invert_matrices(std::vector<Transform3d> matrices)
{
    std::vector<Transform3d> invertedMatrices;

    for(int i = 0; i<matrices.size(); i++)
    {
        invertedMatrices.push_back(matrices.at(i).inverse());
    }
    return invertedMatrices;
}

double compute_average(std::vector<double> const &v)
{
    double sum = 0;
    for(int i=0; i<v.size(); i++)
        sum += v[i];
    return sum/v.size();
}

double compute_variance(std::vector<double> const &v, double mean)
{
    double sum = 0.0;
    double temp =0.0;
    double var =0.0;

    for ( int j =0; j <= v.size()-1; j++)
    {
        temp = std::pow(v[j]-mean,2);
        sum += temp;
    }

    return var = sum/(v.size()-2);
}

std::vector<double> compute_linspace(double a, double b, int n)
{
    std::vector<double> array;
    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;
    }
    return array;
}

double sgn(double val)
{
    return (double(0) < val) - (val < double(0));
}

}