#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <iostream>
#include <unsupported/Eigen/Polynomials>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <boost/timer.hpp>
#include <math.h>

using namespace Eigen;
using namespace std;

using namespace std::chrono;

typedef vector<double> vec;

struct SplineSet{
    double a;
    double b;
    double c;
    double d;
    double x;
};

vector<SplineSet> spline(vec &x, vec &y)
{
    int n = x.size()-1;
    vec a;
    a.insert(a.begin(), y.begin(), y.end());
    vec b(n);
    vec d(n);
    vec h;

    for(int i = 0; i < n; ++i)
        h.push_back(x[i+1]-x[i]);

    vec alpha;
    for(int i = 0; i < n; ++i)
        alpha.push_back( 3*(a[i+1]-a[i])/h[i] - 3*(a[i]-a[i-1])/h[i-1]  );

    vec c(n+1);
    vec l(n+1);
    vec mu(n+1);
    vec z(n+1);
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;

    for(int i = 1; i < n; ++i)
    {
        l[i] = 2 *(x[i+1]-x[i-1])-h[i-1]*mu[i-1];
        mu[i] = h[i]/l[i];
        z[i] = (alpha[i]-h[i-1]*z[i-1])/l[i];
    }

    l[n] = 1;
    z[n] = 0;
    c[n] = 0;

    for(int j = n-1; j >= 0; --j)
    {
        c[j] = z [j] - mu[j] * c[j+1];
        b[j] = (a[j+1]-a[j])/h[j]-h[j]*(c[j+1]+2*c[j])/3;
        d[j] = (c[j+1]-c[j])/3/h[j];
    }

    vector<SplineSet> output_set(n);
    for(int i = 0; i < n; ++i)
    {
        output_set[i].a = a[i];
        output_set[i].b = b[i];
        output_set[i].c = c[i];
        output_set[i].d = d[i];
        output_set[i].x = x[i];
    }
    return output_set;
}

int main()

{
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    vec x(50);
    vec y(50);
    for(int i = 0; i < x.size(); ++i)
    {
        x[i] = i;
        y[i] = sin(i);
    }

    vector<SplineSet> cs = spline(x, y);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();


    auto duration = duration_cast<microseconds>( t2 - t1 ).count();

    std::cout << "Duration: " << duration << std::endl;
    for(int i = 0; i < cs.size(); ++i)
        cout << cs[i].d << "\t" << cs[i].c << "\t" << cs[i].b << "\t" << cs[i].a << endl;
}
/*class SplineFunction {
  
public:
  SplineFunction(Eigen::VectorXd const &x_vec,
                 Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()),
      x_max(x_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1> >::Interpolate(
                y_vec.transpose(),
                3, // No more than cubic spline, but accept short vectors.
                scaled_values(x_vec)))
  { }

  double operator()(double x) const {
    // x values need to be scaled down in extraction as well.
    return spline_(scaled_value(x))(0);
  }

private:
  // Helpers to scale X values down to [0, 1]
  double scaled_value(double x) const {
    return (x - x_min) / (x_max - x_min);
  }

  Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
  }

  double x_min;
  double x_max;

  // Spline of one-dimensional "points."
  Eigen::Spline<double, 1> spline_;
};


int main(int argc, char const* argv[])
{
  Eigen::VectorXd xvals(3);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 15, 30;
  yvals << 0, 12, 17;

 // Eigen::Spline<double,1 ,3> s = Eigen::Spline<double, 1, 3>(yvals, xvals);

  SplineFunction s(xvals, yvals);

  std::cout << s(12.34) << std::endl;
}
*/

