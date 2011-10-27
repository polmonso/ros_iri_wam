// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include <robot_pose_ekf/nonlinearanalyticconditionalgaussianodo.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianOdo::NonLinearAnalyticConditionalGaussianOdo(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE),
      df(6,6)
  {
    // initialize df matrix 6x6
    for (unsigned int i=1; i<=6; i++){
      for (unsigned int j=1; j<=6; j++){
        if (i==j) df(i,j) = 1;
        else df(i,j) = 0;
      }
    }
  }


  NonLinearAnalyticConditionalGaussianOdo::~NonLinearAnalyticConditionalGaussianOdo(){}

  ColumnVector NonLinearAnalyticConditionalGaussianOdo::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);

    state(1) += vel(1) * cos(state(5)) * cos(state(6));
    state(2) += vel(2) * cos(state(5)) * sin(state(6));
    state(3) += vel(3) * sin(state(5));
    state(4) += vel(4);
    state(5) += vel(5);
    state(6) += vel(6);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianOdo::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
    {
      ColumnVector state = ConditionalArgumentGet(0);
      ColumnVector vel  = ConditionalArgumentGet(1);

      df(1,1) = df(2,2) = df(3,3) = df(4,4) = df(5,5) = df(6,6) = 1;
      df(1,5) = -vel(1) * cos(state(5)) * sin(state(6));
      df(1,6) = -vel(1) * sin(state(5)) * cos(state(6));
      df(2,5) = -vel(1) * sin(state(5)) * sin(state(6));
      df(2,6) =  vel(1) * cos(state(5)) * cos(state(6));
      df(3,5) =  vel(1) * cos(state(5));

      return df;
    }
    else
    {
      if (i >= NumConditionalArgumentsGet())
        {
          cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
          exit(-BFL_ERRMISUSE);
        }
      else{
        cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
        exit(-BFL_ERRMISUSE);
      }
    }
  }

}//namespace BFL

