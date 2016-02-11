/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */

#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H
     
#include "near_identity.h"
#include <iostream>     // std::cout

#include <vector>



//[ rhs_function
/* The type of container used to hold the state vector */
//typedef std::vector< double > state_type;



//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_func {

public:
    virtual void init ( const state_type &x0 )
    {
        std::cout << "This should only print if an init function is not defined" << std::endl;
    }
    
    virtual void dState ( const state_type &x , state_type &dxdt , const double  t  )=0;
    

};
//]



class traj_generator {

  public:
  
  traj_generator();
  
  void setFunc(traj_func* func);
  
  std::size_t run(state_type &x0,  std::vector<state_type> &x_vec, std::vector<double> &times);
  std::size_t run(state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, double t0, double tf);
  std::size_t run(state_type &x0,  std::vector<state_type> &x_vec, std::vector<double> &times, double t0, double tf, double dt, double cp, double cd, double cl, double eps);
  
  void setIntegratorParams(double abs_err, double rel_err, double a_x, double a_dxdt);
  
  void setNIParams(double cp, double cd, double cl, double eps);
  
  void setTimeParams(double t0, double tf, double dt);
  
  private:
  
  //ni_controller controller_;
  
  double abs_err_, rel_err_, a_x_, a_dxdt_;
  
  double t0_, tf_, dt_;
  double cp_, cd_, cl_, eps_;
  
  traj_func* trajectory_func_;
  
  std::vector<state_type> x_vec;
  std::vector<double> times;

};


#endif  /* ! traj_generator.h seen */
