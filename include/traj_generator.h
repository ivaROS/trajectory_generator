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
     


#include <vector>



//[ rhs_function
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_func {

public:

    virtual void dState ( const state_type &x , state_type &dxdt , const double  t  )=0;
};
//]



class traj_generator {

  public:
  
  traj_generator();
  
  void setFunc(traj_func* func);
  
  size_t run(state_type &x0,  std::vector<state_type> &x_vec, std::vector<double> &times);
  
  void setIntegratorParams(double abs_err_, double rel_err_, double a_x_, double a_dxdt_);
  
  private:
  
  //ni_controller controller_;
  
  double abs_err_, rel_err_, a_x_, a_dxdt_;
  
  double t0_, tf_, dt_;
  
  traj_func* trajectory_func_;
  
  std::vector<state_type> x_vec;
  std::vector<double> times;

};


#endif  /* ! traj_generator.h seen */
