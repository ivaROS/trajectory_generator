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
#include <memory>


//[ rhs_function
/* The type of container used to hold the state vector */
//typedef std::vector< double > state_type;



//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_func 
{

public:
    virtual void init ( const state_type &x0 )
    {
        //std::cout << "This should only print if an init function is not defined" << std::endl;
    }
    
    virtual void dState ( const state_type &x , state_type &dxdt , const double  t  )=0;
    

};
//]

struct traj_params 
{

public:
    double tf,t0,dt,cp,cd,cl,eps,abs_err,rel_err,a_x,a_dxdt;
    
    traj_params(traj_params_ptr pntr)
    {
       tf = pntr->tf;
       t0 = pntr->t0;
       dt = pntr->dt;
       cp = pntr->cp;
       cd = pntr->cd;
       cl = pntr->cl;
       eps = pntr->eps;
       abs_err = pntr->abs_err;
       rel_err = pntr->rel_err;
       a_x = pntr->a_x;
       a_dxdt = pntr->a_dxdt;
    }
    
};

typedef std::shared_ptr<traj_params> traj_params_ptr;

class traj_generator {

  public:
  
  traj_generator();
  
  void setFunc(traj_func* func);
  
  std::size_t run(state_type &x0,  std::vector<state_type> &x_vec, std::vector<double> &times);
  std::size_t run(state_type &x0,  std::vector<state_type> &x_vec, std::vector<double> &times, traj_params_ptr params);
  std::size_t run(traj_func* func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times);
  std::size_t run(traj_func* func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, traj_params_ptr params);  
  
  traj_params_ptr getDefaultParams();
  traj_params_ptr copyDefaultParams();
  void setDefaultParams(traj_params_ptr params);
  
  private:
  
  //ni_controller controller_;
  
  double abs_err_, rel_err_, a_x_, a_dxdt_;
  
  double t0_, tf_, dt_;
  double cp_, cd_, cl_, eps_;
  
  traj_params_ptr default_params_;
  
  traj_func* trajectory_func_;
  
  std::vector<state_type> x_vec;
  std::vector<double> times;

};


#endif  /* ! traj_generator.h seen */
