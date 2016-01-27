/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#include <iostream>
#include <vector>

#include <boost/numeric/odeint.hpp>
#include "near_identity.h"
#include "traj_generator.h"
#include <chrono>






//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller {

    near_identity ni_;
    traj_func* traj_;
    

public:
    ni_controller( near_identity ni) :  ni_(ni) { }

    void setTrajFunc(traj_func* traj)
    {
      traj_ = traj;
    }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        traj_->dState(x,dxdt,t);
        ni_(x,dxdt,t);
    }
};






//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};
//]


traj_generator::traj_generator()
{

 abs_err_ = 1.0e-10;
 rel_err_ = 1.0e-6;
 a_x_ = 1.0;
 a_dxdt_ = 1.0;
   
  t0_ = 0.0;
  tf_ = 21.0;
  dt_ = .1;
  
  cp_ = 100;
  cd_ = 100;
  cl_ = 100;
  eps_ = .01;
 
}


void traj_generator::setNIParams(double cp, double cd, double cl, double eps)
{
  cp_ = cp;
  cd_ = cd;
  cl_ = cl;
  eps_ = eps;

}


void traj_generator::setIntegratorParams(double abs_err, double rel_err, double a_x, double a_dxdt)
{
  abs_err_ = abs_err;
  rel_err_ = rel_err;
  a_x_ = a_x;
  a_dxdt_ = a_dxdt;

}


void traj_generator::setFunc(traj_func* func)
{
  trajectory_func_ = func;
}


size_t traj_generator::run(state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times)
{
    return traj_generator::run(x0, x_vec, times, t0_, tf_, dt_, cp_, cd_, cl_, eps_);
}
  
size_t traj_generator::run(state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, double t0, double tf, double dt, double cp, double cd, double cl, double eps)
{
using namespace boost::numeric::odeint;

  if (x0[5] ==0);
  //error!
  
    

    //[ Observer samples
    
    
    size_t steps;

    x_vec.clear();
    times.clear();
    
    near_identity ni(cp,cd,cl,eps);

    
    ni_controller controller(ni);
    controller.setTrajFunc(trajectory_func_);
  

    {
     typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err_ , rel_err_ , a_x_ , a_dxdt_ ) );



    //[ equidistant observer calls with adaptive internal step size:
    steps = integrate_const( controlled_stepper , controller , x0 , t0, tf, dt, push_back_state_and_time( x_vec , times ) );
    
    }
    
    return steps;
  
}  
  


