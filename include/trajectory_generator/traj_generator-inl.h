/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#include <vector>

#include <boost/numeric/odeint.hpp>

#include "traj_generator.h"



//[ integrate_observer
template <typename state_type>
class push_back_state_and_time
{
    public:
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

template<typename state_type>
traj_generator<state_type>::traj_generator()
{

  double abs_err, rel_err, a_x, a_dxdt;
  
  double t0, tf, dt;

  
  abs_err = 1.0e-10;
  rel_err = 1.0e-6;
  a_x = 1.0;
  a_dxdt = 1.0;
   
  t0 = 0.0;
  tf = 5.0;
  dt = .1;
  
  
  

  default_params_ = (traj_params) 
  { 
    .tf=tf, .t0=t0, .dt=dt, .abs_err=abs_err, .rel_err=rel_err, 
    .a_x=a_x, .a_dxdt=a_dxdt
  };
  
}

traj_params traj_generator<state_type>::getDefaultParams()
{
  return default_params_; 
}

void traj_generator<state_type>::setDefaultParams(traj_params &new_params)
{
  default_params_ = new_params;
}

size_t traj_generator<state_type>::run(traj_func<state_type>& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times)
{
  return traj_generator::run(func, x0, x_vec, times, default_params_);
}

size_t traj_generator<state_type>::run(traj_func<state_type>& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, traj_params& params)
{
using namespace boost::numeric::odeint;

    size_t steps;

    x_vec.clear();
    times.clear();
        
    func.init(x0);

    {
      typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
      typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
      controlled_stepper_type controlled_stepper(default_error_checker< double , range_algebra , default_operations >( params.abs_err, params.rel_err, params.a_x, params.a_dxdt) );

      //[ equidistant observer calls with adaptive internal step size:
      steps = integrate_const( controlled_stepper , controller , x0 , params.t0, params.tf, params.dt, push_back_state_and_time( x_vec , times ) );
    }
    
    return steps;
  
}  
  



