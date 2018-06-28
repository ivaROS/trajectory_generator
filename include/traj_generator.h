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

#include <boost/numeric/odeint.hpp>

// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Twist.h>

//[ rhs_function
/* The type of container used to hold the state vector */
//typedef std::vector< double > state_type;


template<typename T, size_t N>
class TrajectoryState
{
  typedef std::vector<double> array;
  
protected:
  array data;
  
public:
  
  typedef typename array::value_type value_type;
  
  typedef typename array::iterator iterator;
  typedef typename array::const_iterator const_iterator;
  
  
  
  iterator begin() {return data.begin();}
  iterator end() {return data.end();}
  
  const_iterator begin() const {return data.begin();}
  const_iterator end() const {return data.end();}
  
  inline const double& operator[] (size_t n) const { return (data[n]); }
  inline double& operator[] (size_t n)  { return (data[n]); }
  
  TrajectoryState() : data(N)
  {
  }
  
  template<typename S>
  void to(S& obj)
  {
    return static_cast<T*>(this)->to<S>(obj);
  }
  
  template<typename S>
  void from(const S& obj)
  {
    return static_cast<T*>(this)->from<S>(obj);
  }
  
  bool checkState()
  {
    return static_cast<T*>(this)->checkState();
  }

/*  
  template<>
  void to<geometry_msgs::Point>()
  {
    geometry_msgs::Pose = static_cast<T*>(this)->to<geometry_msgs::Pose>(obj) 
    return pose.position;
  }
  
  template<>
  void to<geometry_msgs::Quaternion>()
  {
    geometry_msgs::Pose = static_cast<T*>(this)->to<geometry_msgs::Pose>(obj) 
    return pose.orientation;
  }
  
  template<>
  void to<geometry_msgs::Twist>()
  {
    geometry_msgs::Pose = static_cast<T*>(this)->to<geometry_msgs::Pose>(obj) 
    return pose.orientation;
  }
  */
  
  
  
};
/*
// A Functor
class increment
{
private:
  int num;
public:
  increment(int n) : num(n) {  }
  
  // This operator overloading enables calling
  // operator function () on objects of increment
  int operator () (int arr_num) const {
    return num + arr_num;
  }
};*/


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
template<typename T, typename state_type>
class traj_func {

public:
    
  void operator() ( const state_type &x , state_type &dxdt , const double  t  )
  {
    static_cast<T*>(this)->operator_impl(x, dxdt, t);
  }
    

};
//]









struct traj_params {

public:
    double tf=5,t0=0,dt=.1,abs_err=1.0e-10,rel_err=1.0e-6,a_x=1.0,a_dxdt=1.0;
    
    };

    
    
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

template <typename state_type, typename F>
class traj_generator {

private:
  
  //ni_controller controller_;
  
  traj_params default_params_;
  
  public:
  
  
  traj_generator()
  {
    
  }
  
  traj_params getDefaultParams()
  {
    return default_params_; 
  }
  
  void setDefaultParams(traj_params &new_params)
  {
    default_params_ = new_params;
  }
  
  size_t run(F& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times)
  {
    return traj_generator::run(func, x0, x_vec, times, default_params_);
  }
  
  size_t run(F& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, traj_params& params)
  {
    using namespace boost::numeric::odeint;
    
    size_t steps;
    
    x_vec.clear();
    times.clear();
    
    //func.init(x0);
    
    {
      typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
      typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
      controlled_stepper_type controlled_stepper(default_error_checker< double , range_algebra , default_operations >( params.abs_err, params.rel_err, params.a_x, params.a_dxdt) );
      
      //[ equidistant observer calls with adaptive internal step size:
      steps = integrate_const( controlled_stepper , func , x0 , params.t0, params.tf, params.dt, push_back_state_and_time<state_type>( x_vec , times ) );
    }
    
    return steps;
    
  }  
  



};


#endif  /* ! traj_generator.h seen */
