/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */

#ifndef TRAJECTORY_GENERATOR_TRAJ_GENERATOR_H
#define TRAJECTORY_GENERATOR_TRAJ_GENERATOR_H

#include <vector>
#include <boost/numeric/odeint.hpp>

namespace trajectory_generator
{

template <typename T>
class VectorReference
{
private:
  T& vector_;
  typedef typename T::value_type S;
  int ind_;
  
public:
  VectorReference(T& vector, int ind) : vector_(vector), ind_(ind) {}
  
  VectorReference(const VectorReference<T>& other) : vector_(other.vector_), ind_(other.ind_) {}
  
  operator S& ()
  {
    return vector_[ind_];
  }
  
  operator const S&() const
  {
    return vector_[ind_];
  }
  
  VectorReference<T>& operator= (const S& other)
  {
    vector_[ind_] = other;
    return *this; // Apparently it should return itself: https://en.wikipedia.org/wiki/Assignment_operator_(C%2B%2B)
  }
};

template<typename T, size_t N>
class TrajectoryState
{
public:
  //typedef std::vector<double> array;
  typedef boost::array<double, N> array;
  
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
  
  TrajectoryState() : data({}) {}
  
  TrajectoryState(const TrajectoryState& state) : data(state.data) {}
  
  template<typename S>
  void to(S& obj)
  {
    return static_cast<T*>(this)->template to< S >(obj);
  }
  
  template<typename S>
  void from(const S& obj)
  {
    return static_cast<T*>(this)->template from< S >(obj);
  }
  
  template<typename S>
  void operator=(const S& obj)
  {
    from(obj);
  }
  
  bool checkState()
  {
    return static_cast<T*>(this)->checkState();
  }
  
};


template<typename T>
struct Desired
{
  T& data;
  
  Desired(T& data) :
    data(data)
  {}
  
  operator T&()
  {
    return data;
  }
};


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
template<typename T, typename state_type>
class traj_func 
{
public:
    
  void operator() ( const state_type &x , state_type &dxdt , const double  t  )
  {
    static_cast<T*>(this)->operator_impl(x, dxdt, t);
  }
};
//]


struct traj_params 
{
    double tf=5,t0=0,dt=.1,abs_err=1.0e-10,rel_err=1.0e-6,a_x=1.0,a_dxdt=1.0;
};


/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */
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
class traj_generator 
{
public:  
  
  traj_generator() {}
  
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
  
  
  static size_t run(F& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, traj_params& params)
  { 
    using namespace boost::numeric::odeint;
    
    size_t steps=0;
    
    x_vec.clear();
    times.clear();
    
    //func.init(x0);
    
    if(x0.isValid())
    {
      typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
      typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
      controlled_stepper_type controlled_stepper(default_error_checker< double , range_algebra , default_operations >( params.abs_err, params.rel_err, params.a_x, params.a_dxdt) );
      
      //[ equidistant observer calls with adaptive internal step size:
      steps = integrate_const( controlled_stepper , func , x0 , params.t0, params.tf, params.dt, push_back_state_and_time<state_type>( x_vec , times ) );
    }

    return steps;
  }
  
private:
  traj_params default_params_;
};

} //namespace trajectory_generator

#endif  /* TRAJECTORY_GENERATOR_TRAJ_GENERATOR_H */
