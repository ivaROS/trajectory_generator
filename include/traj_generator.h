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
  
  
};


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
template <typename state_type>
class traj_func {

public:
    virtual void init ( const state_type &x0 )
    {
        //std::cout << "This should only print if an init function is not defined" << std::endl;
    }
    
    virtual void dState ( const state_type &x , state_type &dxdt , const double  t  )=0;
    

};
//]

struct traj_params {

public:
    double tf,t0,dt,eps,abs_err,rel_err;
    
    };


template <typename state_type>
class traj_generator {

  public:
  
  traj_generator();
  
  std::size_t run(traj_func& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times);
  std::size_t run(traj_func& func, state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times, traj_params& params);  
  
  //Passing parameters by value here
  traj_params getDefaultParams();
  void setDefaultParams(traj_params& params);
  
  private:
  
  //ni_controller controller_;
  
  traj_params default_params_;


};


#endif  /* ! traj_generator.h seen */
