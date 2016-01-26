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
class traj_gen : public virtual traj_func{

    double m_amp;
    double m_f;

public:
    traj_gen( double amp, double f ) : m_amp(amp), m_f(f) { }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = 1;
        dxdt[7] = std::sin(t*2.0*3.14*m_f) * m_amp;
        std::cout << "hi";
    }
};
//]




//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller {

    near_identity ni_;
    traj_func traj_;
    

public:
    ni_controller( near_identity ni) :  ni_(ni) { }

    void setTrajFunc(traj_func traj)
    {
      traj_ = traj;
    }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        traj_(x,dxdt,t);
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
 
}

void traj_generator::setIntegratorParams(double abs_err, double rel_err, double a_x, double a_dxdt)
{
  abs_err_ = abs_err;
  rel_err_ = rel_err;
  a_x_ = a_x;
  a_dxdt_ = a_dxdt;

}


void traj_generator::setFunc(traj_func func)
{
  trajectory_func_ = func;
}

  
size_t traj_generator::run(state_type &x0, std::vector<state_type> &x_vec, std::vector<double> &times)
{
using namespace boost::numeric::odeint;

  if (x0[5] ==0);
  //error!
  
    

    //[ Observer samples
    
    
    size_t steps;

    x_vec.clear();
    times.clear();
    
    near_identity ni(100,100,100,.01);

    
    ni_controller controller(ni);
    controller.setTrajFunc(trajectory_func_);
  

    {
     typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err_ , rel_err_ , a_x_ , a_dxdt_ ) );



    //[ equidistant observer calls with adaptive internal step size:
    steps = integrate_const( controlled_stepper , controller , x0 , t0_, tf_, dt_, push_back_state_and_time( x_vec , times ) );
    
    }
    
    return steps;
  
}  
  




int main(int /* argc */ , char** /* argv */ )
{
    using namespace std;
    using namespace boost::numeric::odeint;


    //[ initial state
    state_type x0(8); 
    x0[0] = 1.0; //x
    x0[1] =-2.0; //y
    x0[2] = 0.0; //theta
    x0[3] = 0.0; //v
    x0[4] = 0.0; //w
    x0[5] = 1.0; //lambda: must be > 0!
    x0[6] = 0.0; //x_d
    x0[7] = 0.0; //y_d
    //]
    



    //double abs_err_ = 1.0e-10 , rel_err_ = 1.0e-6 , a_x_ = 1.0 , a_dxdt_ = 1.0;
    
    traj_generator trajectory_gen;
    traj_gen traj(0.15,.1);
    
    traj_func* trajpntr = &traj;
    
    trajectory_gen.setFunc(*trajpntr);
    
    
    
    //[ Observer samples
    vector<state_type> x_vec;
    vector<double> times;
    
    size_t steps;


    //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();

    steps = trajectory_gen.run(x0, x_vec, times);


    
    //How long did it take? 
    //Stop the clock before all of the printouts
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    std::cout << "Integration took " << fp_ms.count() << " ms\n";



    std::cout<< "const observer: "  << steps << " steps. final: " << '\t' << x0[0] << '\t' << x0[1]<< std::endl;
    /* output */
    std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << 'x' << '\t' << 'y' << std::endl;
    
    for( size_t i=0; i<=steps; i++ )
    {
        double error_x = x_vec[i][0] - x_vec[i][6];
        double error_y = x_vec[i][1] - x_vec[i][7];
        
        double error = std::sqrt(error_x*error_x + error_y*error_y);
        printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
        
        //cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << "\t\t" << x_vec[i][2]<< "\t\t" << x_vec[i][3]<< "\t\t" << x_vec[i][4]<< '\t' << x_vec[i][5]<< '\t' << x_vec[i][6]<< '\t' << x_vec[i][7] << '\n';
    }
    //]




    


}
