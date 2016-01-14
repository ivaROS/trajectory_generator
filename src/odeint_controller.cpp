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
#include <chrono>



//[ rhs_function
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_gen {

    double m_amp;
    double m_f;

public:
    traj_gen( double amp, double f ) : m_amp(amp), m_f(f) { }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = 1;
        dxdt[7] = std::sin(t*2.0*3.14*m_f) * m_amp;
    }
};
//]




//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller {

    near_identity ni_;
    traj_gen traj_;
    

public:
    ni_controller( near_identity ni, traj_gen traj) :  ni_(ni), traj_(traj) { }

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



int main(int /* argc */ , char** /* argv */ )
{
    using namespace std;
    using namespace boost::numeric::odeint;


    //[ initial state
    state_type x0(8); 
    x0[0] = 0.0; //x
    x0[1] = 0.0; //y
    x0[2] = 0.0; //theta
    x0[3] = 0.0; //v
    x0[4] = 0.0; //w
    x0[5] = 1.0; //lambda: must be > 0!
    x0[6] = 0.0; //x_d
    x0[7] = 0.0; //y_d
    //]
    
    const double t0 = 0.0;
    const double tf = 21.0;
    const double observer_dt = .1;

    state_type x = x0;
    

    //[ Observer samples
    vector<state_type> x_vec;
    vector<double> times;
    
    size_t steps;

    x_vec.clear();
    times.clear();

    double abs_err = 1.0e-10 , rel_err = 1.0e-6 , a_x = 1.0 , a_dxdt = 1.0;
    
    near_identity ni(100,100,100,.01);
    traj_gen traj(0.15,.1);
    
    ni_controller ho(ni, traj);
    


    //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();


    x = x0;

    {
     typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err , rel_err , a_x , a_dxdt ) );



    //[ equidistant observer calls with adaptive internal step size:
    steps = integrate_const( controlled_stepper , ho , x , t0, tf, observer_dt, push_back_state_and_time( x_vec , times ) );
    
    }
    
    //How long did it take? 
    //Stop the clock before all of the printouts
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    std::cout << "Integration took " << fp_ms.count() << " ms\n";



    std::cout<< "const observer: "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;
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
