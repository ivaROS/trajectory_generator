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



//[ rhs_function
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;

const double m_amp = 0.15;
const double f=.1;



//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class harm_osc {

    double m_amp;
    double m_f;

public:
    harm_osc( double amp, double f ) : m_amp(amp), m_f(f) { }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[0] = 1;
        dxdt[1] = std::sin(t*2.0*3.14*f) * m_amp;
    }
};
//]





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


    //[ state_initialization
    state_type x0(2);
    x0[0] = 0.0; // start at x=1.0, p=0.0
    x0[1] = 0.0;
    //]
    
    const double t0 = 0.0;
    const double tf = 21.0;
    const double observer_dt = 5.0;

    state_type x = x0;
    

    //[ Observer samples
    vector<state_type> x_vec;
    vector<double> times;
    
    size_t steps;

    x_vec.clear();
    times.clear();

    double abs_err = 1.0e-10 , rel_err = 1.0e-6 , a_x = 1.0 , a_dxdt = 1.0;
    harm_osc ho(0.15,.1);
       x = x0;

    {
     typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err , rel_err , a_x , a_dxdt ) );



    //[ equidistant observer calls with adaptive internal step size:
    steps = integrate_const( controlled_stepper , ho , x , t0, tf, observer_dt, push_back_state_and_time( x_vec , times ) );
    
    }


    std::cout<< "const observer: "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;
    /* output */
    for( size_t i=0; i<=steps; i++ )
    {
        cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
    }
    //]




    


}
