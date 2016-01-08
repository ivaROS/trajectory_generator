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

/* The rhs of x' = f(x) */
void harmonic_oscillator( const state_type &x , state_type &dxdt , const double  t  )
{
    dxdt[0] = 1;
    dxdt[1] = std::sin(t*2.0*3.14*f) * m_amp;
}
//]





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

struct write_state
{
    void operator()( const state_type &x ) const
    {
        std::cout << x[0] << "\t" << x[1] << "\n";
    }
};


int main(int /* argc */ , char** /* argv */ )
{
    using namespace std;
    using namespace boost::numeric::odeint;


    //[ state_initialization
    state_type x(2);
    x[0] = 0.0; // start at x=1.0, p=0.0
    x[1] = 0.0;
    //]



    //[ integration
    size_t steps = integrate( harmonic_oscillator ,
            x , 0.0 , 20.0 , 0.1 );
    //]

    std::cout<< "Basic version: " << steps << " steps" << std::endl;



    //[ integration_class
    harm_osc ho(1,.1);
    steps = integrate( ho ,
            x , 0.0 , 20.0 , 0.1 );
    //]

    std::cout<< "Class version"  << steps << " steps" << std::endl;




    //[ integrate_observ
    vector<state_type> x_vec;
    vector<double> times;

    steps = integrate( harmonic_oscillator ,
            x , 0.0 , 20.0 , 0.1 ,
            push_back_state_and_time( x_vec , times ) );

    std::cout<< "Class with observations"  << steps << " steps" << std::endl;

    /* output */
    for( size_t i=0; i<=steps; i++ )
    {
        cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
    }
    //]







    //[ define_const_stepper
    runge_kutta4< state_type > stepper;
    integrate_const( stepper , harmonic_oscillator , x , 0.0 , 20.0 , 0.01 );
    //]

    std::cout<< "const stepper"  << steps << " steps" << std::endl;


    //[ integrate_const_loop
    const double dt = 0.01;
    for( double t=0.0 ; t<20.0 ; t+= dt )
        stepper.do_step( harmonic_oscillator , x , t , dt );
    //]

    std::cout<< "const stepper"  << steps << " steps" << std::endl;




    //[ define_adapt_stepper
    typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    //]



    //[ integrate_adapt
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper;
    integrate_adaptive( controlled_stepper , harmonic_oscillator , x , 0.0 , 20.0 , 0.01 );
    //]

    {
    //[integrate_adapt_full
    double abs_err = 1.0e-10 , rel_err = 1.0e-6 , a_x = 1.0 , a_dxdt = 1.0;
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err , rel_err , a_x , a_dxdt ) );
    integrate_adaptive( controlled_stepper , harmonic_oscillator , x , 0.0 , 20.0 , 0.01 );
    //]
    }
    std::cout<< "adaptive version"  << std::endl;



    //[integrate_adapt_make_controlled
    integrate_adaptive( make_controlled< error_stepper_type >( 1.0e-10 , 1.0e-6 ) , 
                        harmonic_oscillator , x , 0.0 , 10.0 , 0.01 );
    //]

    std::cout<< "controlled version"  << std::endl;



    //[integrate_adapt_make_controlled_alternative
    integrate_adaptive( make_controlled( 1.0e-10 , 1.0e-6 , error_stepper_type() ) , 
                        harmonic_oscillator , x , 0.0 , 10.0 , 0.01 );
    //]

    #ifdef BOOST_NUMERIC_ODEINT_CXX11
    //[ define_const_stepper_cpp11
    {
    runge_kutta4< state_type > stepper;
    integrate_const( stepper , []( const state_type &x , state_type &dxdt , double t ) {
            dxdt[0] = x[1]; dxdt[1] = -x[0] - gam*x[1]; }
        , x , 0.0 , 10.0 , 0.01 );
    }
    //]
    
    
    
    //[ harm_iterator_const_step]
    std::for_each( make_const_step_time_iterator_begin( stepper , harmonic_oscillator, x , 0.0 , 0.1 , 10.0 ) ,
                   make_const_step_time_iterator_end( stepper , harmonic_oscillator, x ) ,
                   []( std::pair< const state_type & , const double & > x ) {
                       cout << x.second << " " << x.first[0] << " " << x.first[1] << "\n"; } );
    //]
    #endif
    
    


}
