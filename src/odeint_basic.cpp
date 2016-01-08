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
    state_type x0(2);
    x0[0] = 1.0; // start at x=1.0, p=0.0
    x0[1] = 0.0;
    //]
    
    const double t0 = 0.0;
    const double tf = 21.0;
    const double dt = 0.01;

    state_type x = x0;
    
    //[ integration
    size_t steps = integrate( harmonic_oscillator ,
            x , t0, tf, dt );
    //]

    std::cout<< "Basic version: " << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

    x = x0;

    //[ integration_class
    harm_osc ho(.15,.1);
    steps = integrate( ho ,
            x , t0, tf, dt);
    //]

    std::cout<< "Class version: "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

  

    x = x0;
    
    //[ integrate_observ
    vector<state_type> x_vec;
    vector<double> times;

    steps = integrate( harmonic_oscillator ,
            x , t0, tf, dt ,
            push_back_state_and_time( x_vec , times ) );

    std::cout<< "func observe:  "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

    /* output */
    for( size_t i=0; i<=steps; i++ )
    {
    //    cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
    }
    //]

    x = x0;
    x_vec.clear();
    times.clear();

    steps = integrate( ho ,
            x , t0, tf, dt ,
            push_back_state_and_time( x_vec , times ) );

    std::cout<< "class observe: "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

    /* output */
    for( size_t i=0; i<=steps; i++ )
    {
    //    cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << '\n';
    }
    //]




    x = x0;

    //[ define_const_stepper
    runge_kutta4< state_type > stepper;
    steps = integrate_const( stepper , ho , x , t0, tf, dt );
    //]

    std::cout<< "const stepper: "  << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

    x = x0;
    
    std::cout<< "const stepper loop: \t" ;
    //[ integrate_const_loop
    const double dt2 = 0.01;
    for( double t=t0 ; t<tf ; t+= dt2 )
    {
        stepper.do_step( ho , x , t , dt2 );
    //    cout << t << '\t' << x[0] << '\t' << x[1] << '\n';
    }
        
    //]

    std::cout<<" final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;




    //[ define_adapt_stepper
    typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
    //]

    x = x0;

    //[ integrate_adapt
    typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper;
    steps = integrate_adaptive( controlled_stepper , ho , x , t0, tf, dt2 );
    //]
    std::cout<<"adaptive: " << steps << " steps; \t final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;

    x = x0;
    double abs_err = 1.0e-10 , rel_err = 1.0e-6 , a_x = 1.0 , a_dxdt = 1.0;
    
    {
    //[integrate_adapt_full
    
    controlled_stepper_type controlled_stepper( 
        default_error_checker< double , range_algebra , default_operations >( abs_err , rel_err , a_x , a_dxdt ) );
    steps = integrate_adaptive( controlled_stepper , ho , x , t0, tf, dt2 );
    //]
    }
    std::cout<<"adaptive full: " << steps << " steps; final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;


    x = x0;
    //[integrate_adapt_make_controlled
    integrate_adaptive( make_controlled< error_stepper_type >( abs_err ,rel_err) , 
                        ho , x , t0, tf, dt2 );
    //]

    std::cout<<"controlled: " << steps << " steps; \t final: " << '\t' << x[0] << '\t' << x[1]<< std::endl;


    x = x0;
    //[integrate_adapt_make_controlled_alternative
    integrate_adaptive( make_controlled( abs_err , rel_err , error_stepper_type() ) , 
                        ho , x , t0 , tf , dt2 );
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
