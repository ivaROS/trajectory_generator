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
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <array>
#include <Eigen/Eigen>


//[ rhs_function
/* The type of container used to hold the state vector */


// template<typename T, size_t N>
// class TrajectoryState
// {
//   typedef std::array<double, N> array;
//   
// protected:
//   array data;
//   
// public:
//   
//   typedef typename array::value_type value_type;
//   
//   typedef typename array::iterator iterator;
//   typedef typename array::const_iterator const_iterator;
//   
//   iterator begin() {return data.begin();}
//   iterator end() {return data.end();}
//   
//   const_iterator begin() const {return data.begin();}
//   const_iterator end() const {return data.end();}
//   
//   inline const double& operator[] (size_t n) const { return (data[n]); }
//   inline double& operator[] (size_t n)  { return (data[n]); }
//   
//   
//   geometry_msgs::Pose getPose()
//   {
//     return (T)this->getPose();
//   }
// 
// };
// 
// class ni_state : public TrajectoryState<ni_state,8>
// {
//   
//   geometry_msgs::Pose getPose()
//   {
//     geometry_msgs::Pose pose;
//     pose.position.x = data[0];
//     return pose;
//   }
//   
// };

typedef std::vector<double> ni_state;


class near_identity {
  
  
  
  double c_p_;
  double c_d_;
  double c_lambda_;
  double epsilon_;
  
  double v_max_ = std::numeric_limits<double>::infinity();
  double w_max_ = std::numeric_limits<double>::infinity();
  double a_max_ = std::numeric_limits<double>::infinity();
  double w_dot_max_ = std::numeric_limits<double>::infinity();
  
public:
  
  enum STATE_INDICIES { X_IND=0, Y_IND=1, THETA_IND=2, V_IND=3, W_IND=4, LAMBDA_IND=5, XD_IND=6, YD_IND=7 };
  
  near_identity( double c_p, double c_d, double c_lambda, double epsilon ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon) { }
  
  near_identity( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon), v_max_(v_max), w_max_(w_max), a_max_(a_max), w_dot_max_(w_dot_max) { }
  
  void operator() ( const ni_state &state , ni_state &state_dot, const double /* t*/  )
  {
    //load state variables
    double x = state[near_identity::X_IND];
    double y = state[near_identity::Y_IND];
    double theta = state[near_identity::THETA_IND];
    double v = state[near_identity::V_IND];
    double w = state[near_identity::W_IND];
    double lambda = state[near_identity::LAMBDA_IND];
    double x_d = state[near_identity::XD_IND];
    double y_d = state[near_identity::YD_IND];
    double x_d_dot = state_dot[near_identity::XD_IND];
    double y_d_dot = state_dot[near_identity::YD_IND];
    
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta), 
    sin(theta), cos(theta);
    
    double lambda_dot = -c_lambda_*(lambda - epsilon_);
    
    
    //Now find derivatives of state variables
    //x_dot = (R*e1)*v1;
    double x_dot = R(0,0)*v;
    double y_dot = R(1,0)*v;
    double theta_dot = w;
    
    //Now find tau
    Eigen::Vector2d tau = getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);
    
    double v_dot = limitV(tau, v);
    double w_dot = limitW(tau, w);
    
    
    state_dot[near_identity::X_IND] = x_dot;
    state_dot[near_identity::Y_IND] = y_dot;
    state_dot[near_identity::THETA_IND] = theta_dot;
    state_dot[near_identity::V_IND] = v_dot;
    state_dot[near_identity::W_IND] = w_dot;
    state_dot[near_identity::LAMBDA_IND] = lambda_dot;
    //std::vector<double> state_dot = {x_dot, y_dot, theta_dot, v_dot, w_dot, lambda_dot};
    
  }
  
  inline
  Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot )
  {
    
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta), 
    sin(theta), cos(theta);
    
    double lambda_dot = -c_lambda_*(lambda - epsilon_);
    return getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);
  }
  
  
  inline
  Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot, const Eigen::Matrix2d& R, double lambda_dot )
  {
    Eigen::Vector2d xy;
    xy << x,
    y;
    
    Eigen::Vector2d vw;
    vw << v,
    w;
    
    Eigen::Vector2d xy_d;
    xy_d << x_d, 
    y_d;
    
    Eigen::Vector2d xy_d_dot;
    xy_d_dot << x_d_dot,
    y_d_dot;
    
    //R_lambda = [R*e1 lambda*R*e2];
    Eigen::Matrix2d R_lambda;
    R_lambda << R(0,0), lambda*R(0,1), 
    R(1,0), lambda*R(1,1);
    
    //R_lambda_inv = [R*e1 R*e2/lambda]';
    Eigen::Matrix2d R_lambda_inv;
    R_lambda_inv << R(0,0),        R(1,0), 
    R(0,1)/lambda, R(1,1)/lambda;
    
    Eigen::Matrix2d w_hat;
    w_hat << 0,             -lambda*w, 
    (1.0/lambda)*w, lambda_dot/lambda;
    
    //q = xy + lambda*R*e1;
    /*    Eigen::Vector2d q(x + lambda*R(0,0), 
     *                        y + lambda*R(1,0)); */
    Eigen::Vector2d q = xy + lambda*R.col(0);     
    
    //   std::cout << "q: " << q(0) << ", " << q(1) << std::endl;
    
    //p = R_lambda*v + lambda_dot*R_lambda*e1;
    /*   Eigen::Vector2d p(v + lambda_dot*R_lambda(0,0), 
     *                        w + lambda_dot*R_lambda(1,0));*/
    
    Eigen::Vector2d p = R_lambda*vw + lambda_dot*R_lambda.col(0);
    
    //   std::cout << "p: " << p(0) << ", " << p(1) << std::endl;
    
    
    //u can be any expression that will feedback stabilize a linear system
    //here, using 2nd order feedback controller   ; + x_d_dot_dot
    Eigen::Vector2d u = -c_p_*(q - xy_d) - c_d_*(p - xy_d_dot);
    
    //     std::cout << "u: " << u(0) << ", " << u(1) << std::endl;
    
    //Now find tau
    Eigen::Vector2d tau = R_lambda_inv*u - w_hat*vw - lambda_dot*(w_hat - c_lambda_*Eigen::Matrix2d::Identity())*Eigen::Matrix<double, 2, 1>::Identity();
    
    return tau;
    
  }
  
  
  /* Generic saturation function for variable X. */
  inline
  static double saturate(double X, double minX, double maxX)
  {
    if(X >= maxX)
    {
      return maxX;
    }
    else if(X <= minX)
    {
      return minX;
    }
    else
    {
      return X;
    }
  }
  
  /* Generic saturation function for variable X_dot given limits on X_dot and X. */
  inline
  static double applyLimits(double X_dot, double X, double minX, double maxX, double minX_dot, double maxX_dot)
  {
    if(X >= maxX)
    {
      return saturate(X_dot, minX_dot, 0);
    }
    else if(X <= minX)
    {
      return saturate(X_dot, 0, maxX_dot);
    }
    else
    {
      return saturate(X_dot, minX_dot, maxX_dot);
    }   
  }
  
  inline
  double limitV(const Eigen::Vector2d& tau, double v)
  {
    return applyLimits(tau[0], v, -v_max_, v_max_, -a_max_, a_max_);
  }
  
  inline
  double limitW(const Eigen::Vector2d& tau, double w)
  {
    return applyLimits(tau[1], w, -w_max_, w_max_, -w_dot_max_, w_dot_max_);
  }
};

//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_gen {

    double m_amp;
    double m_f;

public:
    traj_gen( double amp, double f ) : m_amp(amp), m_f(f) { }

    void operator() ( const ni_state &x , ni_state &dxdt , const double  t  )
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

    void operator() ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        traj_(x,dxdt,t);
        ni_(x,dxdt,t);
    }
};






//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< ni_state >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< ni_state > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const ni_state &x , double t )
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
    ni_state x0(8); 
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

    ni_state x = x0;
    

    //[ Observer samples
    vector<ni_state> x_vec;
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
      typedef runge_kutta_cash_karp54< ni_state > error_stepper_type;
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
