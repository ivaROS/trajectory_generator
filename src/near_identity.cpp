
class ni_state : public TrajectoryState<ni_state,8>
{
public:
  enum STATE_INDICIES { X_IND=0, Y_IND=1, THETA_IND=2, V_IND=3, W_IND=4, LAMBDA_IND=5, XD_IND=6, YD_IND=7 };
  
  
  
  bool checkState()
  {
    return !(data[near_identity::LAMBDA_IND] ==0);
  }
  
  void to(geometry_msgs::Pose& pose)
  {
    pose.position.x = data[ni_state::X_IND];
    return pose;
  }
  
  pips_trajectory_msgs::trajectory_points trajectoryMsg()
  {
    return pips_trajectory_msgs::trajectory_points;
    //todo:
  }
  
  pips_trajectory_msgs::trajectory_point trajectoryStateMsg()
  {
    pips_trajectory_msgs::trajectory_point point;
    point.time = ros::Duration(times[i]);
    point.x = x_vec[i][near_identity::X_IND];
    point.y = x_vec[i][near_identity::Y_IND];
    point.theta = x_vec[i][near_identity::THETA_IND];
    point.v = x_vec[i][near_identity::V_IND];
    point.w = x_vec[i][near_identity::W_IND];
    return point;
  }
  
};


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller {

    near_identity ni_;
    traj_func* traj_;   //Will look into using reference to function 
    

public:
    ni_controller( near_identity ni) :  ni_(ni) { }

    void setTrajFunc(traj_func* traj)
    {
      traj_ = traj;
    }

    void operator() ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        traj_->dState(x,dxdt,t);
        ni_(x,dxdt,t);
    }
};


/*

near_identity ni(params.cp,params.cd,params.cl,params.eps,params.v_max,params.w_max,params.a_max,params.w_dot_max);
cp = 100;
cd = 100;
cl = 100;
eps = .01;
double cp, cd, cl, eps;

double v_max, w_max, a_max, w_dot_max;

v_max = std::numeric_limits<double>::infinity();
w_max = std::numeric_limits<double>::infinity();
a_max = std::numeric_limits<double>::infinity();
w_dot_max = std::numeric_limits<double>::infinity();

.cp=cp, .cd=cd, .cl=cl, .eps=eps, .v_max=v_max, .w_max=w_max, .a_max=a_max,.w_dot_max=w_dot_max

*/
