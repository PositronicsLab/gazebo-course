#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/LinAlgd.h>
#include <stdio.h>
#include <fstream>
#include <sstream>

namespace gazebo
{
  // converts a Pose to a Gazebo Matrix4
  math::Matrix4 toMatrix(const math::Pose& p)
  {
    math::Matrix3 m = p.rot.GetAsMatrix3();
    math::Matrix4 T;
    for (unsigned i=0; i< 3; i++)
      for (unsigned j=0; j< 3; j++)
        T[i][j] = m[i][j];

    for (unsigned i=0; i< 3; i++)
      T[i][3] = p.pos[i];

    // set bottom row of the matrix
    T[3][0] = T[3][1] = T[3][2] = 0.0;
    T[3][3] = 1.0;
    return T;
  }

  class IKPlanarPlugin : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2, _j3;
    private: Ravelin::VectorNd _dq, _qdes;
    private: bool _do_position;   // if 'true' does IK on position, otherwise
                                  // uses orientation

    public: IKPlanarPlugin()
    {
    }

    public: void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      _model = model;

      // store the pointer to the world
      _world = _model->GetWorld();

     // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&IKPlanarPlugin::OnUpdate, this, _1));

      // get the joints
      _j1 = model->GetJoint("joint_1");
      _j2 = model->GetJoint("joint_2");
      _j3 = model->GetJoint("joint_3");

      // get the joint angles
      double theta1 = _j1->GetAngle(0).Radian();
      double theta2 = _j2->GetAngle(0).Radian();
      double theta3 = _j3->GetAngle(0).Radian();

      // set qdes
      _qdes.resize(3);
      _qdes[0] = theta1;
      _qdes[1] = theta2;
      _qdes[2] = theta3;
    }

    // open up files for writing 
    public: void Init()
    {
      // set the seed for randomization
      srand(time(NULL));
    }

    // solves J*dq = dx for dq
    private: void SolveJ(const Ravelin::MatrixNd& J, const Ravelin::VectorNd& dx, Ravelin::VectorNd& dq)
    {
      static Ravelin::LinAlgd LA;
      static Ravelin::MatrixNd A, U, V;
      static Ravelin::VectorNd S;

      A = J;
      dq = dx;
      LA.svd(A, U, S, V);
      LA.solve_LS_fast(U, S, V, dq);
    } 

    // does the Jacobian transpose method: dq = J'*dx
    private: void TransposeJ(const Ravelin::MatrixNd& J, const Ravelin::VectorNd& dx, Ravelin::VectorNd& dq)
    {
      J.transpose_mult(dx, dq);
    } 

    // computes the numerical gradient of 1/2 ||x_des - f(q)|| w.r.t. q 
    // (used for backtracking line search)
    private: Ravelin::VectorNd GradG(const math::Pose& target, double theta1, double theta2, double theta3)
    {
      const double DQ = 1e-6;

      // get the current end-effector pose
      double y = CalcOSDiff(FKin(theta1, theta2, theta3), target).GetLength();

      // compute the other end effector poses
      double y1 = CalcOSDiff(FKin(theta1+DQ, theta2, theta3), target).GetLength();
      double y2 = CalcOSDiff(FKin(theta1, theta2+DQ, theta3), target).GetLength();
      double y3 = CalcOSDiff(FKin(theta1, theta2, theta3+DQ), target).GetLength();

      // get the error between the poses
      Ravelin::VectorNd f(3);
      f[0] = y1 - y; 
      f[1] = y2 - y; 
      f[2] = y3 - y; 
      f *= 0.5 / DQ;
      return f;
    }

    // TODO: implement this using coord_frame_planar2.cpp from coord_frames
    // computes the forward kinematics function for the planar manipulator 
    private: math::Pose FKin(double theta1, double theta2, double theta3)
    {
      math::Matrix4 _0T0x, _0xT1, _1T1x, _1xT2, _2T2x, _2xT3;

      // TODO: fill in the function here

      // get the manipulator end pose
      return (_0T0x * _0xT1 * _1T1x * _1xT2 * _2T2x * _2xT3).GetAsPose();
    }

    // TODO: implement this by copying the same function from Jacobians
    // gets the Jacobian for the planar manipulator
    private: Ravelin::MatrixNd CalcJacobian(double theta1, double theta2, double theta3)
    {
    }

    // "wraps" angle on a differential to interval [-pi, pi]
    private: double WrapAngle(double x)
    {
      // TODO: implement this
    }

    // computes the operational space differential between two poses
    private: math::Vector3 CalcOSDiff(const math::Pose& xcurrent, const math::Pose& xdes)
    {
      const unsigned X = 0, Y = 1, Z = 2;
      const double INFINITY = std::numeric_limits<double>::max();
      const double CHANGEME = INFINITY;

      // get the transformation matrices for xcurrent and xdes
      math::Matrix4 Tcurrent = toMatrix(xcurrent); 
      math::Matrix4 Tdes = toMatrix(xdes); 

      // TODO: get the angles of rotation about z from Tcurrent and Tdes

      // TODO: compute the difference in angles
      double theta_diff = CHANGEME;

      // wrap the differential to [-pi,pi]
      theta_diff = WrapAngle(theta_diff);

      // TODO: get the difference in position
      double x_diff = CHANGEME;
      double y_diff = CHANGEME; 
 
      // construct the differential
      math::Vector3 dx(x_diff, y_diff, theta_diff);

      // zero the parts that we do not want to use for IK
      if (_do_position)
        dx.z = 0.0;
      else
        dx.x = dx.y = 0.0;

      return dx;
    }

    // does inverse kinematics using Resolved Motion Rate Control
    private: void DoIK(const math::Pose& target)
    {
      const double DX_TOL = 1e-4, LOCAL_MIN_TOL = 1e-8;
      double min_dx = std::numeric_limits<double>::max();

      // get the current joint angles
      double theta1 = _j1->GetAngle(0).Radian();
      double theta2 = _j2->GetAngle(0).Radian();
      double theta3 = _j3->GetAngle(0).Radian();

      // set the number of iterations and restarts 
      unsigned iter = 0, restarts = 0;

      // do the IK process
      while (true)
      {
        // TODO: get the current end-effector pose
        math::Pose x;// = FILL ME IN 

        // get the error between the current and desired poses
        math::Vector3 deltax = CalcOSDiff(x, target); 

        // compute the gradient of 1/2 * ||x_des - f(q)|| - we will use this
        // for the backtracking line search below
        Ravelin::VectorNd grad = GradG(target, theta1, theta2, theta3);

        // convert the error to a Ravelin vector
        Ravelin::VectorNd dx(3);
        dx[0] = deltax[0];
        dx[1] = deltax[1];
        dx[2] = deltax[2];

        // if there's hardly any error, quit
        const double DX_NORM = dx.norm();
        if (DX_NORM < DX_TOL)
          break;

        // record smallest dx
        min_dx = std::min(min_dx, DX_NORM);
        std::cout << "dx norm: " << DX_NORM << "  minimum dx observed: " << min_dx << std::endl;

        // TODO: get the Jacobian
        Ravelin::MatrixNd J;// = FILL ME IN 

        // TODO: "solve" J*dq = dx for _dq using SolveJ or TransposeJ

        // do backtracking search to determine value of t 
        const double ALPHA = 0.05, BETA = 0.5;
        double t = 1.0;

        // TODO: compute f(q + dq*t)
        math::Pose xprime;// = FILL ME IN
        math::Vector3 deltax_prime = CalcOSDiff(xprime, target); 
        while (0.5*deltax_prime.GetLength() > 0.5*deltax.GetLength() + ALPHA*t*grad.dot(_dq))
        {
          // reduce t geometrically
          t*= BETA;

          // if t becomes too small (smaller than machine epsilon), quit
          if (t < std::numeric_limits<double>::epsilon())
            break;

          // TODO: recompute f(q + dq*t)
          // xprime = FILL ME IN 

          // recompute deltax_prime
          deltax_prime = CalcOSDiff(xprime, target); 
        }

        // update change in q         
        _dq *= t;

        // if this block of code is triggered, we do not have a good descent
        // direction - we hit a local minimum; try again from a random starting
        // point 
        if (_dq.norm() < LOCAL_MIN_TOL)
        {
          std::cout << "-- hit a local minimum: norm dq (" << _dq.norm() << "), norm dx (" << DX_NORM <<") " << std::endl << " -- resetting joint angles and trying again" << std::endl;
          theta1 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          theta2 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          theta3 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          iter = 0;
          continue;
        }

        // TODO: update thetas using _dq
      }

      // update qdes using the IK solution: this will allow the robot to go
      // to the IK solution (using the controller in OnUpdate(.)) 
      _qdes += _dq;

      // indicate IK solution found
      std::cout << "IK solution found after " << restarts << " and " << iter << " iterations" << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double theta[3], dtheta[3], perr[3];
      double f[3];
      const double KP = 100.0, KD = 25.0; 
      static bool first_time = true;

      // setup the target
      if (first_time)
      {
        // determine whether to solve for position
        _do_position = true;

        // find the target pose using randomization 
        math::Pose target = FKin((double) rand() / RAND_MAX, (double) rand() / RAND_MAX, (double) rand() / RAND_MAX);
        std::cout << "Solving to target: " << target << std::endl;
        std::cout << "Solving for position? " << _do_position << std::endl;
        DoIK(target);
        first_time = false;
      }

      // get current joint angles 
      theta[0] = _j1->GetAngle(0).Radian();
      theta[1] = _j2->GetAngle(0).Radian();
      theta[2] = _j3->GetAngle(0).Radian();

      // get current joint velocities
      dtheta[0] = _j1->GetVelocity(0);
      dtheta[1] = _j2->GetVelocity(0);
      dtheta[2] = _j3->GetVelocity(0);

      // compute position error
      perr[0] = (_qdes[0] - theta[0]);
      perr[1] = (_qdes[1] - theta[1]);
      perr[2] = (_qdes[2] - theta[2]);
      
      // compute PD torques
      f[0] = KP*perr[0] - KD*dtheta[0];
      f[1] = KP*perr[1] - KD*dtheta[1];
      f[2] = KP*perr[2] - KD*dtheta[2];

      // apply PD torques
      _j1->SetForce(0, f[0]);
      _j2->SetForce(0, f[1]);
      _j3->SetForce(0, f[2]);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(IKPlanarPlugin)
}


