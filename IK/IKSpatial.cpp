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
  // generates a random double in [-pi, pi]
  double randD() { return (double) rand() / RAND_MAX * (M_PI * 2.0) - M_PI; }

  // converts a Pose to a Gazebo Matrix4
  math::Matrix4 ToMatrix(const math::Pose& p)
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

  class IKSpatialPlugin : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j[6];
    private: Ravelin::VectorNd _dq, _qdes;
    private: bool _do_position;

    public: IKSpatialPlugin()
    {
    }

    // gets the number of degrees-of-freedom of a model in Gazebo
    private: unsigned nDOF(physics::ModelPtr model)
    {
      unsigned ndof = 0;

      // iterate over all joints
      for (unsigned i=0, k=0; i< model->GetJoints().size(); i++)
      {
        physics::JointPtr joint = model->GetJoints()[i];

        // iterate over all degrees-of-freedom in the joint
        for (unsigned j=0; j< joint->GetAngleCount(); j++)
        {
          // make sure that the joint is active
          if (joint->GetLowerLimit(j) < joint->GetUpperLimit(j))
          {
            k++;
            ndof++;
          }
        }
      }

      return ndof;
    }

    // computes forward kinematics for the robot
    private: math::Pose FKin(double q[6])
    {
      // save the state of the model
      physics::ModelState ms(_model);

      // iterate over all joints
      for (unsigned i=0, k=0; i< _model->GetJoints().size(); i++)
      {
        physics::JointPtr joint = _model->GetJoints()[i];

        // iterate over all degrees-of-freedom in the joint
        for (unsigned j=0; j< joint->GetAngleCount(); j++)
        {
          // make sure that the joint is active
          if (joint->GetLowerLimit(j) >= joint->GetUpperLimit(j))
            continue;

          // update the joint position by DQ
          joint->SetAngle(j, q[k]); 
        }
      }

      // get the end link of the industrial robot
      physics::LinkPtr wrist3 = _model->GetLink("ur10::wrist_3");

      // get the new point on the link and the new orientation of the link
      math::Pose x = wrist3->GetWorldCoGPose();

      // restore the configuration of the robot 
      _model->SetState(ms);

      return x;
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
          boost::bind(&IKSpatialPlugin::OnUpdate, this, _1));

      // get the joints
      const unsigned NJOINTS = 6;
      _j[0] = model->GetJoint("shoulder_pan");
      _j[1] = model->GetJoint("shoulder_lift");
      _j[2] = model->GetJoint("elbow");
      _j[3] = model->GetJoint("wrist_1");
      _j[4] = model->GetJoint("wrist_2");
      _j[5] = model->GetJoint("wrist_3");

      // get the joint angles and setup _qdes
      _qdes.resize(NJOINTS);      
      for (unsigned i=0; i< NJOINTS; i++)
        _qdes[i] = _j[i]->GetAngle(0).Radian();
    }

    // open up files for writing 
    public: void Init()
    {
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
    private: Ravelin::VectorNd GradG(const math::Pose& target, double q[6])
    {
      const double DQ = 1e-6;
      const unsigned NJOINTS = 6;

      // get the current end-effector pose
      double y = CalcOSDiff(FKin(q), target).norm();

      // compute the other end effector poses
      Ravelin::VectorNd f(NJOINTS);
      double yi[NJOINTS];
      for (unsigned i=0; i< NJOINTS; i++)
      {
        q[i] += DQ;
        f[i] = CalcOSDiff(FKin(q), target).norm() - y;
      }

      // scale f
      f *= 0.5 / DQ;

      return f;
    }

    // TODO: implement this function using the algorithm in the assignment 
    private: Ravelin::MatrixNd CalcJacobianNumerical(double q[6])
    {
      const double DQ = 1e-6;

      // save the state of the model
      physics::ModelState ms(_model);

      // set the current state of the model
      for (unsigned i=0, k=0; i< _model->GetJoints().size(); i++)
      {
        physics::JointPtr joint = _model->GetJoints()[i];

        // iterate over all degrees-of-freedom in the joint
        for (unsigned j=0; j< joint->GetAngleCount(); j++)
        {
          // make sure that the joint is active
          if (joint->GetLowerLimit(j) >= joint->GetUpperLimit(j))
            continue;

          // update the joint position by DQ
          joint->SetAngle(j, q[k]); 
        }
      }

      // get the end link of the industrial robot
      physics::LinkPtr wrist3 = _model->GetLink("ur10::wrist_3");

      // initialize the matrix to the proper size
      Ravelin::MatrixNd J(6, nDOF(_model));

      // get the current point on the link and the current orientation of the
      // link
      math::Pose x = wrist3->GetWorldCoGPose();

      // TODO: iterate over all joints (see code immediately above that shows
      //       how to do this) for computing the Jacobian

      // restore the configuration of the model
      _model->SetState(ms);

      return J;
    }

    // computes operational space differential between two poses
    private: Ravelin::VectorNd CalcOSDiff(const math::Pose& xcurrent, const math::Pose& xdes)
    {
      const unsigned SPATIAL_DIM = 6;

      // init the vector
      Ravelin::VectorNd dx(SPATIAL_DIM);

      // convert the poses to matrices
      math::Matrix4 T = ToMatrix(xcurrent);
      math::Matrix4 Tdes = ToMatrix(xdes);

      // TODO: compute the translational difference from T to Tdes and put
      //       that into dx[0], dx[1], dx[2] 

      // get the necessary rotation matrices to compute angular rotation
      math::Matrix3 Rdes = Tdes.GetRotation().GetAsMatrix3();
      math::Matrix3 RT = T.GetRotation().GetInverse().GetAsMatrix3();

      // TODO: compute the angular difference from R (not transpose(R) = RT)
      //       to Rdes and put that into dx[3], dx[4], dx[5]

      // zero parts of dx that we do not want to use for IK
      if (_do_position)
        dx[3] = dx[4] = dx[5] = 0.0;
      else
        dx[0] = dx[1] = dx[2] = 0.0;

      return dx;
    }

    // does inverse kinematics using Resolved Motion Rate Control
    private: void DoIK(const math::Pose& target)
    {
      const double DX_TOL = 1e-4, LOCAL_MIN_TOL = 1e-8;

      // set minimum dx observed 
      double min_dx = std::numeric_limits<double>::max();

      // get the current joint angles
      const unsigned NJOINTS = 6;
      double theta[NJOINTS];
      for (unsigned i=0; i< NJOINTS; i++)
       theta[i] = _j[i]->GetAngle(0).Radian();

      // initialize dq2
      double dq2[6];

      // set the number of iterations and restarts 
      unsigned iter = 0, restarts = 0;

      // do the IK process
      while (true)
      {
        // get the error between the current and desired poses

        // TODO: get the current end-effector pose 
        math::Pose x;// = FILL ME IN 

        // get the error between current and desired poses
        Ravelin::VectorNd dx = CalcOSDiff(x, target);

        // compute the gradient of 1/2 * ||x_des - f(q)|| - we will use this
        // for the backtracking line search below
        Ravelin::VectorNd grad = GradG(target, theta); 

        // if there's hardly any error, quit
        const double DX_NORM = dx.norm();
        if (DX_NORM < LOCAL_MIN_TOL)
          break;

        // record smallest dx
        min_dx = std::min(min_dx, DX_NORM);
        std::cout << "dx norm: " << DX_NORM << "  minimum dx observed: " << min_dx << std::endl;

        // TODO: get the Jacobian
        Ravelin::MatrixNd J;// = FILL ME IN

        // "solve" J*dq = dx for _dq using SolveJ or TransposeJ

        // do backtracking search to determine value of t 
        const double ALPHA = 0.05, BETA = 0.5;
        double t = 1.0;

        // compute dq2 = _dq * t
        dq2 = _dq;
        dq2 *= t;

        // TODO: compute f(dq2) 
        math::Pose xprime;// = FILL ME IN
        Ravelin::VectorNd dx_prime = CalcOSDiff(xprime, target); 
        while (0.5*dx_prime.norm() > 0.5*dx.norm() + ALPHA*t*grad.dot(_dq))
        {
          // reduce t geometrically
          t*= BETA;

          // if t becomes too small (smaller than machine epsilon), quit
          if (t < std::numeric_limits<double>::epsilon())
            break;

          // update dq2
          dq2 = _dq;
          dq2 *= t;

          // TODO: recompute f(dq2)
          // xprime = FILL ME IN

          // recompute dx_prime
          dx_prime = CalcOSDiff(xprime, target); 
        }

        // update change in q         
        _dq *= t;

        // try again if necessary
        if (_dq.norm() < LOCAL_MIN_TOL)
        {
          std::cout << "-- hit a local minimum: norm dq (" << _dq.norm() << "), norm dx (" << DX_NORM <<") " << std::endl << " -- resetting joint angles and trying again" << std::endl;
          for (unsigned i=0; i< NJOINTS; i++)
            theta[i] = randD();
          restarts++;
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
      const unsigned NJOINTS = 6;
      double theta[NJOINTS], dtheta[NJOINTS], perr[NJOINTS];
      double f[NJOINTS];
      const double KP = 100.0, KD = 25.0; 
      static bool first_time = true;

      // setup the target
      if (first_time)
      {
        // setup q
        double q[6] = { randD(), randD(), randD(), randD(), randD(), randD() };

        // indicate we want to solve for position
        _do_position = true;

        // parse the string
        math::Pose target = FKin(q);
        std::cout << "Solving to target: " << target << std::endl;
        DoIK(target);
        first_time = false;
      }

      // get current joint angles and velocities and compute and apply PD torques
      for (unsigned i=0; i< NJOINTS; i++)
      {
        theta[i] = _j[i]->GetAngle(0).Radian();
        dtheta[i] = _j[i]->GetVelocity(0);
        perr[i] = _qdes[i] - theta[i];
        f[i] = KP*perr[i] - KD*dtheta[i];
        _j[i]->SetForce(0, f[i]);
      }
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(IKSpatialPlugin)
}


