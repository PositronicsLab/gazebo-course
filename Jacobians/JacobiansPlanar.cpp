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

  class JacobiansPlanarPlugin : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2, _j3;

    public: JacobiansPlanarPlugin()
    {
    }

    // sets the arm to a particular configuration 
    private: void SetConfig(Ravelin::VectorNd& q)
    {
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

          // set the joint angle
          joint->SetPosition(j, q[k]);
          k++;
        }
      }
    }

    // gets the Jacobian for the planar manipulator
    private: Ravelin::MatrixNd CalcJacobian(double theta1, double theta2, double theta3)
    {
      const math::Vector3 ORIGIN1(0.0, 0.0, 0.0);  // origin of frame 1
      const math::Vector3 ORIGIN2(0.0, 0.0, 0.0);  // origin of frame 2
      const math::Vector3 ORIGIN3(0.0, 0.0, 0.0);  // origin of frame 3 
      const unsigned X = 0, Y = 1, THETA = 2, J1 = 0, J2 = 1, J3 = 2;

      // temporary values
      const double INF = std::numeric_limits<double>::max();
      const double CHANGEME = INF;
      const math::Vector3 CHANGEME_VEC3(CHANGEME, CHANGEME, CHANGEME);
 
      // the poses (for you, the student, to set) 
      math::Pose _0P0x, _0xP1, _1P1x, _1xP2, _2P2x, _2xP3; 

      // TODO: compute the first transform: frame 0' to 0

      // TODO: compute the second transform: frame 1 to 0' 
      
      // TODO: compute the third transform: frame 1' to 1
      
      // TODO: compute the fourth transform: frame 2 to 1' 

      // TODO: compute the fifth transform: frame 2' to 2 
      
      // TODO: compute the fourth transform: frame 3 to 2' 

      // convert all poses to transforms
      math::Matrix4 _0T0x = ToMatrix(_0P0x);
      math::Matrix4 _0xT1 = ToMatrix(_0xP1);
      math::Matrix4 _1T1x = ToMatrix(_1P1x);
      math::Matrix4 _1xT2 = ToMatrix(_1xP2);
      math::Matrix4 _2T2x = ToMatrix(_2P2x);
      math::Matrix4 _2xT3 = ToMatrix(_2xP3);

      // position of the first joint is always (0,0,0)
      math::Vector3 p1(0.0, 0.0, 0.0);

      // TODO: compute the position of the second joint
      math::Vector3 p2 = CHANGEME_VEC3;

      // TODO: compute the position of the third joint
      math::Vector3 p3 = CHANGEME_VEC3; 

      // TODO: get the position of the manipulator end point
      math::Vector3 p = CHANGEME_VEC3;
      
      // setup the Jacobian: 3 degrees of freedom x 3 joints
      Ravelin::MatrixNd J(3,3);
      J(X,J1) = CHANGEME;  J(Y,J1) = CHANGEME;  J(THETA,J1) = CHANGEME;
      J(X,J2) = CHANGEME;  J(Y,J2) = CHANGEME;  J(THETA,J2) = CHANGEME;
      J(X,J3) = CHANGEME;  J(Y,J3) = CHANGEME;  J(THETA,J3) = CHANGEME;

      return J;
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
          boost::bind(&JacobiansPlanarPlugin::OnUpdate, this, _1));

      // get the joints
      _j1 = model->GetJoint("joint_1");
      _j2 = model->GetJoint("joint_2");
      _j3 = model->GetJoint("joint_3");
    }

    // open up files for writing 
    public: void Init()
    {
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      Ravelin::VectorNd theta(3);

      // set the joint angles
      theta[0] = 0.0;  theta[1] = 0.0;  theta[2] = 0.0;
      SetConfig(theta);
      Ravelin::MatrixNd J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = 0.0;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = M_PI_2;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = M_PI_2;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = 0.0;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = 0.0;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = M_PI_2;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = M_PI_2;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // exit when we're done
      exit(-1);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JacobiansPlanarPlugin)
}


