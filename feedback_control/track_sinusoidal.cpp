#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <stdio.h>
#include <fstream>

namespace gazebo
{
  class PDController : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2;
    private: std::ofstream _output_des, _output_current;

    public: PDController()
    {
    }

    public: ~PDController()
    {
      _output_current.close();
      _output_des.close();
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      _model = _parent;

      // store the pointer to the world
      _world = _model->GetWorld();

     // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PDController::OnUpdate, this, _1));

      // get the joints
      this->_j1 = _model->GetJoint("upper_joint");
      this->_j2 = _model->GetJoint("lower_joint");
    }

    // open up files for writing 
    public: void Init()
    {
      _output_current.open("track_sinusoidal.state");
      _output_des.open("track_sinusoidal.desired");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      const double CHANGEME = 0.0;
      static double sum_p1_err = CHANGEME;
      static double sum_p2_err = CHANGEME;

      // get the current time
      double t = _world->GetSimTime().Double();

      // set the desired positions and velocities
      // TODO: set the position/velocity for joint 2
      const double J1_DES = std::sin(t), J2_DES = CHANGEME;
      const double vJ1_DES = std::cos(t), vJ2_DES = CHANGEME;

      // setup gains
      const double KP = CHANGEME, KV = CHANGEME, KI = CHANGEME;

      // output current and desired state
      _output_current << _j1->GetAngle(0).Radian() << " " << _j2->GetAngle(0).Radian() << std::endl;
      _output_des << J1_DES << " " << J2_DES << std::endl;

      // TODO: setup position and velocity errors
      const double perr1 = CHANGEME; 
      const double derr1 = CHANGEME; 
      const double perr2 = CHANGEME; 
      const double derr2 = CHANGEME; 

      // TODO: compute torques
      double tau1 = CHANGEME;
      double tau2 = CHANGEME;

      // set torques
      this->_j1->SetForce(0, tau1);
      this->_j2->SetForce(0, tau2);

      // TODO: update p1 and p2 error sums
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PDController)
}

