#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <stdio.h>

enum { RIGHT, LEFT };

namespace gazebo
{
  class PDController : public ModelPlugin
  {
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::JointPtr _j1, _j2;
    private: std::ofstream _output_current, _output_des, _output_torque;

    public: PDController()
    {
    }

    public: ~PDController()
    {
      _output_current.close();
      _output_des.close();
      _output_torque.close();
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      model = _parent;

      // store the pointer to the world
      world = model->GetWorld();

     // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PDController::OnUpdate, this, _1));

      // get the joints
      this->_j1 = model->GetJoint("upper_joint");
      this->_j2 = model->GetJoint("lower_joint");
    }

    // open the files for writing 
    public: void Init()
    {
      _output_current.open("track_sinusoidal.state");
      _output_des.open("track_sinusoidal.desired");
      _output_torque.open("track_sinusoidal.torque");
    }

    // TODO: this way we only need to modify the inverse dynamics code once
    #include "inverse_dynamics.cpp"

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // get the current time
      double t = world->GetSimTime().Double();

      // set the desired positions and velocities
      const double J1_DES = std::sin(t) , J2_DES = std::sin(3.0*t);
      const double vJ1_DES = std::cos(t), vJ2_DES = std::cos(3.0*t)*3.0;

      // setup gains
      const double KP = 10.0, KV = 2.50, KI = 0.1;

      // setup position and velocity errors
      const double perr1 = (J1_DES - _j1->GetAngle(0).Radian());
      const double derr1 = (vJ1_DES - _j1->GetVelocity(0));
      const double perr2 = (J2_DES - _j2->GetAngle(0).Radian());
      const double derr2 = (vJ2_DES - _j2->GetVelocity(0));

      // setup inverse dynamics torques
      double tau1, tau2;
      CalcInvDyn((_j1->GetAngle(0).Radian()+M_PI_2), _j2->GetAngle(0).Radian(),
                 _j1->GetVelocity(0), _j2->GetVelocity(0), 
                 0.0, 0.0, tau1, tau2);

      // output current and desired state and motor torques
      _output_current << _j1->GetAngle(0).Radian() << " " << _j2->GetAngle(0).Radian() << std::endl;
      _output_des << J1_DES << " " << J2_DES << std::endl;
      _output_torque << tau1 << " " << (KP*perr1+KV*derr1) << " " << tau2 << " " << (KP*perr2+KV*derr2) << std::endl;

      // set torques
      this->_j1->SetForce(0, tau1 + KP*perr1 + KV*derr1);
      this->_j2->SetForce(0, tau2 + KP*perr2 + KV*derr2);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PDController)
}

