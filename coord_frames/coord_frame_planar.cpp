// **********************************************************************
// coord_frame_planar.cpp: plugin for computing forward kinematics of
// 3-link planar arm by computing a frame attached to the end link
// of the planar arm
// **********************************************************************

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  // converts a Gazebo Pose object to a Matrix4 object
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

  class CoordFramePlanar : public ModelPlugin
  {
    private: physics::ModelPtr widget;
    private: physics::ModelPtr planar_robot;

    public: CoordFramePlanar() : ModelPlugin()
    {
    }

    // loads the coordinate frame and gets the joints for the robot
    public: void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
      // get the coordinate frame
      planar_robot = model;

      // get the widget model
      widget = model->GetWorld()->GetModel("coord_frame");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CoordFramePlanar::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      const double INFINITY = std::numeric_limits<double>::max();
      const double CHANGEME = INFINITY;

      // get the end link of the planar robot
      physics::LinkPtr link3 = planar_robot->GetLink("link_3");

      // get the matrix corresponding to the pose for the end link
      math::Matrix4 _0T3 = ToMatrix(link3->GetWorldCoGPose());

      // TODO: setup the constant transformation- put at tip of end-effector 
      // (using a translation of [0.4356 0 0] with y-axis pointing away from 
      // robot, x-axis pointing "down", and z-axis unchanged (pointing "up and 
      // out of the plane")
      math::Pose constant_pose;
      constant_pose.pos = math::Vector3(CHANGEME,CHANGEME,CHANGEME);
      constant_pose.rot.SetFromAxis(CHANGEME,CHANGME,CHANGEME,CHANGEME);
      math::Matrix4 _3Tc = ToMatrix(constant_pose);

      // TODO: compute the target pose from _0T3 and _3Tc
      math::Matrix4 _0Tc;
      
      // set the pose for the widget 
      widget->SetLinkWorldPose(_0Tc.GetAsPose(), "link");
    }

    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(CoordFramePlanar)
}
