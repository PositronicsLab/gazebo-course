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
    private: physics::JointPtr _j1, _j2, _j3;

    public: CoordFramePlanar() : ModelPlugin()
    {
    }

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

      // get the joints
      _j1 = model->GetJoint("joint_1");
      _j2 = model->GetJoint("joint_2");
      _j3 = model->GetJoint("joint_3");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
     // setup the first pose
      math::Pose _0P0x, _0xP1, _1P1x, _1xP2, _2P2x, _2xP3, _3Pc; 

      // get the joint angles
      double theta1 = _j1->GetAngle(0).Radian();
      double theta2 = _j2->GetAngle(0).Radian();
      double theta3 = _j3->GetAngle(0).Radian();

      // TODO: compute the first pose: frame 0' defined relative to frame 0

      // TODO: compute the second pose: frame 1 defined relative to frame 0' 
      
      // TODO: compute the third pose: frame 1' defined relative to frame 1
      
      // TODO: compute the fourth pose: frame 2 defined relative to frame 1' 

      // TODO: compute the fifth pose: frame 2' defined relative to frame 2 
      
      // TODO: compute the sixth pose: frame 3 defined relative to frame 2' 

      // TODO: compute the seventh pose: frame c defined relative to frame 3 

      // convert each pose to a Matrix4 
      math::Matrix4 _0T0x = ToMatrix(_0P0x);
      math::Matrix4 _0xT1 = ToMatrix(_0xP1);
      math::Matrix4 _1T1x = ToMatrix(_1P1x);
      math::Matrix4 _1xT2 = ToMatrix(_1xP2);
      math::Matrix4 _2T2x = ToMatrix(_2P2x);
      math::Matrix4 _2xT3 = ToMatrix(_2xP3);
      math::Matrix4 _3Tc = ToMatrix(_3Pc);

      // TODO: compute _0Tc from matrices above
      math::Matrix4 _0Tc;
      
      // set the pose for the widget 
      widget->SetLinkWorldPose(_0Tc.GetAsPose(), "link");
    }

    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(CoordFramePlanar)
}
