#pragma once

#include "kindr/Core"

namespace romo
{
//private:
//    /* data */
//public:
//    romo(/* args */);
//    ~romo();

    typedef kindr::HomTransformQuatD Pose;//6D, position and rotation
//    typedef kindr::TwistGlobalD Twist;
    typedef kindr::TwistLinearVelocityLocalAngularVelocityD Twist;
    typedef kindr::RotationQuaternionD RotationQuaternion;
    typedef kindr::AngleAxisD AngleAxis;
    typedef kindr::RotationMatrixD RotationMatrix;
    typedef kindr::EulerAnglesZyxD EulerAnglesZyx;
    typedef kindr::EulerAnglesZyxDiffD EulerAnglesZyxDiff;
    typedef kindr::RotationVectorD RotationVector;
    typedef kindr::EulerAnglesXyzD EulerAnglesXyz;
    typedef kindr::EulerAnglesXyzDiffD EulerAnglesXyzDiff;
    typedef kindr::Position3D Position;
    typedef kindr::LocalAngularVelocity<double> LocalAngularVelocity;
    typedef kindr::Velocity3D LinearVelocity;
    typedef kindr::Acceleration3D LinearAcceleration;
    typedef kindr::AngularAcceleration3D AngularAcceleration;
    typedef kindr::Force3D Force;
    typedef kindr::Torque3D Torque;
    typedef kindr::VectorTypeless3D Vector;
};

//romo::romo(/* args */)
//{
//}

//romo::~romo()
//{
//}
