#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

class MoCapPosition3Factor : public NoiseModelFactorN<Pose3> {   
 private:
  // Measurement information.
  Point3 m_;

  public:
  /**
   * Constructor
   * @param poseKey    The key of the pose.
   * @param m          Point3 measurement.
   * @param model      Noise model for Motion Capture sensor.
   */
  MoCapPosition3Factor(gtsam::Key poseKey,
                      const gtsam::Point3& m,
                      const gtsam::SharedNoiseModel& model)
      : NoiseModelFactorN<Pose3>(model, poseKey), m_(m) {}

  // Error function.
  // @param p 3D pose.
  // @param H optional Jacobian matrix.
  gtsam::Vector evaluateError(const gtsam::Pose3& p,
                              OptionalMatrixType H = OptionalNone) const override {
    // 3a. Complete definition of factor
    // TODO:
    // Return error vector and jacobian if requested (aka H !=
    if (H){
      // auto H_jacobian = OptionalJacobian<3, 6>();
      Matrix36 t_H_T;
      p.translation(&t_H_T);   // calculates jacobian of translation w.r.t. pose variables and stores it in H
      *H = (Matrix(3,6) << t_H_T.row(0), t_H_T.row(1), t_H_T.row(2)).finished();
    }
    // boost::none).
    return (gtsam::Vector(3) << p.x() -m_.x(), p.y() - m_.y(), p.z() - m_.z()).finished();
    // End 3a.
  }

  ~MoCapPosition3Factor() {}
};

}  // namespace gtsam
