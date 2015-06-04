
/***************************************************************************
 *  Object-Estimator.h
 *
 *  Created:
 *  Copyright  2015  Sebastian Reuter
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __KALMAN_FILTER_H_
#define __KALMAN_FILTER_H_

#include <utils/time/clock.h>
#include <string>
#include <queue>
#include <vector>
#include <logging/logger.h>
#include <config/config.h>
#include <tf/transformer.h>
#include <tf/types.h>

// BFL-specific includes
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <interfaces/MotorInterface.h>

namespace fawkes{
class Logger;
class Time;
class Transform;
}


/****************************************************************************************
 *
 *
 ********** System model ****************************************************************
 *
 * This system model describes a moving robot in 3D
 * The robot-state is defined by a position-vector{x,y,z} a velocity-vector{vx,vy,vz}
 *                       and a acceleration-vector{ax,ay,az}
 * Process model:
 * X_k = A X + B U + W
 *
 *              A          *  X  +    B     * U   * E_x
 *
 *       |1  0  1  0|  |x |  |0|
 *   x = |0  1  0  1|  |y |  |0|
 *       |0  0  1  0|  |dx|+ |0|* U
 *       |0  0  0  1|  |dy|  |0|
 *
 *
 * x_k = x_{k-1} + dt * vx_{k-1} + 1/2 * pow(dt,2) ax_{k-1}
 *
 * W_K = process_noise ~ N(0,Q_k) with Q_k = Covariance Matrix
 *
 *
 * Measurement model:
 * Z_k = H * X_k + V_k
 *
 *       |1 0 0 0|
 * Z_k = |0 1 0 0| * X_k + V_k
 *
 *
 * V_k = observation_noise ~ N(0,R_K) with R_k = Covariance Matrix
 *
 **************** Kalman Procedure ****************************************************
 *
 * Prediction:
 * - project State ahead      - X_k = A X + B U
 * - project Covariance ahead - P_k = A P_{k-1} * A-transponiert + Q
 *
 *
 * Update:
 * - compute Kalman Gain - K_k = P_k * A-transponiert * ( A * P_k * A-transponiert + R_k )
 *    ... where ( A * P_k * A-transponiert + R_k ) is covariance Matrix S_k
 * - update State Estimate - X_k = X_{k-1} + K_k* (Z_k -A * X_{k-1})
 *    ... where (Z_k -A * X_{k-1}) is the measurement update
 * - update Covariance P_k = (I - K_k * A) P_{k-1}
 *
 ***************** Software CALLS ******************************************************
 *
 * Prediction:
 * - call prediction with time to predict since last measurement
 *
 * Update:
 * - call measurement update wit new measurements
 *
 **************************************************************************************/


class ObjectEstimator
{

 public:
  ~ObjectEstimator();
  ObjectEstimator(fawkes::Logger* logger, fawkes::Clock* clock, fawkes::Configuration* config, fawkes::tf::Transformer* tf_transformer, fawkes::MotorInterface* odom_if);

  void initialize(const fawkes::tf::Stamped<fawkes::tf::Point> point);
  void add_measurement(const fawkes::tf::Stamped<fawkes::tf::Point> point);
  void update(const fawkes::tf::Stamped<fawkes::tf::Point> point);
  void reset(const fawkes::tf::Stamped<fawkes::tf::Point> prior);
  fawkes::tf::Stamped<fawkes::tf::Point> getPositionEstimate(fawkes::Time time);
  //void getEstimate(fawkes::Time time);


 private:

  std::queue<fawkes::tf::Stamped<fawkes::tf::Point>> measurement_queue_;

  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::LinearAnalyticConditionalGaussian*		          sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 cluster_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* cluster_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  fawkes::tf::Stamped<fawkes::tf::Point> filter_estimate_old_;
  fawkes::tf::Stamped<fawkes::tf::Point> cluster_meas_, cluster_meas_old_;
  fawkes::Time filter_time_old_;

  // tf transformer
  fawkes::tf::Transformer* transformer_;

  fawkes::MotorInterface* odom_if_;

  std::string reference_frame_id_;
  std::string measurement_frame_id_;

  fawkes::Logger* logger_;
  fawkes::Clock* clock_;

  float old_odom_x_;
  float old_odom_y_;
  float old_odom_phi_;

//  float sysNoise_Mu_;
//  float sysNoise_Cov_;
//  float measNoise_Mu_;
//  float measNoise_Cov_;


  bool filter_initialized_=false;



};

#endif
