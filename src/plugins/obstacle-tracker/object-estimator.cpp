/***************************************************************************
 *  Object-Estimator.cpp
 *
 *  Created:
 *  Copyright  2015  Sebastian Reuter
 *
* ****************************************************************************/

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

#include "object-estimator.h"

using namespace fawkes;
using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;

/** Constructor. */
ObjectEstimator::ObjectEstimator(fawkes::Logger* logger, fawkes::Clock* clock, std::string frame_id, std::string child_frame_id)
{

  frame_id_ = frame_id;
  child_frame_id_ = child_frame_id;
  logger_ = logger;
  clock_ = clock;

  /*
   * Create SYSTEM MODEL according to model defined in header
   */
  double dt = 0.001;
  Matrix A(6,6);
  A(1,1) = 1; A(1,2) = 0; A(1,3) = 0;   A(1,4) = dt; A(1,5) = 0; A(1,6) = 0;
  A(2,1) = 0; A(2,2) = 1; A(2,3) = 0;   A(2,4) = 0; A(2,5) = dt; A(2,6) = 0;
  A(3,1) = 0; A(3,2) = 0; A(3,3) = 1;   A(3,4) = 0; A(3,5) = 0; A(3,6) = dt;
  A(4,1) = 0; A(4,2) = 0; A(4,3) = 0;   A(4,4) = 1; A(3,5) = 0; A(4,6) = 0;
  A(5,1) = 0; A(5,2) = 0; A(5,3) = 0;   A(5,4) = 0; A(3,5) = 1; A(5,6) = 0;
  A(6,1) = 0; A(6,2) = 0; A(6,3) = 0;   A(6,4) = 0; A(3,5) = 0; A(6,6) = 1;

  Matrix B(1,6); B = 0.0;
  B(1,1) = pow(dt,2)/2;
  B(1,2) = pow(dt,2)/2;
  B(1,3) = pow(dt,2)/2;
  B(1,4) = dt;
  B(1,5) = dt;
  B(1,6) = dt;

  std::vector<Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  //  Matrix Q(6,6);
  //  Q(1,1) = pow(dt,4)/4;   Q(1,2) = 0;           Q(1,3) = 0;           Q(1,4) = pow(dt,3)/3; Q(1,5) = 0;           Q(1,6) = 0;
  //  Q(2,1) = 0;             Q(2,2) = pow(dt,4)/4; Q(2,3) = 0;           Q(2,4) = 0;           Q(2,5) = pow(dt,3)/3; Q(2,6) = 0;
  //  Q(3,1) = 0;             Q(3,2) = 0;           Q(3,3) = pow(dt,4)/4; Q(3,4) = 0;           Q(3,5) = 0;           Q(3,6) = pow(dt,3)/3;
  //  Q(4,1) = pow(dt,3)/3;   Q(4,2) = 0;           Q(4,3) = 0;           Q(4,4) = pow(dt,2);   Q(3,5) = 0;           Q(4,6) = 0;
  //  Q(5,1) = 0;             Q(5,2) = pow(dt,3)/3; Q(5,3) = 0;           Q(5,4) = 0;           Q(3,5) = pow(dt,2);   Q(5,6) = 0;
  //  Q(6,1) = 0;             Q(6,2) = 0;           Q(6,3) = pow(dt,3)/3; Q(6,4) = 0;           Q(3,5) = 0;           Q(6,6) = pow(dt,2);

  // System noise (has to have state-size)
  ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
  SymmetricMatrix sysNoise_Cov(6,6);
  for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
  BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

  sys_pdf_   = new LinearAnalyticConditionalGaussian(A,system_Uncertainty);
  sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);



  // create MEASUREMENT MODEL CLUSTER - only positions can be measured
  Matrix C(3,6);
  C(1,1) = 1; C(1,2) = 0; C(1,3) = 0;   C(1,4) = 0; C(1,5) = 0; C(1,6) = 0;
  C(2,1) = 0; C(2,2) = 1; C(2,3) = 0;   C(2,4) = 0; C(2,5) = 0; C(2,6) = 0;
  C(3,1) = 0; C(3,2) = 0; C(3,3) = 1;   C(3,4) = 0; C(3,5) = 0; C(3,6) = 0;

  ColumnVector measNoiseCluster_Mu(3);  measNoiseCluster_Mu = 0;
  SymmetricMatrix measNoiseCluster_Cov(3);  measNoiseCluster_Cov = 0;
  for (unsigned int i=1; i<=3; i++) measNoiseCluster_Cov(i,i) = 1; // Diagonal Matrix for C-Matrix
  Gaussian measurement_Uncertainty_Cluster(measNoiseCluster_Mu, measNoiseCluster_Cov);

  cluster_meas_pdf_   = new LinearAnalyticConditionalGaussian(C, measurement_Uncertainty_Cluster);
  cluster_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(cluster_meas_pdf_);

  logger_->log_info("ObjectEstimator", "created Sytem and Cluster Model");

}

/** Destructor. */
ObjectEstimator::~ObjectEstimator()
{
  if (filter_) delete filter_;
  if (prior_)  delete prior_;
}

void
ObjectEstimator::initialize(const fawkes::tf::Transform& prior, const Time& time)
{

  // set prior (first guess) of filter
  ColumnVector prior_Mu(6);
  prior_Mu(1) = prior.getOrigin().getX();
  prior_Mu(2)=prior.getOrigin().getY();
  prior_Mu(3)=prior.getOrigin().getZ();
  prior_Mu(4)= 0;
  prior_Mu(5)= 0;
  prior_Mu(6)= 0;

  SymmetricMatrix prior_Cov(6);
  for (unsigned int i=1; i<=6; i++) {
    for (unsigned int j=1; j<=6; j++){
    	if (i==j)  prior_Cov(i,j) = pow(0.001,2);			// low Covariance states that we are very confident, that the obstacle is at that position
    	else prior_Cov(i,j) = 0;
    }
  }
  prior_  = new Gaussian(prior_Mu,prior_Cov);

  logger_->log_info("ObjectEstimator", "Initialized prior with x=%f y=%f z=%f", prior.getOrigin().getX(), prior.getOrigin().getY(), prior.getOrigin().getZ());

  filter_ = new ExtendedKalmanFilter(prior_);

  logger_->log_info("ObjectEstimator", "Created Filter");

  // remember prior
  fawkes::Time now(clock_);
  this->update(now, StampedTransform(prior, time, frame_id_, child_frame_id_));

  filter_estimate_old_vec_ = prior_Mu;
  filter_estimate_old_ = prior;
  filter_time_old_ = now;

  filter_initialized_=true;

}

void
ObjectEstimator::update(const Time&  filter_time, const fawkes::tf::StampedTransform& meas){

	if(filter_initialized_){

		// get Time-diff since last update
		double dt = filter_time.in_sec() - filter_time_old_.in_sec();
		logger_->log_info("ObjectTracker", "Time diff %f", dt);

		ColumnVector input(6);
		input = 0;

		//odom_rel = get_odom()
		ColumnVector cluster_rel(3);
		cluster_rel(1) = meas.getOrigin().getX();
		cluster_rel(2) = meas.getOrigin().getY();
		cluster_rel(3) = meas.getOrigin().getZ();


		logger_->log_info("ObjectEstimator", "Update Filter");


		filter_->Update(sys_model_, input, cluster_meas_model_, cluster_rel);


		logger_->log_info("ObjectEstimator", "Ask for Estimate and Covariance after Update");

		// LOG
		ColumnVector estimate = filter_->PostGet()->ExpectedValueGet();
		SymmetricMatrix covariance = filter_->PostGet()->CovarianceGet();
		//double prob = filter_->PostGet()->ProbabilityGet(input).getValue();

		logger_->log_info("ObjectEstimator", "Filter Accuracy:");
		logger_->log_info("ObjectEstimator", "Measurement:    %f %f %f", meas.getOrigin().getX(), meas.getOrigin().getY(), meas.getOrigin().getZ());
		logger_->log_info("ObjectEstimator", "Posterior Mean: %f %f %f", estimate(1), estimate(2), estimate(3));
		logger_->log_info("ObjectEstimator", "Estimate size: %f %f %f", estimate(4), estimate(5), estimate(6) );
		logger_->log_info("ObjectEstimator", "Covariance: %f %f %f", covariance(1,1), covariance(1,2), covariance(1,3));
		logger_->log_info("ObjectEstimator", "Covariance: %f %f %f", covariance(2,1), covariance(2,2), covariance(2,3));
		logger_->log_info("ObjectEstimator", "Covariance: %f %f %f", covariance(3,1), covariance(3,2), covariance(3,3));
		//logger_->log_info("ObjectEstimator", "Probability: %f", prob);

		// remember values
		cluster_meas_old_ = cluster_meas_;
		filter_estimate_old_vec_ = estimate;
		filter_time_old_ = filter_time;

	}
}

// get filter posterior at time 'time' as Transform
fawkes::tf::Transform
ObjectEstimator::getEstimate(fawkes::Time time){


	  tf::Quaternion q(0,0,0,1);
	  tf::Vector3 vtrans(0,0,0);
	  fawkes::tf::Transform trans(q,vtrans);
	return trans;

}

