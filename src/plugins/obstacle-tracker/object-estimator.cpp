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

  Matrix A(3,3);
  A(1,1) = 1; A(1,2) = 0; A(1,3) = 0;
  A(2,1) = 0; A(2,2) = 1; A(2,3) = 0;
  A(3,1) = 0; A(3,2) = 0; A(3,3) = 1;

  Matrix B(3,3);


  // create SYSTEM MODEL
  ColumnVector sysNoise_Mu(3);  sysNoise_Mu = 0;
  SymmetricMatrix sysNoise_Cov(3,3); sysNoise_Cov = 0;
  for (unsigned int i=1; i<=3; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
  BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
  sys_pdf_   = new LinearAnalyticConditionalGaussian(A,system_Uncertainty);
  sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

  // create MEASUREMENT MODEL CLUSTER
  ColumnVector measNoiseCluster_Mu(3);  measNoiseCluster_Mu = 0;
  SymmetricMatrix measNoiseCluster_Cov(3);  measNoiseCluster_Cov = 0;
  for (unsigned int i=1; i<=3; i++) measNoiseCluster_Cov(i,i) = 1; // Diagonal Matrix for H-Matrix
  Gaussian measurement_Uncertainty_Cluster(measNoiseCluster_Mu, measNoiseCluster_Cov);
  Matrix Hcluster(3,3);  Hcluster = 0;
  Hcluster(1,1) = 1;    Hcluster(2,2) = 1; Hcluster(3,3) = 1;
  cluster_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hcluster, measurement_Uncertainty_Cluster);
  cluster_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(cluster_meas_pdf_);


  logger_->log_info("ObjectEstimator", "created Sytem and Cluster Model");

}

/** Destructor. */
ObjectEstimator::~ObjectEstimator()
{
  if (filter_) delete filter_;
  if (prior_)  delete prior_;
  // delete odom_meas_model_;
  // delete odom_meas_pdf_;
  // delete cluster_meas_model_;
  // delete cluster_meas_pdf_;
  // delete sys_pdf_;
  // delete sys_model_;
}

void
ObjectEstimator::initialize(const fawkes::tf::Transform& prior, const Time& time)
{

  // set prior of filter
  ColumnVector prior_Mu(3);
  prior_Mu(1) = prior.getOrigin().getX();
  prior_Mu(2)=prior.getOrigin().getY();
  prior_Mu(3)=prior.getOrigin().getZ();
  SymmetricMatrix prior_Cov(3);
  for (unsigned int i=1; i<=3; i++) {
    for (unsigned int j=1; j<=3; j++){
  if (i==j)  prior_Cov(i,j) = pow(0.001,2);
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
		logger_->log_error("ObjectTracker", "Time diff %f", dt);
		// only update filter when it is initialized

		// only update filter for time later than current filter time

		// system update filter
		// --------------------
		// for now only add system noise
		ColumnVector input(3); input = 0;

		logger_->log_info("ObjectEstimator", "Updated Filter with sysmodel and 0,0,0");


		// process cluster measurement
		// ------------------------

		//odom_rel = get_odom()
		ColumnVector cluster_rel(3);
		cluster_rel(1) = meas.getOrigin().getX();
		cluster_rel(2) = meas.getOrigin().getY();
		cluster_rel(3) = meas.getOrigin().getZ();

	    //  CRASHER HIER !!!
	    //  update filter
	    // filter_->Update(sys_model_, input, cluster_meas_model_, cluster_rel);
	    filter_->Update(sys_model_, cluster_meas_model_, cluster_rel);

		logger_->log_info("ObjectEstimator", "Ask for Estimate and Covariance after Update");

		// LOG
		ColumnVector estimate = filter_->PostGet()->ExpectedValueGet();
		double prob = filter_->PostGet()->ProbabilityGet(input).getValue();

		SymmetricMatrix covariance = filter_->PostGet()->CovarianceGet();
		logger_->log_info("ObjectEstimator", "Updates Filter with Cluster: %f %f ", meas.getOrigin().getX(), meas.getOrigin().getY());
		logger_->log_info("ObjectEstimator", "Posterior Mean: %f %f", estimate(1), estimate(2));
		logger_->log_info("ObjectEstimator", "Covariance: %f %f", covariance(1,1), covariance(1,2));
		logger_->log_info("ObjectEstimator", "Covariance: %f %f", covariance(2,1), covariance(2,2));
		logger_->log_info("ObjectEstimator", "Probability: %f", prob);

		// remember values
		cluster_meas_old_ = cluster_meas_;
		filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
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

