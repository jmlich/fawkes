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
ObjectEstimator::ObjectEstimator(fawkes::Logger* logger, fawkes::Clock* clock, fawkes::Configuration* config, fawkes::tf::Transformer* tf_transformer, fawkes::MotorInterface* odom_if)
{

  logger_ = logger;
  clock_ = clock;
  transformer_ = tf_transformer;
  odom_if_ = odom_if;

  // GET CONFIGS
  std::string cfg_prefix = "/obstacle-tracker/";
  measurement_frame_id_ = config->get_string((cfg_prefix + "frame/measurement-frame-id").c_str());
  reference_frame_id_ = config->get_string((cfg_prefix + "frame/reference-frame-id").c_str());
  double cfg_sysNoise_Mu = config->get_float((cfg_prefix + "kalman/sysNoise_Mu").c_str());
  double cfg_sysNoise_Cov = config->get_float((cfg_prefix + "kalman/sysNoise_Cov").c_str());
  double cfg_measNoise_Mu = config->get_float((cfg_prefix + "kalman/measNoise_Mu").c_str());
  double cfg_measNoise_Cov = config->get_float((cfg_prefix + "kalman/measNoise_Cov").c_str());

  /*
   * Create SYSTEM MODEL according to model defined in header
   */
  Matrix A(4,4);
  A(1,1) = 1; A(1,2) = 0; A(1,3) = 1;   A(1,4) = 0;
  A(2,1) = 0; A(2,2) = 1; A(2,3) = 0;   A(2,4) = 1;
  A(3,1) = 0; A(3,2) = 0; A(3,3) = 1;   A(3,4) = 0;
  A(4,1) = 0; A(4,2) = 0; A(4,3) = 0;   A(4,4) = 1;

  //logger_->log_info("ObjectEstimator", "Matrix A done");

  // System noise (has to have state-size)
  ColumnVector sysNoise_Mu(4);  sysNoise_Mu = cfg_sysNoise_Mu;
  SymmetricMatrix sysNoise_Cov(4);
  for (unsigned int i=1; i<=4; i++) sysNoise_Cov(i,i) = cfg_sysNoise_Cov;
  BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

  //logger_->log_info("ObjectEstimator", "Gaussian created");

  sys_pdf_   = new LinearAnalyticConditionalGaussian(A,system_Uncertainty);
  sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

  //logger_->log_info("ObjectEstimator", "Sys model done");

  // create MEASUREMENT MODEL CLUSTER - only positions can be measured
  Matrix C(2,4);
  C(1,1) = 1; C(1,2) = 0; C(1,3) = 0;   C(1,4) = 0;
  C(2,1) = 0; C(2,2) = 1; C(2,3) = 0;   C(2,4) = 0;

  //logger_->log_info("ObjectEstimator", "Matrix C done");

  ColumnVector measNoiseCluster_Mu(2);  measNoiseCluster_Mu = cfg_measNoise_Mu;
  SymmetricMatrix measNoiseCluster_Cov(2);  measNoiseCluster_Cov = 0;
  for (unsigned int i=1; i<=2; i++) measNoiseCluster_Cov(i,i) = cfg_measNoise_Cov; // Diagonal Matrix for C-Matrix
  Gaussian measurement_Uncertainty_Cluster(measNoiseCluster_Mu, measNoiseCluster_Cov);

  cluster_meas_pdf_   = new LinearAnalyticConditionalGaussian(C, measurement_Uncertainty_Cluster);
  cluster_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(cluster_meas_pdf_);

  //logger_->log_info("ObjectEstimator", "Cluster model done");

  logger_->log_info("ObjectEstimator", "Created Sytem and Cluster Model");
  logger_->log_info("ObjectEstimator", "Config Values SysModel Mu=%f Cov=%f MeasModel Mu=%f Cov=%f", cfg_sysNoise_Mu, cfg_sysNoise_Cov, cfg_measNoise_Mu, cfg_measNoise_Cov);

}




/** Destructor. */
ObjectEstimator::~ObjectEstimator()
{
  if (filter_) delete filter_;
  if (prior_)  delete prior_;
  if(sys_model_) delete sys_model_;
  if(sys_pdf_) delete sys_pdf_;
  if(cluster_meas_pdf_) delete cluster_meas_pdf_;
  if(cluster_meas_model_) delete cluster_meas_model_;
}



/* initialize filter
 *
 * create system model
 * create measurement model
 * create fi  logger_->log_info("ObjectEstimator", "Prior initialized with %f %f", transformed_prior.getX(), transformed_prior.getY());
 * lter
 *
 */
void
ObjectEstimator::initialize(const fawkes::tf::Stamped<fawkes::tf::Point> prior)
{

  // transform prior into /map
  fawkes::tf::Stamped<tf::Point> transformed_prior;
  while(true){
	  try{
		  transformer_->transform_point(reference_frame_id_, prior, transformed_prior);
		  if(transformed_prior.getX()!=0) break;
	  }
	  catch(Exception &e){}
  }

  // set prior (first guess) of filter
  ColumnVector prior_Mu(4);
  prior_Mu(1)= transformed_prior.getX();
  prior_Mu(2)= transformed_prior.getY();
  prior_Mu(3)= 0;
  prior_Mu(4)= 0;

  SymmetricMatrix prior_Cov(4);
  for (unsigned int i=1; i<=4; i++) {
    for (unsigned int j=1; j<=4; j++){
    	if (i==j)  prior_Cov(i,j) = pow(0.001,2);	// low Covariance states that we are very confident, that the obstacle is at that position
    	else prior_Cov(i,j) = 0;
    }
  }
  prior_  = new Gaussian(prior_Mu,prior_Cov);
  filter_ = new ExtendedKalmanFilter(prior_);

  logger_->log_info("ObjectEstimator", "Filter initialized");
  logger_->log_info("ObjectEstimator", "Prior is %f %f", transformed_prior.getX(), transformed_prior.getY());



  // remember prior
  fawkes::Time now(clock_);
  this->update(transformed_prior);




  filter_estimate_old_vec_ = prior_Mu;
  filter_estimate_old_ = transformed_prior;
  filter_time_old_ = now;


  filter_initialized_=true;

}



/* resets the filter and adds prior as first guess
 *
 * a while loop is used to force wait till transform is ready
 * this will most probably break the loop
 * we have to think about another solution
 *
 */
void
ObjectEstimator::reset(const fawkes::tf::Stamped<fawkes::tf::Point> prior){

	// transform prior into /map
	fawkes::tf::Stamped<tf::Point> transformed_prior;
	while(true){
		try{
			transformer_->transform_point(reference_frame_id_, prior, transformed_prior);
			if(transformed_prior.getX()!=0) break;
		}
		catch(Exception &e){
			  logger_->log_warn("ObjectEstimator", "Looping till transform in %s is ready", reference_frame_id_.c_str());
			  logger_->log_warn("ObjectEstimator", e);
		}
	}
    logger_->log_info("ObjectEstimator", "Transform of PRIOR is ready: %f %f", transformed_prior.getX(), transformed_prior.getY());

    ColumnVector prior_Mu(4);
	prior_Mu(1)= transformed_prior.getX();
	prior_Mu(2)= transformed_prior.getY();
	prior_Mu(3)= 0;
	prior_Mu(4)= 0;

    SymmetricMatrix prior_Cov(4);
	for (unsigned int i=1; i<=4; i++) {
	  for (unsigned int j=1; j<=4; j++){
	  	if (i==j)  prior_Cov(i,j) = pow(0.001,2);	// low Covariance states that we are very confident, that the obstacle is at that position
	  	else prior_Cov(i,j) = 0;
	  }
	}

	prior_->CovarianceSet(prior_Cov);
	prior_->ExpectedValueSet(prior_Mu);

	// reset filterconst fawkes::tf::Stamped<fawkes::tf::Point> meas
	filter_->Reset(prior_);

	// clear measurement queue
	measurement_queue_ = {};

    logger_->log_info("ObjectEstimator", "Filter reseted !!!!!!!!!!!!!!!!");

}

/* updates filter
 *
 * adds the measurement to the measurement queue
 * runs through queue and tries to transform measurement into map frame
 * if transform is ready, the measurement is added to the filter
 *
 */
void
ObjectEstimator::update(const fawkes::tf::Stamped<fawkes::tf::Point> meas){


	// push measurement into queue
	measurement_queue_.push(meas);

	if(filter_initialized_){

		// tray to transform each element of the queue
		for(uint i=0; i<measurement_queue_.size(); i++){

			bool transform_ready = false;

			// measurement in reference-frame
			fawkes::tf::Stamped<tf::Point> transformed_meas;

			try{
			   transformer_->transform_point(reference_frame_id_, measurement_queue_.front(), transformed_meas);
				   if(transformed_meas.getX()!=0){
					   transform_ready = true;
				   }
			}
			catch(Exception &e){
			    logger_->log_info("ObjectEstimator", "Transform of measurement was not ready");
			}

			// transformation of measurement was successfull
			if(transform_ready){


				// measurement is ready - add it to filter

				this->add_measurement(transformed_meas);

				// measurement is added - delete from queue
				measurement_queue_.pop();

//			    logger_->log_info("ObjectEstimator", "Transform of measurement is ready: %f %f", transformed_meas.getX(), transformed_meas.getY());
//				logger_->log_info("ObjectEstimator", "Add measurement. Current queue size is: %d", measurement_queue_.size() );

			}

			// if not ready break loop - the successors will have a later timestamp and must not be tested
			else{
				break;
			}
		}
	}

	/*
	 * TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 * Handle overflow of queue
	 * If amcl is not loaded, the transform to \map will never be ready
	 * catch this failure by unloading plugin or flushing the queue if max-size is exceeded
	 *
	 */
}


/* add measurement to filter
 *
 * expects measurements to be in /map frame
 * is called by update function
 *
 */
void
ObjectEstimator::add_measurement(const fawkes::tf::Stamped<fawkes::tf::Point> transformed_meas){


	if(filter_initialized_){

		// safe cluster measurement in Column Vector
		ColumnVector cluster_rel(2);
		cluster_rel(1) =  transformed_meas.getX();
		cluster_rel(2) = transformed_meas.getY();

		// update bfl filter
		filter_->Update(sys_model_, cluster_meas_model_, cluster_rel);

		// remember values
		cluster_meas_old_ = cluster_meas_;
		filter_time_old_ = transformed_meas.stamp;

	}
}

/* get estimate
 *
 * simulate filter for n timesteps
 *
 */
fawkes::tf::Stamped<fawkes::tf::Point>
ObjectEstimator::getPositionEstimate(fawkes::Time time){


	// estimate position
	ColumnVector estimate = filter_->PostGet()->ExpectedValueGet();
	SymmetricMatrix covariance = filter_->PostGet()->CovarianceGet();

	//double prob = filter_->PostGet()->ProbabilityGet(input).getValue();

	//  LOGGING
	logger_->log_info("ObjectEstimator", "Posterior Position: %f %f", estimate(1), estimate(2));
	logger_->log_info("ObjectEstimator", "Posterior Velocity: %f %f", estimate(3), estimate(4));
	logger_->log_info("ObjectEstimator", "Covariance: %f %f %f %f", covariance(1,1), covariance(1,2), covariance(1,3), covariance(1,4));
	logger_->log_info("ObjectEstimator", "Covariance: %f %f %f %f", covariance(2,1), covariance(2,2), covariance(2,3), covariance(2,4));
	logger_->log_info("ObjectEstimator", "Covariance: %f %f %f %f", covariance(3,1), covariance(3,2), covariance(3,3), covariance(3,4));
	logger_->log_info("ObjectEstimator", "Covariance: %f %f %f %f", covariance(4,1), covariance(4,2), covariance(4,3), covariance(4,4));

	fawkes::tf::Point estimate_as_point( estimate(1), estimate(2), 0.0 );
	fawkes::tf::Stamped<fawkes::tf::Point> estimated_stamped_point(estimate_as_point, time, reference_frame_id_);

	return estimated_stamped_point;

}

