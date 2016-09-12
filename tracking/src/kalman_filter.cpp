/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-2012, Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
 *
 */

#include <ros/ros.h>

#include <open_ptrack/tracking/kalman_filter.h>

namespace open_ptrack
{
  namespace tracking
  {

    PredictModel::PredictModel(double dt, double acceleration_variance) :
		    Bayesian_filter::Linear_predict_model(4, 2), dt_(dt)
    {
      for(size_t i = 0; i < 4; i++)
        for(size_t j = 0; j < 4; j++)
          Fx(i, j) = 0.0;

      Fx(0, 0) = 1.0;
      Fx(1, 1) = 1.0;
      Fx(2, 2) = 1.0;
      Fx(3, 3) = 1.0;
      Fx(0, 2) = dt;
      Fx(1, 3) = dt;

      q[0] = acceleration_variance;
      q[1] = acceleration_variance;

      for(size_t i = 0; i < 4; i++)
        for(size_t j = 0; j < 2; j++)
          G(i, j) = 0.0;

      G(0, 0) = dt * dt / 2.0;
      G(1, 1) = dt * dt / 2.0;
      G(2, 0) = dt;
      G(3, 1) = dt;
    }

    PredictModel::~PredictModel()
    {

    }

    ObserveModel::ObserveModel(double position_variance, int output_dimension) :
		    Bayesian_filter::Linear_uncorrelated_observe_model(4, output_dimension), position_variance_(position_variance)
    {

      for(size_t i = 0; i < output_dimension; i++)
        for(size_t j = 0; j < 4; j++)
          Hx(i, j) = 0.0;

      Hx(0, 0) = 1.0;
      Hx(1, 1) = 1.0;

      Zv[0] = position_variance_;
      Zv[1] = position_variance_;
      if (output_dimension == 4)
      {
        Hx(2, 2) = 1.0;
        Hx(3, 3) = 1.0;
        Zv[2] = 16 * position_variance_;
        Zv[3] = 16 * position_variance_;
      }

    }

    ObserveModel::~ObserveModel()
    {

    }

    KalmanFilter::KalmanFilter(double dt, double position_variance, double acceleration_variance, int output_dimension) :
		    dt_(dt), position_variance_(position_variance), depth_multiplier_(std::pow(0.005 / 1.96, 2)),
		    acceleration_variance_(acceleration_variance), output_dimension_(output_dimension)
    {
      predict_model_ = new PredictModel(dt, acceleration_variance);
      observe_model_ = new ObserveModel(position_variance, output_dimension);
      filter_ = new Bayesian_filter::Unscented_scheme(4);
    }

    KalmanFilter::KalmanFilter(const KalmanFilter& orig)
    {
      *this = orig;
    }

    KalmanFilter&
    KalmanFilter::operator=(const KalmanFilter& orig)
    {
      this->dt_ = orig.dt_;
      this->position_variance_ = orig.position_variance_;
      this->depth_multiplier_ = orig.depth_multiplier_;
      this->acceleration_variance_ = orig.acceleration_variance_;
      delete this->predict_model_;
      delete this->observe_model_;
      delete this->filter_;
      this->predict_model_ = new PredictModel(dt_, acceleration_variance_);
      this->observe_model_ = new ObserveModel(position_variance_, output_dimension_);
      this->filter_ = new Bayesian_filter::Unscented_scheme(4, 2);

      this->filter_->s.resize(orig.filter_->s.size(), false);
      for(size_t i = 0; i < orig.filter_->s.size(); i++)
      {
        this->filter_->s(i) = orig.filter_->s(i);
      }

      this->filter_->S.resize(orig.filter_->S.size1(), orig.filter_->S.size2(), false);
      for(size_t i = 0; i < orig.filter_->S.size1(); i++)
      {
        for(size_t j = 0; j < orig.filter_->S.size2(); j++)
        {
          this->filter_->S(i, j) = orig.filter_->S(i, j);
        }
      }

      this->filter_->SI.resize(orig.filter_->SI.size1(), orig.filter_->SI.size2(), false);
      for(size_t i = 0; i < orig.filter_->SI.size1(); i++)
      {
        for(size_t j = 0; j < orig.filter_->SI.size2(); j++)
        {
          this->filter_->SI(i, j) = orig.filter_->SI(i, j);
        }
      }

      this->filter_->x.resize(orig.filter_->x.size(), false);
      for(size_t i = 0; i < orig.filter_->x.size(); i++)
      {
        this->filter_->x(i) = orig.filter_->x(i);
      }

      this->filter_->X.resize(orig.filter_->X.size1(), orig.filter_->X.size2(), false);
      for(size_t i = 0; i < orig.filter_->X.size1(); i++)
      {
        for(size_t j = 0; j < orig.filter_->X.size2(); j++)
        {
          this->filter_->X(i, j) = orig.filter_->X(i, j);
        }
      }

      this->filter_->XX.resize(orig.filter_->XX.size1(), orig.filter_->XX.size2(), false);
      for(size_t i = 0; i < orig.filter_->XX.size1(); i++)
      {
        for(size_t j = 0; j < orig.filter_->XX.size2(); j++)
        {
          this->filter_->XX(i, j) = orig.filter_->XX(i, j);
        }
      }

      return *this;
    }

    KalmanFilter::~KalmanFilter()
    {
      delete predict_model_;
      delete observe_model_;
      delete filter_;
    }

    void
    KalmanFilter::init(double x, double y, double distance, bool velocity_in_motion_term)
    {

      Bayesian_filter_matrix::Vec state(4);
      Bayesian_filter_matrix::SymMatrix cov(4, 4);

      state[0] = x;
      state[1] = y;
      state[2] = 0.0;
      state[3] = 0.0;

      for(size_t i = 0; i < 4; i++)
        for(size_t j = 0; j < 4; j++)
          cov(i, j) = 0.0;

      cov(2, 2) = 100; //1000.0;
      cov(3, 3) = 100; //1000.0;

      // Filter initialization:
      filter_->init_kalman(state, cov);

      // First update:
      if (velocity_in_motion_term)
        update(x, y, 0, 0, distance);
      else
        update(x, y, distance);

    }

    void
    KalmanFilter::predict()
    {
      filter_->predict(*predict_model_);
    }

    void
    KalmanFilter::predict(double& x, double& y, double& vx, double& vy)
    {
      predict();

      x = filter_->x[0];
      y = filter_->x[1];
      vx = filter_->x[2];
      vy = filter_->x[3];
    }

    void
    KalmanFilter::update()
    {
      filter_->update();
      //filter_->update_XX(2.0);
    }

    void
    KalmanFilter::update(double x, double y, double distance)
    {

      Bayesian_filter_matrix::Vec observation(2);
      observation[0] = x;
      observation[1] = y;

      //printf("%d %f %f %f ", _id, x, y, height);

      observe_model_->Zv[0] = position_variance_ + std::pow(distance, 4) * depth_multiplier_;
      observe_model_->Zv[1] = position_variance_ + std::pow(distance, 4) * depth_multiplier_;

      filter_->observe(*observe_model_, observation);
      filter_->update();
      //filter_->update_XX(2.0);

    }

    void
    KalmanFilter::update(double x, double y, double vx, double vy, double distance)
    {

      Bayesian_filter_matrix::Vec observation(4);
      observation[0] = x;
      observation[1] = y;
      observation[2] = vx;
      observation[3] = vy;

      //printf("%d %f %f %f ", _id, x, y, height);

      //	observe_model_->Zv[0] = position_variance_ + std::pow(distance, 4) * depth_multiplier_;
      //	observe_model_->Zv[1] = position_variance_ + std::pow(distance, 4) * depth_multiplier_;
      //	observe_model_->Zv[2] = 16 * position_variance_ + std::pow(distance, 4) * depth_multiplier_;
      //	observe_model_->Zv[3] = 16 * position_variance_ + std::pow(distance, 4) * depth_multiplier_;

      filter_->observe(*observe_model_, observation);
      filter_->update();
      //filter_->update_XX(2.0);

    }

    void
    KalmanFilter::getMahalanobisParameters(MahalanobisParameters2d& mp)
    {
      mp.SI = filter_->SI;
      mp.x = filter_->x[0];
      mp.y = filter_->x[1];
    }

    void
    KalmanFilter::getMahalanobisParameters(MahalanobisParameters4d& mp)
    {
      mp.SI = filter_->SI;
      mp.x = filter_->x[0];
      mp.y = filter_->x[1];
      mp.vx = filter_->x[2];
      mp.vy = filter_->x[3];
    }

    double
    KalmanFilter::performMahalanobisDistance(double x, double y, const MahalanobisParameters2d& mp)
    {
      Bayesian_filter_matrix::Vec v(2);
      v[0] = x - mp.x;
      v[1] = y - mp.y;
      return Bayesian_filter_matrix::prod_SPDT(v, mp.SI);
    }

    double
    KalmanFilter::performMahalanobisDistance(double x, double y, double vx, double vy, const MahalanobisParameters4d& mp)
    {
      Bayesian_filter_matrix::Vec v(4);
      v[0] = x - mp.x;
      v[1] = y - mp.y;
      v[2] = vx - mp.vx;
      v[3] = vy - mp.vy;

      // Symmetric Positive (Semi) Definite multiply: p = v'*(mp.SI)*v
      return Bayesian_filter_matrix::prod_SPDT(v, mp.SI);
    }

    Bayesian_filter::FM::SymMatrix
    KalmanFilter::getInnovationCovariance()
    {
      return filter_->SI;
    }

    void
    KalmanFilter::getState(double& x, double& y, double& vx, double& vy)
    {
      x = filter_->x[0];
      y = filter_->x[1];
      vx = filter_->x[2];
      vy = filter_->x[3];
    }

    void
    KalmanFilter::getState(double& x, double& y)
    {
      x = filter_->x[0];
      y = filter_->x[1];
    }

    void
    KalmanFilter::setPredictModel (double acceleration_variance)
    {
      acceleration_variance_ = acceleration_variance;
      predict_model_ = new PredictModel(dt_, acceleration_variance_);
    }

    void
    KalmanFilter::setObserveModel (double position_variance)
    {
      position_variance_ = position_variance;
      observe_model_ = new ObserveModel(position_variance_, output_dimension_);
    }
  } /* namespace tracking */
} /* namespace open_ptrack */
