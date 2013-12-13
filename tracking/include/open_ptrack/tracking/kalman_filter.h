/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-, Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso
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
 */

#ifndef OPEN_PTRACK_TRACKING_KALMAN_FILTER_H_
#define OPEN_PTRACK_TRACKING_KALMAN_FILTER_H_

#include <cmath>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>

#include <open_ptrack/bayes/allFilters.hpp>

namespace open_ptrack
{
  namespace tracking
  {
    class PredictModel : public Bayesian_filter::Linear_predict_model
    {
      protected:
        const double dt_;
      public:
        PredictModel(double dt, double acceleration_variance);
        virtual ~PredictModel();
    };

    class ObserveModel : public Bayesian_filter::Linear_uncorrelated_observe_model
    {
      protected:
        const double position_variance_;
      public:
        ObserveModel(double position_variance, int ouput_dimension);
        virtual ~ObserveModel();
    };

    class MahalanobisParameters2d
    {
      public:

        MahalanobisParameters2d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0)
      {
          SI.resize(2, 2, false);
      }

        Bayesian_filter::FM::SymMatrix SI;
        double x;
        double y;
    };

    class MahalanobisParameters4d
    {
      public:

        MahalanobisParameters4d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0), vx(0), vy(0)
      {
          SI.resize(4, 4, false);
      }

        Bayesian_filter::FM::SymMatrix SI;
        double x;
        double y;
        double vx;
        double vy;
    };

    class KalmanFilter
    {
      protected:

        double dt_;
        double position_variance_;
        double depth_multiplier_;
        double acceleration_variance_;
        int output_dimension_;

        Bayesian_filter::Unscented_scheme* filter_;

        PredictModel* predict_model_;
        ObserveModel* observe_model_;

      public:

        KalmanFilter(double dt, double position_variance, double acceleration_variance, int output_dimension);
        KalmanFilter(const KalmanFilter& orig);
        KalmanFilter& operator=(const KalmanFilter& orig);
        virtual ~KalmanFilter();

        void init(double x, double y, double distance, bool velocity_in_motion_term);
        void predict();
        void predict(double& x, double& y, double& vx, double& vy);
        void update();
        void update(double x, double y, double distance);
        void update(double x, double y, double vx, double vy, double distance);
        void getState(double& x, double& y, double& vx, double& vy);
        void getState(double& x, double& y);
        void getMahalanobisParameters(MahalanobisParameters2d& mp);
        void getMahalanobisParameters(MahalanobisParameters4d& mp);
        static double performMahalanobisDistance(double x, double y, const MahalanobisParameters2d& mp);
        static double performMahalanobisDistance(double x, double y, double vx, double vy, const MahalanobisParameters4d& mp);

        Bayesian_filter::FM::SymMatrix getInnovationCovariance();

    };

  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_TRACKING_KALMAN_FILTER_H_ */
