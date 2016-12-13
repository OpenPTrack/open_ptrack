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
    /** \brief PredictModel Prediction model (linear state predict model) */
    class PredictModel : public Bayesian_filter::Linear_predict_model
    {
      protected:
        /* \brief time step */
        const double dt_;

      public:
        /** \brief Constructor. */
        PredictModel(double dt, double acceleration_variance);

        /** \brief Destructor. */
        virtual ~PredictModel();
    };

    /** \brief ObserveModel Observation model (linear observation is additive uncorrelated model) */
    class ObserveModel : public Bayesian_filter::Linear_uncorrelated_observe_model
    {
      protected:
        /** \brief Position variance. */
        double position_variance_;

      public:
        /** \brief Constructor. */
        ObserveModel(double position_variance, int ouput_dimension);

        /** \brief Destructor. */
        virtual ~ObserveModel();
    };

    /** \brief MahalanobisParameters2d Contains variables for bayesian estimation with state dimension = 2. */
    class MahalanobisParameters2d
    {
      public:

        MahalanobisParameters2d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0)
      {
          SI.resize(2, 2, false);
      }

        /** \brief Innovation covariance matrix. */
        Bayesian_filter::FM::SymMatrix SI;

        /** \brief Position x component. */
        double x;

        /** \brief Position y component. */
        double y;
    };

    /** \brief MahalanobisParameters4d Contains variables for bayesian estimation with state dimension = 4. */
    class MahalanobisParameters4d
    {
      public:

        MahalanobisParameters4d() : SI(Bayesian_filter_matrix::Empty), x(0), y(0), vx(0), vy(0)
      {
          SI.resize(4, 4, false);
      }

        /** \brief Innovation covariance matrix. */
        Bayesian_filter::FM::SymMatrix SI;

        /** \brief Position x component. */
        double x;

        /** \brief Position y component. */
        double y;

        /** \brief Velocity x component. */
        double vx;

        /** \brief Velocity y component. */
        double vy;
    };

    /** \brief KalmanFilter provides methods for bayesian estimation with Kalman Filter. */
    class KalmanFilter
    {
      protected:

        /** \brief Time interval.*/
        double dt_;

        /** \brief Scale factor for computing depth noise variance.*/
        double depth_multiplier_;

        /** \brief Position variance. */
        double position_variance_;

        /** \brief Acceleration variance.*/
        double acceleration_variance_;

        /** \brief State/output dimension.*/
        int output_dimension_;

        Bayesian_filter::Unscented_scheme* filter_;

        /** \brief Prediction model. */
        PredictModel* predict_model_;

        /** \brief Observation model. */
        ObserveModel* observe_model_;

      public:

        /** \brief Constructor. */
        KalmanFilter(double dt, double position_variance, double acceleration_variance, int output_dimension);

        /** \brief Constructor initializing a new KalmanFilter with another one. */
        KalmanFilter(const KalmanFilter& orig);

        /** \brief Overload of = operator for copying KalmanFilter objects. */
        KalmanFilter& operator=(const KalmanFilter& orig);

        /** \brief Destructor. */
        virtual ~KalmanFilter();

        /**
         * \brief Filter initialization procedure.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] distance Distance from the sensor.
         * \param[in] velocity_in_motion_term If true, both target position and velocity constitute the output vector.
         */
        void
        init(double x, double y, double distance, bool velocity_in_motion_term);

        /**
         * \brief Prediction step.
         */
        void
        predict();

        /**
         * \brief Prediction step.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         * \param[out] vx Velocity x component.
         * \param[out] vy Velocity y component.
         */
        void
        predict(double& x, double& y, double& vx, double& vy);

        /**
         * \brief Update step.
         */
        void
        update();

        /**
         * \brief Update step.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] distance Distance from the sensor.
         */
        void
        update(double x, double y, double distance);

        /**
         * \brief Update step.
         *
         * \param[in] x Position x component.
         * \param[in] y Position y component.
         * \param[in] vx Velocity x component.
         * \param[in] vy Velocity y component.
         * \param[in] distance Distance from the sensor.
         */
        void
        update(double x, double y, double vx, double vy, double distance);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         * \param[out] vx Velocity x component.
         * \param[out] vy Velocity y component.
         */
        void
        getState(double& x, double& y, double& vx, double& vy);

        /**
         * \brief Get filter state.
         *
         * \param[out] x Position x component.
         * \param[out] y Position y component.
         */
        void
        getState(double& x, double& y);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 2.
         *
         * \param[out] mp Object of class MahalanobisParameters2d.
         */
        void
        getMahalanobisParameters(MahalanobisParameters2d& mp);

        /**
         * \brief Obtain variables for bayesian estimation with output dimension = 4.
         *
         * \param[out] mp Object of class MahalanobisParameters4d.
         */
        void
        getMahalanobisParameters(MahalanobisParameters4d& mp);

        /**
         * \brief Compute Mahalanobis distance between measurement and target predicted state.
         *
         * \param[in] x Input x position.
         * \param[in] y Input y position.
         * \param[in] mp Object of class MahalanobisParameters2d.
         *
         * \return Mahalanobis distance between measurement (x,y) and target predicted state.
         */
        static double
        performMahalanobisDistance(double x, double y, const MahalanobisParameters2d& mp);

        /**
         * \brief Compute Mahalanobis distance between measurement and target predicted state.
         *
         * \param[in] x Input x position.
         * \param[in] y Input y position.
         * \param[in] vx Input x velocity.
         * \param[in] vy Input y velocity.
         * \param[in] mp Object of class MahalanobisParameters4d.
         *
         * \return Mahalanobis distance between measurement (x,y,vx,vy) and target predicted state.
         */
        static double
        performMahalanobisDistance(double x, double y, double vx, double vy, const MahalanobisParameters4d& mp);

        /**
         * \brief Get filter innovation covariance.
         *
         * \return innovation covariance matrix.
         */
        Bayesian_filter::FM::SymMatrix
        getInnovationCovariance();

        /**
         * \brief Set prediction model.
         *
         * \param[in] acceleration_variance Acceleration variance.
         */
        void
        setPredictModel (double acceleration_variance);

        /**
         * \brief Set observation model.
         *
         * \param[in] position_variance Position variance.
         */
        void
        setObserveModel (double position_variance);

    };

  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_TRACKING_KALMAN_FILTER_H_ */
