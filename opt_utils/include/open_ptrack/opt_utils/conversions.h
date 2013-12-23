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

#ifndef OPEN_PTRACK_OPT_UTILS_CONVERSIONS_H_
#define OPEN_PTRACK_OPT_UTILS_CONVERSIONS_H_

#include <geometry_msgs/Vector3.h>
#include <Eigen/Eigen>

namespace open_ptrack
{
  namespace opt_utils
  {
    /** \brief Conversions provides useful data conversions */
    class Conversions
    {
      public:

        /**
         * \brief Conversion from Eigen::Vector3d to geometry_msgs::Vector3.
         *
         * \param[in] v Input vector.
         *
         * \return the converted vector.
         */
        static geometry_msgs::Vector3
        Vector3dToVector3(const Eigen::Vector3d& v);

        /**
         * \brief Conversion from Eigen::Vector3f to geometry_msgs::Vector3.
         *
         * \param[in] v Input vector.
         *
         * \return the converted vector.
         */
        static geometry_msgs::Vector3
        Vector3fToVector3(const Eigen::Vector3f& v);

        /**
         * \brief Conversion from Eigen::Vector3d to geometry_msgs::Vector3
         *
         * \param[in] v Input vector.
         * \param[out] out Output vector.
         */
        static void
        Vector3dToVector3(const Eigen::Vector3d& v, geometry_msgs::Vector3& out);

        /**
         * \brief Conversion from Eigen::Vector3f to geometry_msgs::Vector3
         *
         * \param[in] v Input vector.
         * \param[out] out Output vector.
         */
        static void
        Vector3fToVector3(const Eigen::Vector3f& v, geometry_msgs::Vector3& out);

        /**
         * \brief Find projection of a 3D point to an image.
         *
         * \param[in] world The 3D point to project.
         * \param[in] intrinsics Intrinsic parameters of the camera.
         *
         * \return the 2D point resulting from the projection (in homogeneous coordinates).
         */
        static Eigen::Vector3d
        world2cam(const Eigen::Vector3d& world, const Eigen::Matrix3d& intrinsics);

        /**
         * \brief Find projection of a 3D point to an image.
         *
         * \param[in] world The 3D point to project.
         * \param[in] intrinsics Intrinsic parameters of the camera.
         *
         * \return the 2D point resulting from the projection (in homogeneous coordinates).
         */
        static Eigen::Vector3f
        world2cam(const Eigen::Vector3f& world, const Eigen::Matrix3f& intrinsics);

        /**
         * \brief Find projection of a 3D point to an image.
         *
         * \param[in] world The 3D point to project.
         * \param[out] cam The 2D point resulting from the projection (in homogeneous coordinates).
         * \param[in] intrinsics Intrinsic parameters of the camera.
         *
         */
        static void
        world2cam(const Eigen::Vector3d& world, Eigen::Vector3d& cam, const Eigen::Matrix3d& intrinsics);

        /**
         * \brief Find projection of a 3D point to an image.
         *
         * \param[in] world The 3D point to project.
         * \param[out] cam The 2D point resulting from the projection (in homogeneous coordinates).
         * \param[in] intrinsics Intrinsic parameters of the camera.
         *
         */
        static void
        world2cam(const Eigen::Vector3f& world, Eigen::Vector3f& cam, const Eigen::Matrix3f& intrinsics);

        /**
         * \brief Conversion from radians to degrees.
         *
         * \param[in] radians The angle in radians.
         *
         * \return the angle in degrees.
         */
        static inline double
        rad2deg(double radians)
        {
          return radians * 180 / M_PI;
        }

        /**
         * \brief Conversion from degrees to radians.
         *
         * \param[in] degrees Angle in degrees.
         *
         * \return the angle in radians.
         */
        static inline double
        deg2rad(double degrees)
        {
          return degrees * M_PI / 180;
        }
    };
  } /* namespace opt_utils */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_OPT_UTILS_CONVERSIONS_H_ */
