/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013 Riccardo Levorato, Filippo Basso
 * Copyright (c) 2014-, Open Perception, Inc.
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Riccardo Levorato [levorato@dei.unipd.it], Filippo Basso [bassofil@dei.unipd.it]
 *
 */

#ifndef OPEN_PTRACK_OPT_CALIBRATION_MULTICAMERA_CALIBRATION_H_
#define OPEN_PTRACK_OPT_CALIBRATION_MULTICAMERA_CALIBRATION_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>
#include <calibration_common/algorithms/automatic_checkerboard_finder.h>
#include <calibration_common/pinhole/camera_model.h>

using namespace camera_info_manager;
using namespace calibration;

namespace open_ptrack
{
  namespace opt_calibration
  {
    const int MAX_LEVEL = 10000;
    const double MAX_DISTANCE = 10000.0;

    /** \brief struct containing camera information */
    struct Camera
    {
        Camera (int id, const image_transport::Subscriber & image_sub,
            const ros::Subscriber & camera_info_sub) :
            id_ (id), image_sub_ (image_sub), camera_info_sub_ (
                camera_info_sub), level_ (MAX_LEVEL), distance_ (MAX_DISTANCE)
        {
          std::stringstream ss;
          ss << "/camera_" << id;
          name_ = ss.str ();
        }

        Camera (int id) :
            id_ (id), level_ (MAX_LEVEL), distance_ (MAX_DISTANCE)
        {
          std::stringstream ss;
          ss << "/camera_" << id;
          name_ = ss.str ();
        }

        /** \brief Camera ID */
        int id_;

        /** \brief Camera name */
        std::string name_;

        /** \brief ROS subscriber to the topic where the camera publishes the RGB image. */
        image_transport::Subscriber image_sub_;

        /** \brief ROS subscriber to the topic where the camera publishes the intrinsic calibration information. */
        ros::Subscriber camera_info_sub_;

        /** \brief PinholeSensor object related to Camera */
        PinholeSensor::Ptr sensor_;

        /** \brief Camera level in the camera tree representing the calibration links. */
        int level_;

        /** \brief Distance between cameras. */
        double distance_;

        /** \brief Message containing the image */
        sensor_msgs::Image::ConstPtr image_msg_;

    };

    /** MultiCameraCalibration performs extrinsic calibration of a network of cameras */
    class MultiCameraCalibration
    {
      public:

        /** \brief Constructor */
        MultiCameraCalibration (ros::NodeHandle & node_handle);

        /**
         * \brief Callback for images.
         *
         * \param[in] msg Message containing the image.
         * \param[in] id Camera id.
         */
        void
        imageCallback (const sensor_msgs::Image::ConstPtr & msg, int id);

        /**
         * \brief Callback for camera_info topic, containing intrinsic camera calibration.
         *
         * \param[in] msg Message containing camera info (intrinsic parameters).
         * \param[in] id Camera id.
         */
        void
        cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr &msg,
            int id);

        /** \brief Callback for string messages which enables saving options.
         *
         * \param[in] msg Message containing the command as a string.
         */
        void
        actionCallback (const std_msgs::String::ConstPtr &msg);

        /** \brief Calibration initialization */
        bool
        initialize ();

        /** \brief Calibration main loop */
        void
        spin ();

      private:

        /** \brief Use OpenCV method to find the checkerboard in the images.
         *
         * \param[in] image The image where the checkerboard has to be found.
         * \param[in] id The camera ID.
         * \param[in] color_view Contains the checkerboard corners position in the image.
         *
         */
        bool
        findCheckerboard (cv::Mat & image, int id,
            typename PinholeView<Checkerboard>::Ptr & color_view);

        /** \brief Save cameras extrinsic calibration as ROS transforms in a launch file */
        void
        saveTF ();

        /** \brief Save cameras extrinsic calibration in a launch file which can be used for multi-camera tracking */
        void
        saveExtrinsicCalibration ();

        /** \brief Save cameras position with respect to checkerboard (world) frame (require all cameras see the checkerboard) */
        void
        saveWorldToCam ();

        /** \brief Handle to the ROS node. */
        ros::NodeHandle node_handle_;

        /** \brief  Handle to ImageTransport, which advertise and subscribe to image topics. */
        image_transport::ImageTransport image_transport_;

        /** \brief Subscriber to topic where action commands are sent. */
        ros::Subscriber action_sub_;

        /** \brief Vector of Camera objects */
        std::vector<Camera> camera_vector_;

        /** \brief Object representing a checkerboard. */
        Checkerboard::Ptr checkerboard_;

        /** \brief Object for finding checkerboards in images. */
        AutomaticCheckerboardFinder finder_;

        /** \brief Number of cameras connected to the network. */
        int num_cameras_;

        /** \brief Number of cameras extrinsically calibrated. */
        int calibrated_cameras_;

        /** \brief Global reference frame. */
        BaseObject::Ptr world_;

        /** \brief Listener to ROS transforms. */
        tf::TransformListener tfListener;

        /** \brief ROS broadcaster publishing transforms (TF). */
        tf::TransformBroadcaster tf_pub_;

        /** \brief Publisher sending checkerboard markers to be visualized with RViz. */
        ros::Publisher marker_pub_;

        /** \brief Flag stating if the world frame has been set or not. */
        bool world_set_;

        /** \brief Frame ID of the base camera (used for global frame calibration). */
        std::string base_camera_frame_id_;

    };
  } /* namespace opt_calibration */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_OPT_CALIBRATION_MULTICAMERA_CALIBRATION_H_ */
