/*
Software License Agreement (BSD License)

Copyright (c) 2013, Southwest Research Institute
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
 * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPEN_PTRACK_DETECTION_HAARDISPADA_H
#define OPEN_PTRACK_DETECTION_HAARDISPADA_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <map>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opt_msgs/Rois.h>
#include <opt_msgs/RoiRect.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace open_ptrack
{
  namespace detection
  {
    using  std::string;
    using  std::vector;
    using  cv::Mat;
    using  cv::Rect;
    using  cv::Range;
    using  cv::Size;
    using  cv::INTER_AREA;

    /*****************************************************************************
     ** Class
     *****************************************************************************/

    class HaarDispAdaClassifier{

      public:
        HaarDispAdaClassifier();
        HaarDispAdaClassifier(string file_name);
        void init();
        bool useMissingDataMask_;
        bool loaded;
        void load(string filename);
        int addToTraining(vector<Rect> &R_in, vector<int> &Labels_in, Mat &Disparity_in);
        void train(string filename);
        void detect(vector<Rect> &R_in,
            vector<int> &L_in,
            Mat &D_in,
            vector<Rect> &R_out,
            vector<int> &L_out,
            bool label_all);

        void detect(vector<Rect> &R_in,
            vector<int> &L_in,
            Mat &D_in,
            vector<Rect> &R_out,
            vector<int> &L_out,
            vector<float> &C_out,
            bool label_all);

        int test();
        int numSamples_;
        float HaarDispAdaPrior_;
        void setMaxSamples(int n);
        int getMaxSamples(){ return(maxSamples_);};
        float getMinConfidence();
        void setMinConfidence(float );

      private:
        CvBoost HDAC_;
        string classifier_filename_; // for loading and saving
        int maxSamples_;
        int num_filters_;
        Mat integralImage_;
        Mat trainingSamples_;
        Mat trainingLabels_;
        Mat trainingMissingMask_;
        Mat AvePosTrainingImg_;

        // images set by alpah_map()
        Mat map; // set by alpha_map() in haar_response()

        // images set by setDImageRoi()
        Mat Image4Haar;
        Mat haar16x16;
        Mat haar8x8;
        Mat haar4x4;

        // Minimum classifier confidence for people detection:
        float min_confidence_;

        // private functions
        void setDImageRoi(Rect &R_in, Mat &I_in);
        void setDImageROI_fast(Rect & R_in, Mat &I_in);
        void alpha_map(int idx);
        void print_map(int k);// a debugging tool
        void print_scaled_map(cv::Mat &A);// a debugging tool
        void print_Image4Haar();// a debugging tool
        int haar_features(Mat & HF, Mat & MH);
        int haar_features_fast(Mat & HF);
        void mask_scale(Mat & input, Mat & output);
        float find_central_disparity(int x, int y, int height, int width, Mat& D_in);
    };
  }  // namespace detection
}  // namespace open_ptrack

#endif /*  OPEN_PTRACK_DETECTION_HAARDISPADA_H */

