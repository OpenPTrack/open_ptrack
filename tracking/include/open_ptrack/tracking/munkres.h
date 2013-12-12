/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-, Matteo Munaro [matteo.munaro@dei.unipd.it]
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

#ifndef OPEN_PTRACK_TRACKING_MUNKRES_H_
#define OPEN_PTRACK_TRACKING_MUNKRES_H_

#include <iostream>
#include <limits.h>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace open_ptrack
{
  namespace tracking
  {

    class Munkres {

      public:

        // max: true (maximum is the best), false (minimum is the best)
        cv::Mat solve(cv::Mat& matrix, bool max);

      private:

        double** preprocess(cv::Mat& matrix_in, bool max);

        void step_one(double** matrix, int rows, int cols, int& step);

        void step_two(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m, int& step);

        void step_three(int rows, int cols, int* colCover, double** m, int& step);

        void find_a_zero(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& row, int& col);

        bool star_in_row(double** m, int rows, int cols, int row);

        void find_star_in_row(double** m, int rows, int cols, int row, int& col);

        void step_four(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m, int& path_row_0, int& path_col_0, int& step);

        void find_star_in_col(int rows, int cols, double** m, int c, int& r);

        void find_prime_in_row(int rows, int cols, double** m, int r, int& c);

        void augment_path(double** m, int path_count, int** path);

        void clear_covers(int rows, int cols, int* rowCover, int* colCover);

        void erase_primes(int rows, int cols, double** m);

        void step_five(int rows, int cols, int* rowCover, int* colCover, double** m, int& path_row_0, int& path_col_0, int& path_count, int** path, int& step);

        void find_smallest(double** matrix, int rows, int cols, int* rowCover, int* colCover, double& minval);

        void step_six(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& step);

    };
  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* !defined(OPEN_PTRACK_TRACKING_MUNKRES_H_) */
