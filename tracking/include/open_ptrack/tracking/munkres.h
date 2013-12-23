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
    /** \brief Munkres solves Global Nearest Neighbor problem with the Hungarian (Munkres) algorithm
     *  Implementation of the algorithm described here: http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
     **/
    class Munkres {

      public:

        /**
         * \brief Main method for solving GNN problems.
         *
         * \param[in] matrix Cost matrix for the GNN problem (rows are workers, columns are jobs)
         * \param[in] max If true, higher values are considered to be better. If false, lower values are better (lower cost).
         */
        cv::Mat solve(cv::Mat& matrix, bool max);

      private:

        /* \brief Preprocessing for applying the Munkres algorithm. */
        double**
        preprocess(cv::Mat& matrix_in, bool max);

        /* \brief First step of the Munkres algorithm. */
        void
        step_one(double** matrix, int rows, int cols, int& step);

        /* \brief Second step of the hungarian algorithm. */
        void
        step_two(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m, int& step);

        /* \brief Third step of the hungarian algorithm. */
        void
        step_three(int rows, int cols, int* colCover, double** m, int& step);

        /* \brief Utility function for step 4. */
        void
        find_a_zero(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& row, int& col);

        /* \brief Utility function for step 4. */
        bool
        star_in_row(double** m, int rows, int cols, int row);

        /* \brief Utility function for step 4. */
        void
        find_star_in_row(double** m, int rows, int cols, int row, int& col);

        /* \brief First step of the hungarian algorithm. */
        void
        step_four(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m, int& path_row_0, int& path_col_0, int& step);

        /* \brief Utility function for step 5. */
        void
        find_star_in_col(int rows, int cols, double** m, int c, int& r);

        /* \brief Utility function for step 5. */
        void
        find_prime_in_row(int rows, int cols, double** m, int r, int& c);

        /* \brief Utility function for step 5. */
        void
        augment_path(double** m, int path_count, int** path);

        /* \brief Utility function for step 5. */
        void
        clear_covers(int rows, int cols, int* rowCover, int* colCover);

        /* \brief Utility function for step 5. */
        void
        erase_primes(int rows, int cols, double** m);

        /* \brief Fifth step of the hungarian algorithm. */
        void
        step_five(int rows, int cols, int* rowCover, int* colCover, double** m, int& path_row_0, int& path_col_0, int& path_count, int** path, int& step);

        /* \brief Utility function for step 6. */
        void
        find_smallest(double** matrix, int rows, int cols, int* rowCover, int* colCover, double& minval);

        /* \brief Sixth step of the hungarian algorithm. */
        void
        step_six(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& step);

    };
  } /* namespace tracking */
} /* namespace open_ptrack */
#endif /* !defined(OPEN_PTRACK_TRACKING_MUNKRES_H_) */
