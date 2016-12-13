/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-2012, Matteo Munaro [matteo.munaro@dei.unipd.it]
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include "open_ptrack/tracking/munkres.h"

namespace open_ptrack
{
  namespace tracking
  {
    using namespace std;

    void
    Munkres::step_one(double** matrix, int rows, int cols, int& step) {
      double min_in_row;

      for (int r = 0; r < rows; r++) {
        min_in_row = matrix[r][0];
        for (int c = 0; c < cols; c++)
          if (matrix[r][c] < min_in_row)
            min_in_row = matrix[r][c];
        for (int c = 0; c < cols; c++)
          matrix[r][c] -= min_in_row;
      }
      step = 2;
    }

    void
    Munkres::step_two(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m,
        int& step) {
      for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
          if (matrix[r][c] == 0 && rowCover[r] == 0 && colCover[c] == 0) {
            m[r][c] = 1;
            rowCover[r] = 1;
            colCover[c] = 1;
          }
        }
      for (int r = 0; r < rows; r++)
        rowCover[r] = 0;
      for (int c = 0; c < cols; c++)
        colCover[c] = 0;
      step = 3;
    }

    void
    Munkres::step_three(int rows, int cols, int* colCover, double** m, int& step) {
      int colcount;
      for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
          if (m[r][c] == 1)
            colCover[c] = 1;

      colcount = 0;
      for (int c = 0; c < cols; c++)
        if (colCover[c] == 1)
          colcount += 1;
      if (colcount >= rows || colcount >= cols)
        step = 7;
      else
        step = 4;
    }

    void
    Munkres::find_a_zero(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& row,
        int& col) {
      int r = 0;
      int c;
      bool done;
      row = -1;
      col = -1;
      done = false;
      while (!done) {
        c = 0;
        while (true) {
          if (matrix[r][c] == 0 && rowCover[r] == 0 && colCover[c] == 0) {
            row = r;
            col = c;
            done = true;
          }
          c += 1;
          if (c >= cols || done)
            break;
        }
        r += 1;
        if (r >= rows)
          done = true;
      }
    }

    bool
    Munkres::star_in_row(double** m, int rows, int cols, int row) {
      bool tmp = false;
      for (int c = 0; c < cols; c++)
        if (m[row][c] == 1)
          tmp = true;
      return tmp;
    }

    void
    Munkres::find_star_in_row(double** m, int rows, int cols, int row, int& col) {
      col = -1;
      for (int c = 0; c < cols; c++)
        if (m[row][c] == 1)
          col = c;
    }

    void
    Munkres::step_four(double** matrix, int rows, int cols, int* rowCover, int* colCover, double** m,
        int& path_row_0, int& path_col_0, int& step) {
      int row = -1;
      int col = -1;
      bool done;

      done = false;
      while (!done) {
        find_a_zero(matrix, rows, cols, rowCover, colCover, row, col);
        if (row == -1) {
          done = true;
          step = 6;
        } else {
          m[row][col] = 2;
          if (star_in_row(m, rows, cols, row)) {
            find_star_in_row(m, rows, cols, row, col);
            rowCover[row] = 1;
            colCover[col] = 0;
          } else {
            done = true;
            step = 5;
            path_row_0 = row;
            path_col_0 = col;
          }
        }
      }
    }

    void
    Munkres::find_star_in_col(int rows, int cols, double** m, int c, int& r) {
      r = -1;
      for (int i = 0; i < rows; i++)
        if (m[i][c] == 1)
          r = i;
    }

    void
    Munkres::find_prime_in_row(int rows, int cols, double** m, int r, int& c) {
      for (int j = 0; j < cols; j++)
        if (m[r][j] == 2)
          c = j;
    }

    void
    Munkres::augment_path(double** m, int path_count, int** path) {
      for (int p = 0; p < path_count; p++)
        if (m[path[p][0]][path[p][1]] == 1)
          m[path[p][0]][path[p][1]] = 0;
        else
          m[path[p][0]][path[p][1]] = 1;
    }

    void
    Munkres::clear_covers(int rows, int cols, int* rowCover, int* colCover) {
      for (int r = 0; r < rows; r++)
        rowCover[r] = 0;
      for (int c = 0; c < cols; c++)
        colCover[c] = 0;
    }

    void
    Munkres::erase_primes(int rows, int cols, double** m) {
      for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
          if (m[r][c] == 2)
            m[r][c] = 0;
    }

    void
    Munkres::step_five(int rows, int cols, int* rowCover, int* colCover, double** m, int& path_row_0,
        int& path_col_0, int& path_count, int** path, int& step) {
      bool done;
      int r = -1;
      int c = -1;

      path_count = 1;
      path[path_count - 1][0] = path_row_0;
      path[path_count - 1][1] = path_col_0;
      done = false;
      while (!done) {
        find_star_in_col(rows, cols, m, path[path_count - 1][1], r);
        if (r > -1) {
          path_count += 1;
          path[path_count - 1][0] = r;
          path[path_count - 1][1] = path[path_count - 2][1];
        } else
          done = true;
        if (!done) {
          find_prime_in_row(rows, cols, m, path[path_count - 1][0], c);
          path_count += 1;
          path[path_count - 1][0] = path[path_count - 2][0];
          path[path_count - 1][1] = c;
        }
      }
      augment_path(m, path_count, path);
      clear_covers(rows, cols, rowCover, colCover);
      erase_primes(rows, cols, m);
      step = 3;
    }

    void
    Munkres::find_smallest(double** matrix, int rows, int cols, int* rowCover, int* colCover,
        double& minval) {
      for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
          if (rowCover[r] == 0 && colCover[c] == 0)
            if (minval > matrix[r][c])
              minval = matrix[r][c];
    }

    void
    Munkres::step_six(double** matrix, int rows, int cols, int* rowCover, int* colCover, int& step) {
      double minval = INFINITY;
      find_smallest(matrix, rows, cols, rowCover, colCover, minval);
      for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
          if (rowCover[r] == 1)
            matrix[r][c] += minval;
          if (colCover[c] == 0)
            matrix[r][c] -= minval;
        }
      step = 4;
    }

    double**
    Munkres::preprocess(cv::Mat& matrix_in, bool max)
    {
      // Pad input matrix in order to make it square:
      int max_dim = MAX(matrix_in.rows, matrix_in.cols);
      cv::Mat matrix_in_padded(max_dim, max_dim, CV_64F, 1000000.0);  // padding values are very high numbers (not valid for association)
      for (int i = 0; i < matrix_in.rows; i++)
      {
        for (int j = 0; j < matrix_in.cols; j++)
        {
          matrix_in_padded.at<double>(i,j) = matrix_in.at<double>(i,j);
        }
      }

      // Conversion from cv::Mat to array:
      int rows = matrix_in_padded.rows;
      int cols = matrix_in_padded.cols;
      double** matrix = new double*[rows];
      for (int i = 0; i < rows; i++)
      {
        matrix[i] = new double[cols];
        for (int j = 0; j < cols; j++)
        {
          matrix[i][j] = matrix_in_padded.at<double>(i,j);
        }
      }

      // Change cost matrix according to the type of optimum to find (minimum or maximum):
      if (max == true) {
        double maxValue = matrix[0][0];
        for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
            if (matrix[i][j] > maxValue) {
              maxValue = matrix[i][j];
            }
          }
        }

        for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
            matrix[i][j] = maxValue - matrix[i][j];
          }
        }
      }

      return matrix;
    }

    cv::Mat
    Munkres::solve(cv::Mat& matrix_in, bool max)
    {
      // Preprocessing:
      double** matrix = preprocess(matrix_in, max);

      int max_dim = MAX(matrix_in.rows, matrix_in.cols);
      int rows = max_dim;
      int cols = max_dim;

      bool done = false;
      int step = 1;
      int* rowCover = new int[rows];
      int* colCover = new int[cols];
      double** m = new double*[rows];
      int path_row_0 = 0;
      int path_col_0 = 0;
      int path_count = 0;
      for (int j = 0; j < cols; j++) {
        colCover[j] = 0;
      }
      for (int i = 0; i < rows; i++) {
        rowCover[i] = 0;
        m[i] = new double[cols];
        for (int j = 0; j < cols; j++) {
          m[i][j] = 0;
        }
      }
      int** path = new int*[rows*cols];
      for (int i = 0; i < rows*cols; i++) {
        path[i] = new int[2];
      }

      // Main loop:
      while (!done) {
        switch (step) {
          case 1:
            step_one(matrix, rows, cols, step);
            break;
          case 2:
            step_two(matrix, rows, cols, rowCover, colCover, m, step);
            break;
          case 3:
            step_three(rows, cols, colCover, m, step);
            break;
          case 4:
            step_four(matrix, rows, cols, rowCover, colCover, m, path_row_0, path_col_0,
                step);
            break;
          case 5:
            step_five(rows, cols, rowCover, colCover, m, path_row_0, path_col_0,
                path_count, path, step);
            break;
          case 6:
            step_six(matrix, rows, cols, rowCover, colCover, step);
            break;
          case 7:
            done = true;
            break;
        }
      }

      // Conversion from array to cv::Mat:
      cv::Mat matrix_out(matrix_in.rows, matrix_in.cols, CV_64F);
      for (int i = 0; i < matrix_in.rows; i++)
      {
        for (int j = 0; j < matrix_in.cols; j++)
        {
          matrix_out.at<double>(i,j) = m[i][j] - 1;
        }
      }

      // Releasing the memory
      for(int r = 0; r < max_dim; ++r)
      {
        delete []matrix[r];
      }
      delete []matrix;
      delete []rowCover;
      delete []colCover;
      for (int i = 0; i < rows; i++) {
        delete []m[i];
      }
      delete []m;
      for (int i = 0; i < rows*cols; i++) {
        delete []path[i];
      }
      delete []path;

      return matrix_out;
    }

  } /* namespace tracking */
} /* namespace open_ptrack */

