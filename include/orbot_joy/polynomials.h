/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef POLYNOMIALS_H_
#define POLYNOMIALS_H_

#include <Eigen/Dense>
#include "mg_msgs/PolyPVA.h"

namespace tucker_polynomials {

// Computes n!
size_t factorial(size_t n);

// Coefficients go from lower order to higher order
// Basis is [1 t/1! t^2/2! ... t^n/n!]
// polynomial time is in the interval t \in [0 1]
//  e.g.: 3 + 2t/(1!) + t^2/(2!) ==> coeff = [3 2 1]
class Polynomial {
 public:
    double t0_;              // Initial time for definition of polynomial
    double tf_;              // Final time for definition of polynomial
    int order_;              // Polynomial order
    Eigen::VectorXd coeff_;  // Coefficients

    // Constructor
    Polynomial();
    Polynomial(const double &t0,
               const double &tf,
               const int &coeff_size);
    Polynomial(const double &t0,
               const double &tf,
               const int &coeff_size,
               const Eigen::VectorXd &coeff);
    Polynomial(const double &t0,
	           const double &tf,
               const std::vector<float> &std_coeff);

    // Methods
    Polynomial& operator=(const Polynomial& other) {
        t0_ = other.t0_;
        tf_ = other.tf_;
        order_ = other.order_;
        coeff_ = other.coeff_;
        return *this;
    }
    void PrintPolyCoeff();                       // Print all coefficients
    void PolyAtTime(const double &time,
                    double *result) const;       // Return polynomial value at given time
    void PolyDiffAtTime(const double &time,
                    double *result) const;       // Return derivative of polynomial value at given time
};

// 2D trajectories characterized by two polynomials
class Poly2D {
 public:
    Polynomial poly_x_;
    Polynomial poly_y_;
    double t0_;              // Initial time for definition of polynomials
    double tf_;              // Final time for definition of polynomials
    int order_;              // Polynomial order

    // Constructors
    Poly2D();
    Poly2D(const mg_msgs::PolyPVA &poly_x, const mg_msgs::PolyPVA &poly_y);

    // Methods
    void PrintCoeffs();                           // Print all coefficients
    void PolyAtTime(const double &time,
                    Eigen::Vector2d *result) const;  // Return 2d value for polynomials at a given time
    Eigen::Vector2d PolyAtTime(const double &time) const;
    Eigen::Vector2d PolyDiffAtTime(const double &time) const;
    void SamplePoly(const double &sampling_period,
		            std::vector<Eigen::Vector2d> *sampled_poly,
		            std::vector<double> *time) const;
};

// 2D trajectories characterized by a set of 2D polynomials
// The first segment is expected to start at t0 = 0
// Each segment's tf has to match the t0 of the subsequent polynomial
class Trajectory2D {
 public:
    std::vector<Poly2D> segments_poly_;
    int n_segments_;
    double tf_;
    // Constructor
    Trajectory2D() { };
    Trajectory2D(const std::vector<mg_msgs::PolyPVA> &segments_x,
                 const std::vector<mg_msgs::PolyPVA> &segments_y);

    // Methods
    // Note: this function does not guarantee that the sampled trajectory will have the desired
    //  sampling frequency, but will be close. Also, some points in the trajectory will be doubled
    //  since tf of a segment is included, as well as t0 on the next segment
    void InitTraj(const std::vector<mg_msgs::PolyPVA> &segments_x,
                 const std::vector<mg_msgs::PolyPVA> &segments_y);
    void SampleTraj(const double &sampling_period,
                    std::vector<Eigen::Vector2d> *sampled_traj,
                    std::vector<double> *time);
    Eigen::Vector2d TrajAtTime(const double &time) const;
    Eigen::Vector2d TrajDiffAtTime(const double &time) const;
};

}  // namespace tucker_polynomials

#endif  // POLYNOMIALS_H_