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

#include "polynomials.h"

namespace tucker_polynomials {

	size_t factorial(size_t n) {
		return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
	}

	// Constructor methods
	Polynomial::Polynomial() {
	    t0_ = 0.0;
	    tf_ = 1.0;
	    order_ = 2;
	    coeff_ = Eigen::MatrixXd::Zero(1, 1);
	}

	Polynomial::Polynomial(const double &t0,
	                       const double &tf,
	                       const int &coeff_size) {
	    t0_ = t0;
	    tf_ = tf;
	    order_ = coeff_size - 1;
	    coeff_ = Eigen::MatrixXd::Zero(coeff_size, 1);
	}

	Polynomial::Polynomial(const double &t0,
	                       const double &tf,
	                       const int &coeff_size,
                           const Eigen::VectorXd &coeff) {
	    t0_ = t0;
	    tf_ = tf;
	    order_ = coeff_size - 1;
	    coeff_ = coeff;
	}
	Polynomial::Polynomial(const double &t0,
	                       const double &tf,
                           const std::vector<float> &std_coeff) {
	    const uint coeff_size = std_coeff.size();
	    t0_ = t0;
	    tf_ = tf;
	    order_ = std_coeff.size() - 1;
	    Eigen::VectorXd coeff = Eigen::MatrixXd::Zero(coeff_size, 1);
	    for (uint i = 0; i < coeff_size; i++) {
	    	coeff[i] = std_coeff[i];
	    }
	    coeff_ = coeff;
	}

	// print all coefficients
	void Polynomial::PrintPolyCoeff() {
	    int n_poly_coeff = order_ + 1;
	    for (int i = 0; i < n_poly_coeff; i++) {
	        std::cout << coeff_(i) << " ";
	    }
	    std::cout << std::endl;
	}

	// return polynomial value at given time
	void Polynomial::PolyAtTime(const double &time,
	                            double *result) const {
	    const double alpha = tf_ - t0_;
	    const int m = order_ + 1;
	    const double t = (time - t0_)/alpha;
	    *result = 0.0;

	    if (t != 0) {
	        for (size_t i = 0; i < m; i++) {
	            *result = *result + coeff_(i)*pow(t, i) / factorial(i);
	        }
	    } else {
	        *result = coeff_(0);
	    }
	}

	void Polynomial::PolyDiffAtTime(const double &time,
	                            double *result) const {
	    const double alpha = tf_ - t0_;
	    const int m = order_ + 1;
	    const double t = (time - t0_)/alpha;
	    *result = 0.0;

	    if (t != 0) {
	        for (size_t i = 0; i < m-1; i++) {
	            *result = *result + coeff_(i+1)*pow(t, i) / factorial(i);
	        }
	    } else {
	        *result = coeff_(1);
	    }
	}


	// Constructors
	Poly2D::Poly2D() {
	    int n_coeff = 1;
	    double t0_in = 0.0;
	    double tf_in = 1.0;
	    poly_x_.coeff_.resize(n_coeff);
	    poly_y_.coeff_.resize(n_coeff);
	    poly_x_ = Polynomial(t0_in, tf_in, n_coeff);
	    poly_y_ = Polynomial(t0_in, tf_in, n_coeff);
	    t0_ = t0_in;
	    tf_ = tf_in;
	}
	Poly2D::Poly2D(const mg_msgs::PolyPVA &poly_x, const mg_msgs::PolyPVA &poly_y) {
	    t0_ = poly_x.t0;
	    tf_ = poly_x.tf;
	    std::vector<float> coeff_x = poly_x.PosCoeff;
	    std::vector<float> coeff_y = poly_y.PosCoeff;
	    poly_x_ = Polynomial(t0_, tf_, coeff_x);
	    poly_y_ = Polynomial(t0_, tf_, coeff_y);
	    order_ = poly_x.order;
	}

	void Poly2D::PrintCoeffs() {
	        std::cout << "t0: " << t0_ << "\ttf: " << tf_ <<  std::endl;
	        std::cout << "polyX: "; poly_x_.PrintPolyCoeff();
	        std::cout << "polyY: "; poly_y_.PrintPolyCoeff();
	}

	void Poly2D::PolyAtTime(const double &time,
	                        Eigen::Vector2d *result) const {
	    double x, y;
	    poly_x_.PolyAtTime(time, &x);
	    poly_y_.PolyAtTime(time, &y);
	    *result = Eigen::Vector2d(x, y);
	}

	Eigen::Vector2d Poly2D::PolyAtTime(const double &time) const {
	    double x, y;
	    poly_x_.PolyAtTime(time, &x);
	    poly_y_.PolyAtTime(time, &y);
	    return Eigen::Vector2d(x, y);
	}

	Eigen::Vector2d Poly2D::PolyDiffAtTime(const double &time) const {
	    double x, y;
	    poly_x_.PolyDiffAtTime(time, &x);
	    poly_y_.PolyDiffAtTime(time, &y);
	    return Eigen::Vector2d(x, y);
	}

	void Poly2D::SamplePoly(const double &sampling_period,
		                    std::vector<Eigen::Vector2d> *sampled_poly,
		                    std::vector<double> *time) const {
	    double t = t0_;
	    while (t < tf_) {
	    	sampled_poly->push_back(this->PolyAtTime(t));
	    	time->push_back(t);
	    	t = t + sampling_period;
	    }
	    sampled_poly->push_back(this->PolyAtTime(tf_));
	}

	// Constructor
    Trajectory2D::Trajectory2D(const std::vector<mg_msgs::PolyPVA> &segments_x,
                               const std::vector<mg_msgs::PolyPVA> &segments_y) {
    	n_segments_ = segments_x.size();
    	tf_ = segments_x[n_segments_-1].tf;
    	for (uint i = 0; i < n_segments_; i++) {
    		segments_poly_.push_back(Poly2D(segments_x[i], segments_y[i]));
    	}
    }
    
    void Trajectory2D::InitTraj(const std::vector<mg_msgs::PolyPVA> &segments_x,
                 const std::vector<mg_msgs::PolyPVA> &segments_y) {
    	n_segments_ = segments_x.size();
    	tf_ = segments_x[n_segments_-1].tf;
    	for (uint i = 0; i < n_segments_; i++) {
    		segments_poly_.push_back(Poly2D(segments_x[i], segments_y[i]));
    	}
    }

    Eigen::Vector2d Trajectory2D::TrajAtTime(const double &time) const {
    	for (int i = 0;i<n_segments_;i++)
    	{
    		if(time>segments_poly_[i].t0_ && time<segments_poly_[i].tf_)
    		{
    			return segments_poly_[i].PolyAtTime(time);
    		}
    	}
    }

    Eigen::Vector2d Trajectory2D::TrajDiffAtTime(const double &time) const {
    	for (int i = 0;i<n_segments_;i++)
    	{
    		if(time>segments_poly_[i].t0_ && time<segments_poly_[i].tf_)
    		{
    			return segments_poly_[i].PolyDiffAtTime(time);
    		}
    	}
    }

    void Trajectory2D::SampleTraj(const double &sampling_period,
		                          std::vector<Eigen::Vector2d> *sampled_traj,
		                          std::vector<double> *time) {
    	double t = 0.0;
    	uint poly_idx = 0;
    	while (t < tf_) {
    		time->push_back(t);
    		if (t >= segments_poly_[poly_idx].tf_) {
    			poly_idx = poly_idx + 1;
    		}
    		sampled_traj->push_back(segments_poly_[poly_idx].PolyAtTime(t));
    		t = t + sampling_period;
    	}
    	// Add the last sample point (at time t_f)
    	sampled_traj->push_back(segments_poly_[n_segments_-1].PolyAtTime(tf_));
    }

}  // namespace tucker_polynomials