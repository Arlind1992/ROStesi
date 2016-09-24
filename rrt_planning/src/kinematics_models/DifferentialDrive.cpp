/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rrt_planning/kinematics_models/DifferentialDrive.h"
#include "rrt_planning/utils/RandomGenerator.h"
#include <boost/numeric/odeint.hpp>

using namespace Eigen;
using namespace std;
using namespace boost::numeric::odeint;

namespace rrt_planning
{

DifferentialDrive::DifferentialDrive()
{
    stateSize = 3;
    actionSize = 2;
}

Eigen::VectorXd DifferentialDrive::compute(const VectorXd& x0, const VectorXd& u, double delta)
{
    this->u = u;

    runge_kutta4<state_type,double,state_type,double,vector_space_algebra> stepper;
    VectorXd x = x0;
    integrate_const(stepper, *this, x, 0.0, delta, dt);

    return x;
}

VectorXd DifferentialDrive::applyTransform(const VectorXd& x0, const VectorXd& T)
{
    VectorXd xf = x0;

    double theta = x0(2);

    xf(0) += cos(theta)*T(0) - sin(theta)*T(1);
    xf(1) += sin(theta)*T(0) + cos(theta)*T(1);
    xf(2) += T(2);

    return xf;
}

Eigen::VectorXd DifferentialDrive::getInitialState()
{
    return VectorXd::Zero(3);
}

Eigen::VectorXd DifferentialDrive::getRandomState(const Bounds& bounds)
{
    VectorXd xRand;
    xRand.setRandom(3);

    xRand += VectorXd::Ones(3);
    xRand /= 2;

    xRand(0) *= bounds.maxX - bounds.minX;
    xRand(1) *= bounds.maxY - bounds.minY;
    xRand(2) *= 2*M_PI;

    xRand(0) += bounds.minX;
    xRand(1) += bounds.minY;
    xRand(2) += -M_PI;

    return xRand;
}


VectorXd DifferentialDrive::anyAngleSampling(vector<geometry_msgs::PoseStamped>& plan,
        double width, double deltaTheta)
{
    // Retrive the total plan length
    double planLength = 0;

    for(int i = 0; i < plan.size() - 1; i++)
    {
        auto&& p1 = plan[i].pose.position;
        auto&& p2 = plan[i+1].pose.position;
        planLength += sqrt( (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) );
    }

    // Sample random params to get the point within the lane
    double l = RandomGenerator::sampleUniform(0.0, planLength);
    double r = RandomGenerator::sampleUniform(0.0, width);
    double a = RandomGenerator::sampleUniform(0.0, 2*M_PI);

    // Retrive the random point on the path
    int pos;
    double parLength = 0;

    for(pos = 0; pos < plan.size() - 1; pos++)
    {
        auto&& p1 = plan[pos].pose.position;
        auto&& p2 = plan[pos+1].pose.position;
        parLength += sqrt( (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) );

        if(parLength >= l) break;
    }

    auto&& p1 = plan[pos].pose.position;
    auto&& p2 = plan[pos+1].pose.position;
    double segLength = sqrt( (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) );
    parLength -= segLength;

    double x_c = p1.x + (l - parLength) / segLength * (p2.x - p1.x);
    double y_c = p1.y + (l - parLength) / segLength * (p2.y - p1.y);

    // Retrive the random point within the circle centered in (x_c, y_c)
    VectorXd p(3);
    p(0) = x_c + r * cos(a);
    p(1) = y_c + r * sin(a);

    // Compute the angle for each segment
    double a_segments[plan.size()];

    for(int i = 0; i < plan.size() - 1; i++)
    {
        auto&& p1 = plan[pos].pose.position;
        auto&& p2 = plan[pos+1].pose.position;
        a_segments[i] = atan2(p2.y - p1.y, p2.x - p1.x);
    }

    // Compute the weight for each segment
    double weights[plan.size()];

    for(int i = 0; i < plan.size() - 1; i++)
    {
        auto&& p1 = plan[pos].pose.position;
        auto&& p2 = plan[pos+1].pose.position;
        Vector2d v1 = {p2.x - p1.x, p2.y - p1.y};
        Vector2d v2 = {p(0) - p1.x, p(1) - p1.y};

        double projectionLength = v1.dot(v2) / v1.norm();

        // Trapezoidal membership function
        if(projectionLength <= -width || projectionLength >= v1.norm() + width)
            weights[i] = 0;
        else if(projectionLength >= width && projectionLength <= v1.norm() - width)
            weights[i] = 1;
        else if(projectionLength > v1.norm() - width)
            weights[i] = 1 + (v1.norm() - projectionLength) / (2 * width);
        else
            weights[i] = projectionLength / (2 * width);
    }

    // Compute the orientation a_bar
    double a_bar = 0;
    double w_norm = 0;

    for(int i = 0; i < plan.size() - 1; i++)
    {
        a_bar += weights[i] * a_segments[i];
        w_norm += weights[i];
    }
    a_bar /= w_norm;

    // Add a random angle
    p(2) = a_bar + RandomGenerator::sampleUniform(-deltaTheta, deltaTheta);

    return p;
}


void DifferentialDrive::operator()(const state_type& x, state_type& dx,
                                   const double /* t */)
{
    Matrix<double, 3, 2> A;

    double theta = x(2);

    A << cos(theta), 0,
    sin(theta), 0,
    0, 1;

    dx = A*u;

}

}
