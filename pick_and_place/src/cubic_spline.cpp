/**
 * @file cubic_spline.cpp
 * @brief Implementation of natural cubic spline interpolation
 * @version 0.1
 * @date 2026-02-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "cubic_spline.h"
#include <cmath>
#include <algorithm>
#include <iostream>

CubicSpline::CubicSpline() : initialized_(false)
{
}

CubicSpline::~CubicSpline()
{
}

bool CubicSpline::setWaypoints(const std::vector<double>& times, 
                               const std::vector<std::vector<double>>& positions)
{
    // Input validation
    if (times.size() < 2)
    {
        std::cerr << "Error: Need at least 2 waypoints for spline interpolation" << std::endl;
        return false;
    }

    if (times.size() != positions.size())
    {
        std::cerr << "Error: Times and positions size mismatch" << std::endl;
        return false;
    }

    // Check that all position vectors have N_DOF elements
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (positions[i].size() != N_DOF)
        {
            std::cerr << "Error: Position " << i << " has " << positions[i].size() 
                      << " elements, expected " << N_DOF << std::endl;
            return false;
        }
    }

    // Check that times are strictly increasing
    for (size_t i = 1; i < times.size(); i++)
    {
        if (times[i] <= times[i-1])
        {
            std::cerr << "Error: Times must be strictly increasing" << std::endl;
            return false;
        }
    }

    // Store waypoints
    times_ = times;
    waypoints_ = positions;

    // Compute spline coefficients for each joint
    spline_joints_.resize(N_DOF);
    
    for (int joint_idx = 0; joint_idx < N_DOF; joint_idx++)
    {
        // Extract values for this joint across all waypoints
        std::vector<double> joint_values(times.size());
        for (size_t wp_idx = 0; wp_idx < times.size(); wp_idx++)
        {
            joint_values[wp_idx] = positions[wp_idx][joint_idx];
        }

        // Compute spline for this joint
        computeSplineCoefficients(times, joint_values, spline_joints_[joint_idx]);
    }

    initialized_ = true;
    return true;
}

void CubicSpline::computeSplineCoefficients(const std::vector<double>& times,
                                           const std::vector<double>& values,
                                           std::vector<SplineSegment>& segments)
{
    int n = times.size();
    segments.resize(n - 1);

    // Compute h (time deltas)
    std::vector<double> h(n - 1);
    for (int i = 0; i < n - 1; i++)
    {
        h[i] = times[i + 1] - times[i];
    }

    //  Set up tridiagonal system for second derivatives (M)
    // Using natural boundary conditions: M[0] = M[n-1] = 0
    std::vector<double> a(n);  // Lower diagonal
    std::vector<double> b(n);  // Main diagonal
    std::vector<double> c(n);  // Upper diagonal
    std::vector<double> d(n);  // Right hand side
    std::vector<double> M(n);  // second derivatives

    // Clamped boundary conditions (Velocity = 0 at start and end)
    
    // Start (v0 = 0): 2*m0 + m1 = 3/h0 * ((y1-y0)/h0 - v0)  <-- Factor 3, not 6
    b[0] = 2.0;
    c[0] = 1.0;
    d[0] = 3.0 * ((values[1] - values[0]) / h[0]) / h[0];

    // End (vn = 0): mn-2 + 2*mn-1 = 3/hn-1 * (vn - (yn-1-yn-2)/hn-1)
    a[n-1] = 1.0;
    b[n-1] = 2.0;
    d[n-1] = -3.0 * ((values[n-1] - values[n-2]) / h[n-2]) / h[n-2];

    // Interior points
    for (int i = 1; i < n - 1; i++)
    {
        a[i] = h[i-1];
        b[i] = 2.0 * (h[i-1] + h[i]);
        c[i] = h[i];
        d[i] = 3.0 * ((values[i+1] - values[i]) / h[i] - 
                      (values[i] - values[i-1]) / h[i-1]);
    }

    // Solve for M (second derivatives)
    solveTridiagonal(a, b, c, d, M);

    // Compute spline coefficients for each segment
    for (int i = 0; i < n - 1; i++)
    {
        double hi = h[i];
        
        segments[i].t_start = times[i];
        segments[i].a = values[i];
        segments[i].b = (values[i+1] - values[i]) / hi - hi * (2.0 * M[i] + M[i+1]) / 3.0;
        segments[i].c = M[i];
        segments[i].d = (M[i+1] - M[i]) / (3.0 * hi);
    }
}

void CubicSpline::solveTridiagonal(const std::vector<double>& a,
                                   const std::vector<double>& b,
                                   const std::vector<double>& c,
                                   const std::vector<double>& d,
                                   std::vector<double>& x)
{
    int n = b.size();
    std::vector<double> c_prime(n);
    std::vector<double> d_prime(n);

    x.resize(n);

    // Forward sweep
    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];

    for (int i = 1; i < n; i++)
    {
        double denom = b[i] - a[i] * c_prime[i-1];
        c_prime[i] = c[i] / denom;
        d_prime[i] = (d[i] - a[i] * d_prime[i-1]) / denom;
    }

    // Back substitution
    x[n-1] = d_prime[n-1];
    for (int i = n - 2; i >= 0; i--)
    {
        x[i] = d_prime[i] - c_prime[i] * x[i+1];
    }
}

int CubicSpline::findSegment(double time) const
{
    if (!initialized_ || time < times_.front() || time > times_.back())
    {
        return -1;
    }

    // Binary search for the segment
    int left = 0;
    int right = times_.size() - 2;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;
        
        if (time >= times_[mid] && time <= times_[mid + 1])
        {
            return mid;
        }
        else if (time < times_[mid])
        {
            right = mid - 1;
        }
        else
        {
            left = mid + 1;
        }
    }

    return -1;
}

bool CubicSpline::interpolate(double time, double* positions) const
{
    if (!initialized_)
    {
        std::cerr << "Error: Spline not initialized" << std::endl;
        return false;
    }

    int segment_idx = findSegment(time);
    if (segment_idx < 0)
    {
        std::cerr << "Error: Time " << time << " out of range [" 
                  << times_.front() << ", " << times_.back() << "]" << std::endl;
        return false;
    }

    // Interpolate each joint
    for (int joint_idx = 0; joint_idx < N_DOF; joint_idx++)
    {
        const SplineSegment& seg = spline_joints_[joint_idx][segment_idx];
        double dt = time - seg.t_start;
        
        // Evaluate: s(t) = a + b*dt + c*dt^2 + d*dt^3
        positions[joint_idx] = seg.a + seg.b * dt + seg.c * dt * dt + seg.d * dt * dt * dt;
    }

    return true;
}

bool CubicSpline::getVelocity(double time, double* velocities) const
{
    if (!initialized_)
    {
        std::cerr << "Error: Spline not initialized" << std::endl;
        return false;
    }

    int segment_idx = findSegment(time);
    if (segment_idx < 0)
    {
        std::cerr << "Error: Time " << time << " out of range [" 
                  << times_.front() << ", " << times_.back() << "]" << std::endl;
        return false;
    }

    // Calculate velocity for each joint
    for (int joint_idx = 0; joint_idx < N_DOF; joint_idx++)
    {
        const SplineSegment& seg = spline_joints_[joint_idx][segment_idx];
        double dt = time - seg.t_start;
        
        // Velocity: v(t) = b + 2*c*dt + 3*d*dt^2
        velocities[joint_idx] = seg.b + 2.0 * seg.c * dt + 3.0 * seg.d * dt * dt;
    }

    return true;
}


bool CubicSpline::validateLimits(const double* lower_limits, const double* upper_limits) const
{
    if (!initialized_)
    {
        std::cerr << "Error: Spline not initialized" << std::endl;
        return false;
    }

    bool all_valid = true;

    for (size_t wp_idx = 0; wp_idx < waypoints_.size(); wp_idx++)
    {
        for (int joint_idx = 0; joint_idx < N_DOF; joint_idx++)
        {
            double value = waypoints_[wp_idx][joint_idx];
            
            if (value < lower_limits[joint_idx] || value > upper_limits[joint_idx])
            {
                std::cerr << "Warning: Waypoint " << wp_idx << ", joint " << joint_idx 
                          << " value " << value << " outside limits [" 
                          << lower_limits[joint_idx] << ", " << upper_limits[joint_idx] << "]" 
                          << std::endl;
                all_valid = false;
            }
        }
    }

    return all_valid;
}

bool CubicSpline::getTimeRange(double& start_time, double& end_time) const
{
    if (!initialized_)
    {
        return false;
    }

    start_time = times_.front();
    end_time = times_.back();
    return true;
}
