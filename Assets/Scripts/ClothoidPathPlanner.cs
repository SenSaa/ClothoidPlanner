using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Utils;
using MathNet.Numerics;
using Accord.Math;

/*
Based on:
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ClothoidPath/clothoid_path_planner.py
Atsushi Sakai
-
Clothoid Path Planner
Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (AtsushiSakai)
Reference paper: Fast and accurate G1 fitting of clothoid curves
https://www.researchgate.net/publication/237062806
*/

public class ClothoidPathPlanner
{

    // Generate multiple clothoid paths from multiple orientations(yaw) at start points,
    // to multiple orientations (yaw) at goal point.

    public List<List<Point>> generate_clothoid_paths(Point start_point, List<double> start_yaw_list,
                                        Point goal_point, List<float> goal_yaw_list, 
                                        int n_path_points)
    {
        var clothoids = new List<List<Point>>();
        foreach (var start_yaw in start_yaw_list)
        {
            foreach(var goal_yaw in goal_yaw_list)
            {
                var clothoid = generate_clothoid_path(start_point, start_yaw,
                                                  goal_point, goal_yaw,
                                                  n_path_points);
                clothoids.Add(clothoid);
            }
        }
        return clothoids;

    }

    private List<Point> generate_clothoid_path(Point start_point, double start_yaw,
                           Point goal_point, double goal_yaw, int n_path_points)
    {
        var dx = goal_point.x - start_point.x;
        var dy = goal_point.y - start_point.y;
        var r = Helpers.Hypotenuse(dx, dy);

        var phi = Math.Atan2(dy, dx);
        var phi1 = normalize_angle(start_yaw - phi);
        var phi2 = normalize_angle(goal_yaw - phi);
        var delta = phi2 - phi1;

        var L = 0.0;
        var curvature = 0.0;
        var curvature_rate = 0.0;

        try
        {
            // Step1: Solve g function
            var A = solve_g_for_root(phi1, phi2, delta);

            // Step2: Calculate path parameters
            L = compute_path_length(r, phi1, delta, A);
            curvature = compute_curvature(delta, A, L);
            curvature_rate = compute_curvature_rate(A, L);
        }
        catch (Exception e)
        {
            Debug.Log("Failed to generate clothoid points: {e}");
            return null;
        }

        // Step3: Construct a path with Fresnel integral
        var points = new List<Point>();
        var s_range = Vector.Interval(0, L, n_path_points).ToList();
        foreach (var s in s_range)
        {
            try
            {
                var x = start_point.x + s * X(curvature_rate * Math.Pow(s, 2), curvature * s, start_yaw);
                var y = start_point.y + s * Y(curvature_rate * Math.Pow(s, 2), curvature * s, start_yaw);
                points.Add(new Point((float)x, (float)y));
            }
            catch (Exception e)
            {
                Debug.Log("Skipping failed clothoid point: {e}");
            }
        }

        return points;
    }

    private double X(double a, double b, double c)
    {
        var lowerLimit = 0;
        var upperLimit = 1;
        return Helpers.GaussLegendreRuleIntegral(t => Math.Cos((a / 2) * Math.Pow(t, 2) + b * t + c), lowerLimit, upperLimit);
    }

    private double Y(double a, double b, double c)
    {
        var lowerLimit = 0;
        var upperLimit = 1;
        return Helpers.GaussLegendreRuleIntegral(t => Math.Sin((a / 2) * Math.Pow(t, 2) + b * t + c), lowerLimit, upperLimit);
    }

    private double solve_g_for_root(double theta1, double theta2, double delta)
    {
        var initial_guess = 3 * (theta1 + theta2);
        return Helpers.FSolve(A => Y(2 * A, delta - A, theta1), initial_guess);
    }
    
    private double compute_path_length(double r, double theta1, double delta, double A)
    {
        return r / X(2 * A, delta - A, theta1);
    }

    private double compute_curvature(double delta, double A, double L)
    {
        return (delta - A) / L;
    }

    private double compute_curvature_rate(double A, double L)
    {
        return 2 * A / (Math.Pow(L,2));
    }

    private double normalize_angle(double angle_rad)
    {
        return (angle_rad + Mathf.PI) % (2 * Mathf.PI) - Mathf.PI;
    }

    private (double,double,double,double) get_axes_limits(List<List<Point>> clothoids)
    {
        var x_vals = new List<double>(); 
        foreach(var clothoid in clothoids)
        {
            foreach(var p in clothoid)
            {
                x_vals.Add(p.x);
            }
        }

        var y_vals = new List<double>();
        foreach (var clothoid in clothoids)
        {
            foreach (var p in clothoid)
            {
                y_vals.Add(p.y);
            }
        }

        var x_min = x_vals.Min();

        var x_max = x_vals.Max();

        var y_min = y_vals.Min();

        var y_max = y_vals.Max();


        var x_offset = 0.1 * (x_max - x_min);

        var y_offset = 0.1 * (y_max - y_min);


        x_min = x_min - x_offset;

        x_max = x_max + x_offset;

        y_min = y_min - y_offset;

        y_max = y_max + y_offset;

        return (x_min, x_max, y_min, y_max);
    }
    
}
