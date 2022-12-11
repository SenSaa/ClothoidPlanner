using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using MathNet.Numerics.Integration;
using Accord.Math.Optimization;

namespace Utils
{

    public class Helpers
    {


        // --------------------------------------------------------------------------------------

        // Geometry

        // Find hypotenuse of triangles, when given the remaining two sides.
        // List<double> as Input and Output.
        public static List<double> Hypotenuse(List<double> side1, List<double> side2)
        {
            List<double> hypot = new List<double>();
            for (int i = 0; i < side1.Count; i++)
            {
                hypot.Add(Math.Sqrt(Math.Pow(side1[i], 2) + Math.Pow(side2[i], 2)));
            }
            return hypot;
        }

        // double as Input & Output. 
        public static double Hypotenuse(double side1, double side2)
        {
            double hypot = 0;
            {
                hypot = Math.Sqrt(Math.Pow(side1, 2) + Math.Pow(side2, 2));
            }
            return hypot;
        }

        // --------------------------------------------------------------------------------------



        // --------------------------------------------------------------------------------------

        // Integration

        // Newton Cotes Trapezium Rule - Adaptive approximation with a relative error (e.g. 1e-5).
        public static double NewtonCotesTrapeziumIntegral(Func<double, double> f, double lowerLimit, double upperLimit)
        {
            double adaptive = NewtonCotesTrapeziumRule.IntegrateAdaptive(f, lowerLimit, upperLimit, 1e-5);
            return adaptive;
        }

        // Double-Exponential Transformation - Approximate using a relative error (e.g. 1e-5).
        public static double DoubleExponentialTransformationIntegral(Func<double, double> f, double lowerLimit, double upperLimit)
        {
            double integrate = DoubleExponentialTransformation.Integrate(f, lowerLimit, upperLimit, 1e-5);
            return integrate;
        }

        // Gauss-Legendre Rule - 1D integration with a specific number of sample points approximation (e.g. 6-point approximation).
        public static double GaussLegendreRuleIntegral(Func<double, double> f, double lowerLimit, double upperLimit)
        {
            double integrate1D = GaussLegendreRule.Integrate(f, lowerLimit, upperLimit, 6);
            return integrate1D;
        }

        // --------------------------------------------------------------------------------------

        // Optimisation (Non-linear)

        // Brent's root finding and minimization algorithms.
        // Find the roots of a function.
        public static double FSolve(Func<double, double> f, double bound)
        {
            BrentSearch search = new BrentSearch(f, 0, bound);
            search.FindRoot();
            return search.Solution;
        }

        // --------------------------------------------------------------------------------------

        public static string ListToString<T>(List<T> list)
        {
            string result = "";
            for (int i = 0; i < list.Count; i++)
            {
                result += list[i] + "   ";
            }
            return result;
        }

        // --------------------------------------------------------------------------------------

    }

}