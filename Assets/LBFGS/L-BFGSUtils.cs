using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
///   Common interface for function optimization methods.
/// </summary>
/// 
/// seealso cref="BroydenFletcherGoldfarbShanno"/>
/// <seealso cref="ConjugateGradient"/>
/// <seealso cref="ResilientBackpropagation"/>
/// <seealso cref="GoldfarbIdnaniQuadraticSolver"/>
/// 
public interface IOptimizationMethod
{

    /// <summary>
    ///   Optimizes the defined function. 
    /// </summary>
    /// 
    /// <param name="values">The initial guess values for the parameters.</param>
    /// 
    double Minimize(double[] values);

    /// <summary>
    ///   Gets the solution found, the values of the parameters which
    ///   optimizes the function.
    /// </summary>
    double[] Solution { get; }

}

/// <summary>
///   Common interface for function optimization methods which depend on
///   having both an objective function and a gradient function definition
///   available.
/// </summary>

public interface IGradientOptimizationMethod : IOptimizationMethod
{
    /// <summary>
    ///   Gets or sets the function to be optimized.
    /// </summary>
    /// 
    /// <value>The function to be optimized.</value>
    /// 
    Func<double[], double> Function { get; set; }

    /// <summary>
    ///   Gets or sets a function returning the gradient
    ///   vector of the function to be optimized for a
    ///   given value of its free parameters.
    /// </summary>
    /// 
    /// <value>The gradient function.</value>
    /// 
    Func<double[], double[]> Gradient { get; set; }

    /// <summary>
    ///   Gets the number of variables (free parameters)
    ///   in the optimization problem.
    /// </summary>
    /// 
    /// <value>The number of parameters.</value>
    /// 
    int Parameters { get; }
}

/// <summary>
///   Optimization progress event arguments.
/// </summary>
public class OptimizationProgressEventArgs : EventArgs
{
    /// <summary>
    ///   Gets the current iteration of the method.
    /// </summary>
    /// 
    public int Iteration { get; private set; }

    /// <summary>
    ///   Gets the number of function evaluations performed.
    /// </summary>
    /// 
    public int Evaluations { get; private set; }

    /// <summary>
    ///   Gets the current gradient of the function being optimized.
    /// </summary>
    /// 
    public double[] Gradient { get; private set; }

    /// <summary>
    ///   Gets the norm of the current <see cref="Gradient"/>.
    /// </summary>
    /// 
    public double GradientNorm { get; private set; }

    /// <summary>
    ///   Gets the current solution parameters for the problem.
    /// </summary>
    /// 
    public double[] Solution { get; private set; }

    /// <summary>
    ///   Gets the norm of the current <see cref="Solution"/>.
    /// </summary>
    /// 
    public double SolutionNorm { get; private set; }

    /// <summary>
    ///   Gets the value of the function to be optimized
    ///   at the current proposed <see cref="Solution"/>.
    /// </summary>
    /// 
    public double Value { get; private set; }

    /// <summary>
    ///   Gets the current step size.
    /// </summary>
    /// 
    public double Step { get; private set; }


    /// <summary>
    ///   Gets or sets a value indicating whether the
    ///   optimization process is about to terminate.
    /// </summary>
    /// 
    /// <value><c>true</c> if finished; otherwise, <c>false</c>.</value>
    /// 
    public bool Finished { get; private set; }

    /// <summary>
    ///   Initializes a new instance of the <see cref="OptimizationProgressEventArgs"/> class.
    /// </summary>
    /// 
    /// <param name="iteration">The current iteration of the optimization method.</param>
    /// <param name="evaluations">The number of function evaluations performed.</param>
    /// <param name="gradient">The current gradient of the function.</param>
    /// <param name="gnorm">The norm of the current gradient</param>
    /// <param name="xnorm">The norm of the current parameter vector.</param>
    /// <param name="solution">The current solution parameters.</param>
    /// <param name="value">The value of the function evaluated at the current solution.</param>
    /// <param name="stp">The current step size.</param>
    /// <param name="finished"><c>True</c> if the method is about to terminate, <c>false</c> otherwise.</param>
    /// 
    public OptimizationProgressEventArgs(
        int iteration, int evaluations,
        double[] gradient, double gnorm,
        double[] solution, double xnorm,
        double value, double stp, bool finished)
    {
        this.Gradient = (double[])gradient.Clone();
        this.Solution = (double[])solution.Clone();
        this.Value = value;
        this.GradientNorm = gnorm;
        this.SolutionNorm = xnorm;

        this.Iteration = iteration;
        this.Evaluations = evaluations;

        this.Finished = finished;
        this.Step = stp;
    }
}

public static class Norm
{
    /// <summary>
    ///   Gets the square root of the sum of squares for all elements in a matrix.
    /// </summary>
    /// 
    public static double Frobenius(this double[,] a)
    {
        if (a == null) 
            throw new ArgumentNullException("a");

        int rows = a.GetLength(0);
        int cols = a.GetLength(1);

        double norm = 0.0;
        for (int j = 0; j < cols; j++)
        {
            for (int i = 0; i < rows; i++)
            {
                double v = a[i, j];
                norm += v * v;
            }
        }

        return System.Math.Sqrt(norm);
    }

    /// <summary>
    ///   Gets the Squared Euclidean norm for a vector.
    /// </summary>
    /// 
    public static float SquareEuclidean(this float[] a)
    {
        float sum = 0;
        for (int i = 0; i < a.Length; i++)
            sum += a[i] * a[i];
        return sum;
    }

    /// <summary>
    ///   Gets the Squared Euclidean norm for a vector.
    /// </summary>
    /// 
    public static double SquareEuclidean(this double[] a)
    {
        double sum = 0.0;
        for (int i = 0; i < a.Length; i++)
            sum += a[i] * a[i];
        return sum;
    }

    /// <summary>
    ///   Gets the Euclidean norm for a vector.
    /// </summary>
    /// 
    public static float Euclidean(this float[] a)
    {
        return (float)Math.Sqrt(SquareEuclidean(a));
    }

    /// <summary>
    ///   Gets the Euclidean norm for a vector.
    /// </summary>
    /// 
    public static double Euclidean(this double[] a)
    {
        return System.Math.Sqrt(SquareEuclidean(a));
    }

    /// <summary>
    ///   Gets the Squared Euclidean norm vector for a matrix.
    /// </summary>
    /// 
    public static double[] SquareEuclidean(this double[,] a)
    {
        return SquareEuclidean(a, 0);
    }

    /// <summary>
    ///   Gets the Squared Euclidean norm vector for a matrix.
    /// </summary>
    /// 
    public static double[] SquareEuclidean(this double[,] a, int dimension)
    {
        int rows = a.GetLength(0);
        int cols = a.GetLength(1);
        
        double[] norm;

        if (dimension == 0)
        {
            norm = new double[cols];

            for (int j = 0; j < norm.Length; j++)
            {
                double sum = 0.0;
                for (int i = 0; i < rows; i++)
                {
                    double v = a[i, j];
                    sum += v * v;
                }
                norm[j] = sum;
            }
        }
        else
        {
            norm = new double[rows];

            for (int i = 0; i < norm.Length; i++)
            {
                double sum = 0.0;
                for (int j = 0; j < cols; j++)
                {
                    double v = a[i, j];
                    sum += v * v;
                }
                norm[i] = sum;
            }
        }

        return norm;
    }

    /// <summary>
    ///   Gets the Euclidean norm for a matrix.
    /// </summary>
    /// 
    public static double[] Euclidean(this double[,] a)
    {
        return Euclidean(a, 0);
    }

    /// <summary>
    ///   Gets the Euclidean norm for a matrix.
    /// </summary>
    /// 
    public static double[] Euclidean(this double[,] a, int dimension)
    {
        double[] norm = Norm.SquareEuclidean(a, dimension);

        for (int i = 0; i < norm.Length; i++)
            norm[i] = System.Math.Sqrt(norm[i]);

        return norm;
    }

    /// <summary>
    ///   Gets the Squared Euclidean norm vector for a matrix.
    /// </summary>
    /// 
    public static float[] SquareEuclidean(this float[,] a, int dimension)
    {
        int rows = a.GetLength(0);
        int cols = a.GetLength(1);

        float[] norm;

        if (dimension == 0)
        {
            norm = new float[cols];

            for (int j = 0; j < norm.Length; j++)
            {
                float sum = 0;
                for (int i = 0; i < rows; i++)
                {
                    float v = a[i, j];
                    sum += v * v;
                }
                norm[j] = sum;
            }
        }
        else
        {
            norm = new float[rows];

            for (int i = 0; i < norm.Length; i++)
            {
                float sum = 0;
                for (int j = 0; j < cols; j++)
                {
                    float v = a[i, j];
                    sum += v * v;
                }
                norm[i] = sum;
            }
        }

        return norm;
    }

    /// <summary>
    ///   Gets the Euclidean norm for a matrix.
    /// </summary>
    /// 
    public static float[] Euclidean(this float[,] a)
    {
        return Euclidean(a, 0);
    }

    /// <summary>
    ///   Gets the Euclidean norm for a matrix.
    /// </summary>
    /// 
    public static float[] Euclidean(this float[,] a, int dimension)
    {
        float[] norm = Norm.SquareEuclidean(a, dimension);

        for (int i = 0; i < norm.Length; i++)
            norm[i] = (float)System.Math.Sqrt(norm[i]);

        return norm;
    }

}
