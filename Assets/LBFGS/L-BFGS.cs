using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
// Accord Math Library
// The Accord.NET Framework
// http://accord-framework.net
//
// Copyright © César Souza, 2009-2014
// cesarsouza at gmail.com
//
// Copyright © Jorge Nocedal, 1990
// http://users.eecs.northwestern.edu/~nocedal/
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
using System;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

/// <summary>
///   Limited-memory Broyden–Fletcher–Goldfarb–Shanno (L-BFGS) optimization method.
/// </summary>
/// 
/// <remarks>
/// <para>
///   The L-BFGS algorithm is a member of the broad family of quasi-Newton optimization
///   methods. L-BFGS stands for 'Limited memory BFGS'. Indeed, L-BFGS uses a limited
///   memory variation of the Broyden–Fletcher–Goldfarb–Shanno (BFGS) update to approximate
///   the inverse Hessian matrix (denoted by Hk). Unlike the original BFGS method which
///   stores a dense  approximation, L-BFGS stores only a few vectors that represent the
///   approximation implicitly. Due to its moderate memory requirement, L-BFGS method is
///   particularly well suited for optimization problems with a large number of variables.</para>
/// <para>
///   L-BFGS never explicitly forms or stores Hk. Instead, it maintains a history of the past
///   <c>m</c> updates of the position <c>x</c> and gradient <c>g</c>, where generally the history
///   <c>m</c>can be short, often less than 10. These updates are used to implicitly do operations
///   requiring the Hk-vector product.</para>
///   
/// <para>
///   The framework implementation of this method is based on the original FORTRAN source code
///   by Jorge Nocedal (see references below). The original FORTRAN source code of L-BFGS (for
///   unconstrained problems) is available at http://www.netlib.org/opt/lbfgs_um.shar and had
///   been made available under the public domain. </para>
/// 
/// <para>
///   References:
///   <list type="bullet">
///     <item><description><a href="http://www.netlib.org/opt/lbfgs_um.shar">
///        Jorge Nocedal. Limited memory BFGS method for large scale optimization (Fortran source code). 1990.
///        Available in http://www.netlib.org/opt/lbfgs_um.shar </a></description></item>
///     <item><description>
///        Jorge Nocedal. Updating Quasi-Newton Matrices with Limited Storage. <i>Mathematics of Computation</i>,
///        Vol. 35, No. 151, pp. 773--782, 1980.</description></item>
///     <item><description>
///        Dong C. Liu, Jorge Nocedal. On the limited memory BFGS method for large scale optimization.</description></item>
///    </list></para>
/// </remarks>
/// 
/// <example>
/// <para>
///   The following example shows the basic usage of the L-BFGS solver
///   to find the minimum of a function specifying its function and
///   gradient. </para>
///   
/// <code>
/// // Suppose we would like to find the minimum of the function
/// // 
/// //   f(x,y)  =  -exp{-(x-1)²} - exp{-(y-2)²/2}
/// //
/// 
/// // First we need write down the function either as a named
/// // method, an anonymous method or as a lambda function:
/// 
/// Func&lt;double[], double> f = (x) =>
///     -math.Exp(-math.Pow(x[0] - 1, 2)) - math.Exp(-0.5 * math.Pow(x[1] - 2, 2));
/// 
/// // Now, we need to write its gradient, which is just the
/// // vector of first partial derivatives del_f / del_x, as:
/// //
/// //   g(x,y)  =  { del f / del x, del f / del y }
/// // 
/// 
/// Func&lt;double[], double[]> g = (x) => new double[] 
/// {
///     // df/dx = {-2 e^(-    (x-1)^2) (x-1)}
///     2 * math.Exp(-math.Pow(x[0] - 1, 2)) * (x[0] - 1),
/// 
///     // df/dy = {-  e^(-1/2 (y-2)^2) (y-2)}
///     math.Exp(-0.5 * math.Pow(x[1] - 2, 2)) * (x[1] - 2)
/// };
/// 
/// // Finally, we can create the L-BFGS solver, passing the functions as arguments
/// var lbfgs = new BroydenFletcherGoldfarbShanno(numberOfVariables: 2, function: f, gradient: g);
/// 
/// // And then minimize the function:
/// double minValue = lbfgs.minimize();
/// double[] solution = lbfgs.Solution;
/// 
/// // The resultant minimum value should be -2, and the solution
/// // vector should be { 1.0, 2.0 }. The answer can be checked on
/// // Wolfram Alpha by clicking the following the link:
/// 
/// // http://www.wolframalpha.com/input/?i=maximize+%28exp%28-%28x-1%29%C2%B2%29+%2B+exp%28-%28y-2%29%C2%B2%2F2%29%29
/// 
/// </code>
/// </example>
/// 
public unsafe class BroydenFletcherGoldfarbShanno : IGradientOptimizationMethod, IDisposable
{
    // those values need not be modified
    private const float ftol = 0.0001f;
    private const float xtol = 1e-16f; // machine precision
    private const float stpmin = 1e-20f;
    private const float stpmax = 1e20f;

    // Line search parameters
    private float gtol = 1e-5f;
    private int maxfev = 40;

    private float tolerance = 1;
    private int iterations;
    private int evaluations;

    private int numberOfVariables;
    private int corrections = 5;

    private NativeArray<float> currentSolution; // current solution x
    private float fitness;   // value at current solution f(x)
    NativeArray<float> gradient;         // gradient at current solution

    private NativeArray<float> lowerBound;
    private NativeArray<float> upperBound;

    private NativeArray<float> work;


    #region Properties

    /// <summary>
    ///   Occurs when progress is made during the optimization.
    /// </summary>
    /// 
    public event EventHandler<OptimizationProgressEventArgs> Progress;

    /// <summary>
    ///   Gets or sets the function to be optimized.
    /// </summary>
    /// 
    /// <value>The function to be optimized.</value>
    /// 
    public Func<NativeArray<float>, float> Function { get; set; }

    /// <summary>
    ///   Gets or sets a function returning the gradient
    ///   vector of the function to be optimized for a
    ///   given value of its free parameters.
    /// </summary>
    /// 
    /// <value>The gradient function.</value>
    /// 
    public Func<NativeArray<float>, NativeArray<float>> Gradient { get; set; }

    /// <summary>
    ///   Gets or sets a function returning the Hessian
    ///   diagonals to be used during optimization.
    /// </summary>
    /// 
    /// <value>A function for the Hessian diagonal.</value>
    /// 
    public Func<NativeArray<float>> Diagonal { get; set; }

    /// <summary>
    ///   Gets the number of variables (free parameters)
    ///   in the optimization problem.
    /// </summary>
    /// 
    /// <value>The number of parameters.</value>
    /// 
    public int Parameters
    {
        get { return numberOfVariables; }
    }

    /// <summary>
    ///   Gets the number of iterations performed in the last
    ///   call to <see cref="Minimize()"/>.
    /// </summary>
    /// 
    /// <value>
    ///   The number of iterations performed
    ///   in the previous optimization.</value>
    ///   
    public int Iterations
    {
        get { return iterations; }
    }

    /// <summary>
    ///   Gets or sets the maximum number of iterations
    ///   to be performed during optimization. Default
    ///   is 0 (iterate until convergence).
    /// </summary>
    /// 
    public int MaxIterations
    {
        get;
        set;
    }

    /// <summary>
    ///   Gets the number of function evaluations performed
    ///   in the last call to <see cref="Minimize()"/>.
    /// </summary>
    /// 
    /// <value>
    ///   The number of evaluations performed
    ///   in the previous optimization.</value>
    ///   
    public int Evaluations
    {
        get { return evaluations; }
    }

    /// <summary>
    ///   Gets or sets the number of corrections used in the L-BFGS
    ///   update. Recommended values are between 3 and 7. Default is 5.
    /// </summary>
    /// 
    public int Corrections
    {
        get { return corrections; }
        set
        {
            if (value <= 0)
                throw new ArgumentOutOfRangeException("value");

            if (corrections != value)
            {
                corrections = value;
                createWorkVector();
            }
        }
    }

    /// <summary>
    ///   Gets or sets the upper bounds of the interval
    ///   in which the solution must be found.
    /// </summary>
    /// 
    public NativeArray<float> UpperBounds
    {
        get { return upperBound; }
        set { upperBound = value; }
    }

    /// <summary>
    ///   Gets or sets the lower bounds of the interval
    ///   in which the solution must be found.
    /// </summary>
    /// 
    public NativeArray<float> LowerBounds
    {
        get { return lowerBound; }
        set { lowerBound = value; }
    }

    /// <summary>
    ///   Gets or sets the accuracy with which the solution
    ///   is to be found. Default value is 1e-10.
    /// </summary>
    /// 
    /// <remarks>
    ///   The optimization routine terminates when ||G|| &lt; EPS max(1,||X||),
    ///   where ||.|| denotes the Euclidean norm and EPS is the value for this
    ///   property.
    /// </remarks>
    /// 
    public float Tolerance
    {
        get { return tolerance; }
        set { tolerance = value; }
    }

    /// <summary>
    ///   Gets or sets a tolerance value controlling the accuracy of the
    ///   line search routine. If the function and gradient evaluations are
    ///   inexpensive with respect to the cost of the iteration (which is
    ///   sometimes the case when solving very large problems) it may be
    ///   advantageous to set this to a small value. A typical small value
    ///   is 0.1. This value should be greater than 1e-4. Default is 0.9.
    /// </summary>
    /// 
    public float Precision
    {
        get { return gtol; }
        set
        {
            if (value <= 1e-4)
                throw new ArgumentOutOfRangeException("value");

            gtol = value;
        }
    }

    /// <summary>
    ///   Gets the solution found, the values of the
    ///   parameters which optimizes the function.
    /// </summary>
    /// 
    public NativeArray<float> Solution
    {
        get { return currentSolution; }
    }

    /// <summary>
    ///   Gets the output of the function at the current solution.
    /// </summary>
    /// 
    public float Value
    {
        get { return fitness; }
    }

    #endregion

    ~BroydenFletcherGoldfarbShanno()
    {
        Dispose();
    }

    #region Constructors

    /// <summary>
    ///   Creates a new instance of the L-BFGS optimization algorithm.
    /// </summary>
    /// 
    /// <param name="numberOfVariables">The number of free parameters in the optimization problem.</param>
    /// 
    public BroydenFletcherGoldfarbShanno(int numberOfVariables)
    {
        if (numberOfVariables <= 0)
            throw new ArgumentOutOfRangeException("numberOfVariables");

        this.numberOfVariables = numberOfVariables;

        this.createWorkVector();

        this.upperBound = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        this.lowerBound = new NativeArray<float>(numberOfVariables, Allocator.Persistent);

        for (int i = 0; i < upperBound.Length; i++)
            lowerBound[i] = float.NegativeInfinity;

        for (int i = 0; i < upperBound.Length; i++)
            upperBound[i] = float.PositiveInfinity;

        currentSolution = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        for (int i = 0; i < currentSolution.Length; i++)
            currentSolution[i] = Unity.Mathematics.Random.CreateFromIndex(0).NextFloat() * 2.0f - 1.0f;
    }

    /// <summary>
    ///   Creates a new instance of the L-BFGS optimization algorithm.
    /// </summary>
    /// 
    /// <param name="numberOfVariables">The number of free parameters in the function to be optimized.</param>
    /// <param name="function">The function to be optimized.</param>
    /// <param name="gradient">The gradient of the function.</param>
    /// 
    public BroydenFletcherGoldfarbShanno(int numberOfVariables, Func<NativeArray<float>, float> function, Func<NativeArray<float>, NativeArray<float>> gradient)
        : this(numberOfVariables)
    {
        if (function == null)
            throw new ArgumentNullException("function");

        if (gradient == null)
            throw new ArgumentNullException("gradient");

        this.Function = function;
        this.Gradient = gradient;

    }

    /// <summary>
    ///   Creates a new instance of the L-BFGS optimization algorithm.
    /// </summary>
    /// 
    /// <param name="numberOfVariables">The number of free parameters in the function to be optimized.</param>
    /// <param name="function">The function to be optimized.</param>
    /// <param name="gradient">The gradient of the function.</param>
    /// <param name="diagonal">The diagonal of the Hessian.</param>
    /// 
    public BroydenFletcherGoldfarbShanno(int numberOfVariables, Func<NativeArray<float>, float> function, Func<NativeArray<float>, NativeArray<float>> gradient, Func<NativeArray<float>> diagonal)
        : this(numberOfVariables, function, gradient)
    {
        this.Diagonal = diagonal;
    }
    #endregion


    /// <summary>
    ///   Minimizes the defined function. 
    /// </summary>
    /// 
    /// <returns>The minimum value found at the <see cref="Solution"/>.</returns>
    /// 
    public float Minimize()
    {
        return minimize();
    }

    /// <summary>
    ///   Minimizes the defined function. 
    /// </summary>
    /// 
    /// <param name="values">The initial guess values for the parameters. Default is the zero vector.</param>
    /// 
    /// <returns>The minimum value found at the <see cref="Solution"/>.</returns>
    /// 
    public float Minimize(NativeArray<float3> values)
    {
        if (values == null)
            throw new ArgumentNullException("values");

        if (values.Length*3 != numberOfVariables)
            throw new DimensionMismatchException("values");

        UnsafeUtility.MemCpy(currentSolution.GetUnsafePtr(), values.GetUnsafePtr(), currentSolution.Length * UnsafeUtility.SizeOf<float>());
        return minimize();
    }
    public float Minimize(NativeArray<float> values)
    {
        if (values == null)
            throw new ArgumentNullException("values");

        if (values.Length != numberOfVariables)
            throw new DimensionMismatchException("values");

        // Copy initial guess for solution
        UnsafeUtility.MemCpy(currentSolution.GetUnsafePtr(), values.GetUnsafePtr(), currentSolution.Length * UnsafeUtility.SizeOf<float>());

        return minimize();
    }

    private unsafe float minimize()
    {
        if (Function == null) throw new InvalidOperationException(
            "The function to be minimized has not been defined.");

        if (Gradient == null) throw new InvalidOperationException(
            "The gradient function has not been defined.");


        // Initialization
        int m_variableCount = numberOfVariables, m_corrections = corrections;

        // Make initial evaluation
        fitness = GetFunction(currentSolution);
        gradient = GetGradient(currentSolution);

        this.iterations = 0;
        this.evaluations = 1;


        // Obtain initial Hessian
        NativeArray<float> diagonal = default(NativeArray<float>);

        if (Diagonal != null)
        {
            diagonal = getDiagonal();
        }
        else
        {
            diagonal = new NativeArray<float>(m_variableCount, Allocator.Persistent);
            for (int i = 0; i < diagonal.Length; i++)
                diagonal[i] = 1.0f;
        }


        float* w = (float*)work.GetUnsafePtr();
        {
            // The first N locations of the work vector are used to
            //  store the gradient and other temporary information.

            float* rho = &w[m_variableCount];                   // Stores the scalars rho.
            float* alpha = &w[m_variableCount + m_corrections];             // Stores the alphas in computation of H*g.
            float* steps = &w[m_variableCount + 2 * m_corrections];         // Stores the last M search steps.
            float* delta = &w[m_variableCount + 2 * m_corrections + m_variableCount * m_corrections]; // Stores the last M gradient differences.


            // Initialize work vector
            for (int i = 0; i < gradient.Length; i++)
                steps[i] = -gradient[i] * diagonal[i];


            // Initialize statistics
            float gnorm = Norm.Euclidean(gradient);
            float xnorm = Norm.Euclidean(currentSolution);
            float stp = 1.0f / gnorm;
            float stp1 = stp;

            // Initialize loop
            int nfev, point = 0;
            int npt = 0, cp = 0;
            bool finish = false;

            // Make initial progress report with initialization parameters
            if (Progress != null) Progress(this, new OptimizationProgressEventArgs
                (iterations, evaluations, gradient.ToArray(), gnorm, currentSolution.ToArray(), xnorm, fitness, stp, finish));
            float ys = 0;

            // Start main
            while (!finish)
            {
                iterations++;
                float bound = iterations - 1;

                if (iterations != 1)
                {
                    if (iterations > m_corrections)
                        bound = m_corrections;




                    // Compute the diagonal of the Hessian
                    // or use an approximation by the user.
                    if (ys == 0)
                    {
                        break;
                    }
                    if (Diagonal != null)
                    {
                        diagonal = getDiagonal();
                    }
                    else
                    {
                        float yy = 0;
                        for (int i = 0; i < m_variableCount; i++)
                            yy += delta[npt + i] * delta[npt + i];
                        float d = ys / yy;

                        for (int i = 0; i < m_variableCount; i++)
                            diagonal[i] = d;
                    }


                    // Compute -H*g using the formula given in:
                    //   Nocedal, J. 1980, "Updating quasi-Newton matrices with limited storage",
                    //   Mathematics of Computation, Vol.24, No.151, pp. 773-782.

                    cp = (point == 0) ? m_corrections : point;

                    rho[cp - 1] = 1.0f / ys;
                    for (int i = 0; i < m_variableCount; i++)
                        w[i] = -gradient[i];

                    cp = point;
                    for (int i = 1; i <= bound; i += 1)
                    {
                        if (--cp == -1) cp = m_corrections - 1;

                        float sq = 0;
                        for (int j = 0; j < m_variableCount; j++)
                            sq += steps[cp * m_variableCount + j] * w[j];

                        float beta = alpha[cp] = rho[cp] * sq;
                        for (int j = 0; j < m_variableCount; j++)
                            w[j] -= beta * delta[cp * m_variableCount + j];
                    }

                    for (int i = 0; i < diagonal.Length; i++)
                        w[i] *= diagonal[i];

                    for (int i = 1; i <= bound; i += 1)
                    {
                        float yr = 0;
                        for (int j = 0; j < m_variableCount; j++)
                            yr += delta[cp * m_variableCount + j] * w[j];

                        float beta = alpha[cp] - rho[cp] * yr;
                        for (int j = 0; j < m_variableCount; j++)
                            w[j] += beta * steps[cp * m_variableCount + j];

                        if (++cp == m_corrections) cp = 0;
                    }

                    npt = point * m_variableCount;

                    // Store the search direction
                    for (int i = 0; i < m_variableCount; i++)
                        steps[npt + i] = w[i];

                    stp = 1;
                }

                // Save original gradient
                for (int i = 0; i < gradient.Length; i++)
                    w[i] = gradient[i];


                // Obtain the one-dimensional minimizer of f by computing a line search
                bool isContinue = mcsrch(currentSolution, ref fitness, ref gradient, &steps[point * m_variableCount], ref stp, out nfev, diagonal);

                // Register evaluations
                evaluations += nfev;

                // Compute the new step and
                // new gradient differences
                for (int i = 0; i < gradient.Length; i++)
                {
                    steps[npt + i] *= stp;
                    delta[npt + i] = gradient[i] - w[i];
                }

                if (++point == m_corrections) point = 0;


                // Check for termination
                gnorm = Norm.Euclidean(gradient);
                xnorm = Norm.Euclidean(currentSolution);
                xnorm = math.max(1f, xnorm);




                if (!isContinue && gnorm / xnorm <= tolerance)
                    finish = true;

                /*                for (int i = 0; i < n; i++)
                                    ys += delta[npt + i] * steps[npt + i];

                                if (ys==0)
                                {
                                    finish = true;
                                }*/

                if (Progress != null) Progress(this, new OptimizationProgressEventArgs
                    (iterations, evaluations, gradient.ToArray(), gnorm, currentSolution.ToArray(), xnorm, fitness, stp, finish));
            }
        }

        return fitness; // return the minimum value found (at solution x)
    }


    #region Line Search (mcsrch)

    /// <summary>
    ///   Finds a step which satisfies a sufficient decrease and curvature condition.
    /// </summary>
    /// 
    private unsafe bool mcsrch(NativeArray<float> x, ref float f, ref NativeArray<float> g, float* s,
        ref float stp, out int nfev, NativeArray<float> wa)
    {
        int n = numberOfVariables;
        float ftest1 = 0;
        int infoc = 1;

        nfev = 0;

        if (stp <= 0)
            throw new LineSearchFailedException(1, "Invalid step size.");

        // Compute the initial gradient in the search direction
        // and check that s is a descent direction.

        float dginit = 0;

        for (int j = 0; j < g.Length; j++)
            dginit = dginit + g[j] * s[j];

        if (dginit >= 0)
        {
            // throw new LineSearchFailedException(0, "The search direction is not a descent direction.");
        }



        bool brackt = false;
        bool stage1 = true;

        float finit = f;
        float dgtest = ftol * dginit;
        float width = stpmax - stpmin;
        float width1 = width / 0.5f;

        for (int j = 0; j < x.Length; j++)
            wa[j] = x[j];

        // The variables stx, fx, dgx contain the values of the
        // step, function, and directional derivative at the best
        // step.

        float stx = 0;
        float fx = finit;
        float dgx = dginit;

        // The variables sty, fy, dgy contain the value of the
        // step, function, and derivative at the other endpoint
        // of the interval of uncertainty.

        float sty = 0;
        float fy = finit;
        float dgy = dginit;

        // The variables stp, f, dg contain the values of the step,
        // function, and derivative at the current step.

        float dg = 0;


        while (true)
        {
            // Set the minimum and maximum steps to correspond
            // to the present interval of uncertainty.

            float stmin, stmax;

            if (brackt)
            {
                stmin = math.min(stx, sty);
                stmax = math.max(stx, sty);
            }
            else
            {
                stmin = stx;
                stmax = stp + 4.0f * (stp - stx);
            }

            // Force the step to be within the bounds stpmax and stpmin.

            stp = math.max(stp, stpmin);
            stp = math.min(stp, stpmax);

            // If an unusual termination is to occur then let
            // stp be the lowest point obtained so far.

            if ((brackt && (stp <= stmin || stp >= stmax)) ||
                (brackt && stmax - stmin <= xtol * stmax) ||
                (nfev >= maxfev - 1) || (infoc == 0))
                stp = stx;

            // Evaluate the function and gradient at stp
            // and compute the directional derivative.
            // We return to main program to obtain F and G.

            for (int j = 0; j < x.Length; j++)
            {
                x[j] = wa[j] + stp * s[j];

                if (x[j] > upperBound[j])
                    x[j] = upperBound[j];
                else if (x[j] < lowerBound[j])
                    x[j] = lowerBound[j];
            }


            // Reevaluate function and gradient
            f = GetFunction(x);
            g = GetGradient(x);

            nfev++;
            dg = 0;

            for (int j = 0; j < g.Length; j++)
                dg = dg + g[j] * s[j];

            ftest1 = finit + stp * dgtest;

            // Test for convergence.

            if (nfev >= maxfev)
            {
                return false;
                //throw new LineSearchFailedException(3, "Maximum number of function evaluations has been reached.");
            }


            if ((brackt && (stp <= stmin || stp >= stmax)) || infoc == 0)
            {
                return false;
                /*                throw new LineSearchFailedException(6, "Rounding errors prevent further progress." +
                                                                       "There may not be a step which satisfies the sufficient decrease and curvature conditions. Tolerances may be too small. \n" +
                                                                       "stp: " + stp.ToString() + ", brackt: " + brackt.ToString() + ", infoc: " + infoc.ToString() + ", stmin: " + stmin.ToString() + ", stmax: " + stmax.ToString());*/
            }

            if (stp == stpmax && f <= ftest1 && dg <= dgtest)
            {
                return false;
                throw new LineSearchFailedException(5, "The step size has reached the upper bound.");
            }


            if (stp == stpmin && (f > ftest1 || dg >= dgtest))
            {
                return false;
                throw new LineSearchFailedException(4, "The step size has reached the lower bound.");
            }


            if (brackt && stmax - stmin <= xtol * stmax)
            {
                return false;
                throw new LineSearchFailedException(2, "Relative width of the interval of uncertainty is at machine precision.");
            }


            if (f <= ftest1 && math.abs(dg) <= gtol * (-dginit))
                return true;

            // Not converged yet. Continuing with the search.

            // In the first stage we seek a step for which the modified
            // function has a nonpositive value and nonnegative derivative.

            if (stage1 && f <= ftest1 && dg >= math.min(ftol, gtol) * dginit)
                stage1 = false;

            // A modified function is used to predict the step only if we
            // have not obtained a step for which the modified function has
            // a nonpositive function value and nonnegative derivative, and
            // if a lower function value has been obtained but the decrease
            // is not sufficient.

            if (stage1 && f <= fx && f > ftest1)
            {
                // Define the modified function and derivative values.

                float fm = f - stp * dgtest;
                float fxm = fx - stx * dgtest;
                float fym = fy - sty * dgtest;

                float dgm = dg - dgtest;
                float dgxm = dgx - dgtest;
                float dgym = dgy - dgtest;

                // Call cstep to update the interval of uncertainty
                // and to compute the new step.

                SearchStep(ref stx, ref fxm, ref dgxm,
                    ref sty, ref fym, ref dgym, ref stp,
                    fm, dgm, ref brackt, out infoc);

                // Reset the function and gradient values for f.
                fx = fxm + stx * dgtest;
                fy = fym + sty * dgtest;
                dgx = dgxm + dgtest;
                dgy = dgym + dgtest;
            }
            else
            {
                // Call mcstep to update the interval of uncertainty
                // and to compute the new step.

                SearchStep(ref stx, ref fx, ref dgx,
                    ref sty, ref fy, ref dgy, ref stp,
                    f, dg, ref brackt, out infoc);
            }

            // Force a sufficient decrease in the size of the
            // interval of uncertainty.

            if (brackt)
            {
                if (math.abs(sty - stx) >= 0.66 * width1)
                    stp = stx + 0.5f * (sty - stx);

                width1 = width;
                width = math.abs(sty - stx);
            }

        }
    }

    // TODO: Move to separate classes
    internal static void SearchStep(ref float stx, ref float fx, ref float dx,
                                ref float sty, ref float fy, ref float dy,
                                ref float stp, float fp, float dp,
                                ref bool brackt, out int info)
    {
        bool bound;
        float stpc, stpf, stpq;

        info = 0;

        if ((brackt && (stp <= math.min(stx, sty) || stp >= math.max(stx, sty))) ||
            (dx * (stp - stx) >= 0.0) || (stpmax < stpmin)) return;

        // Determine if the derivatives have opposite sign.
        float sgnd = dp * (dx / math.abs(dx));

        if (fp > fx)
        {
            // First case. A higher function value.
            // The minimum is bracketed. If the cubic step is closer
            // to stx than the quadratic step, the cubic step is taken,
            // else the average of the cubic and quadratic steps is taken.

            info = 1;
            bound = true;
            float theta = 3.0f * (fx - fp) / (stp - stx) + dx + dp;
            float s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));
            float gamma = s * math.sqrt((theta / s) * (theta / s) - (dx / s) * (dp / s));

            if (stp < stx) gamma = -gamma;

            float p = gamma - dx + theta;
            float q = gamma - dx + gamma + dp;
            float r = p / q;
            stpc = stx + r * (stp - stx);
            stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2) * (stp - stx);

            if (math.abs(stpc - stx) < math.abs(stpq - stx))
                stpf = stpc;
            else
                stpf = stpc + (stpq - stpc) / 2.0f;

            brackt = true;
        }
        else if (sgnd < 0.0)
        {
            // Second case. A lower function value and derivatives of
            // opposite sign. The minimum is bracketed. If the cubic
            // step is closer to stx than the quadratic (secant) step,
            // the cubic step is taken, else the quadratic step is taken.

            info = 2;
            bound = false;
            float theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
            float s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));
            float gamma = s * math.sqrt((theta / s) * (theta / s) - (dx / s) * (dp / s));

            if (stp > stx) gamma = -gamma;

            float p = (gamma - dp) + theta;
            float q = ((gamma - dp) + gamma) + dx;
            float r = p / q;
            stpc = stp + r * (stx - stp);
            stpq = stp + (dp / (dp - dx)) * (stx - stp);

            if (math.abs(stpc - stp) > math.abs(stpq - stp))
                stpf = stpc;
            else stpf = stpq;

            brackt = true;
        }
        else if (math.abs(dp) < math.abs(dx))
        {
            // Third case. A lower function value, derivatives of the
            // same sign, and the magnitude of the derivative decreases.
            // The cubic step is only used if the cubic tends to infinity
            // in the direction of the step or if the minimum of the cubic
            // is beyond stp. Otherwise the cubic step is defined to be
            // either stpmin or stpmax. The quadratic (secant) step is also
            // computed and if the minimum is bracketed then the step
            // closest to stx is taken, else the step farthest away is taken.

            info = 3;
            bound = true;
            float theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
            float s = math.max(math.abs(theta), math.max(math.abs(dx), math.abs(dp)));
            float gamma = s * math.sqrt(math.max(0, (theta / s) * (theta / s) - (dx / s) * (dp / s)));

            if (stp > stx) gamma = -gamma;

            float p = (gamma - dp) + theta;
            float q = (gamma + (dx - dp)) + gamma;
            float r = p / q;

            if (r < 0.0 && gamma != 0.0)
                stpc = stp + r * (stx - stp);
            else if (stp > stx)
                stpc = stpmax;
            else stpc = stpmin;

            stpq = stp + (dp / (dp - dx)) * (stx - stp);

            if (brackt)
            {
                if (math.abs(stp - stpc) < math.abs(stp - stpq))
                    stpf = stpc;
                else stpf = stpq;
            }
            else
            {
                if (math.abs(stp - stpc) > math.abs(stp - stpq))
                    stpf = stpc;
                else stpf = stpq;
            }
        }
        else
        {
            // Fourth case. A lower function value, derivatives of the
            // same sign, and the magnitude of the derivative does
            // not decrease. If the minimum is not bracketed, the step
            // is either stpmin or stpmax, else the cubic step is taken.

            info = 4;
            bound = false;

            if (brackt)
            {
                float theta = 3 * (fp - fy) / (sty - stp) + dy + dp;
                float s = math.max(math.abs(theta), math.max(math.abs(dy), math.abs(dp)));
                float gamma = s * math.sqrt((theta / s) * (theta / s) - (dy / s) * (dp / s));

                if (stp > sty) gamma = -gamma;

                float p = (gamma - dp) + theta;
                float q = ((gamma - dp) + gamma) + dy;
                float r = p / q;
                stpc = stp + r * (sty - stp);
                stpf = stpc;
            }
            else if (stp > stx)
                stpf = stpmax;
            else stpf = stpmin;
        }

        // Update the interval of uncertainty. This update does not
        // depend on the new step or the case analysis above.

        if (fp > fx)
        {
            sty = stp;
            fy = fp;
            dy = dp;
        }
        else
        {
            if (sgnd < 0.0)
            {
                sty = stx;
                fy = fx;
                dy = dx;
            }
            stx = stp;
            fx = fp;
            dx = dp;
        }

        // Compute the new step and safeguard it.
        stpf = math.min(stpmax, stpf);
        stpf = math.max(stpmin, stpf);
        stp = stpf;

        if (brackt && bound)
        {
            if (sty > stx)
                stp = math.min(stx + 0.66f * (sty - stx), stp);
            else
                stp = math.max(stx + 0.66f * (sty - stx), stp);
        }

        return;
    }


    #endregion


    private NativeArray<float> getDiagonal()
    {
        NativeArray<float> diag = Diagonal();
        if (diag.Length != numberOfVariables) throw new ArgumentException(
            "The length of the Hessian diagonal vector does not match the" +
            " number of free parameters in the optimization poblem.");
        for (int i = 0; i < diag.Length; i++)
            if (diag[i] <= 0) throw new ArgumentException(
                "One of the diagonal elements of the inverse" +
                " Hessian approximation is not strictly positive");
        return diag;
    }

    private NativeArray<float> GetGradient(NativeArray<float> args)
    {
        NativeArray<float> grad = Gradient(args);
        if (grad.Length != numberOfVariables) throw new ArgumentException(
            "The length of the gradient vector does not match the" +
            " number of free parameters in the optimization problem.");
        return grad;
    }

    private float GetFunction(NativeArray<float> args)
    {
        float func = Function(args);
        if (float.IsNaN(func) || float.IsInfinity(func))
            throw new NotFiniteNumberException(
                "The function evaluation did not return a finite number.", func);
        return func;
    }

    private void createWorkVector()
    {
        this.work = new NativeArray<float>(numberOfVariables * (2 * corrections + 1) + 2 * corrections, Allocator.Persistent);
    }

    public void Dispose()
    {
        currentSolution.Dispose();
        gradient.Dispose();
        lowerBound.Dispose();
        upperBound.Dispose();
        work.Dispose();
    }
}

