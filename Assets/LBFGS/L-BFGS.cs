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
    private const float FITNESS_TOLERENCE = 1e-3f;
    private const float xTolerance = 1e-20f; // machine precision
    private const float STEP_MIN = 1e-5f;
    private const float STEP_MAX = 180f;

    // Line search parameters
    private float grandientTolerence = 1;
    private int maxInnerLoop = 40;

    private float tolerance =1f;
    private int iterations;
    private int evaluations;

    private int numberOfVariables;
    private int corrections = 5;

    private NativeArray<float> currentSolution; // current solution x
   private float fitness;   // value at current solution f(x)
    NativeArray<float> gradient;         // gradient at current solution

    private NativeArray<float> work;
    private float gnorm;
    private float xnorm;
    private float stp;
    private float stp1;
    private int nfev;
    private int point;
    private int nowPoint;
    private int correctionsTemp;
    private bool isfinish;
    private float fitnesstest1;
    private int funcState;
    private bool isInBracket;
    private bool stage1;
    private float gradientInitial;

    private NativeArray<float> diagonal;
    private NativeArray<float> gradientStore;
    private NativeArray<float> rho;
    private NativeArray<float> alpha;
    private NativeArray<float> steps;
    private NativeArray<float> delta;


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

/*    /// <summary>
    ///   Gets or sets a function returning the Hessian
    ///   diagonals to be used during optimization.
    /// </summary>
    /// 
    /// <value>A function for the Hessian diagonal.</value>
    /// 
    public Func<NativeArray<float>> Diagonal { get; set; }*/

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
        get { return grandientTolerence; }
        set
        {
            if (value <= 1e-4)
                throw new ArgumentOutOfRangeException("value");

            grandientTolerence = value;
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
/*    public BroydenFletcherGoldfarbShanno(int numberOfVariables, Func<NativeArray<float>, float> function, Func<NativeArray<float>, NativeArray<float>> gradient, Func<NativeArray<float>> diagonal)
        : this(numberOfVariables, function, gradient)
    {
        this.Diagonal = diagonal;
    }*/
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

        // Make initial evaluation
        fitness = GetFunction(currentSolution);
        gradient = GetGradient(currentSolution);

        this.iterations = 0;
        this.evaluations = 1;


        // Obtain initial Hessian
        NativeArray<float> diagonal = default(NativeArray<float>);

        diagonal = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        for (int i = 0; i < diagonal.Length; i++)
        {
            diagonal[i] = 1.0f;
        }


            // Initialize work vector
            for (int i = 0; i < gradient.Length; i++)
            {
                steps[i] = -gradient[i] * diagonal[i];
            }

            // Initialize statistics
             gnorm = Norm.Euclidean(gradient);
             xnorm = Norm.Euclidean(currentSolution);
             stp = 1.0f / gnorm;
             stp1 = stp;

            // Initialize loop
            nfev = 0;
            point = 0;
            nowPoint = 0;
            correctionsTemp = 0;
            isfinish = false;

/*            // Make initial progress report with initialization parameters
            if (Progress != null) Progress(this, new OptimizationProgressEventArgs
                (iterations, evaluations, gradient.ToArray(), gnorm, currentSolution.ToArray(), xnorm, fitness, stp, finish));//OYM：输出初始状态*/
            
            // Start main
            while (!isfinish)
            {
                iterations++;
                float bound = iterations - 1;

                if (iterations != 1)
                {
                    if (iterations > corrections)
                        bound = corrections;

                    float sumY = 0;
                    for (int i = 0; i < numberOfVariables; i++)
                    {
                        sumY += delta[nowPoint + i] * steps[nowPoint + i];
                    }



                    // Compute the diagonal of the Hessian
                    // or use an approximation by the user.
                    if (sumY == 0)
                    {
                        break;
                    }
                    float sqrY = 0;
                    for (int i = 0; i < numberOfVariables; i++)
                    {
                        sqrY += delta[nowPoint + i] * delta[nowPoint + i];
                    }

                    float diagonalValue = (float)(sumY / sqrY);

                    for (int i = 0; i < numberOfVariables; i++)
                    {
                        diagonal[i] = diagonalValue;
                    }

                    // Compute -H*g using the formula given in:
                    //   Nocedal, J. 1980, "Updating quasi-Newton matrices with limited storage",
                    //   Mathematics of Computation, Vol.24, No.151, pp. 773-782.

                    correctionsTemp = (point == 0) ? corrections : point;

                    rho[correctionsTemp - 1] = 1.0f /math.max((float)sumY,math.EPSILON);
                    for (int i = 0; i < numberOfVariables; i++)
                    {
                        gradientStore[i] = -gradient[i];
                    }

                    correctionsTemp = point;
                    for (int i = 1; i <= bound; i += 1)
                    {
                        if (--correctionsTemp == -1) correctionsTemp = corrections - 1;

                        float sq = 0;
                        for (int j = 0; j < numberOfVariables; j++)
                            sq += steps[correctionsTemp * numberOfVariables + j] * gradientStore[j];

                        float beta = alpha[correctionsTemp] = rho[correctionsTemp] * sq;
                        for (int j = 0; j < numberOfVariables; j++)
                            gradientStore[j] -= beta * delta[correctionsTemp * numberOfVariables + j];
                    }

                    for (int i = 0; i < diagonal.Length; i++)
                    {
                        gradientStore[i] *= diagonal[i];
                    }


                    for (int i = 1; i <= bound; i += 1)
                    {
                        float yr = 0;
                        for (int j = 0; j < numberOfVariables; j++)
                            yr += delta[correctionsTemp * numberOfVariables + j] * gradientStore[j];

                        float beta = alpha[correctionsTemp] - rho[correctionsTemp] * yr;
                        for (int j = 0; j < numberOfVariables; j++)
                            gradientStore[j] += beta * steps[correctionsTemp * numberOfVariables + j];

                        if (++correctionsTemp == corrections) correctionsTemp = 0;
                    }

                    nowPoint = point * numberOfVariables;

                    // Store the search direction
                    for (int i = 0; i < numberOfVariables; i++)
                    {
                        steps[nowPoint + i] = gradientStore[i];
                    }


                    stp = 1;
                }
                // Save original gradient
                UnsafeUtility.MemCpy(gradientStore.GetUnsafePtr(), gradient.GetUnsafePtr(), gradient.Length * UnsafeUtility.SizeOf<float>());
/*                for (int i = 0; i < gradient.Length; i++)
                {
                    gradientStore[i] = gradient[i];//OYM：原始数据
                }*/
                // Obtain the one-dimensional minimizer of f by computing a line search
                bool isContinue = mcsrch(currentSolution, ref fitness, ref gradient, steps.Slice(point * numberOfVariables, numberOfVariables), ref stp, out nfev, diagonal);

                // Register evaluations
                evaluations += nfev;

                // Compute the new step and
                // new gradient differences
                for (int i = 0; i < gradient.Length; i++)
                {
                    steps[nowPoint + i] *= stp;
                    delta[nowPoint + i] = gradient[i] - gradientStore[i];
                }

                if (++point == corrections) point = 0;


                // Check for termination
                gnorm = Norm.Euclidean(gradient);
                xnorm = Norm.Euclidean(currentSolution);
                xnorm = math.max(1f, xnorm);




                if (!isContinue && gnorm / xnorm <= tolerance)
                    isfinish = true;

                /*                for (int i = 0; i < n; i++)
                                    ys += delta[npt + i] * steps[npt + i];

                                if (ys==0)
                                {
                                    finish = true;
                                }*/

                if (Progress != null) Progress(this, new OptimizationProgressEventArgs
                    (iterations, evaluations, gradient.ToArray(), gnorm, currentSolution.ToArray(), xnorm, fitness, stp, isfinish));
            }
        

        return fitness; // return the minimum value found (at solution x)
    }


    #region Line Search (mcsrch)

    /// <summary>
    ///   Finds a step which satisfies a sufficient decrease and curvature condition.
    /// </summary>
    /// 
    private unsafe bool mcsrch(NativeArray<float> currentSolution, ref float fitness, ref NativeArray<float> gradient, NativeSlice<float> steps,
        ref float step, out int innerLoop, NativeArray<float> diagonal)
    {
         fitnesstest1 = 0;
         funcState = 1;

        innerLoop = 0;

        if (step <= 0)
            throw new LineSearchFailedException(1, "Invalid step size.");

        // Compute the initial gradient in the search direction
        // and check that s is a descent direction.

         gradientInitial = 0;

        for (int j = 0; j < gradient.Length; j++)
        {
            gradientInitial = gradientInitial + gradient[j] * steps[j];
        }


        if (gradientInitial >= 0)//OYM：梯度爆炸
        {
            return true;
            // throw new LineSearchFailedException(0, "The search direction is not a descent direction.");
        }



         isInBracket = false;
         stage1 = true;

        float fitnessInit = fitness;
        float gradientTest = FITNESS_TOLERENCE * gradientInitial;
        float width = STEP_MAX - STEP_MIN;
        float width1 = width / 0.5f;

        for (int j = 0; j < currentSolution.Length; j++)
            diagonal[j] = currentSolution[j];

        // The variables stx, fx, dgx contain the values of the
        // step, function, and directional derivative at the best
        // step.

        float stepBoundX = 0;
        float fitnessX = fitnessInit;
        float gradientInitialX = gradientInitial;

        // The variables sty, fy, dgy contain the value of the
        // step, function, and derivative at the other endpoint
        // of the interval of uncertainty.

        float stepBoundY = 0;
        float fitnessY = fitnessInit;
        float gradientInitialY = gradientInitial;

        // The variables stp, f, dg contain the values of the step,
        // function, and derivative at the current step.

        float gradientTemp = 0;
        int innerIter = 0;

        while (true)
        {
            innerIter++;
            // Set the minimum and maximum steps to correspond
            // to the present interval of uncertainty.

            float stepBoundMin, stepBoundMax;

            if (isInBracket)//OYM：brackt意思是括号?
            {
                stepBoundMin = math.min(stepBoundX, stepBoundY);
                stepBoundMax = math.max(stepBoundX, stepBoundY);
            }
            else
            {
                stepBoundMin = stepBoundX;
                stepBoundMax = step + 4.0f * (step - stepBoundX);
            }

            // If an unusual termination is to occur then let
            // stp be the lowest point obtained so far.
            //OYM：约束step的上下界
            step = math.max(step, STEP_MIN);
            step = math.min(step, STEP_MAX);

            if (
                (isInBracket && (step <= stepBoundMin || step >= stepBoundMax)) ||//OYM：step在bound外
                (isInBracket && stepBoundMax - stepBoundMin <= xTolerance * stepBoundMax) || //OYM：stepBound区间过小
                (innerLoop >= maxInnerLoop - 1) || (funcState == 0))
            {
                step = stepBoundX;
            }
            // Force the step to be within the bounds stpmax and stpmin.


            // Evaluate the function and gradient at stp
            // and compute the directional derivative.
            // We return to main program to obtain F and G.

            for (int j = 0; j < currentSolution.Length; j++)
            {
                currentSolution[j] = diagonal[j] + step * steps[j];
                currentSolution[j] = math.clamp(currentSolution[j], -1, 1);
/*                if (currentSolution[j] > upperBound[j])//OYM: 这里可以用我自己的归一化去完成
                    currentSolution[j] = upperBound[j];
                else if (currentSolution[j] < lowerBound[j])
                    currentSolution[j] = lowerBound[j];*/
            }


            // Reevaluate function and gradient
            fitness = GetFunction(currentSolution);
            gradient = GetGradient(currentSolution);

            innerLoop++;
            gradientTemp = 0;

            for (int j = 0; j < gradient.Length; j++)
            {
                gradientTemp = gradientTemp + gradient[j] * steps[j];
            }


            fitnesstest1 = fitnessInit + step * gradientTest;

            // Test for convergence.

            if (innerLoop >= maxInnerLoop)
            {
                //OYM：Maximum number of function evaluations has been reached
                return false;
                //throw new LineSearchFailedException(3, "Maximum number of function evaluations has been reached.");
            }


            if ((isInBracket && (step <= stepBoundMin || step >= stepBoundMax)) || funcState == 0)
            {
                return false;
                /*                throw new LineSearchFailedException(6, "Rounding errors prevent further progress." +
                                                                       "There may not be a step which satisfies the sufficient decrease and curvature conditions. Tolerances may be too small. \n" +
                                                                       "stp: " + stp.ToString() + ", brackt: " + brackt.ToString() + ", infoc: " + infoc.ToString() + ", stmin: " + stmin.ToString() + ", stmax: " + stmax.ToString());*/
            }

            if (step == STEP_MAX && fitness <= fitnesstest1 && gradientTemp <= gradientTest)
            {
                //OYM：answer out the max
                return false;
                throw new LineSearchFailedException(5, "The step size has reached the upper bound.");
            }


            if (step == STEP_MIN && (fitness > fitnesstest1 || gradientTemp >= gradientTest))
            {
                return false;
                throw new LineSearchFailedException(4, "The step size has reached the lower bound.");
            }


            if (isInBracket && stepBoundMax - stepBoundMin <= xTolerance * stepBoundMax)
            {
                return false;
                throw new LineSearchFailedException(2, "Relative width of the interval of uncertainty is at machine precision.");
            }


            if (fitness <= fitnesstest1 && math.abs(gradientTemp) <= grandientTolerence * (-gradientInitial))
                return true;

            // Not converged yet. Continuing with the search.

            // In the first stage we seek a step for which the modified
            // function has a nonpositive value and nonnegative derivative.

            if (stage1 && fitness <= fitnesstest1 && gradientTemp >= math.min(FITNESS_TOLERENCE, grandientTolerence) * gradientInitial)
                stage1 = false;

            // A modified function is used to predict the step only if we
            // have not obtained a step for which the modified function has
            // a nonpositive function value and nonnegative derivative, and
            // if a lower function value has been obtained but the decrease
            // is not sufficient.

            if (stage1 && fitness <= fitnessX && fitness > fitnesstest1)
            {
                // Define the modified function and derivative values.

                float fm = fitness - step * gradientTest;
                float fxm = fitnessX - stepBoundX * gradientTest;
                float fym = fitnessY - stepBoundY * gradientTest;

                float dgm = gradientTemp - gradientTest;
                float dgxm = gradientInitialX - gradientTest;
                float dgym = gradientInitialY - gradientTest;

                // Call cstep to update the interval of uncertainty
                // and to compute the new step.

                SearchStep(ref stepBoundX, ref fxm, ref dgxm,
                    ref stepBoundY, ref fym, ref dgym, ref step,
                    fm, dgm, ref isInBracket, out funcState);

                // Reset the function and gradient values for f.
                fitnessX = fxm + stepBoundX * gradientTest;
                fitnessY = fym + stepBoundY * gradientTest;
                gradientInitialX = dgxm + gradientTest;
                gradientInitialY = dgym + gradientTest;
            }
            else
            {
                // Call mcstep to update the interval of uncertainty
                // and to compute the new step.

                SearchStep(
                    ref stepBoundX, ref fitnessX, ref gradientInitialX,
                    ref stepBoundY, ref fitnessY, ref gradientInitialY, 
                    ref step,
                    fitness, gradientTemp, ref isInBracket, out funcState);
            }

            // Force a sufficient decrease in the size of the
            // interval of uncertainty.

            if (isInBracket)
            {
                if (math.abs(stepBoundY - stepBoundX) >= 0.66f * width1)
                    step = stepBoundX + 0.5f * (stepBoundY - stepBoundX);

                width1 = width;
                width = math.abs(stepBoundY - stepBoundX);
            }

        }
    }

    // TODO: Move to separate classes
    internal static void SearchStep(ref float stepBoundX, ref float fitnessX , ref float gradientX,
                                ref float stepBoundY, ref float fitnessY, ref float gradientY,
                                ref float step, float fitnessTemp, float gradientTemp,
                                ref bool isInBracket, out int funcState)
    {
        bool bound;
        float stpc, stpf, stpq;

        funcState = 0;

        if ((isInBracket && (step <= math.min(stepBoundX, stepBoundY) || step >= math.max(stepBoundX, stepBoundY))) ||//OYM: 不连续解的情况
            (gradientX * (step - stepBoundX) >= 0.0) || (STEP_MAX < STEP_MIN))//OYM: 丢解的情况（跟算法有关）
        {
            return;
        }

        // Determine if the derivatives have opposite sign.
        float signDerivatives = gradientTemp * (gradientX / math.abs(gradientX));//OYM: 这里应该换上一个符号函数

        if (fitnessTemp > fitnessX)//OYM: 可以被优化？
        {
            // First case. A higher function value.
            // The minimum is bracketed. If the cubic step is closer
            // to stx than the quadratic step, the cubic step is taken,
            // else the average of the cubic and quadratic steps is taken.

            funcState = 1;
            bound = true;
            float theta = 3.0f * (fitnessX - fitnessTemp) / (step - stepBoundX) + gradientX + gradientTemp;
            float s = math.max(math.abs(theta), math.max(math.abs(gradientX), math.abs(gradientTemp)));
            float gamma = s * math.sqrt((theta / s) * (theta / s) - (gradientX / s) * (gradientTemp / s));

            if (step < stepBoundX)
            {
                gamma = -gamma;
            }
            float p = gamma - gradientX + theta;
            float q = gamma - gradientX + gamma + gradientTemp;
            float r = p / q;
            stpc = stepBoundX + r * (step - stepBoundX);
            stpq = stepBoundX + ((gradientX / ((fitnessX - fitnessTemp) / (step - stepBoundX) + gradientX)) / 2) * (step - stepBoundX);

            if (math.abs(stpc - stepBoundX) < math.abs(stpq - stepBoundX))
            {
                stpf = stpc;
            }

            else
            {
                stpf = stpc + (stpq - stpc) / 2.0f;
            }
            isInBracket = true;
        }
        else if (signDerivatives < 0.0)
        {
            // Second case. A lower function value and derivatives of
            // opposite sign. The minimum is bracketed. If the cubic
            // step is closer to stx than the quadratic (secant) step,
            // the cubic step is taken, else the quadratic step is taken.

            funcState = 2;
            bound = false;
            float theta = 3 * (fitnessX - fitnessTemp) / (step - stepBoundX) + gradientX + gradientTemp;
            float s = math.max(math.abs(theta), math.max(math.abs(gradientX), math.abs(gradientTemp)));
            float gamma = s * math.sqrt((theta / s) * (theta / s) - (gradientX / s) * (gradientTemp / s));

            if (step > stepBoundX)
            {
                gamma = -gamma;
            }
            float p = (gamma - gradientTemp) + theta;
            float q = ((gamma - gradientTemp) + gamma) + gradientX;
            float r = p / q;
            stpc = step + r * (stepBoundX - step);
            stpq = step + (gradientTemp / (gradientTemp - gradientX)) * (stepBoundX - step);

            if (math.abs(stpc - step) > math.abs(stpq - step))
            {
                stpf = stpc;
            }
            else
            {
                stpf = stpq;
            }
            isInBracket = true;
        }
        else if (math.abs(gradientTemp) < math.abs(gradientX))
        {
            // Third case. A lower function value, derivatives of the
            // same sign, and the magnitude of the derivative decreases.
            // The cubic step is only used if the cubic tends to infinity
            // in the direction of the step or if the minimum of the cubic
            // is beyond stp. Otherwise the cubic step is defined to be
            // either stpmin or stpmax. The quadratic (secant) step is also
            // computed and if the minimum is bracketed then the step
            // closest to stx is taken, else the step farthest away is taken.

            funcState = 3;
            bound = true;
            float theta = 3 * (fitnessX - fitnessTemp) / (step - stepBoundX) + gradientX + gradientTemp;
            float s = math.max(math.abs(theta), math.max(math.abs(gradientX), math.abs(gradientTemp)));
            float gamma = s * math.sqrt(math.max(0, (theta / s) * (theta / s) - (gradientX / s) * (gradientTemp / s)));

            if (step > stepBoundX)
            {
                gamma = -gamma;
            }

            float p = (gamma - gradientTemp) + theta;
            float q = (gamma + (gradientX - gradientTemp)) + gamma;
            float r = p / q;

            if (r < 0.0 && gamma != 0.0)
            {
                stpc = step + r * (stepBoundX - step);
            }
            else if (step > stepBoundX)
            {
                stpc = STEP_MAX;
            }
            else
            {
                stpc = STEP_MIN;
            }
            stpq = step + (gradientTemp / (gradientTemp - gradientX)) * (stepBoundX - step);

            if (isInBracket)
            {
                if (math.abs(step - stpc) < math.abs(step - stpq))
                    stpf = stpc;
                else stpf = stpq;
            }
            else
            {
                if (math.abs(step - stpc) > math.abs(step - stpq))
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

            funcState = 4;
            bound = false;

            if (isInBracket)
            {
                float theta = 3 * (fitnessTemp - fitnessY) / (stepBoundY - step) + gradientY + gradientTemp;
                float s = math.max(math.abs(theta), math.max(math.abs(gradientY), math.abs(gradientTemp)));
                float gamma = s * math.sqrt((theta / s) * (theta / s) - (gradientY / s) * (gradientTemp / s));

                if (step > stepBoundY)
                {
                    gamma = -gamma;
                }
                float p = (gamma - gradientTemp) + theta;
                float q = ((gamma - gradientTemp) + gamma) + gradientY;
                float r = p / q;
                stpc = step + r * (stepBoundY - step);
                stpf = stpc;
            }
            else if (step > stepBoundX)
            {
                stpf = STEP_MAX;
            }

            else
            {
                stpf = STEP_MIN;           
            } 
        }

        // Update the interval of uncertainty. This update does not
        // depend on the new step or the case analysis above.

        if (fitnessTemp > fitnessX)
        {
            stepBoundY = step;
            fitnessY = fitnessTemp;
            gradientY = gradientTemp;
        }
        else
        {
            if (signDerivatives < 0.0)
            {
                stepBoundY = stepBoundX;
                fitnessY = fitnessX;
                gradientY = gradientX;
            }
            stepBoundX = step;
            fitnessX = fitnessTemp;
            gradientX = gradientTemp;
        }

        // Compute the new step and safeguard it.
        stpf = math.min(STEP_MAX, stpf);
        stpf = math.max(STEP_MIN, stpf);
        step = stpf;

        if (isInBracket && bound)
        {
            if (stepBoundY > stepBoundX)
            {
                step = math.min(stepBoundX + 0.66f * (stepBoundY - stepBoundX), step);
            }
            else
            {
                step = math.max(stepBoundX + 0.66f * (stepBoundY - stepBoundX), step);
            }
        }

        return;
    }


    #endregion


/*    private NativeArray<float> GetDiagonal()
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
    }*/


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
        diagonal = new NativeArray<float>(numberOfVariables, Allocator.Persistent);

        gradientStore = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        rho = new NativeArray<float>(corrections, Allocator.Persistent);                  // Stores the scalars rho.
        alpha = new NativeArray<float>(corrections, Allocator.Persistent);               // Stores the alphas in computation of H*g.
        steps = new NativeArray<float>(numberOfVariables * corrections, Allocator.Persistent);          // Stores the last M search steps.
        delta= new NativeArray<float>(numberOfVariables * corrections, Allocator.Persistent);
    }

    public void Dispose()
    {
        currentSolution.Dispose();
        gradient.Dispose();
        work.Dispose();

        diagonal.Dispose();
        gradientStore.Dispose();
        rho.Dispose();
        alpha.Dispose();
        steps.Dispose();
        delta.Dispose();

    }
}

