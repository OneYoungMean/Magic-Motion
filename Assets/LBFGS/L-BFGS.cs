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
    public const float STEP_MIN = 1e-5f;
    private const float STEP_MAX = 180f;
    private const float RANGLE_MIN = -math.PI / 6;
    private const float RANGE_MAX = math.PI / 6;

    // Line search parameters
    public float fitnessTolerance = 1f;
    public int maxInnerLoop = 40;

    public float gradientTolerance = 1e-5f;
    private int iterations;
    private int evaluations;

    public int numberOfVariables;
    public int corrections = 5;

    private NativeArray<float> currentSolution; // current solution x
    private float fitness;   // value at current solution f(x)
    NativeArray<float> gradient;         // gradient at current solution
    private bool isLoopInside;
    private float innerLoopStep;

    private int innerLoopCount;
    private int point;
    private int matrixPoint;
    private bool isLoopOutside;
    private int funcState;
    private bool isInBracket;
    private bool stage1;
    private float preFitness;
    private float width;
    private float width1;
    private NativeArray<float> diagonal;
    private NativeArray<float> gradientStore;
    private NativeArray<float> rho;
    private NativeArray<float> alpha;
    private NativeArray<float> steps;
    private NativeArray<float> delta;
    private NativeSlice<float> innerLoopSteps;
    private float stepBoundX;
    private float fitnessX;
    private float gradientInitialX;
    private float stepBoundY;
    private float fitnessY;
    private float gradientInitialY;
    private float preGradientSum;
    private float stepBoundMin;
    private float stepBoundMax;



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
        get { return gradientTolerance; }
        set { gradientTolerance = value; }
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
        get { return fitnessTolerance; }
        set
        {
            if (value <= 1e-4)
                throw new ArgumentOutOfRangeException("value");

            fitnessTolerance = value;
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

    public static void Initialize(ref float innerLoopStep, ref int iterations, ref int evaluations, ref int innerLoopCount, ref int point,ref int matrixPoint,ref bool isLoopOutside)
    {
        innerLoopStep = 0;
        iterations = 0;
        evaluations = 1;
        innerLoopCount = 0;
        point = 0;
        matrixPoint = 0;
        isLoopOutside = true;
    }
    public static void InitializeOutsideLoop(ref float width,ref float width1,ref float stepBoundX,ref float stepBoundY, ref float preGradientSum, ref int funcState,ref int innerLoopCount,ref bool isLoopOutside,ref bool isLoopInside, ref bool isInBracket,ref bool stage1)
    {
        width = STEP_MAX - STEP_MIN;
        width1 = width / 0.5f;
        stepBoundX = 0f;
        stepBoundY = 0f;
        preGradientSum = 0f;

        funcState = 1;
        innerLoopCount = 0;

        isLoopOutside = false;
        isLoopInside = true;

        isInBracket = false;
        stage1 = true;
    }
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

        if (values.Length * 3 != numberOfVariables)
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

    private static void InitializeLoop(
        ref float innerLoopStep,
        ref int iterations, ref int evaluations, ref int innerLoopCount, ref int point, ref int matrixPoint,
        ref bool isLoopOutside,
        ref NativeArray<float> gradient, ref NativeArray<float> diagonal, ref NativeArray<float> steps
        )
    {
        // Initialization
        Initialize(ref innerLoopStep, ref iterations, ref evaluations, ref innerLoopCount, ref point, ref matrixPoint,ref isLoopOutside);

        // Obtain initial Hessian
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
        float gnormInit = Norm.Euclidean(gradient);
        innerLoopStep = 1.0f / gnormInit;
    }

    private static bool OutsideLoopHead(
ref float width, ref float width1, ref float stepBoundX, ref float stepBoundY, ref float preGradientSum, ref float innerLoopStep, ref float preFitness, ref float fitness, ref float fitnessX, ref float fitnessY, ref float gradientInitialX, ref float gradientInitialY,
ref int funcState, ref int innerLoopCount, ref int iterations, ref int matrixPoint, ref int numberOfVariables, ref int point,
ref bool isLoopOutside, ref bool isLoopInside, ref bool isInBracket, ref bool stage1,
ref NativeArray<float> delta, ref NativeArray<float> steps, ref NativeArray<float> diagonal,ref int corrections, ref NativeArray<float> gradientStore, ref NativeArray<float> gradient, ref NativeArray<float> rho, ref NativeArray<float> alpha,
ref NativeArray<float> currentSolution,
ref NativeSlice<float> innerLoopSteps)
    {
        InitializeOutsideLoop(ref width, ref width1, ref stepBoundX, ref stepBoundY,ref preGradientSum,ref funcState, ref innerLoopCount, ref isLoopOutside, ref isLoopInside, ref isInBracket, ref stage1);
        iterations++;
        if (iterations != 1)
        {
            float sumY = GetSumY(delta, steps, matrixPoint, numberOfVariables);

            // Compute the diagonal of the Hessian
            // or use an approximation by the user.
            ComputeDiagonal(diagonal, delta, matrixPoint, numberOfVariables, sumY);

            // Compute -H*g using the formula given in:
            //   Nocedal, J. 1980, "Updating quasi-Newton matrices with limited storage",
            //   Mathematics of Computation, Vol.24, No.151, pp. 773-782.
            int bound = math.min(iterations - 1, corrections);
            for (int i = 0; i < numberOfVariables; i++)
            {
                gradientStore[i] = -gradient[i];
            }
            UpdatingQuasi_Newton(diagonal, rho, gradientStore, steps, alpha, delta, numberOfVariables, corrections, point, bound, sumY);

            // Store the search direction
            matrixPoint = point * numberOfVariables;
            for (int i = 0; i < numberOfVariables; i++)
            {
                steps[matrixPoint + i] = gradientStore[i];
            }
            innerLoopStep = 1;
        }

        // Save original gradient
        UnsafeUtility.MemCpy(gradientStore.GetUnsafePtr(), gradient.GetUnsafePtr(), gradient.Length * UnsafeUtility.SizeOf<float>());

         innerLoopSteps = steps.Slice(point * numberOfVariables, numberOfVariables);

        if (innerLoopStep <= 0)
        {
            //OYM：Invalid step size
            return isLoopOutside;
        }

        // Compute the initial gradient in the search direction
        // and check that s is a descent direction.
         preGradientSum = GetGradientInitial(gradient, innerLoopSteps, numberOfVariables);

        if (preGradientSum >= 0)//OYM：梯度爆炸
        {
            return isLoopOutside;
            // throw new LineSearchFailedException(0, "The search direction is not a descent direction.");
        }
        // safe fitness and gradient
        preFitness = fitness;
        UnsafeUtility.MemCpy(diagonal.GetUnsafePtr(), currentSolution.GetUnsafePtr(), numberOfVariables * UnsafeUtility.SizeOf<float>());

        // The variables stx, fx, dgx contain the values of the
        // step, function, and directional derivative at the best
        // step.
        // The variables sty, fy, dgy contain the value of the
        // step, function, and derivative at the other endpoint
        // of the interval of uncertainty.
        fitnessX = fitnessY = preFitness;
        gradientInitialX = gradientInitialY = preGradientSum;

        return true;
    }
    private static bool InsideLoopHead(
        ref float stepBoundMin, ref float stepBoundMax, ref float stepBoundX, ref float stepBoundY, ref float innerLoopStep,
        ref int innerLoopCount,ref int maxInnerLoop,ref int numberOfVariables, ref int funcState,
        ref bool isInBracket,
        ref NativeArray<float> currentSolution, ref NativeArray<float> diagonal, ref NativeSlice<float> innerLoopSteps
        )
    {
        // Set the minimum and maximum steps to correspond
        // to the present interval of uncertainty.

        if (isInBracket)//OYM：brackt意思是括号?
        {
            stepBoundMin = math.min(stepBoundX, stepBoundY);
            stepBoundMax = math.max(stepBoundX, stepBoundY);
        }
        else
        {
            stepBoundMin = stepBoundX;
            stepBoundMax = innerLoopStep + 4.0f * (innerLoopStep - stepBoundX);
        }

        // If an unusual termination is to occur then let
        // stp be the lowest point obtained so far.
        //OYM：约束step的上下界
        innerLoopStep = math.max(innerLoopStep, STEP_MIN);
        innerLoopStep = math.min(innerLoopStep, STEP_MAX);

        if (
            (isInBracket && (innerLoopStep <= stepBoundMin || innerLoopStep >= stepBoundMax)) ||//OYM：step在bound外
            (isInBracket && stepBoundMax - stepBoundMin <= xTolerance * stepBoundMax) || //OYM：stepBound区间过小
            (innerLoopCount >= maxInnerLoop - 1) || (funcState == 0))
        {
            innerLoopStep = stepBoundX;
        }

        // Force the step to be within the bounds stpmax and stpmin.
        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        // We return to main program to obtain F and G.
        for (int j = 0; j < numberOfVariables; j++)
        {
            currentSolution[j] = diagonal[j] + innerLoopStep * innerLoopSteps[j];
            currentSolution[j] = math.clamp(currentSolution[j], RANGLE_MIN, RANGE_MAX);
        }
        return true;
    }

    private static bool InisdeLoopTail(
        ref float preGradientSum, ref float preFitness, ref float innerLoopStep, ref float stepBoundMin, ref float  stepBoundMax,ref float fitness,ref float fitnessTolerance, ref float fitnessX, ref float fitnessY, ref float stepBoundX, ref float stepBoundY, ref float gradientInitialX, ref float gradientInitialY, ref float width, ref float width1,
        ref int innerLoopCount,ref int numberOfVariables, ref int maxInnerLoop ,ref int funcState,
        ref bool isLoopOutside, ref bool isInBracket,ref bool stage1,
        ref NativeArray<float> gradient, ref NativeSlice<float> innerLoopSteps
        )
    {
        innerLoopCount++;
        float gradientTemp = GetGreadientTemp(gradient
            , innerLoopSteps, numberOfVariables);

        float gradientTest = FITNESS_TOLERENCE * preGradientSum;
        float fitnesstest1 = preFitness + innerLoopStep * gradientTest;

        // Test for convergence.
        if (innerLoopCount >= maxInnerLoop)
        {
            isLoopOutside = false;
            return false;
            //throw new LineSearchFailedException(3, "Maximum number of function evaluations has been reached.");
        }

        //Rounding errors prevent further progress.
        if ((isInBracket && (innerLoopStep <= stepBoundMin || innerLoopStep >= stepBoundMax)) || funcState == 0)
        {
            isLoopOutside = false;
            return false;
        }

        //The step size has reached the upper bound.
        if (innerLoopStep == STEP_MAX && fitness <= fitnesstest1 && gradientTemp <= gradientTest)
        {
            //OYM：answer out the max
            isLoopOutside = false;
            return false;
        }

        //The step size has reached the lower bound.
        if (innerLoopStep == STEP_MIN && (fitness > fitnesstest1 || gradientTemp >= gradientTest))
        {
            isLoopOutside = false;
            return false;
        }

        //Relative width of the interval of uncertainty is at machine precision
        if (isInBracket && stepBoundMax - stepBoundMin <= xTolerance * stepBoundMax)
        {
            isLoopOutside = false;
            return false;
        }

        // successful
        if (fitness <= fitnesstest1 && math.abs(gradientTemp) <= fitnessTolerance * (-preGradientSum))
        {
            isLoopOutside = true;
            return false;

        }

        // Not converged yet. Continuing with the search.
        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.
        if (stage1 &&
            fitness <= fitnesstest1 &&
            gradientTemp >= math.min(FITNESS_TOLERENCE, fitnessTolerance) * preGradientSum)
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we
        // have not obtained a step for which the modified function has
        // a nonpositive function value and nonnegative derivative, and
        // if a lower function value has been obtained but the decrease
        // is not sufficient.
        if (stage1 && fitness <= fitnessX && fitness > fitnesstest1)
        {
            // Define the modified function and derivative values.

            float fm = fitness - innerLoopStep * gradientTest;
            float fxm = fitnessX - stepBoundX * gradientTest;
            float fym = fitnessY - stepBoundY * gradientTest;

            float dgm = gradientTemp - gradientTest;
            float dgxm = gradientInitialX - gradientTest;
            float dgym = gradientInitialY - gradientTest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            SearchStep(ref stepBoundX, ref fxm, ref dgxm,
                ref stepBoundY, ref fym, ref dgym, ref innerLoopStep,
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
                ref innerLoopStep,
                fitness, gradientTemp, ref isInBracket, out funcState);
        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.
        if (isInBracket)
        {
            if (math.abs(stepBoundY - stepBoundX) >= 0.66f * width1)
                innerLoopStep = stepBoundX + 0.5f * (stepBoundY - stepBoundX);

            width1 = width;
            width = math.abs(stepBoundY - stepBoundX);
        }
        return true;
    }

    private static bool OutsideLoopTail(
         ref float innerLoopStep, ref float gradientTolerance,
           ref int innerLoopCount, ref int evaluations, ref int matrixPoint, ref int point, ref int corrections, ref int numberOfVariables,
           ref bool isLoopOutside, 
           ref NativeArray<float> gradient, ref NativeArray<float> steps, ref NativeArray<float> delta, ref NativeArray<float> gradientStore, ref NativeArray<float> currentSolution
        )
    {
        // Register evaluations
        evaluations += innerLoopCount;

        // Compute the new step and
        // new gradient differences
        for (int i = 0; i < numberOfVariables; i++)
        {
            steps[matrixPoint + i] *= innerLoopStep;
            delta[matrixPoint + i] = gradient[i] - gradientStore[i];
        }

        // point loop
        point++;
        if (point == corrections)
        {
            point = 0;
        }

        // Check for termination
        float gnorm = Norm.Euclidean(gradient);
        float xnorm = Norm.Euclidean(currentSolution);
        xnorm = math.max(1f, xnorm);

        // isSuccessful
        if (gnorm / xnorm <= gradientTolerance)
        {
            isLoopOutside = false;
        }
        return true;
    }

    private float minimize()
    {
        // Make initial evaluation
        fitness = GetFunction(currentSolution);
        gradient = GetGradient(currentSolution);
        //OYM：InitializeLoop
        InitializeLoop(ref innerLoopStep, ref iterations,ref evaluations,ref innerLoopCount,ref point,ref matrixPoint,ref isLoopOutside,ref gradient,ref diagonal,ref steps);

        // outside loop
        while (isLoopOutside)
        {
             isLoopOutside= OutsideLoopHead(ref width, ref width1,ref stepBoundX,ref stepBoundY,ref preGradientSum,ref innerLoopStep,ref preFitness,ref fitness,ref fitnessX,ref fitnessY,ref gradientInitialX,ref gradientInitialY,ref funcState,ref innerLoopCount,ref iterations,ref matrixPoint,ref numberOfVariables, ref point, ref isLoopOutside,ref isLoopInside, ref isInBracket,ref stage1,ref delta,ref steps,ref diagonal,ref corrections,ref gradientStore,ref gradient,ref rho,ref alpha, ref currentSolution, ref innerLoopSteps);
            if (!isLoopOutside) break;

            //inner loop
            while (isLoopInside)
            {
                InsideLoopHead(ref stepBoundMin, ref stepBoundMax,ref stepBoundX,ref stepBoundY,ref innerLoopStep,ref innerLoopCount,ref maxInnerLoop,ref numberOfVariables,ref funcState,ref isInBracket,ref currentSolution,ref diagonal,ref innerLoopSteps);

                // Reevaluate function and gradient
                fitness = GetFunction(currentSolution);
                gradient = GetGradient(currentSolution);

                isLoopInside = InisdeLoopTail(ref preGradientSum,ref preFitness,ref innerLoopStep,ref stepBoundMin,ref stepBoundMax,ref fitness,ref fitnessTolerance,ref fitnessX,ref fitnessY,ref stepBoundX,ref stepBoundY,ref gradientInitialX,ref gradientInitialY,ref width,ref width1,ref innerLoopCount,ref numberOfVariables,ref maxInnerLoop,ref funcState,ref isLoopOutside,ref isInBracket,ref stage1,ref gradient,ref innerLoopSteps);
                if (!isLoopInside) break;
            }

            OutsideLoopTail(ref innerLoopStep, ref gradientTolerance,
                ref innerLoopCount,ref evaluations,ref matrixPoint,ref point, ref corrections, ref  numberOfVariables,
                ref isLoopOutside,
                ref gradient,ref steps,ref delta,ref gradientStore,ref currentSolution);
        }

        return fitness; // return the minimum value found (at solution x)
    }

    private static float GetGreadientTemp(NativeArray<float> gradient, NativeSlice<float> innerLoopSteps, int numOf)
    {
        float gradientTemp = 0;
        for (int j = 0; j < gradient.Length; j++)
        {
            gradientTemp = gradientTemp + gradient[j] * innerLoopSteps[j];
        }
        return gradientTemp;
    }

    private static float GetGradientInitial(NativeArray<float> gradient, NativeSlice<float> innerLoopSteps, int numberOfVariables)
    {
        float gradientInitial = 0f;
        for (int j = 0; j < numberOfVariables; j++)
        {
            gradientInitial += gradient[j] * innerLoopSteps[j];
        }
        return gradientInitial;
    }

    private static void UpdatingQuasi_Newton(
     NativeArray<float> diagonal, NativeArray<float> rho, NativeArray<float> gradientStore, NativeArray<float> steps, NativeArray<float> alpha, NativeArray<float> delta,
        int numberOfVariables, int corrections, int point, int bound, float sumY)
    {
        int prePointLoop = ((point == 0) ? corrections : point) - 1;
        rho[prePointLoop] = 1.0f / sumY;



        prePointLoop = point;
        for (int i = 0; i < bound; i++)
        {
            prePointLoop--;
            if (prePointLoop == -1) prePointLoop = corrections - 1;

            float lastSum = 0;
            for (int j = 0; j < numberOfVariables; j++)
            {
                lastSum += steps[prePointLoop * numberOfVariables + j] * gradientStore[j];
            }


            alpha[prePointLoop] = rho[prePointLoop] * lastSum;
            for (int j = 0; j < numberOfVariables; j++)
            {
                gradientStore[j] -= alpha[prePointLoop] * delta[prePointLoop * numberOfVariables + j];
            }
        }

        for (int i = 0; i < diagonal.Length; i++)
        {
            gradientStore[i] *= diagonal[i];
        }


        for (int i = 0; i < bound; i++)
        {
            float yr = 0;
            for (int j = 0; j < numberOfVariables; j++)
            {
                yr += delta[prePointLoop * numberOfVariables + j] * gradientStore[j];
            }


            float beta = alpha[prePointLoop] - rho[prePointLoop] * yr;
            for (int j = 0; j < numberOfVariables; j++)
            {
                gradientStore[j] += beta * steps[prePointLoop * numberOfVariables + j];
            }

            prePointLoop++;

            if (prePointLoop == corrections) prePointLoop = 0;
        }
    }

    private static void ComputeDiagonal(NativeArray<float> diagonal, NativeArray<float> delta, int nowPoint, int numberOfVariables, float sumY)
    {
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
    }

    private static float GetSumY(NativeArray<float> delta, NativeArray<float> steps, int nowPoint, int numberOfVariables)
    {
        float sumY = 0;
        for (int i = 0; i < numberOfVariables; i++)
        {
            sumY += delta[nowPoint + i] * steps[nowPoint + i];
        }

        return sumY;
    }


    #region Line Search (mcsrch)
    /*
        /// <summary>
        ///   Finds a step which satisfies a sufficient decrease and curvature condition.
        /// </summary>
        /// 
        private unsafe bool mcsrch(NativeArray<float> currentSolution, ref float fitness, ref NativeArray<float> gradient, NativeSlice<float> innerLoopSteps,
            ref float innerLoopStep, out int innerLoopCount, NativeArray<float> diagonal)
        {

        }*/

    // TODO: Move to separate classes
    internal static void SearchStep(ref float stepBoundX, ref float fitnessX, ref float gradientX,
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
        delta = new NativeArray<float>(numberOfVariables * corrections, Allocator.Persistent);
    }

    public void Dispose()
    {
        currentSolution.Dispose();

        diagonal.Dispose();
        gradientStore.Dispose();
        rho.Dispose();
        alpha.Dispose();
        steps.Dispose();
        delta.Dispose();

    }
}

