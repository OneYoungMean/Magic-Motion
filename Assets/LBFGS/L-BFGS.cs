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
//    MERCHANTABILITY or loss FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
using System;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using static L_BFGSStatic;
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

    // Line search parameters
    public float lossTolerance = 1f;

    public float gradientTolerance = 1e-5f;
    private int iterations;
    private int evaluations;

    public int numberOfVariables;


    private float loss;   // value at current solution f(x)

    private bool isLoopInside;
    private float innerLoopStep;

    private int loopCount;
    private int point;
    private int matrixPoint;
    private bool isLoopOutside;
    private int funcState;
    private bool isInBracket;
    private bool stage1;
    private float preloss;
    private float width;
    private float width1;

    private NativeArray<float> diagonal;
    private NativeArray<float> gradientStore;
    private NativeArray<float> rho;
    private NativeArray<float> alpha;
    private NativeArray<float> steps;
    private NativeArray<float> delta;
    private NativeArray<float> currentSolution; // current solution x
    private NativeArray<float> gradient;         // gradient at current solution

    private float stepBoundX;
    private float lossX;
    private float gradientInitialX;
    private float stepBoundY;
    private float lossY;
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
        get { return lossTolerance; }
        set
        {
            if (value <= 1e-4)
                throw new ArgumentOutOfRangeException("value");

            lossTolerance = value;
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
        get { return loss; }
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

        CreateWorkVector(numberOfVariables,out diagonal,out gradientStore,out rho,out alpha,out steps,out delta);

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

        minimize();

        UnsafeUtility.MemCpy(values.GetUnsafePtr(), currentSolution.GetUnsafePtr(), currentSolution.Length * UnsafeUtility.SizeOf<float>());

        return loss;
    }

    private float minimize()
    {
        ClearData(diagonal, gradientStore, rho, alpha, steps, delta);
        // Make initial evaluation
        loss = GetFunction(currentSolution);
        gradient = GetGradient(currentSolution);
        //OYM：InitializeLoop
        InitializeLoop(ref innerLoopStep, ref iterations,ref evaluations,ref loopCount,ref point,ref matrixPoint,ref isLoopOutside,ref gradient,ref diagonal,ref steps);

        // outside loop
        while (isLoopOutside)
        {
            OutsideLoopHead(ref width, ref width1,ref stepBoundX,ref stepBoundY,ref preGradientSum,ref innerLoopStep,ref preloss,ref loss,ref lossX,ref lossY,ref gradientInitialX,ref gradientInitialY,ref funcState,ref iterations,ref matrixPoint,ref numberOfVariables, ref point, ref isLoopOutside,ref isLoopInside, ref isInBracket,ref stage1,ref delta,ref steps,ref diagonal,ref gradientStore,ref gradient,ref rho,ref alpha, ref currentSolution);
            if (!isLoopOutside) break;

            //inner loop
            while (isLoopInside)
            {
                int temp = L_BFGSStatic.MAXLOOPCOUNT;
                InsideLoopHead(ref stepBoundMin, ref stepBoundMax,ref stepBoundX,ref stepBoundY,ref innerLoopStep,ref loopCount,ref numberOfVariables,ref funcState,ref matrixPoint,ref temp, ref isInBracket,ref currentSolution,ref diagonal, ref steps);

                // Reevaluate function and gradient
                loss = GetFunction(currentSolution);
                gradient = GetGradient(currentSolution);

              InisdeLoopTail(ref preGradientSum,ref preloss,ref innerLoopStep,ref stepBoundMin,ref stepBoundMax,ref loss,ref lossTolerance,ref lossX,ref lossY,ref stepBoundX,ref stepBoundY,ref gradientInitialX,ref gradientInitialY,ref width,ref width1,ref loopCount,ref numberOfVariables,ref funcState, ref matrixPoint,ref temp, ref isLoopOutside, ref isLoopInside , ref isInBracket,ref stage1,ref gradient,ref steps);
                if (!isLoopInside) break;
            }
            if (!isLoopOutside) break;
            OutsideLoopTail(ref innerLoopStep, ref gradientTolerance,
                ref loopCount,ref matrixPoint,ref point, ref  numberOfVariables,
                ref isLoopOutside,
                ref gradient,ref steps,ref delta,ref gradientStore,ref currentSolution);
        }

        return loss; // return the minimum value found (at solution x)
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

    public void Dispose()
    {
        currentSolution.Dispose();
        Disposed(diagonal, gradientStore, rho, alpha, steps, delta);
    }
}

