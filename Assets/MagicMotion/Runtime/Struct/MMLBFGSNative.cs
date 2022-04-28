//Copyright ? OneYoungMean, 2022 
//https://github.com/OneYoungMean
//
// Accord Math Library
// The Accord.NET Framework
// http://accord-framework.net
//
// Copyright ? César Souza, 2009-2014
// cesarsouza at gmail.com
//
// Copyright ? Jorge Nocedal, 1990
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
using Unity.Collections;
using static L_BFGSStatic;

namespace MagicMotion
{
    public enum LBFGSState : Byte
    {
        
        Initialize = 0,
        Refresh=1,
        OutsideLoopHead = 2,
        InsideLoopHead = 3,
        InsideLoopTail = 4,
        OutsideLoopTail = 5,
        Finish = 6
    }

    #region summary
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
    /// //   f(x,y)  =  -exp{-(x-1)?} - exp{-(y-2)?/2}
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
    #endregion
    public unsafe struct MMLBFGSSolver
    {
        /// <summary>
        ///   Gets or sets a tolerance value controlling the accuracy of the
        ///   line search routine. If the function and gradient evaluations are
        ///   inexpensive with respect to the cost of the iteration (which is
        ///   sometimes the case when solving very large problems) it may be
        ///   advantageous to set this to a small value. A typical small value
        ///   is 0.1. This value should be greater than 1e-4. Default is 0.9.
        /// </summary>
        public  float lossTolerance;
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
        public  float gradientTolerance;

        /// <summary>
        ///   Gets the number of variables (free parameters)
        ///   in the optimization problem.
        /// </summary>
        public int numberOfVariables;
        /// <summary>
        ///   Gets the number of iterations performed in the last
        /// </summary>
        private int iterations;
        /// <summary>
        /// Gets the number of function evaluations performe  in the last loop
        /// dont know how to use that
        /// </summary>
        public int evaluations;
        /// <summary>
        ///  State
        /// </summary>
        public LBFGSState state;

        private float preloss;
        private float lossX;
        private float lossY;

        private float preGradientSum;
        private float gradientInitialX;
        private float gradientInitialY;

        private float stepBoundX;
        private float stepBoundY;
        private float stepBoundMin;
        private float stepBoundMax;

        private float width;
        private float width1;

        private float innerLoopStep;

        private int point;
        private int matrixPoint;

        private int loopCount;
        private int funcState;

        private bool isLoopInside;
        private bool isLoopOutside;
        private bool isFinish;

        private bool isInBracket;
        private bool stage1;
        public float loss;
        public static readonly MMLBFGSSolver identity = new MMLBFGSSolver()
        {
            lossTolerance =1f,
            gradientTolerance =0f,
            state = LBFGSState.Initialize
        };

        public void Reset()
        {
            state = LBFGSState.Initialize;
        }

        public void Optimize(float loss, ref int leastLoopCount,//OYM：innerloop里面可以判断leastloopCount，避免性能浪费，或者更好一点，每次开始都重设一下
NativeArray<float> diagonal, NativeArray<float> gradientStore, NativeArray<float> rho, NativeArray<float> alpha, NativeArray<float> steps, NativeArray<float> delta, NativeArray<float> currentSolution, NativeArray<float> gradient
            )
        {
            this.loss = loss;
            while (true)
            {
                switch (state)
                {
                    case LBFGSState.Initialize://OYM：deal first in;
                        {
                            state = LBFGSState.Refresh;
                            break;
                        }
                    case LBFGSState.Refresh:
                        {
                            ClearData(diagonal, gradientStore, rho, alpha, steps, delta);
                            InitializeLoop(ref innerLoopStep, ref iterations, ref evaluations, ref loopCount, ref point, ref matrixPoint, ref isLoopOutside, ref gradient, ref diagonal, ref steps);
                            state = LBFGSState.OutsideLoopHead;
                        }
                        break;
                    case LBFGSState.OutsideLoopHead:
                        if (isLoopOutside)
                        {
                            OutsideLoopHead(ref width, ref width1, ref stepBoundX, ref stepBoundY, ref preGradientSum, ref innerLoopStep, ref preloss, ref loss, ref lossX, ref lossY, ref gradientInitialX, ref gradientInitialY, ref funcState, ref iterations, ref matrixPoint, ref numberOfVariables, ref point, ref isLoopOutside, ref isLoopInside, ref isInBracket, ref stage1, ref delta, ref steps, ref diagonal , ref gradientStore, ref gradient, ref rho, ref alpha, ref currentSolution);
                            if (isLoopOutside)
                            {
                                state = LBFGSState.InsideLoopHead;
                            }
                            else
                            {
                                state =LBFGSState.Finish;
                                leastLoopCount--;
                                return;
                            }

                        }
                        else
                        {
                            state = LBFGSState.Finish;
                        }
                        break;
                    case LBFGSState.InsideLoopHead:
                        if (isLoopInside)
                        {
                            InsideLoopHead(ref stepBoundMin, ref stepBoundMax, ref stepBoundX, ref stepBoundY, ref innerLoopStep, ref loopCount, ref numberOfVariables, ref funcState, ref matrixPoint, ref leastLoopCount, ref isInBracket, ref currentSolution, ref diagonal, ref steps);
                            state = LBFGSState.InsideLoopTail;
                            leastLoopCount--;
                            return;
                        }
                        else
                        {
                            state = LBFGSState.OutsideLoopTail;
                        }
                        break;
                    case LBFGSState.InsideLoopTail:
                        InisdeLoopTail(ref preGradientSum, ref preloss, ref innerLoopStep, ref stepBoundMin, ref stepBoundMax, ref loss, ref lossTolerance, ref lossX, ref lossY, ref stepBoundX, ref stepBoundY, ref gradientInitialX, ref gradientInitialY, ref width, ref width1, ref loopCount, ref numberOfVariables, ref funcState, ref matrixPoint, ref leastLoopCount, ref isLoopOutside, ref isLoopInside, ref isInBracket, ref stage1, ref gradient, ref steps);
                        if (isLoopInside)
                        {
                            state = LBFGSState.InsideLoopHead;
                        }
                        else
                        {
                            state = LBFGSState.OutsideLoopTail;
                        }
                        break;
                    case LBFGSState.OutsideLoopTail:
                        if (isLoopOutside)
                        {
                            OutsideLoopTail(ref innerLoopStep, ref gradientTolerance,
                                ref loopCount, ref matrixPoint, ref point,  ref numberOfVariables,
                                ref isLoopOutside,
                                ref gradient, ref steps, ref delta, ref gradientStore, ref currentSolution);
                            state = LBFGSState.OutsideLoopHead;
                        }
                        else
                        {
                            state = LBFGSState.Finish;
                        }
                        break;
                    case LBFGSState.Finish:
                        if (leastLoopCount>0)
                        {
                            state = LBFGSState.Refresh;
                            break;
                        }
                        else
                        {
                            return;
                        }

                    default:
                        return;
                }
            }
            //OYM：InitializeLoop

        }
    }
}