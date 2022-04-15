//Copyright ? OneYoungMean, 2022 
//https://github.com/OneYoungMean
//
// Accord Math Library
// The Accord.NET Framework
// http://accord-framework.net
//
// Copyright ? C谷sar Souza, 2009-2014
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
using Unity.Collections;

namespace MagicMotion
{
    #region summary
    /// <summary>
    ///   Limited-memory Broyden每Fletcher每Goldfarb每Shanno (L-BFGS) optimization method.
    /// </summary>
    /// 
    /// <remarks>
    /// <para>
    ///   The L-BFGS algorithm is a member of the broad family of quasi-Newton optimization
    ///   methods. L-BFGS stands for 'Limited memory BFGS'. Indeed, L-BFGS uses a limited
    ///   memory variation of the Broyden每Fletcher每Goldfarb每Shanno (BFGS) update to approximate
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
    public unsafe struct MMLBFGSNative
    {
        /// <summary>
        /// 
        /// </summary>
        private const float FITNESS_TOLERENCE = 1e-2f;
        private const float xTolerance = 1e-2f; // machine precision
        private const float STEP_MIN = 1e-2f;
        private const float STEP_MAX = 180f;
        /// <summary>
        ///   Gets or sets a tolerance value controlling the accuracy of the
        ///   line search routine. If the function and gradient evaluations are
        ///   inexpensive with respect to the cost of the iteration (which is
        ///   sometimes the case when solving very large problems) it may be
        ///   advantageous to set this to a small value. A typical small value
        ///   is 0.1. This value should be greater than 1e-4. Default is 0.9.
        /// </summary>
        public static  float grandientTolerence = 1f;

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
        public static float tolerance = 0.1f;
        /// <summary>
        ///   Gets the number of iterations performed in the last
        /// </summary>
        public static float iterations=0;
        /// <summary>
        /// Gets the number of function evaluations performe  in the last loop
        /// dont know how to use that
        /// </summary>
        public static int evaluations=1;
        /*        /// <summary>
                /// number of variables (free parameters), in the optimization problem.
                /// </summary>
                public int numberOfVariables;*/
        /// <summary>
        ///   Gets or sets the number of corrections used in the L-BFGS
        ///   update. Recommended values are between 3 and 7. Default is 5.
        /// </summary>
        public static int correction=5;
        /// <summary>
        ///   Gets the number of variables (free parameters)
        ///   in the optimization problem.
        /// </summary>
        public static int numberOfVariables=0;
        /// <summary>
        ///  value at current solution f(x)
        /// </summary>
        public float fitness;

        private float preFitness;
        private float fitnessX;
        private float fitnessY;

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

        private int innerLoopCount;
        private int funcState;

        private bool isLoopInside;
        private bool isLoopOutside;
        private bool isFinish;

        private bool isInBracket;
        private bool stage1;


        public void Optimize(
            //NativeArray<float> upperBound,  NativeArray<float> lowerBound,//蘇�珂�1睿-1
            NativeArray<float> currentSolution,
            NativeArray<float> diagonal, NativeArray<float> work,
            NativeArray<float> gradient,float fitness

            )
        {
            float* workArea = (float*)work.GetUnsafePtr();

            // The first N locations of the work vector are used to
            //  store the gradient and other temporary information.

            float* rho = &workArea[numberOfVariables];                   // Stores the scalars rho.
            float* alpha = &workArea[numberOfVariables + correction];             // Stores the alphas in computation of H*g.
            float* steps = &workArea[numberOfVariables + 2 * correction];         // Stores the last M search steps.
            float* delta = &workArea[numberOfVariables + 2 * correction + numberOfVariables * correction]; // Stores the last M gradient differences.

            // Initialize work vector
            for (int i = 0; i < gradient.Length; i++)
            {
                steps[i] = -gradient[i] * diagonal[i];
            }

            // Initialize statistics
            float gnorm = Euclidean(gradient);
            float xnorm = Euclidean(currentSolution);
            float stp = 1.0f / gnorm;
            float stp1 = stp;

            // Initialize loop
            int nfev, point = 0;
            int nowPoint = 0, cp = 0;
            bool finish = false;


        }

        public static float Euclidean( NativeArray<float> a)
        {
            return math.sqrt(SquareEuclidean(a));
        }
        public static float SquareEuclidean( NativeArray<float> a)
        {
            float sum = 0;
            for (int i = 0; i < a.Length; i++)
                sum += a[i] * a[i];
            return sum;

        }
    }
}