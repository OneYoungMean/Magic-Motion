using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

public unsafe static class L_BFGSStatic
{
    #region Constandard
    /// <summary>
    ///   Gets or sets the number of corrections used in the L-BFGS
    ///   update. Recommended values are between 3 and 7. Default is 5.
    /// </summary>
    public const int CORRECTION=3;
    /// <summary>
    /// Max Inner LoopCount
    /// Looks we dont need that ,we controll loop outside
    /// </summary>
    public const int MAXLOOPCOUNT = 128;
    /// <summary>
    /// Min gradient step of BFGS
    /// it should less than STEP_MIN.
    /// </summary>
    public const float EPSILION = STEP_MIN;

    private const float loss_TOLERENCE =0f;
    private const float xTolerance = 1e-20f; // machine precision
    private const float STEP_MIN =1e-5f;
    private const float STEP_MAX =2f;
    private const float RANGLE_MIN = -1f;
    private const float RANGE_MAX = 1f;
    #endregion

    #region  PublicFunc
    public static void InitializeLoop(
ref float innerLoopStep,
ref int iterations, ref int evaluations, ref int loopCount, ref int point, ref int matrixPoint,
ref bool isLoopOutside,
ref NativeArray<float> gradient, ref NativeArray<float> diagonal, ref NativeArray<float> steps
)
    {
        // Initialization
        InitializeBeforeLoop(ref innerLoopStep, ref iterations, ref loopCount, ref point, ref matrixPoint, ref isLoopOutside);

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
        float gnormInit = Euclidean(gradient);
        innerLoopStep = 1.0f / gnormInit;
    }


    public static void OutsideLoopHead(
ref float width, ref float width1, ref float stepBoundX, ref float stepBoundY, ref float preGradientSum, ref float innerLoopStep, ref float preloss, ref float loss, ref float lossX, ref float lossY, ref float gradientInitialX, ref float gradientInitialY,
ref int funcState,  ref int iterations, ref int matrixPoint, ref int numberOfVariables, ref int point,
ref bool isLoopOutside, ref bool isLoopInside, ref bool isInBracket, ref bool stage1,
ref NativeArray<float> delta, ref NativeArray<float> steps, ref NativeArray<float> diagonal, ref NativeArray<float> gradientStore, ref NativeArray<float> gradient, ref NativeArray<float> rho, ref NativeArray<float> alpha,
ref NativeArray<float> currentSolution)
    {
        InitializeOutsideLoop(ref width, ref width1, ref stepBoundX, ref stepBoundY, ref preGradientSum, ref funcState,  ref isLoopInside, ref isInBracket, ref stage1);
        iterations++;
        if (iterations != 1)
        {
            float sumY = GetSumY(delta, steps, matrixPoint, numberOfVariables);
            if (sumY == 0)//OYM：巧妙的小设计，用来重新选择收敛方向
            {
                isLoopOutside=false;
                return;
            }
            // Compute the diagonal of the Hessian
            // or use an approximation by the user.
            ComputeDiagonal(diagonal, delta, matrixPoint, numberOfVariables, sumY);

            // Compute -H*g using the formula given in:
            //   Nocedal, J. 1980, "Updating quasi-Newton matrices with limited storage",
            //   Mathematics of Computation, Vol.24, No.151, pp. 773-782.
            int bound = math.min(iterations - 1, CORRECTION);
            for (int i = 0; i < numberOfVariables; i++)
            {
                gradientStore[i] = -gradient[i];
            }
            UpdatingQuasi_Newton(diagonal, rho, gradientStore, steps, alpha, delta, numberOfVariables, CORRECTION, point, bound, sumY);

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

        if (innerLoopStep <= 0)
        {
            isLoopOutside = false;
            return;
            //OYM：Invalid step size
        }

        // Compute the initial gradient in the search direction
        // and check that s is a descent direction.
        preGradientSum = GetGreadientSum(gradient, steps, matrixPoint, numberOfVariables);

        if (preGradientSum >= 0)//OYM：梯度爆炸
        {
            isLoopOutside = false;
            return;
            // throw new LineSearchFailedException(0, "The search direction is not a descent direction.");
        }
        // safe loss and gradient
        preloss = loss;
        UnsafeUtility.MemCpy(diagonal.GetUnsafePtr(), currentSolution.GetUnsafePtr(), numberOfVariables * UnsafeUtility.SizeOf<float>());

        // The variables stx, fx, dgx contain the values of the
        // step, function, and directional derivative at the best
        // step.
        // The variables sty, fy, dgy contain the value of the
        // step, function, and derivative at the other endpoint
        // of the interval of uncertainty.
        lossX = lossY = preloss;
        gradientInitialX = gradientInitialY = preGradientSum;

    }

    public static bool InsideLoopHead(
    ref float stepBoundMin, ref float stepBoundMax, ref float stepBoundX, ref float stepBoundY, ref float innerLoopStep,
    ref int loopCount,  ref int numberOfVariables, ref int funcState,ref int matrixPoint, ref int leastLoopCount,
    ref bool isInBracket,
    ref NativeArray<float> currentSolution, ref NativeArray<float> diagonal, ref NativeArray<float> steps
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
            (loopCount >= MAXLOOPCOUNT - 1) ||(leastLoopCount<=1)|| (funcState == 0))
        {
            innerLoopStep = stepBoundX;
        }

        // Force the step to be within the bounds stpmax and stpmin.
        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        // We return to main program to obtain F and G.
        for (int j = 0; j < numberOfVariables; j++)
        {
            currentSolution[j] = diagonal[j] + innerLoopStep * steps[matrixPoint+ j];
            currentSolution[j] = math.clamp(currentSolution[j], RANGLE_MIN, RANGE_MAX);
        }
        return true;
    }

    public static void InisdeLoopTail(
        ref float preGradientSum, ref float preloss, ref float innerLoopStep, ref float stepBoundMin, ref float stepBoundMax, ref float loss, ref float lossTolerance, ref float lossX, ref float lossY, ref float stepBoundX, ref float stepBoundY, ref float gradientInitialX, ref float gradientInitialY, ref float width, ref float width1,
        ref int loopCount, ref int numberOfVariables, ref int funcState, ref int matrixPoint,ref int leastLoopCount,
        ref bool isLoopOutside, ref bool isLoopInside, ref bool isInBracket, ref bool stage1,
ref NativeArray<float> gradient, ref NativeArray<float> steps
        )
    {
        loopCount++;
        float gradientTemp = GetGreadientSum(gradient
            , steps, matrixPoint, numberOfVariables);

        float gradientTest = loss_TOLERENCE * preGradientSum;
        float losstest1 = preloss + innerLoopStep * gradientTest;



        // successful
        if (loss <= losstest1 &&(
            math.abs(gradientTemp) <= lossTolerance * (-preGradientSum)|| 
            (loopCount >= L_BFGSStatic.MAXLOOPCOUNT || leastLoopCount <= 0)))
        {
            isLoopOutside = true;
            isLoopInside = false;
            return;

        }

        if(loopCount >= L_BFGSStatic.MAXLOOPCOUNT || leastLoopCount <= 0)
        {

            isLoopOutside = false;
            isLoopInside = false;
            return;
            //throw new LineSearchFailedException(3, "Maximum number of function evaluations has been reached.");
        }
        //Rounding errors prevent further progress.
        if ((isInBracket && (innerLoopStep <= stepBoundMin || innerLoopStep >= stepBoundMax)) || funcState == 0)
        {
            isLoopOutside = false;
            isLoopInside = false;
            return;
        }

        //The step size has reached the upper bound.
        if (innerLoopStep == STEP_MAX && loss <= losstest1 && gradientTemp <= gradientTest)
        {
            //OYM：answer out the max
            isLoopOutside = false;
            isLoopInside = false;
            return;
        }

        //The step size has reached the lower bound.
        //OYM：主要就是这个地方作妖
        if (innerLoopStep == STEP_MIN && (loss >= losstest1 || gradientTemp >= gradientTest))
        {

            isLoopOutside = false;
            isLoopInside = false;
            return;
        } 

        //Relative width of the interval of uncertainty is at machine precision
        if (isInBracket && stepBoundMax - stepBoundMin <= xTolerance * stepBoundMax)
        {
            isLoopOutside = false;
            isLoopInside = false;
            return;
        }


        // Not converged yet. Continuing with the search.
        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.
        if (stage1 &&
            loss <= losstest1 &&
            gradientTemp >= math.min(loss_TOLERENCE, lossTolerance) * preGradientSum)
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we
        // have not obtained a step for which the modified function has
        // a nonpositive function value and nonnegative derivative, and
        // if a lower function value has been obtained but the decrease
        // is not sufficient.
        if (stage1 && loss <= lossX && loss > losstest1)
        {
            // Define the modified function and derivative values.

            float fm = loss - innerLoopStep * gradientTest;
            float fxm = lossX - stepBoundX * gradientTest;
            float fym = lossY - stepBoundY * gradientTest;

            float dgm = gradientTemp - gradientTest;
            float dgxm = gradientInitialX - gradientTest;
            float dgym = gradientInitialY - gradientTest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            SearchStep(ref stepBoundX, ref fxm, ref dgxm,
                ref stepBoundY, ref fym, ref dgym, ref innerLoopStep,
                fm, dgm, ref isInBracket, out funcState);

            // Reset the function and gradient values for f.
            lossX = fxm + stepBoundX * gradientTest;
            lossY = fym + stepBoundY * gradientTest;
            gradientInitialX = dgxm + gradientTest;
            gradientInitialY = dgym + gradientTest;
        }
        else
        {
            // Call mcstep to update the interval of uncertainty
            // and to compute the new step.

            SearchStep(
                ref stepBoundX, ref lossX, ref gradientInitialX,
                ref stepBoundY, ref lossY, ref gradientInitialY,
                ref innerLoopStep,
                loss, gradientTemp, ref isInBracket, out funcState);
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
        isLoopInside = true;
    }

    public static void OutsideLoopTail(
         ref float innerLoopStep, ref float gradientTolerance,
           ref int loopCount, ref int matrixPoint, ref int point, ref int numberOfVariables,ref int leastLoopCount,
           ref bool isLoopOutside,
           ref NativeArray<float> gradient, ref NativeArray<float> steps, ref NativeArray<float> delta, ref NativeArray<float> gradientStore, ref NativeArray<float> diagonal, ref NativeArray<float> currentSolution
        )
    {
             if (!isLoopOutside)
                {
                    UnsafeUtility.MemCpy(currentSolution.GetUnsafePtr(), diagonal.GetUnsafePtr(), numberOfVariables * UnsafeUtility.SizeOf<float>());
                    return;
                }

                        // Compute the new step and
                        // new gradient differences
        for (int i = 0; i < numberOfVariables; i++)
        {
            steps[matrixPoint + i] *= innerLoopStep;
            delta[matrixPoint + i] = gradient[i] - gradientStore[i];
        }

        // point loop
        point++;
        if (point == CORRECTION)
        {
            point = 0;
        }

        // Check for termination
        float gnorm = Euclidean(gradient);
        float xnorm = Euclidean(currentSolution);
        xnorm = math.max(1f, xnorm);

        // isSuccessful
        if (gnorm/ numberOfVariables <= gradientTolerance||leastLoopCount <= 0)
        {
            isLoopOutside = false;
        }

    }
    public static void CreateWorkVector(int numberOfVariables,
    out NativeArray<float> diagonal, out NativeArray<float> gradientStore, out NativeArray<float> rho, out NativeArray<float> alpha, out NativeArray<float> steps, out NativeArray<float> delta
    )
    {
        diagonal = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        gradientStore = new NativeArray<float>(numberOfVariables, Allocator.Persistent);
        rho = new NativeArray<float>(CORRECTION, Allocator.Persistent);                  // Stores the scalars rho.
        alpha = new NativeArray<float>(CORRECTION, Allocator.Persistent);               // Stores the alphas in computation of H*g.
        steps = new NativeArray<float>(numberOfVariables * CORRECTION, Allocator.Persistent);          // Stores the last M search steps.
        delta = new NativeArray<float>(numberOfVariables * CORRECTION, Allocator.Persistent);
    }

    public static void Disposed(NativeArray<float> diagonal, NativeArray<float> gradientStore, NativeArray<float> rho, NativeArray<float> alpha, NativeArray<float> steps, NativeArray<float> delta)
    {
        diagonal.Dispose();
        gradientStore.Dispose();
        rho.Dispose();
        alpha.Dispose();
        steps.Dispose();
        delta.Dispose();
    }

    public static void ClearData(NativeArray<float> diagonal, NativeArray<float> gradientStore, NativeArray<float> rho, NativeArray<float> alpha, NativeArray<float> steps, NativeArray<float> delta)
    {
        ClearData(diagonal);
        ClearData(gradientStore);
        ClearData(rho);
        ClearData(alpha);
        ClearData(steps);
        ClearData(delta);
    }
    #endregion


    #region Line Search (mcsrch)

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

    // TODO: Move to separate classes
    internal static void SearchStep(ref float stx, ref float fx, ref float dx,
                            ref float sty, ref float fy, ref float dy,
                            ref float stp, float fp, float dp,
                            ref bool brackt, out int info)
    {
        bool bound;
        float stpc, stpf, stpq;
        float stpmax = STEP_MAX;
        float stpmin = STEP_MIN;
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


    #region PrivateFunc
    private static void ClearData<T>(NativeArray<T> data) where T : struct
    {
        UnsafeUtility.MemClear(data.GetUnsafePtr(), data.Length * UnsafeUtility.SizeOf<T>());
    }

    private static void InitializeOutsideLoop(ref float width, ref float width1, ref float stepBoundX, ref float stepBoundY, ref float preGradientSum, ref int funcState, ref bool isLoopInside, ref bool isInBracket, ref bool stage1)
    {
        width = STEP_MAX - STEP_MIN;
        width1 = width / 0.5f;
        stepBoundX = STEP_MIN;
        stepBoundY = STEP_MAX;
        preGradientSum = 0f;

        funcState = 1;

        isLoopInside = true;

        isInBracket = false;
        stage1 = true;
    }

    private static void InitializeBeforeLoop(ref float innerLoopStep, ref int iterations, ref int loopCount, ref int point, ref int matrixPoint, ref bool isLoopOutside)
    {
        innerLoopStep = 0;
        iterations = 0;
        loopCount = 0;
        point = 0;
        matrixPoint = 0;
        isLoopOutside = true;
    }
    private static float Euclidean(NativeArray<float> a)
    {
        return math.sqrt(SquareEuclidean(a));
    }
    private static float SquareEuclidean(NativeArray<float> a)
    {
        float sum = 0;
        for (int i = 0; i < a.Length; i++)
            sum += a[i] * a[i];
        return sum;

    }

    private static float GetGreadientSum(NativeArray<float> gradient, NativeArray<float> steps, int matrixPoint, int numberOfVariables)
    {
        float gradientTemp = 0;
        for (int j = 0; j < numberOfVariables; j++)
        {
            gradientTemp = gradientTemp + gradient[j] * steps[matrixPoint + j];
        }
        return gradientTemp;
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
    #endregion
}