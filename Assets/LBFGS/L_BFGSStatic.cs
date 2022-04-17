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
    public const int CORRECTION=5;
    /// <summary>
    /// Max Inner LoopCount
    /// Looks we dont need that ,we controll loop outside
    /// </summary>
    public const int MAXLOOPCOUNT=40;
    /// <summary>
    /// Min gradient step of BFGS
    /// it should less than STEP_MIN.
    /// </summary>
    public const float EPSILION = STEP_MIN / 2;

    private const float loss_TOLERENCE = 1e-3f;
    private const float xTolerance = 1e-20f; // machine precision
    private const float STEP_MIN = 1e-5f;
    private const float STEP_MAX = 180f;
    private const float RANGLE_MIN = -1;
    private const float RANGE_MAX = 1;


    #endregion

    #region  PublicFunc
    public static void InitializeLoop(
ref float innerLoopStep,
ref int iterations, ref int evaluations, ref int innerLoopCount, ref int point, ref int matrixPoint,
ref bool isLoopOutside,
ref NativeArray<float> gradient, ref NativeArray<float> diagonal, ref NativeArray<float> steps
)
    {
        // Initialization
        InitializeBeforeLoop(ref innerLoopStep, ref iterations, ref evaluations, ref innerLoopCount, ref point, ref matrixPoint, ref isLoopOutside);

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
ref int funcState, ref int innerLoopCount, ref int iterations, ref int matrixPoint, ref int numberOfVariables, ref int point,
ref bool isLoopOutside, ref bool isLoopInside, ref bool isInBracket, ref bool stage1,
ref NativeArray<float> delta, ref NativeArray<float> steps, ref NativeArray<float> diagonal, ref NativeArray<float> gradientStore, ref NativeArray<float> gradient, ref NativeArray<float> rho, ref NativeArray<float> alpha,
ref NativeArray<float> currentSolution)
    {
        InitializeOutsideLoop(ref width, ref width1, ref stepBoundX, ref stepBoundY, ref preGradientSum, ref funcState, ref innerLoopCount, ref isLoopInside, ref isInBracket, ref stage1);
        iterations++;
        if (iterations != 1)
        {
            float sumY = GetSumY(delta, steps, matrixPoint, numberOfVariables);//OYM：这个地方等于0？
            if (sumY==0)
            {
                isLoopOutside = false;
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
    ref int innerLoopCount,  ref int numberOfVariables, ref int funcState,ref int matrixPoint,
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
            (innerLoopCount >= MAXLOOPCOUNT - 1) || (funcState == 0))
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
        ref int innerLoopCount, ref int numberOfVariables, ref int funcState, ref int matrixPoint,
        ref bool isLoopOutside, ref bool isLoopInside, ref bool isInBracket, ref bool stage1,
ref NativeArray<float> gradient, ref NativeArray<float> steps
        )
    {
        innerLoopCount++;
        float gradientTemp = GetGreadientSum(gradient
            , steps, matrixPoint, numberOfVariables);

        float gradientTest = loss_TOLERENCE * preGradientSum;
        float losstest1 = preloss + innerLoopStep * gradientTest;

        // Test for convergence.
        if (innerLoopCount >= MAXLOOPCOUNT)
        {
            isLoopOutside = false;
            isLoopInside= false;
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

        // successful
        if (loss <= losstest1 && math.abs(gradientTemp) <= lossTolerance * (-preGradientSum))
        {
            isLoopOutside = true;
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

    public static bool OutsideLoopTail(
         ref float innerLoopStep, ref float gradientTolerance,
           ref int innerLoopCount, ref int evaluations, ref int matrixPoint, ref int point, ref int numberOfVariables,
           ref bool isLoopOutside,
           ref NativeArray<float> gradient, ref NativeArray<float> steps, ref NativeArray<float> delta, ref NativeArray<float> gradientStore, ref NativeArray<float> currentSolution
        )
    {
        // Register evaluations
        evaluations += innerLoopCount;

        // Compute the new step and
        // new gradient differences
        bool isZero = true;
        for (int i = 0; i < numberOfVariables; i++)
        {
            steps[matrixPoint + i] *= innerLoopStep;
            delta[matrixPoint + i] = gradient[i] - gradientStore[i];
            if (delta[matrixPoint + i]!=0)
            {
                isZero = false;
            }

        }
        if (isZero)
        {

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
        if (gnorm / xnorm <= gradientTolerance)
        {
            isLoopOutside = false;
        }
        return true;
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
    private static void SearchStep(ref float stepBoundX, ref float lossX, ref float gradientX,
                                ref float stepBoundY, ref float lossY, ref float gradientY,
                                ref float step, float lossTemp, float gradientTemp,
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

        if (lossTemp > lossX)//OYM: 可以被优化？
        {
            // First case. A higher function value.
            // The minimum is bracketed. If the cubic step is closer
            // to stx than the quadratic step, the cubic step is taken,
            // else the average of the cubic and quadratic steps is taken.

            funcState = 1;
            bound = true;
            float theta = 3.0f * (lossX - lossTemp) / (step - stepBoundX) + gradientX + gradientTemp;
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
            stpq = stepBoundX + ((gradientX / ((lossX - lossTemp) / (step - stepBoundX) + gradientX)) / 2) * (step - stepBoundX);

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
            float theta = 3 * (lossX - lossTemp) / (step - stepBoundX) + gradientX + gradientTemp;
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
            float theta = 3 * (lossX - lossTemp) / (step - stepBoundX) + gradientX + gradientTemp;
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
                float theta = 3 * (lossTemp - lossY) / (stepBoundY - step) + gradientY + gradientTemp;
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

        if (lossTemp > lossX)
        {
            stepBoundY = step;
            lossY = lossTemp;
            gradientY = gradientTemp;
        }
        else
        {
            if (signDerivatives < 0.0)
            {
                stepBoundY = stepBoundX;
                lossY = lossX;
                gradientY = gradientX;
            }
            stepBoundX = step;
            lossX = lossTemp;
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


    #region PrivateFunc
    private static void ClearData<T>(NativeArray<T> data) where T : struct
    {
        UnsafeUtility.MemClear(data.GetUnsafePtr(), data.Length * UnsafeUtility.SizeOf<T>());
    }

    private static void InitializeOutsideLoop(ref float width, ref float width1, ref float stepBoundX, ref float stepBoundY, ref float preGradientSum, ref int funcState, ref int innerLoopCount, ref bool isLoopInside, ref bool isInBracket, ref bool stage1)
    {
        width = STEP_MAX - STEP_MIN;
        width1 = width / 0.5f;
        stepBoundX = 0f;
        stepBoundY = 0f;
        preGradientSum = 0f;

        funcState = 1;
        innerLoopCount = 0;

        isLoopInside = true;

        isInBracket = false;
        stage1 = true;
    }

    private static void InitializeBeforeLoop(ref float innerLoopStep, ref int iterations, ref int evaluations, ref int innerLoopCount, ref int point, ref int matrixPoint, ref bool isLoopOutside)
    {
        innerLoopStep = 0;
        iterations = 0;
        evaluations = 1;
        innerLoopCount = 0;
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