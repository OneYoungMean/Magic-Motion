using UnityEngine;

namespace Grounding
{

    public partial class GroundingIK
    {
        public static Vector3 LineToPlane(Vector3 origin, Vector3 direction, Vector3 planeNormal, Vector3 planePoint) //OYM:点与平面相交的交点
        {
            float dot = Vector3.Dot(planePoint - origin, planeNormal);
            float normalDot = Vector3.Dot(direction, planeNormal);

            if (normalDot == 0.0f) return Vector3.zero;

            float dist = dot / normalDot;
            return origin + direction.normalized * dist;
        }
        public static float LerpValue(float value, float target, float increaseSpeed, float decreaseSpeed) //OYM:线性差值
        {
            if (value == target) return target;
            if (value < target) return Mathf.Clamp(value + Time.deltaTime * increaseSpeed, -Mathf.Infinity, target);
            else return Mathf.Clamp(value - Time.deltaTime * decreaseSpeed, target, Mathf.Infinity);
        }
    }
    public partial class GroundingIK
    {

        /// <summary>
        /// The %Grounding %Leg.
        /// </summary>
        public class Leg
        {

            /// <summary>
            /// Returns true distance from foot to ground is less that maxStep
            /// </summary>
            public bool isGrounded { get; private set; } //OYM:是否着地
            /// <summary>
            /// Gets the current IK position of the foot.
            /// </summary>
            public Vector3 IKPosition { get; private set; } //OYM:IK位置
            /// <summary>
            /// Gets the current rotation offset of the foot.
            /// </summary>
            public Quaternion rotationOffset = Quaternion.identity;
            /// <summary>
            /// Returns true, if the leg is valid and initiated
            /// </summary>
            public bool initiated { get; private set; }
            /// <summary>
            /// The height of foot from ground.
            /// </summary>
            public float heightFromGround { get; private set; }
            /// <summary>
            /// Velocity of the foot
            /// </summary>
            public Vector3 velocity { get; private set; }
            /// <summary>
            /// Gets the foot Transform.
            /// </summary>
            public Transform footTransform { get; private set; } //OYM:这里是脚的速度,划重点 
            /// <summary>  
            /// Gets the current IK offset.
            /// </summary>
            public float IKOffset { get; private set; }

            public bool invertFootCenter;

            public RaycastHit heelHit { get; private set; }
            public RaycastHit capsuleHit { get; private set; }

            /// <summary>
            /// Gets the RaycastHit last used by the Grounder to get ground height at foot position.
            /// </summary>
            public RaycastHit GetHitPoint
            {
                get
                { return capsuleHit; }
            }

            /// <summary>
            /// Overrides the animated position of the foot.
            /// </summary>
            public void SetFootPosition(Vector3 position) //OYM:覆盖脚的位置
            {
                doOverrideFootPosition = true;
                overrideFootPosition = position;
            }

            private GroundingIK grounding; //OYM:
            private float lastTime, deltaTime; //OYM:最后的时间与单位时间
            private Vector3 lastPosition;
            private Quaternion toHitNormal, r;
            private Vector3 up = Vector3.up;
            private bool doOverrideFootPosition;
            private Vector3 overrideFootPosition;
            private Vector3 transformPosition;

            // Initiates the Leg
            public void Initiate(GroundingIK grounding, Transform transform) //OYM:初始化
            {
                initiated = false;
                this.grounding = grounding;
                this.footTransform = transform;
                up = Vector3.up; //OYM:向上
                IKPosition = transform.position; //OYM:
                rotationOffset = Quaternion.identity;

                initiated = true;
                OnEnable();
            }

            // Should be called each time the leg is (re)activated
            public void OnEnable()
            {
                if (!initiated) return;

                lastPosition = footTransform.position; //OYM:上一次的位置
                lastTime = Time.deltaTime; //OYM:上一次的时间
            }

            // Set everything to 0
            public void Reset()
            {
                lastPosition = footTransform.position; //OYM:重置
                lastTime = Time.deltaTime;
                IKOffset = 0f;
                IKPosition = footTransform.position;
                rotationOffset = Quaternion.identity;
            }

            // Raycasting, processing the leg's position
            public void Process() //OYM:处理
            {
                if (!initiated) return;
                if (grounding.maxStep <= 0) return; //OYM:接地测试大于最大的解

                transformPosition = doOverrideFootPosition ? overrideFootPosition : footTransform.position;//OYM:当前脚的位置
                doOverrideFootPosition = false;

                deltaTime = Time.time - lastTime; //OYM:一帧的时间
                lastTime = Time.time;
                if (deltaTime == 0f) return;

                up = grounding.up;
                heightFromGround = Mathf.Infinity;

                // Calculating velocity
                velocity = (transformPosition - lastPosition) / deltaTime; //OYM:获取角的移动速度
                                                                           //velocity = grounding.Flatten(velocity);
                lastPosition = transformPosition;

                Vector3 prediction = velocity * grounding.prediction; //OYM:预测

                if (grounding.footRadius <= 0) return;

                isGrounded = false;

                // Raycasting
                heelHit = GetRaycastHit(invertFootCenter ? -grounding.GetFootCenterOffset() : Vector3.zero);
                capsuleHit = GetCapsuleHit(prediction);

                if (heelHit.collider != null || capsuleHit.collider != null) isGrounded = true;

                SetFootToPlane(capsuleHit.normal, capsuleHit.point, heelHit.point);

                float offsetTarget = stepHeightFromGround;
                if (!grounding.rootGrounded) offsetTarget = 0f;

                IKOffset = LerpValue(IKOffset, offsetTarget, grounding.footSpeed, grounding.footSpeed);
                IKOffset = Mathf.Lerp(IKOffset, offsetTarget, deltaTime * grounding.footSpeed);

                float legHeight = grounding.GetVerticalOffset(transformPosition, grounding.root.position);
                float currentMaxOffset = Mathf.Clamp(grounding.maxStep - legHeight, 0f, grounding.maxStep);

                IKOffset = Mathf.Clamp(IKOffset, -currentMaxOffset, IKOffset);

                // Getting the full target rotation
                Quaternion rotationOffsetTarget = GetRotationOffsetTarget();

                // Slerping the rotation offset
                r = Quaternion.Slerp(r, rotationOffsetTarget, deltaTime * grounding.footRotationSpeed);

                // Update IK values
                IKPosition = transformPosition - up * IKOffset;
                Debug.DrawLine(IKPosition - Vector3.forward * 0.1f, IKPosition + Vector3.forward * 0.1f);
                Debug.DrawLine(IKPosition - Vector3.up * 0.1f, IKPosition + Vector3.up * 0.1f);
                Debug.DrawLine(IKPosition - Vector3.left * 0.1f, IKPosition + Vector3.left * 0.1f);
                float rW = grounding.footRotationWeight;
                rotationOffset = rW >= 1 ? r : Quaternion.Slerp(Quaternion.identity, r, rW);
            }

            // Gets the height from ground clamped between min and max step height
            public float stepHeightFromGround
            {
                get
                {
                    return Mathf.Clamp(heightFromGround, -grounding.maxStep, grounding.maxStep);
                }
            }

            // Get predicted Capsule hit from the middle of the foot
            private RaycastHit GetCapsuleHit(Vector3 offsetFromHeel)
            {
                RaycastHit hit = new RaycastHit();
                Vector3 f = grounding.GetFootCenterOffset();
                if (invertFootCenter) f = -f;
                Vector3 origin = transformPosition + f;

                if (grounding.overstepFallsDown)
                {
                    hit.point = origin - up * grounding.maxStep;
                }
                else
                {
                    hit.point = new Vector3(origin.x, grounding.root.position.y, origin.z);
                }
                hit.normal = up;

                // Start point of the capsule
                Vector3 capsuleStart = origin + grounding.maxStep * up;
                // End point of the capsule depending on the foot's velocity.
                Vector3 capsuleEnd = capsuleStart + offsetFromHeel;

                if (Physics.CapsuleCast(capsuleStart, capsuleEnd, grounding.footRadius, -up, out hit, grounding.maxStep * 2, grounding.layers, QueryTriggerInteraction.Ignore))
                {
                    // Safeguarding from a CapsuleCast bug in Unity that might cause it to return NaN for hit.point when cast against large colliders.
                    if (float.IsNaN(hit.point.x))
                    {
                        hit.point = origin - up * grounding.maxStep * 2f;
                        hit.normal = up;
                    }
                }

                // Since Unity2017 Raycasts will return Vector3.zero when starting from inside a collider
                if (hit.point == Vector3.zero && hit.normal == Vector3.zero)
                {
                    if (grounding.overstepFallsDown)
                    {
                        hit.point = origin - up * grounding.maxStep;
                    }
                    else
                    {
                        hit.point = new Vector3(origin.x, grounding.root.position.y, origin.z);
                    }
                }

                return hit;
            }

            // Get simple Raycast from the heel
            private RaycastHit GetRaycastHit(Vector3 offsetFromHeel) //OYM:获取脚跟的ray
            {
                RaycastHit hit = new RaycastHit();
                Vector3 origin = transformPosition + offsetFromHeel; //OYM:获取ray的原点

                if (grounding.overstepFallsDown) //OYM:如果不允许双脚悬空的情况
                {
                    hit.point = origin - up * grounding.maxStep; //OYM:point点向下探
                }
                else
                {
                    hit.point = new Vector3(origin.x, grounding.root.position.y, origin.z);
                }
                hit.normal = up;

                if (grounding.maxStep <= 0f) return hit;//OYM:不射,就是0

                Physics.Raycast(origin + grounding.maxStep * up, -up, out hit, grounding.maxStep * 2, grounding.layers, QueryTriggerInteraction.Ignore);
                //OYM:这句话大概意是这样的
                //OYM:首先,把origin 向上移动maxStep的长度,然后向着下射出maxStepx2的长度,简单来说就是对称的射出一个射线

                // Since Unity2017 Raycasts will return Vector3.zero when starting from inside a collider
                if (hit.point == Vector3.zero && hit.normal == Vector3.zero) //OYM:没有射到
                {
                    if (grounding.overstepFallsDown) //OYM:如果没有探测到,且需要向下探的话,则继续往下延伸
                    {
                        hit.point = origin - up * grounding.maxStep;
                    }
                    else
                    {
                        hit.point = new Vector3(origin.x, grounding.root.position.y, origin.z);//OYM: 否则获取当前脚的位置与root节点的y值作为长度 
                    }
                }

                return hit;
            }

            // Set foot height from ground relative to a plane
            private void SetFootToPlane(Vector3 planeNormal, Vector3 planePoint, Vector3 heelHitPoint)
            {
                toHitNormal = Quaternion.FromToRotation(up, planeNormal);

                Vector3 pointOnPlane = LineToPlane(transformPosition + up * grounding.maxStep, -up, planeNormal, planePoint); //OYM计算脚的位置

                // Get the height offset of the point on the plane
                heightFromGround = GetHeightFromGround(pointOnPlane);

                // Making sure the heel doesn't penetrate the ground
                float heelHeight = GetHeightFromGround(heelHitPoint);
                heightFromGround = Mathf.Clamp(heightFromGround, -Mathf.Infinity, heelHeight);
            }

            // Calculate height offset of a point
            private float GetHeightFromGround(Vector3 hitPoint)
            {
                return grounding.GetVerticalOffset(transformPosition, hitPoint) - rootYOffset;
            }

            // Gets the target hit normal offset as a Quaternion
            private Quaternion GetRotationOffsetTarget()
            {
                if (grounding.maxFootRotationAngle <= 0f) return Quaternion.identity;
                if (grounding.maxFootRotationAngle >= 180f) return toHitNormal;
                return Quaternion.RotateTowards(Quaternion.identity, toHitNormal, grounding.maxFootRotationAngle);
            }

            // The foot's height from ground in the animation
            private float rootYOffset
            {
                get
                {
                    return grounding.GetVerticalOffset(transformPosition, grounding.root.position - up * grounding.heightOffset);//OYM:当前脚的位置到root与高度偏移的位置
                }
            }
        }
    }

    public partial class GroundingIK : MonoBehaviour
    {

        /// <summary>
        /// Layers to ground the character to. Make sure to exclude the layer of the character controller.
        /// </summary>
        [Tooltip("Layers to ground the character to. Make sure to exclude the layer of the character controller.")]
        public LayerMask layers;
        /// <summary>
        /// Max step height. Maximum vertical distance of Grounding from the root of the character.
        /// </summary>
        [Tooltip("Max step height. Maximum vertical distance of Grounding from the root of the character.")]
        public float maxStep = 0.5f;
        /// <summary>
        /// The height offset of the root.
        /// </summary>
        [Tooltip("The height offset of the root.")]
        public float heightOffset;
        /// <summary>
        /// The speed of moving the feet up/down.
        /// </summary>
        [Tooltip("The speed of moving the feet up/down.")]
        public float footSpeed = 2.5f;
        /// <summary>
        /// CapsuleCast radius. Should match approximately with the size of the feet.
        /// </summary>
        [Tooltip("CapsuleCast radius. Should match approximately with the size of the feet.")]
        public float footRadius = 0.15f;
        /// <summary>
        /// Offset of the foot center along character forward axis.
        /// </summary>
        [Tooltip("Offset of the foot center along character forward axis.")]
        [HideInInspector] public float footCenterOffset; // TODO make visible in inspector if Grounder Visualization is finished.
        /// <summary>
        /// Amount of velocity based prediction of the foot positions.
        /// </summary>
        [Tooltip("Amount of velocity based prediction of the foot positions.")]
        public float prediction = 0.05f;
        /// <summary>
        /// Weight of rotating the feet to the ground normal offset.
        /// </summary>
        [Tooltip("Weight of rotating the feet to the ground normal offset.")]
        [Range(0f, 1f)]
        public float footRotationWeight = 1f;
        /// <summary>
        /// Speed of slerping the feet to their grounded rotations.
        /// </summary>
        [Tooltip("Speed of slerping the feet to their grounded rotations.")]
        public float footRotationSpeed = 7f;
        /// <summary>
        /// Max Foot Rotation Angle, Max angular offset from the foot's rotation (Reasonable range: 0-90 degrees).
        /// </summary>
        [Tooltip("Max Foot Rotation Angle. Max angular offset from the foot's rotation.")]
        [Range(0f, 90f)]
        public float maxFootRotationAngle = 45f;
        /// <summary>
        /// If true, solver will rotate with the character root so the character can be grounded for example to spherical planets. 
        /// For performance reasons leave this off unless needed.
        /// </summary>
        [Tooltip("If true, solver will rotate with the character root so the character can be grounded for example to spherical planets. For performance reasons leave this off unless needed.")]
        public bool rotateSolver;
        /// <summary>
        /// The speed of moving the character up/down.
        /// </summary>
        [Tooltip("The speed of moving the character up/down.")]
        public float pelvisSpeed = 5f;
        /// <summary>
        /// Used for smoothing out vertical pelvis movement (range 0 - 1).
        /// </summary>
        [Tooltip("Used for smoothing out vertical pelvis movement (range 0 - 1).")]
        [Range(0f, 1f)]
        public float pelvisDamper;
        /// <summary>
        /// The weight of lowering the pelvis to the lowest foot.
        /// </summary>
        [Tooltip("The weight of lowering the pelvis to the lowest foot.")]
        public float lowerPelvisWeight = 1f;
        /// <summary>
        /// The weight of lifting the pelvis to the highest foot. This is useful when you don't want the feet to go too high relative to the body when crouching.
        /// </summary>
        [Tooltip("The weight of lifting the pelvis to the highest foot. This is useful when you don't want the feet to go too high relative to the body when crouching.")]
        public float liftPelvisWeight;
        /// <summary>
        /// The radius of the spherecast from the root that determines whether the character root is grounded.
        /// </summary>
        [Tooltip("The radius of the spherecast from the root that determines whether the character root is grounded.")]
        public float rootSphereCastRadius = 0.1f;
        /// <summary>
        /// If false, keeps the foot that is over a ledge at the root level. If true, lowers the overstepping foot and body by the 'Max Step' value.
        /// </summary>
        [Tooltip("If false, keeps the foot that is over a ledge at the root level. If true, lowers the overstepping foot and body by the 'Max Step' value.")]
        public bool overstepFallsDown = true;//OYM:如果为非,则会一脚悬空
        /// <summary>
        /// The leg Transform
        /// </summary>
        public Transform[] legTramsform;

        /// <summary>
        /// The %Grounding legs.
        /// </summary>
        public Leg[] legs { get; private set; }

        /// <summary>
        /// Gets a value indicating whether any of the legs are grounded
        /// </summary>
        public bool isGrounded { get; private set; }
        /// <summary>
        /// The root Transform
        /// </summary>
        public Transform root { get; private set; }



        /// <summary>
        /// Ground height at the root position.
        /// </summary>
        public RaycastHit rootHit { get; private set; }
        /// <summary>
        /// Is the RaycastHit from the root grounded?
        /// </summary>
        public bool rootGrounded
        {
            get
            {
                return rootHit.distance < maxStep * 2f;
            }
        }
        private void Start()
        {
            root = transform;

            Initiate(root, legTramsform);

        }

        /// <summary>
        /// Updates the Grounding.
        /// </summary>
        public void Update()
        {
            if (!initiated) return;

            if (layers == 0) Debug.Log("Grounding layers are set to nothing. Please add a ground layer.");

            maxStep = Mathf.Clamp(maxStep, 0f, maxStep);
            footRadius = Mathf.Clamp(footRadius, 0.0001f, maxStep);
            pelvisDamper = Mathf.Clamp(pelvisDamper, 0f, 1f);
            rootSphereCastRadius = Mathf.Clamp(rootSphereCastRadius, 0.0001f, rootSphereCastRadius);
            maxFootRotationAngle = Mathf.Clamp(maxFootRotationAngle, 0f, 90f);
            prediction = Mathf.Clamp(prediction, 0f, prediction);
            footSpeed = Mathf.Clamp(footSpeed, 0f, footSpeed);

            isGrounded = false;

            // Process legs
            foreach (Leg leg in legs)
            {
                leg.Process();

                if (leg.isGrounded) isGrounded = true;
            }

        }

        /// <summary>
        /// Gets a value indicating whether this <see cref="GroundingIK"/> is valid.
        /// </summary>
        public bool IsValid(ref string errorMessage)
        {
            if (root == null)
            {
                errorMessage = "Root transform is null. Can't initiate Grounding.";
                return false;
            }
            if (legs == null)
            {
                errorMessage = "Grounding legs is null. Can't initiate Grounding.";
                return false;
            }


            if (legs.Length == 0)
            {
                errorMessage = "Grounding has 0 legs. Can't initiate Grounding.";
                return false;
            }
            return true;
        }

        /// <summary>
        /// Initiate the %Grounding as an integrated solver by providing the root Transform, leg solvers, pelvis Transform and spine solver.
        /// </summary>
        public void Initiate(Transform root, Transform[] feet)
        {
            this.root = root;
            initiated = false;

            rootHit = new RaycastHit();

            // Constructing Legs
            if (legs == null) legs = new Leg[feet.Length];
            if (legs.Length != feet.Length) legs = new Leg[feet.Length];
            for (int i = 0; i < feet.Length; i++) if (legs[i] == null) legs[i] = new Leg();



            string errorMessage = string.Empty;
            if (!IsValid(ref errorMessage))
            {
                Debug.Log(errorMessage);
                return;
            }

            // Initiate solvers only if application is playing
            if (Application.isPlaying)
            {
                for (int i = 0; i < feet.Length; i++) legs[i].Initiate(this, feet[i]);
                initiated = true;
            }
        }

        // Set everything to 0
        public void Reset()
        {
            if (!Application.isPlaying) return;
            foreach (Leg leg in legs) leg.Reset();
        }


        private bool initiated;


        // The up vector in solver rotation space.
        public Vector3 up
        {
            get
            {
                return (useRootRotation ? root.up : Vector3.up);
            }
        }

        // Gets the vertical offset between two vectors in solver rotation space
        public float GetVerticalOffset(Vector3 p1, Vector3 p2)
        {
            if (useRootRotation)
            {
                Vector3 v = Quaternion.Inverse(root.rotation) * (p1 - p2);
                return v.y;
            }

            return p1.y - p2.y;
        }

        // Determines whether to use root rotation as solver rotation
        private bool useRootRotation
        {
            get
            {
                if (!rotateSolver) return false;
                if (root.up == Vector3.up) return false;
                return true;
            }
        }

        public Vector3 GetFootCenterOffset()
        {
            return root.forward * footRadius + root.forward * footCenterOffset;
        }
    }
}

