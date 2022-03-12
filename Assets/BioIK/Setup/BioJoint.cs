
using UnityEngine;

namespace BIOIK {

	[AddComponentMenu("")]
	public class BioJoint : MonoBehaviour {
		public BioSegment Segment;
		public Motion X,Y,Z;
		public JointType JointType = JointType.Rotational;							//Type of the joint

		[SerializeField] private Vector3 Anchor = Vector3.zero;						//Joint anchor  position ,锚点
		[SerializeField] private Vector3 Orientation = Vector3.zero;				//Joint orientation		 朝向，
		[SerializeField] private float DPX, DPY, DPZ, DRX, DRY, DRZ, DRW;			//Default frame 这里是transform的position和rotation       

		private Vector3 AnimationPosition, AnimatedDefaultPosition;
		private Quaternion AnimationRotation, AnimatedDefaultRotation;
		private double R1, R2, R3, R4, R5, R6, R7, R8, R9;							//Precomputed rotation information ，这里其实是一个旋转矩阵
		private Vector3 LSA;														//LocalScaledAnchor
		private Vector3 ADPADRSA;													//AnimatedDefaultPosition + AnimatedDefaultRotation * LocalScaledAnchor
		private Vector3 LastPosition;
		private Quaternion LastRotation;
		private Vector3 Scale;

		void Awake() {

		}

		void Start() {
			LastPosition = transform.localPosition;
			LastRotation = transform.localRotation;
		}

		void OnEnable() {
			LastPosition = transform.localPosition;
			LastRotation = transform.localRotation;
			if(Segment != null) {
				Segment.Character.Refresh();
			}
		}

		void OnDisable() {
			Segment.Character.Refresh();
		}

		void OnDestroy() {

		}

		public BioJoint Create(BioSegment segment) {
			Segment = segment;
			Segment.Transform.hideFlags = HideFlags.NotEditable;
			hideFlags = HideFlags.HideInInspector;

			X = new Motion(this, Vector3.right);
			Y = new Motion(this, Vector3.up);
			Z = new Motion(this, Vector3.forward);

			SetDefaultFrame(transform.localPosition, transform.localRotation);
			SetAnchor(Anchor);

			Vector3 forward = Vector3.zero;
			if(Segment.Childs.Length == 1) {
				forward = Segment.Childs[0].Transform.localPosition;
			} else if(Segment.Parent != null) {
				forward = Quaternion.Inverse(Segment.Transform.localRotation) * Segment.Transform.localPosition;
			}
			if(forward.magnitude != 0f) {
				SetOrientation(Quaternion.LookRotation(forward, Vector3.up).eulerAngles);
			} else {
				SetOrientation(Orientation);
			}

			LastPosition = transform.localPosition;
			LastRotation = transform.localRotation;

			return this;
		}

		public void Remove() {
			RestoreDefaultFrame();
			Segment.Transform.hideFlags = HideFlags.None;
			if(Segment != null) {
				Segment.Joint = null;
				if(Segment.Character != null) {
					Segment.Character.Refresh();
				}
			}
			Utility.Destroy(this);
		}

		public void Erase() {
			RestoreDefaultFrame();
			Segment.Transform.hideFlags = HideFlags.None;
			Utility.Destroy(this);
		}

		public void PrecaptureAnimation() {
			transform.hasChanged = false;
		}

		public void PostcaptureAnimation() {
			if(transform.hasChanged) {
				AnimationPosition = transform.localPosition;
				AnimationRotation = transform.localRotation;
			} else {
				AnimationPosition = new Vector3(DPX, DPY, DPZ);
				AnimationRotation = new Quaternion(DRX, DRY, DRZ, DRW);
			}
		}

		public void UpdateData() {
			AnimatedDefaultPosition = (1f-Segment.Character.AnimationWeight) * new Vector3(DPX, DPY, DPZ) + Segment.Character.AnimationWeight * AnimationPosition;
			AnimatedDefaultRotation = Quaternion.Slerp(new Quaternion(DRX, DRY, DRZ, DRW), AnimationRotation, Segment.Character.AnimationWeight);
            //OYM:更新旋转矩阵
            R1 = (1.0 - 2.0 * (AnimatedDefaultRotation.y * AnimatedDefaultRotation.y + AnimatedDefaultRotation.z * AnimatedDefaultRotation.z));
			R2 = 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.y + AnimatedDefaultRotation.w * AnimatedDefaultRotation.z);
			R3 = 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.z - AnimatedDefaultRotation.w * AnimatedDefaultRotation.y);
			R4 = 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.y - AnimatedDefaultRotation.w * AnimatedDefaultRotation.z);
			R5 = (1.0 - 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.x + AnimatedDefaultRotation.z * AnimatedDefaultRotation.z));
			R6 = 2.0 * (AnimatedDefaultRotation.y * AnimatedDefaultRotation.z + AnimatedDefaultRotation.w * AnimatedDefaultRotation.x);
			R7 = 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.z + AnimatedDefaultRotation.w * AnimatedDefaultRotation.y);
			R8 = 2.0 * (AnimatedDefaultRotation.y * AnimatedDefaultRotation.z - AnimatedDefaultRotation.w * AnimatedDefaultRotation.x);
			R9 = (1.0 - 2.0 * (AnimatedDefaultRotation.x * AnimatedDefaultRotation.x + AnimatedDefaultRotation.y * AnimatedDefaultRotation.y));
            //OYM:更新本地大小
            LSA = Vector3.Scale(Anchor, transform.localScale);//OYM:本地大小变化导致锚点变化
            //OYM:更新
            ADPADRSA = AnimatedDefaultPosition + AnimatedDefaultRotation * LSA; //OYM: 纠正后的本地锚点
            Scale = transform.lossyScale;
		}

		public void ProcessMotion() {
			if(!enabled) {
				return;
			}

			//Compute local transformation
			double lpX, lpY, lpZ, lrX, lrY, lrZ, lrW;
			if(JointType == JointType.Rotational) {
				ComputeLocalTransformation(Utility.Deg2Rad*X.ProcessMotion(Segment.Character.MotionType), Utility.Deg2Rad*Y.ProcessMotion(Segment.Character.MotionType), Utility.Deg2Rad*Z.ProcessMotion(Segment.Character.MotionType), out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW);
			} else {
				ComputeLocalTransformation(X.ProcessMotion(Segment.Character.MotionType), Y.ProcessMotion(Segment.Character.MotionType), Z.ProcessMotion(Segment.Character.MotionType), out lpX, out lpY, out lpZ, out lrX, out lrY, out lrZ, out lrW);
			}

			//Apply local transformation
			if(Application.isPlaying) {
				//Assigning transformation
				transform.localPosition = (1f-Segment.Character.Smoothing) * new Vector3((float)lpX, (float)lpY, (float)lpZ) + Segment.Character.Smoothing * LastPosition;
				transform.localRotation = Quaternion.Slerp(new Quaternion((float)lrX, (float)lrY, (float)lrZ, (float)lrW), LastRotation, Segment.Character.Smoothing);

				//Blending animation
				transform.localPosition = (1f-Segment.Character.AnimationBlend) * transform.localPosition + Segment.Character.AnimationBlend * AnimationPosition;
				transform.localRotation = Quaternion.Slerp(transform.localRotation, AnimationRotation, Segment.Character.AnimationBlend);
			} else {
				transform.localPosition = new Vector3((float)lpX, (float)lpY, (float)lpZ);
				transform.localRotation = new Quaternion((float)lrX, (float)lrY, (float)lrZ, (float)lrW);
			}

			//Remember transformation
			LastPosition = transform.localPosition;
			LastRotation = transform.localRotation;

			transform.hasChanged = false;
		}

		//Fast implementation to compute the local transform given the joint values (in radians / metres)
		public void ComputeLocalTransformation(double valueX, double valueY, double valueZ, out double lpX, out double lpY, out double lpZ, out double lrX, out double lrY, out double lrZ, out double lrW) {
			if(JointType == JointType.Translational) {
				//LocalPosition = DefaultLocalPosition + (Values . Axes)
				//LocalRotation = DefaultLocalRotation
				valueX /= Scale.x;
				valueY /= Scale.y;
				valueZ /= Scale.z;
				double x = valueX * X.Axis.x + valueY * Y.Axis.x + valueZ * Z.Axis.x;
				double y = valueX * X.Axis.y + valueY * Y.Axis.y + valueZ * Z.Axis.y;
				double z = valueX * X.Axis.z + valueY * Y.Axis.z + valueZ * Z.Axis.z;
				//Local position for translational motion
				lpX = AnimatedDefaultPosition.x + R1 * x + R4 * y + R7 * z;
				lpY = AnimatedDefaultPosition.y + R2 * x + R5 * y + R8 * z;
				lpZ = AnimatedDefaultPosition.z + R3 * x + R6 * y + R9 * z;
				//Local rotation for translational motion
				lrX = AnimatedDefaultRotation.x; lrY = AnimatedDefaultRotation.y; lrZ = AnimatedDefaultRotation.z; lrW = AnimatedDefaultRotation.w;
			} else {
				//LocalPosition = WorldAnchor + AngleAxis(roll) * AngleAxis(pitch) * AngleAxis(yaw) * (-LocalAnchor)
				//LocalRotation = DefaultLocalRotation * AngleAxis(roll) * AngleAxis(pitch) * AngleAxis(yaw)
/*				double sin, x1, y1, z1, w1, x2, y2, z2, w2, qx, qy, qz, qw = 0.0;*/
				/*				
								if(valueZ != 0.0) {
									sin = System.Math.Sin(valueZ/2.0);
									qx = Z.Axis.x * sin;
									qy = Z.Axis.y * sin;
									qz = Z.Axis.z * sin;
									qw = System.Math.Cos(valueZ/2.0);
									if(valueX != 0.0) {
										sin = System.Math.Sin(valueX/2.0);
										x1 = X.Axis.x * sin;
										y1 = X.Axis.y * sin;
										z1 = X.Axis.z * sin;
										w1 = System.Math.Cos(valueX/2.0);
										x2 = qx; y2 = qy; z2 = qz; w2 = qw;
										qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
										qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
										qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
										qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
										if(valueY != 0.0) {
											sin = System.Math.Sin(valueY/2.0);
											x1 = Y.Axis.x * sin;
											y1 = Y.Axis.y * sin;
											z1 = Y.Axis.z * sin;
											w1 = System.Math.Cos(valueY/2.0);
											x2 = qx; y2 = qy; z2 = qz; w2 = qw;
											qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
											qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
											qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
											qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
										} else {

										}
									} else if(valueY != 0.0) {
										sin = System.Math.Sin(valueY/2.0);
										x1 = Y.Axis.x * sin;
										y1 = Y.Axis.y * sin;
										z1 = Y.Axis.z * sin;
										w1 = System.Math.Cos(valueY/2.0);
										x2 = qx; y2 = qy; z2 = qz; w2 = qw;
										qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
										qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
										qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
										qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
									} else {

									}
								} else if(valueX != 0.0) {
									sin = System.Math.Sin(valueX/2.0);
									qx = X.Axis.x * sin;
									qy = X.Axis.y * sin;
									qz = X.Axis.z * sin;
									qw = System.Math.Cos(valueX/2.0);
									if(valueY != 0.0) {
										sin = System.Math.Sin(valueY/2.0);
										x1 = Y.Axis.x * sin;
										y1 = Y.Axis.y * sin;
										z1 = Y.Axis.z * sin;
										w1 = System.Math.Cos(valueY/2.0);
										x2 = qx; y2 = qy; z2 = qz; w2 = qw;
										qx = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2;
										qy = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2;
										qz = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2;
										qw = -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2;
									} else {

									}
								} else if(valueY != 0.0) {
									sin = System.Math.Sin(valueY/2.0);
									qx = Y.Axis.x * sin;
									qy = Y.Axis.y * sin;
									qz = Y.Axis.z * sin;
									qw = System.Math.Cos(valueY/2.0);
								} else {
									lpX = AnimatedDefaultPosition.x;
									lpY = AnimatedDefaultPosition.y;
									lpZ = AnimatedDefaultPosition.z;
									lrX = AnimatedDefaultRotation.x;
									lrY = AnimatedDefaultRotation.y;
									lrZ = AnimatedDefaultRotation.z;
									lrW = AnimatedDefaultRotation.w;
									return;
								}*/
				Quaternion test = Quaternion.Euler((float)valueX*Mathf.Rad2Deg, (float)valueY * Mathf.Rad2Deg, (float)valueZ * Mathf.Rad2Deg);
/*				qx = test.x;
				qy = test.y;
				qz = test.z;
				qw = test.w;*/
				test = AnimatedDefaultRotation* test ;
				//Local Rotation
				//R' = R*Q
				/*				lrX = AnimatedDefaultRotation.x * qw + AnimatedDefaultRotation.y * qz - AnimatedDefaultRotation.z * qy + AnimatedDefaultRotation.w * qx;
								lrY = -AnimatedDefaultRotation.x * qz + AnimatedDefaultRotation.y * qw + AnimatedDefaultRotation.z * qx + AnimatedDefaultRotation.w * qy;
								lrZ = AnimatedDefaultRotation.x * qy - AnimatedDefaultRotation.y * qx + AnimatedDefaultRotation.z * qw + AnimatedDefaultRotation.w * qz;
								lrW = -AnimatedDefaultRotation.x * qx - AnimatedDefaultRotation.y * qy - AnimatedDefaultRotation.z * qz + AnimatedDefaultRotation.w * qw;*/

				lrX = test.x;
				lrY = test.y;
				lrZ = test.z;
				lrW = test.w;
				//Local Position
				if (LSA.x == 0.0 && LSA.y == 0.0 && LSA.z == 0.0) {
					//P' = Pz
					lpX = AnimatedDefaultPosition.x;
					lpY = AnimatedDefaultPosition.y;
					lpZ = AnimatedDefaultPosition.z;
				} else {
					//P' = P + RA + R*Q*(-A)
					Vector3 localPosition = ADPADRSA + test * LSA;

					lpX = localPosition.x;
					lpY = localPosition.y;
					lpZ = localPosition.z;
					/*					lpX = ADPADRSA.x + 2.0 * ((0.5 - lrY * lrY - lrZ * lrZ) * -LSA.x + (lrX * lrY - lrW * lrZ) * -LSA.y + (lrX * lrZ + lrW * lrY) * -LSA.z);
										lpY = ADPADRSA.y + 2.0 * ((lrX * lrY + lrW * lrZ) * -LSA.x + (0.5 - lrX * lrX - lrZ * lrZ) * -LSA.y + (lrY * lrZ - lrW * lrX) * -LSA.z);
										lpZ = ADPADRSA.z + 2.0 * ((lrX * lrZ - lrW * lrY) * -LSA.x + (lrY * lrZ + lrW * lrX) * -LSA.y + (0.5 - lrX * lrX - lrY * lrY) * -LSA.z);*/

					Vector3 test2 = ADPADRSA + test * LSA;
				}
			}
		}

		public Vector3 GetAnchorInWorldSpace() {
			return transform.position + transform.rotation * Vector3.Scale(transform.lossyScale, Anchor);
		}

		public void SetAnchor(Vector3 anchor) {
			Anchor = anchor;
		}

		public Vector3 GetAnchor() {
			return Anchor;
		}

		public void SetOrientation(Vector3 orientation) {
			Orientation = orientation;
			Quaternion o = Quaternion.Euler(Orientation);
			X.Axis = o * Vector3.right;
			Y.Axis = o * Vector3.up;
			Z.Axis = o * Vector3.forward;
		}

		public Vector3 GetOrientation() {
			return Orientation;
		}

		public Vector3 GetDefaultPosition() {
			return new Vector3(DPX, DPY, DPZ);
		}

		public Quaternion GetDefaultRotation() {
			return new Quaternion(DRX, DRY, DRZ, DRW);
		}

		public int GetDoF() {
			int dof = 0;
			if(X.IsEnabled()) {
				dof += 1;
			}
			if(Y.IsEnabled()) {
				dof += 1;
			}
			if(Z.IsEnabled()) {
				dof += 1;
			}
			return dof;
		}

		public void SetDefaultFrame(Vector3 localPosition, Quaternion localRotation) {
			DPX = localPosition.x;
			DPY = localPosition.y;
			DPZ = localPosition.z;
			DRX = localRotation.x;
			DRY = localRotation.y;
			DRZ = localRotation.z;
			DRW = localRotation.w;
		}

		public void RestoreDefaultFrame() {
			transform.localPosition = new Vector3(DPX, DPY, DPZ);
			transform.localRotation = new Quaternion(DRX, DRY, DRZ, DRW);
		}

		[System.Serializable]
		public class Motion {
			public BioJoint Joint;

			public bool Enabled;
			public bool Constrained = true;
			public double LowerLimit;
			public double UpperLimit;
			public double TargetValue;
			public double CurrentValue;
			public double CurrentError;
			public double CurrentVelocity;
			public double CurrentAcceleration;
			public Vector3 Axis;

			private const double Speedup = 1.0;
			private const double Slowdown = 1.0;

			public Motion(BioJoint joint, Vector3 axis) {
				Joint = joint;
				Axis = axis;
			}

			//Runs one motion control cycle
			public double ProcessMotion(MotionType type) {
				if(!Enabled) {
					//return CurrentValue;
					return 0.0;
				}

				if(!Application.isPlaying) {
					UpdateInstantaneous();
				} else {
					if(type == MotionType.Instantaneous) {
						UpdateInstantaneous();
					}
					if(type == MotionType.Realistic) {
						UpdateRealistic();
					}
				}

				return CurrentValue;
			}

			//Performs instantaneous motion control
			private void UpdateInstantaneous() {
				CurrentValue = TargetValue;
				CurrentError = 0f;
				CurrentVelocity = 0f;
				CurrentAcceleration = 0f;
			}

			//Performs realistic motion control
			//Input: TargetValue, CurrentValue, CurrentVelocity (initially 0), CurrentAcceleration (initially 0), MaximumVelocity, MaximumAcceleration
			//Output: CurrentValue, CurrentVelocity, CurrentAcceleration
			private void UpdateRealistic() {
				if(Time.deltaTime == 0f) {
					return;
				}

				double maxVel = Joint.JointType == JointType.Rotational ? Utility.Rad2Deg * Joint.Segment.Character.MaximumVelocity : Joint.Segment.Character.MaximumVelocity;
				double maxAcc = Joint.JointType == JointType.Rotational ? Utility.Rad2Deg * Joint.Segment.Character.MaximumAcceleration : Joint.Segment.Character.MaximumAcceleration;

				//Compute current error
				CurrentError = TargetValue-CurrentValue;	

				//Minimum distance to stop: s = |(v^2)/(2a_max)| + |a/2*t^2| + |v*t|
				double stoppingDistance = 
					System.Math.Abs((CurrentVelocity*CurrentVelocity)/(2.0*maxAcc*Slowdown))
					+ System.Math.Abs(CurrentAcceleration)/2.0*Time.deltaTime*Time.deltaTime
					+ System.Math.Abs(CurrentVelocity)*Time.deltaTime;

				if(System.Math.Abs(CurrentError) > stoppingDistance) {
					//Accelerate
					CurrentAcceleration = System.Math.Sign(CurrentError)*System.Math.Min(System.Math.Abs(CurrentError) / Time.deltaTime, maxAcc*Speedup);
				} else {
					//Deccelerate
					if(CurrentError == 0.0) {
						CurrentAcceleration = -System.Math.Sign(CurrentVelocity)*
						System.Math.Min(System.Math.Abs(CurrentVelocity) / Time.deltaTime, maxAcc);
						
					} else {
						CurrentAcceleration = -System.Math.Sign(CurrentVelocity)*
						System.Math.Min(
							System.Math.Min(System.Math.Abs(CurrentVelocity) / Time.deltaTime, maxAcc), 
							System.Math.Abs((CurrentVelocity*CurrentVelocity)/(2.0*CurrentError))
						);
					}
				}

				//Compute new velocity
				CurrentVelocity += CurrentAcceleration*Time.deltaTime;

				//Clamp velocity
				if(CurrentVelocity > maxVel) {
					CurrentVelocity = maxVel;
				}
				if(CurrentVelocity < -maxVel) {
					CurrentVelocity = -maxVel;
				}
				
				//Update Current Value
				CurrentValue += CurrentVelocity*Time.deltaTime;
			}

			public void SetEnabled(bool enabled) {
				if(Enabled != enabled) {
					Enabled = enabled;
					Joint.Segment.Character.Refresh();
				}
			}

			public bool IsEnabled() {
				return Enabled;
			}

			public void SetLowerLimit(double value) {
				LowerLimit = System.Math.Min(0.0, value);
			}

			public double GetLowerLimit(bool normalised = false) {
				if(Constrained) {
					if(normalised && Joint.JointType == JointType.Rotational) {
						return Utility.Deg2Rad * LowerLimit;
					} else {
						return LowerLimit;
					}
				} else {
					return double.MinValue;
				}
			}

			public void SetUpperLimit(double value) {
				UpperLimit = System.Math.Max(0.0, value);
			}

			public double GetUpperLimit(bool normalised = false) {
				if(Constrained) {
					if(normalised && Joint.JointType == JointType.Rotational) {
						return Utility.Deg2Rad * UpperLimit;
					} else {
						return UpperLimit;
					}
				} else {
					return double.MaxValue;
				}
			}

			public void SetTargetValue(double value, bool normalised = false) {
				if(normalised && Joint.JointType == JointType.Rotational) {
					value *= Utility.Rad2Deg;
				}
				if(Constrained) {
					if(TargetValue > UpperLimit) {
						value = UpperLimit;
					}
					if(TargetValue < LowerLimit) {
						value = LowerLimit;
					}
				}
				TargetValue = value;
			}

			public double GetTargetValue(bool normalised = false) {
				if(normalised && Joint.JointType == JointType.Rotational) {
					return Utility.Deg2Rad * TargetValue;
				} else {
					return TargetValue;
				}
			}

			public double GetCurrentValue() {
				return CurrentValue;
			}

			public double GetCurrentError() {
				return CurrentError;
			}

			public double GetCurrentVelocity() {
				return CurrentVelocity;
			}

			public double GetCurrentAcceleration() {
				return CurrentAcceleration;
			}

		}
	}

}