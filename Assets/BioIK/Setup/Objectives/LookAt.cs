using UnityEngine;

namespace BIOIK {

	//This objective aims to minimise the viewing distance between the transform and the target.
	[AddComponentMenu("")]
    public class LookAt : BioObjective
	{//OYM:该目标旨在最小化变换和目标之间的观看距离。

		[SerializeField] private Transform Target;
		[SerializeField] private double TPX, TPY, TPZ;
		[SerializeField] private Vector3 ViewingDirection = Vector3.forward;
		[SerializeField] private double MaximumError = 0.1;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.LookAt;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			if(Target != null) {
				Vector3 position = Target.position;
				TPX = position.x;
				TPY = position.y;
				TPZ = position.z;
			}
		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double aX = 2.0 * ((0.5 - (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			double aY = 2.0 * ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5 - (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			double aZ = 2.0 * ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5 - (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			double bX = TPX-WPX;
			double bY = TPY-WPY;
			double bZ = TPZ-WPZ;
			double dot = aX*bX + aY*bY + aZ*bZ;
			double len = System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ);
			double arg = dot/len;
			if(arg > 1.0) {
				arg = 1.0;
			} else if(arg < -1.0) {
				arg = -1.0;
			}
			double loss = System.Math.Acos(arg);
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double aX = 2.0 * ((0.5 - (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			double aY = 2.0 * ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5 - (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			double aZ = 2.0 * ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5 - (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			double bX = TPX-WPX;
			double bY = TPY-WPY;
			double bZ = TPZ-WPZ;
			double dot = aX*bX + aY*bY + aZ*bZ;
			double len = System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ);
			double arg = dot/len;
			if(arg > 1.0) {
				arg = 1.0;
			} else if(arg < -1.0) {
				arg = -1.0;
			}
			return System.Math.Acos(arg) <= Utility.Deg2Rad * MaximumError;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double aX = 2.0 * ((0.5 - (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			double aY = 2.0 * ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5 - (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			double aZ = 2.0 * ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5 - (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			double bX = TPX-WPX;
			double bY = TPY-WPY;
			double bZ = TPZ-WPZ;
			double dot = aX*bX + aY*bY + aZ*bZ;
			double len = System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ);
			double arg = dot/len;
			if(arg > 1.0) {
				arg = 1.0;
			} else if(arg < -1.0) {
				arg = -1.0;
			}
			return Utility.Rad2Deg * System.Math.Acos(arg);
		}

		public void SetTargetTransform(Transform target) {
			Target = target;
			if(target != null) {
				SetTargetPosition(Target.position);
			}
		}

		public Transform GetTargetTransform() {
			return Target;
		}

		public void SetTargetPosition(Vector3 position) {
			TPX = position.x;
			TPY = position.y;
			TPZ = position.z;
		}

		public Vector3 GetTargetPosition() {
			return new Vector3((float)TPX, (float)TPY, (float)TPZ);
		}

		public void SetViewingDirection(Vector3 vector) {
			ViewingDirection = vector;
		}

		public Vector3 GetViewingDirection() {
			return ViewingDirection;
		}

		public void SetMaximumError(double degrees) {
			MaximumError = degrees;
		}

		public double GetMaximumError() {
			return MaximumError;
		}
	}
	
}