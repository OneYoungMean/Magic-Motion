using UnityEngine;

namespace BIOIK {

	//This objective aims to minimise the rotational distance between the transform and the target.
	[AddComponentMenu("")]
    public class Orientation : BioObjective
    { //OYM:hmm最小化旋转,看上去好像是保持旋转一致的

        [SerializeField] private Transform Target;
		[SerializeField] private float rotTRX, rotTRY, rotTRZ, rotTRW;
		[SerializeField] private float MaximumRotationError = 0.1f;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Orientation;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			if(Target != null) {
				Quaternion rotation = Target.rotation;
				rotTRX = rotation.x;
				rotTRY = rotation.y;
				rotTRZ = rotation.z;
				rotTRW = rotation.w;
			}
		}

		public override float ComputeLoss(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float d = WRX*rotTRX + WRY*rotTRY + WRZ*rotTRZ + WRW*rotTRW;
			if(d < 0.0f) {
				d = -d;
				if(d > 1.0f) {
					d = 1.0f;
				}
			} else if(d > 1.0f) {
				d = 1.0f;
			}
			float loss =(float)( 2.0f* System.Math.Acos(d));
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float d = WRX*rotTRX + WRY*rotTRY + WRZ*rotTRZ + WRW*rotTRW;
			if(d < 0.0f) {
				d = -d;
				if(d > 1.0f) {
					d = 1.0f;
				}
			} else if(d > 1.0f) {
				d = 1.0f;
			}
			return 2.0f* System.Math.Acos(d) <= Utility.Deg2Rad * MaximumRotationError;
		}

		public override float ComputeValue(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float d = WRX*rotTRX + WRY*rotTRY + WRZ*rotTRZ + WRW*rotTRW;
			if(d < 0.0f) {
				d = -d;
				if(d > 1.0f) {
					d = 1.0f;
				}
			} else if(d > 1.0f) {
				d = 1.0f;
			}
			return (float) (Utility.Rad2Deg * 2.0f* System.Math.Acos(d));
		}

		public void SetTargetTransform(Transform target) {
			Target = target;
			if(Target != null) {
				SetTargetRotation(Target.rotation);
			}
		}

		public Transform GetTargetTransform() {
			return Target;
		}

		public void SetTargetRotation(Quaternion rotation) {
			rotTRX = rotation.x;
			rotTRY = rotation.y;
			rotTRZ = rotation.z;
			rotTRW = rotation.w;
		}

		public void SetTargetRotation(Vector3 angles) {
			SetTargetRotation(Quaternion.Euler(angles));
		}

		public Vector3 GetTargetRotattion() {
			return new Quaternion((float)rotTRX, (float)rotTRY, (float)rotTRZ, (float)rotTRW).eulerAngles;
		}

		public void SetMaximumError(float degrees) {
			MaximumRotationError = degrees;
		}

		public float GetMaximumError() {
			return MaximumRotationError;
		}
		
	}

}