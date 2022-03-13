using UnityEngine;

namespace BIOIK {

	//This objective aims to minimise the viewing distance between the transform and the target.
	[AddComponentMenu("")]
    public class LookAt : BioObjective
	{//OYM:该目标旨在最小化变换和目标之间的观看距离。

		[SerializeField] private Transform Target;
		[SerializeField] private float TPX, TPY, TPZ;
		[SerializeField] private Vector3 ViewingDirection = Vector3.forward;
		[SerializeField] private float MaximumError = 0.1f;

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

		public override float ComputeLoss(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float aX = 2.0f* ((0.5f- (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			float aY = 2.0f* ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5f- (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			float aZ = 2.0f* ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5f- (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			float bX = TPX-WPX;
			float bY = TPY-WPY;
			float bZ = TPZ-WPZ;
			float dot = aX*bX + aY*bY + aZ*bZ;
			float len = (float)(System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ));
			float arg = dot/len;
			if(arg > 1.0f) {
				arg = 1.0f;
			} else if(arg < -1.0f) {
				arg = -1.0f;
			}
			float loss = (float)System.Math.Acos(arg);
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float aX = 2.0f* ((0.5f- (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			float aY = 2.0f* ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5f- (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			float aZ = 2.0f* ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5f- (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			float bX = TPX-WPX;
			float bY = TPY-WPY;
			float bZ = TPZ-WPZ;
			float dot = aX*bX + aY*bY + aZ*bZ;
			float len = (float)(System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ));
			float arg = dot/len;
			if(arg > 1.0f) {
				arg = 1.0f;
			} else if(arg < -1.0f) {
				arg = -1.0f;
			}
			return System.Math.Acos(arg) <= Utility.Deg2Rad * MaximumError;
		}

		public override float ComputeValue(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float aX = 2.0f* ((0.5f- (WRY * WRY + WRZ * WRZ)) * ViewingDirection.x + (WRX * WRY - WRW * WRZ) * ViewingDirection.y + (WRX * WRZ + WRW * WRY) * ViewingDirection.z);
			float aY = 2.0f* ((WRX * WRY + WRW * WRZ) * ViewingDirection.x + (0.5f- (WRX * WRX + WRZ * WRZ)) * ViewingDirection.y + (WRY * WRZ - WRW * WRX) * ViewingDirection.z);
			float aZ = 2.0f* ((WRX * WRZ - WRW * WRY) * ViewingDirection.x + (WRY * WRZ + WRW * WRX) * ViewingDirection.y + (0.5f- (WRX * WRX + WRY * WRY)) * ViewingDirection.z);
			float bX = TPX-WPX;
			float bY = TPY-WPY;
			float bZ = TPZ-WPZ;
			float dot = aX*bX + aY*bY + aZ*bZ;
			float len = (float)(System.Math.Sqrt(aX*aX + aY*aY + aZ*aZ) * System.Math.Sqrt(bX*bX + bY*bY + bZ*bZ));
			float arg = dot/len;
			if(arg > 1.0f) {
				arg = 1.0f;
			} else if(arg < -1.0f) {
				arg = -1.0f;
			}
			return (float)(Utility.Rad2Deg * System.Math.Acos(arg));
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

		public void SetMaximumError(float degrees) {
			MaximumError = degrees;
		}

		public float GetMaximumError() {
			return MaximumError;
		}
	}
	
}