using UnityEngine;

namespace BIOIK {

	//This objective aims to minimise both the translational and rotational distance between
	//the projected transform with respect to the normal of the game object to other collideable objects in the scene.
	[AddComponentMenu("")]
	public class Projection : BioObjective {
		//OYM://这个目标旨在最小化之间的平移和旋转距离
		//相对于游戏对象法线到场景中其他可碰撞对象的投影变换。

		[SerializeField] private Transform Target;
		[SerializeField] private double TPX, TPY, TPZ;
		[SerializeField] private double TRX, TRY, TRZ, TRW;
		[SerializeField] private double MaximumError = 0.001;
		[SerializeField] private Vector3 Normal = Vector3.up;
		[SerializeField] private float Length;
		[SerializeField] private float Sensitivity = 0.75f;

		private Vector3 LastPosition;
		private Quaternion LastRotation;

		//private Vector3 Position;
		//private Quaternion Rotation;
		private double ChainLength;
		private double Rescaling;

		//public Correction[] Corrections = new Correction[0];

		private RaycastHit[] Hits;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Projection;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}

			ChainLength = 0.0;
			Transform[] chain = Segment.Character.Evolution.GetModel().FindObjectivePtr(this).Node.Chain;;
			for(int i=0; i<chain.Length-1; i++) {
				ChainLength += Vector3.Distance(chain[i].position, chain[i+1].position);
			}
			Rescaling = (Utility.PI * Utility.PI) / (ChainLength * ChainLength);

			if(Target != null) {
				SetTargetPosition(Target.position);
				SetTargetRotation(Target.rotation);
			}

			Vector3 position = new Vector3((float)TPX, (float)TPY, (float)TPZ);
			Quaternion rotation = new Quaternion((float)TRX, (float)TRY, (float)TRZ, (float)TRW);

			Vector3 normal = rotation * Normal;
			Vector3 start = position - Length * normal;
			Vector3 end = start + Length * normal;

			Hits = Physics.RaycastAll(start, end-start, Length);
			if(Hits.Length > 0) {
				SortByDistance(ref Hits);
				for(int i=0; i<Hits.Length; i++) {
					RaycastHit hit = Hits[i];
					if(!hit.collider.isTrigger && Segment.Character.FindSegment(hit.collider.transform) == null) {
						position = hit.point;
						rotation = Quaternion.FromToRotation(normal, -hit.normal.normalized) * rotation;
						break;
					}
				}
			}

			position = Vector3.Slerp(LastPosition, position, Sensitivity);
			rotation = Quaternion.Slerp(LastRotation, rotation, Sensitivity);

			LastPosition = position;
			LastRotation = rotation;

			/*
			Vector3 correctionPosition = Vector3.zero;
			Quaternion correctionRotation = Quaternion.identity;
			for(int i=0; i<Corrections.Length; i++) {
				if(Physics.Raycast(position + rotation*Corrections[i].Offset, rotation*Corrections[i].Direction.normalized, out Hit, Corrections[i].Length)) {
					if(Hit.collider.transform.root != transform.root) {
						float weight = (Corrections[i].Length-Hit.distance) / Corrections[i].Length;
						correctionPosition += weight * (Hit.point-position);
						correctionRotation = Quaternion.Slerp(Quaternion.identity, Quaternion.FromToRotation(rotation*Normal.normalized, -Hit.normal.normalized), weight) * correctionRotation;
					}
				}
			}

			position += correctionPosition;
			rotation = correctionRotation * rotation;
			*/

			TPX = position.x;
			TPY = position.y;
			TPZ = position.z;

			TRX = rotation.x;
			TRY = rotation.y;
			TRZ = rotation.z;
			TRW = rotation.w;
		}

		private void SortByDistance(ref RaycastHit[] hits) {
			System.Array.Sort(hits,
				delegate(RaycastHit a, RaycastHit b) {
					return a.distance.CompareTo(b.distance);
				}
			);
		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			/*
			double d = System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
			double s = System.Math.Sqrt((node.Chain.Length+d)*(System.Math.Sqrt((WPX-node.RootX)*(WPX-node.RootX) + (WPY-node.RootY)*(WPY-node.RootY) + (WPZ-node.RootZ)*(WPZ-node.RootZ))+d));
			d = PI * d / s;
			double o = WRX*TRX + WRY*TRY + WRZ*TRZ + WRW*TRW;
			if(o < 0.0) {
				o = -o;
			}
			if(o > 1.0) {
				o = 1.0;
			}
			o = 2.0 * System.Math.Acos(o);
			return Weight * 0.5 * (d*d + o*o);
			*/
			double d = Rescaling * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
			double o = WRX*TRX + WRY*TRY + WRZ*TRZ + WRW*TRW;
			if(o < 0.0) {
				o = -o;
				if(o > 1.0) {
					o = 1.0;
				}
			} else if(o > 1.0) {
				o = 1.0;
			}
			o = 2.0 * System.Math.Acos(o);
			return Weight * 0.5 * (d*d + o*o);
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double d = System.Math.Sqrt((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
			double o = WRX*TRX + WRY*TRY + WRZ*TRZ + WRW*TRW;
			if(o < 0.0) {
				o = -o;
				if(o > 1.0) {
					o = 1.0;
				}
			} else if(o > 1.0) {
				o = 1.0;
			}
			o = Utility.Rad2Deg * 2.0 * System.Math.Acos(o);
			return d <= MaximumError && o <= MaximumError;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return 0.0;
		}

		public void SetTargetTransform(Transform target) {
			Target = target;
			if(Target != null) {
				SetTargetPosition(Target.position);
				SetTargetRotation(Target.rotation);
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

		public void SetTargetRotation(Quaternion rotation) {
			TRX = rotation.x;
			TRY = rotation.y;
			TRZ = rotation.z;
			TRW = rotation.w;
		}

		public void SetTargetRotation(Vector3 angles) {
			SetTargetRotation(Quaternion.Euler(angles));
		}

		public Vector3 GetTargetRotation() {
			return new Quaternion((float)TRX, (float)TRY, (float)TRZ, (float)TRW).eulerAngles;
		}

		public void SetMaximumError(double value) {
			MaximumError = value;
		}

		public double GetMaximumError() {
			return MaximumError;
		}

		public void SetNormal(Vector3 normal) {
			Normal = normal;
		}

		public Vector3 GetNormal() {
			return Normal;
		}

		public void SetSensitivity(float sensitivity) {
			Sensitivity = Mathf.Clamp(sensitivity, 0f, 1f);
		}

		public float GetSensitivity() {
			return Sensitivity;
		}

		public void SetLength(float length) {
			Length = length;
		}

		public float GetLength() {
			return Length;
		}
		
	}
}