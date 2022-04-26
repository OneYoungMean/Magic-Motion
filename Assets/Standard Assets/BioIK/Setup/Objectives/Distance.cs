using UnityEngine;

//!!!!!!
//This objective type is still under development
//!!!!!!

namespace BIOIK {

	//This objective aims to keep particular distances to the defined transform positions. This can be used to integrate
	//real-time collision avoidance. However, note that collision avoidance is typically a very challenging thing for motion generation,
	//so please do not expect any wonders or some sort of black magic. It works well for typical scenarios, but it will not solve the world for you.
	//Note that you should use preferably small weights in order to get good-looking results. Best thing is to play around with it and see what happens.
	//It is not generally clear how to chose those weights.
	[AddComponentMenu("")]
    public class Distance : BioObjective
    { //OYM:与某一点保持特定的距离

        [SerializeField] private float Radius = 0.1f;
		[SerializeField] private DistancePoint[] Points = new DistancePoint[0];

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Distance;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			for(int i=0; i<Points.Length; i++) {
				if(Points[i].Target != null) {
					Vector3 position = Points[i].Target.position;
					Points[i].TPX = position.x;
					Points[i].TPY = position.y;
					Points[i].TPZ = position.z;
				}
			}
		}

		public override float ComputeLoss(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float loss = 0.0f;
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					float dist = (float)System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPY-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ));
					float x = dist - Radius;
					if(x <= 0.0f) {
						return float.MaxValue;
					} else {
						loss += 1.0f/x;
					}
				}
			}
			loss /= Points.Length;
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					if(System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPZ-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ)) <= Radius) {
						return false;
					}
				}
			}
			return true;
		}

		public override float ComputeValue(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float dist = 0.0f;
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					dist =(float) System.Math.Max(dist, System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPY-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ)));
				}
			}
			return dist;
		}

		public void SetRadius(float radius) {
			Radius = radius;
		}

		public float GetRadius() {
			return Radius;
		}

		public DistancePoint[] GetPoints() {
			return Points;
		}

		public DistancePoint AddPoint() {
			System.Array.Resize(ref Points, Points.Length+1);
			Points[Points.Length-1] = new DistancePoint();
			return Points[Points.Length-1];
		}

		public void RemovePoint() {
			if(Points.Length > 0) {
				System.Array.Resize(ref Points, Points.Length-1);
			}
		}
	}

	[System.Serializable]
	public class DistancePoint {
		public Transform Target;
		public float Radius;
		public float TPX, TPY, TPZ;

		public void SetTargetTransform(Transform t) {
			Target = t;
			if(Target != null) {
				SetTargetPoint(Target.position);
			}
		}

		public void SetTargetPoint(Vector3 point) {
			TPX = point.x;
			TPY = point.y;
			TPZ = point.z;
		}

		public Vector3 GetTargetPoint() {
			return new Vector3((float)TPX, (float)TPY, (float)TPZ);
		}

		public void SetRadius(float radius) {
			Radius = radius;
		}

		public float GetRadius() {
			return Radius;
		}
	}

}