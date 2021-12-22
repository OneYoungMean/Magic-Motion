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

        [SerializeField] private double Radius = 0.1;
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

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double loss = 0.0;
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					double dist = System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPY-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ));
					double x = dist - Radius;
					if(x <= 0.0) {
						return float.MaxValue;
					} else {
						loss += 1.0/x;
					}
				}
			}
			loss /= Points.Length;
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					if(System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPZ-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ)) <= Radius) {
						return false;
					}
				}
			}
			return true;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double dist = 0.0;
			for(int i=0; i<Points.Length; i++) {
				if(Points[i] != null) {
					dist = System.Math.Max(dist, System.Math.Sqrt((Points[i].TPX-WPX)*(Points[i].TPX-WPX) + (Points[i].TPY-WPY)*(Points[i].TPY-WPY) + (Points[i].TPZ-WPZ)*(Points[i].TPZ-WPZ)));
				}
			}
			return dist;
		}

		public void SetRadius(double radius) {
			Radius = radius;
		}

		public double GetRadius() {
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
		public double Radius;
		public double TPX, TPY, TPZ;

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

		public void SetRadius(double radius) {
			Radius = radius;
		}

		public double GetRadius() {
			return Radius;
		}
	}

}