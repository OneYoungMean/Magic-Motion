using UnityEngine;

//!!!!!!
//This objective type is still under development
//!!!!!!

namespace BIOIK {

	//This objective aims to keep particular joint values acting as soft-constraints. Using this requires
	//a joint added to the segment, and introduces some sort of stiffness controlled by the weight to the joint.
	//Currently, you will require one objective for each >enabled< motion axis you wish to control.
	[AddComponentMenu("")]
	public class JointValue : BioObjective {

		public double TargetValue = 0.0;
		public bool X, Y, Z;
		
		private BioJoint.Motion Motion;
		private bool Valid;
		private Model.MotionPtr Ptr;
		private double NormalisedTargetValue;

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.JointValue;
		}

		public override void UpdateData() {
			if(Segment.Character.Evolution == null) {
				return;
			}
			Valid = IsValid();
			if(!Valid) {
				return;
			}
			if(X) {
				Motion = Segment.Joint.X;
			}
			if(Y) {
				Motion = Segment.Joint.Y;
			}
			if(Z) {
				Motion = Segment.Joint.Z;
			}
			Ptr = Segment.Character.Evolution.GetModel().FindMotionPtr(Motion);
			NormalisedTargetValue = Segment.Joint.JointType == JointType.Rotational ? NormalisedTargetValue = Utility.Deg2Rad * TargetValue : TargetValue;
		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			if(!Valid) {
				return 0.0;
			}
			double loss = configuration[Ptr.Index] - NormalisedTargetValue;
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return true;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			if(!Valid) {
				return 0.0;
			}
			return System.Math.Abs(configuration[Ptr.Index] - NormalisedTargetValue);
		}

		public void SetTargetValue(double value) {
			TargetValue = value;
		}

		public double GetTargetValue() {
			return TargetValue;
		}

		public void SetXMotion(bool enabled) {
			if(X != enabled) {
				ResetMotion();
				X = enabled;
			}
		}

		public bool IsXMotion() {
			return X;
		}

		public void SetYMotion(bool enabled) {
			if(Y != enabled) {
				ResetMotion();
				Y = enabled;
			}
		}

		public bool IsYMotion() {
			return Y;
		}

		public void SetZMotion(bool enabled) {
			if(Z != enabled) {
				ResetMotion();
				Z = enabled;
			}
		}

		public bool IsZMotion() {
			return Z;
		}

		private void ResetMotion() {
			X = false; Y = false; Z = false;
		}

		private bool IsValid() {
			if(Segment.Joint == null) {
				return false;
			}
			if(!X && !Y && !Z) {
				return false;
			}
			if(X && !Segment.Joint.X.IsEnabled()) {
				return false;
			}
			if(Y && !Segment.Joint.Y.IsEnabled()) {
				return false;
			}
			if(Z && !Segment.Joint.Z.IsEnabled()) {
				return false;
			}
			return true;
		}
	}
}