using UnityEngine;

//!!!!!!
//This objective type is still under development
//!!!!!!

namespace BIOIK {

	//This objective aims to minimise the joint configuration changes between consecutive solutions.
	//It should only be used once as it acts as a global objective for the whole body posture.
	//Preferably add it to the root of your character.
	[AddComponentMenu("")]
	public class Displacement : BioObjective {

		public override ObjectiveType GetObjectiveType() {
			return ObjectiveType.Displacement;
		}

		public override void UpdateData() {

		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double loss = 0.0;
			for(int i=0; i<configuration.Length; i++) {
				double diff = System.Math.Abs(Segment.Character.Evolution.GetSolution()[i] - configuration[i]) / (Segment.Character.Evolution.GetUpperBounds()[i] - Segment.Character.Evolution.GetLowerBounds()[i]);
				loss += diff;
			}
			loss /= configuration.Length;
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			return true;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration) {
			double value = 0.0;
			for(int i=0; i<configuration.Length; i++) {
				double diff = System.Math.Abs(Segment.Character.Evolution.GetSolution()[i] - configuration[i]) / (Segment.Character.Evolution.GetUpperBounds()[i] - Segment.Character.Evolution.GetLowerBounds()[i]);
				value += diff;
			}
			return value /= configuration.Length;
		}
	}
	
}