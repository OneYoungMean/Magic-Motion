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

		public override float ComputeLoss(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float loss = 0.0f;
			for(int i=0; i<configuration.Length; i++) {
				float diff = System.Math.Abs(Segment.Character.Evolution.GetSolution()[i] - configuration[i]) / (Segment.Character.Evolution.GetUpperBounds()[i] - Segment.Character.Evolution.GetLowerBounds()[i]);
				loss += diff;
			}
			loss /= configuration.Length;
			return Weight * loss * loss;
		}

		public override bool CheckConvergence(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			return false;
		}

		public override float ComputeValue(float WPX, float WPY, float WPZ, float WRX, float WRY, float WRZ, float WRW, Model.Node node, float[] configuration) {
			float value = 0.0f;
			for(int i=0; i<configuration.Length; i++) {
				float diff = System.Math.Abs(Segment.Character.Evolution.GetSolution()[i] - configuration[i]) / (Segment.Character.Evolution.GetUpperBounds()[i] - Segment.Character.Evolution.GetLowerBounds()[i]);
				value += diff;
			}
			return value /= configuration.Length;
		}
	}
	
}