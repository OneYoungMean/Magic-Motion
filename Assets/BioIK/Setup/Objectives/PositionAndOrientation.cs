/*
using UnityEngine;

namespace BIOIK
{

	//This objective aims to minimise the translational distance between the transform and the target.
	//OYM:找到距离当前点最近的状态
	[AddComponentMenu("")]
	public class PositionAndOrientation : BioObjective
	{

		[SerializeField] public Transform Target;
		[SerializeField] private double posTPX, posTPY, posTPZ;
		[SerializeField] public double MaximumPositionError = 0.001;
		[SerializeField] private double rotTRX, rotTRY, rotTRZ, rotTRW;
		[SerializeField] public double MaximumRotationError = 0.1;
		private double ChainLength;
		private double Rescaling;
		//private Vector3 Root;

		public override ObjectiveType GetObjectiveType()
		{
			return ObjectiveType.PositionAndOrientation;
		}

		public override void UpdateData()
		{
			if (Segment.Character.Evolution == null)
			{
				return;
			}
			ChainLength = 0.0;
			Transform[] chain = Segment.Character.Evolution.GetModel().FindObjectivePtr(this).Node.Chain; ;
			for (int i = 0; i < chain.Length - 1; i++)
			{
				ChainLength += Vector3.Distance(chain[i].position, chain[i + 1].position);
			}
			Rescaling = (Utility.PI * Utility.PI) / (ChainLength * ChainLength);
			//Root = chain[0].position;
			if (Target != null)
			{
				Vector3 position = Target.position;
				posTPX = position.x;
				posTPY = position.y;
				posTPZ = position.z;

				Quaternion rotation = Target.rotation;
				rotTRX = rotation.x;
				rotTRY = rotation.y;
				rotTRZ = rotation.z;
				rotTRW = rotation.w;
			}
		}

		public override double ComputeLoss(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration)
		{

			//Fast and slightly adaptive
			double d = WRX * rotTRX + WRY * rotTRY + WRZ * rotTRZ + WRW * rotTRW;
			if (d < 0.0)
			{
				d = -d;
				if (d > 1.0)
				{
					d = 1.0;
				}
			}
			else if (d > 1.0)
			{
				d = 1.0;
			}
			double loss = 2.0 * System.Math.Acos(d);


			return Weight *( Rescaling * ((posTPX - WPX) * (posTPX - WPX) + (posTPY - WPY) * (posTPY - WPY) + (posTPZ - WPZ) * (posTPZ - WPZ))+ loss * loss);

			//Fastest but not adaptive
			//return Weight * ((TPX-WPX)*(TPX-WPX) + (TPY-WPY)*(TPY-WPY) + (TPZ-WPZ)*(TPZ-WPZ));
		}

		public override bool CheckConvergence(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration)
		{

			double d = WRX * rotTRX + WRY * rotTRY + WRZ * rotTRZ + WRW * rotTRW;
			if (d < 0.0)
			{
				d = -d;
				if (d > 1.0)
				{
					d = 1.0;
				}
			}
			else if (d > 1.0)
			{
				d = 1.0;
			}
			return System.Math.Sqrt((posTPX - WPX) * (posTPX - WPX) + (posTPY - WPY) * (posTPY - WPY) + (posTPZ - WPZ) * (posTPZ - WPZ)) <= MaximumPositionError&&
				2.0 * System.Math.Acos(d) <= Utility.Deg2Rad * MaximumRotationError;
		}

		public override double ComputeValue(double WPX, double WPY, double WPZ, double WRX, double WRY, double WRZ, double WRW, Model.Node node, double[] configuration)
		{
			return System.Math.Sqrt((posTPX - WPX) * (posTPX - WPX) + (posTPY - WPY) * (posTPY - WPY) + (posTPZ - WPZ) * (posTPZ - WPZ));
		}

		public void SetTargetTransform(Transform target)
		{
			Target = target;
			if (Target != null)
			{
				SetTargetPosition(Target.position);
			}
		}

		public Transform GetTargetTransform()
		{
			return Target;
		}

		public void SetTargetPosition(Vector3 position)
		{
			posTPX = position.x;
			posTPY = position.y;
			posTPZ = position.z;
		}

		public void SetTargetRotation(Quaternion rotation)
		{
			rotTRX = rotation.x;
			rotTRY = rotation.y;
			rotTRZ = rotation.z;
			rotTRW = rotation.w;
		}

		public Vector3 GetTargetPosition()
		{
			return new Vector3((float)posTPX, (float)posTPY, (float)posTPZ);
		}

		public void SetMaximumError(double units)
		{
			MaximumPositionError = units;
		}

		public double GetMaximumError()
		{
			return MaximumPositionError;
		}

	}

}
*/