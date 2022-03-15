
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;

namespace BIOIK2
{
	internal unsafe static class Utility
	{
		internal static void Cleanup(Transform transform)
		{
			foreach (BioJoint joint in transform.GetComponentsInChildren<BioJoint>())
			{
				joint.Erase();
			}
			foreach (BioObjective objective in transform.GetComponentsInChildren<BioObjective>())
			{
				objective.Erase();
			}
			foreach (BioSegment segment in transform.GetComponentsInChildren<BioSegment>())
			{
				Destroy(segment);
			}
		}

		internal static void Destroy(Component c)
		{
			if (c == null)
			{
				return;
			}
#if UNITY_EDITOR
			if (Application.isPlaying)
			{
				Object.Destroy(c);
			}
			else if (!EditorApplication.isPlayingOrWillChangePlaymode)
			{
				Undo.DestroyObjectImmediate(c);
			}
#else
			Object.Destroy(c);
#endif
		}

		internal static System.DateTime GetTimestamp()
		{
			throw new System.NotImplementedException();
		}

		internal static float GetElapsedTime(System.DateTime timestamp)
		{
			return (float)(System.DateTime.Now - timestamp).Duration().TotalSeconds;
		}

		internal static float[] ToFloatArray(this float3[] values)
        {
			var result = new float[values.Length * 3];
			for (int i = 0; i < values.Length; i++)
			{
				result[i * 3] = values[i].x;
				result[i * 3 + 1] = values[i].y;
				result[i * 3 + 2] = values[i].z;
			}
			return result;
        }
		internal static float3[] ToFloat3Array(this float[] values)
		{
			var result = new float3[values.Length / 3];
			for (int i = 0; i < values.Length; i++)
			{
				result[i].x = values[i * 3];
				result[i].y = values[i * 3 + 1];
				result[i].z = values[i * 3 + 2];
			}
			return result;
		}
	}
}