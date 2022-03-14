
using UnityEditor;
using UnityEngine;

namespace BIOIK2
{
    internal class Utility
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
    }
}