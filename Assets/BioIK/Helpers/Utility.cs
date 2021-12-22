using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace BIOIK {

	public static class Utility {
		public const double Deg2Rad = 0.017453292;
		public const double Rad2Deg = 57.29578049;
		public const double PI = 3.14159265358979;

		public static void Destroy(Component c) {
			if(c == null) {
				return;
			}
			#if UNITY_EDITOR
			if(Application.isPlaying) {
				Object.Destroy(c);
			} else if(!EditorApplication.isPlayingOrWillChangePlaymode) {
				Undo.DestroyObjectImmediate(c);
			}
			#else
			Object.Destroy(c);
			#endif
		}

		public static Vector3 DAZ2Unity(Vector3 angles) {
			return (Quaternion.Euler(180f, 0f, 0f) * Quaternion.Euler(angles.x, angles.y, angles.z)).eulerAngles;
		}

		public static Vector3 Unity2DAZ(Vector3 angles) {
			return (Quaternion.Euler(-180f, 0f, 0f) * Quaternion.Euler(angles.x, angles.y, angles.z)).eulerAngles;
		}

		public static System.DateTime GetTimestamp() {
			return System.DateTime.Now;
		}

		public static double GetElapsedTime(System.DateTime timestamp) {
			return (System.DateTime.Now-timestamp).Duration().TotalSeconds;
		}

		public static void Cleanup(Transform t) {
			/*
			BioSegment segment = t.GetComponent<BioSegment>();
			if(segment != null) {
				if(segment.Joint != null) {
					segment.Joint.Remove();
				}
				foreach(BioObjective objective in segment.Objectives) {
					objective.Remove();
				}
				Destroy(segment);
			}
			*/
			
			foreach(BioJoint joint in t.GetComponents<BioJoint>()) {
				joint.Erase();
			}
			foreach(BioObjective objective in t.GetComponents<BioObjective>()) {
				objective.Erase();
			}
			foreach(BioSegment segment in t.GetComponents<BioSegment>()) {
				Destroy(segment);
			}
			
			for(int i=0; i<t.childCount; i++) {
				Cleanup(t.GetChild(i));
			}
		}

		public static BioSegment AddBioSegment(BioIK character, Transform t) {
			#if UNITY_EDITOR
			if(Application.isPlaying) {
				return (t.gameObject.AddComponent(typeof(BioSegment)) as BioSegment).Create(character);
			} else {
				return (Undo.AddComponent(t.gameObject, typeof(BioSegment)) as BioSegment).Create(character);
			}
			#else
			return (t.gameObject.AddComponent(typeof(BioSegment)) as BioSegment).Create(character);
			#endif
		}

		public static BioJoint AddBioJoint(BioSegment segment) {
			#if UNITY_EDITOR
			if(Application.isPlaying) {
				return (segment.gameObject.AddComponent(typeof(BioJoint)) as BioJoint).Create(segment);
			} else {
				return (Undo.AddComponent(segment.gameObject, typeof(BioJoint)) as BioJoint).Create(segment);
			}
			#else
			return (segment.gameObject.AddComponent(typeof(BioJoint)) as BioJoint).Create(segment);
			#endif
		}

		public static BioObjective AddObjective(BioSegment segment, ObjectiveType type) {
			#if UNITY_EDITOR
			if(Application.isPlaying) {
				switch(type) {
					case ObjectiveType.Position:
					return (segment.gameObject.AddComponent(typeof(Position)) as BioObjective).Create(segment);

					case ObjectiveType.Orientation:
					return (segment.gameObject.AddComponent(typeof(Orientation)) as BioObjective).Create(segment);

					case ObjectiveType.LookAt:
					return (segment.gameObject.AddComponent(typeof(LookAt)) as BioObjective).Create(segment);;

					case ObjectiveType.Distance:
					return (segment.gameObject.AddComponent(typeof(Distance)) as BioObjective).Create(segment);

					case ObjectiveType.JointValue:
					return (segment.gameObject.AddComponent(typeof(JointValue)) as BioObjective).Create(segment);

					case ObjectiveType.Displacement:
					return (segment.gameObject.AddComponent(typeof(Displacement)) as BioObjective).Create(segment);

					case ObjectiveType.Projection:
					return (segment.gameObject.AddComponent(typeof(Projection)) as BioObjective).Create(segment);
				}
			} else {
				switch(type) {
					case ObjectiveType.Position:
					return (Undo.AddComponent(segment.gameObject, typeof(Position)) as BioObjective).Create(segment);

					case ObjectiveType.Orientation:
					return (Undo.AddComponent(segment.gameObject, typeof(Orientation)) as BioObjective).Create(segment);

					case ObjectiveType.LookAt:
					return (Undo.AddComponent(segment.gameObject, typeof(LookAt)) as BioObjective).Create(segment);

					case ObjectiveType.Distance:
					return (Undo.AddComponent(segment.gameObject, typeof(Distance)) as BioObjective).Create(segment);

					case ObjectiveType.JointValue:
					return (Undo.AddComponent(segment.gameObject, typeof(JointValue)) as BioObjective).Create(segment);

					case ObjectiveType.Displacement:
					return (Undo.AddComponent(segment.gameObject, typeof(Displacement)) as BioObjective).Create(segment);

					case ObjectiveType.Projection:
					return (Undo.AddComponent(segment.gameObject, typeof(Projection)) as BioObjective).Create(segment);
				}
			}
			return null;
			#else
			switch(type) {
				case ObjectiveType.Position:
				return (segment.gameObject.AddComponent(typeof(Position)) as BioObjective).Create(segment);

				case ObjectiveType.Orientation:
				return (segment.gameObject.AddComponent(typeof(Orientation)) as BioObjective).Create(segment);

				case ObjectiveType.LookAt:
				return (segment.gameObject.AddComponent(typeof(LookAt)) as BioObjective).Create(segment);;

				case ObjectiveType.Distance:
				return (segment.gameObject.AddComponent(typeof(Distance)) as BioObjective).Create(segment);

				case ObjectiveType.JointValue:
				return (segment.gameObject.AddComponent(typeof(JointValue)) as BioObjective).Create(segment);

				case ObjectiveType.Displacement:
				return (segment.gameObject.AddComponent(typeof(Displacement)) as BioObjective).Create(segment);

				case ObjectiveType.Projection:
				return (segment.gameObject.AddComponent(typeof(Projection)) as BioObjective).Create(segment);
			}
			return null;
			#endif
		}

	}

}