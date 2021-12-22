using UnityEngine;
using UnityEditor;

namespace BIOIK {
	[CustomEditor(typeof(BioIK))]
	public class BioIKEditor : Editor {

		public BioIK Target;
		public Transform TargetTransform;

		private Color Color1 = Color.white;
		//private Color Color2 = Color.black;
		private Color Color3 = new Color(0.6f, 0.6f, 0.6f, 1f);
		private Color Color4 = new Color(0.3f, 0.8f, 0.8f, 1f);
		private Color Color5 = new Color(1f, 0.7f, 0.3f, 1f);
		private Color Color6 = new Color(0.9f, 0.3f, 0.9f, 1f);
		private Color Color7 = new Color(1.0f, 0.3f, 0.3f, 1f);
		private Color Color8 = new Color(0.3f, 0.6f, 0.6f, 1f);
		private Color Color9 = new Color(0.4f, 0.9f, 0.4f, 1f);	
		//private Color Color10 = new Color(1f, 0.5f, 1f, 1f);
		private Color Color11 = new Color(0.3f, 0.3f, 0.3f, 1f);
		//private Color Color12 = new Color(0.9f, 0.6f, 0.9f, 1f);
		private Color Color13 = new Color(0.5f, 0.5f, 0.5f, 1f);
		private Color Color14 = new Color(0.75f, 0.75f, 0.75f, 1f);

		private bool ChosingObjectiveType = false;

		private bool IsPlaying = false;
		private bool IsEnabled = false;

		private int DoF;

		void Awake() {
			EditorApplication.playmodeStateChanged += PlaymodeStateChanged;
			Target = (BioIK)target;
			TargetTransform = Target.transform;
			Target.Refresh(false);
			DoF = 0;
			//MakeVisible(TargetTransform);
			//MakeInvisible(TargetTransform);
		}

		void OnEnable() {
			IsEnabled = true;
			MakeVisible(TargetTransform); 
		}

		void OnDisable() {
			IsEnabled = false;
		}

		private void MakeVisible(Transform t) {
			if(t.GetComponent<BioSegment>()) {
				t.GetComponent<BioSegment>().hideFlags = HideFlags.None;
			}
			foreach(BioObjective o in t.GetComponents<BioObjective>()) {
				o.hideFlags = HideFlags.None;
			}
			if(t.GetComponent<BioJoint>()) {
				t.GetComponent<BioJoint>().hideFlags = HideFlags.None;
			}
			for(int i=0; i<t.childCount; i++) {
				MakeVisible(t.GetChild(i));
			}
		}

		private void MakeInvisible(Transform t) {
			if(t.GetComponent<BioSegment>()) {
				t.GetComponent<BioSegment>().hideFlags = HideFlags.HideInInspector;
			}
			foreach(BioObjective o in t.GetComponents<BioObjective>()) {
				o.hideFlags = HideFlags.HideInInspector;
			}
			if(t.GetComponent<BioJoint>()) {
				t.GetComponent<BioJoint>().hideFlags = HideFlags.HideInInspector;
			}
			for(int i=0; i<t.childCount; i++) {
				MakeInvisible(t.GetChild(i));
			}
		}

		void OnDestroy() {
			if(
				Target == null 
				&& 
				TargetTransform != null 
				&& 
				!IsPlaying 
				&& 
				!IsEnabled
				&&
				!EditorApplication.isPlayingOrWillChangePlaymode
				) {
				Utility.Cleanup(TargetTransform);
			}
		}

		private void PlaymodeStateChanged() {
			IsPlaying = Application.isPlaying;
		}

		public override void OnInspectorGUI() {
			Undo.RecordObject(Target, Target.name);

			SetGUIColor(Color3);
			using(new EditorGUILayout.VerticalScope ("Button")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Settings                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				//SetGUIColor(Color1);
				//Target.SolveInEditMode = EditorGUILayout.Toggle("Solve In Edit Mode", Target.SolveInEditMode);
				SetGUIColor(Color1);
				Target.SetThreading(EditorGUILayout.Toggle("Use Threading", Target.GetThreading()));
				SetGUIColor(Color1);
				Target.SetGenerations(EditorGUILayout.IntField("Generations", Target.GetGenerations()));
				SetGUIColor(Color1);
				Target.SetPopulationSize(EditorGUILayout.IntField("Individuals", Target.GetPopulationSize()));
				SetGUIColor(Color1);
				Target.SetElites(EditorGUILayout.IntField("Elites", Target.GetElites()));
				SetGUIColor(Color1);
				Target.Smoothing = EditorGUILayout.Slider("Smoothing", Target.Smoothing, 0f, 1f);
				SetGUIColor(Color1);
				Target.AnimationWeight = EditorGUILayout.Slider("Animation Weight", Target.AnimationWeight, 0f, 1f);
				SetGUIColor(Color1);
				Target.AnimationBlend = EditorGUILayout.Slider("Animation Blend", Target.AnimationBlend, 0f, 1f);
				SetGUIColor(Color1);
				Target.MotionType = (MotionType)EditorGUILayout.EnumPopup("Motion Type", Target.MotionType);
				if(Target.MotionType == MotionType.Realistic) {
					Target.MaximumVelocity = EditorGUILayout.FloatField("Maximum Velocity", Target.MaximumVelocity);
					Target.MaximumAcceleration = EditorGUILayout.FloatField("Maximum Acceleration", Target.MaximumAcceleration);
				}
				SetGUIColor(Color1);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				if(GUILayout.Button("          Reset Posture          ")) {
					Target.ResetPosture(Target.Root);
				}
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}

			SetGUIColor(Color3);
			using(new EditorGUILayout.VerticalScope ("Button")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                   Character                   ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				//SetGUIColor(Color5);
				//EditorGUILayout.HelpBox("Degree of Freedom: " + new Model(Target).GetDoF() + " / " + DoF, MessageType.None);

				int maxIndent = 0;
				ComputeMaxIndentLevel(Target.transform, 0, ref maxIndent);

				Target.Scroll = EditorGUILayout.BeginScrollView(Target.Scroll, GUILayout.Height(500f));
				InspectBody(Target.FindSegment(Target.transform), 0, maxIndent);
				EditorGUILayout.EndScrollView();
			}

			SetGUIColor(Color1);
			if(Target != null) {
				EditorUtility.SetDirty(Target);
			}
		}

		private void InspectBody(BioSegment segment, int indent, int maxIndent) {
			SetGUIColor(Color11);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				EditorGUILayout.BeginHorizontal();
				
				if(Target.SelectedSegment != segment) {
					if(indent > 0) {
						SetGUIColor(Color13);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							int width = 10*(indent-1);
							EditorGUILayout.LabelField("", GUILayout.Width(width));
						}
					}
				}

				GUI.skin.button.alignment = TextAnchor.MiddleLeft;
				if(segment == Target.SelectedSegment) {
					SetGUIColor(Color5);
				} else {
					SetGUIColor(Color.Lerp(Color4, Color8, (float)indent / (float)maxIndent));
				}
				if(GUILayout.Button(segment.Transform.name, GUILayout.Height(25f), GUILayout.ExpandWidth(true))) {
					if(Target.SelectedSegment == segment) {
						Target.SelectedSegment = null;
						ChosingObjectiveType = false;
					} else {
						Target.SelectedSegment = segment;
					}
				}

				EditorGUILayout.EndHorizontal();

				if(Target.SelectedSegment == segment) {
					InspectSegment(segment);
				} else {
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(segment.Joint != null) {
						SetGUIColor(Color6);
						GUILayout.Box(" Joint ");
					}
					foreach(BioObjective objective in segment.Objectives) {
						SetGUIColor(Color9);
						GUILayout.Box(" " + objective.GetObjectiveType().ToString() + " ");
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				}
			}
			
			for(int i=0; i<segment.Childs.Length; i++) {
				InspectBody(segment.Childs[i], indent+1, maxIndent);
			}
		}

		private void InspectSegment(BioSegment segment) {
			Undo.RecordObject(segment, segment.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				
				SetGUIColor(Color4);
				Vector3 A = segment.Parent == null ? segment.GetAnchoredPosition() : segment.Parent.GetAnchoredPosition();
				Vector3 B = segment.GetAnchoredPosition();
				EditorGUILayout.HelpBox("Link Length: " + Vector3.Distance(A,B), MessageType.None);

				SetGUIColor(Color6);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					if(segment.Joint == null) {
						GUI.skin.button.alignment = TextAnchor.MiddleCenter;
						SetGUIColor(Color1);
						if(GUILayout.Button("Add Joint")) {
							segment.AddJoint();
						}
					} else {
						InspectJoint(segment.Joint);
					}
				}

			}

			for(int i=0; i<segment.Objectives.Length; i++) {
				SetGUIColor(Color13);
				using(new EditorGUILayout.VerticalScope ("Box")) {

					SetGUIColor(Color9);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						InspectObjective(segment.Objectives[i]);
					}

				}
			}

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {

				SetGUIColor(Color9);
				using(new EditorGUILayout.VerticalScope ("Box")) {

					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(ChosingObjectiveType ? Color14 : Color1);
					if(GUILayout.Button("Add Objective")) {
						ChosingObjectiveType = !ChosingObjectiveType;
					}
					if(ChosingObjectiveType) {
						SetGUIColor(Color8);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							int count = System.Enum.GetValues(typeof(ObjectiveType)).Length;
							string[] names = System.Enum.GetNames(typeof(ObjectiveType));
							for(int i=0; i<count; i++) {
								SetGUIColor(Color1);
								if(GUILayout.Button(names[i])) {
									ChosingObjectiveType = false;
									segment.AddObjective((ObjectiveType)i);
								}
							}
						}
					}
				}

			}
			
			if(segment != null) {
				EditorUtility.SetDirty(segment);
			}
		}

		private void InspectJoint(BioJoint joint) {
			Undo.RecordObject(joint, joint.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Joint                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				SetGUIColor(Color1);
				joint.enabled = EditorGUILayout.Toggle("Enabled", joint.enabled);

				SetGUIColor(Color4);
				EditorGUILayout.HelpBox("Geometry", MessageType.None);
				SetGUIColor(Color1);
				joint.JointType = (JointType)EditorGUILayout.EnumPopup("Joint Type", joint.JointType);
				SetGUIColor(Color1);
				joint.SetAnchor(EditorGUILayout.Vector3Field("Anchor", joint.GetAnchor()));
				SetGUIColor(Color1);
				joint.SetOrientation(EditorGUILayout.Vector3Field("Orientation", joint.GetOrientation()));
				SetGUIColor(Color4);
				EditorGUILayout.HelpBox("Default Frame", MessageType.None);
				SetGUIColor(Color1);
				Vector3 defaultPosition = EditorGUILayout.Vector3Field("Position", joint.GetDefaultPosition());
				SetGUIColor(Color1);
				Quaternion defaultRotation = Quaternion.Euler(EditorGUILayout.Vector3Field("Rotation", joint.GetDefaultRotation().eulerAngles));
				joint.SetDefaultFrame(defaultPosition, defaultRotation);

				InspectMotion(joint.X, "     X Motion     ");
				InspectMotion(joint.Y, "     Y Motion     ");
				InspectMotion(joint.Z, "     Z Motion     ");

				GUI.skin.button.alignment = TextAnchor.MiddleCenter;
				SetGUIColor(Color7);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				if(GUILayout.Button("Remove", GUILayout.Width(100f))) {
					joint.Remove();
				}
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}

			if(joint != null) {
				joint.PrecaptureAnimation();
				joint.PostcaptureAnimation();
				joint.UpdateData();
				joint.ProcessMotion();
				EditorUtility.SetDirty(joint);
			}
		}

		private void InspectMotion(BioJoint.Motion motion, string name) {
			SetGUIColor(Color8);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox(name, MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				if(motion.IsEnabled()) {
					SetGUIColor(Color1);
					motion.Constrained = EditorGUILayout.Toggle("Constrained", motion.Constrained);
					if(motion.Constrained) {
						SetGUIColor(Color1);
						motion.SetLowerLimit(EditorGUILayout.DoubleField("Lower Limit", motion.GetLowerLimit()));
						SetGUIColor(Color1);
						motion.SetUpperLimit(EditorGUILayout.DoubleField("Upper Limit", motion.GetUpperLimit()));
						SetGUIColor(Color1);
						motion.SetTargetValue(EditorGUILayout.Slider("Target Value", (float)motion.GetTargetValue(), (float)motion.GetLowerLimit(), (float)motion.GetUpperLimit()));
					} else {
						SetGUIColor(Color1);
						motion.SetTargetValue(EditorGUILayout.DoubleField("Target Value", motion.GetTargetValue()));
					}
					
					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(Color1);
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(GUILayout.Button("Disable", GUILayout.Width(250f), GUILayout.Height(20f))) {
						motion.SetEnabled(false);
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				} else {
					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(Color1);
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(GUILayout.Button("Enable", GUILayout.Width(250f), GUILayout.Height(20f))) {
						motion.SetEnabled(true);
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				}
			}
		}

		private void InspectObjective(BioObjective objective) {
			Undo.RecordObject(objective, objective.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Objective (" + objective.GetObjectiveType().ToString() + ")                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
				
				SetGUIColor(Color1);
				objective.enabled = EditorGUILayout.Toggle("Enabled", objective.enabled);

				SetGUIColor(Color1);
				objective.Weight = EditorGUILayout.DoubleField("Weight", objective.Weight);

				switch(objective.GetObjectiveType()) {
					case ObjectiveType.Position:
					InspectPosition((Position)objective);
					break;

					case ObjectiveType.Orientation:
					InspectOrientation((Orientation)objective);
					break;

					case ObjectiveType.LookAt:
					InspectLookAt((LookAt)objective);
					break;

					case ObjectiveType.Distance:
					InspectDistance((Distance)objective);
					break;

					case ObjectiveType.JointValue:
					InspectJointValue((JointValue)objective);
					break;

					case ObjectiveType.Displacement:
					InspectDisplacement((Displacement)objective);
					break;

					case ObjectiveType.Projection:
					InspectProjection((Projection)objective);
					break;

					default:
						break;
				}

				GUI.skin.button.alignment = TextAnchor.MiddleCenter;
				SetGUIColor(Color7);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				if(GUILayout.Button("Remove", GUILayout.Width(100f))) {
					objective.Remove();
				}
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}

			if(objective != null) {
				EditorUtility.SetDirty(objective);
			}
		}

		private void InspectPosition(Position objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetPosition(EditorGUILayout.Vector3Field("Target Position", objective.GetTargetPosition()));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		private void InspectOrientation(Orientation objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetRotation(EditorGUILayout.Vector3Field("Target Rotation", objective.GetTargetRotattion()));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		private void InspectLookAt(LookAt objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetPosition(EditorGUILayout.Vector3Field("Target Position", objective.GetTargetPosition()));
			SetGUIColor(Color1);
			objective.SetViewingDirection(EditorGUILayout.Vector3Field("Viewing Direction", objective.GetViewingDirection()));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		private void InspectDistance(Distance objective) {
			SetGUIColor(Color1);
			objective.SetRadius(EditorGUILayout.DoubleField("Radius", objective.GetRadius()));

			SetGUIColor(Color1);
			GUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorGUILayout.HelpBox("Points", MessageType.None);
			GUILayout.FlexibleSpace();
			GUILayout.EndHorizontal();

			DistancePoint[] points = objective.GetPoints();
			for(int i=0; i<points.Length; i++) {
				SetGUIColor(Color8);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					SetGUIColor(Color1);
					points[i].SetTargetTransform(EditorGUILayout.ObjectField("Target", points[i].Target, typeof(Transform), true) as Transform);
					SetGUIColor(Color1);
					points[i].SetRadius(EditorGUILayout.DoubleField("Radius", points[i].Radius));
				}
			}
			SetGUIColor(Color8);
			if(GUILayout.Button("+")) {
				objective.AddPoint();
			}
			SetGUIColor(Color8);
			if(GUILayout.Button("-")) {
				objective.RemovePoint();
			}
		}

		private void InspectJointValue(JointValue objective) {
			SetGUIColor(Color1);
			objective.SetTargetValue(EditorGUILayout.DoubleField("Target Value", objective.GetTargetValue()));
			SetGUIColor(Color1);
			objective.SetXMotion(EditorGUILayout.Toggle("X Motion", objective.IsXMotion()));
			SetGUIColor(Color1);
			objective.SetYMotion(EditorGUILayout.Toggle("Y Motion", objective.IsYMotion()));
			SetGUIColor(Color1);
			objective.SetZMotion(EditorGUILayout.Toggle("Z Motion", objective.IsZMotion()));
		}

		private void InspectDisplacement(Displacement objective) {

		}

		private void InspectProjection(Projection objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetPosition(EditorGUILayout.Vector3Field("Target Position", objective.GetTargetPosition()));
			SetGUIColor(Color1);
			objective.SetTargetRotation(EditorGUILayout.Vector3Field("Target Rotation", objective.GetTargetRotation()));
			SetGUIColor(Color1);
			objective.SetNormal(EditorGUILayout.Vector3Field("Projection Normal", objective.GetNormal()));
			SetGUIColor(Color1);
			objective.SetLength(EditorGUILayout.FloatField("Projection Length", objective.GetLength()));
			SetGUIColor(Color1);
			objective.SetSensitivity(EditorGUILayout.Slider("Projection Sensitivitiy", objective.GetSensitivity(), 0f, 1f));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		public void OnSceneGUI() {
			DoF = 0;
			DrawSkeleton(Target.FindSegment(Target.transform));
			DrawSetup(Target.FindSegment(Target.transform), false);
			if(Target.SelectedSegment != null) {
				DrawSegment(Target.SelectedSegment, true);
			}
		}

		private bool StopDraw(BioSegment segment) {
			if(segment.Parent == null) {
				return false;
			}
			return false;
			//return segment.Parent.name == "Kopf" && segment.name != "Head" && segment.name != "Ohr_R" && segment.name != "Ohr_L" || segment.name == "Ohr_L_end" || segment.name == "Ohr_R_end";
		}

		private void DrawSkeleton(BioSegment segment) {
			if(!StopDraw(segment)) {
			if(segment.Parent != null) {
				DrawLine(segment.Parent.GetAnchoredPosition(), segment.GetAnchoredPosition(), 5f, Color.cyan);
			}
			for(int i=0; i<segment.Childs.Length; i++) {
				DrawSkeleton(segment.Childs[i]);
			}
			}
		}

		private void DrawSetup(BioSegment segment, bool final) {
			if(!StopDraw(segment)) {
			DrawSegment(segment, final);
			for(int i=0; i<segment.Childs.Length; i++) {
				DrawSetup(segment.Childs[i], final);
			}
			}
		}

		private void DrawSegment(BioSegment segment, bool final) {
			Vector3 P = segment.GetAnchoredPosition();
			if(Target.SelectedSegment == segment && final) {
				DrawSphere(P, 0.25f, new Color(0f, 0f, 0f, 0.4f));
				DrawSphere(P, 0.02f, Color5);
			} else if(Target.SelectedSegment != segment) {
				DrawSphere(P, 0.02f, Color.cyan);
			}
			if(segment.Joint != null) {
				DrawJoint(segment.Joint, final);
			}
			for(int i=0; i<segment.Objectives.Length; i++) {
				DrawObjective(segment.Objectives[i], final);
			}
		}

		private void DrawJoint(BioJoint joint, bool final) {
			if(!final) {
				DoF += joint.GetDoF();
			}
			//DrawDottedLine(joint.Segment.Transform.position, joint.GetAnchorInWorldSpace(), 5f, Color.magenta);
			DrawCube(joint.GetAnchorInWorldSpace(), joint.Segment.Transform.rotation * Quaternion.Euler(joint.GetOrientation()), 0.025f, Color.magenta);
			DrawMotion(joint.X, Color.red, final);
			DrawMotion(joint.Y, Color.green, final);
			DrawMotion(joint.Z, Color.blue, final);
		}

		private void DrawMotion(BioJoint.Motion motion, Color color, bool final) {
			if(Target.SelectedSegment == motion.Joint.Segment && final) {
				DrawArrow(motion.Joint.GetAnchorInWorldSpace(), motion.Joint.Segment.Transform.rotation * Quaternion.LookRotation(motion.Axis), 0.125f, motion.IsEnabled() ? color : Color.grey);
				
				if(!motion.IsEnabled() || !motion.Constrained) {
					return;
				}

				if(motion.Joint.JointType == JointType.Rotational) {
					if(motion == motion.Joint.X) {
						DrawSolidArc(
							motion.Joint.GetAnchorInWorldSpace(), 
							motion.Joint.Segment.Transform.rotation * motion.Axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), motion.Joint.Segment.Transform.rotation * motion.Axis) * motion.Joint.Segment.Transform.rotation * motion.Joint.Y.Axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new Color(1f, 0f, 0f, 0.25f)
							);
					}
					if(motion == motion.Joint.Y) {
						DrawSolidArc(
							motion.Joint.GetAnchorInWorldSpace(), 
							motion.Joint.Segment.Transform.rotation * motion.Axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), motion.Joint.Segment.Transform.rotation * motion.Axis) * motion.Joint.Segment.Transform.rotation * motion.Joint.Z.Axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new Color(0f, 1f, 0f, 0.25f)
							);
					}
					if(motion == motion.Joint.Z) {
						DrawSolidArc(
							motion.Joint.GetAnchorInWorldSpace(), 
							motion.Joint.Segment.Transform.rotation * motion.Axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), motion.Joint.Segment.Transform.rotation * motion.Axis) * motion.Joint.Segment.Transform.rotation * motion.Joint.X.Axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new Color(0f, 0f, 1f, 0.25f)
							);
					}
				}

				if(motion.Joint.JointType == JointType.Translational) {
					Vector3 A = motion.Joint.GetAnchorInWorldSpace() + (float)motion.GetLowerLimit() * (motion.Joint.Segment.Transform.rotation * motion.Axis);
					Vector3 B = motion.Joint.GetAnchorInWorldSpace() + (float)motion.GetUpperLimit() * (motion.Joint.Segment.Transform.rotation * motion.Axis);
					Color c = Color.white;
					if(motion == motion.Joint.X) {
						c = Color.red;
					}
					if(motion == motion.Joint.Y) {
						c = Color.green;
					}
					if(motion == motion.Joint.Z) {
						c = Color.blue;
					}
					DrawLine(A,	B, 3f, c);
					DrawCube(A, motion.Joint.Segment.Transform.rotation, 0.0125f, new Color(c.r, c.g, c.b, 0.5f));
					DrawCube(B, motion.Joint.Segment.Transform.rotation, 0.0125f, new Color(c.r, c.g, c.b, 0.5f));
				}

			} else if(Target.SelectedSegment != motion.Joint.Segment) {
				DrawArrow(motion.Joint.GetAnchorInWorldSpace(), motion.Joint.Segment.Transform.rotation * Quaternion.LookRotation(motion.Axis), 0.05f, motion.IsEnabled() ? color : Color.clear);
			}
		}

		private void DrawObjective(BioObjective objective, bool selected) {
			if(!selected) {
				return;
			}

			switch(objective.GetObjectiveType()) {
				case ObjectiveType.Position:
				DrawPosition((Position)objective);
				break;

				case ObjectiveType.Orientation:
				DrawOrientation((Orientation)objective);
				break;

				case ObjectiveType.LookAt:
				DrawLookAt((LookAt)objective);
				break;

				case ObjectiveType.Distance:
				DrawDistance((Distance)objective);
				break;

				case ObjectiveType.JointValue:
				DrawJointValue((JointValue)objective);
				break;

				case ObjectiveType.Displacement:
				DrawDisplacement((Displacement)objective);
				break;

				case ObjectiveType.Projection:
				DrawProjection((Projection)objective);
				break;
			}
		}

		private void DrawPosition(Position objective) {
			DrawSphere(objective.GetTargetPosition(), 0.1f, new Color(1f, 0f, 0f, 0.75f));
			Handles.Label(objective.GetTargetPosition(), "Target");
		}

		private void DrawOrientation(Orientation objective) {
			Quaternion rotation = Quaternion.Euler(objective.GetTargetRotattion());
			Vector3 right = rotation * Vector3.right;
			Vector3 up = rotation * Vector3.up;
			Vector3 forward = rotation * Vector3.forward;
			float length = 0.1f;
			DrawLine(objective.Segment.Transform.position - length * right, objective.Segment.Transform.position + length * right, 5f, new Color(1f, 0f, 0f, 0.75f));
			DrawLine(objective.Segment.Transform.position - length * up, objective.Segment.Transform.position + length * up, 5f, new Color(0f, 1f, 0f, 0.75f));
			DrawLine(objective.Segment.Transform.position - length * forward, objective.Segment.Transform.position + length * forward, 5f, new Color(0f, 0f, 1f, 0.75f));
			Handles.Label(objective.Segment.Transform.position, "Target");
		}

		private void DrawLookAt(LookAt objective) {
			if(objective.GetViewingDirection().magnitude != 0f) {
				DrawArrow(objective.transform.position, objective.transform.rotation*Quaternion.LookRotation((objective.GetViewingDirection())), 0.125f, Color5);
			}
		}

		private void DrawDistance(Distance objective) {
			DrawSphere(objective.transform.position, (float)objective.GetRadius(), new Color(0f, 1f, 0f, 0.5f));
			DistancePoint[] points = objective.GetPoints();
			for(int i=0; i<points.Length; i++) {
				DistancePoint point = points[i];
				if(point != null) {
					DrawSphere(point.GetTargetPoint(), (float)point.GetRadius(), new Color(1f, 0f, 0f, 0.5f));
				}
			}
		}

		private void DrawJointValue(JointValue objective) {

		}

		private void DrawDisplacement(Displacement objective) {

		}

		private void DrawProjection(Projection objective) {
			Vector3 normal = objective.Segment.Transform.rotation * objective.GetNormal();
			Vector3 start = objective.Segment.Transform.position - objective.GetLength() * normal;
			Vector3 end = start + objective.GetLength() * normal;

			DrawSphere(start, 0.025f, Color5);
			DrawArrow(start, Quaternion.FromToRotation(Vector3.forward, end-start), objective.GetLength(), Color5);

			DrawSphere(objective.GetTargetPosition(), 0.025f, Color.red);

			/*
			Quaternion rotation = Quaternion.Euler(objective.GetTargetRotation());
			Vector3 right = rotation * Vector3.right;
			Vector3 up = rotation * Vector3.up;
			Vector3 forward = rotation * Vector3.forward;

			//float length = 0.05f;
			
			//DrawLine(objective.Segment.Transform.position - length * right, objective.Segment.Transform.position + length * right, 5f, Color.red);
			//DrawLine(objective.Segment.Transform.position - length * up, objective.Segment.Transform.position + length * up, 5f, Color.green);
			//DrawLine(objective.Segment.Transform.position - length * forward, objective.Segment.Transform.position + length * forward, 5f, Color.blue);
			
			//for(int i=0; i<Corrections.Length; i++) {
			//	normal = transform.rotation * Corrections[i].Direction;
			//	start = transform.position + transform.rotation * Corrections[i].Offset;
			//	end = start + Corrections[i].Length * normal;

			//	Gizmos.color = Color.cyan;
			//	Gizmos.DrawSphere(start, 0.025f);
			//	Gizmos.DrawLine(start, end);
			//	Gizmos.DrawSphere(end, 0.025f);
			//}
			*/
		}

		private void DrawSphere(Vector3 position, float radius, Color color) {
			Handles.color = color;
			Handles.SphereHandleCap(0, position, Quaternion.identity, radius, EventType.Repaint);
		}

		private void DrawCube(Vector3 position, Quaternion rotation, float size, Color color) {
			Handles.color = color;
			Handles.CubeHandleCap(0, position, rotation, size, EventType.Repaint);
		}

		private void DrawLine(Vector3 a, Vector3 b, float width, Color color) {
			Handles.color = color;
			Handles.DrawAAPolyLine(width, new Vector3[2] {a,b});
		}

		private void DrawDottedLine(Vector3 a, Vector3 b, float width, Color color) {
			Handles.color = color;
			Handles.DrawDottedLine(a, b, width);
		}

		private void DrawArrow(Vector3 position, Quaternion rotation, float length, Color color) {
			Handles.color = color;
			Handles.ArrowHandleCap(0, position, rotation, length, EventType.Repaint);
		}

		private void DrawSolidArc(Vector3 position, Vector3 normal, Vector3 from, float angle, float radius, Color color) {
			Handles.color = color;
			Handles.DrawSolidArc(position, normal, from, angle, radius);
		}

		private void SetGUIColor(Color color) {
			GUI.backgroundColor = color;
		}

		private void ComputeMaxIndentLevel(Transform t, int level, ref int maxIndent) {
			maxIndent = System.Math.Max(maxIndent, level);
			for(int i=0; i<t.childCount; i++) {
				ComputeMaxIndentLevel(t.GetChild(i), level+1, ref maxIndent);
			}
		}

	}
}
