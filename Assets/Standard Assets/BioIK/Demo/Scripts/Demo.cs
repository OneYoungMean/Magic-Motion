using UnityEngine;
using UnityEngine.UI;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class Demo : MonoBehaviour {

	public BIOIK.BioIK[] Models;

	public GameObject[] IgnoreRealistic;

	public BIOIK.MotionType MotionType = BIOIK.MotionType.Instantaneous;

	public Dropdown Dropdown;

	private BIOIK.BioIK ModelCharacter = null;
	private GameObject ModelGO = null;

	void Start() {
		LoadModel(1);
	}

	public void LoadModel(int index) {
		index -= 1;
		if(Models.Length > index) {
			if(ModelGO != null) {
				ModelGO.SetActive(false);
			}
			ModelCharacter = Models[index];
			ModelGO = ModelCharacter.transform.root.gameObject;
			ModelGO.SetActive(true);
			UpdateMotionType();
		}
	}

	public bool Ignore(GameObject go) {
		for(int i=0; i<IgnoreRealistic.Length; i++) {
			if(IgnoreRealistic[i] == go) {
				return true;
			}
		}
		return false;
	}

	public void UpdateMotionType() {
		MotionType = Dropdown.value == 0 ? BIOIK.MotionType.Instantaneous : BIOIK.MotionType.Realistic;
		if(Ignore(ModelGO)) {
			ModelCharacter.MotionType = BIOIK.MotionType.Instantaneous;
		} else {
			ModelCharacter.MotionType = MotionType;
		}
	}
}

#if UNITY_EDITOR
[CustomEditor(typeof(Demo))]
public class DemoEditor : Editor {

	public Demo Target;

	void Awake() {
		Target = (Demo)target;
	}

	public override void OnInspectorGUI() {
		DrawDefaultInspector();

		if(GUILayout.Button("Enable Threading")) {
			SetThreading(true);
		}
		if(GUILayout.Button("Disable Threading")) {
			SetThreading(false);
		}
	}

	private void SetThreading(bool value) {
		for(int i=0; i<Target.Models.Length; i++) {
			Target.Models[i].SetThreading(value);
		}
	}
}
#endif