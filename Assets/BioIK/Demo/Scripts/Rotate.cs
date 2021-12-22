using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotate : MonoBehaviour {

	public float Speed = 100f;

	void Update () {
		transform.rotation *= Quaternion.Euler(0f, -Speed*Time.deltaTime, 0f);
	}

}
