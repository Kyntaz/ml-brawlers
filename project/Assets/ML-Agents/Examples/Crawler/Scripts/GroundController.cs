using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GroundController : MonoBehaviour {

	[HideInInspector]
	public Vector3 originalPosition;

	[HideInInspector]
	public Rigidbody rb;

	// Use this for initialization
	void Start () {
		this.rb = GetComponent<Rigidbody>();
		this.originalPosition = this.rb.position;
	}
	
	// Reset the ground.
	public void Reset () {
		this.rb.position = this.originalPosition;
		this.rb.rotation = Quaternion.identity;
		this.rb.velocity = Vector3.zero;
		this.rb.angularVelocity = Vector3.zero;
	}
}
