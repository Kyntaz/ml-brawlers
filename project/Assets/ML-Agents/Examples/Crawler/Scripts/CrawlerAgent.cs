using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class CrawlerAgent : Agent
{
	public bool randomizePositions;

    [Header("External Elements")] [Space(10)]
    public Transform ground;
	public Transform sphere;
	public Transform buddyBody;
	public Agent buddyAgent;

    [Header("Body Parts")] [Space(10)] public Transform body;
    public Transform leg0Upper;
    public Transform leg0Lower;
    public Transform leg1Upper;
    public Transform leg1Lower;
    public Transform leg2Upper;
    public Transform leg2Lower;
    public Transform leg3Upper;
    public Transform leg3Lower;

    [Header("Joint Settings")] [Space(10)] JointDriveController jdController;
    float movingTowardsDot;
    float facingDot;

    [Header("Reward Functions To Use")] [Space(10)]
	public bool rewardUseTime; // Reward not dying
	public bool rewardFaceEnemy;
	public bool rewardAggressive;

    [Header("Foot Grounded Visualization")] [Space(10)]
    public bool useFootGroundedVisualization;

    public MeshRenderer foot0;
    public MeshRenderer foot1;
    public MeshRenderer foot2;
    public MeshRenderer foot3;
    public Material groundedMaterial;
    public Material unGroundedMaterial;
    bool isNewDecisionStep;
    int currentDecisionStep;

	[HideInInspector]
	public Vector3 initialPosition;

    public override void InitializeAgent()
    {
        jdController = GetComponent<JointDriveController>();
        currentDecisionStep = 1;

        //Setup each body part
        jdController.SetupBodyPart(body);
        jdController.SetupBodyPart(leg0Upper);
        jdController.SetupBodyPart(leg0Lower);
        jdController.SetupBodyPart(leg1Upper);
        jdController.SetupBodyPart(leg1Lower);
        jdController.SetupBodyPart(leg2Upper);
        jdController.SetupBodyPart(leg2Lower);
        jdController.SetupBodyPart(leg3Upper);
        jdController.SetupBodyPart(leg3Lower);

		initialPosition = body.position;
    }

    /// <summary>
    /// We only need to change the joint settings based on decision freq.
    /// </summary>
    public void IncrementDecisionTimer()
    {
        if (currentDecisionStep == agentParameters.numberOfActionsBetweenDecisions
            || agentParameters.numberOfActionsBetweenDecisions == 1)
        {
            currentDecisionStep = 1;
            isNewDecisionStep = true;
        }
        else
        {
            currentDecisionStep++;
            isNewDecisionStep = false;
        }
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp)
    {
        var rb = bp.rb;
        AddVectorObs(bp.groundContact.touchingGround ? 1 : 0); // Whether the bp touching the ground
        AddVectorObs(rb.velocity);
        AddVectorObs(rb.angularVelocity);

        if (bp.rb.transform != body)
        {
            Vector3 localPosRelToBody = body.InverseTransformPoint(rb.position);
            AddVectorObs(localPosRelToBody);
            AddVectorObs(bp.currentXNormalizedRot); // Current x rot
            AddVectorObs(bp.currentYNormalizedRot); // Current y rot
            AddVectorObs(bp.currentZNormalizedRot); // Current z rot
            AddVectorObs(bp.currentStrength / jdController.maxJointForceLimit);
        }
    }

    public override void CollectObservations()
    {
        jdController.GetCurrentJointForces();

        // Forward & up to help with orientation
        AddVectorObs(body.transform.position.y);
		AddVectorObs(body.transform.position - ground.position);
		AddVectorObs(ground.position - sphere.position);
        AddVectorObs(body.forward);
        AddVectorObs(body.up);

        foreach (var bodyPart in jdController.bodyPartsDict.Values)
        {
            CollectObservationBodyPart(bodyPart);
        }

		// Add information related to the platform's rotation.
		AddVectorObs(ground.rotation);

		// Add information of the buddy.
		AddVectorObs(buddyBody.position.y);
		AddVectorObs(buddyBody.position - ground.position);
		AddVectorObs(buddyBody.forward);
		AddVectorObs(buddyBody.up);
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        // If enabled the feet will light up green when the foot is grounded.
        // This is just a visualization and isn't necessary for function
        if (useFootGroundedVisualization)
        {
            foot0.material = jdController.bodyPartsDict[leg0Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = jdController.bodyPartsDict[leg1Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot2.material = jdController.bodyPartsDict[leg2Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot3.material = jdController.bodyPartsDict[leg3Lower].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        // Joint update logic only needs to happen when a new decision is made
        if (isNewDecisionStep)
        {
            // The dictionary with all the body parts in it are in the jdController
            var bpDict = jdController.bodyPartsDict;

            int i = -1;

            // Pick a new target joint rotation
            bpDict[leg0Upper].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[leg1Upper].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[leg2Upper].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[leg3Upper].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[leg0Lower].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[leg1Lower].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[leg2Lower].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[leg3Lower].SetJointTargetRotation(vectorAction[++i], 0, 0);

            // Update joint strength
            bpDict[leg0Upper].SetJointStrength(vectorAction[++i]);
            bpDict[leg1Upper].SetJointStrength(vectorAction[++i]);
            bpDict[leg2Upper].SetJointStrength(vectorAction[++i]);
            bpDict[leg3Upper].SetJointStrength(vectorAction[++i]);
            bpDict[leg0Lower].SetJointStrength(vectorAction[++i]);
            bpDict[leg1Lower].SetJointStrength(vectorAction[++i]);
            bpDict[leg2Lower].SetJointStrength(vectorAction[++i]);
            bpDict[leg3Lower].SetJointStrength(vectorAction[++i]);
        }

		if (body.position.y < sphere.position.y - 50.0f)
        {
            Done();
			buddyAgent.Done();

			AddReward(-1.0f);
        }
        else
        {
			if (rewardUseTime) AddReward(0.001f);
			if (rewardFaceEnemy) RewardFaceEnemy();
			if (rewardAggressive) RewardAggressive();
        }
			

        IncrementDecisionTimer();
    }

	void RewardFaceEnemy() {
		Vector3 toBuddy = (buddyBody.position - body.position).normalized;
		AddReward(0.001f * Vector3.Dot(body.forward, toBuddy));
	}

	void RewardAggressive() {
		Vector3 toBuddy = (buddyBody.position - body.position).normalized;
		AddReward(0.003f * Vector3.Dot(jdController.bodyPartsDict[body].rb.velocity, toBuddy));
	}

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void AgentReset()
    {
        foreach (var bodyPart in jdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        isNewDecisionStep = true;
        currentDecisionStep = 1;
		ground.GetComponent<GroundController>().Reset();
		//GetRandomTargetPos();

		if (randomizePositions) {
			Vector3 pos = initialPosition + Random.insideUnitSphere * 3.0f;
			pos.y = 4;

			gameObject.transform.position = pos;

			Vector3 lookAt = sphere.position;
			lookAt.y = pos.y;
			gameObject.transform.rotation = Quaternion.LookRotation(lookAt, Vector3.up);
		}
    }
}
