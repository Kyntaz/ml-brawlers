using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class WalkerAgent : Agent
{
    [Header("Specific to Walker")] [Header("Buddy")] [Space(10)]
	public WalkerAgent buddy;
	public Transform ground;
	public Transform sphere;

	[Header("Rewards to Use")]
	public bool rewardUseTime;
	public bool rewardAggressive;
	public bool rewardFaceEnemy;
	public bool rewardHigh;

	[Header("Body Parts")]
    public Transform hips;
    public Transform chest;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;
    JointDriveController jdController;
    bool isNewDecisionStep;
    int currentDecisionStep;

    public override void InitializeAgent()
    {
        jdController = GetComponent<JointDriveController>();
        jdController.SetupBodyPart(hips);
        jdController.SetupBodyPart(chest);
        jdController.SetupBodyPart(spine);
        jdController.SetupBodyPart(head);
        jdController.SetupBodyPart(thighL);
        jdController.SetupBodyPart(shinL);
        jdController.SetupBodyPart(footL);
        jdController.SetupBodyPart(thighR);
        jdController.SetupBodyPart(shinR);
        jdController.SetupBodyPart(footR);
        jdController.SetupBodyPart(armL);
        jdController.SetupBodyPart(forearmL);
        jdController.SetupBodyPart(handL);
        jdController.SetupBodyPart(armR);
        jdController.SetupBodyPart(forearmR);
        jdController.SetupBodyPart(handR);
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
	public void CollectObservationBodyPart(BodyPart bp, WalkerAgent a)
    {
        var rb = bp.rb;
        AddVectorObs(bp.groundContact.touchingGround ? 1 : 0); // Is this bp touching the ground
        AddVectorObs(rb.velocity);
        AddVectorObs(rb.angularVelocity);
        Vector3 localPosRelToHips = a.hips.InverseTransformPoint(rb.position);
        AddVectorObs(localPosRelToHips);

        if (bp.rb.transform != a.hips && bp.rb.transform != a.handL && bp.rb.transform != a.handR &&
            bp.rb.transform != a.footL && bp.rb.transform != a.footR && bp.rb.transform != a.head)
        {
            AddVectorObs(bp.currentXNormalizedRot);
            AddVectorObs(bp.currentYNormalizedRot);
            AddVectorObs(bp.currentZNormalizedRot);
            AddVectorObs(bp.currentStrength / jdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations()
    {
        jdController.GetCurrentJointForces();

        AddVectorObs(jdController.bodyPartsDict[hips].rb.position);
        AddVectorObs(hips.forward);
        AddVectorObs(hips.up);
		AddVectorObs(hips.position - ground.position);

		AddVectorObs(ground.position - sphere.position);
		AddVectorObs(ground.up);
		AddVectorObs(ground.rotation);

        foreach (var bodyPart in jdController.bodyPartsDict.Values)
        {
            CollectObservationBodyPart(bodyPart, this);
        }

		// Collect info on buddy.
		AddVectorObs(buddy.jdController.bodyPartsDict[buddy.hips].rb.position);
		AddVectorObs(buddy.hips.forward);
		AddVectorObs(buddy.hips.up);
		AddVectorObs(buddy.hips.position - ground.position);

		foreach (var bodyPart in buddy.jdController.bodyPartsDict.Values)
		{
			CollectObservationBodyPart(bodyPart, buddy);
		}
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        // Apply action to all relevant body parts. 
        if (isNewDecisionStep)
        {
            var bpDict = jdController.bodyPartsDict;
            int i = -1;

            bpDict[chest].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);
            bpDict[spine].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);

            bpDict[thighL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[thighR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[shinL].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[shinR].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[footR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);
            bpDict[footL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);


            bpDict[armL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[armR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
            bpDict[forearmL].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[forearmR].SetJointTargetRotation(vectorAction[++i], 0, 0);
            bpDict[head].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);

            //update joint strength settings
            bpDict[chest].SetJointStrength(vectorAction[++i]);
            bpDict[spine].SetJointStrength(vectorAction[++i]);
            bpDict[head].SetJointStrength(vectorAction[++i]);
            bpDict[thighL].SetJointStrength(vectorAction[++i]);
            bpDict[shinL].SetJointStrength(vectorAction[++i]);
            bpDict[footL].SetJointStrength(vectorAction[++i]);
            bpDict[thighR].SetJointStrength(vectorAction[++i]);
            bpDict[shinR].SetJointStrength(vectorAction[++i]);
            bpDict[footR].SetJointStrength(vectorAction[++i]);
            bpDict[armL].SetJointStrength(vectorAction[++i]);
            bpDict[forearmL].SetJointStrength(vectorAction[++i]);
            bpDict[armR].SetJointStrength(vectorAction[++i]);
            bpDict[forearmR].SetJointStrength(vectorAction[++i]);
        }

        IncrementDecisionTimer();

		if (hips.position.y < sphere.position.y - 50.0f)
		{
			Done();
			buddy.Done();

			AddReward(-1.0f);
		}
		else
		{
			if (rewardUseTime) AddReward(0.001f);
			if (rewardFaceEnemy) RewardFaceEnemy();
			if (rewardAggressive) RewardAggressive();
			if (rewardHigh) RewardHigh();
		}
    }

	void RewardFaceEnemy() {
		Vector3 toBuddy = (buddy.head.position - head.position).normalized;
		AddReward(0.001f * Vector3.Dot(head.forward, toBuddy));
	}

	void RewardAggressive() {
		Vector3 toBuddy = (buddy.chest.position - chest.position).normalized;
		AddReward(0.003f * Vector3.Dot(jdController.bodyPartsDict[chest].rb.velocity, toBuddy));
	}

	void RewardHigh() {
		AddReward(0.001f * head.position.y);
	}

    /// <summary>
    /// Only change the joint settings based on decision frequency.
    /// </summary>
    public void IncrementDecisionTimer()
    {
        if (currentDecisionStep == agentParameters.numberOfActionsBetweenDecisions ||
            agentParameters.numberOfActionsBetweenDecisions == 1)
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
    }
}
