using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;

public class ParkourAgent : Agent
{
    [Header("Goal")]
    public Transform goal;

    [Header("Movement")]
    public float moveSpeed = 5f;
    public float mouseSense = 100f;
    public float groundDrag = 5f;

    public float jumpForce = 12f;
    public float airMultiplier = 0.4f;
    public float jumpCooldown = 0.1f;
    private bool isReadyToJump;

    private Vector3 rotation;

    [Header("Ground Check")]
    public float agentHeight = 1;
    public LayerMask groundLayer;
    [SerializeField] private bool isGrounded;

    [Header("Training Curriculum")]
    public float goalRadius;
    public float startGoalRadius = 5f;
    public float maxGoalRadius = 5f;
    public float goalRadiusIncrement = 1f;
    public float sucessRatioLimit = 0.8f;
    [SerializeField] private int successfulEpisodesCount;
    public int maxStepsPerEpisodeMulti = 50;
    [SerializeField] private float sucessRatio;

    [Header("Reward")]
    [SerializeField] private float clostestDistanceThisEpoch;
    [SerializeField] private float rewardPerStep = -0.01f;
    [SerializeField] private float minimumDistanceToGoal = 1.5f;

    [Header("DebugControls")]
    private float verticalValue;
    private float horizontalValue;
    private float rotateValue;

    private AgentControls controls;
    private Rigidbody rbody;
    void Start()
    {
        //_Controller = GetComponent<CharacterController>();
        rbody = GetComponent<Rigidbody>();

        //Cursor.lockState = CursorLockMode.Locked;
        //Cursor.visible = false;

        controls = new AgentControls();
        controls.Movement.Enable();

        goalRadius = startGoalRadius;
    }


    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (this.transform.localPosition.y < 0)
        {
            rbody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0, 0);
        }

        // Move the target to a new spot
        goal.localPosition = GetRandomPosition();

        clostestDistanceThisEpoch = Vector3.Distance(this.transform.localPosition, goal.localPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //relative goal position
        sensor.AddObservation(goal.position - transform.position);

        //current speed, acceleration, etc.
        sensor.AddObservation(rbody.velocity);
        sensor.AddObservation(transform.rotation.eulerAngles);

        // Target and Agent absolute positions
        sensor.AddObservation(goal.position);
        sensor.AddObservation(this.transform.position);

        //Depth map with raycasts
        //_Camera.depthTextureMode = DepthTextureMode.Depth;



        //3D occupancy Map with boxcast
        //should be cashed/baked


    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        //Agent rotation
        Rotation(actionBuffers.ContinuousActions[2]);

        // isGrounded
        isGrounded = Physics.Raycast(transform.position + transform.up * agentHeight * 0.5f, Vector3.down, 0.1f + agentHeight * 0.5f, groundLayer);
        Debug.DrawRay(transform.position + transform.up * agentHeight * 0.5f, Vector3.down * (0.1f + agentHeight * 0.5f), Color.red);

        //Agent movement
        Movement(actionBuffers.ContinuousActions[0], actionBuffers.ContinuousActions[1]);

        //handle drag
        Drag();

        // speed control
        SpeedControl();

        // Jump
        if (actionBuffers.ContinuousActions[3] >= 0.9f)
        {
            if (isGrounded || isReadyToJump)
            {
                Jump();
            }
        }

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, goal.localPosition);

        float reward = Mathf.Max(clostestDistanceThisEpoch - distanceToTarget, 0) + rewardPerStep + (distanceToTarget <= minimumDistanceToGoal ? 1 : 0);

        print(reward);

        SetReward(reward);

        if (distanceToTarget < clostestDistanceThisEpoch)
        {
            clostestDistanceThisEpoch = distanceToTarget;
        }

        // Reached target
        if (distanceToTarget < minimumDistanceToGoal)
        {
            EndEpisode(true);
        }

        // Fell off platform
        else if (this.transform.localPosition.y < -0.3f)
        {
            EndEpisode(false);
        }
        else if (StepCount >= maxStepsPerEpisodeMulti * goalRadius)
        {
            EndEpisode(false);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        
        var continuousActionsOut = actionsOut.ContinuousActions;
        var DiscreteActionsOut = actionsOut.DiscreteActions;

        // Move + Strafe
        controls.Movement.VerticalMove.performed += ctx => verticalValue = ctx.ReadValue<float>();
        controls.Movement.VerticalMove.canceled += ctx => verticalValue = 0;
        controls.Movement.HorizontalMove.performed += ctx => horizontalValue = ctx.ReadValue<float>();
        controls.Movement.HorizontalMove.canceled += ctx => horizontalValue = 0;
        continuousActionsOut[0] = verticalValue;
        continuousActionsOut[1] = horizontalValue;

        //continuousActionsOut[0] = Input.GetAxis("Vertical");
        //continuousActionsOut[1] = Input.GetAxis("Horizontal");

        // Rotate
        controls.Movement.Rotate.performed += ctx => rotateValue = ctx.ReadValue<float>();
        controls.Movement.Rotate.canceled += ctx => rotateValue = 0;
        continuousActionsOut[2] = rotateValue;

        //continuousActionsOut[2] = Input.GetAxis("Mouse X");

        // Jump
        controls.Movement.Jump.performed += ctx => continuousActionsOut[3] = 1;

        //DiscreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;

    }
    
    private Vector3 GetRandomPosition()
    {
        while (true)
        {
            Vector3 randomPosition = new Vector3(Random.Range(-goalRadius, goalRadius), 0, Random.Range(-goalRadius, goalRadius));
            if (Physics.CheckSphere(randomPosition, 1f, groundLayer))
            {
                return randomPosition;
            }
        }
    }

    private void Rotation(float x)
    {
        float mouseX = x * Time.deltaTime * mouseSense;
        rotation.y += mouseX;
        transform.rotation = Quaternion.Euler(rotation);
    }

    private void Movement(float vertical, float horizontal)
    {
        Vector3 moveDirection = transform.forward * vertical + transform.right * horizontal;
        if (isGrounded)
            rbody.AddForce(moveDirection * moveSpeed * 10f, ForceMode.Force);

        if (!isGrounded)
            rbody.AddForce(moveDirection * moveSpeed * 10f * airMultiplier, ForceMode.Force);
    }

    private void Jump()
    {
        isReadyToJump = false;
        if (isGrounded)
            Invoke(nameof(ResetDoubleJump), jumpCooldown);

        rbody.velocity = new Vector3(rbody.velocity.x, 0f, rbody.velocity.z);
        rbody.AddForce(transform.up * jumpForce, ForceMode.Impulse);
    }

    private void ResetDoubleJump()
    {
        isReadyToJump = true;
    }

    private void Drag()
    {
        if (isGrounded)
        {
            rbody.drag = groundDrag;
        }
        else
        {
            rbody.drag = 0;
        }
    }

    private void SpeedControl()
    {
        Vector3 flatVel = new Vector3(rbody.velocity.x, 0f, rbody.velocity.z);

        if (flatVel.magnitude > moveSpeed)
        {
            Vector3 limitedVel = flatVel.normalized * moveSpeed;
            rbody.velocity = new Vector3(limitedVel.x, rbody.velocity.y, limitedVel.z);
        }
    }

    private void EndEpisode(bool reachedGoal)
    {
        if (this.CompletedEpisodes == 0)
        {
            EndEpisode();
            return;
        }

        if (reachedGoal)
        {
            successfulEpisodesCount++;
        }
        
        sucessRatio = successfulEpisodesCount / (float)CompletedEpisodes;

        if (sucessRatio > sucessRatioLimit)
        {
            if (goalRadius < maxGoalRadius)
            {
                goalRadius += goalRadiusIncrement;
            }
        }

        EndEpisode();
    }
    
}
