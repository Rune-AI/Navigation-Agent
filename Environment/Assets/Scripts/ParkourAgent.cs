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
    private bool isGrounded;

    [Header("Previous Actions")]
    //private List<float> previousContinuousActions;
    //private List<int> previousDiscreteActions;

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
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //relative goal position
        sensor.AddObservation(goal.position - transform.position);

        //current speed, acceleration, etc.
        sensor.AddObservation(rbody.velocity);
        sensor.AddObservation(transform.rotation.eulerAngles);

        //previous actions
        //sensor.AddObservation(previousContinuousActions[0]);
        //sensor.AddObservation(previousContinuousActions[1]);
        //sensor.AddObservation(previousContinuousActions[2]);
        //sensor.AddObservation(previousDiscreteActions[0]);

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
        //previousContinuousActions.Clear();
        //previousDiscreteActions.Clear();
        //foreach (float value in actionBuffers.ContinuousActions)
        //{
        //    previousContinuousActions.Add(value);
        //}
        //foreach (int value in actionBuffers.DiscreteActions)
        //{
        //    previousDiscreteActions.Add(value);
        //}

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
        if (actionBuffers.DiscreteActions[0] == 1)
        {
            if (isGrounded || isReadyToJump)
            {
                Jump();
            }
        }

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, goal.localPosition);

        // Reached target
        if (distanceToTarget < 1.5f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        // Fell off platform
        else if (this.transform.localPosition.y < -0.3f)
        {
            EndEpisode();
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
        controls.Movement.Jump.performed += ctx => DiscreteActionsOut[0] = 1;

        //DiscreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;

    }
    
    private Vector3 GetRandomPosition()
    {
        float radius = 5F;
        while (true)
        {
            Vector3 randomPosition = new Vector3(Random.Range(-radius, radius), 0, Random.Range(-radius, radius));
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
    
}
