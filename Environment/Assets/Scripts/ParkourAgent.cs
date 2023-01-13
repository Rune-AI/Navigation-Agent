using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class ParkourAgent : Agent
{
    public float _moveSpeed = 5f;
    public float _jumpSpeed = 3.5f;

    public float _mouseSense = 20f;

    public float _gravity = 9.81f;
    public Transform _Goal;

    private Vector3 _velocity;
    private Vector3 _rotation;

    //private CharacterController _Controller;
    private Rigidbody _rigidbody;
    private Camera _Camera;
    void Start()
    {
        //_Controller = GetComponent<CharacterController>();
        _rigidbody = GetComponent<Rigidbody>();
        _Camera = GetComponent<Camera>();

        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }


    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (this.transform.localPosition.y < 0)
        {
            this.transform.localPosition = new Vector3(0, 0.5f, 0);
        }

        // Move the target to a new spot
        _Goal.localPosition = GetRandomPosition();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //relative goal position
        sensor.AddObservation(_Goal.localPosition - transform.localPosition);

        //current speed, acceleration, etc.
        sensor.AddObservation(_rigidbody.velocity);

        // Target and Agent absolute positions
        sensor.AddObservation(_Goal.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        //Depth map with raycasts
        //_Camera.depthTextureMode = DepthTextureMode.Depth;



        //3D occupancy Map with boxcast
        //should be cashed/baked


    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        //Agent rotation
        float mouseX = actionBuffers.ContinuousActions[2] * Time.deltaTime * _mouseSense;
        _rotation.y += mouseX;
        transform.rotation = Quaternion.Euler(_rotation);


        //Agent movement
        Vector3 moveDirection = transform.forward * actionBuffers.ContinuousActions[0] + transform.right * actionBuffers.ContinuousActions[1];
        _rigidbody.AddForce(moveDirection * _moveSpeed, ForceMode.Force);



        //Vector3 acceleration = Vector3.zero;
        //acceleration.x += actionBuffers.ContinuousActions[0] * _moveSpeed;
        //acceleration.z += actionBuffers.ContinuousActions[1] * _moveSpeed;

        //if (actionBuffers.DiscreteActions[0] == 1)
        //{
        //    acceleration.y = _jumpSpeed;
        //}
        
        //acceleration.y -= _gravity;


        //_velocity = _velocity + acceleration * Time.deltaTime;

        //_velocity.x = Mathf.Clamp(_velocity.x, -_moveSpeed, _moveSpeed);
        //_velocity.z = Mathf.Clamp(_velocity.z, -_moveSpeed, _moveSpeed);

        //_Controller.Move(_velocity);

        

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, _Goal.localPosition);

        // Reached target
        if (distanceToTarget < 1f)
        {
            SetReward(1.0f);
            EndEpisode();
        }

        // Fell off platform
        else if (this.transform.localPosition.y < 0)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        
        var continuousActionsOut = actionsOut.ContinuousActions;
        var DiscreteActionsOut = actionsOut.DiscreteActions;
        
        // Move + Strafe
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");

        // Rotate
        continuousActionsOut[2] = Input.GetAxis("Mouse X");

        // Jump
        DiscreteActionsOut[0] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

    private Vector3 GetRandomPosition()
    {
        float radius = 50f;
        while (true)
        {
            Vector3 randomPosition = new Vector3(Random.Range(-radius, radius), 0, Random.Range(-radius, radius));
            if (Physics.CheckSphere(randomPosition, 1f, LayerMask.GetMask("Ground")))
            {
                return randomPosition;
            }
        }
    }
}
