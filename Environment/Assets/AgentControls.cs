//------------------------------------------------------------------------------
// <auto-generated>
//     This code was auto-generated by com.unity.inputsystem:InputActionCodeGenerator
//     version 1.4.4
//     from Assets/AgentControls.inputactions
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public partial class @AgentControls : IInputActionCollection2, IDisposable
{
    public InputActionAsset asset { get; }
    public @AgentControls()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""AgentControls"",
    ""maps"": [
        {
            ""name"": ""Movement"",
            ""id"": ""6e840f00-b7ae-4c64-9eaa-4762ec1e4e42"",
            ""actions"": [
                {
                    ""name"": ""Vertical Move"",
                    ""type"": ""Value"",
                    ""id"": ""06e82222-57ed-46d0-b3da-587c6777da6e"",
                    ""expectedControlType"": ""Axis"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""Horizontal Move"",
                    ""type"": ""Value"",
                    ""id"": ""8a6100f8-191c-4620-9f13-e0d219b07ba6"",
                    ""expectedControlType"": ""Axis"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""Rotate"",
                    ""type"": ""Value"",
                    ""id"": ""f5f50257-c2ef-4665-81ce-d73f998b00e0"",
                    ""expectedControlType"": ""Axis"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": true
                },
                {
                    ""name"": ""Jump"",
                    ""type"": ""Button"",
                    ""id"": ""b8f4f122-96d1-48fc-9bb5-6e47df4cfbb1"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": """",
                    ""initialStateCheck"": false
                }
            ],
            ""bindings"": [
                {
                    ""name"": """",
                    ""id"": ""e6e3274a-40c7-4062-8f28-4816d617bede"",
                    ""path"": ""<Gamepad>/leftStick/y"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Vertical Move"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""92330127-54d9-4bd7-af06-2d43f6cc1a15"",
                    ""path"": ""<Gamepad>/rightStick/x"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Rotate"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""78e0353b-2485-46b2-bbba-a795b41b5de2"",
                    ""path"": ""<Gamepad>/leftStick/x"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Horizontal Move"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": """",
                    ""id"": ""302bebfd-2388-4e0f-8f7e-7147f63883d2"",
                    ""path"": ""<Gamepad>/buttonSouth"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Jump"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        }
    ],
    ""controlSchemes"": []
}");
        // Movement
        m_Movement = asset.FindActionMap("Movement", throwIfNotFound: true);
        m_Movement_VerticalMove = m_Movement.FindAction("Vertical Move", throwIfNotFound: true);
        m_Movement_HorizontalMove = m_Movement.FindAction("Horizontal Move", throwIfNotFound: true);
        m_Movement_Rotate = m_Movement.FindAction("Rotate", throwIfNotFound: true);
        m_Movement_Jump = m_Movement.FindAction("Jump", throwIfNotFound: true);
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }
    public IEnumerable<InputBinding> bindings => asset.bindings;

    public InputAction FindAction(string actionNameOrId, bool throwIfNotFound = false)
    {
        return asset.FindAction(actionNameOrId, throwIfNotFound);
    }
    public int FindBinding(InputBinding bindingMask, out InputAction action)
    {
        return asset.FindBinding(bindingMask, out action);
    }

    // Movement
    private readonly InputActionMap m_Movement;
    private IMovementActions m_MovementActionsCallbackInterface;
    private readonly InputAction m_Movement_VerticalMove;
    private readonly InputAction m_Movement_HorizontalMove;
    private readonly InputAction m_Movement_Rotate;
    private readonly InputAction m_Movement_Jump;
    public struct MovementActions
    {
        private @AgentControls m_Wrapper;
        public MovementActions(@AgentControls wrapper) { m_Wrapper = wrapper; }
        public InputAction @VerticalMove => m_Wrapper.m_Movement_VerticalMove;
        public InputAction @HorizontalMove => m_Wrapper.m_Movement_HorizontalMove;
        public InputAction @Rotate => m_Wrapper.m_Movement_Rotate;
        public InputAction @Jump => m_Wrapper.m_Movement_Jump;
        public InputActionMap Get() { return m_Wrapper.m_Movement; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(MovementActions set) { return set.Get(); }
        public void SetCallbacks(IMovementActions instance)
        {
            if (m_Wrapper.m_MovementActionsCallbackInterface != null)
            {
                @VerticalMove.started -= m_Wrapper.m_MovementActionsCallbackInterface.OnVerticalMove;
                @VerticalMove.performed -= m_Wrapper.m_MovementActionsCallbackInterface.OnVerticalMove;
                @VerticalMove.canceled -= m_Wrapper.m_MovementActionsCallbackInterface.OnVerticalMove;
                @HorizontalMove.started -= m_Wrapper.m_MovementActionsCallbackInterface.OnHorizontalMove;
                @HorizontalMove.performed -= m_Wrapper.m_MovementActionsCallbackInterface.OnHorizontalMove;
                @HorizontalMove.canceled -= m_Wrapper.m_MovementActionsCallbackInterface.OnHorizontalMove;
                @Rotate.started -= m_Wrapper.m_MovementActionsCallbackInterface.OnRotate;
                @Rotate.performed -= m_Wrapper.m_MovementActionsCallbackInterface.OnRotate;
                @Rotate.canceled -= m_Wrapper.m_MovementActionsCallbackInterface.OnRotate;
                @Jump.started -= m_Wrapper.m_MovementActionsCallbackInterface.OnJump;
                @Jump.performed -= m_Wrapper.m_MovementActionsCallbackInterface.OnJump;
                @Jump.canceled -= m_Wrapper.m_MovementActionsCallbackInterface.OnJump;
            }
            m_Wrapper.m_MovementActionsCallbackInterface = instance;
            if (instance != null)
            {
                @VerticalMove.started += instance.OnVerticalMove;
                @VerticalMove.performed += instance.OnVerticalMove;
                @VerticalMove.canceled += instance.OnVerticalMove;
                @HorizontalMove.started += instance.OnHorizontalMove;
                @HorizontalMove.performed += instance.OnHorizontalMove;
                @HorizontalMove.canceled += instance.OnHorizontalMove;
                @Rotate.started += instance.OnRotate;
                @Rotate.performed += instance.OnRotate;
                @Rotate.canceled += instance.OnRotate;
                @Jump.started += instance.OnJump;
                @Jump.performed += instance.OnJump;
                @Jump.canceled += instance.OnJump;
            }
        }
    }
    public MovementActions @Movement => new MovementActions(this);
    public interface IMovementActions
    {
        void OnVerticalMove(InputAction.CallbackContext context);
        void OnHorizontalMove(InputAction.CallbackContext context);
        void OnRotate(InputAction.CallbackContext context);
        void OnJump(InputAction.CallbackContext context);
    }
}
