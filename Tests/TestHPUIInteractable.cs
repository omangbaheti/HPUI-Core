using System;
using System.Collections.Generic;
using ubco.ovilab.HPUI.Interaction;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

namespace ubco.ovilab.HPUI.Tests
{
    class TestHPUIInteractable : IHPUIInteractable
    {
        public Vector2 interactorPosition;
        public bool handlesTap, handlesGesture;
        public System.Action<HPUITapEventArgs> onTapCallback;
        public System.Action<HPUIGestureEventArgs> onGestureCallback;

        public int tapCalled = 0;
        public int swipCalled = 0;

        public TestHPUIInteractable(int zOrder, bool handlesTap, bool handlesGesture, Action<HPUITapEventArgs> onTapCallback = null, Action<HPUIGestureEventArgs> onGestureCallback = null)
        {
            this.zOrder = zOrder;
            this.handlesTap = handlesTap;
            this.handlesGesture = handlesGesture;
            if (onTapCallback != null)
                this.onTapCallback = onTapCallback;
            if (onGestureCallback != null)
                this.onGestureCallback = onGestureCallback;
            Reset();
        }

        public void Reset()
        {
            this.tapCalled = 0;
            this.swipCalled = 0;
        }

        #region IHPUIInteracttable only
        public int zOrder { get; set; }

        public Vector2 boundsMax { get; set; }

        public Vector2 boundsMin { get; set; }


        Vector2 IHPUIInteractable.ComputeInteractorPostion(IHPUIInteractor interactor)
        {
            return interactorPosition;
        }

        bool IHPUIInteractable.HandlesGesture(HPUIGesture state)
        {
            switch (state)
            {
                case HPUIGesture.Tap:
                    return handlesTap;
                case HPUIGesture.Gesture:
                    return handlesGesture;
                default:
                    throw new InvalidOperationException($"Gesture state {state} is not handled");
            }
        }

        void IHPUIInteractable.OnGesture(HPUIGestureEventArgs args)
        {
            swipCalled += 1;
            onGestureCallback?.Invoke(args);
        }

        void IHPUIInteractable.OnTap(HPUITapEventArgs args)
        {
            tapCalled += 1;
            onTapCallback?.Invoke(args);
        }
        #endregion

        #region Implement all other interfaces
        SelectEnterEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.firstSelectEntered => throw new NotImplementedException();

        SelectExitEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.lastSelectExited => throw new NotImplementedException();

        SelectEnterEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.selectEntered => throw new NotImplementedException();

        SelectExitEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.selectExited => throw new NotImplementedException();

        List<UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor> UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.interactorsSelecting => throw new NotImplementedException();

        UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.firstInteractorSelecting => throw new NotImplementedException();

        bool UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.isSelected => throw new NotImplementedException();

        UnityEngine.XR.Interaction.Toolkit.Interactables.InteractableSelectMode UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.selectMode => throw new NotImplementedException();

        InteractionLayerMask UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.interactionLayers => throw new NotImplementedException();

        List<Collider> UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.colliders => throw new NotImplementedException();

        Transform UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.transform => throw new NotImplementedException();

        HoverEnterEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.firstHoverEntered => throw new NotImplementedException();

        HoverExitEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.lastHoverExited => throw new NotImplementedException();

        HoverEnterEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.hoverEntered => throw new NotImplementedException();

        HoverExitEvent UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.hoverExited => throw new NotImplementedException();

        List<UnityEngine.XR.Interaction.Toolkit.Interactors.IXRHoverInteractor> UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.interactorsHovering => throw new NotImplementedException();

        bool UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.isHovered => throw new NotImplementedException();

        HPUITapEvent IHPUIInteractable.TapEvent => throw new NotImplementedException();

        HPUIGestureEvent IHPUIInteractable.GestureEvent => throw new NotImplementedException();

        event Action<InteractableRegisteredEventArgs> UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.registered
        {
            add
            {
                throw new NotImplementedException();
            }

            remove
            {
                throw new NotImplementedException();
            }
        }

        event Action<InteractableUnregisteredEventArgs> UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.unregistered
        {
            add
            {
                throw new NotImplementedException();
            }

            remove
            {
                throw new NotImplementedException();
            }
        }

        Pose UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.GetAttachPoseOnSelect(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor interactor)
        {
            throw new NotImplementedException();
        }

        Transform UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.GetAttachTransform(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRInteractor interactor)
        {
            throw new NotImplementedException();
        }

        float UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.GetDistanceSqrToInteractor(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRInteractor interactor)
        {
            throw new NotImplementedException();
        }

        Pose UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.GetLocalAttachPoseOnSelect(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor interactor)
        {
            throw new NotImplementedException();
        }

        bool UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.IsSelectableBy(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor interactor)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.OnRegistered(InteractableRegisteredEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.OnSelectEntered(SelectEnterEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.OnSelectEntering(SelectEnterEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.OnSelectExited(SelectExitEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRSelectInteractable.OnSelectExiting(SelectExitEventArgs args)
        {
            throw new NotImplementedException();
        }
        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.OnUnregistered(InteractableUnregisteredEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRInteractable.ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
        {
            throw new NotImplementedException();
        }

        bool UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.IsHoverableBy(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRHoverInteractor interactor)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.OnHoverEntering(HoverEnterEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.OnHoverEntered(HoverEnterEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.OnHoverExiting(HoverExitEventArgs args)
        {
            throw new NotImplementedException();
        }

        void UnityEngine.XR.Interaction.Toolkit.Interactables.IXRHoverInteractable.OnHoverExited(HoverExitEventArgs args)
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}
