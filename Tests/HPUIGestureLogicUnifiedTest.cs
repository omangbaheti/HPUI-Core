using System.Collections;
using UnityEngine.TestTools;
using ubco.ovilab.HPUI.Interaction;
using UnityEngine.XR.Interaction.Toolkit;
using System.Collections.Generic;
using UnityEngine;
using System;
using NUnit.Framework;

namespace ubco.ovilab.HPUI.Tests
{
    public class HPUIGestureLogicUnifiedTest
    {
        const float TapTimeThreshold = 0.4f;
        const int TapDistanceThreshold = 1;
        private IHPUIInteractable lastTapInteractable, lastGestureInteractable;
        private int tapsCount = 0;
        private int gesturesCount = 0;

        void OnTapCallback(HPUITapEventArgs args)
        {
            tapsCount += 1;
            lastTapInteractable = args.interactableObject;

        }
        void OnGestureCallback(HPUIGestureEventArgs args)
        {
            gesturesCount += 1;
            Debug.Log($"{args.interactableObject}");
            lastGestureInteractable = args.interactableObject;
        }

        private void Reset()
        {
            tapsCount = 0;
            gesturesCount = 0;
            lastTapInteractable = null;
            lastGestureInteractable = null;
        }
            
        // A UnityTest behaves like a coroutine in Play Mode. In Edit Mode you can use
        // `yield return null;` to skip a frame.
        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_SimpleTap()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);
            // First tap
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold /2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);

            // Second tap
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold /2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 2);
            Assert.AreEqual(gesturesCount, 0);
        }

        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_SimpleGesture()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            // Tap and hold
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold * 2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);

            // Move
            logic.OnSelectEntering(i1);
            logic.Update();
            i1.interactorPosition = Vector2.one * 2;
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);
        }

        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_TapThenGesture()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);
            // First tap
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold /2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);

            // Gesture
            Reset();
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold * 2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);
        }

        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_GestureThenTap()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);
            // Gesture
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold * 2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);

            // tap
            Reset();
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(TapTimeThreshold / 2);
            logic.Update();
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
        }

        [Test]
        public void HPUIGestureLogicUnifiedTest_TwoItem_tap_time()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            // Tap 1-2---1-2
            logic.OnSelectEntering(i1);
            logic.OnSelectEntering(i2);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
            Assert.AreEqual(lastTapInteractable, i1);
            Assert.AreEqual(i1.tapCalled, 1);
            Assert.AreEqual(i2.tapCalled, 0);

            Reset();
            i1.Reset();
            i2.Reset();
            // Tap 1-2---2-1
            logic.OnSelectEntering(i1);
            logic.OnSelectEntering(i2);
            logic.Update();
            logic.OnSelectExiting(i2);
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
            Assert.AreEqual(lastTapInteractable, i1);
            Assert.AreEqual(i1.tapCalled, 1);
            Assert.AreEqual(i2.tapCalled, 0);
        }

        [Test]
        public void HPUIGestureLogicUnifiedTest_TwoItem_tap_zOrder()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(1, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            // Tap 1-2---1-2
            logic.OnSelectEntering(i1);
            logic.OnSelectEntering(i2);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
            Assert.AreEqual(lastTapInteractable, i1);
            Assert.AreEqual(i1.tapCalled, 1);
            Assert.AreEqual(i2.tapCalled, 0);

            Reset();
            i1.Reset();
            i2.Reset();
            // Tap 2-1---2-1
            logic.OnSelectEntering(i2);
            logic.OnSelectEntering(i1);
            logic.Update();
            logic.OnSelectExiting(i2);
            logic.OnSelectExiting(i1);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
            Assert.AreEqual(lastTapInteractable, i1);
            Assert.AreEqual(i1.tapCalled, 1);
            Assert.AreEqual(i2.tapCalled, 0);
        }

        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_TwoItem_gesture()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(1, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            logic.OnSelectEntering(i2);
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);
            Assert.AreEqual(lastGestureInteractable, i1);
            Assert.Greater(i1.swipCalled, 0);
            Assert.AreEqual(i2.swipCalled, 0);
        }

        // Anything ouside the priority window should not get selected
        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_TwoItem_gesture_priority_window()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(1, true, true, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(1);
            // Even though this has lower zOrder, this should not get selected
            logic.OnSelectEntering(i2);
            logic.Update();
            yield return new WaitForSeconds(1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);
            Assert.AreEqual(lastGestureInteractable, i1);
            Assert.Greater(i1.swipCalled, 0);
            Assert.AreEqual(i2.swipCalled, 0);
        }

        // When an event is not handled, hand over to next item in the priority list
        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_TwoItem_gesture_handle_events()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, true, true, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(0, false, false, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            logic.OnSelectEntering(i2);
            // even though this is coming in second, this should get the tap
            logic.OnSelectEntering(i1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 1);
            Assert.AreEqual(gesturesCount, 0);
            Assert.AreEqual(lastTapInteractable, i1);

            Reset();
            i1.Reset();
            i2.Reset();
            logic.OnSelectEntering(i2);
            // even though this is coming in second, this should get the gesture
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 0);
            Assert.Greater(gesturesCount, 0);
            Assert.AreEqual(lastGestureInteractable, i1);
            Assert.AreEqual(i2.swipCalled, 0);
            Assert.Greater(i1.swipCalled, 0);
        }

        //There can be instances where the event is not hanled by any interactable
        [UnityTest]
        public IEnumerator HPUIGestureLogicUnifiedTest_TwoItem_gesture_no_handle_events()
        {
            Reset();
            TestHPUIInteractable i1 = new TestHPUIInteractable(0, false, false, OnTapCallback, OnGestureCallback);
            TestHPUIInteractable i2 = new TestHPUIInteractable(0, false, false, OnTapCallback, OnGestureCallback);
            IHPUIGestureLogic logic = new HPUIGestureLogicUnified(new HPUIInteractor(), TapTimeThreshold, TapDistanceThreshold);

            // Tap not handled by any interactable
            Reset();
            i1.Reset();
            i2.Reset();
            logic.OnSelectEntering(i2);
            logic.OnSelectEntering(i1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 0);
            Assert.AreEqual(gesturesCount, 0);

            // Gesture not handled by any interactable
            Reset();
            i1.Reset();
            i2.Reset();
            logic.OnSelectEntering(i2);
            logic.OnSelectEntering(i1);
            logic.Update();
            yield return new WaitForSeconds(1);
            logic.Update();
            logic.OnSelectExiting(i1);
            logic.OnSelectExiting(i2);
            Assert.AreEqual(tapsCount, 0);
            Assert.AreEqual(gesturesCount, 0);
        }

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


            Vector2 IHPUIInteractable.ComputeInteractorPostion(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRInteractor interactor)
            {
                return interactorPosition;
            }

            bool IHPUIInteractable.HandlesGesture(HPUIGesture state)
            {
                switch (state) {
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
}
