using UnityEngine;


namespace ubco.ovilab.HPUI.Interaction 
{
    public interface IHPUIInteractor: UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor, UnityEngine.XR.Interaction.Toolkit.Interactors.IXRHoverInteractor
    {
        /// <summary>
        /// Event triggered on tap
        /// </summary>
        public HPUITapEvent TapEvent { get; }

        /// <summary>
        /// Event triggered on gesture
        /// </summary>
        public HPUIGestureEvent GestureEvent { get; }

        /// <summary>
        /// This is called when a tap event occurs on the interactable.
        /// </summary>
        void OnTap(HPUITapEventArgs args);

        /// <summary>
        /// This is called when a gesture event occurs on the interactable.
        /// </summary>
        void OnGesture(HPUIGestureEventArgs args);

        /// <summary>
        /// Get the point of interaction.
        /// </summary>
        Vector3 GetCollisionPoint(IHPUIInteractable interactable);
    }
}
