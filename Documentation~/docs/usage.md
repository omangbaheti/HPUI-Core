# Usage
This can be imported as a git package in Unity. The package is built to use [Unity XRHands](https://docs.unity3d.com/Packages/com.unity.xr.hands@1.4/manual/index.html) and [Unity XR Interaction Toolkit](https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@3.0/manual/index.html). Hence, it is interoperable with any package that implements the Unity XRHands.
The `Sample scene with HPUI` has an example with the HPUI components setup.

## Scene setup
- Setup the Scene with an XR Origin & XR Interaction Manager (see documentation [XRI documentation for more details](https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@3.0/manual/general-setup.html#create-the-xr-origin-camera-rig-for-tracked-devices)).
- Place atleast one HPUIInteractor component. You may use the `HPUIInteractor` prefab that is provided with the package.
- Create intractables with the HPUI Interactables components added to them (i.e., `HPUIBaseInteractable`, `HPUIGeneratedContinuousInteractable`, & `HPUIMeshContinuousInteractable`).
- Add and configure the `JointFollower` component to all gameobjects with HPUI Interactables or HPUI Interactor. This component makes sure the game objects location is set to the respective joint(s) of a given hand. The Interactor and Interactables don't depend on these, but they play nice with each other - i.e., the HPUI Interactables and HPUI Interactors will respect the configuration (Handedness) of the JointFollower.

Note that, the HPUI interactables do not have to be under the `XROrigin` even though the data from the XRHands subsystem is relative to the `XROrigin`. `JointFollower` transforms the location so that its not necessay for the components to be under the `XROrigin`.

## Interactables
### `HPUIBaseInteractable`
### `HPUIGeneratedContinuousInteractable`
### `HPUIMeshContinuousInteractable`

## Interactors
### `HPUIInteractor`
#### Detection logic
#### Gesture logic

## `JointFollower`


# Usage of V1 (Legacy)
The following is the documentation of the Legacy version of HPUI (the `Legacy` namespace) which did not use the XRI/XR-Hands.

A sample demo with v1 is available here: https://github.com/ahmed-shariff/SampleHPUI

## Prefabs
### DeformableSurface
- Set root's `TransformLinker` parent to the `palmbase` of hand
- Configure `width`, `height` and `HandCoordinateManager` (a hand proxy) of `DeformableCoordinateManager`
- Set the `handIndex` in `PlaneMeshGenerator` of the `Surface` GameObject. This refers to one of the `HandCoordinateManager`s in the `HandsManager`. The index refers to the index on the list of `HandCoordinateManager`s.
- Configure parents in `TransformLinker` of the `PlaneMeshTransformAnchors` GameObject
- Configure the offset by modifying the transform of the `offset` GameObject
- Add the instantiated prefab to the `managers` list in `CalibrateButton2` (in `InteractionManager` prefab)

### InteractionManager
- This has the `InteractionManager` component and also the `HandsManager` component. One instance of this prefab must be in the scene.
- Add the `HandCoordinateManager` in the corresponding indices under the `HandCoordinateManager` of the `HandManager` component.
- This also houses the `ThumbCollider` which dictates when a button can get triggered.
- The `InteractionManager` component dictates which `ButtonController` gets triggered when the `ThumbCollider` interacts with them.
- This also has the `CalibrateButton2`, which is used to setup the deformable surfaces. (`InteractionManager` > `TriggerBase` > `contactBtn`). When wearing the HMD you can trigger this by moving the thumb close to the base of the index. In Editor, you can configure a canvas button to trigger the `OnClick` method in the `CalibrateButton2` component. All deformable surfaces must be added to the list of `managers` in this component.

### OVRHandCoordinateProxyR
- Represents the proxy for the right hand when used with the OVRCustomHandPrefab from the oculus plugin.
- Drop this into the scene, and set the `SkeletonRoot` to the `OVRCustomHandPrefab` representing the right hand.
- Set the `TransformLinker`'s parent of `PalmBase_base` to `b_r_wrist` on the `OVRCustomHandPrefab` representing the right hand.

## Scripts
### HandCoordinateManager
- Use this to abstract the platform specific details of the hand coordinates used by HPUI. See `OVRHandCoordinateProxyR` for example.
- Set the `SkeletonRoot` to the GameObject that represents all the joints of the hand
- Set the GameObject representing the palm base as `PalmBase`
- Under `ManagedCoordinates`, list all the locations you may track.
- Under `ProxyToSeletonNameMapping`, add the proxyname and corresponding name on the skeleton being abstracted.


