using UnityEditor;
using ubco.ovilab.HPUI.Interaction;

using System.Collections.Generic;

namespace ubco.ovilab.HPUI.Editor
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(HPUIBaseInteractable), true)]
    public class HPUIBaseInteractableEditor: UnityEditor.XR.Interaction.Toolkit.Interactables.XRBaseInteractableEditor
    {
        private HPUIBaseInteractable t;
        protected List<SerializedProperty> eventProperties;
        protected virtual List<string> EventPropertyNames => new List<string>() { "tapEvent", "gestureEvent" };

        protected bool hpuiInteractablesExpanded;

        /// <inheritdoc />
        protected override void OnEnable()
        {
            base.OnEnable();
            t = target as HPUIBaseInteractable;

            eventProperties = new List<SerializedProperty>();
            foreach (string eventName in EventPropertyNames)
            {
                eventProperties.Add(serializedObject.FindProperty(eventName));
            }
        }

        /// <inheritdoc />
        protected override void DrawInspector()
        {
            base.DrawInspector();

            EditorGUILayout.Space();

            hpuiInteractablesExpanded = EditorGUILayout.Foldout(hpuiInteractablesExpanded, EditorGUIUtility.TrTempContent("HPUI Events"), true);
            if (hpuiInteractablesExpanded)
            {
                using (new EditorGUI.IndentLevelScope())
                {
                    foreach (SerializedProperty property in eventProperties)
                    {
                        EditorGUILayout.PropertyField(property);
                    }
                }
            }
        }

        /// <inheritdoc />
        protected override List<string> GetDerivedSerializedPropertyNames()
        {
            List<string> props = base.GetDerivedSerializedPropertyNames();
            props.AddRange(EventPropertyNames);
            return props;
        }

        // Selection mode and distance calculation mode are programatically set.
        /// <inheritdoc />
        protected override void DrawSelectionConfiguration() { }

        /// <inheritdoc />
        protected override void DrawDistanceCalculationMode() { }
    }
}
