using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Cutting;
using MiracleTwin.UI;
using MiracleTwin.Visualization;

namespace MiracleTwin.CNC
{
    /// <summary>
    /// Manages switching between multiple CNC machines in the scene.
    /// Only one CNC machine is active at a time. Integrates with
    /// the dashboard dropdown to let users select machines at runtime.
    /// </summary>
    public class CNCMachineSelector : MonoBehaviour
    {
        [Header("Available Machines")]
        [Tooltip("All CNC machine GameObjects. Each must have a component implementing ICNCController.")]
        [SerializeField] private List<MonoBehaviour> machineControllers = new();

        [Header("UI")]
        [SerializeField] private UIDocument uiDocument;
        [Tooltip("Name of the DropdownField in the UXML to populate with machine names.")]
        [SerializeField] private string dropdownName = "machine-dropdown";

        private DropdownField dropdown;
        private int activeIndex = -1;

        /// <summary>The currently active CNC controller, or null if none.</summary>
        public ICNCController ActiveMachine { get; private set; }

        /// <summary>The currently active CNC MonoBehaviour, for assigning to CuttingSimulationManager etc.</summary>
        public MonoBehaviour ActiveMachineBehaviour { get; private set; }

        void Start()
        {
            // Validate controllers implement ICNCController
            for (int i = machineControllers.Count - 1; i >= 0; i--)
            {
                if (machineControllers[i] == null || machineControllers[i] is not ICNCController)
                {
                    Debug.LogWarning($"[CNCMachineSelector] Entry {i} is null or does not implement ICNCController. Removing.");
                    machineControllers.RemoveAt(i);
                }
            }

            if (machineControllers.Count == 0)
            {
                Debug.LogWarning("[CNCMachineSelector] No valid CNC machine controllers assigned.");
                return;
            }

            // Select first machine by default
            SelectMachine(0);

            // Bind to dropdown if UIDocument is available
            BindDropdown();
        }

        private void BindDropdown()
        {
            if (uiDocument == null) return;

            var root = uiDocument.rootVisualElement;
            if (root == null) return;

            dropdown = root.Q<DropdownField>(dropdownName);
            if (dropdown == null) return;

            var choices = new List<string>();
            foreach (var mc in machineControllers)
            {
                var cnc = mc as ICNCController;
                choices.Add(cnc?.MachineName ?? mc.gameObject.name);
            }

            dropdown.choices = choices;
            dropdown.index = activeIndex;

            dropdown.RegisterValueChangedCallback(evt =>
            {
                int idx = dropdown.index;
                if (idx >= 0 && idx < machineControllers.Count)
                    SelectMachine(idx);
            });
        }

        /// <summary>
        /// Activate the machine at the given index and deactivate all others.
        /// </summary>
        public void SelectMachine(int index)
        {
            if (index < 0 || index >= machineControllers.Count) return;
            if (index == activeIndex) return;

            // Enable only the selected controller (keep all GameObjects visible)
            for (int i = 0; i < machineControllers.Count; i++)
            {
                if (machineControllers[i] != null)
                    machineControllers[i].enabled = (i == index);
            }

            activeIndex = index;
            ActiveMachineBehaviour = machineControllers[index];
            ActiveMachine = ActiveMachineBehaviour as ICNCController;

            Debug.Log($"[CNCMachineSelector] Active machine: {ActiveMachine?.MachineName ?? "Unknown"}");

            // Notify DashboardOverlay to swap CuttingSimulationManager reference
            var simManager = ActiveMachineBehaviour.GetComponent<CuttingSimulationManager>();
            if (simManager != null)
            {
                var overlay = Object.FindFirstObjectByType<DashboardOverlay>();
                if (overlay != null)
                    overlay.SetActiveCuttingSimManager(simManager);
            }

            // Swap GCodeEditor → GCodeExecutor reference for the active machine
            var newExecutor = ActiveMachineBehaviour.GetComponent<GCodeExecutor>();
            if (newExecutor != null)
            {
                var gCodeEditor = Object.FindFirstObjectByType<GCodeEditor>();
                if (gCodeEditor != null)
                    gCodeEditor.SetExecutor(newExecutor);

                var toolpathPreview = Object.FindFirstObjectByType<ToolpathPreview>();
                if (toolpathPreview != null)
                    toolpathPreview.SetExecutor(newExecutor);
            }

            // Clear all chart buffers on machine switch
            Object.FindFirstObjectByType<ForceChart>()?.Clear();
            Object.FindFirstObjectByType<ThermalChart>()?.Clear();
            Object.FindFirstObjectByType<ToolWearChart>()?.Clear();
            Object.FindFirstObjectByType<PowerChart>()?.Clear();

            // Enable/disable per-machine visualization overlays
            for (int i = 0; i < machineControllers.Count; i++)
            {
                bool isActive = (i == index);
                var mc = machineControllers[i];
                if (mc == null) continue;

                var heatMap = mc.GetComponentInChildren<HeatMapOverlay>(true);
                if (heatMap != null) heatMap.enabled = isActive;

                var forceArrows = mc.GetComponentInChildren<ForceArrowRenderer>(true);
                if (forceArrows != null) forceArrows.enabled = isActive;

                var surfaceRoughness = mc.GetComponentInChildren<SurfaceRoughnessOverlay>(true);
                if (surfaceRoughness != null) surfaceRoughness.enabled = isActive;
            }

            // Update dropdown if it exists
            if (dropdown != null)
                dropdown.SetValueWithoutNotify(dropdown.choices[index]);
        }

        /// <summary>Select a machine by name.</summary>
        public void SelectMachine(string machineName)
        {
            for (int i = 0; i < machineControllers.Count; i++)
            {
                var cnc = machineControllers[i] as ICNCController;
                if (cnc != null && cnc.MachineName == machineName)
                {
                    SelectMachine(i);
                    return;
                }
            }
            Debug.LogWarning($"[CNCMachineSelector] Machine not found: {machineName}");
        }
    }
}
