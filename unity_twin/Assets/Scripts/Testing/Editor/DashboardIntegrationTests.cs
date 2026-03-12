#if UNITY_EDITOR
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UIElements;

namespace MiracleTwin.Testing.Editor
{
    /// <summary>
    /// Integration tests validating that DashboardWiring produces a fully
    /// connected dashboard and that UXML elements expected by runtime code
    /// are present in the visual tree.
    /// </summary>
    [TestFixture]
    public class DashboardIntegrationTests
    {
        private VisualTreeAsset dashboardUxml;
        private VisualElement root;

        [OneTimeSetUp]
        public void LoadDashboardUxml()
        {
            dashboardUxml = UnityEditor.AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(
                "Assets/UI/Dashboard.uxml");
            Assert.IsNotNull(dashboardUxml, "Dashboard.uxml not found at Assets/UI/Dashboard.uxml");

            root = dashboardUxml.Instantiate();
            Assert.IsNotNull(root, "Dashboard.uxml failed to instantiate");
        }

        // ── Layout Structure ───────────────────────────────────────────────

        [Test]
        public void DashboardRoot_Exists()
        {
            Assert.IsNotNull(root.Q("dashboard-root"), "dashboard-root element missing");
        }

        [Test]
        public void Header_ContainsTitleAndConnectionBadge()
        {
            Assert.IsNotNull(root.Q<Label>("title"), "title label missing");
            Assert.IsNotNull(root.Q<Label>("connection-status"), "connection-status label missing");
        }

        [Test]
        public void MachineDropdown_Exists()
        {
            Assert.IsNotNull(root.Q<DropdownField>("machine-dropdown"),
                "machine-dropdown DropdownField missing");
        }

        [Test]
        public void LeftSidebar_Exists()
        {
            Assert.IsNotNull(root.Q("left-sidebar"), "left-sidebar element missing");
        }

        [Test]
        public void Viewport_Exists()
        {
            Assert.IsNotNull(root.Q("viewport"), "viewport element missing");
        }

        [Test]
        public void RightSidebar_GCodeEditor_Exists()
        {
            Assert.IsNotNull(root.Q("right-sidebar"), "right-sidebar missing");
            Assert.IsNotNull(root.Q<TextField>("gcode-editor"), "gcode-editor TextField missing");
        }

        // ── Machine Status Data Labels ─────────────────────────────────────

        [Test]
        public void MachinePanel_HasAllDataValues()
        {
            string[] requiredNames = {
                "status-value", "program-value", "rpm-value", "feed-value",
                "load-value", "line-value", "pos-x-value", "pos-y-value", "pos-z-value"
            };
            foreach (var name in requiredNames)
            {
                Assert.IsNotNull(root.Q<Label>(name), $"Label '{name}' missing from Machine panel");
            }
        }

        // ── Force Labels ───────────────────────────────────────────────────

        [Test]
        public void ForcePanel_HasFxFyFzLabels()
        {
            Assert.IsNotNull(root.Q<Label>("force-x-value"), "force-x-value missing");
            Assert.IsNotNull(root.Q<Label>("force-y-value"), "force-y-value missing");
            Assert.IsNotNull(root.Q<Label>("force-z-value"), "force-z-value missing");
        }

        // ── Tool Wear ──────────────────────────────────────────────────────

        [Test]
        public void ToolWearPanel_HasProgressBarAndLabels()
        {
            Assert.IsNotNull(root.Q<ProgressBar>("wear-bar"), "wear-bar ProgressBar missing");
            Assert.IsNotNull(root.Q<Label>("wear-vb-value"), "wear-vb-value missing");
            Assert.IsNotNull(root.Q<Label>("wear-pct-value"), "wear-pct-value missing");
            Assert.IsNotNull(root.Q<Label>("wear-life-value"), "wear-life-value missing");
        }

        // ── Temperature ────────────────────────────────────────────────────

        [Test]
        public void TemperaturePanel_HasLabels()
        {
            Assert.IsNotNull(root.Q<Label>("tool-temp-value"), "tool-temp-value missing");
            Assert.IsNotNull(root.Q<Label>("interface-temp-value"), "interface-temp-value missing");
        }

        // ── Charts ─────────────────────────────────────────────────────────

        [Test]
        public void Charts_ForceChartElement_Exists()
        {
            Assert.IsNotNull(root.Q("force-chart"), "force-chart container missing");
        }

        [Test]
        public void Charts_ThermalChartElement_Exists()
        {
            Assert.IsNotNull(root.Q("thermal-chart"), "thermal-chart container missing");
        }

        [Test]
        public void Charts_ToolWearChartElement_Exists()
        {
            Assert.IsNotNull(root.Q("tool-wear-chart"), "tool-wear-chart container missing");
        }

        [Test]
        public void Charts_PowerChartElement_Exists()
        {
            Assert.IsNotNull(root.Q("power-chart"), "power-chart container missing");
        }

        // ── G-Code Editor Panel ────────────────────────────────────────────

        [Test]
        public void GCodePanel_HasToolbarButtons()
        {
            Assert.IsNotNull(root.Q<Button>("load-file-button"), "load-file-button missing");
            Assert.IsNotNull(root.Q<Button>("run-button"), "run-button missing");
            Assert.IsNotNull(root.Q<Button>("stop-button"), "stop-button missing");
        }

        [Test]
        public void GCodePanel_HasProgressBar()
        {
            Assert.IsNotNull(root.Q<ProgressBar>("gcode-progress-bar"),
                "gcode-progress-bar missing");
        }

        // ── Control Bar ────────────────────────────────────────────────────

        [Test]
        public void ControlBar_HasPlayPauseStopButtons()
        {
            Assert.IsNotNull(root.Q<Button>("play-btn"), "play-btn missing");
            Assert.IsNotNull(root.Q<Button>("pause-btn"), "pause-btn missing");
            Assert.IsNotNull(root.Q<Button>("stop-btn"), "stop-btn missing");
        }

        [Test]
        public void ControlBar_HasSpeedSliderAndLabel()
        {
            Assert.IsNotNull(root.Q<Slider>("speed-slider"), "speed-slider missing");
            Assert.IsNotNull(root.Q<Label>("speed-label"), "speed-label missing");
        }

        [Test]
        public void ControlBar_HasModeDropdown()
        {
            Assert.IsNotNull(root.Q<DropdownField>("mode-dropdown"), "mode-dropdown missing");
        }

        [Test]
        public void ControlBar_HasSimTimeLabel()
        {
            Assert.IsNotNull(root.Q<Label>("sim-time-value"), "sim-time-value missing");
        }

        [Test]
        public void ControlBar_HasEStopButton()
        {
            var estop = root.Q<Button>("estop-btn");
            Assert.IsNotNull(estop, "estop-btn missing");
            Assert.AreEqual("E-STOP", estop.text);
        }

        [Test]
        public void ControlBar_HasRobotStatusLabels()
        {
            Assert.IsNotNull(root.Q<Label>("ned2-status"), "ned2-status missing");
            Assert.IsNotNull(root.Q<Label>("xarm6-status"), "xarm6-status missing");
        }

        // ── Record / Replay UI ─────────────────────────────────────────────

        [Test]
        public void ControlBar_HasRecordButton()
        {
            var btn = root.Q<Button>("record-btn");
            Assert.IsNotNull(btn, "record-btn missing");
            Assert.AreEqual("REC", btn.text);
        }

        [Test]
        public void ControlBar_HasReplayButton()
        {
            Assert.IsNotNull(root.Q<Button>("replay-btn"), "replay-btn missing");
        }

        [Test]
        public void ControlBar_HasReplayTimeline()
        {
            var slider = root.Q<Slider>("replay-timeline");
            Assert.IsNotNull(slider, "replay-timeline slider missing");
        }

        [Test]
        public void ControlBar_HasReplayTimeLabel()
        {
            Assert.IsNotNull(root.Q<Label>("replay-time-label"), "replay-time-label missing");
        }

        // ── OEE Panel ──────────────────────────────────────────────────────

        [Test]
        public void OEEPanel_HasLabels()
        {
            Assert.IsNotNull(root.Q<Label>("oee-value"), "oee-value missing");
            Assert.IsNotNull(root.Q<Label>("availability-value"), "availability-value missing");
            Assert.IsNotNull(root.Q<Label>("performance-value"), "performance-value missing");
            Assert.IsNotNull(root.Q<Label>("quality-value"), "quality-value missing");
        }

        // ── Surface Roughness ──────────────────────────────────────────────

        [Test]
        public void SurfaceRoughnessPanel_HasLabels()
        {
            Assert.IsNotNull(root.Q<Label>("roughness-ra-value"), "roughness-ra-value missing");
            Assert.IsNotNull(root.Q<Label>("roughness-grade-value"), "roughness-grade-value missing");
        }

        // ── Sensor Panel ───────────────────────────────────────────────────

        [Test]
        public void SensorPanel_HasLabels()
        {
            string[] sensorLabels = {
                "sensor-vibration-value", "sensor-current-value",
                "sensor-temp-value", "sensor-acoustic-value",
                "sensor-coolant-value", "sensor-status-value"
            };
            foreach (var name in sensorLabels)
            {
                Assert.IsNotNull(root.Q<Label>(name), $"Label '{name}' missing from Sensor panel");
            }
        }
    }
}
#endif
