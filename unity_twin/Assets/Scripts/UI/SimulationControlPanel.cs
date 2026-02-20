using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Simulation control panel: Play/Pause/Stop/Speed/Mode/Load G-code.
    /// </summary>
    public class SimulationControlPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;

        private Button playBtn, pauseBtn, stopBtn, loadGCodeBtn;
        private Slider speedSlider;
        private DropdownField modeDropdown;
        private Label speedLabel;

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            playBtn = root.Q<Button>("play-btn");
            pauseBtn = root.Q<Button>("pause-btn");
            stopBtn = root.Q<Button>("stop-btn");
            loadGCodeBtn = root.Q<Button>("load-gcode-btn");
            speedSlider = root.Q<Slider>("speed-slider");
            modeDropdown = root.Q<DropdownField>("mode-dropdown");
            speedLabel = root.Q<Label>("speed-label");

            playBtn?.RegisterCallback<ClickEvent>(_ => OnPlay());
            pauseBtn?.RegisterCallback<ClickEvent>(_ => OnPause());
            stopBtn?.RegisterCallback<ClickEvent>(_ => OnStop());
            loadGCodeBtn?.RegisterCallback<ClickEvent>(_ => OnLoadGCode());

            speedSlider?.RegisterValueChangedCallback(evt =>
            {
                float speed = evt.newValue;
                speedLabel?.SetText($"{speed:F1}x");
                if (speed > 1.1f)
                    SimulationClock.Instance?.SetAccelerated(speed);
                else
                    SimulationClock.Instance?.Play();
            });

            modeDropdown?.RegisterValueChangedCallback(evt =>
            {
                switch (evt.newValue)
                {
                    case "Real-Time":
                        SimulationClock.Instance?.Play();
                        break;
                    case "Accelerated":
                        SimulationClock.Instance?.SetAccelerated(
                            speedSlider?.value ?? 2f);
                        break;
                    case "Replay":
                        SimulationClock.Instance?.SetMode(SimulationClock.Mode.Replay);
                        break;
                }
            });
        }

        private void OnPlay() => SimulationClock.Instance?.Play();
        private void OnPause() => SimulationClock.Instance?.Pause();

        private void OnStop()
        {
            SimulationClock.Instance?.ResetSimulation();
        }

        private void OnLoadGCode()
        {
            Debug.Log("[SimControlPanel] Load G-code requested");
        }
    }
}
