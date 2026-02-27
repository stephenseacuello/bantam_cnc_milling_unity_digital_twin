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
        private Button recordBtn, replayBtn;
        private Slider replayTimeline;
        private Label replayTimeLabel;
        private DataRecorder dataRecorder;
        private ReplayController replayController;

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
            recordBtn = root.Q<Button>("record-btn");
            replayBtn = root.Q<Button>("replay-btn");
            replayTimeline = root.Q<Slider>("replay-timeline");
            replayTimeLabel = root.Q<Label>("replay-time-label");

            dataRecorder = Object.FindFirstObjectByType<DataRecorder>();
            replayController = Object.FindFirstObjectByType<ReplayController>();

            recordBtn?.RegisterCallback<ClickEvent>(_ => OnRecord());
            replayBtn?.RegisterCallback<ClickEvent>(_ => OnReplay());
            replayTimeline?.RegisterValueChangedCallback(evt =>
            {
                replayController?.Seek(evt.newValue);
            });

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

        private void OnRecord()
        {
            if (dataRecorder == null) return;
            dataRecorder.ToggleRecording();

            if (dataRecorder.IsRecording)
            {
                recordBtn?.SetText("STOP");
                recordBtn?.AddToClassList("recording");
            }
            else
            {
                recordBtn?.SetText("REC");
                recordBtn?.RemoveFromClassList("recording");
            }
        }

        private void OnReplay()
        {
            if (replayController == null) return;

            if (replayController.IsPlaying)
            {
                replayController.PauseReplay();
                replayBtn?.SetText("Replay");
                return;
            }

            if (!replayController.IsLoaded)
            {
                // Load most recent .miracle file
                string folder = System.IO.Path.Combine(
                    Application.streamingAssetsPath, "Recordings");
                if (System.IO.Directory.Exists(folder))
                {
                    var files = System.IO.Directory.GetFiles(folder, "*.miracle");
                    if (files.Length > 0)
                    {
                        System.Array.Sort(files);
                        replayController.LoadRecording(files[files.Length - 1]);
                    }
                }
            }

            if (replayController.IsLoaded)
            {
                replayController.Play();
                replayBtn?.SetText("Pause");
            }
        }

        void Update()
        {
            if (replayController != null && replayController.IsPlaying)
            {
                if (replayTimeline != null)
                    replayTimeline.SetValueWithoutNotify(replayController.Progress);
                if (replayTimeLabel != null)
                {
                    double t = replayController.CurrentTime;
                    int min = (int)(t / 60);
                    double sec = t % 60;
                    replayTimeLabel.text = $"{min}:{sec:00.0}";
                }
            }
        }
    }
}
