using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Cutting;
using MiracleTwin.Core;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace MiracleTwin.UI
{
    /// <summary>
    /// In-app G-code text viewer/loader.
    /// Displays current program with line highlighting for active line.
    ///
    /// Phase 3 additions:
    /// - "Load File" button using EditorUtility.OpenFilePanel (editor mode)
    /// - GCodeExecutor reference for "Run" and "Stop" buttons
    /// - File filter for *.nc;*.gcode;*.ngc;*.tap
    /// </summary>
    public class GCodeEditor : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private GCodeExecutor gCodeExecutor;

        public string CurrentProgram { get; private set; }
        public int HighlightedLine { get; set; }

        private TextField codeField;
        private Label lineCountLabel;
        private Label progressLabel;
        private ProgressBar progressBar;
        private Button loadFileButton;
        private Button loadGcodeBtn;
        private Button runButton;
        private Button stopButton;

        private static readonly string[] SamplePrograms = {
            "pocket_50x50.nc", "face_3x3_block.nc", "contour_circle.nc", "full_demo.nc"
        };

        private RuntimeFileBrowser runtimeBrowser;

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            codeField = root.Q<TextField>("gcode-editor");
            lineCountLabel = root.Q<Label>("line-count");
            progressLabel = root.Q<Label>("gcode-progress");
            progressBar = root.Q<ProgressBar>("gcode-progress-bar");

            // Bind "Load File" button (right sidebar)
            loadFileButton = root.Q<Button>("load-file-button");
            if (loadFileButton != null)
                loadFileButton.clicked += OnLoadFileClicked;

            // Also bind the control bar "Load G-code" button
            loadGcodeBtn = root.Q<Button>("load-gcode-btn");
            if (loadGcodeBtn != null)
                loadGcodeBtn.clicked += OnLoadFileClicked;

            // Bind "Run" button
            runButton = root.Q<Button>("run-button");
            if (runButton != null)
                runButton.clicked += OnRunClicked;

            // Bind "Stop" button
            stopButton = root.Q<Button>("stop-button");
            if (stopButton != null)
                stopButton.clicked += OnStopClicked;
        }

        void Update()
        {
            // Keyboard shortcut: G = load first sample program and run it
            // Skip when a text field has focus (user is typing)
            if (!InputManager.IsTextInputFocused() && Input.GetKeyDown(KeyCode.G) && !Input.GetKey(KeyCode.LeftControl) && !Input.GetKey(KeyCode.LeftShift))
            {
                if (gCodeExecutor != null && !gCodeExecutor.IsExecuting)
                {
                    LoadSampleProgram(0);
                }
            }

            // Update progress display when executing
            if (gCodeExecutor != null && gCodeExecutor.IsExecuting)
            {
                float pct = gCodeExecutor.Progress * 100f;
                if (progressLabel != null)
                    progressLabel.text = $"{pct:F0}% | Seg {gCodeExecutor.CurrentSegmentIndex}/{gCodeExecutor.TotalSegments} | Line {gCodeExecutor.CurrentGCodeLine}";
                if (progressBar != null)
                    progressBar.value = pct;
            }
            else if (gCodeExecutor == null || !gCodeExecutor.IsExecuting)
            {
                if (progressLabel != null && !string.IsNullOrEmpty(progressLabel.text))
                    progressLabel.text = "";
                if (progressBar != null && progressBar.value > 0)
                    progressBar.value = 0;
            }
        }

        void OnDestroy()
        {
            if (loadFileButton != null)
                loadFileButton.clicked -= OnLoadFileClicked;
            if (loadGcodeBtn != null)
                loadGcodeBtn.clicked -= OnLoadFileClicked;
            if (runButton != null)
                runButton.clicked -= OnRunClicked;
            if (stopButton != null)
                stopButton.clicked -= OnStopClicked;
        }

        /// <summary>
        /// Handle "Load File" button click.
        /// Opens a native file browser filtered for G-code file types.
        /// Uses EditorUtility.OpenFilePanel in editor mode.
        /// </summary>
        private void OnLoadFileClicked()
        {
#if UNITY_EDITOR
            string path = EditorUtility.OpenFilePanel(
                "Open G-Code File",
                "",
                "nc,gcode,ngc,tap"
            );

            if (!string.IsNullOrEmpty(path))
            {
                LoadFromFile(path);
                Debug.Log($"[GCodeEditor] Loaded G-code file: {path}");
            }
#else
            ShowRuntimeFileBrowser();
#endif
        }

        /// <summary>
        /// Handle "Run" button click.
        /// Sends the current program text to GCodeExecutor for execution.
        /// </summary>
        private void OnRunClicked()
        {
            if (gCodeExecutor == null)
            {
                Debug.LogWarning("[GCodeEditor] No GCodeExecutor assigned. Cannot run program.");
                return;
            }

            string program = GetProgram();
            if (string.IsNullOrWhiteSpace(program))
            {
                Debug.LogWarning("[GCodeEditor] No program loaded. Load or type a G-code program first.");
                return;
            }

            gCodeExecutor.LoadAndExecute(program);
        }

        /// <summary>
        /// Handle "Stop" button click.
        /// Stops the currently executing G-code program.
        /// </summary>
        private void OnStopClicked()
        {
            if (gCodeExecutor == null)
            {
                Debug.LogWarning("[GCodeEditor] No GCodeExecutor assigned. Cannot stop.");
                return;
            }

            gCodeExecutor.Stop();
            Debug.Log("[GCodeEditor] Execution stopped.");
        }

        public void LoadProgram(string programText)
        {
            CurrentProgram = programText;
            if (codeField != null)
                codeField.value = programText;

            int lineCount = programText.Split('\n').Length;
            lineCountLabel?.SetText($"{lineCount} lines");
        }

        public void LoadFromFile(string path)
        {
            if (System.IO.File.Exists(path))
            {
                string content = System.IO.File.ReadAllText(path);
                LoadProgram(content);
            }
            else
            {
                Debug.LogWarning($"[GCodeEditor] File not found: {path}");
            }
        }

        public string GetProgram()
        {
            return codeField?.value ?? CurrentProgram ?? "";
        }

        /// <summary>
        /// Swap the GCodeExecutor reference (called by CNCMachineSelector on machine switch).
        /// </summary>
        public void SetExecutor(GCodeExecutor executor)
        {
            gCodeExecutor = executor;
        }

        /// <summary>
        /// Load a sample G-code program from StreamingAssets/SamplePrograms/ and run it.
        /// </summary>
        public void LoadSampleProgram(int index)
        {
            if (index < 0 || index >= SamplePrograms.Length) return;

            string fileName = SamplePrograms[index];
            string path = System.IO.Path.Combine(Application.streamingAssetsPath, "SamplePrograms", fileName);

            if (System.IO.File.Exists(path))
            {
                LoadFromFile(path);
                Debug.Log($"[GCodeEditor] Loaded sample program: {fileName}");

                // Auto-run after loading
                if (gCodeExecutor != null)
                    gCodeExecutor.LoadAndExecute(GetProgram());
            }
            else
            {
                Debug.LogWarning($"[GCodeEditor] Sample program not found: {path}");
            }
        }

        /// <summary>
        /// Show the runtime file browser for standalone builds.
        /// </summary>
        private void ShowRuntimeFileBrowser()
        {
            if (uiDocument == null) return;

            if (runtimeBrowser == null)
                runtimeBrowser = new RuntimeFileBrowser();

            if (runtimeBrowser.IsOpen) return;

            runtimeBrowser.Show(uiDocument.rootVisualElement, path =>
            {
                if (!string.IsNullOrEmpty(path))
                {
                    LoadFromFile(path);
                    Debug.Log($"[GCodeEditor] Loaded G-code from runtime browser: {path}");
                }
            });
        }
    }
}
