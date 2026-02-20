using UnityEngine;
using UnityEngine.UIElements;

namespace MiracleTwin.UI
{
    /// <summary>
    /// In-app G-code text viewer/loader.
    /// Displays current program with line highlighting for active line.
    /// </summary>
    public class GCodeEditor : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;

        public string CurrentProgram { get; private set; }
        public int HighlightedLine { get; set; }

        private TextField codeField;
        private Label lineCountLabel;

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            codeField = root.Q<TextField>("gcode-editor");
            lineCountLabel = root.Q<Label>("line-count");
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
        }

        public string GetProgram()
        {
            return codeField?.value ?? CurrentProgram ?? "";
        }
    }
}
