using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UIElements;

namespace MiracleTwin.UI
{
    /// <summary>
    /// In-game file browser for standalone builds where native OS file dialogs
    /// are unavailable. Scans StreamingAssets/SamplePrograms/ and
    /// persistentDataPath/GCode/ for .nc, .gcode, .ngc, .tap files.
    /// Presents a modal overlay with a scrollable list and Open/Cancel buttons.
    /// </summary>
    public class RuntimeFileBrowser
    {
        private static readonly string[] Extensions = { ".nc", ".gcode", ".ngc", ".tap" };

        private VisualElement overlay;
        private VisualElement root;
        private Action<string> onFileSelected;
        private ListView fileListView;
        private List<string> filePaths = new();
        private List<string> fileNames = new();

        public bool IsOpen => overlay != null && overlay.style.display == DisplayStyle.Flex;

        /// <summary>
        /// Show the file browser overlay.
        /// </summary>
        /// <param name="rootElement">The UIDocument root to attach to.</param>
        /// <param name="callback">Called with the selected file path, or null if cancelled.</param>
        public void Show(VisualElement rootElement, Action<string> callback)
        {
            root = rootElement;
            onFileSelected = callback;

            ScanForFiles();
            BuildUI();

            root.Add(overlay);
        }

        /// <summary>Close the file browser without selecting a file.</summary>
        public void Close()
        {
            onFileSelected?.Invoke(null);
            RemoveOverlay();
        }

        private void ScanForFiles()
        {
            filePaths.Clear();
            fileNames.Clear();

            // 1. StreamingAssets/SamplePrograms/
            string samplesDir = Path.Combine(Application.streamingAssetsPath, "SamplePrograms");
            ScanDirectory(samplesDir);

            // 2. persistentDataPath/GCode/
            string userDir = Path.Combine(Application.persistentDataPath, "GCode");
            ScanDirectory(userDir);
        }

        private void ScanDirectory(string directory)
        {
            if (!Directory.Exists(directory)) return;

            foreach (string ext in Extensions)
            {
                string pattern = "*" + ext;
                try
                {
                    foreach (string file in Directory.GetFiles(directory, pattern, SearchOption.AllDirectories))
                    {
                        filePaths.Add(file);
                        fileNames.Add(Path.GetFileName(file));
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[RuntimeFileBrowser] Error scanning {directory}: {ex.Message}");
                }
            }
        }

        private void BuildUI()
        {
            // Dark semi-transparent overlay
            overlay = new VisualElement();
            overlay.style.position = Position.Absolute;
            overlay.style.left = 0;
            overlay.style.top = 0;
            overlay.style.right = 0;
            overlay.style.bottom = 0;
            overlay.style.backgroundColor = new Color(0, 0, 0, 0.7f);
            overlay.style.alignItems = Align.Center;
            overlay.style.justifyContent = Justify.Center;
            overlay.pickingMode = PickingMode.Position;

            // Modal panel
            var panel = new VisualElement();
            panel.style.width = 400;
            panel.style.maxHeight = 500;
            panel.style.backgroundColor = new Color(0.12f, 0.12f, 0.15f, 1f);
            panel.style.borderTopLeftRadius = 8;
            panel.style.borderTopRightRadius = 8;
            panel.style.borderBottomLeftRadius = 8;
            panel.style.borderBottomRightRadius = 8;
            panel.style.paddingTop = 16;
            panel.style.paddingBottom = 16;
            panel.style.paddingLeft = 16;
            panel.style.paddingRight = 16;
            panel.style.borderTopWidth = 1;
            panel.style.borderBottomWidth = 1;
            panel.style.borderLeftWidth = 1;
            panel.style.borderRightWidth = 1;
            panel.style.borderTopColor = new Color(0.3f, 0.5f, 0.8f, 0.6f);
            panel.style.borderBottomColor = new Color(0.3f, 0.5f, 0.8f, 0.6f);
            panel.style.borderLeftColor = new Color(0.3f, 0.5f, 0.8f, 0.6f);
            panel.style.borderRightColor = new Color(0.3f, 0.5f, 0.8f, 0.6f);

            // Title
            var title = new Label("Open G-Code File");
            title.style.fontSize = 16;
            title.style.color = new Color(0.85f, 0.9f, 1f, 1f);
            title.style.unityFontStyleAndWeight = FontStyle.Bold;
            title.style.marginBottom = 12;
            panel.Add(title);

            // File count label
            var countLabel = new Label($"{filePaths.Count} file(s) found");
            countLabel.style.fontSize = 11;
            countLabel.style.color = new Color(0.6f, 0.6f, 0.7f, 1f);
            countLabel.style.marginBottom = 8;
            panel.Add(countLabel);

            // File list
            if (filePaths.Count > 0)
            {
                fileListView = new ListView(fileNames, 28, MakeItem, BindItem);
                fileListView.style.flexGrow = 1;
                fileListView.style.maxHeight = 300;
                fileListView.style.backgroundColor = new Color(0.08f, 0.08f, 0.1f, 1f);
                fileListView.style.borderTopLeftRadius = 4;
                fileListView.style.borderTopRightRadius = 4;
                fileListView.style.borderBottomLeftRadius = 4;
                fileListView.style.borderBottomRightRadius = 4;
                fileListView.selectionType = SelectionType.Single;
                panel.Add(fileListView);
            }
            else
            {
                var noFiles = new Label("No G-code files found.\nPlace .nc/.gcode/.ngc/.tap files in:\n  StreamingAssets/SamplePrograms/\n  or persistentDataPath/GCode/");
                noFiles.style.whiteSpace = WhiteSpace.Normal;
                noFiles.style.fontSize = 12;
                noFiles.style.color = new Color(0.7f, 0.5f, 0.3f, 1f);
                noFiles.style.marginTop = 16;
                noFiles.style.marginBottom = 16;
                panel.Add(noFiles);
            }

            // Button row
            var buttonRow = new VisualElement();
            buttonRow.style.flexDirection = FlexDirection.Row;
            buttonRow.style.justifyContent = Justify.FlexEnd;
            buttonRow.style.marginTop = 12;

            var cancelBtn = new Button(() => { RemoveOverlay(); onFileSelected?.Invoke(null); });
            cancelBtn.text = "Cancel";
            cancelBtn.style.marginRight = 8;
            cancelBtn.style.paddingLeft = 16;
            cancelBtn.style.paddingRight = 16;
            buttonRow.Add(cancelBtn);

            var openBtn = new Button(OnOpenClicked);
            openBtn.text = "Open";
            openBtn.style.paddingLeft = 20;
            openBtn.style.paddingRight = 20;
            openBtn.style.backgroundColor = new Color(0.2f, 0.4f, 0.7f, 1f);
            openBtn.style.color = Color.white;
            buttonRow.Add(openBtn);

            panel.Add(buttonRow);
            overlay.Add(panel);
        }

        private VisualElement MakeItem()
        {
            var label = new Label();
            label.style.fontSize = 12;
            label.style.color = new Color(0.8f, 0.85f, 0.9f, 1f);
            label.style.paddingLeft = 8;
            label.style.paddingTop = 4;
            label.style.paddingBottom = 4;
            return label;
        }

        private void BindItem(VisualElement element, int index)
        {
            (element as Label).text = fileNames[index];
        }

        private void OnOpenClicked()
        {
            if (fileListView == null || fileListView.selectedIndex < 0)
            {
                Debug.LogWarning("[RuntimeFileBrowser] No file selected.");
                return;
            }

            int idx = fileListView.selectedIndex;
            string path = filePaths[idx];
            RemoveOverlay();
            onFileSelected?.Invoke(path);
        }

        private void RemoveOverlay()
        {
            if (overlay != null && root != null)
            {
                root.Remove(overlay);
            }
            overlay = null;
        }
    }
}
