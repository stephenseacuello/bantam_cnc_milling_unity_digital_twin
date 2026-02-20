using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Robots;

namespace MiracleTwin.UI
{
    /// <summary>Per-robot status display: state, assigned task, and joint positions.</summary>
    public class RobotStatusPanel : MonoBehaviour
    {
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private MultiAgentCoordinator coordinator;

        private Label ned2StatusLabel, xarm6StatusLabel;
        private Label lastAwardLabel, cyclesLabel;

        void Start()
        {
            if (uiDocument == null) return;
            var root = uiDocument.rootVisualElement;

            ned2StatusLabel = root.Q<Label>("ned2-status");
            xarm6StatusLabel = root.Q<Label>("xarm6-status");
            lastAwardLabel = root.Q<Label>("last-award");
            cyclesLabel = root.Q<Label>("cycles-completed");
        }

        void Update()
        {
            if (coordinator == null) return;

            ned2StatusLabel?.SetText($"Ned2: {coordinator.Ned2Status}");
            xarm6StatusLabel?.SetText($"xArm6: {coordinator.XArm6Status}");
            lastAwardLabel?.SetText($"Last: {coordinator.LastAwardedRobot ?? "None"}");
            cyclesLabel?.SetText($"Cycles: {coordinator.TotalCyclesCompleted}");
        }
    }
}
