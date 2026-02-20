using UnityEngine;

namespace MiracleTwin.CNC
{
    public enum MachineState { IDLE, RUNNING, PAUSED, ERROR, ESTOP, UNKNOWN }

    public interface ICNCController
    {
        string MachineName { get; }
        string MachineId { get; }
        string Status { get; }
        MachineState CurrentMachineState { get; }
        float SpindleRPM { get; }
        float FeedRate { get; }
        Vector3 AxisPositionsMM { get; }
        string CurrentProgram { get; }
        uint CurrentLine { get; }
        float SpindleLoad { get; }
        float CoolantLevel { get; }
        double CycleTimeElapsed { get; }
        double CycleTimeRemaining { get; }
        bool IsNearSoftLimit { get; }
        Vector3 MinPositionMM { get; }
        Vector3 MaxPositionMM { get; }
        float MaxRPM { get; }

        Vector3 GetToolTipWorldPosition();
        Vector3 GetToolDirection();
        void SetTargetPosition(Vector3 positionMM);
        void SetSpindleSpeed(float rpm);
        void SetFeedRate(float mmPerMin);
        void EmergencyStop();
        void Home();
    }
}
