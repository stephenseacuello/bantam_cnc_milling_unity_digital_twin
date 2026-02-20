using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class MachineStateMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/MachineState";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string status;
        public double spindle_speed;
        public double feed_rate;
        public double[] axis_positions;
        public double[] axis_velocities;
        public double spindle_load;
        public double coolant_level;
        public string current_program;
        public uint current_line;
        public double cycle_time_elapsed;
        public double cycle_time_remaining;

        public MachineStateMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.status = "IDLE";
            this.spindle_speed = 0.0;
            this.feed_rate = 0.0;
            this.axis_positions = new double[0];
            this.axis_velocities = new double[0];
            this.spindle_load = 0.0;
            this.coolant_level = 0.0;
            this.current_program = "";
            this.current_line = 0;
            this.cycle_time_elapsed = 0.0;
            this.cycle_time_remaining = 0.0;
        }

        public MachineStateMsg(
            TimeMsg timestamp,
            string machine_id,
            string status,
            double spindle_speed,
            double feed_rate,
            double[] axis_positions,
            double[] axis_velocities,
            double spindle_load,
            double coolant_level,
            string current_program,
            uint current_line,
            double cycle_time_elapsed,
            double cycle_time_remaining)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.status = status;
            this.spindle_speed = spindle_speed;
            this.feed_rate = feed_rate;
            this.axis_positions = axis_positions;
            this.axis_velocities = axis_velocities;
            this.spindle_load = spindle_load;
            this.coolant_level = coolant_level;
            this.current_program = current_program;
            this.current_line = current_line;
            this.cycle_time_elapsed = cycle_time_elapsed;
            this.cycle_time_remaining = cycle_time_remaining;
        }

        public static MachineStateMsg Deserialize(MessageDeserializer deserializer) => new MachineStateMsg(deserializer);

        private MachineStateMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.status);
            deserializer.Read(out this.spindle_speed);
            deserializer.Read(out this.feed_rate);
            deserializer.Read(out this.axis_positions, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.axis_velocities, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.spindle_load);
            deserializer.Read(out this.coolant_level);
            deserializer.Read(out this.current_program);
            deserializer.Read(out this.current_line);
            deserializer.Read(out this.cycle_time_elapsed);
            deserializer.Read(out this.cycle_time_remaining);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.status);
            serializer.Write(this.spindle_speed);
            serializer.Write(this.feed_rate);
            serializer.WriteLength(this.axis_positions);
            serializer.Write(this.axis_positions);
            serializer.WriteLength(this.axis_velocities);
            serializer.Write(this.axis_velocities);
            serializer.Write(this.spindle_load);
            serializer.Write(this.coolant_level);
            serializer.Write(this.current_program);
            serializer.Write(this.current_line);
            serializer.Write(this.cycle_time_elapsed);
            serializer.Write(this.cycle_time_remaining);
        }

        public override string ToString()
        {
            return "MachineStateMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nstatus: " + status.ToString() +
            "\nspindle_speed: " + spindle_speed.ToString() +
            "\nfeed_rate: " + feed_rate.ToString() +
            "\naxis_positions: " + System.String.Join(", ", axis_positions.ToList()) +
            "\naxis_velocities: " + System.String.Join(", ", axis_velocities.ToList()) +
            "\nspindle_load: " + spindle_load.ToString() +
            "\ncoolant_level: " + coolant_level.ToString() +
            "\ncurrent_program: " + current_program.ToString() +
            "\ncurrent_line: " + current_line.ToString() +
            "\ncycle_time_elapsed: " + cycle_time_elapsed.ToString() +
            "\ncycle_time_remaining: " + cycle_time_remaining.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
