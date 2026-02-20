using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class GCodeBlockMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/GCodeBlock";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string program_name;
        public uint line_number;
        public string raw_line;
        public string command;
        public double[] parameters;
        public double feed_rate;
        public double spindle_speed;
        public string comment;
        public bool is_rapid;

        public GCodeBlockMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.program_name = "";
            this.line_number = 0;
            this.raw_line = "";
            this.command = "";
            this.parameters = new double[0];
            this.feed_rate = 0.0;
            this.spindle_speed = 0.0;
            this.comment = "";
            this.is_rapid = false;
        }

        public GCodeBlockMsg(
            TimeMsg timestamp,
            string machine_id,
            string program_name,
            uint line_number,
            string raw_line,
            string command,
            double[] parameters,
            double feed_rate,
            double spindle_speed,
            string comment,
            bool is_rapid)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.program_name = program_name;
            this.line_number = line_number;
            this.raw_line = raw_line;
            this.command = command;
            this.parameters = parameters;
            this.feed_rate = feed_rate;
            this.spindle_speed = spindle_speed;
            this.comment = comment;
            this.is_rapid = is_rapid;
        }

        public static GCodeBlockMsg Deserialize(MessageDeserializer deserializer) => new GCodeBlockMsg(deserializer);

        private GCodeBlockMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.program_name);
            deserializer.Read(out this.line_number);
            deserializer.Read(out this.raw_line);
            deserializer.Read(out this.command);
            deserializer.Read(out this.parameters, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.feed_rate);
            deserializer.Read(out this.spindle_speed);
            deserializer.Read(out this.comment);
            deserializer.Read(out this.is_rapid);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.program_name);
            serializer.Write(this.line_number);
            serializer.Write(this.raw_line);
            serializer.Write(this.command);
            serializer.WriteLength(this.parameters);
            serializer.Write(this.parameters);
            serializer.Write(this.feed_rate);
            serializer.Write(this.spindle_speed);
            serializer.Write(this.comment);
            serializer.Write(this.is_rapid);
        }

        public override string ToString()
        {
            return "GCodeBlockMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nprogram_name: " + program_name.ToString() +
            "\nline_number: " + line_number.ToString() +
            "\nraw_line: " + raw_line.ToString() +
            "\ncommand: " + command.ToString() +
            "\nparameters: " + System.String.Join(", ", parameters.ToList()) +
            "\nfeed_rate: " + feed_rate.ToString() +
            "\nspindle_speed: " + spindle_speed.ToString() +
            "\ncomment: " + comment.ToString() +
            "\nis_rapid: " + is_rapid.ToString();
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
