using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class JobStatusMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/JobStatus";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string job_id;
        public string machine_id;
        public string status;
        public string program_name;
        public uint total_lines;
        public uint current_line;
        public double progress;
        public double estimated_remaining_sec;
        public double elapsed_sec;
        public string[] warnings;
        public string[] errors;

        public JobStatusMsg()
        {
            this.timestamp = new TimeMsg();
            this.job_id = "";
            this.machine_id = "";
            this.status = "";
            this.program_name = "";
            this.total_lines = 0;
            this.current_line = 0;
            this.progress = 0.0;
            this.estimated_remaining_sec = 0.0;
            this.elapsed_sec = 0.0;
            this.warnings = new string[0];
            this.errors = new string[0];
        }

        public JobStatusMsg(
            TimeMsg timestamp,
            string job_id,
            string machine_id,
            string status,
            string program_name,
            uint total_lines,
            uint current_line,
            double progress,
            double estimated_remaining_sec,
            double elapsed_sec,
            string[] warnings,
            string[] errors)
        {
            this.timestamp = timestamp;
            this.job_id = job_id;
            this.machine_id = machine_id;
            this.status = status;
            this.program_name = program_name;
            this.total_lines = total_lines;
            this.current_line = current_line;
            this.progress = progress;
            this.estimated_remaining_sec = estimated_remaining_sec;
            this.elapsed_sec = elapsed_sec;
            this.warnings = warnings;
            this.errors = errors;
        }

        public static JobStatusMsg Deserialize(MessageDeserializer deserializer) => new JobStatusMsg(deserializer);

        private JobStatusMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.job_id);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.status);
            deserializer.Read(out this.program_name);
            deserializer.Read(out this.total_lines);
            deserializer.Read(out this.current_line);
            deserializer.Read(out this.progress);
            deserializer.Read(out this.estimated_remaining_sec);
            deserializer.Read(out this.elapsed_sec);
            deserializer.Read(out this.warnings, deserializer.ReadLength());
            deserializer.Read(out this.errors, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.job_id);
            serializer.Write(this.machine_id);
            serializer.Write(this.status);
            serializer.Write(this.program_name);
            serializer.Write(this.total_lines);
            serializer.Write(this.current_line);
            serializer.Write(this.progress);
            serializer.Write(this.estimated_remaining_sec);
            serializer.Write(this.elapsed_sec);
            serializer.WriteLength(this.warnings);
            serializer.Write(this.warnings);
            serializer.WriteLength(this.errors);
            serializer.Write(this.errors);
        }

        public override string ToString()
        {
            return "JobStatusMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\njob_id: " + job_id.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nstatus: " + status.ToString() +
            "\nprogram_name: " + program_name.ToString() +
            "\ntotal_lines: " + total_lines.ToString() +
            "\ncurrent_line: " + current_line.ToString() +
            "\nprogress: " + progress.ToString() +
            "\nestimated_remaining_sec: " + estimated_remaining_sec.ToString() +
            "\nelapsed_sec: " + elapsed_sec.ToString() +
            "\nwarnings: " + System.String.Join(", ", warnings.ToList()) +
            "\nerrors: " + System.String.Join(", ", errors.ToList());
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
