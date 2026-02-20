using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class TaskAnnouncementMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/TaskAnnouncement";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string auction_id;
        public string task_type;
        public string job_id;
        public string material;
        public double complexity;
        public TimeMsg deadline;
        public string[] required_capabilities;
        public double estimated_duration;
        public string priority;

        public TaskAnnouncementMsg()
        {
            this.timestamp = new TimeMsg();
            this.auction_id = "";
            this.task_type = "";
            this.job_id = "";
            this.material = "";
            this.complexity = 0.0;
            this.deadline = new TimeMsg();
            this.required_capabilities = new string[0];
            this.estimated_duration = 0.0;
            this.priority = "";
        }

        public TaskAnnouncementMsg(
            TimeMsg timestamp,
            string auction_id,
            string task_type,
            string job_id,
            string material,
            double complexity,
            TimeMsg deadline,
            string[] required_capabilities,
            double estimated_duration,
            string priority)
        {
            this.timestamp = timestamp;
            this.auction_id = auction_id;
            this.task_type = task_type;
            this.job_id = job_id;
            this.material = material;
            this.complexity = complexity;
            this.deadline = deadline;
            this.required_capabilities = required_capabilities;
            this.estimated_duration = estimated_duration;
            this.priority = priority;
        }

        public static TaskAnnouncementMsg Deserialize(MessageDeserializer deserializer) => new TaskAnnouncementMsg(deserializer);

        private TaskAnnouncementMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.auction_id);
            deserializer.Read(out this.task_type);
            deserializer.Read(out this.job_id);
            deserializer.Read(out this.material);
            deserializer.Read(out this.complexity);
            this.deadline = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.required_capabilities, deserializer.ReadLength());
            deserializer.Read(out this.estimated_duration);
            deserializer.Read(out this.priority);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.auction_id);
            serializer.Write(this.task_type);
            serializer.Write(this.job_id);
            serializer.Write(this.material);
            serializer.Write(this.complexity);
            serializer.Write(this.deadline);
            serializer.WriteLength(this.required_capabilities);
            serializer.Write(this.required_capabilities);
            serializer.Write(this.estimated_duration);
            serializer.Write(this.priority);
        }

        public override string ToString()
        {
            return "TaskAnnouncementMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nauction_id: " + auction_id.ToString() +
            "\ntask_type: " + task_type.ToString() +
            "\njob_id: " + job_id.ToString() +
            "\nmaterial: " + material.ToString() +
            "\ncomplexity: " + complexity.ToString() +
            "\ndeadline: " + deadline.ToString() +
            "\nrequired_capabilities: " + System.String.Join(", ", required_capabilities.ToList()) +
            "\nestimated_duration: " + estimated_duration.ToString() +
            "\npriority: " + priority.ToString();
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
