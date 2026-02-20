using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class TaskAwardMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/TaskAward";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string auction_id;
        public string task_type;
        public string awarded_agent_id;
        public string job_id;
        public double agreed_cost;
        public double agreed_completion_time;

        public TaskAwardMsg()
        {
            this.timestamp = new TimeMsg();
            this.auction_id = "";
            this.task_type = "";
            this.awarded_agent_id = "";
            this.job_id = "";
            this.agreed_cost = 0.0;
            this.agreed_completion_time = 0.0;
        }

        public TaskAwardMsg(
            TimeMsg timestamp,
            string auction_id,
            string task_type,
            string awarded_agent_id,
            string job_id,
            double agreed_cost,
            double agreed_completion_time)
        {
            this.timestamp = timestamp;
            this.auction_id = auction_id;
            this.task_type = task_type;
            this.awarded_agent_id = awarded_agent_id;
            this.job_id = job_id;
            this.agreed_cost = agreed_cost;
            this.agreed_completion_time = agreed_completion_time;
        }

        public static TaskAwardMsg Deserialize(MessageDeserializer deserializer) => new TaskAwardMsg(deserializer);

        private TaskAwardMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.auction_id);
            deserializer.Read(out this.task_type);
            deserializer.Read(out this.awarded_agent_id);
            deserializer.Read(out this.job_id);
            deserializer.Read(out this.agreed_cost);
            deserializer.Read(out this.agreed_completion_time);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.auction_id);
            serializer.Write(this.task_type);
            serializer.Write(this.awarded_agent_id);
            serializer.Write(this.job_id);
            serializer.Write(this.agreed_cost);
            serializer.Write(this.agreed_completion_time);
        }

        public override string ToString()
        {
            return "TaskAwardMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nauction_id: " + auction_id.ToString() +
            "\ntask_type: " + task_type.ToString() +
            "\nawarded_agent_id: " + awarded_agent_id.ToString() +
            "\njob_id: " + job_id.ToString() +
            "\nagreed_cost: " + agreed_cost.ToString() +
            "\nagreed_completion_time: " + agreed_completion_time.ToString();
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
