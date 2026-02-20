using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class HeartbeatMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/Heartbeat";
        public override string RosMessageName => k_RosMessageName;

        public HeaderMsg header;
        public TimeMsg timestamp;
        public string node_name;
        public string node_namespace;
        public string criticality;
        public string lifecycle_state;
        public string[] dependencies;
        public float cpu_usage;
        public float memory_usage;

        public HeartbeatMsg()
        {
            this.header = new HeaderMsg();
            this.timestamp = new TimeMsg();
            this.node_name = "";
            this.node_namespace = "";
            this.criticality = "";
            this.lifecycle_state = "";
            this.dependencies = new string[0];
            this.cpu_usage = 0.0f;
            this.memory_usage = 0.0f;
        }

        public HeartbeatMsg(
            HeaderMsg header,
            TimeMsg timestamp,
            string node_name,
            string node_namespace,
            string criticality,
            string lifecycle_state,
            string[] dependencies,
            float cpu_usage,
            float memory_usage)
        {
            this.header = header;
            this.timestamp = timestamp;
            this.node_name = node_name;
            this.node_namespace = node_namespace;
            this.criticality = criticality;
            this.lifecycle_state = lifecycle_state;
            this.dependencies = dependencies;
            this.cpu_usage = cpu_usage;
            this.memory_usage = memory_usage;
        }

        public static HeartbeatMsg Deserialize(MessageDeserializer deserializer) => new HeartbeatMsg(deserializer);

        private HeartbeatMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.node_name);
            deserializer.Read(out this.node_namespace);
            deserializer.Read(out this.criticality);
            deserializer.Read(out this.lifecycle_state);
            deserializer.Read(out this.dependencies, deserializer.ReadLength());
            deserializer.Read(out this.cpu_usage);
            deserializer.Read(out this.memory_usage);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.timestamp);
            serializer.Write(this.node_name);
            serializer.Write(this.node_namespace);
            serializer.Write(this.criticality);
            serializer.Write(this.lifecycle_state);
            serializer.WriteLength(this.dependencies);
            serializer.Write(this.dependencies);
            serializer.Write(this.cpu_usage);
            serializer.Write(this.memory_usage);
        }

        public override string ToString()
        {
            return "HeartbeatMsg: " +
            "\nheader: " + header.ToString() +
            "\ntimestamp: " + timestamp.ToString() +
            "\nnode_name: " + node_name.ToString() +
            "\nnode_namespace: " + node_namespace.ToString() +
            "\ncriticality: " + criticality.ToString() +
            "\nlifecycle_state: " + lifecycle_state.ToString() +
            "\ndependencies: " + System.String.Join(", ", dependencies.ToList()) +
            "\ncpu_usage: " + cpu_usage.ToString() +
            "\nmemory_usage: " + memory_usage.ToString();
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
