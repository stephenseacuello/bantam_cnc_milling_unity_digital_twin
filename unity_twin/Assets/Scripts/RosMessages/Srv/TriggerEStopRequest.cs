using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class TriggerEStopRequest : Message
    {
        public const string k_RosMessageName = "miracle_msgs/TriggerEStop_Request";
        public override string RosMessageName => k_RosMessageName;

        public string machine_id;
        public string reason;
        public string requesting_node;

        public TriggerEStopRequest()
        {
            machine_id = "";
            reason = "";
            requesting_node = "";
        }

        public TriggerEStopRequest(
            string machine_id,
            string reason,
            string requesting_node)
        {
            this.machine_id = machine_id;
            this.reason = reason;
            this.requesting_node = requesting_node;
        }

        private TriggerEStopRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.reason);
            deserializer.Read(out this.requesting_node);
        }

        public static TriggerEStopRequest Deserialize(MessageDeserializer deserializer) => new TriggerEStopRequest(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.machine_id);
            serializer.Write(this.reason);
            serializer.Write(this.requesting_node);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
