using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class TriggerEStopResponse : Message
    {
        public const string k_RosMessageName = "miracle_msgs/TriggerEStop_Response";
        public override string RosMessageName => k_RosMessageName;

        public bool success;
        public string message;

        public TriggerEStopResponse()
        {
            success = false;
            message = "";
        }

        public TriggerEStopResponse(
            bool success,
            string message)
        {
            this.success = success;
            this.message = message;
        }

        private TriggerEStopResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
            deserializer.Read(out this.message);
        }

        public static TriggerEStopResponse Deserialize(MessageDeserializer deserializer) => new TriggerEStopResponse(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
            serializer.Write(this.message);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
