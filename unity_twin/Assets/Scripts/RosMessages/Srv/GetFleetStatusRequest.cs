using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class GetFleetStatusRequest : Message
    {
        public const string k_RosMessageName = "miracle_msgs/GetFleetStatus_Request";
        public override string RosMessageName => k_RosMessageName;

        public string filter_criticality;
        public string filter_state;

        public GetFleetStatusRequest()
        {
            filter_criticality = "";
            filter_state = "";
        }

        public GetFleetStatusRequest(
            string filter_criticality,
            string filter_state)
        {
            this.filter_criticality = filter_criticality;
            this.filter_state = filter_state;
        }

        private GetFleetStatusRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.filter_criticality);
            deserializer.Read(out this.filter_state);
        }

        public static GetFleetStatusRequest Deserialize(MessageDeserializer deserializer) => new GetFleetStatusRequest(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.filter_criticality);
            serializer.Write(this.filter_state);
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
