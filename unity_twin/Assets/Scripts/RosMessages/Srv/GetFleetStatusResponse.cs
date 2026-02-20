using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class GetFleetStatusResponse : Message
    {
        public const string k_RosMessageName = "miracle_msgs/GetFleetStatus_Response";
        public override string RosMessageName => k_RosMessageName;

        public FleetHealthMsg fleet_health;
        public string[] node_details_json;

        public GetFleetStatusResponse()
        {
            fleet_health = new FleetHealthMsg();
            node_details_json = new string[0];
        }

        public GetFleetStatusResponse(
            FleetHealthMsg fleet_health,
            string[] node_details_json)
        {
            this.fleet_health = fleet_health;
            this.node_details_json = node_details_json;
        }

        private GetFleetStatusResponse(MessageDeserializer deserializer)
        {
            this.fleet_health = FleetHealthMsg.Deserialize(deserializer);
            deserializer.Read(out this.node_details_json, deserializer.ReadLength());
        }

        public static GetFleetStatusResponse Deserialize(MessageDeserializer deserializer) => new GetFleetStatusResponse(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.fleet_health);
            serializer.WriteLength(this.node_details_json);
            serializer.Write(this.node_details_json);
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
