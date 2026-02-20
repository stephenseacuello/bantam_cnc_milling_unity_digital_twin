using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class SecurityAlertMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/SecurityAlert";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string alert_id;
        public string severity;
        public string category;
        public string source_node;
        public string description;
        public string[] affected_nodes;
        public string recommended_action;
        public bool requires_isolation;
        public double confidence;

        public SecurityAlertMsg()
        {
            this.timestamp = new TimeMsg();
            this.alert_id = "";
            this.severity = "";
            this.category = "";
            this.source_node = "";
            this.description = "";
            this.affected_nodes = new string[0];
            this.recommended_action = "";
            this.requires_isolation = false;
            this.confidence = 0.0;
        }

        public SecurityAlertMsg(
            TimeMsg timestamp,
            string alert_id,
            string severity,
            string category,
            string source_node,
            string description,
            string[] affected_nodes,
            string recommended_action,
            bool requires_isolation,
            double confidence)
        {
            this.timestamp = timestamp;
            this.alert_id = alert_id;
            this.severity = severity;
            this.category = category;
            this.source_node = source_node;
            this.description = description;
            this.affected_nodes = affected_nodes;
            this.recommended_action = recommended_action;
            this.requires_isolation = requires_isolation;
            this.confidence = confidence;
        }

        public static SecurityAlertMsg Deserialize(MessageDeserializer deserializer) => new SecurityAlertMsg(deserializer);

        private SecurityAlertMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.alert_id);
            deserializer.Read(out this.severity);
            deserializer.Read(out this.category);
            deserializer.Read(out this.source_node);
            deserializer.Read(out this.description);
            deserializer.Read(out this.affected_nodes, deserializer.ReadLength());
            deserializer.Read(out this.recommended_action);
            deserializer.Read(out this.requires_isolation);
            deserializer.Read(out this.confidence);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.alert_id);
            serializer.Write(this.severity);
            serializer.Write(this.category);
            serializer.Write(this.source_node);
            serializer.Write(this.description);
            serializer.WriteLength(this.affected_nodes);
            serializer.Write(this.affected_nodes);
            serializer.Write(this.recommended_action);
            serializer.Write(this.requires_isolation);
            serializer.Write(this.confidence);
        }

        public override string ToString()
        {
            return "SecurityAlertMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nalert_id: " + alert_id.ToString() +
            "\nseverity: " + severity.ToString() +
            "\ncategory: " + category.ToString() +
            "\nsource_node: " + source_node.ToString() +
            "\ndescription: " + description.ToString() +
            "\naffected_nodes: " + System.String.Join(", ", affected_nodes.ToList()) +
            "\nrecommended_action: " + recommended_action.ToString() +
            "\nrequires_isolation: " + requires_isolation.ToString() +
            "\nconfidence: " + confidence.ToString();
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
