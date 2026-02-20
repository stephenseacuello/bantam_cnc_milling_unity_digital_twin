using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class AnomalyAlertMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/AnomalyAlert";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string anomaly_type;
        public double confidence;
        public double severity;
        public string[] contributing_factors;
        public double[] feature_contributions;
        public string recommended_action;
        public bool requires_immediate_stop;

        public AnomalyAlertMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.anomaly_type = "";
            this.confidence = 0.0;
            this.severity = 0.0;
            this.contributing_factors = new string[0];
            this.feature_contributions = new double[0];
            this.recommended_action = "";
            this.requires_immediate_stop = false;
        }

        public AnomalyAlertMsg(
            TimeMsg timestamp,
            string machine_id,
            string anomaly_type,
            double confidence,
            double severity,
            string[] contributing_factors,
            double[] feature_contributions,
            string recommended_action,
            bool requires_immediate_stop)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.anomaly_type = anomaly_type;
            this.confidence = confidence;
            this.severity = severity;
            this.contributing_factors = contributing_factors;
            this.feature_contributions = feature_contributions;
            this.recommended_action = recommended_action;
            this.requires_immediate_stop = requires_immediate_stop;
        }

        public static AnomalyAlertMsg Deserialize(MessageDeserializer deserializer) => new AnomalyAlertMsg(deserializer);

        private AnomalyAlertMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.anomaly_type);
            deserializer.Read(out this.confidence);
            deserializer.Read(out this.severity);
            deserializer.Read(out this.contributing_factors, deserializer.ReadLength());
            deserializer.Read(out this.feature_contributions, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.recommended_action);
            deserializer.Read(out this.requires_immediate_stop);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.anomaly_type);
            serializer.Write(this.confidence);
            serializer.Write(this.severity);
            serializer.WriteLength(this.contributing_factors);
            serializer.Write(this.contributing_factors);
            serializer.WriteLength(this.feature_contributions);
            serializer.Write(this.feature_contributions);
            serializer.Write(this.recommended_action);
            serializer.Write(this.requires_immediate_stop);
        }

        public override string ToString()
        {
            return "AnomalyAlertMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nanomaly_type: " + anomaly_type.ToString() +
            "\nconfidence: " + confidence.ToString() +
            "\nseverity: " + severity.ToString() +
            "\ncontributing_factors: " + System.String.Join(", ", contributing_factors.ToList()) +
            "\nfeature_contributions: " + System.String.Join(", ", feature_contributions.ToList()) +
            "\nrecommended_action: " + recommended_action.ToString() +
            "\nrequires_immediate_stop: " + requires_immediate_stop.ToString();
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
