using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class CorrelatedAlertMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/CorrelatedAlert";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string alert_id;
        public string summary;
        public string[] correlated_anomaly_types;
        public double[] anomaly_severities;
        public string root_cause_hypothesis;
        public double hypothesis_confidence;
        public string[] recommended_actions;
        public double estimated_impact;
        public bool requires_operator_decision;

        public CorrelatedAlertMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.alert_id = "";
            this.summary = "";
            this.correlated_anomaly_types = new string[0];
            this.anomaly_severities = new double[0];
            this.root_cause_hypothesis = "";
            this.hypothesis_confidence = 0.0;
            this.recommended_actions = new string[0];
            this.estimated_impact = 0.0;
            this.requires_operator_decision = false;
        }

        public CorrelatedAlertMsg(
            TimeMsg timestamp,
            string machine_id,
            string alert_id,
            string summary,
            string[] correlated_anomaly_types,
            double[] anomaly_severities,
            string root_cause_hypothesis,
            double hypothesis_confidence,
            string[] recommended_actions,
            double estimated_impact,
            bool requires_operator_decision)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.alert_id = alert_id;
            this.summary = summary;
            this.correlated_anomaly_types = correlated_anomaly_types;
            this.anomaly_severities = anomaly_severities;
            this.root_cause_hypothesis = root_cause_hypothesis;
            this.hypothesis_confidence = hypothesis_confidence;
            this.recommended_actions = recommended_actions;
            this.estimated_impact = estimated_impact;
            this.requires_operator_decision = requires_operator_decision;
        }

        public static CorrelatedAlertMsg Deserialize(MessageDeserializer deserializer) => new CorrelatedAlertMsg(deserializer);

        private CorrelatedAlertMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.alert_id);
            deserializer.Read(out this.summary);
            deserializer.Read(out this.correlated_anomaly_types, deserializer.ReadLength());
            deserializer.Read(out this.anomaly_severities, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.root_cause_hypothesis);
            deserializer.Read(out this.hypothesis_confidence);
            deserializer.Read(out this.recommended_actions, deserializer.ReadLength());
            deserializer.Read(out this.estimated_impact);
            deserializer.Read(out this.requires_operator_decision);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.alert_id);
            serializer.Write(this.summary);
            serializer.WriteLength(this.correlated_anomaly_types);
            serializer.Write(this.correlated_anomaly_types);
            serializer.WriteLength(this.anomaly_severities);
            serializer.Write(this.anomaly_severities);
            serializer.Write(this.root_cause_hypothesis);
            serializer.Write(this.hypothesis_confidence);
            serializer.WriteLength(this.recommended_actions);
            serializer.Write(this.recommended_actions);
            serializer.Write(this.estimated_impact);
            serializer.Write(this.requires_operator_decision);
        }

        public override string ToString()
        {
            return "CorrelatedAlertMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nalert_id: " + alert_id.ToString() +
            "\nsummary: " + summary.ToString() +
            "\ncorrelated_anomaly_types: " + System.String.Join(", ", correlated_anomaly_types.ToList()) +
            "\nanomaly_severities: " + System.String.Join(", ", anomaly_severities.ToList()) +
            "\nroot_cause_hypothesis: " + root_cause_hypothesis.ToString() +
            "\nhypothesis_confidence: " + hypothesis_confidence.ToString() +
            "\nrecommended_actions: " + System.String.Join(", ", recommended_actions.ToList()) +
            "\nestimated_impact: " + estimated_impact.ToString() +
            "\nrequires_operator_decision: " + requires_operator_decision.ToString();
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
