using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class RunPredictionResponse : Message
    {
        public const string k_RosMessageName = "miracle_msgs/RunPrediction_Response";
        public override string RosMessageName => k_RosMessageName;

        public bool success;
        public string summary;
        public double confidence;
        public string detailed_report_json;

        public RunPredictionResponse()
        {
            success = false;
            summary = "";
            confidence = 0.0;
            detailed_report_json = "";
        }

        public RunPredictionResponse(
            bool success,
            string summary,
            double confidence,
            string detailed_report_json)
        {
            this.success = success;
            this.summary = summary;
            this.confidence = confidence;
            this.detailed_report_json = detailed_report_json;
        }

        private RunPredictionResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
            deserializer.Read(out this.summary);
            deserializer.Read(out this.confidence);
            deserializer.Read(out this.detailed_report_json);
        }

        public static RunPredictionResponse Deserialize(MessageDeserializer deserializer) => new RunPredictionResponse(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
            serializer.Write(this.summary);
            serializer.Write(this.confidence);
            serializer.Write(this.detailed_report_json);
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
