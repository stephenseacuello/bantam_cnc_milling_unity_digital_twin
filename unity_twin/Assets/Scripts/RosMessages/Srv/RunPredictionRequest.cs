using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class RunPredictionRequest : Message
    {
        public const string k_RosMessageName = "miracle_msgs/RunPrediction_Request";
        public override string RosMessageName => k_RosMessageName;

        public string machine_id;
        public string scenario_type;
        public double prediction_horizon_hours;

        public RunPredictionRequest()
        {
            machine_id = "";
            scenario_type = "";
            prediction_horizon_hours = 0.0;
        }

        public RunPredictionRequest(
            string machine_id,
            string scenario_type,
            double prediction_horizon_hours)
        {
            this.machine_id = machine_id;
            this.scenario_type = scenario_type;
            this.prediction_horizon_hours = prediction_horizon_hours;
        }

        private RunPredictionRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.scenario_type);
            deserializer.Read(out this.prediction_horizon_hours);
        }

        public static RunPredictionRequest Deserialize(MessageDeserializer deserializer) => new RunPredictionRequest(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.machine_id);
            serializer.Write(this.scenario_type);
            serializer.Write(this.prediction_horizon_hours);
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
