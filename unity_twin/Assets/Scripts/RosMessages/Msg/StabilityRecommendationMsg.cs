using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class StabilityRecommendationMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/StabilityRecommendation";
        public override string RosMessageName => k_RosMessageName;

        public string machine_id;
        public float current_rpm;
        public float recommended_rpm;
        public float current_depth;
        public float max_stable_depth;
        public string risk_level;
        public float stability_margin;
        public string recommendation;

        public StabilityRecommendationMsg()
        {
            this.machine_id = "";
            this.current_rpm = 0f;
            this.recommended_rpm = 0f;
            this.current_depth = 0f;
            this.max_stable_depth = 0f;
            this.risk_level = "LOW";
            this.stability_margin = 1f;
            this.recommendation = "";
        }

        public StabilityRecommendationMsg(
            string machine_id,
            float current_rpm,
            float recommended_rpm,
            float current_depth,
            float max_stable_depth,
            string risk_level,
            float stability_margin,
            string recommendation)
        {
            this.machine_id = machine_id;
            this.current_rpm = current_rpm;
            this.recommended_rpm = recommended_rpm;
            this.current_depth = current_depth;
            this.max_stable_depth = max_stable_depth;
            this.risk_level = risk_level;
            this.stability_margin = stability_margin;
            this.recommendation = recommendation;
        }

        public static StabilityRecommendationMsg Deserialize(MessageDeserializer deserializer) =>
            new StabilityRecommendationMsg(deserializer);

        private StabilityRecommendationMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.current_rpm);
            deserializer.Read(out this.recommended_rpm);
            deserializer.Read(out this.current_depth);
            deserializer.Read(out this.max_stable_depth);
            deserializer.Read(out this.risk_level);
            deserializer.Read(out this.stability_margin);
            deserializer.Read(out this.recommendation);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.machine_id);
            serializer.Write(this.current_rpm);
            serializer.Write(this.recommended_rpm);
            serializer.Write(this.current_depth);
            serializer.Write(this.max_stable_depth);
            serializer.Write(this.risk_level);
            serializer.Write(this.stability_margin);
            serializer.Write(this.recommendation);
        }

        public override string ToString()
        {
            return "StabilityRecommendationMsg: " +
                "\nmachine_id: " + machine_id +
                "\ncurrent_rpm: " + current_rpm +
                "\nrecommended_rpm: " + recommended_rpm +
                "\ncurrent_depth: " + current_depth +
                "\nmax_stable_depth: " + max_stable_depth +
                "\nrisk_level: " + risk_level +
                "\nstability_margin: " + stability_margin +
                "\nrecommendation: " + recommendation;
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
