using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class ToolWearEstimateMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/ToolWearEstimate";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string tool_id;
        public double wear_percentage;
        public double remaining_life_minutes;
        public double confidence;
        public string wear_type;
        public double flank_wear_mm;
        public double crater_wear_mm;
        public string recommended_action;

        public ToolWearEstimateMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.tool_id = "";
            this.wear_percentage = 0.0;
            this.remaining_life_minutes = 0.0;
            this.confidence = 0.0;
            this.wear_type = "";
            this.flank_wear_mm = 0.0;
            this.crater_wear_mm = 0.0;
            this.recommended_action = "";
        }

        public ToolWearEstimateMsg(
            TimeMsg timestamp,
            string machine_id,
            string tool_id,
            double wear_percentage,
            double remaining_life_minutes,
            double confidence,
            string wear_type,
            double flank_wear_mm,
            double crater_wear_mm,
            string recommended_action)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.tool_id = tool_id;
            this.wear_percentage = wear_percentage;
            this.remaining_life_minutes = remaining_life_minutes;
            this.confidence = confidence;
            this.wear_type = wear_type;
            this.flank_wear_mm = flank_wear_mm;
            this.crater_wear_mm = crater_wear_mm;
            this.recommended_action = recommended_action;
        }

        public static ToolWearEstimateMsg Deserialize(MessageDeserializer deserializer) => new ToolWearEstimateMsg(deserializer);

        private ToolWearEstimateMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.tool_id);
            deserializer.Read(out this.wear_percentage);
            deserializer.Read(out this.remaining_life_minutes);
            deserializer.Read(out this.confidence);
            deserializer.Read(out this.wear_type);
            deserializer.Read(out this.flank_wear_mm);
            deserializer.Read(out this.crater_wear_mm);
            deserializer.Read(out this.recommended_action);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.tool_id);
            serializer.Write(this.wear_percentage);
            serializer.Write(this.remaining_life_minutes);
            serializer.Write(this.confidence);
            serializer.Write(this.wear_type);
            serializer.Write(this.flank_wear_mm);
            serializer.Write(this.crater_wear_mm);
            serializer.Write(this.recommended_action);
        }

        public override string ToString()
        {
            return "ToolWearEstimateMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\ntool_id: " + tool_id.ToString() +
            "\nwear_percentage: " + wear_percentage.ToString() +
            "\nremaining_life_minutes: " + remaining_life_minutes.ToString() +
            "\nconfidence: " + confidence.ToString() +
            "\nwear_type: " + wear_type.ToString() +
            "\nflank_wear_mm: " + flank_wear_mm.ToString() +
            "\ncrater_wear_mm: " + crater_wear_mm.ToString() +
            "\nrecommended_action: " + recommended_action.ToString();
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
