using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class PHMPredictionMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/PHMPrediction";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string component;
        public string prediction_type;
        public double remaining_useful_life_hours;
        public double confidence;
        public double health_index;
        public string recommended_action;
        public TimeMsg predicted_failure_time;
        public double[] trend_data;

        public PHMPredictionMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.component = "";
            this.prediction_type = "";
            this.remaining_useful_life_hours = 0.0;
            this.confidence = 0.0;
            this.health_index = 0.0;
            this.recommended_action = "";
            this.predicted_failure_time = new TimeMsg();
            this.trend_data = new double[0];
        }

        public PHMPredictionMsg(
            TimeMsg timestamp,
            string machine_id,
            string component,
            string prediction_type,
            double remaining_useful_life_hours,
            double confidence,
            double health_index,
            string recommended_action,
            TimeMsg predicted_failure_time,
            double[] trend_data)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.component = component;
            this.prediction_type = prediction_type;
            this.remaining_useful_life_hours = remaining_useful_life_hours;
            this.confidence = confidence;
            this.health_index = health_index;
            this.recommended_action = recommended_action;
            this.predicted_failure_time = predicted_failure_time;
            this.trend_data = trend_data;
        }

        public static PHMPredictionMsg Deserialize(MessageDeserializer deserializer) => new PHMPredictionMsg(deserializer);

        private PHMPredictionMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.component);
            deserializer.Read(out this.prediction_type);
            deserializer.Read(out this.remaining_useful_life_hours);
            deserializer.Read(out this.confidence);
            deserializer.Read(out this.health_index);
            deserializer.Read(out this.recommended_action);
            this.predicted_failure_time = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.trend_data, sizeof(double), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.component);
            serializer.Write(this.prediction_type);
            serializer.Write(this.remaining_useful_life_hours);
            serializer.Write(this.confidence);
            serializer.Write(this.health_index);
            serializer.Write(this.recommended_action);
            serializer.Write(this.predicted_failure_time);
            serializer.WriteLength(this.trend_data);
            serializer.Write(this.trend_data);
        }

        public override string ToString()
        {
            return "PHMPredictionMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\ncomponent: " + component.ToString() +
            "\nprediction_type: " + prediction_type.ToString() +
            "\nremaining_useful_life_hours: " + remaining_useful_life_hours.ToString() +
            "\nconfidence: " + confidence.ToString() +
            "\nhealth_index: " + health_index.ToString() +
            "\nrecommended_action: " + recommended_action.ToString() +
            "\npredicted_failure_time: " + predicted_failure_time.ToString() +
            "\ntrend_data: " + System.String.Join(", ", trend_data.ToList());
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
