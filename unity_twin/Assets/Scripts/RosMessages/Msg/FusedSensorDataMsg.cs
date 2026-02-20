using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class FusedSensorDataMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/FusedSensorData";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public double[] imu_features;
        public double[] current_features;
        public double[] audio_features;
        public double[] vision_features;
        public double[] feature_vector;
        public byte sensor_health;
        public double synchronization_quality;

        public FusedSensorDataMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.imu_features = new double[0];
            this.current_features = new double[0];
            this.audio_features = new double[0];
            this.vision_features = new double[0];
            this.feature_vector = new double[0];
            this.sensor_health = 0;
            this.synchronization_quality = 0.0;
        }

        public FusedSensorDataMsg(
            TimeMsg timestamp,
            string machine_id,
            double[] imu_features,
            double[] current_features,
            double[] audio_features,
            double[] vision_features,
            double[] feature_vector,
            byte sensor_health,
            double synchronization_quality)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.imu_features = imu_features;
            this.current_features = current_features;
            this.audio_features = audio_features;
            this.vision_features = vision_features;
            this.feature_vector = feature_vector;
            this.sensor_health = sensor_health;
            this.synchronization_quality = synchronization_quality;
        }

        public static FusedSensorDataMsg Deserialize(MessageDeserializer deserializer) => new FusedSensorDataMsg(deserializer);

        private FusedSensorDataMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.imu_features, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.current_features, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.audio_features, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.vision_features, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.feature_vector, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.sensor_health);
            deserializer.Read(out this.synchronization_quality);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.WriteLength(this.imu_features);
            serializer.Write(this.imu_features);
            serializer.WriteLength(this.current_features);
            serializer.Write(this.current_features);
            serializer.WriteLength(this.audio_features);
            serializer.Write(this.audio_features);
            serializer.WriteLength(this.vision_features);
            serializer.Write(this.vision_features);
            serializer.WriteLength(this.feature_vector);
            serializer.Write(this.feature_vector);
            serializer.Write(this.sensor_health);
            serializer.Write(this.synchronization_quality);
        }

        public override string ToString()
        {
            return "FusedSensorDataMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nimu_features: " + System.String.Join(", ", imu_features.ToList()) +
            "\ncurrent_features: " + System.String.Join(", ", current_features.ToList()) +
            "\naudio_features: " + System.String.Join(", ", audio_features.ToList()) +
            "\nvision_features: " + System.String.Join(", ", vision_features.ToList()) +
            "\nfeature_vector: " + System.String.Join(", ", feature_vector.ToList()) +
            "\nsensor_health: " + sensor_health.ToString() +
            "\nsynchronization_quality: " + synchronization_quality.ToString();
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
