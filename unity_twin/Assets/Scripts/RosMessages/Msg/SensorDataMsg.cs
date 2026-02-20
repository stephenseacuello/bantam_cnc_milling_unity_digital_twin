using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class SensorDataMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/SensorData";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public string sensor_type;
        public string sensor_id;
        public double[] values;
        public string[] labels;
        public double sample_rate;
        public uint sequence_number;
        public double quality;

        public SensorDataMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.sensor_type = "";
            this.sensor_id = "";
            this.values = new double[0];
            this.labels = new string[0];
            this.sample_rate = 0.0;
            this.sequence_number = 0;
            this.quality = 0.0;
        }

        public SensorDataMsg(
            TimeMsg timestamp,
            string machine_id,
            string sensor_type,
            string sensor_id,
            double[] values,
            string[] labels,
            double sample_rate,
            uint sequence_number,
            double quality)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.sensor_type = sensor_type;
            this.sensor_id = sensor_id;
            this.values = values;
            this.labels = labels;
            this.sample_rate = sample_rate;
            this.sequence_number = sequence_number;
            this.quality = quality;
        }

        public static SensorDataMsg Deserialize(MessageDeserializer deserializer) => new SensorDataMsg(deserializer);

        private SensorDataMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.sensor_type);
            deserializer.Read(out this.sensor_id);
            deserializer.Read(out this.values, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.labels, deserializer.ReadLength());
            deserializer.Read(out this.sample_rate);
            deserializer.Read(out this.sequence_number);
            deserializer.Read(out this.quality);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.sensor_type);
            serializer.Write(this.sensor_id);
            serializer.WriteLength(this.values);
            serializer.Write(this.values);
            serializer.WriteLength(this.labels);
            serializer.Write(this.labels);
            serializer.Write(this.sample_rate);
            serializer.Write(this.sequence_number);
            serializer.Write(this.quality);
        }

        public override string ToString()
        {
            return "SensorDataMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nsensor_type: " + sensor_type.ToString() +
            "\nsensor_id: " + sensor_id.ToString() +
            "\nvalues: " + System.String.Join(", ", values.ToList()) +
            "\nlabels: " + System.String.Join(", ", labels.ToList()) +
            "\nsample_rate: " + sample_rate.ToString() +
            "\nsequence_number: " + sequence_number.ToString() +
            "\nquality: " + quality.ToString();
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
