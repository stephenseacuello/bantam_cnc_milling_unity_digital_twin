using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class FeedOverrideMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/FeedOverride";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public double feed_override_pct;
        public double spindle_override_pct;
        public string reason;
        public double confidence;
        public double revert_after_sec;

        public FeedOverrideMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.feed_override_pct = 100.0;
            this.spindle_override_pct = 100.0;
            this.reason = "";
            this.confidence = 0.0;
            this.revert_after_sec = 0.0;
        }

        public FeedOverrideMsg(
            TimeMsg timestamp,
            string machine_id,
            double feed_override_pct,
            double spindle_override_pct,
            string reason,
            double confidence,
            double revert_after_sec)
        {
            this.timestamp = timestamp;
            this.machine_id = machine_id;
            this.feed_override_pct = feed_override_pct;
            this.spindle_override_pct = spindle_override_pct;
            this.reason = reason;
            this.confidence = confidence;
            this.revert_after_sec = revert_after_sec;
        }

        public static FeedOverrideMsg Deserialize(MessageDeserializer deserializer) => new FeedOverrideMsg(deserializer);

        private FeedOverrideMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.feed_override_pct);
            deserializer.Read(out this.spindle_override_pct);
            deserializer.Read(out this.reason);
            deserializer.Read(out this.confidence);
            deserializer.Read(out this.revert_after_sec);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.feed_override_pct);
            serializer.Write(this.spindle_override_pct);
            serializer.Write(this.reason);
            serializer.Write(this.confidence);
            serializer.Write(this.revert_after_sec);
        }

        public override string ToString()
        {
            return "FeedOverrideMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id +
            "\nfeed_override_pct: " + feed_override_pct +
            "\nspindle_override_pct: " + spindle_override_pct +
            "\nreason: " + reason +
            "\nconfidence: " + confidence +
            "\nrevert_after_sec: " + revert_after_sec;
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
