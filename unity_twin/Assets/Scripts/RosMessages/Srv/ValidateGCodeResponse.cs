using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class ValidateGCodeResponse : Message
    {
        public const string k_RosMessageName = "miracle_msgs/ValidateGCode_Response";
        public override string RosMessageName => k_RosMessageName;

        public bool is_valid;
        public string[] errors;
        public string[] warnings;
        public double estimated_duration_sec;

        public ValidateGCodeResponse()
        {
            is_valid = false;
            errors = new string[0];
            warnings = new string[0];
            estimated_duration_sec = 0.0;
        }

        public ValidateGCodeResponse(
            bool is_valid,
            string[] errors,
            string[] warnings,
            double estimated_duration_sec)
        {
            this.is_valid = is_valid;
            this.errors = errors;
            this.warnings = warnings;
            this.estimated_duration_sec = estimated_duration_sec;
        }

        private ValidateGCodeResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.is_valid);
            deserializer.Read(out this.errors, deserializer.ReadLength());
            deserializer.Read(out this.warnings, deserializer.ReadLength());
            deserializer.Read(out this.estimated_duration_sec);
        }

        public static ValidateGCodeResponse Deserialize(MessageDeserializer deserializer) => new ValidateGCodeResponse(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.is_valid);
            serializer.WriteLength(this.errors);
            serializer.Write(this.errors);
            serializer.WriteLength(this.warnings);
            serializer.Write(this.warnings);
            serializer.Write(this.estimated_duration_sec);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
