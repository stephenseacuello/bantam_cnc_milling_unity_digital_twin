using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class ValidateGCodeRequest : Message
    {
        public const string k_RosMessageName = "miracle_msgs/ValidateGCode_Request";
        public override string RosMessageName => k_RosMessageName;

        public string program_content;
        public string machine_id;

        public ValidateGCodeRequest()
        {
            program_content = "";
            machine_id = "";
        }

        public ValidateGCodeRequest(
            string program_content,
            string machine_id)
        {
            this.program_content = program_content;
            this.machine_id = machine_id;
        }

        private ValidateGCodeRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.program_content);
            deserializer.Read(out this.machine_id);
        }

        public static ValidateGCodeRequest Deserialize(MessageDeserializer deserializer) => new ValidateGCodeRequest(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.program_content);
            serializer.Write(this.machine_id);
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
