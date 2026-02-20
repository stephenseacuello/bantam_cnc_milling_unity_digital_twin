using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class RobotJointStateMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/RobotJointState";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string robot_id;
        public double[] positions;
        public double[] velocities;
        public double[] efforts;
        public string gripper_state;
        public string task_state;

        public RobotJointStateMsg()
        {
            this.timestamp = new TimeMsg();
            this.robot_id = "";
            this.positions = new double[6];
            this.velocities = new double[6];
            this.efforts = new double[6];
            this.gripper_state = "OPEN";
            this.task_state = "IDLE";
        }

        public RobotJointStateMsg(
            TimeMsg timestamp,
            string robot_id,
            double[] positions,
            double[] velocities,
            double[] efforts,
            string gripper_state,
            string task_state)
        {
            this.timestamp = timestamp;
            this.robot_id = robot_id;
            this.positions = positions;
            this.velocities = velocities;
            this.efforts = efforts;
            this.gripper_state = gripper_state;
            this.task_state = task_state;
        }

        public static RobotJointStateMsg Deserialize(MessageDeserializer deserializer) => new RobotJointStateMsg(deserializer);

        private RobotJointStateMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.robot_id);
            deserializer.Read(out this.positions, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.velocities, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.efforts, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out this.gripper_state);
            deserializer.Read(out this.task_state);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.robot_id);
            serializer.WriteLength(this.positions);
            serializer.Write(this.positions);
            serializer.WriteLength(this.velocities);
            serializer.Write(this.velocities);
            serializer.WriteLength(this.efforts);
            serializer.Write(this.efforts);
            serializer.Write(this.gripper_state);
            serializer.Write(this.task_state);
        }

        public override string ToString()
        {
            return "RobotJointStateMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nrobot_id: " + robot_id.ToString() +
            "\npositions: " + System.String.Join(", ", positions.ToList()) +
            "\nvelocities: " + System.String.Join(", ", velocities.ToList()) +
            "\nefforts: " + System.String.Join(", ", efforts.ToList()) +
            "\ngripper_state: " + gripper_state.ToString() +
            "\ntask_state: " + task_state.ToString();
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
