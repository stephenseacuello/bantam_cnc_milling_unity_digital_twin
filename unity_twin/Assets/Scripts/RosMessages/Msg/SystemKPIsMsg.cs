using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class SystemKPIsMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/SystemKPIs";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public double oee;
        public double availability;
        public double performance;
        public double quality;
        public double cpk;
        public double mtbf;
        public double mttr;
        public double energy_efficiency;
        public double schedule_adherence;
        public double tool_life_utilization;
        public uint jobs_completed_today;
        public uint jobs_in_progress;
        public uint jobs_queued;

        public SystemKPIsMsg()
        {
            this.timestamp = new TimeMsg();
            this.oee = 0.0;
            this.availability = 0.0;
            this.performance = 0.0;
            this.quality = 0.0;
            this.cpk = 0.0;
            this.mtbf = 0.0;
            this.mttr = 0.0;
            this.energy_efficiency = 0.0;
            this.schedule_adherence = 0.0;
            this.tool_life_utilization = 0.0;
            this.jobs_completed_today = 0;
            this.jobs_in_progress = 0;
            this.jobs_queued = 0;
        }

        public SystemKPIsMsg(
            TimeMsg timestamp,
            double oee,
            double availability,
            double performance,
            double quality,
            double cpk,
            double mtbf,
            double mttr,
            double energy_efficiency,
            double schedule_adherence,
            double tool_life_utilization,
            uint jobs_completed_today,
            uint jobs_in_progress,
            uint jobs_queued)
        {
            this.timestamp = timestamp;
            this.oee = oee;
            this.availability = availability;
            this.performance = performance;
            this.quality = quality;
            this.cpk = cpk;
            this.mtbf = mtbf;
            this.mttr = mttr;
            this.energy_efficiency = energy_efficiency;
            this.schedule_adherence = schedule_adherence;
            this.tool_life_utilization = tool_life_utilization;
            this.jobs_completed_today = jobs_completed_today;
            this.jobs_in_progress = jobs_in_progress;
            this.jobs_queued = jobs_queued;
        }

        public static SystemKPIsMsg Deserialize(MessageDeserializer deserializer) => new SystemKPIsMsg(deserializer);

        private SystemKPIsMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.oee);
            deserializer.Read(out this.availability);
            deserializer.Read(out this.performance);
            deserializer.Read(out this.quality);
            deserializer.Read(out this.cpk);
            deserializer.Read(out this.mtbf);
            deserializer.Read(out this.mttr);
            deserializer.Read(out this.energy_efficiency);
            deserializer.Read(out this.schedule_adherence);
            deserializer.Read(out this.tool_life_utilization);
            deserializer.Read(out this.jobs_completed_today);
            deserializer.Read(out this.jobs_in_progress);
            deserializer.Read(out this.jobs_queued);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.oee);
            serializer.Write(this.availability);
            serializer.Write(this.performance);
            serializer.Write(this.quality);
            serializer.Write(this.cpk);
            serializer.Write(this.mtbf);
            serializer.Write(this.mttr);
            serializer.Write(this.energy_efficiency);
            serializer.Write(this.schedule_adherence);
            serializer.Write(this.tool_life_utilization);
            serializer.Write(this.jobs_completed_today);
            serializer.Write(this.jobs_in_progress);
            serializer.Write(this.jobs_queued);
        }

        public override string ToString()
        {
            return "SystemKPIsMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\noee: " + oee.ToString() +
            "\navailability: " + availability.ToString() +
            "\nperformance: " + performance.ToString() +
            "\nquality: " + quality.ToString() +
            "\ncpk: " + cpk.ToString() +
            "\nmtbf: " + mtbf.ToString() +
            "\nmttr: " + mttr.ToString() +
            "\nenergy_efficiency: " + energy_efficiency.ToString() +
            "\nschedule_adherence: " + schedule_adherence.ToString() +
            "\ntool_life_utilization: " + tool_life_utilization.ToString() +
            "\njobs_completed_today: " + jobs_completed_today.ToString() +
            "\njobs_in_progress: " + jobs_in_progress.ToString() +
            "\njobs_queued: " + jobs_queued.ToString();
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
