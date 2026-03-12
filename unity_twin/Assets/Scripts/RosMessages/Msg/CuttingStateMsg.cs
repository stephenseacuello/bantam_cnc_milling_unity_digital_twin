using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Miracle
{
    [Serializable]
    public class CuttingStateMsg : Message
    {
        public const string k_RosMessageName = "miracle_msgs/CuttingState";
        public override string RosMessageName => k_RosMessageName;

        public TimeMsg timestamp;
        public string machine_id;
        public double spindle_rpm;
        public double feed_rate;
        public double[] tool_position;
        public double[] tool_direction;
        public double axial_depth;
        public double radial_depth;
        public double tool_diameter;
        public bool is_cutting;
        public uint current_gcode_line;
        public double force_fx;
        public double force_fy;
        public double force_fz;
        public double power_watts;
        public double torque_nm;
        public double mrr;
        public double tool_temperature;
        public double interface_temperature;
        public double flank_wear_vb;
        public double wear_percentage;
        public double chip_thickness_ratio;
        public double shear_angle_deg;
        public double chip_curl_radius;
        public double chip_velocity;
        public double surface_roughness_ra;
        public double surface_roughness_rz;
        public string surface_grade;

        public CuttingStateMsg()
        {
            this.timestamp = new TimeMsg();
            this.machine_id = "";
            this.spindle_rpm = 0.0;
            this.feed_rate = 0.0;
            this.tool_position = new double[3];
            this.tool_direction = new double[3];
            this.axial_depth = 0.0;
            this.radial_depth = 0.0;
            this.tool_diameter = 0.0;
            this.is_cutting = false;
            this.current_gcode_line = 0;
            this.force_fx = 0.0;
            this.force_fy = 0.0;
            this.force_fz = 0.0;
            this.power_watts = 0.0;
            this.torque_nm = 0.0;
            this.mrr = 0.0;
            this.tool_temperature = 0.0;
            this.interface_temperature = 0.0;
            this.flank_wear_vb = 0.0;
            this.wear_percentage = 0.0;
            this.chip_thickness_ratio = 0.0;
            this.shear_angle_deg = 0.0;
            this.chip_curl_radius = 0.0;
            this.chip_velocity = 0.0;
            this.surface_roughness_ra = 0.0;
            this.surface_roughness_rz = 0.0;
            this.surface_grade = "";
        }

        public static CuttingStateMsg Deserialize(MessageDeserializer deserializer) => new CuttingStateMsg(deserializer);

        private CuttingStateMsg(MessageDeserializer deserializer)
        {
            this.timestamp = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.machine_id);
            deserializer.Read(out this.spindle_rpm);
            deserializer.Read(out this.feed_rate);
            deserializer.Read(out this.tool_position, sizeof(double), 3);
            deserializer.Read(out this.tool_direction, sizeof(double), 3);
            deserializer.Read(out this.axial_depth);
            deserializer.Read(out this.radial_depth);
            deserializer.Read(out this.tool_diameter);
            deserializer.Read(out this.is_cutting);
            deserializer.Read(out this.current_gcode_line);
            deserializer.Read(out this.force_fx);
            deserializer.Read(out this.force_fy);
            deserializer.Read(out this.force_fz);
            deserializer.Read(out this.power_watts);
            deserializer.Read(out this.torque_nm);
            deserializer.Read(out this.mrr);
            deserializer.Read(out this.tool_temperature);
            deserializer.Read(out this.interface_temperature);
            deserializer.Read(out this.flank_wear_vb);
            deserializer.Read(out this.wear_percentage);
            deserializer.Read(out this.chip_thickness_ratio);
            deserializer.Read(out this.shear_angle_deg);
            deserializer.Read(out this.chip_curl_radius);
            deserializer.Read(out this.chip_velocity);
            deserializer.Read(out this.surface_roughness_ra);
            deserializer.Read(out this.surface_roughness_rz);
            deserializer.Read(out this.surface_grade);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.timestamp);
            serializer.Write(this.machine_id);
            serializer.Write(this.spindle_rpm);
            serializer.Write(this.feed_rate);
            serializer.Write(this.tool_position);
            serializer.Write(this.tool_direction);
            serializer.Write(this.axial_depth);
            serializer.Write(this.radial_depth);
            serializer.Write(this.tool_diameter);
            serializer.Write(this.is_cutting);
            serializer.Write(this.current_gcode_line);
            serializer.Write(this.force_fx);
            serializer.Write(this.force_fy);
            serializer.Write(this.force_fz);
            serializer.Write(this.power_watts);
            serializer.Write(this.torque_nm);
            serializer.Write(this.mrr);
            serializer.Write(this.tool_temperature);
            serializer.Write(this.interface_temperature);
            serializer.Write(this.flank_wear_vb);
            serializer.Write(this.wear_percentage);
            serializer.Write(this.chip_thickness_ratio);
            serializer.Write(this.shear_angle_deg);
            serializer.Write(this.chip_curl_radius);
            serializer.Write(this.chip_velocity);
            serializer.Write(this.surface_roughness_ra);
            serializer.Write(this.surface_roughness_rz);
            serializer.Write(this.surface_grade);
        }

        public override string ToString()
        {
            return "CuttingStateMsg: " +
            "\ntimestamp: " + timestamp.ToString() +
            "\nmachine_id: " + machine_id.ToString() +
            "\nspindle_rpm: " + spindle_rpm.ToString() +
            "\nfeed_rate: " + feed_rate.ToString() +
            "\nis_cutting: " + is_cutting.ToString() +
            "\nforce_fx: " + force_fx.ToString() +
            "\nforce_fy: " + force_fy.ToString() +
            "\nforce_fz: " + force_fz.ToString() +
            "\npower_watts: " + power_watts.ToString() +
            "\nwear_percentage: " + wear_percentage.ToString();
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
