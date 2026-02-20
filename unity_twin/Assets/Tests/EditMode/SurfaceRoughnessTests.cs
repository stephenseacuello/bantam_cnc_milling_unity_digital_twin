using NUnit.Framework;
using MiracleTwin.Cutting;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class SurfaceRoughnessTests
    {
        [Test]
        public void Roughness_Increases_With_Feed()
        {
            float Ra_low = SurfaceRoughnessModel.CalculateRa(0.02f);
            float Ra_high = SurfaceRoughnessModel.CalculateRa(0.10f);
            Assert.Greater(Ra_high, Ra_low);
        }

        [Test]
        public void Standard_Conditions_Produce_Expected_Ra()
        {
            // fz=0.05mm, rε=0.4mm → Ra ≈ fz²/(32·rε) * 1000 ≈ 0.195 µm
            float Ra = SurfaceRoughnessModel.CalculateRa(0.05f, 0.4f);
            Assert.AreEqual(0.2f, Ra, 0.15f);
        }

        [Test]
        public void Vibration_Increases_Roughness()
        {
            float Ra_no_vib = SurfaceRoughnessModel.CalculateRa(0.05f, 0.4f, 0f);
            float Ra_with_vib = SurfaceRoughnessModel.CalculateRa(0.05f, 0.4f, 5f);
            Assert.Greater(Ra_with_vib, Ra_no_vib);
        }

        [Test]
        public void Surface_Grade_Classification()
        {
            Assert.AreEqual("N1 (Mirror)", SurfaceRoughnessModel.GetSurfaceGrade(0.05f));
            Assert.AreEqual("N5 (Milled)", SurfaceRoughnessModel.GetSurfaceGrade(1.2f));
        }
    }
}
