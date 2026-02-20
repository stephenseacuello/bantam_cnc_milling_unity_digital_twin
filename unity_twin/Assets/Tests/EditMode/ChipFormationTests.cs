using NUnit.Framework;
using MiracleTwin.Cutting;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class ChipFormationTests
    {
        [Test]
        public void Aluminum_Shear_Angle_Is_Reasonable()
        {
            var chip = ChipFormationModel.Calculate(150f, 0.05f, 6.35f);
            // Merchant: φ = 45 - β/2 + α/2 ≈ 29° for Al/HSS
            Assert.AreEqual(29f, chip.shearAngle, 5f);
        }

        [Test]
        public void Aluminum_Is_Continuous_At_Normal_Speeds()
        {
            var chip = ChipFormationModel.Calculate(150f, 0.05f, 6.35f);
            Assert.IsTrue(chip.isContinuous);
        }

        [Test]
        public void Chip_Curl_Radius_Is_Physical()
        {
            var chip = ChipFormationModel.Calculate(150f, 0.05f, 6.35f);
            // Expected ~4.8mm for Al
            Assert.Greater(chip.chipCurlRadius, 1f);
            Assert.Less(chip.chipCurlRadius, 20f);
        }

        [Test]
        public void Chip_Velocity_Is_Positive()
        {
            var chip = ChipFormationModel.Calculate(150f, 0.05f, 6.35f);
            Assert.Greater(chip.chipVelocity, 0f);
        }
    }
}
