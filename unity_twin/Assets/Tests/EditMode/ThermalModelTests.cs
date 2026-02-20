using NUnit.Framework;
using MiracleTwin.Cutting;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class ThermalModelTests
    {
        [Test]
        public void Initial_Temperature_Is_Ambient()
        {
            var model = new ThermalModel();
            Assert.AreEqual(20f, model.ToolTemperature, 0.1f);
            Assert.AreEqual(20f, model.InterfaceTemperature, 0.1f);
        }

        [Test]
        public void Temperature_Increases_During_Cutting()
        {
            var model = new ThermalModel();
            model.Update(150f, 0.05f, 1.0f, 100f, 1.0f);

            Assert.Greater(model.ToolTemperature, 20f);
            Assert.Greater(model.InterfaceTemperature, 20f);
        }

        [Test]
        public void Temperature_Decreases_During_Cooldown()
        {
            var model = new ThermalModel();
            // Heat up
            for (int i = 0; i < 60; i++)
                model.Update(150f, 0.05f, 1.0f, 100f, 1.0f);

            float peakTemp = model.ToolTemperature;

            // Cool down
            for (int i = 0; i < 60; i++)
                model.Cooldown(1.0f);

            Assert.Less(model.ToolTemperature, peakTemp);
        }

        [Test]
        public void Interface_Temp_Follows_Stephenson_Agapiou()
        {
            var model = new ThermalModel();
            // T_interface = 20 + 8.5 * V^0.5 * f^0.35 * ap^0.15
            // V=150, f=0.05, ap=1.0
            // Expected: 20 + 8.5 * 12.25 * 0.261 * 1.0 = 20 + 27.2 ≈ 47
            model.Update(150f, 0.05f, 1.0f, 100f, 0.01f);

            Assert.AreEqual(47f, model.InterfaceTemperature, 10f);
        }

        [Test]
        public void Reset_Returns_To_Ambient()
        {
            var model = new ThermalModel();
            model.Update(150f, 0.05f, 1.0f, 100f, 10.0f);
            model.Reset();

            Assert.AreEqual(20f, model.ToolTemperature, 0.1f);
            Assert.AreEqual(20f, model.InterfaceTemperature, 0.1f);
        }
    }
}
