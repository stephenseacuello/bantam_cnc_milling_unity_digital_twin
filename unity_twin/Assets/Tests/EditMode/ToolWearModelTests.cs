using NUnit.Framework;
using MiracleTwin.Cutting;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class ToolWearModelTests
    {
        [Test]
        public void Initial_Wear_Is_Break_In_Value()
        {
            var model = new ToolWearModel();
            Assert.AreEqual(0.02f, model.VB, 0.001f);
        }

        [Test]
        public void Wear_Increases_Over_Time()
        {
            var model = new ToolWearModel();
            float initialVB = model.VB;

            // Simulate 5 minutes of cutting
            for (int i = 0; i < 300; i++)
                model.Update(100f, 0.05f, 1.0f, 1.0f);

            Assert.Greater(model.VB, initialVB);
        }

        [Test]
        public void Break_In_Stage_Completes()
        {
            var model = new ToolWearModel();

            // Simulate 3 minutes (past break-in)
            for (int i = 0; i < 180; i++)
                model.Update(100f, 0.05f, 1.0f, 1.0f);

            Assert.GreaterOrEqual(model.CurrentStage, 2);
        }

        [Test]
        public void End_Of_Life_Detected()
        {
            var model = new ToolWearModel();

            // Simulate long cutting at high speed
            for (int i = 0; i < 3600; i++)
                model.Update(200f, 0.05f, 1.0f, 1.0f);

            Assert.IsTrue(model.IsEndOfLife || model.VB > 0.25f);
        }

        [Test]
        public void Reset_Returns_To_Initial()
        {
            var model = new ToolWearModel();

            for (int i = 0; i < 60; i++)
                model.Update(100f, 0.05f, 1.0f, 1.0f);

            model.Reset();
            Assert.AreEqual(0.02f, model.VB, 0.001f);
            Assert.AreEqual(0f, model.CuttingTime, 0.001f);
        }

        [Test]
        public void Higher_Speed_Increases_Wear_Rate()
        {
            var model1 = new ToolWearModel();
            var model2 = new ToolWearModel();

            // Both run 5 minutes
            for (int i = 0; i < 300; i++)
            {
                model1.Update(100f, 0.05f, 1.0f, 1.0f);
                model2.Update(200f, 0.05f, 1.0f, 1.0f);
            }

            Assert.Greater(model2.VB, model1.VB, "Higher speed should cause more wear");
        }
    }
}
