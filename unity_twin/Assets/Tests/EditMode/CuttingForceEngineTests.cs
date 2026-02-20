using NUnit.Framework;
using MiracleTwin.Cutting;
using Unity.Collections;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class CuttingForceEngineTests
    {
        [Test]
        public void Force_Calculation_Produces_Nonzero_Results()
        {
            var output = new NativeArray<float>(12, Allocator.Temp);
            var job = new CuttingForceJob
            {
                spindleRPM = 16000f,
                feedPerTooth = 0.05f,
                axialDepth = 0.5f,
                radialDepth = 3.175f,
                toolDiameter = 6.35f,
                fluteCount = 2,
                helixAngle = 0.5236f,  // 30 degrees
                Ktc = 796f, Krc = 168f, Kac = 80f,
                Kte = 14.5f, Kre = 10.2f, Kae = 4.8f,
                flankWearVB = 0f,
                wearForceMultiplier = 12.5f,
                output = output
            };

            job.Execute();

            // Peak forces should be positive
            Assert.Greater(output[0], 0f, "Fx_peak should be > 0");
            Assert.Greater(output[1], 0f, "Fy_peak should be > 0");
            Assert.Greater(output[2], 0f, "Fz_peak should be > 0");

            // Power should be positive
            Assert.Greater(output[6], 0f, "Power should be > 0");

            // MRR should be positive
            Assert.Greater(output[8], 0f, "MRR should be > 0");

            output.Dispose();
        }

        [Test]
        public void Force_Increases_With_Depth_Of_Cut()
        {
            var output1 = new NativeArray<float>(12, Allocator.Temp);
            var output2 = new NativeArray<float>(12, Allocator.Temp);

            var baseJob = new CuttingForceJob
            {
                spindleRPM = 16000f,
                feedPerTooth = 0.05f,
                radialDepth = 3.175f,
                toolDiameter = 6.35f,
                fluteCount = 2,
                helixAngle = 0.5236f,
                Ktc = 796f, Krc = 168f, Kac = 80f,
                Kte = 14.5f, Kre = 10.2f, Kae = 4.8f,
                flankWearVB = 0f,
                wearForceMultiplier = 12.5f,
            };

            var job1 = baseJob;
            job1.axialDepth = 0.5f;
            job1.output = output1;
            job1.Execute();

            var job2 = baseJob;
            job2.axialDepth = 2.0f;
            job2.output = output2;
            job2.Execute();

            Assert.Greater(output2[0], output1[0], "Deeper cut should produce higher Fx_peak");
            Assert.Greater(output2[6], output1[6], "Deeper cut should produce higher power");

            output1.Dispose();
            output2.Dispose();
        }

        [Test]
        public void Wear_Increases_Forces()
        {
            var output1 = new NativeArray<float>(12, Allocator.Temp);
            var output2 = new NativeArray<float>(12, Allocator.Temp);

            var baseJob = new CuttingForceJob
            {
                spindleRPM = 16000f,
                feedPerTooth = 0.05f,
                axialDepth = 0.5f,
                radialDepth = 3.175f,
                toolDiameter = 6.35f,
                fluteCount = 2,
                helixAngle = 0.5236f,
                Ktc = 796f, Krc = 168f, Kac = 80f,
                Kte = 14.5f, Kre = 10.2f, Kae = 4.8f,
                wearForceMultiplier = 12.5f,
            };

            var job1 = baseJob;
            job1.flankWearVB = 0f;
            job1.output = output1;
            job1.Execute();

            var job2 = baseJob;
            job2.flankWearVB = 0.2f;
            job2.output = output2;
            job2.Execute();

            Assert.Greater(output2[0], output1[0], "Worn tool should produce higher forces");

            output1.Dispose();
            output2.Dispose();
        }

        [Test]
        public void Zero_RPM_Returns_Zero_Forces()
        {
            var output = new NativeArray<float>(12, Allocator.Temp);
            var job = new CuttingForceJob
            {
                spindleRPM = 0f,
                feedPerTooth = 0.05f,
                axialDepth = 0.5f,
                radialDepth = 3.175f,
                toolDiameter = 6.35f,
                fluteCount = 2,
                helixAngle = 0.5236f,
                Ktc = 796f, Krc = 168f, Kac = 80f,
                Kte = 14.5f, Kre = 10.2f, Kae = 4.8f,
                flankWearVB = 0f,
                wearForceMultiplier = 12.5f,
                output = output
            };

            job.Execute();

            Assert.AreEqual(0f, output[0], "Should be zero at 0 RPM");
            Assert.AreEqual(0f, output[6], "Power should be zero at 0 RPM");

            output.Dispose();
        }
    }
}
