using NUnit.Framework;
using MiracleTwin.Cutting;

namespace MiracleTwin.Tests.EditMode
{
    [TestFixture]
    public class GCodeParserTests
    {
        private GCodeParser parser;

        [SetUp]
        public void Setup()
        {
            parser = new GCodeParser();
        }

        [Test]
        public void Parse_G0_Rapid_Move()
        {
            var commands = parser.Parse("G0 X10 Y20 Z5");
            Assert.AreEqual(1, commands.Count);
            Assert.IsTrue(commands[0].IsRapid);
            Assert.AreEqual(10f, commands[0].GetParam('X'));
            Assert.AreEqual(20f, commands[0].GetParam('Y'));
            Assert.AreEqual(5f, commands[0].GetParam('Z'));
        }

        [Test]
        public void Parse_G1_Linear_Feed()
        {
            var commands = parser.Parse("G1 X50 Y30 F1000");
            Assert.AreEqual(1, commands.Count);
            Assert.IsTrue(commands[0].IsLinearFeed);
            Assert.AreEqual(1000f, commands[0].GetParam('F'));
        }

        [Test]
        public void Parse_G2_CW_Arc()
        {
            var commands = parser.Parse("G2 X10 Y0 I5 J0 F500");
            Assert.AreEqual(1, commands.Count);
            Assert.IsTrue(commands[0].IsCWArc);
            Assert.AreEqual(5f, commands[0].GetParam('I'));
        }

        [Test]
        public void Parse_M3_Spindle_On()
        {
            var commands = parser.Parse("M3 S16000");
            Assert.AreEqual(1, commands.Count);
            Assert.IsTrue(commands[0].IsMCode);
            Assert.AreEqual(3f, commands[0].code);
            Assert.AreEqual(16000f, commands[0].GetParam('S'));
        }

        [Test]
        public void Parse_Skips_Comments()
        {
            var commands = parser.Parse("(This is a comment)\nG0 X10\n; Another comment");
            Assert.AreEqual(1, commands.Count);
            Assert.AreEqual(10f, commands[0].GetParam('X'));
        }

        [Test]
        public void Parse_Negative_Coordinates()
        {
            var commands = parser.Parse("G1 X-5.5 Y-10.2 Z-0.5");
            Assert.AreEqual(1, commands.Count);
            Assert.AreEqual(-5.5f, commands[0].GetParam('X'), 0.01f);
            Assert.AreEqual(-10.2f, commands[0].GetParam('Y'), 0.01f);
            Assert.AreEqual(-0.5f, commands[0].GetParam('Z'), 0.01f);
        }

        [Test]
        public void Parse_Multi_Line_Program()
        {
            string program = "G90 G21\nM3 S16000\nG0 X0 Y0 Z5\nG1 Z-0.5 F500\nM5\nM30";
            var commands = parser.Parse(program);
            Assert.AreEqual(6, commands.Count);
        }

        [Test]
        public void Parse_Empty_Program()
        {
            var commands = parser.Parse("");
            Assert.AreEqual(0, commands.Count);
        }

        [Test]
        public void Validate_Detects_Missing_Spindle()
        {
            var errors = parser.Validate("G1 X10 F500\nM30");
            Assert.IsTrue(errors.Count > 0);
            Assert.IsTrue(errors[0].Contains("spindle"));
        }

        [Test]
        public void Validate_Detects_Missing_Program_End()
        {
            var errors = parser.Validate("M3 S16000\nG1 X10 F500\nM5");
            Assert.IsTrue(errors.Exists(e => e.Contains("M30")));
        }
    }
}
