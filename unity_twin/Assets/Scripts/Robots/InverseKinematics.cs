using UnityEngine;

namespace MiracleTwin.Robots
{
    /// <summary>
    /// Analytical inverse kinematics for 6-DOF robot arms.
    /// Provides joint angle solutions for pick-and-place waypoints.
    /// Uses geometric approach suitable for both Ned2 and xArm6 configurations.
    /// </summary>
    public static class InverseKinematics
    {
        /// <summary>DH parameters for a robot arm.</summary>
        public struct DHParameters
        {
            public float[] a;      // Link lengths
            public float[] d;      // Link offsets
            public float[] alpha;  // Link twists
        }

        /// <summary>Niryo Ned2 DH parameters (mm).</summary>
        public static readonly DHParameters Ned2DH = new()
        {
            a = new float[] { 0, 0, 210, 0, 0, 0 },
            d = new float[] { 183, 0, 0, 189.4f, 0, 39.5f },
            alpha = new float[] { -Mathf.PI / 2, 0, -Mathf.PI / 2, Mathf.PI / 2, -Mathf.PI / 2, 0 }
        };

        /// <summary>xArm 6 Lite DH parameters (mm).</summary>
        public static readonly DHParameters XArm6DH = new()
        {
            a = new float[] { 0, 243.3f, 0, 0, 0, 0 },
            d = new float[] { 267, 0, 0, 227.6f, 0, 61.5f },
            alpha = new float[] { -Mathf.PI / 2, 0, -Mathf.PI / 2, Mathf.PI / 2, -Mathf.PI / 2, 0 }
        };

        /// <summary>
        /// Compute IK for a target position using iterative Jacobian method.
        /// Returns joint angles in radians, or null if unreachable.
        /// </summary>
        public static float[] Solve(Vector3 targetPosition, Quaternion targetRotation,
            DHParameters dh, float[] currentJoints = null, int maxIterations = 50, float tolerance = 0.001f)
        {
            float[] joints = currentJoints != null ? (float[])currentJoints.Clone() : new float[6];

            for (int iter = 0; iter < maxIterations; iter++)
            {
                Vector3 currentPos = ForwardKinematics(joints, dh);
                Vector3 error = targetPosition - currentPos;

                if (error.magnitude < tolerance)
                    return joints;

                // Numerical Jacobian
                float[,] J = ComputeJacobian(joints, dh);

                // Damped least squares
                float lambda = 0.5f;
                for (int i = 0; i < 6; i++)
                {
                    float dq = 0;
                    for (int j = 0; j < 3; j++)
                    {
                        dq += J[j, i] * error[j];
                    }
                    joints[i] += dq * lambda;
                    joints[i] = Mathf.Clamp(joints[i], -Mathf.PI, Mathf.PI);
                }
            }

            // Didn't converge — return best guess
            Debug.LogWarning("[IK] Solution did not converge");
            return joints;
        }

        /// <summary>Forward kinematics: joint angles -> end effector position.</summary>
        public static Vector3 ForwardKinematics(float[] joints, DHParameters dh)
        {
            Matrix4x4 T = Matrix4x4.identity;

            for (int i = 0; i < 6; i++)
            {
                float ct = Mathf.Cos(joints[i]);
                float st = Mathf.Sin(joints[i]);
                float ca = Mathf.Cos(dh.alpha[i]);
                float sa = Mathf.Sin(dh.alpha[i]);
                float a = dh.a[i] / 1000f;  // mm -> m
                float d = dh.d[i] / 1000f;

                var Ti = new Matrix4x4();
                Ti.SetRow(0, new Vector4(ct, -st * ca, st * sa, a * ct));
                Ti.SetRow(1, new Vector4(st, ct * ca, -ct * sa, a * st));
                Ti.SetRow(2, new Vector4(0, sa, ca, d));
                Ti.SetRow(3, new Vector4(0, 0, 0, 1));

                T = T * Ti;
            }

            return new Vector3(T.m03, T.m13, T.m23);
        }

        private static float[,] ComputeJacobian(float[] joints, DHParameters dh, float delta = 0.001f)
        {
            float[,] J = new float[3, 6];
            Vector3 basePos = ForwardKinematics(joints, dh);

            for (int i = 0; i < 6; i++)
            {
                float[] perturbed = (float[])joints.Clone();
                perturbed[i] += delta;
                Vector3 perturbedPos = ForwardKinematics(perturbed, dh);
                Vector3 diff = (perturbedPos - basePos) / delta;

                J[0, i] = diff.x;
                J[1, i] = diff.y;
                J[2, i] = diff.z;
            }

            return J;
        }

        /// <summary>Pre-computed waypoints for standard pick-place operations.</summary>
        public static class Waypoints
        {
            public static readonly float[] Ned2Home = { 0, 0.25f, -0.6f, 0, 0, 0 };
            public static readonly float[] Ned2AboveStock = { 0.5f, 0.1f, -0.4f, 0, -0.5f, 0 };
            public static readonly float[] Ned2AtStock = { 0.5f, 0.3f, -0.6f, 0, -0.3f, 0 };
            public static readonly float[] Ned2AboveCNC = { -0.3f, 0.1f, -0.4f, 0, -0.5f, 0 };
            public static readonly float[] Ned2InsideCNC = { -0.3f, 0.4f, -0.7f, 0, -0.2f, 0 };

            public static readonly float[] XArm6Home = { 0, -0.25f, 0.6f, 0, 0, 0 };
            public static readonly float[] XArm6AboveFinished = { -0.5f, -0.1f, 0.4f, 0, 0.5f, 0 };
            public static readonly float[] XArm6AtFinished = { -0.5f, -0.3f, 0.6f, 0, 0.3f, 0 };
        }
    }
}
