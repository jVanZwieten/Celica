using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;

namespace Celica
{
    public static class TwoBPF
    {
        public static float SpecificEnergy(float r, float v, float mu) => v * v / 2 - mu / r;
        public static float SpecificEnergy(float a, float mu) => -mu / (2 * a);

        public static float SpecificAngularMomentum(float p, float mu) => MathF.Sqrt(mu * p);
        public static float SpecificAngularMomentum(float a, float e, float mu) => MathF.Sqrt(mu * a * (1 - e * e));
        public static Vector3 SpecificAngularMomentum(Vector3 r, Vector3 v) => Vector3.Cross(r, v);
        public static float SpecificAngularMomentumFPA(float r, float v, float phi = 0) => r * v * MathF.Cos(phi);

        public static float SemiMajorAxis(float r_a, float r_p) => (r_a + r_p) / 2;

        public static float Eccentricity(float r_a, float r_p) => (r_a - r_p) / (r_a + r_p);
        public static float Eccentricity(float energy, float h, float mu) => MathF.Sqrt(1 + 2 * energy * h * h / (mu * mu));
        public static Vector3 Eccentricity(Vector3 rVec, Vector3 vVec, float mu)
        {
            var r = rVec.Length();
            var v = vVec.Length();

            return ((v * v - mu / r) * rVec - Vector3.Dot(rVec, vVec) * vVec) / mu;
        }
        public static Vector3 Eccentricity(Vector3 rVec, Vector3 vVec, Vector3 hVec, float mu)
        {
            var r = rVec.Length();
            var v = vVec.Length();

            return Vector3.Cross(vVec, hVec) / mu - rVec / r;
        }

        public static float SemiLatusRectum(float a, float e) => a * (1 - e * e);
        public static float SemiLatusRectumAM(float h, float mu) => h * h / mu;

        public static float ConicRadius(float p, float e, float nu) => p / (1 + e * MathF.Cos(nu));
        public static float ConicRadius(float h, float e, float nu, float mu) => h * h / (mu * (1 + e * MathF.Cos(nu)));

        public static float Apoapsis(float a, float e) => a * (1 + e);
        public static float ApoapsisSLA(float p, float e) => p / (1 - e);
        public static float Periapsis(float a, float e) => a * (1 - e);
        public static float PeriapsisSLA(float p, float e) => p / (1 + e);

        /// <summary>
        /// Calculate the velocity in the Perifocal Frame (planar) from ground station observations.
        /// </summary>
        /// <param name="r">range</param>
        /// <param name="rDot">range rate</param>
        /// <param name="thetaDot">angular rate</param>
        /// <returns>velocity in the Perifocal Frame [rHat, thetaHat]</returns>
        public static Vector2 VelocityPF(float r, float rDot, float thetaDot) => new(rDot, r * thetaDot);
        public static float Velocity_rHat(float h, float e, float nu, float mu) => mu / h * e * MathF.Sin(nu);
        public static float Velocity_thetaHat(float h, float e, float nu, float mu) => mu / h * (1 + e * MathF.Cos(nu));

        public static float FlightPathAngle(float e, float nu) => MathF.Atan2(e * MathF.Sin(nu), 1 + e * MathF.Cos(nu));

        public static float CircularVelocity(float r, float mu) => MathF.Sqrt(mu / r);
        public static float EscapeVelocity(float r, float mu) => MathF.Sqrt(2 * mu / r);
        public static float V_Infinity(float e) => MathF.Acos(-1 / e);
        public static float V_Infinity(float a, float mu) => MathF.Sqrt(mu / -a);
        public static float Deflection(float e) => 2 * MathF.Asin(1 / e);

        public static float Period(float a, float mu) => 2 * MathF.PI * MathF.Sqrt(a * a * a / mu);
        public static float SemiMajorAxisFromPeriod(float T, float mu) => MathF.Pow(mu * MathF.Pow(T / (2 * MathF.PI), 2), 1f / 3f);
    }
}
