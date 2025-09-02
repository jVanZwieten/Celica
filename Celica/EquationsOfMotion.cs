using System.Numerics;

namespace Celica
{
    public static class EquationsOfMotion
    {
        /// <summary>
        /// Computes the acceleration due to a central gravitational body.
        /// </summary>
        /// <param name="mu">Standard gravitational parameter of the central body (m^3/s^2).</param>
        /// <param name="rVec">Position vector of the orbiter at an epoch.</param>
        /// <returns>rDotDotVec, the .</returns>
        public static Vector3 GravityAcceleration(Vector3 rVec, float mu)
        {
            float r = rVec.Length();
            return -mu / (r * r * r) * rVec;
        }
        public static Vector3 GravityAcceleration(State state, float mu) => GravityAcceleration(state.Position, mu);


        public static Vector3 J2Acceleration(Vector3 xVec, float J2, float mu, float r_equitorial)
        {
            var r = xVec.Length();
            var x = xVec.X;
            var y = xVec.Y;
            var z = xVec.Z;

            var factor = -1.5f * J2 * mu * r_equitorial * r_equitorial / (r * r * r * r);

            var a_x = (1 - 5 * (z * z) / (r * r)) * x / r;
            var a_y = (1 - 5 * (z * z) / (r * r)) * y / r;
            var a_z = (3 - 5 * (z * z) / (r * r)) * z / r;

            return factor * new Vector3(a_x, a_y, a_z);
        }
        public static Vector3 J2Acceleration(State state, float J2, float mu, float r_equitorial) => J2Acceleration(state.Position, J2, mu, r_equitorial);
    }
}
