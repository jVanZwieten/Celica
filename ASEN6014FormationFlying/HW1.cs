using System.CodeDom;
using System.Numerics;

using Celica;
using TBP = Celica.TwoBPF;
using Units = Celica.PhysicalUnitsF;

namespace ASEN6014FormationFlying
{
    internal static class HW1
    {
        const float mu_earth = CelestialParameters.Earth.Mu;
        const float J2_earth = CelestialParameters.Earth.J2;
        const float r_earth = CelestialParameters.Earth.Radius;

        /// <summary>
        /// A satellite is orbiting the Earth with a semi-major axis of 8000 km and an eccentricity of 0.23.  Note that this trajectory will intercept with the Earth.  Determine radius at apoapses, radius at periapses, semi-latus rectum, semi-minor axis and the eccentric anomaly if the true anomaly is 295 degrees.
        /// </summary>
        internal static void Quiz4Problem8()
        {
            float a = 8000 * Units.km;
            float e = 0.23f;
            float nu = 295 * Units.deg;

            var r_a = TBP.Apoapsis(a, e);
            var r_p = TBP.Periapsis(a, e);
            var p = TBP.SemiLatusRectum(a, e);
            var b = TBP.SemiMinorAxis(a, e);
            var E = TBP.EccentricAnomaly(nu, e);

            Console.WriteLine($"Radius at Apoapsis: {r_a / Units.km} km");
            Console.WriteLine($"Radius at Periapsis: {r_p / Units.km} km");
            Console.WriteLine($"Semi-latus rectum: {p / Units.km} km");
            Console.WriteLine($"Semi-minor axis: {b / Units.km} km");
            Console.WriteLine($"Eccentric Anomaly: {E / Units.deg} deg");
        }

        /// <summary>
        /// A satellite is flying about the Earth on an elliptic orbit.  The initial inertial position vector is
        /// rVec(t_0) = (2466.69,5941.54,3282.71)km
        /// while the initial inertial velocity is 
        /// vVec(t_0) = (−6.80822,1.04998,3.61939)km/s
        /// Determine the inertial position and velocity vectors 60 minutes later by writing a numerical simulation to integrate the unperturbed orbital differential equations of motion.
        /// </summary>
        internal static void Quiz5Problem3()
        {
            Vector3 rVec_0 = new(2466.69f * Units.km, 5941.54f * Units.km, 3282.71f * Units.km);
            Vector3 vVec_0 = new(-6.80822f * Units.km, 1.04998f * Units.km, 3.61939f * Units.km);
            var xVec_0 = new StateEpoch(new State(rVec_0, vVec_0), 0);

            Func<State, float, State> dxVecDt = (state, epoch)
                => new State(state.Velocity, EquationsOfMotion.GravityAcceleration(state, mu_earth));

            float t_f = 60 * Units.min;
            var trajectory = NumericalMethods.RungeKutta4Integration(dxVecDt, xVec_0, 1f, t_f);

            var xVec_f = trajectory[^1];
            Console.WriteLine($"Final Position Vector: {xVec_f.State.Position / Units.km} km");
            Console.WriteLine($"Final Velocity Vector: {xVec_f.State.Velocity / Units.km} km");
        }

        /// <summary>
        /// Write a numerical simulation of the J_2 perturbed inertial orbital motion
        /// rDotDotVec_i = -mu/r_i^3 + aVec_J2
        /// The disturbance acceleration aVec_J2 is the graviational J_2 disturbance given by
        /// aVec_J2 = -3/2*J_2*mu/r^2*(r_eq/r)^2*[(1-5*(z/r)^2)*x/r; (1-5*(z/r)^2)*y/r; (3-5*(z/r)^2)*z/y]
        /// and J_2 = 1082.63e-6, r_eq = 6378.14 km, and mu = 398600 km^3/s^2
        /// Make the code generic enough to integrate the motion of 2 satellites simultaneously.  Demonstrate that your code is working using the following inertial initial condition:
        /// rVec_1(t_0) = (−6685.20926,601.51244,3346.06634) km
        /// rDotVec_1(t_0) = (−1.74294,−6.70242,−2.27739) km/sec
        /// rVec_2(t_0) = (−6685.21657,592.52839,3345.6716) km
        /// rDotVec_2(t_0) = (−1.74283,−6.70475,−2.27334) km/sec
        /// Integrate the differential equations of motion forward for 4848 seconds and submit the resulting states for verification. 
        /// </summary>
        internal static void Quiz5Problem4()
        {
            Vector3 rVec_1_0 = new(-6685.20926f * Units.km, 601.51244f * Units.km, 3346.06634f * Units.km);
            Vector3 vVec_1_0 = new(-1.74294f * Units.km, -6.70242f * Units.km, -2.27739f * Units.km);
            Vector3 rVec_2_0 = new(-6685.21657f * Units.km, 592.52839f * Units.km, 3345.6716f * Units.km);
            Vector3 vVec_2_0 = new(-1.74283f * Units.km, -6.70475f * Units.km, -2.27334f * Units.km);

            StateEpoch xVec_1_0 = new(new State(rVec_1_0, vVec_1_0), 0);
            StateEpoch xVec_2_0 = new(new State(rVec_2_0, vVec_2_0), 0);

            Func<State, float, State> dxVecDt = (state, epoch)
                => new State(state.Velocity, EquationsOfMotion.GravityAcceleration(state.Position, mu_earth) + EquationsOfMotion.J2Acceleration(state.Position, J2_earth, mu_earth, r_earth));

            float t_f = 4848f;
            var initialStates = new StateEpoch[] { xVec_1_0, xVec_2_0 };

            var trajectories = NumericalMethods.RungeKutta4Integration(dxVecDt, initialStates, .1f, t_f);
            var xVec_1_f = trajectories[0][^1];
            var xVec_2_f = trajectories[1][^1];

            Console.WriteLine($"Final Position Vector 1: {xVec_1_f.State.Position / Units.km} km");
            Console.WriteLine($"Final Velocity Vector 1: {xVec_1_f.State.Velocity / Units.km} km/s");
            Console.WriteLine($"Final Position Vector 2: {xVec_2_f.State.Position / Units.km} km");
            Console.WriteLine($"Final Velocity Vector 2: {xVec_2_f.State.Velocity / Units.km} km/s");
        }

        /// <summary>
        /// Assume the satellite inertial position vector is 
        /// rVec = (2466.69,5941.54,3282.71)km
        /// and the inertial velocity vector is
        /// vVec = (−6.80822,1.04998,3.61939)km/sec
        /// Determine the corresponding orbital angular momentum vector hVec.
        /// </summary>
        internal static void Quiz6Problem2()
        {
            var rVec = new Vector3(2466.69f * Units.km, 5941.54f * Units.km, 3282.71f * Units.km);
            var vVec = new Vector3(-6.80822f * Units.km, 1.04998f * Units.km, 3.61939f * Units.km);
            var hVec = TBP.SpecificAngularMomentumVec(rVec, vVec);

            Console.WriteLine($"Orbital Angular Momentum Vector: {hVec / (Units.km * Units.km)} km^2/s");
        }

        /// <summary>
        /// Assume the satellite inertial position vector is 
        /// rVec = (2466.69,5941.54,3282.71)km
        /// and the inertial velocity vector is
        /// vVec = (−6.80822,1.04998,3.61939)km/sec
        /// What is the true anomaly rate fDot when the radius is 8000 km, expressed in degrees per second?
        /// </summary>
        internal static void Quiz6Problem3()
        {
            var rVec = new Vector3(2466.69f * Units.km, 5941.54f * Units.km, 3282.71f * Units.km);
            var vVec = new Vector3(-6.80822f * Units.km, 1.04998f * Units.km, 3.61939f * Units.km);

            var h = TBP.SpecificAngularMomentum(rVec, vVec);
            var r = 8000 * Units.km;

            var fDot = h / (r * r); // rad/s
            Console.WriteLine($"True Anomaly Rate: {fDot / Units.deg} deg/s");
        }

        /// <summary>
        /// Assume the initial satellite inertial position vector is
        /// rVec(t_0) = (2466.69,5941.54,3282.71)km
        /// and the inertial velocity vector is
        /// vVec(t_0) = (−6.80822,1.04998,3.61939)km/sec
        /// If at time t the velocity vector is
        /// vVec(t) = (5.57433,−0.92203,−3.00873)km/sec
        /// what is the unit vector rHat(t)  pointing from the Earth to the satellite at time t?
        /// </summary>
        internal static void Quiz7Problem2()
        {
            var rVec_0 = new Vector3(2466.69f * Units.km, 5941.54f * Units.km, 3282.71f * Units.km);
            var vVec_0 = new Vector3(-6.80822f * Units.km, 1.04998f * Units.km, 3.61939f * Units.km);
            var vVec_t = new Vector3(5.57433f * Units.km, -0.92203f * Units.km, -3.00873f * Units.km);

            var hVec = TBP.SpecificAngularMomentumVec(rVec_0, vVec_0);
            var eVec = TBP.Eccentricity(rVec_0, vVec_0, hVec, mu_earth);

            var rHat_t = (Vector3.Cross(vVec_t, hVec) / mu_earth - eVec);

            Console.WriteLine($"Unit Vector rHat(t): {rHat_t}");
            Console.WriteLine($"Check: ||rHat_t|| = {rHat_t.Length()} (should equal close to 1).");
        }

        /// <summary>
        /// A satellite is orbiting the Earth with a semi-major axis of a = 8000 km. If the current velocity vector is
        /// vVec = (−6.80822,1.04998,3.61939)km/sec
        /// What is the orbit radius at this time?
        /// </summary>
        internal static void Quiz8Problem2()
        {
            var a = 8000f * Units.km;
            var vVec = new Vector3(-6.80822f * Units.km, 1.04998f * Units.km, 3.61939f * Units.km);
            var r = TBP.OrbitRadius(a, vVec.Length(), mu_earth);

            Console.WriteLine($"Orbit radius r_t: {r / Units.km} km");
        }

        /// <summary>
        /// A satellite is orbiting Earth with a semi-major axis of 7500 km and an eccentricity of 0.05.  The initial true anomaly is
        /// f(t_0) = 25deg
        ///  What is the true anomaly 1 hour later?  Provide your answer in radians, and keep the angle between 0 and 2π.
        /// </summary>
        internal static void Quiz9Problem3()
        {
            var a = 7500f * Units.km;
            var e = .05f;
            var nu_0 = 25f * Units.deg;
            var t_f = 1 * Units.hour;

            var n = TBP.MeanOrbit(a, mu_earth);

            var E_0 = TBP.EccentricAnomaly(nu_0, e);
            var M_0 = TBP.MeanAnomaly(E_0, e);

            var M_f = TBP.MeanAnomalyAfterTime(M_0, t_f, n);
            var E_f = TBP.EccentricFromMeanAnomaly(M_f, e);
            var nu_f = TBP.TrueAnomaly(E_f, e);

            Console.WriteLine($"True Anomaly f(t_f): {nu_f / Units.deg} deg");
        }
    }
}
