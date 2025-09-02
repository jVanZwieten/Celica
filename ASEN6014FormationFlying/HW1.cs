using Celica;
using System.Numerics;
using TBP = Celica.TwoBodyProblem;
using TBPF = Celica.TwoBPF;
using Units = Celica.PhysicalUnits;
using UnitsF = Celica.PhysicalUnitsF;
using Vector = MathNet.Numerics.LinearAlgebra.Double.Vector;

namespace ASEN6014FormationFlying
{
    internal static class HW1
    {
        const float mu_earth = CelestialParametersF.Earth.Mu;
        const float J2_earth = CelestialParametersF.Earth.J2;
        const float r_earth = CelestialParametersF.Earth.Radius;

        /// <summary>
        /// A satellite is orbiting the Earth with a semi-major axis of 8000 km and an eccentricity of 0.23.  Note that this trajectory will intercept with the Earth.  Determine radius at apoapses, radius at periapses, semi-latus rectum, semi-minor axis and the eccentric anomaly if the true anomaly is 295 degrees.
        /// </summary>
        internal static void Quiz4Problem8()
        {
            float a = 8000 * UnitsF.km;
            float e = 0.23f;
            float nu = 295 * UnitsF.deg;

            var r_a = TBPF.Apoapsis(a, e);
            var r_p = TBPF.Periapsis(a, e);
            var p = TBPF.SemiLatusRectum(a, e);
            var b = TBPF.SemiMinorAxis(a, e);
            var E = TBPF.EccentricAnomaly(nu, e);

            Console.WriteLine($"Radius at Apoapsis: {r_a / UnitsF.km} km");
            Console.WriteLine($"Radius at Periapsis: {r_p / UnitsF.km} km");
            Console.WriteLine($"Semi-latus rectum: {p / UnitsF.km} km");
            Console.WriteLine($"Semi-minor axis: {b / UnitsF.km} km");
            Console.WriteLine($"Eccentric Anomaly: {E / UnitsF.deg} deg");
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
            Vector3 rVec_0 = new(2466.69f * UnitsF.km, 5941.54f * UnitsF.km, 3282.71f * UnitsF.km);
            Vector3 vVec_0 = new(-6.80822f * UnitsF.km, 1.04998f * UnitsF.km, 3.61939f * UnitsF.km);
            var xVec_0 = new StateEpochF(new StateF(rVec_0, vVec_0), 0);

            Func<StateF, float, StateF> dxVecDt = (state, epoch)
                => new StateF(state.Velocity, EquationsOfMotion.GravityAcceleration(state, mu_earth));

            float t_f = 60 * UnitsF.min;
            var trajectory = NumericalMethods.RungeKutta4Integration(dxVecDt, xVec_0, 1f, t_f);

            var xVec_f = trajectory[^1];
            Console.WriteLine($"Final Position Vector: {xVec_f.State.Position / UnitsF.km} km");
            Console.WriteLine($"Final Velocity Vector: {xVec_f.State.Velocity / UnitsF.km} km");
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
            Vector3 rVec_1_0 = new(-6685.20926f * UnitsF.km, 601.51244f * UnitsF.km, 3346.06634f * UnitsF.km);
            Vector3 vVec_1_0 = new(-1.74294f * UnitsF.km, -6.70242f * UnitsF.km, -2.27739f * UnitsF.km);
            Vector3 rVec_2_0 = new(-6685.21657f * UnitsF.km, 592.52839f * UnitsF.km, 3345.6716f * UnitsF.km);
            Vector3 vVec_2_0 = new(-1.74283f * UnitsF.km, -6.70475f * UnitsF.km, -2.27334f * UnitsF.km);

            StateEpochF xVec_1_0 = new(new StateF(rVec_1_0, vVec_1_0), 0);
            StateEpochF xVec_2_0 = new(new StateF(rVec_2_0, vVec_2_0), 0);

            Func<StateF, float, StateF> dxVecDt = (state, epoch)
                => new StateF(state.Velocity, EquationsOfMotion.GravityAcceleration(state.Position, mu_earth) + EquationsOfMotion.J2Acceleration(state.Position, J2_earth, mu_earth, r_earth));

            float t_f = 4848f;
            var initialStates = new StateEpochF[] { xVec_1_0, xVec_2_0 };

            var trajectories = NumericalMethods.RungeKutta4Integration(dxVecDt, initialStates, .1f, t_f);
            var xVec_1_f = trajectories[0][^1];
            var xVec_2_f = trajectories[1][^1];

            Console.WriteLine($"Final Position Vector 1: {xVec_1_f.State.Position / UnitsF.km} km");
            Console.WriteLine($"Final Velocity Vector 1: {xVec_1_f.State.Velocity / UnitsF.km} km/s");
            Console.WriteLine($"Final Position Vector 2: {xVec_2_f.State.Position / UnitsF.km} km");
            Console.WriteLine($"Final Velocity Vector 2: {xVec_2_f.State.Velocity / UnitsF.km} km/s");
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
            var rVec = new Vector3(2466.69f * UnitsF.km, 5941.54f * UnitsF.km, 3282.71f * UnitsF.km);
            var vVec = new Vector3(-6.80822f * UnitsF.km, 1.04998f * UnitsF.km, 3.61939f * UnitsF.km);
            var hVec = TBPF.SpecificAngularMomentumVec(rVec, vVec);

            Console.WriteLine($"Orbital Angular Momentum Vector: {hVec / (UnitsF.km * UnitsF.km)} km^2/s");
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
            var rVec = new Vector3(2466.69f * UnitsF.km, 5941.54f * UnitsF.km, 3282.71f * UnitsF.km);
            var vVec = new Vector3(-6.80822f * UnitsF.km, 1.04998f * UnitsF.km, 3.61939f * UnitsF.km);

            var h = TBPF.SpecificAngularMomentum(rVec, vVec);
            var r = 8000 * UnitsF.km;

            var fDot = h / (r * r); // rad/s
            Console.WriteLine($"True Anomaly Rate: {fDot / UnitsF.deg} deg/s");
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
            var rVec_0 = new Vector3(2466.69f * UnitsF.km, 5941.54f * UnitsF.km, 3282.71f * UnitsF.km);
            var vVec_0 = new Vector3(-6.80822f * UnitsF.km, 1.04998f * UnitsF.km, 3.61939f * UnitsF.km);
            var vVec_t = new Vector3(5.57433f * UnitsF.km, -0.92203f * UnitsF.km, -3.00873f * UnitsF.km);

            var hVec = TBPF.SpecificAngularMomentumVec(rVec_0, vVec_0);
            var eVec = TBPF.Eccentricity(rVec_0, vVec_0, hVec, mu_earth);

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
            var a = 8000f * UnitsF.km;
            var vVec = new Vector3(-6.80822f * UnitsF.km, 1.04998f * UnitsF.km, 3.61939f * UnitsF.km);
            var r = TBPF.OrbitRadius(a, vVec.Length(), mu_earth);

            Console.WriteLine($"Orbit radius r_t: {r / UnitsF.km} km");
        }

        /// <summary>
        /// A satellite is orbiting Earth with a semi-major axis of 7500 km and an eccentricity of 0.05.  The initial true anomaly is
        /// f(t_0) = 25deg
        ///  What is the true anomaly 1 hour later?  Provide your answer in radians, and keep the angle between 0 and 2π.
        /// </summary>
        internal static void Quiz9Problem3()
        {
            var a = 7500 * Units.km;
            var e = .05;
            var ν_0 = 25 * Units.deg;
            var t_f = 1 * Units.hour;

            var M_0 = TBP.MeanFromTrueAnomaly(ν_0, e);
            var M_f = TBP.MeanAnomaly(M_0, t_f, a, mu_earth);

            var ν_f = TBP.TrueFromMeanAnomaly(M_f, e);

            Console.WriteLine($"True Anomaly f(t_f): {ν_f} rads");
        }

        /// <summary>
        /// Write a subroutine to convert the following elements and true anomaly to the equivalent inertial position and velocity vector states.  The method inputs should be
        /// μ,a,e,i,Ω,ω,f
        /// To validate the conversion from orbit elements to cartesian states is working, input below the response for a case where an Earth orbiting satellite has a=8000 km, e = .1, i = 30 degrees, Ω = 145 degrees, ω = 120 degrees, and M(t_0) = 10deg. Find the oe states one hour later and determine the corresponding inertial position and velocity vector.
        /// </summary>
        internal static void Quiz10Problem6()
        {
            var a = 8000d * Units.km;
            var e = .1;
            var i = 30d * Units.deg;
            var Ω = 145d * Units.deg;
            var ω = 120d * Units.deg;
            var M_0 = 10d * Units.deg;
            var deltaT = 1d * Units.hour;

            var M_f = TBP.MeanAnomaly(M_0, deltaT, a, mu_earth);
            var ν_f = TBP.TrueFromMeanAnomaly(M_f, e);

            var state_f = TBP.StateVector(a, e, i, Ω, ω, ν_f, mu_earth);

            Console.WriteLine($"Position Vector at t_f: [{state_f.X / Units.km}, {state_f.Y / Units.km}, {state_f.Z / Units.km}] km");
            Console.WriteLine($"Velocity Vector at t_f: [{state_f.V_x / Units.km}, {state_f.V_y / Units.km}, {state_f.V_z / Units.km}] km");
        }

        /// <summary>
        /// Write a subroutine to convert the inertial position and velocity vector components to the corresponding classical orbit elements.  The subroutine inputs should be
        /// mu, Nr, Nv
        /// To test the subroutine, assume the satellite is orbiting the Earth with the current inertial states
        /// NrVec = (−820.865,−1905.95,−7445.9) km
        /// NvVec = (−6.75764,−1.85916,0.930651) km/sec
        /// </summary>
        internal static void Quiz10Problem7()
        {
            Vector rVec = (Vector)Vector.Build.Dense([-820.865 * Units.km, -1905.95 * Units.km, -7445.9 * Units.km]);
            Vector vVec = (Vector)Vector.Build.Dense([-6.75764 * Units.km, -1.85916 * Units.km, 0.930651 * Units.km]);
            State state = new State(rVec, vVec);

            var elements = TBP.ClassicalElementsFromState(state, mu_earth);

            Console.WriteLine($"Semi-major axis a: {elements.a / Units.km} km");
            Console.WriteLine($"Eccentricity e: {elements.e}");
            Console.WriteLine($"Inclination i: {elements.i} rad");
            Console.WriteLine($"Right Ascension of Ascending Node Ω: {elements.Ω - 2 * Math.PI} rad");
            Console.WriteLine($"Argument of Periapsis ω: {elements.ω} rad");
            Console.WriteLine($"True Anomaly f: {elements.υ} rad");
        }
    }
}
