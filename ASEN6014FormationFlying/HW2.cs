using Celica;
using MathNet.Numerics.LinearAlgebra.Double;
using Units = Celica.PhysicalUnits;

namespace ASEN6014FormationFlying
{

    internal static class HW2
    {
        const double mu_Earth = CelestialParameters.Earth.Mu;
        const double r_eq = CelestialParameters.Earth.Radius;
        const double J2_Earth = CelestialParameters.Earth.J2;

        internal static void Mod3Quiz3Prob1()
        {
            Vector NrVec_c = DenseVector.OfArray([-4893.268, 3864.478, 3169.646]) * Units.km;
            Vector NvVec_c = DenseVector.OfArray([-3.91386, -6.257673, 1.59797]) * Units.km;

            Vector NrVec_d = DenseVector.OfArray([-4892.98, 3863.073, 3170.619]) * Units.km;
            Vector NvVec_d = DenseVector.OfArray([-3.913302, -6.258661, 1.598199]) * Units.km;

            State state_chief = new(NrVec_c, NvVec_c);
            State state_deputy = new(NrVec_d, NvVec_d);

            State relState_deputy = RelativeMotion.OrbitRelativeState(state_chief, state_deputy);
            Console.WriteLine($"Relative State Deputy (km, km/s):");
            Console.WriteLine(relState_deputy / Units.km);
        }

        internal static void Mod3Quiz3Prob2()
        {
            Vector NrVec_c = DenseVector.OfArray([-4893.268, 3864.478, 3169.646]) * Units.km;
            Vector NvVec_c = DenseVector.OfArray([-3.91386, -6.257673, 1.59797]) * Units.km;

            Vector OrhoVec_d = DenseVector.OfArray([-0.537, 1.221, 1.106]) * Units.km;
            Vector OrhoDotVec_d = DenseVector.OfArray([0.000486, 0.001158, 0.0005590]) * Units.km;

            State state_chief = new(NrVec_c, NvVec_c);
            State relState_deputy = new(OrhoVec_d, OrhoDotVec_d);

            State Nstate_deputy = RelativeMotion.InertialState(state_chief, relState_deputy);
            Console.WriteLine($"Inertial State Deputy (km, km/s):");
            Console.WriteLine(Nstate_deputy / Units.km);
        }

        internal static void Mod3Quiz3Prob3()
        {
            var x = 10 * Units.km;
            var y = 500 * Units.km;
            var r = 7000 * Units.km;
            var xDot = .1 * Units.km;
            var yDot = -.1 * Units.km;
            var rDot = .05 * Units.km;

            var Ostate_deputyCurvilinear = RelativeMotion.RectiToCurvilinear(r, new State(x, y, 0, xDot, yDot, 0));
            Console.WriteLine($"Curvilinear Relative State Deputy (km, km/s):");
            Console.WriteLine(Ostate_deputyCurvilinear / Units.km);
        }

        internal static void Mod3Quiz3Prob4()
        {
            var dr = 10 * Units.km;
            var s = 500 * Units.km;
            var r = 7000 * Units.km;
            var drDot = .1 * Units.km;
            var sDot = -.1 * Units.km;
            var rDot = .05 * Units.km;

            var Ostate_deputyCurvilinear = new State(dr, s, 0, drDot, sDot, 0);
            var Ostate_deputyRectilinear = RelativeMotion.CurviToRectilinear(r, Ostate_deputyCurvilinear);
            Console.WriteLine($"Rectilinear Relative State Deputy (km, km/s):");
            Console.WriteLine(Ostate_deputyRectilinear / Units.km);
        }

        internal static void Mod3Quiz4Prob2()
        {
            var NrVec_c0 = DenseVector.OfArray([-6685.20926, 601.51244, 3346.06634]) * Units.km;
            var NrDotVec_c0 = DenseVector.OfArray([-1.74294, -6.70242, -2.27739]) * Units.km;
            var Orho_0 = DenseVector.OfArray([-81.22301, 248.14201, 94.95904]) * Units.km;
            var OrhoDot_0 = DenseVector.OfArray([0.47884, 0.14857, 0.13577]) * Units.km;

            var Nstate_chief0 = new State(NrVec_c0, NrDotVec_c0);
            var Ostate_deputy0 = new State(Orho_0, OrhoDot_0);
            var Nstate_deputy0 = RelativeMotion.InertialState(Nstate_chief0, Ostate_deputy0);
            Console.WriteLine("Initial Inertial State Deputy (km, km/s):");
            Console.WriteLine(Nstate_deputy0 / Units.km);

            var dXUnperturbed = EquationsOfMotion.dXUnperturbed(mu_Earth);
            var NchiefTrajectory = NumericalMethods.RungeKutta4Integration(dXUnperturbed, new StateEpoch(Nstate_chief0), 1d, 2000d);
            var NdeputyTrajectory = NumericalMethods.RungeKutta4Integration(dXUnperturbed, new StateEpoch(Nstate_deputy0), 1d, 2000d);

            var NdeputyTrajectory_final = NdeputyTrajectory[^1].State;
            var NchiefTrajectory_final = NchiefTrajectory[^1].State;
            var NrhoVec_final = NdeputyTrajectory_final - NchiefTrajectory_final;
            var OrhoVec_finalCheck = RelativeMotion.OrbitRelativeState(NchiefTrajectory_final, NdeputyTrajectory_final);

            var dXRelativeUnperturbed = EquationsOfMotion.dXRelativeUnperturbed(NchiefTrajectory, mu_Earth);
            var OdeputyTrajectory = NumericalMethods.RungeKutta4Integration(dXRelativeUnperturbed, new StateEpoch(Ostate_deputy0), 1d, 2000d);
            Console.WriteLine("Final Relative State Deputy from Inertial Propagation (km, km/s):");
            Console.WriteLine(OrhoVec_finalCheck / Units.km);

            Console.WriteLine("Final Relative State Deputy from Relative Propagation (km, km/s):");
            Console.WriteLine(OdeputyTrajectory[^1].State / Units.km);
        }

        internal static void Mod3Quiz5Prob2()
        {
            var NrVec_c0 = DenseVector.OfArray([-6685.20926, 601.51244, 3346.06634]) * Units.km;
            var NrDotVec_c0 = DenseVector.OfArray([-1.74294, -6.70242, -2.27739]) * Units.km;
            var Orho_0 = DenseVector.OfArray([-81.22301, 248.14201, 94.95904]) * Units.km;
            var OrhoDot_0 = DenseVector.OfArray([0.47884, 0.14857, 0.13577]) * Units.km;

            var Nstate_chief0 = new State(NrVec_c0, NrDotVec_c0);
            var Ostate_deputy0 = new State(Orho_0, OrhoDot_0);

            var dXUnperturbed = EquationsOfMotion.dXUnperturbed(mu_Earth);
            var NchiefTrajectory = NumericalMethods.RungeKutta4Integration(dXUnperturbed, new StateEpoch(Nstate_chief0), 1d, 2000d);

            var dXRelativeUnperturbedLinearized = EquationsOfMotion.dXRelativeUnperturbedLinearized(NchiefTrajectory, mu_Earth);
            var OdeputyTrajectory = NumericalMethods.RungeKutta4Integration(dXRelativeUnperturbedLinearized, new StateEpoch(Ostate_deputy0), 1d, 2000d);

            Console.WriteLine("Final Relative State Deputy from Linearized Relative Propagation (km, km/s):");
            Console.WriteLine(OdeputyTrajectory[^1].State / Units.km);
        }

        internal static void Mod3Quiz6Prob1()
        {
            var r_chief = 6800 * Units.km;
            Vector Orho_0 = DenseVector.OfArray([1.299038, -1.0000, 0.3213938]) * Units.km;
            Vector OrhoDot_0 = DenseVector.OfArray([-0.000844437, -0.00292521, -0.000431250]) * Units.km;

            var Ostate_deputy0 = new State(Orho_0, OrhoDot_0);
            var n = Math.Sqrt(mu_Earth / Math.Pow(r_chief, 3));

            var dXCw = EquationsOfMotion.dXClohessyWiltshire(n);
            var OdeputyTrajectory = NumericalMethods.RungeKutta4Integration(dXCw, new StateEpoch(Ostate_deputy0), 1d, 1300d);
            Console.WriteLine("Final Relative State Deputy from CW Propagation (km, km/s):");
            Console.WriteLine(OdeputyTrajectory[^1].State / Units.km);
        }

        internal static void Mod3Quiz6Prob2()
        {
            var r_chief0 = 7500 * Units.km;
            var y_off = 200 * Units.km;

            Vector NrVec_chief0 = DenseVector.OfArray([r_chief0, 0, 0]);
            var v_chief0 = TwoBodyProblem.CirucularVelocity(r_chief0, mu_Earth);
            Vector NvVec_chief0 = DenseVector.OfArray([0, v_chief0, 0]);
            State Nstate_chief0 = new(NrVec_chief0, NvVec_chief0);

            Vector OrhoVec_0 = DenseVector.OfArray([0, y_off, 0]);
            Vector NrVec_deputy0 = RelativeMotion.InertialPosition(Nstate_chief0, OrhoVec_0);
            State Nstate_deputy0 = new(NrVec_deputy0, NvVec_chief0);

            double t_f = 2000;
            var dXUnperturbed = EquationsOfMotion.dXUnperturbed(mu_Earth);
            var NchiefTrajectory = NumericalMethods.RungeKutta4Integration(dXUnperturbed, Nstate_chief0, 1d, t_f);
            var NdeputyTrajectory = NumericalMethods.RungeKutta4Integration(dXUnperturbed, Nstate_deputy0, 1d, t_f);

            var NrhoVec_final = NdeputyTrajectory[^1].State.Position - NchiefTrajectory[^1].State.Position;
            var rho_final = NrhoVec_final.L2Norm();
            Console.WriteLine($"Final Deputy-Cheif Range (km) after {t_f} seconds:");
            Console.WriteLine(rho_final / Units.km);
        }

        internal static void Mod3Quiz12Prob3()
        {
            Matrix A = RelativeMotion.LinearElementDifferenceToLvlhMap(7500, 13 * Units.deg, 22 * Units.deg, .00707107, .00707107, 70 * Units.deg, mu_Earth / Math.Pow(Units.km, 3));
            Console.WriteLine("A Matrix:");
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    Console.Write($"{A[i, j]},");
                }
                Console.WriteLine();
            }
        }

        internal static void Mod3Quiz12Prob4()
        {
            Matrix Ainv = RelativeMotion.LinearLvlhToElementDifferenceMap(7500, 13 * Units.deg, 22 * Units.deg, .00707107, .00707107, 70 * Units.deg, mu_Earth / Math.Pow(Units.km, 3));
            Console.WriteLine("Ainv Matrix:");
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    Console.Write($"{Ainv[i, j]},");
                }
                Console.WriteLine();
            }
        }

        internal static void Mod3Quiz14()
        {
            ClassicalElementSet chiefElements = new(7000 * Units.km, 0.01, 78 * Units.deg, 120 * Units.deg, 33 * Units.deg, 45 * Units.deg);
            State chiefState = TwoBodyProblem.StateVector(chiefElements, mu_Earth);

            ClassicalElementSet deputyElements = new(chiefElements, δυ: 1 * Units.deg);
            State deputyState = TwoBodyProblem.StateVector(deputyElements, mu_Earth);

            State cartesianCom = RelativeMotion.CartesianCenterOfMass(
            [
                (chiefState, 1),
                (deputyState, 1)
            ]);

            Console.WriteLine("Center of Mass State (km, km/s):");
            Console.WriteLine(cartesianCom / Units.km);

            ClassicalElementSet elementsCom = RelativeMotion.ElementBarycenter(
                [
                    (chiefElements, 1),
                    (deputyElements, 1)
                ]);
            Vector NrVec_elBarycenter = TwoBodyProblem.StateVector(elementsCom, mu_Earth).Position;

            Console.WriteLine("Center of Mass State from Elements (km):");
            Console.WriteLine((Vector)(NrVec_elBarycenter / Units.km));

            ClassicalElementSet cartesianComElset = TwoBodyProblem.ClassicalElementsFromState(cartesianCom, mu_Earth);
            var cartesianPeriod = TwoBodyProblem.Period(cartesianComElset.a, mu_Earth);
            var elementsPeriod = TwoBodyProblem.Period(elementsCom.a, mu_Earth);
            var periodDiff = Math.Abs(cartesianPeriod - elementsPeriod);

            Console.WriteLine($"Period difference between cartesian and elements COMs (s): {periodDiff}");
        }

        internal static void Mod3Quiz15Prob1()
        {
            var a = 7500 * Units.km;
            var e = 0.01;
            var dela = 50 * Units.km;
            var delM_0 = 15 * Units.deg;
            var t_f = 14400;

            var delM_f = RelativeMotion.LinearizedMeanAnomalyDifference(a, dela, delM_0, t_f, mu_Earth);
            Console.WriteLine($"Mean Anomaly Difference after {t_f} seconds: {delM_f / Units.deg} deg");
        }

        internal static void Mod3Quiz16Prob1()
        {
            ClassicalElementSet oe_chief0 = new(10000 * Units.km, 0.2, 37 * Units.deg, 40 * Units.deg, 65 * Units.deg, 10 * Units.deg);
            ClassicalElementSetMeanAnomaly delOe_deputy = new(0, .0001, .001, .001, .001, -.001);

            var OrhoVec_0 = RelativeMotion.OrbitalRelativePosition(oe_chief0, delOe_deputy);
            Console.WriteLine("Initial Relative Position Deputy (km):");
            Console.WriteLine(OrhoVec_0 / Units.km);

            var chiefElements_2 = new ClassicalElementSet(oe_chief0, δυ: 60 * Units.deg);
            var OrhoVec_2 = RelativeMotion.OrbitalRelativePosition(chiefElements_2, delOe_deputy);
            Console.WriteLine("Relative Position Deputy 2 (km):");
            Console.WriteLine(OrhoVec_2 / Units.km);
        }

        internal static void Mod3Quiz17Prob4()
        {
            ClassicalElementSet oe_chief = new(8000 * Units.km, .1, 80 * Units.deg, 0, 0, 0);
            ClassicalElementSet delOe_deputy = new(10 * Units.km, .05, 1 * Units.deg, 0, 0, 0);


            double deltaOmega = RelativeMotion.J2EffectDeltaDelOmega(oe_chief, delOe_deputy, 2 * Math.PI, r_eq, J2_Earth, mu_Earth);
            Console.WriteLine("Change in Deputy Orbital Elements due to J2 over one period:");
            Console.WriteLine(deltaOmega / Units.deg);
        }

        internal static void Mod4Quiz2Prob2()
        {
            double e = .1;
            double a = 8000 * Units.km;
            double x_off = -10 * Units.km;

            double yDot = RelativeMotion.YDotAtPeriapsisForBoundedMotion(a, e, x_off, mu_Earth);
            Console.WriteLine($"yDot at Periapsis for Bounded Motion (km/s): {yDot / Units.km}");
        }

        internal static void Mod4Quiz3Prob4()
        {
            double a = 8000 * Units.km;
            double e = .1;
            double i = 33 * Units.deg;

            double dOmegaDt = Perturbation.J2MeanRaanRate(a, e, i, mu_Earth, J2_Earth, r_eq);
            double timeTo1Revolution = 2 * Math.PI / Math.Abs(dOmegaDt);
            Console.WriteLine($"Time for RAAN to Precess One Revolution (days): {timeTo1Revolution / Units.day}");
        }

        internal static void Mod4Quiz9Prob2()
        {
            double a = 8000 * Units.km;
            double e = .1;
            double i = 33 * Units.deg;
            double deli = .1 * Units.deg;

            double delomegaDot = RelativeMotion.J2InvarientDelomegaDot(a, e, i, deli, J2_Earth, r_eq, mu_Earth);
            Console.WriteLine($"Change in Argument of Periapsis Rate due to Inclination Change (deg/s): {Math.Abs(delomegaDot) / Units.deg}");
        }

        internal static void Mod4Quiz10()
        {
            double a = 8000 * Units.km;
            double e = .1;
            double deltaOmega = .1 * Units.deg;

            double deltaV_peri = RelativeMotion.DeltaVChangeDelomegaDelMConstantPeriapsis(a, e, deltaOmega, mu_Earth);
            Console.WriteLine($"DeltaV for periapsis manuever: {deltaV_peri}");
            double deltaV_ap = RelativeMotion.DeltaVChangeDelomegaDelMConstantApoapsis(a, e, deltaOmega, mu_Earth);
            Console.WriteLine($"DeltaV for apoapsis manuever: {deltaV_ap}");
        }
    }
}
