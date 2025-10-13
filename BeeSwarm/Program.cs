// TODO:
// x initialize chief and 3 deputies
// x propagate chief and deputies using CW
// x plot relative trajectories for validation
// x propagate using 2BP
// x plot each trajectory for validation
// x implement multi-colored plotting
// x check geometry of all relative orbits
// o implement observability

// Nice to have
// o progress meter on integrator
// o multi-thread integrator

using BeeSwarm;
using Celica;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Windows.Forms;
using Units = Celica.PhysicalUnits;

const double MU_EARTH = CelestialParameters.Earth.Mu;
const double INTEGRATION_DT = .1; // .001 ~ 5m accuracy for 1 orbit
const double PLOT_DT = 60;
int SAMPLE_INTERVAL = (int)Math.Floor(PLOT_DT / INTEGRATION_DT);

var altitude_chief = 1000 * Units.km;
var r_chief = CelestialParameters.Earth.R(altitude_chief);
var v_chief = TwoBodyProblem.CirucularVelocity(r_chief, MU_EARTH);

var NrVec_chief0 = DenseVector.OfArray([r_chief, 0, 0]);
var NvVec_chief0 = DenseVector.OfArray([0, v_chief, 0]);
var Nstate_chief_0 = new State(NrVec_chief0, NvVec_chief0);

var orbels_chief = TwoBodyProblem.ClassicalElementsFromState(Nstate_chief_0, MU_EARTH);
var P_chief = TwoBodyProblem.Period(orbels_chief.a, MU_EARTH);
var n_chief = TwoBodyProblem.MeanMotion(orbels_chief.a, MU_EARTH);

var dXVecDt_2BP = EquationsOfMotion.dXUnperturbed(MU_EARTH);
var trajectory_chief = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, Nstate_chief_0, INTEGRATION_DT, P_chief);

Console.WriteLine("Chief propagation ECI 2BP:");
PrintStateDeltas(trajectory_chief.First().State, trajectory_chief.Last().State);
PlotUtilities.Plot2D(trajectory_chief, sampleInterval: SAMPLE_INTERVAL, title: "Chief Orbit (2BP)");

var deputyStates = InitializeDeputyInitialStates(n_chief);
var Ostate_deputy1_0 = deputyStates[0];
var Ostate_deputy2_0 = deputyStates[1];
var Ostate_deputy3_0 = deputyStates[2];

StateEpoch[][] cwTrajectories = PropagateDeputyCwTrajectories(INTEGRATION_DT, SAMPLE_INTERVAL, P_chief, n_chief, Ostate_deputy1_0, Ostate_deputy2_0, Ostate_deputy3_0);
//PropagateInertialTrajectories(INTEGRATION_DT, SAMPLE_INTERVAL, Nstate_chief_0, P_chief, dXVecDt_2BP, Ostate_deputy1_0, Ostate_deputy2_0, Ostate_deputy3_0);
//PropagateInertialCurviTrajectories(INTEGRATION_DT, SAMPLE_INTERVAL, r_chief, Nstate_chief_0, P_chief, dXVecDt_2BP, Ostate_deputy1_0, Ostate_deputy2_0, Ostate_deputy3_0);

var NrHat_sun = DenseVector.OfArray([1, 0, 0]); // sun is always assumed to be in the ECI +x direction
Vector[] OrHat_sun = trajectory_chief.Select(se => (Vector)(RelativeMotion.InertialToOrbitDcm(se.State) * NrHat_sun)).ToArray();
var observations = Observation.ObservationScores(cwTrajectories, OrHat_sun);
PlotUtilities.Plot2DTimeSeries(observations, INTEGRATION_DT, "Deputy Observation Scores", "Observation Score", seriesNames: ["deputy1", "deputy2", "deputy3"]);

Application.Run();

static State[] InitializeDeputyInitialStates(double n_chief)
{
    // initialize deputies in CW frame
    var Orho_deputy1_0 = DenseVector.OfArray([100, 0, 0]);
    var yDotBounded_deputy1_0 = ClohessyWiltshire.BoundedYDot(Orho_deputy1_0[0], n_chief);
    var Ov_deputy1_0 = DenseVector.OfArray([0, yDotBounded_deputy1_0, 0]);
    var Ostate_deputy1_0 = new State(Orho_deputy1_0, Ov_deputy1_0);
    var Orho_deputy2_0 = DenseVector.OfArray([0, 2000, 0]);
    var xDotBounded_deputy2_0 = ClohessyWiltshire.BoundedXDot(Orho_deputy2_0[1], n_chief);
    var Ov_deputy2_0 = DenseVector.OfArray([xDotBounded_deputy2_0, 0, 0]);
    var Ostate_deputy2_0 = new State(Orho_deputy2_0, Ov_deputy2_0);
    var Orho_deputy3_0 = DenseVector.OfArray([200, 200, 200]);
    var yDotBounded_deputy3_0 = ClohessyWiltshire.BoundedYDot(Orho_deputy3_0[0], n_chief);
    var xDotBounded_deputy3_0 = ClohessyWiltshire.BoundedXDot(Orho_deputy3_0[1], n_chief);
    var Ov_deputy3_0 = DenseVector.OfArray([xDotBounded_deputy3_0, yDotBounded_deputy3_0, 0]);
    var Ostate_deputy3_0 = new State(Orho_deputy3_0, Ov_deputy3_0);

    return [Ostate_deputy1_0, Ostate_deputy2_0, Ostate_deputy3_0];
}

static StateEpoch[][] PropagateDeputyCwTrajectories(double INTEGRATION_DT, int SAMPLE_INTERVAL, double P_chief, double n_chief, State Ostate_deputy1_0, State Ostate_deputy2_0, State Ostate_deputy3_0)
{
    Console.WriteLine("\nCW propagation:");

    var dXVecDt_CW = ClohessyWiltshire.dXVecDt(n_chief);
    var cwTrajectory_deputy1 = NumericalMethods.RungeKutta4Integration(dXVecDt_CW, Ostate_deputy1_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Ostate_deputy1_0, cwTrajectory_deputy1.Last().State);
    var cwTrajectory_deputy2 = NumericalMethods.RungeKutta4Integration(dXVecDt_CW, Ostate_deputy2_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Ostate_deputy2_0, cwTrajectory_deputy2.Last().State);
    var cwTrajectory_deputy3 = NumericalMethods.RungeKutta4Integration(dXVecDt_CW, Ostate_deputy3_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Ostate_deputy3_0, cwTrajectory_deputy3.Last().State);

    StateEpoch[][] trajectories = [cwTrajectory_deputy1, cwTrajectory_deputy2, cwTrajectory_deputy3];
    PlotUtilities.Plot2D(trajectories, sampleInterval: SAMPLE_INTERVAL, title: "CW propagation of deputies about chief");
    return trajectories;
}

static void PropagateInertialTrajectories(double INTEGRATION_DT, int SAMPLE_INTERVAL, State Nstate_chief_0, double P_chief, Func<State, double, State> dXVecDt_2BP, State Ostate_deputy1_0, State Ostate_deputy2_0, State Ostate_deputy3_0)
{
    Console.WriteLine("\ndeputy ECI 2BP propagation:");

    var Nstate_deputy1_0 = RelativeMotion.InertialState(Nstate_chief_0, Ostate_deputy1_0);
    var Ntrajectory_deputy1 = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, Nstate_deputy1_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Nstate_deputy1_0, Ntrajectory_deputy1.Last().State);

    var Nstate_deputy2_0 = RelativeMotion.InertialState(Nstate_chief_0, Ostate_deputy2_0);
    var Ntrajectory_deputy2 = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, Nstate_deputy2_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Nstate_deputy2_0, Ntrajectory_deputy2.Last().State);

    var Nstate_deputy3_0 = RelativeMotion.InertialState(Nstate_chief_0, Ostate_deputy3_0);
    var Ntrajectory_deputy3 = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, Nstate_deputy3_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(Nstate_deputy3_0, Ntrajectory_deputy3.Last().State);

    StateEpoch[][] Ntrajectories = [Ntrajectory_deputy1, Ntrajectory_deputy2, Ntrajectory_deputy3];
    PlotUtilities.Plot2D(Ntrajectories, sampleInterval: SAMPLE_INTERVAL, title: "Inertial propagation of deputies.");
}

static void PropagateInertialCurviTrajectories(double INTEGRATION_DT, int SAMPLE_INTERVAL, double r_chief, State Nstate_chief_0, double P_chief, Func<State, double, State> dXVecDt_2BP, State Ostate_deputy1_0, State Ostate_deputy2_0, State Ostate_deputy3_0)
{
    Console.WriteLine("\ndeputy ECI curvilinear 2BP propagation:");

    var NFromCurvilinearState_deputy1_0 = RelativeMotion.InertialState(Nstate_chief_0, RelativeMotion.CurviToRectilinear(r_chief, Ostate_deputy1_0));
    var Ntrajectory_deputy1_fromCurvilinear = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, NFromCurvilinearState_deputy1_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(NFromCurvilinearState_deputy1_0, Ntrajectory_deputy1_fromCurvilinear.Last().State);

    var NFromCurvilinearState_deputy2_0 = RelativeMotion.InertialState(Nstate_chief_0, RelativeMotion.CurviToRectilinear(r_chief, Ostate_deputy2_0));
    var Ntrajectory_deputy2_fromCurvilinear = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, NFromCurvilinearState_deputy2_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(NFromCurvilinearState_deputy2_0, Ntrajectory_deputy2_fromCurvilinear.Last().State);

    var NFromCurvilinearState_deputy3_0 = RelativeMotion.InertialState(Nstate_chief_0, RelativeMotion.CurviToRectilinear(r_chief, Ostate_deputy3_0));
    var Ntrajectory_deputy3_fromCurvilinear = NumericalMethods.RungeKutta4Integration(dXVecDt_2BP, NFromCurvilinearState_deputy3_0, INTEGRATION_DT, P_chief);
    PrintStateDeltas(NFromCurvilinearState_deputy3_0, Ntrajectory_deputy3_fromCurvilinear.Last().State);

    StateEpoch[][] NCuriTrajectories = [Ntrajectory_deputy1_fromCurvilinear, Ntrajectory_deputy2_fromCurvilinear, Ntrajectory_deputy3_fromCurvilinear];
    PlotUtilities.Plot2D(NCuriTrajectories, sampleInterval: SAMPLE_INTERVAL, title: "Inertial propagation of deputies, taken as curvilinear intial conditions.");
}

static void PrintStateDeltas(State state1, State state2)
{
    var deltaState = state2 - state1;
    Console.WriteLine($"ΔState: {deltaState}");
    Console.WriteLine($"Δr = {deltaState.Position.L2Norm()}, Δv = {deltaState.Velocity.L2Norm()}");
}