using Celica;
using System.Numerics;

using units = Celica.PhysicalUnitsF;

const float mu_earth = CelestialParameters.Earth.Mu;

var altitude = 700f * units.km;
var a = CelestialParameters.Earth.R(altitude);
var v = TwoBPF.CircularVelocity(a, mu_earth);
v += 200f;

var xVec_0 = new StateEpoch(
    new State(a, 0, 0, 0, v, 0),
    0);

Func<State, float, State> dxVec = (state, epoch) => new State(state.Velocity, -mu_earth / MathF.Pow(state.Position.Length(), 3) * state.Position);

float T = TwoBPF.Period(a, mu_earth);

var trajectory = NumericalMethods.RungeKutta4Integration(dxVec, xVec_0, .01f, T);

PlotUtilities.Plot2D(trajectory);