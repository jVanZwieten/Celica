using Celica;

using units = Celica.PhysicalUnits;

const float mu_earth = CelestialParametersF.Earth.Mu;

var altitude = 700f * units.km;
var a = CelestialParametersF.Earth.R(altitude);
var v = TwoBPF.CircularVelocity(a, mu_earth);
v += 200f;

var xVec_0 = new StateEpochF(
    new StateF(a, 0, 0, 0, v, 0),
    0);

Func<StateF, float, StateF> dxVec = (state, epoch) => new StateF(state.Velocity, -mu_earth / MathF.Pow(state.Position.Length(), 3) * state.Position);

float T = TwoBPF.Period(a, mu_earth);

var trajectory = NumericalMethods.RungeKutta4Integration(dxVec, xVec_0, .01f, T);

PlotUtilities.Plot2D(trajectory);