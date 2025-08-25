using System.Numerics;

namespace Celica
{
    public static class NumericalMethods
    {
        public static StateEpoch[] RungeKutta4Integration(Func<State, float, State> dxVec, StateEpoch xVec_0, float dt, float timeSpan)
        {
            var steps = (int)MathF.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVec, xVec_0, dt, steps);
        }
        public static StateEpoch[] RungeKutta4Integration(Func<State, float, State> dxVec, StateEpoch xVec_0, float dt, int steps)
        {
            var states = new StateEpoch[steps + 1];
            states[0] = xVec_0;

            for (int i = 1; i <= steps; i++)
            {
                var t_n = states[i - 1].Epoch;
                states[i] = RungeKutta4(dxVec, states[i - 1], dt);
            }

            return states;
        }

        public static StateEpoch RungeKutta4(Func<State, float, State> dxVec, StateEpoch xVec_n, float dt)
        {
            var K1 = dxVec(xVec_n.State, xVec_n.Epoch);
            var K2 = dxVec(xVec_n.State + dt / 2 * K1, xVec_n.Epoch + dt / 2);
            var K3 = dxVec(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);
            var K4 = dxVec(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);

            var xVec_next = xVec_n.State + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
            var t_next = xVec_n.Epoch + dt;
            return new StateEpoch(xVec_next, t_next);
        }
    }
}
