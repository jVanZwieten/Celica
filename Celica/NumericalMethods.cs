using System.Numerics;

namespace Celica
{
    public static class NumericalMethods
    {
        public static StateEpoch[] RungeKutta4Integration(Func<State, float, State> dxVecDt, StateEpoch xVec_0, float dt, float timeSpan)
        {
            var steps = (int)MathF.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0, dt, steps);
        }
        public static StateEpoch[] RungeKutta4Integration(Func<State, float, State> dxVecDt, StateEpoch xVec_0, float dt, int steps)
        {
            var states = new StateEpoch[steps + 1];
            states[0] = xVec_0;

            for (int i = 1; i <= steps; i++)
            {
                states[i] = RungeKutta4(dxVecDt, states[i - 1], dt);
            }

            return states;
        }

        public static StateEpoch[][] RungeKutta4Integration(Func<State, float, State> dxVecDt, StateEpoch[] xVec_0s, float dt, float timeSpan)
        {
            var steps = (int)MathF.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0s, dt, steps);
        }

        public static StateEpoch[][] RungeKutta4Integration(Func<State, float, State> dxVecDt, StateEpoch[] xVec_0s, float dt, int steps)
        {
            var orbiterStates = new StateEpoch[xVec_0s.Length][];
            for (int i = 0; i < xVec_0s.Length; i++)
            {
                orbiterStates[i] = new StateEpoch[steps + 1];
                orbiterStates[i][0] = xVec_0s[i];
            }

            for (int stepIndex = 1; stepIndex <= steps; stepIndex++)
            {
                for (int orbiterIndex = 0; orbiterIndex < orbiterStates.Length; orbiterIndex++)
                {
                    orbiterStates[orbiterIndex][stepIndex] = RungeKutta4(dxVecDt, orbiterStates[orbiterIndex][stepIndex - 1], dt);
                }
            }
            return orbiterStates;
        }

        public static StateEpoch RungeKutta4(Func<State, float, State> dxVecDt, StateEpoch xVec_n, float dt)
        {
            var K1 = dxVecDt(xVec_n.State, xVec_n.Epoch);
            var K2 = dxVecDt(xVec_n.State + dt / 2 * K1, xVec_n.Epoch + dt / 2);
            var K3 = dxVecDt(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);
            var K4 = dxVecDt(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);

            var xVec_next = xVec_n.State + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
            var t_next = xVec_n.Epoch + dt;
            return new StateEpoch(xVec_next, t_next);
        }
    }
}
