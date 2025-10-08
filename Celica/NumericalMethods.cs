
using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class NumericalMethods
    {
        public static StateEpoch[] RungeKutta4Integration(Func<State, double, State> dxVecDt, StateEpoch xVec_0, double dt, double timeSpan)
        {
            var steps = (int)Math.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0, dt, steps);
        }
        public static StateEpoch[] RungeKutta4Integration(Func<State, double, State> dxVecDt, State xVec_0, double dt, double timeSpan) => RungeKutta4Integration(dxVecDt, new StateEpoch(xVec_0), dt, timeSpan);

        public static StateEpoch[] RungeKutta4Integration(Func<State, double, State> dxVecDt, StateEpoch xVec_0, double dt, int steps)
        {
            var states = new StateEpoch[steps + 1];
            states[0] = xVec_0;
            for (int i = 1; i <= steps; i++)
            {
                states[i] = RungeKutta4(dxVecDt, states[i - 1], dt);
            }
            return states;
        }

        public static StateEpoch[][] RungeKutta4Integration(Func<State, double, State> dxVecDt, StateEpoch[] xVec_0s, double dt, double timeSpan)
        {
            var steps = (int)Math.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0s, dt, steps);
        }

        public static StateEpoch[][] RungeKutta4Integration(Func<State, double, State> dxVecDt, StateEpoch[] xVec_0s, double dt, int steps)
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

        public static StateEpoch RungeKutta4(Func<State, double, State> dxVecDt, StateEpoch xVec_n, double dt)
        {
            var K1 = dxVecDt(xVec_n.State, xVec_n.Epoch);
            var K2 = dxVecDt(xVec_n.State + dt / 2 * K1, xVec_n.Epoch + dt / 2);
            var K3 = dxVecDt(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);
            var K4 = dxVecDt(xVec_n.State + dt * K3, xVec_n.Epoch + dt);
            var xVec_next = xVec_n.State + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
            var t_next = xVec_n.Epoch + dt;
            return new StateEpoch(xVec_next, t_next);
        }

        public static Vector RungeKutta4(Func<Vector, double, Vector> dxVecDt, Vector xVec_n, double t_n, double dt)
        {
            var K1 = dxVecDt(xVec_n, t_n);
            var K2 = dxVecDt((Vector)(xVec_n + dt / 2 * K1), t_n + dt / 2);
            var K3 = dxVecDt((Vector)(xVec_n + dt / 2 * K2), t_n + dt / 2);
            var K4 = dxVecDt((Vector)(xVec_n + dt * K3), t_n + dt);
            return (Vector)(xVec_n + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4));
        }

        public static (double, Vector)[] RungeKutta4Integration(Func<Vector, double, Vector> dxVecDt, Vector xVec_0, double dt, int steps)
        {
            var result = new (double, Vector)[steps + 1];
            result[0] = (0, xVec_0);

            for (int i = 1; i <= steps; i++)
            {
                var t_previous = result[i - 1].Item1;
                var xVec_previous = result[i - 1].Item2;
                var xVec_i = RungeKutta4(dxVecDt, xVec_previous, t_previous, dt);
                var t_i = t_previous + dt;
                result[i] = (t_i, xVec_i);
            }
            return result;
        }
        public static (double, Vector)[] RungeKutta4Integration(Func<Vector, double, Vector> dxVecDt, Vector xVec_0, double dt, double timeSpan)
        {
            var steps = (int)Math.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0, dt, steps);
        }
    }

    public static class NumericalMethodsF
    {
        public static StateEpochF[] RungeKutta4Integration(Func<StateF, float, StateF> dxVecDt, StateEpochF xVec_0, float dt, float timeSpan)
        {
            var steps = (int)MathF.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0, dt, steps);
        }
        public static StateEpochF[] RungeKutta4Integration(Func<StateF, float, StateF> dxVecDt, StateEpochF xVec_0, float dt, int steps)
        {
            var states = new StateEpochF[steps + 1];
            states[0] = xVec_0;

            for (int i = 1; i <= steps; i++)
            {
                states[i] = RungeKutta4(dxVecDt, states[i - 1], dt);
            }

            return states;
        }

        public static StateEpochF[][] RungeKutta4Integration(Func<StateF, float, StateF> dxVecDt, StateEpochF[] xVec_0s, float dt, float timeSpan)
        {
            var steps = (int)MathF.Ceiling(timeSpan / dt);
            return RungeKutta4Integration(dxVecDt, xVec_0s, dt, steps);
        }

        public static StateEpochF[][] RungeKutta4Integration(Func<StateF, float, StateF> dxVecDt, StateEpochF[] xVec_0s, float dt, int steps)
        {
            var orbiterStates = new StateEpochF[xVec_0s.Length][];
            for (int i = 0; i < xVec_0s.Length; i++)
            {
                orbiterStates[i] = new StateEpochF[steps + 1];
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

        public static StateEpochF RungeKutta4(Func<StateF, float, StateF> dxVecDt, StateEpochF xVec_n, float dt)
        {
            var K1 = dxVecDt(xVec_n.State, xVec_n.Epoch);
            var K2 = dxVecDt(xVec_n.State + dt / 2 * K1, xVec_n.Epoch + dt / 2);
            var K3 = dxVecDt(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);
            var K4 = dxVecDt(xVec_n.State + dt / 2 * K2, xVec_n.Epoch + dt / 2);

            var xVec_next = xVec_n.State + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
            var t_next = xVec_n.Epoch + dt;
            return new StateEpochF(xVec_next, t_next);
        }
    }
}
