using System.Numerics;

namespace Celica
{
    public static class NumericalMethods
    {
        public record StateRecord(Time Epoch, Vector<double> State);

        public static StateRecord[] RungeKutta4Integration<Func<Vector<double>, double, Vector<double>>>(Func<Vector<double>, double, Vector<double>> dxVec, Vector<double> xVec_0, double t_0, double dt, int steps)
        {
            var states = new StateRecord[steps + 1];
            states[0] = new StateRecord(t_0, xVec_0);
            for (int i = 1; i <= steps; i++)
            {
                var t_n = t_0 + (i - 1) * dt;
                states[i] = new StateRecord(t_n + dt, RungeKutta4(dxVec, states[i - 1].State, t_n, dt));
            }
            return states;
        }
        public static Vector<double> RungeKutta4(Func<Vector<double>, double, Vector<double>> dxVec, Vector<double> xVec_n, double t_n, double dt)
        {
            var K1 = dxVec(xVec_n, t_n);
            var K2 = dxVec(xVec_n + dt / 2 * K1, t_n + dt / 2);
            var K3 = dxVec(xVec_n + dt / 2 * K2, t_n + dt / 2);
            var K4 = dxVec(xVec_n + dt / 2 * K2, t_n + dt / 2);

            return xVec_n + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
        }
    }
}
