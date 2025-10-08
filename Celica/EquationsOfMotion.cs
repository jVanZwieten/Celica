using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class EquationsOfMotion
    {
        public static Vector GravityAcceleration(Vector rVec, double mu)
        {
            double r = rVec.L2Norm();
            return (Vector)(-mu / Math.Pow(r, 3) * rVec);
        }

        public static Func<State, double, State> dXUnperturbed(double mu) => (State state, double epoch) => new State(state.Velocity, GravityAcceleration(state.Position, mu));

        public static Vector RelativeUnperturbedAcceleration(double r_chief, double rDot_chief, double fDot, State Ostate_deputy, double mu)
        {
            double x = Ostate_deputy.X;
            double y = Ostate_deputy.Y;
            double z = Ostate_deputy.Z;
            double xDot = Ostate_deputy.V_x;
            double yDot = Ostate_deputy.V_y;

            double r_d = Math.Sqrt(Math.Pow(r_chief + x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));

            double xDotDot = 2 * fDot * (yDot - y * rDot_chief / r_chief) + x * Math.Pow(fDot, 2) + mu / Math.Pow(r_chief, 2) - mu / Math.Pow(r_d, 3) * (r_chief + x);
            double yDotDot = -2 * fDot * (xDot - x * rDot_chief / r_chief) + y * Math.Pow(fDot, 2) - mu / Math.Pow(r_d, 3) * y;
            double zDotDot = -mu / Math.Pow(r_d, 3) * z;
            return DenseVector.OfArray([xDotDot, yDotDot, zDotDot]);
        }

        public static Func<State, double, State> dXRelativeUnperturbed(StateEpoch[] NchiefTrajectory, double mu) =>
            (State Ostate_deputy, double epoch) =>
            {
                StateEpoch closestEpoch = NchiefTrajectory
                    .OrderBy(se => Math.Abs(se.Epoch - epoch))
                    .First();
                State NchiefState_epoch = closestEpoch.State;
                double r_chief = NchiefState_epoch.Position.L2Norm();

                var NorbitFrame = RelativeMotion.OrbitFrameRelInertial(NchiefState_epoch);
                Matrix ON = RelativeMotion.InertialToOrbitDcm(NorbitFrame);
                var NomegaVec_ON = NorbitFrame[3];

                var OvVec_chiefEpoch = RelativeMotion.OrbitVelocity(NchiefState_epoch, NorbitFrame[3], ON);
                double rDot_chief = OvVec_chiefEpoch[0];

                double fDot = NomegaVec_ON.L2Norm();

                return new State(Ostate_deputy.Velocity, RelativeUnperturbedAcceleration(r_chief, rDot_chief, fDot, Ostate_deputy, mu));
            };

        public static Vector RelativeUnperturbedLinearizedAcceleration(double r_chief, double rDot_chief, double h, State Ostate_deputy, double mu)
        {
            double x = Ostate_deputy.X;
            double y = Ostate_deputy.Y;
            double z = Ostate_deputy.Z;

            double xDot = Ostate_deputy.V_x;
            double yDot = Ostate_deputy.V_y;

            var p = h * h / mu;
            var fDot = h / (r_chief * r_chief);

            double xDotDot = x * fDot * fDot * (1 + 2 * r_chief / p) + 2 * fDot * (yDot - y * rDot_chief / r_chief);
            double yDotDot = y * fDot * fDot * (1 - r_chief / p) - 2 * fDot * (xDot - x * rDot_chief / r_chief);
            double zDotDot = -r_chief / p * fDot * fDot * z;

            return DenseVector.OfArray([xDotDot, yDotDot, zDotDot]);
        }

        public static Func<State, double, State> dXRelativeUnperturbedLinearized(StateEpoch[] NchiefTrajectory, double mu) =>
            (State Ostate_deputy, double epoch) =>
            {
                StateEpoch closestEpoch = NchiefTrajectory
                    .OrderBy(se => Math.Abs(se.Epoch - epoch))
                    .First();
                State NchiefState_epoch = closestEpoch.State;
                double r_chief = NchiefState_epoch.Position.L2Norm();

                var NorbitFrame = RelativeMotion.OrbitFrameRelInertial(NchiefState_epoch);
                Matrix ON = RelativeMotion.InertialToOrbitDcm(NorbitFrame);
                var NomegaVec_ON = NorbitFrame[3];

                var OvVec_chiefEpoch = RelativeMotion.OrbitVelocity(NchiefState_epoch, NorbitFrame[3], ON);
                double rDot_chief = OvVec_chiefEpoch[0];

                var NhVec = NchiefState_epoch.Position.Cross(NchiefState_epoch.Velocity);
                double h = NhVec.L2Norm();

                return new State(Ostate_deputy.Velocity, RelativeUnperturbedLinearizedAcceleration(r_chief, rDot_chief, h, Ostate_deputy, mu));
            };

        public static Vector ClohessyWiltshireAcceleration(State Ostate_deputy, double n)
        {
            double x = Ostate_deputy.X;
            double y = Ostate_deputy.Y;
            double z = Ostate_deputy.Z;
            double xDot = Ostate_deputy.V_x;
            double yDot = Ostate_deputy.V_y;

            double xDotDot = 3 * n * n * x + 2 * n * yDot;
            double yDotDot = -2 * n * xDot;
            double zDotDot = -n * n * z;
            return DenseVector.OfArray([xDotDot, yDotDot, zDotDot]);
        }

        public static Func<State, double, State> dXClohessyWiltshire(double n) =>
            (State Ostate_deputy, double epoch) =>
            new State(Ostate_deputy.Velocity, ClohessyWiltshireAcceleration(Ostate_deputy, n));
    }
}
