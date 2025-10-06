using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class RelativeMotion
    {
        public static Vector[] OrbitFrameRelInertial(State state_chief)
        {
            var NrVec = state_chief.Position;
            var NvVec = state_chief.Velocity;

            Vector NoHat_r = (Vector)NrVec.Normalize(2);

            Vector NhVec = NrVec.Cross(NvVec);
            Vector NoHat_h = (Vector)NhVec.Normalize(2);

            Vector NoHat_theta = NoHat_h.Cross(NoHat_r);

            Vector NomegaVec_ON = (Vector)(NhVec / Math.Pow(NrVec.L2Norm(), 2));

            return [NoHat_r, NoHat_theta, NoHat_h, NomegaVec_ON];
        }

        public static Matrix InertialToOrbitDcm(Vector[] NorbitFrame) => DenseMatrix.OfRowVectors(NorbitFrame[0], NorbitFrame[1], NorbitFrame[2]);
        public static Matrix InertialToOrbitDcm(State state_chief) => InertialToOrbitDcm(OrbitFrameRelInertial(state_chief));

        public static Vector OrbitRelativePosition(Vector NrVec_chief, Vector NrVec_deputy, Matrix ON) => (Vector)(ON * (NrVec_deputy - NrVec_chief));
        public static Vector OrbitRelativeVelocity(Vector NvVec_chief, Vector NvVec_deputy, Vector OomegaVecON, Vector OrhoVec, Matrix ON) => (Vector)(ON * (NvVec_deputy - NvVec_chief) - OomegaVecON.Cross(OrhoVec));

        public static State OrbitRelativeState(State state_chief, State state_deputy)
        {
            var NorbitFrame = OrbitFrameRelInertial(state_chief);
            Matrix ON = InertialToOrbitDcm(NorbitFrame);

            var fDot = NorbitFrame[3].L2Norm();
            var OomegaVec_ON = (Vector)DenseVector.OfArray([0, 0, fDot]);

            Vector OrhoVec = OrbitRelativePosition(state_chief.Position, state_deputy.Position, ON);
            Vector OrhoDotVec = OrbitRelativeVelocity(state_chief.Velocity, state_deputy.Velocity, OomegaVec_ON, OrhoVec, ON);
            return new State(OrhoVec, OrhoDotVec);
        }

        public static Matrix OrbitToInertialDcm(Vector[] NorbitFrame) => DenseMatrix.OfColumnVectors(NorbitFrame[0], NorbitFrame[1], NorbitFrame[2]);
        public static Matrix OrbitToInertialDcm(State Nstate_chief) => OrbitToInertialDcm(OrbitFrameRelInertial(Nstate_chief));

        public static Vector InertialPosition(Vector NrVec_chief, Vector OrhoVec, Matrix NO) => (Vector)(NrVec_chief + (NO * OrhoVec));
        public static Vector InertialVelocity(Vector NvVec_chief, State Ostate_deputy, Vector OomegaVec_ON, Matrix NO) => (Vector)(NvVec_chief + NO * (Ostate_deputy.Velocity + OomegaVec_ON.Cross(Ostate_deputy.Position)));

        public static State InertialState(State Nstate_chief, State Ostate_deputy)
        {
            var NorbitFrame = OrbitFrameRelInertial(Nstate_chief);
            Matrix NO = OrbitToInertialDcm(NorbitFrame);

            var fDot = NorbitFrame[3].L2Norm();
            var OomegaVec_ON = (Vector)DenseVector.OfArray([0, 0, fDot]);

            Vector NrVec_deputy = InertialPosition(Nstate_chief.Position, Ostate_deputy.Position, NO);
            Vector NvVec_deputy = InertialVelocity(Nstate_chief.Velocity, Ostate_deputy, OomegaVec_ON, NO);
            return new State(NrVec_deputy, NvVec_deputy);
        }

        public static State RectiToCurvilinear(double r_c, State Ostate_deputy)
        {
            var x = Ostate_deputy.X;
            var y = Ostate_deputy.Y;
            var z = Ostate_deputy.Z;
            var xDot = Ostate_deputy.V_x;
            var yDot = Ostate_deputy.V_y;
            var zDot = Ostate_deputy.V_z;

            var r_dSquared = Math.Pow(r_c + x, 2) + Math.Pow(y, 2);
            var r_d = Math.Sqrt(r_dSquared);

            var dr = r_d - r_c;
            var dTheta = Math.Atan2(y, r_c + x);
            var s = r_c * dTheta;
            var OrhoVec = new DenseVector([dr, s, z]);

            var drDot = ((r_c + x) * xDot + y * yDot) / r_d;
            var dThetaDot = ((r_c + x) * yDot - y * xDot) / r_dSquared;
            var sDot = r_c * dThetaDot;
            var OrhoDotVec = new DenseVector([drDot, sDot, zDot]);

            return new State(OrhoVec, OrhoDotVec);
        }

        public static State CurviToRectilinear(double r_chief, State Ostate_deputyCurvilinear)
        {
            var dr = Ostate_deputyCurvilinear.X;
            var s = Ostate_deputyCurvilinear.Y;
            var z = Ostate_deputyCurvilinear.Z;

            var drDot = Ostate_deputyCurvilinear.V_x;
            var sDot = Ostate_deputyCurvilinear.V_y;
            var zDot = Ostate_deputyCurvilinear.V_z;

            var r_deputy = r_chief + dr;
            var dTheta = s / r_chief;
            var dThetaDot = sDot / r_chief;
            var cosTheta = Math.Cos(dTheta);
            var sinTheta = Math.Sin(dTheta);

            var x = r_deputy * cosTheta - r_chief;
            var y = r_deputy * sinTheta;
            var xDot = drDot * cosTheta - r_deputy * sinTheta * dThetaDot;
            var yDot = drDot * sinTheta + r_deputy * cosTheta * dThetaDot;
            return new State(x, y, z, xDot, yDot, zDot);
        }
    }
}
