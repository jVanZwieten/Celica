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

        public static Vector OrbitPosition(Vector NrVec, Matrix ON) => (Vector)(ON * NrVec);
        public static Vector OrbitVelocity(Vector NvVec, Vector OomegaVecON, Vector OrhoVec, Matrix ON) => (Vector)(ON * NvVec - OomegaVecON.Cross(OrhoVec));
        public static Vector OrbitVelocity(State Nstate, Vector OomegaVecON, Matrix ON) => OrbitVelocity(Nstate.Velocity, OomegaVecON, OrbitPosition(Nstate.Position, ON), ON);

        public static Vector OrbitRelativePosition(Vector NrVec_chief, Vector NrVec_deputy, Matrix ON) => OrbitPosition((Vector)(NrVec_deputy - NrVec_chief), ON);
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
        public static Vector InertialPosition(State Nstate_chief, Vector OrhoVec) => InertialPosition(Nstate_chief.Position, OrhoVec, OrbitToInertialDcm(Nstate_chief));

        public static Vector InertialVelocity(Vector NvVec_chief, State Ostate_deputy, Vector OomegaVec_ON, Matrix NO) => (Vector)(NvVec_chief + NO * (Ostate_deputy.Velocity + OomegaVec_ON.Cross(Ostate_deputy.Position)));
        public static Vector InertialVelocity(State Nstate_chief, State Ostate_deputy)
        {
            var NOrbitFrame = OrbitFrameRelInertial(Nstate_chief);
            Matrix NO = OrbitToInertialDcm(NOrbitFrame);
            var OomegaVec_ON = (Vector)DenseVector.OfArray([0, 0, NOrbitFrame[3].L2Norm()]);

            return InertialVelocity(Nstate_chief.Velocity, Ostate_deputy, OomegaVec_ON, NO);
        }

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

        static double Eccentricity(double q_1, double q_2) => Math.Sqrt(q_1 * q_1 + q_2 * q_2);
        static double ArgumentOfPeriapsis(double q_1, double q_2) => Math.Atan2(q_2, q_1);
        static double RadialVelocity(double h, double p, double theta, double q_1, double q_2) => h / p * (q_1 * Math.Sin(theta) - q_2 * Math.Cos(theta));
        static double TransverseVelocity(double h, double p, double theta, double q_1, double q_2) => h / p * (1 + q_1 * Math.Cos(theta) + q_2 * Math.Sin(theta));

        public static Matrix LinearElementDifferenceToLvlhMap(double a, double theta, double i, double q_1, double q_2, double Omega, double mu)
        {
            double e = Eccentricity(q_1, q_2);
            double p = TwoBodyProblem.SemiLatusRectum(a, e);
            double h = TwoBodyProblem.SpecificAngularMomentum(p, mu);
            double omega = ArgumentOfPeriapsis(q_1, q_2);
            double f = theta - omega;
            double r = TwoBodyProblem.RConic(e, p, f);

            double v_r = RadialVelocity(h, p, theta, q_1, q_2);
            double v_t = TransverseVelocity(h, p, theta, q_1, q_2);

            double cosTheta = Math.Cos(theta);
            double sinTheta = Math.Sin(theta);
            double cosi = Math.Cos(i);
            double sini = Math.Sin(i);

            Vector x = DenseVector.OfArray([r / a, v_r / v_t * r, 0, -r / p * (2 * a * q_1 + r * cosTheta), -r / p * (2 * a * q_2 + r * sinTheta), 0]);
            Vector y = DenseVector.OfArray([0, r, 0, 0, 0, r * cosi]);
            Vector z = DenseVector.OfArray([0, 0, r * sinTheta, 0, 0, -r * cosTheta * sini]);
            Vector xDot = DenseVector.OfArray([-v_r / 2 / a, h * (1 / r - 1 / p), 0, (v_r * a * q_1 + h * sinTheta) / p, (v_r * a * q_2 - h * cosTheta) / p, 0]);
            Vector yDot = DenseVector.OfArray([-3 * v_t / 2 / a, -v_r, 0, (3 * v_t * a * q_1 + 2 * h * cosTheta) / p, (3 * v_t * a * q_2 + 2 * h * sinTheta) / p, v_r * cosi]);
            Vector zDot = DenseVector.OfArray([0, 0, v_t * cosTheta + v_r * sinTheta, 0, 0, (v_t * sinTheta - v_r * cosTheta) * sini]);

            return DenseMatrix.OfRowVectors(x, y, z, xDot, yDot, zDot);
        }

        public static Matrix LinearLvlhToElementDifferenceMap(double a, double theta, double i, double q_1, double q_2, double Omega, double mu)
        {
            double e = Eccentricity(q_1, q_2);
            double p = TwoBodyProblem.SemiLatusRectum(a, e);
            double h = TwoBodyProblem.SpecificAngularMomentum(p, mu);
            double omega = ArgumentOfPeriapsis(q_1, q_2);
            double f = theta - omega;
            double r = TwoBodyProblem.RConic(e, p, f);

            double v_r = RadialVelocity(h, p, theta, q_1, q_2);
            double v_t = TransverseVelocity(h, p, theta, q_1, q_2);

            double alpha = a / r;
            double nu = v_r / v_t;
            double rho = r / p;
            double k_1 = alpha * (1 / rho - 1);
            double k_2 = alpha * nu * nu / rho;

            double cosTheta = Math.Cos(theta);
            double cos2Theta = Math.Cos(2 * theta);
            double sinTheta = Math.Sin(theta);
            double sin2Theta = Math.Sin(2 * theta);
            double cosi = Math.Cos(i);
            double sini = Math.Sin(i);
            double coti = cosi / sini;

            double A_11 = 2 * alpha * (2 + 3 * k_1 + 2 * k_2);
            double A_12 = -2 * alpha * nu * (1 + 2 * k_1 + k_2);
            double A_14 = 2 * alpha * alpha * nu * p / v_t;
            double A_15 = 2 * a / v_t * (1 + 2 * k_1 + k_2);

            double A_22 = 1 / r;
            double A_23 = coti / r * (cosTheta + nu * sinTheta);
            double A_26 = -sinTheta * coti / v_t;

            double A_33 = (sinTheta - nu * cosTheta) / r;
            double A_36 = cosTheta / v_t;

            double A_41 = 1 / rho / r * (3 * cosTheta + 2 * nu * sinTheta);
            double A_42 = -1 / r * (nu * nu / rho * sinTheta + q_1 * sin2Theta - q_2 * cos2Theta);
            double A_43 = -q_2 * coti / r * (cosTheta + nu * sinTheta);
            double A_44 = sinTheta / rho / v_t;
            double A_45 = 1 / rho / v_t * (2 * cosTheta + nu * sinTheta);
            double A_46 = q_2 * coti * sinTheta / v_t;

            double A_51 = 1 / rho / r * (3 * sinTheta - 2 * nu * cosTheta);
            double A_52 = 1 / r * (nu * nu / rho * cosTheta + q_2 * sin2Theta + q_1 * cos2Theta);
            double A_53 = q_1 * coti / r * (cosTheta + nu * sinTheta);
            double A_54 = -cosTheta / rho / v_t;
            double A_55 = 1 / rho / v_t * (2 * sinTheta - nu * cosTheta);
            double A_56 = -q_1 * coti * sinTheta / v_t;

            double A_63 = -(cosTheta + nu * sinTheta) / r / sini;
            double A_66 = sinTheta / v_t / sini;

            return DenseMatrix.OfArray(new double[,]
            {
                { A_11, A_12, 0, A_14, A_15, 0 },
                { 0, A_22, A_23, 0, 0, A_26 },
                { 0, 0, A_33, 0, 0, A_36 },
                { A_41, A_42, A_43, A_44, A_45, A_46 },
                { A_51, A_52, A_53, A_54, A_55, A_56 },
                { 0, 0, A_63, 0, 0, A_66 }
            });
        }

        public static State CartesianCenterOfMass(IEnumerable<(State state, double mass)> satellites)
        {
            if (satellites == null || !satellites.Any())
                throw new ArgumentException("satellites list must not be null or empty");

            State stateSum = new();
            double massSum = 0.0;

            foreach (var (state, mass) in satellites)
            {
                stateSum += state * mass;
                massSum += mass;
            }

            if (massSum == 0.0)
                throw new ArgumentException("Total mass must not be zero.");

            return stateSum / massSum;
        }

        public static ClassicalElementSet ElementBarycenter(IEnumerable<(ClassicalElementSet elements, double mass)> satellites)
        {
            if (satellites == null || !satellites.Any())
                throw new ArgumentException("satellites list must not be null or empty");

            ClassicalElementSet elementsSum = new(0, 0, 0, 0, 0, 0);
            double massSum = 0.0;
            foreach (var (elements, mass) in satellites)
            {
                elementsSum += elements;
                massSum += mass;
            }
            if (massSum == 0.0)
                throw new ArgumentException("Total mass must not be zero.");
            return elementsSum / massSum;
        }

        public static double LinearizedMeanAnomalyDrift(double a, double n, double dela) => -3.0 / 2.0 * n * dela / a;
        public static double LinearizedMeanAnomalyDriftMu(double a, double dela, double mu) => LinearizedMeanAnomalyDrift(a, TwoBodyProblem.MeanMotion(a, mu), dela);
        public static double LinearizedMeanAnomalyDifference(double a, double dela, double delM_0, double dt, double mu) => delM_0 + LinearizedMeanAnomalyDriftMu(a, dela, mu) * dt;

        static double Eta(double e) => Math.Sqrt(1 - e * e);

        public static Vector OrbitalRelativePosition(ClassicalElementSet chiefElset, ClassicalElementSetMeanAnomaly deputyDifferentialElset)
        {
            double a = chiefElset.a;
            double e = chiefElset.e;
            double i = chiefElset.i;
            double f = chiefElset.nu;
            double theta = f + chiefElset.omega;

            double dela = deputyDifferentialElset.a;
            double delM = deputyDifferentialElset.M;
            double deli = deputyDifferentialElset.i;
            double delomega = deputyDifferentialElset.omega;
            double dele = deputyDifferentialElset.e;
            double delOmega = deputyDifferentialElset.Omega;

            double eta = Eta(e);
            double r = a * eta * eta / (1 + e * Math.Cos(f));
            double sinf = Math.Sin(f);
            double cosf = Math.Cos(f);

            double x = r / a * dela + a * e * sinf / eta * delM - a * cosf * dele;
            double y = r / Math.Pow(eta, 3) * Math.Pow(1 + e * cosf, 2) * delM + r * delomega + r * sinf / Math.Pow(eta, 2) * (2 + e * cosf) * dele + r * Math.Cos(i) * delOmega;
            double z = r * (Math.Sin(theta) * deli - Math.Cos(theta) * Math.Sin(i) * delOmega);

            return new DenseVector([x, y, z]);
        }

        public static ClassicalElementSetMeanAnomaly J2DifferentialElsetRate(ClassicalElementSet oe_chief, ClassicalElementSet delOe_deputy, double r_equator, double J_2, double mu)
        {
            double a = oe_chief.a;
            double e = oe_chief.e;
            double i = oe_chief.i;

            double dela = delOe_deputy.a;
            double dele = delOe_deputy.e;
            double deli = delOe_deputy.i;

            double sini = Math.Sin(i);
            double sin2i = Math.Sin(2 * i);
            double cosi = Math.Cos(i);
            double cosSquaredi = cosi * cosi;

            double eta = Eta(e);
            double epsilon = 3 * J_2 * Math.Pow(r_equator / a / (1 - e * e), 2);
            double n = TwoBodyProblem.MeanMotion(a, mu);

            double delK_Omega = 7 / 4 * cosi * dela / a - 2 * e / eta / eta * cosi * dele + 1 / 2 * sini * deli;
            double delOmegaDot = epsilon * n * delK_Omega;

            double delK_omega = -7 / 8 * (5 * cosSquaredi - 1) * dela / a + e / eta / eta * (5 * cosSquaredi - 1) * dele - 5 / 4 * sin2i * deli;
            double delomegaDot = epsilon * n * delK_omega;

            double delK_M = -7 / 8 * (3 * cosSquaredi - 1) * dela / a + 3 / 4 * e / eta * (3 * cosSquaredi - 1) * dele - 3 / 4 * eta * sin2i * deli;
            double delMDot = n + epsilon * n * delK_M;

            return new ClassicalElementSetMeanAnomaly(0, 0, 0, delOmegaDot, delomegaDot, delMDot);
        }

        public static ClassicalElementSetMeanAnomaly J2DifferentialElsetChange(ClassicalElementSet oe_chief, ClassicalElementSet delOe_deputy, double r_equator, double J_2, double mu, double dt) =>
            J2DifferentialElsetRate(oe_chief, delOe_deputy, r_equator, J_2, mu) * dt;

        public static double J2EffectDeltaDelOmega(ClassicalElementSet oe_chief, ClassicalElementSet delOe_deputy, double deltaM, double r_equator, double J_2, double mu)
        {

            double a = oe_chief.a;
            double e = oe_chief.e;
            double i = oe_chief.i;

            double dela = delOe_deputy.a;
            double dele = delOe_deputy.e;
            double deli = delOe_deputy.i;

            double epsilon = 3 * CelestialParameters.Earth.J2 * Math.Pow(CelestialParameters.Earth.Radius / (a * (1 - e * e)), 2);
            double eta = Eta(e);

            double delk_Omega = 7 / 4 * Math.Cos(i) * dela / a - 2 * e / Math.Pow(eta, 2) * Math.Cos(i) * dele + Math.Sin(i) / 2 * deli;
            return epsilon * delk_Omega * deltaM;
        }

        public static double YDotAtPeriapsisForBoundedMotion(double a, double e, double x_off, double mu)
        {
            if (e < 0 || e >= 1)
                throw new ArgumentException("Eccentricity must be in the range [0, 1) for bounded motion.");

            double n = TwoBodyProblem.MeanMotion(a, mu);
            return -x_off * n * (2 + e) / Math.Sqrt((1 + e) * Math.Pow(1 - e, 3));
        }

        static double J2L(double a, double r_equatorial) => Math.Sqrt(a / r_equatorial);
        static double J2n(double r_equatorial, double mu) => Math.Sqrt(mu / Math.Pow(r_equatorial, 3));

        public static double J2InvarientDelomegaDot(double a, double e, double i, double deli, double J_2, double r_equatorial, double mu)
        {
            double L = J2L(a, r_equatorial);
            double n = J2n(r_equatorial, mu);
            double eta = Eta(e);

            double delomegaPrime = J_2 * 3 / (4 * Math.Pow(L, 7) * Math.Pow(eta, 4)) * (Math.Tan(i) * (5 * Math.Pow(Math.Cos(i), 2) - 1) - 5 * Math.Sin(2 * i)) * deli;
            return delomegaPrime * n;
        }
        public static double J2InvarientDelomegaDot(ClassicalElementSet oe_chief, double deli, double J_2, double r_equatorial, double mu) =>
            J2InvarientDelomegaDot(oe_chief.a, oe_chief.e, oe_chief.i, deli, J_2, r_equatorial, mu);

        public static double DeltaVChangeDelomegaDelMConstantPeriapsis(double a, double e, double deltaOmega, double mu)
        {
            double n = TwoBodyProblem.MeanMotion(a, mu);
            double eta = Eta(e);
            return n * a / 4 * (1 - Math.Pow(1 + e, 2) / eta) * deltaOmega;
        }

        public static double DeltaVChangeDelomegaDelMConstantApoapsis(double a, double e, double deltaOmega, double mu)
        {
            double n = TwoBodyProblem.MeanMotion(a, mu);
            double eta = Eta(e);
            return n * a / 4 * (1 - Math.Pow(1 - e, 2) / eta) * deltaOmega;
        }
    }
}
