using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public static class TwoBodyProblem
    {
        public static double SemiLatusRectum(double a, double e) => a * (1 - e * e);
        public static double RConic(double e, double p, double ν) => p / (1 + e * Math.Cos(ν));
        public static double SpecificAngularMomentum(double p, double μ) => Math.Sqrt(μ * p);

        public static double CirucularVelocity(double r, double μ) => Math.Sqrt(μ / r);


        public static State StateVector(double a, double e, double i, double Ω, double ω, double ν, double μ)
        {
            double p = SemiLatusRectum(a, e);
            double r = RConic(e, p, ν);

            var rVec_perifocal = (Vector)Vector.Build.DenseOfArray([
                r * Math.Cos(ν),
                r * Math.Sin(ν),
                0
            ]);

            var vVec_perifocal = (Vector)Vector.Build.DenseOfArray([
                -Math.Sqrt(μ / p) * Math.Sin(ν),
                Math.Sqrt(μ / p) * (e + Math.Cos(ν)),
                0
            ]);

            var DCM = DirectionCosineMatrixPerifocalToInertial(Ω, i, ω);

            var rVec_inertial = (Vector)(DCM * rVec_perifocal);
            var vVec_inertial = (Vector)(DCM * vVec_perifocal);

            return new State(rVec_inertial, vVec_inertial);
        }
        public static State StateVector(ClassicalElementSet elements, double μ) =>
            StateVector(elements.a, elements.e, elements.i, elements.Omega, elements.omega, elements.nu, μ);

        public static Matrix DirectionCosineMatrixPerifocalToInertial(double Ω, double i, double ω)
        {
            double cosΩ = Math.Cos(Ω);
            double sinΩ = Math.Sin(Ω);
            double cosi = Math.Cos(i);
            double sini = Math.Sin(i);
            double cosω = Math.Cos(ω);
            double sinω = Math.Sin(ω);

            return (Matrix)Matrix.Build.DenseOfArray(new double[,]
            {
                { cosΩ * cosω - sinΩ * sinω * cosi, -cosΩ * sinω - sinΩ * cosω * cosi, sinΩ * sini },
                { sinΩ * cosω + cosΩ * sinω * cosi, -sinΩ * sinω + cosΩ * cosω * cosi, -cosΩ * sini },
                { sinω * sini,                      cosω * sini,                       cosi }
            });
        }

        public static double Period(double a, double μ) => 2 * Math.PI * Math.Sqrt(a * a * a / μ);
        public static double MeanMotion(double a, double μ) => Math.Sqrt(μ / (a * a * a));
        public static double MeanAnomaly(double M, double dt, double n) => M + n * dt;
        public static double MeanAnomaly(double M, double dt, double a, double μ) => MeanAnomaly(M, dt, MeanMotion(a, μ));
        public static double MeanFromEccentricAnomaly(double E, double e) => E - e * Math.Sin(E);
        public static double MeanFromTrueAnomaly(double ν, double e) => MeanFromEccentricAnomaly(EccentricFromTrueAnomaly(ν, e), e);

        public static double EccentricFromMeanAnomaly(double M, double e, double tolerance = 1e-10, int maxIterations = 100)
        {
            double E = M; // Initial guess
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                double f = MeanFromEccentricAnomaly(E, e) - M;
                double fPrime = 1 - e * Math.Cos(E);
                double deltaE = -f / fPrime;
                E += deltaE;
                if (Math.Abs(deltaE) < tolerance)
                    return E;
            }
            throw new Exception("Eccentric anomaly did not converge");
        }
        public static double EccentricFromTrueAnomaly(double ν, double e)
        {
            double cosE = e + Math.Cos(ν);
            double sinE = Math.Sqrt(1 - e * e) * Math.Sin(ν);
            return Math.Atan2(sinE, cosE);
        }

        public static double TrueFromEccentricAnomaly(double E, double e)
        {
            double cosν = (Math.Cos(E) - e) / (1 - e * Math.Cos(E));
            double sinν = (Math.Sqrt(1 - e * e) * Math.Sin(E)) / (1 - e * Math.Cos(E));
            return Math.Atan2(sinν, cosν);
        }

        public static double TrueFromMeanAnomaly(double M, double e, double tolerance = 1e-10, int maxIterations = 100)
        {
            double E = EccentricFromMeanAnomaly(M, e, tolerance, maxIterations);
            return Utilities.NormalizeAngleZeroToTwoPi(TrueFromEccentricAnomaly(E, e));
        }

        public static ClassicalElementSet ClassicalElementsFromState(State state, double μ)
        {
            Vector rVec = state.Position;
            double r = rVec.L2Norm();
            Vector vVec = state.Velocity;
            double v = vVec.L2Norm();

            var hVec = rVec.Cross(vVec);
            double h = hVec.L2Norm();

            var nVec = new DenseVector([0, 0, 1]).Cross(hVec);
            double n = nVec.L2Norm();

            var eVec = (Vector)((v * v - μ / r) * rVec - (rVec.DotProduct(vVec)) * vVec) / μ;
            double e = eVec.L2Norm();

            double a = 1 / (2 / r - v * v / μ);

            double i = Math.Acos(hVec[2] / h);
            double Ω = nVec[1] >= 0 ? Math.Acos(nVec[0] / n) : 2 * Math.PI - Math.Acos(nVec[0] / n);
            if (n < 1e-10) Ω = 0;
            double ω = eVec[2] >= 0 ? Math.Acos(nVec.DotProduct(eVec) / (n * e)) : 2 * Math.PI - Math.Acos(nVec.DotProduct(eVec) / (n * e));
            if (e < 1e-10) ω = 0;
            if (n < 1e-10) ω = Math.Acos(eVec[0] / e);
            if (e < 1e-10 && n < 1e-10) ω = 0;

            var ν = Math.Acos(eVec.DotProduct(rVec) / (e * r));

            return new ClassicalElementSet(a, e, i, Ω, ω, ν);
        }

    }
}
