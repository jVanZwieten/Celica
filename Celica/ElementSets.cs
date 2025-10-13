namespace Celica
{
    public struct ClassicalElementSet(double a, double e, double i, double Ω, double ω, double υ)
    {
        public double a { get; } = a;
        public double e { get; } = e;
        public double i { get; } = i;
        public double Omega { get; } = Ω;
        public double omega { get; } = ω;
        public double nu { get; } = υ;

        public ClassicalElementSet(ClassicalElementSet referenceElset, double δa = 0, double δe = 0, double δi = 0, double δΩ = 0, double δω = 0, double δυ = 0) :
            this(referenceElset.a + δa, referenceElset.e + δe, referenceElset.i + δi, referenceElset.Omega + δΩ, referenceElset.omega + δω, referenceElset.nu + δυ)
        { }

        public static ClassicalElementSet operator +(ClassicalElementSet left, ClassicalElementSet right) => new ClassicalElementSet(
                left.a + right.a,
                left.e + right.e,
                left.i + right.i,
                left.Omega + right.Omega,
                left.omega + right.omega,
                left.nu + right.nu
            );

        public static ClassicalElementSet operator /(ClassicalElementSet elset, double scalar) => new ClassicalElementSet(
                elset.a / scalar,
                elset.e / scalar,
                elset.i / scalar,
                elset.Omega / scalar,
                elset.omega / scalar,
                elset.nu / scalar
            );

        public override string ToString() => $"[{a}; {e}; {i}; {Omega}; {omega}; {nu}]";
    }

    public struct ClassicalElementSetMeanAnomaly(double a, double e, double i, double Omega, double omega, double M)
    {
        public double a { get; } = a;
        public double e { get; } = e;
        public double i { get; } = i;
        public double Omega { get; } = Omega;
        public double omega { get; } = omega;
        public double M { get; } = M;

        public static ClassicalElementSetMeanAnomaly operator +(ClassicalElementSetMeanAnomaly left, ClassicalElementSetMeanAnomaly right) => new ClassicalElementSetMeanAnomaly(
                left.a + right.a,
                left.e + right.e,
                left.i + right.i,
                left.Omega + right.Omega,
                left.omega + right.omega,
                left.M + right.M
            );
        public static ClassicalElementSetMeanAnomaly operator -(ClassicalElementSetMeanAnomaly left, ClassicalElementSetMeanAnomaly right) => new ClassicalElementSetMeanAnomaly(
                left.a - right.a,
                left.e - right.e,
                left.i - right.i,
                left.Omega - right.Omega,
                left.omega - right.omega,
                left.M - right.M
            );
        public static ClassicalElementSetMeanAnomaly operator *(ClassicalElementSetMeanAnomaly elset, double scalar) => new ClassicalElementSetMeanAnomaly(
                elset.a * scalar,
                elset.e * scalar,
                elset.i * scalar,
                elset.Omega * scalar,
                elset.omega * scalar,
                elset.M * scalar
            );
        public static ClassicalElementSetMeanAnomaly operator /(ClassicalElementSetMeanAnomaly elset, double scalar) => new ClassicalElementSetMeanAnomaly(
                elset.a / scalar,
                elset.e / scalar,
                elset.i / scalar,
                elset.Omega / scalar,
                elset.omega / scalar,
                elset.M / scalar
            );

        public override string ToString() => $"[{a}; {e}; {i}; {Omega}; {omega}; {M}]";
    }
}
