namespace Celica
{
    public class ClassicElements(double a, double e, double i, double Ω, double ω, double υ)
    {
        public double a { get; } = a;
        public double e { get; } = e;
        public double i { get; } = i;
        public double Ω { get; } = Ω;
        public double ω { get; } = ω;
        public double υ { get; } = υ;
        public override string ToString() => $"[{a}; {e}; {i}; {Ω}; {ω}; {υ}]";
    }
}
