namespace Celica
{
    public enum Unit
    {
        Meter,
        Kilogram,
        Second,
        Newton,
        Radians
    }

    public enum Prefix
    {
        Kilo = 3,
        Hecto = 2,
        Deca = 1,
        None = 0,
        Deci = -1,
        Centi = -2,
        Milli = -3,
    }

    public struct UnitArgument
    {
        public Unit Unit { get; }
        public Prefix Magnetude { get; }
        public int Order { get; }
        public UnitArgument(Unit unit, Prefix magnetude = Prefix.None, int order = 1)
        {
            Unit = unit;
            Magnetude = magnetude;
            Order = order;
        }
        public override string ToString() => $"{Unit}{(Math.Abs(Order) > 1 ? $"^{Order}" : string.Empty)}";
    }

    public struct PhysicalQuantity
    {
        public double Value { get; }
        public Dictionary<Unit, int> Units { get; }

        public PhysicalQuantity(Unit unit, double value = 1, Prefix magnetude = Prefix.None, int order = 1) : this([new(unit, magnetude, order)], value) { }
        public PhysicalQuantity(IEnumerable<UnitArgument> units, double value = 1)
        {
            double scale = Math.Pow(10, units.Sum(u => (int)u.Magnetude * u.Order));
            Value = value * scale;
            Units = units.ToDictionary(u => u.Unit, u => u.Order);
        }

        public PhysicalQuantity(double value, Dictionary<Unit, int> units)
        {
            Value = value;
            Units = units;
        }

        public static implicit operator double(PhysicalQuantity q) => q.Value;
        public override readonly string ToString() =>
            Value + string.Join(string.Empty, Units.Select(u => u.Key + (Math.Abs(u.Value) > 1 ? $"^{u.Value}" : string.Empty)));

        public static PhysicalQuantity operator *(double a, PhysicalQuantity b) => new(a * b.Value, b.Units);
        public static PhysicalQuantity operator *(PhysicalQuantity a, double b) => b * a;
        public static PhysicalQuantity operator *(PhysicalQuantity a, PhysicalQuantity b) => new(a.Value * b.Value, MultiplyUnits(a.Units, b.Units));
        public static PhysicalQuantity operator /(double a, PhysicalQuantity b) => new(a / b.Value, b.Units.ToDictionary(u => u.Key, u => -u.Value));
        public static PhysicalQuantity operator /(PhysicalQuantity a, double b) => new(a.Value / b, a.Units);
        public static PhysicalQuantity operator /(PhysicalQuantity a, PhysicalQuantity b)
        {
            Dictionary<Unit, int> newUnits = DivideUnits(a.Units, b.Units);
            return new PhysicalQuantity(a.Value / b.Value, newUnits);
        }
        public static PhysicalQuantity operator ^(PhysicalQuantity a, int exponent)
        {
            if (exponent == 0) throw new ArgumentOutOfRangeException("Exponent cannot be zero for exponentiation.");
            if (exponent < 0) return new PhysicalQuantity(1 / a.Value, a.Units.ToDictionary(u => u.Key, u => -u.Value));
            return new PhysicalQuantity(Math.Pow(a.Value, exponent), a.Units.ToDictionary(u => u.Key, u => u.Value * exponent));
        }

        public static Dictionary<Unit, int> MultiplyUnits(Dictionary<Unit, int> a, Dictionary<Unit, int> b) => a
                .Concat(b)
                .GroupBy(u => u.Key)
                .Where(g => g.Sum(u => u.Value) != 0)
                .ToDictionary(
                    g => g.Key,
                    g => g.Sum(u => u.Value)
                );

        public static Dictionary<Unit, int> DivideUnits(Dictionary<Unit, int> a, Dictionary<Unit, int> b) => MultiplyUnits(a, b.ToDictionary(u => u.Key, u => -u.Value));

        public bool OnlyHasUnit(Unit unit) => Units.Count == 1 && Units.ContainsKey(unit) && Units[unit] == 1;

        public static PhysicalQuantity Meters => new(Unit.Meter);
        public static PhysicalQuantity m => Meters;
        public static PhysicalQuantity Kilometers => new(Unit.Meter, magnetude: Prefix.Kilo);
        public static PhysicalQuantity km => Kilometers;
        public static PhysicalQuantity Kilograms => new(Unit.Kilogram);
        public static PhysicalQuantity kg => Kilograms;

        public static PhysicalQuantity Seconds => new(Unit.Second);
        public static PhysicalQuantity s => Seconds;
        public static PhysicalQuantity Minutes => new(Unit.Second, value: 60);
        public static PhysicalQuantity min => Minutes;
        public static PhysicalQuantity Hours => 60 * Minutes;
        public static PhysicalQuantity Days => 24 * Hours;
        public static PhysicalQuantity Years => 365.25 * Days;
    }

    public struct Angle
    {
        public const double DegreesToRadians = Math.PI / 180.0;

        PhysicalQuantity value { get; }

        public double Degrees => (double)this / DegreesToRadians;

        public Angle(double radians)
        {
            value = new PhysicalQuantity(Unit.Radians, radians);
        }

        public static implicit operator double(Angle a) => a.value.Value;
        public static implicit operator Angle(double radians) => new(radians);
        public static implicit operator Angle(PhysicalQuantity q) => q.OnlyHasUnit(Unit.Radians) ? new Angle(q.Value) : throw new ArgumentException("PhysicalQuantity must be in radians to convert to Angle.");
        public static implicit operator PhysicalQuantity(Angle a) => a.value;

        public override string ToString() => value.ToString();

        public static Angle FromDegrees(double degrees) => new(degrees * DegreesToRadians);
    }
}
