using MathNet.Numerics.LinearAlgebra.Double;

namespace Celica
{
    public struct StateEpoch(State state, double epoch)
    {
        public double Epoch { get; } = epoch;
        public State State { get; } = state;
        public double X => State.X;
        public double Y => State.Y;
        public double Z => State.Z;
        public double V_x => State.V_x;
        public double V_y => State.V_y;
        public double V_z => State.V_z;
    }

    public struct State(Vector position, Vector velocity)
    {
        public Vector Position { get; } = position;
        public double X => Position[0];
        public double Y => Position[1];
        public double Z => Position[2];
        public Vector Velocity { get; } = velocity;
        public double V_x => Velocity[0];
        public double V_y => Velocity[1];
        public double V_z => Velocity[2];
        public State(double x, double y, double z, double v_x, double v_y, double v_z) : this(new DenseVector([x, y, z]), new DenseVector([v_x, v_y, v_z])) { }
        public static State operator +(State a, State b) => new State((Vector)(a.Position + b.Position), (Vector)(a.Velocity + b.Velocity));
        public static State operator *(double scalar, State a) => new State((Vector)(scalar * a.Position), (Vector)(scalar * a.Velocity));
        public static State operator *(State a, double scalar) => scalar * a;
        public override string ToString() => $"[{X}; {Y}; {Z}; {V_x}; {V_y}; {V_z}]";
    }
}
