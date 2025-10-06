using Celica;
using MathNet.Numerics.LinearAlgebra.Double;
using Units = Celica.PhysicalUnits;

namespace ASEN6014FormationFlying
{

    internal static class HW2
    {
        const double muEarth = CelestialParameters.Earth.Mu;

        internal static void Mod2Quiz3Prob1()
        {
            Vector NrVec_c = DenseVector.OfArray([-4893.268, 3864.478, 3169.646]) * Units.km;
            Vector NvVec_c = DenseVector.OfArray([-3.91386, -6.257673, 1.59797]) * Units.km;

            Vector NrVec_d = DenseVector.OfArray([-4892.98, 3863.073, 3170.619]) * Units.km;
            Vector NvVec_d = DenseVector.OfArray([-3.913302, -6.258661, 1.598199]) * Units.km;

            State state_chief = new(NrVec_c, NvVec_c);
            State state_deputy = new(NrVec_d, NvVec_d);

            State relState_deputy = RelativeMotion.OrbitRelativeState(state_chief, state_deputy);
            Console.WriteLine($"Relative State Deputy (km, km/s):");
            Console.WriteLine(relState_deputy / Units.km);
        }

        internal static void Mod2Quiz3Prob2()
        {
            Vector NrVec_c = DenseVector.OfArray([-4893.268, 3864.478, 3169.646]) * Units.km;
            Vector NvVec_c = DenseVector.OfArray([-3.91386, -6.257673, 1.59797]) * Units.km;

            Vector OrhoVec_d = DenseVector.OfArray([-0.537, 1.221, 1.106]) * Units.km;
            Vector OrhoDotVec_d = DenseVector.OfArray([0.000486, 0.001158, 0.0005590]) * Units.km;

            State state_chief = new(NrVec_c, NvVec_c);
            State relState_deputy = new(OrhoVec_d, OrhoDotVec_d);

            State Nstate_deputy = RelativeMotion.InertialState(state_chief, relState_deputy);
            Console.WriteLine($"Inertial State Deputy (km, km/s):");
            Console.WriteLine(Nstate_deputy / Units.km);
        }
    }
}
