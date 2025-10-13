using Celica;
using MathNet.Numerics.LinearAlgebra.Double;

namespace BeeSwarm
{
    internal static class Observation
    {
        internal static double ObservationScore(Vector OrhoVec_deputy, Vector OrhoHat_sun)
        {
            Vector OrhoHat_deputy = (Vector)OrhoVec_deputy.Normalize(2);
            return OrhoHat_deputy.DotProduct(OrhoHat_sun);
        }

        internal static double[][] ObservationScores(StateEpoch[][] deputyTrajectories, Vector[] OrHats_sun)
        {
            int n_deputies = deputyTrajectories.Length;
            int n_epochs = deputyTrajectories[0].Length;
            double[][] scores = new double[n_deputies][];

            for (int i_deputy = 0; i_deputy < n_deputies; i_deputy++)
            {
                scores[i_deputy] = new double[n_epochs];
                var deputyTrajectory = deputyTrajectories[i_deputy];
                for (int i_epoch = 0; i_epoch < n_epochs; i_epoch++)
                {
                    scores[i_deputy][i_epoch] = ObservationScore(deputyTrajectory[i_epoch].State.Position, OrHats_sun[i_epoch]);
                }
            }

            return scores;
        }
    }
}
