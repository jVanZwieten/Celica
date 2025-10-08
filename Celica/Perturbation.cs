namespace Celica
{
    public static class Perturbation
    {
        public static double J2MeanRaanRate(double i, double n, double p, double J_2, double r_equator) => -1.5 * J_2 * n * Math.Pow(r_equator / p, 2) * Math.Cos(i);
        public static double J2MeanRaanRate(double a, double e, double i, double mu, double J_2, double r_equator) => J2MeanRaanRate(i, TwoBodyProblem.MeanMotion(a, mu), TwoBodyProblem.SemiLatusRectum(a, e), J_2, r_equator);
        public static double J2MeanRaanRate(ClassicalElementSet elements, double mu, double J_2, double r_equator) => J2MeanRaanRate(elements.a, elements.e, elements.i, mu, J_2, r_equator);
    }
}
