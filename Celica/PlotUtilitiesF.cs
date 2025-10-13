using ScottPlot;
using ScottPlot.WinForms;

namespace Celica
{
    public static class PlotUtilitiesF
    {
        public static void Plot2D(IEnumerable<StateEpochF> trajectory)
        {
            Plot2D(trajectory.Select(se => se.State));
        }

        public static void Plot2D(IEnumerable<StateF> trajectory, Plot? plot = null)
        {
            var xData = trajectory.Select(state => state.X).ToArray();
            var yData = trajectory.Select(state => state.Y).ToArray();

            plot ??= new Plot();
            plot.Add.Scatter(xData, yData);
            plot.Axes.SquareUnits();

            ShowPlot(plot);
        }

        static void ShowPlot(Plot plot)
        {
            var form = new Form
            {
                Text = "Plot",
                Width = 800,
                Height = 600
            };

            var formsPlot = new FormsPlot
            {
                Dock = DockStyle.Fill
            };
            formsPlot.Reset(plot);
            formsPlot.Refresh();

            form.Controls.Add(formsPlot);
            form.ShowDialog();
        }
    }
}
