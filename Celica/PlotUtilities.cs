using ScottPlot;
using ScottPlot.WinForms;

namespace Celica
{
    public static class PlotUtilities
    {
        static ScottPlot.Color startMarkerColor = ScottPlot.Color.FromColor(System.Drawing.Color.Red);

        static Func<State, int, bool> SampledIndex(int sampleInterval, int length) => (State state, int index) => index % sampleInterval == 0 || index == length - 1;

        public static void Plot2D(IEnumerable<IEnumerable<State>> trajectories, Plot? plot = null, int? sampleInterval = null, string? title = null)
        {
            plot ??= new Plot();
            plot.Axes.SquareUnits();
            plot.Add.Palette = new ScottPlot.Palettes.Nord();

            int i_series = 0;
            foreach (var trajectory in trajectories)
            {
                var sampledTrajectory = sampleInterval.HasValue ? trajectory.Where(SampledIndex(sampleInterval.Value, trajectory.Count())) : trajectory;
                var xData = sampledTrajectory.Select(state => state.X).ToArray();
                var yData = sampledTrajectory.Select(state => state.Y).ToArray();
                var series = plot.Add.Scatter(xData, yData);
                series.LegendText = $"Trajectory {i_series++}";
                series.MarkerStyle = MarkerStyle.None;
                series.LineWidth = 2;

                plot.Add.Marker(sampledTrajectory.First().X, sampledTrajectory.First().Y, color: startMarkerColor);
            }
            plot.ShowLegend(Alignment.MiddleRight);

            if (title is not null)
            {
                plot.Title(title);
            }

            ShowPlot(plot);
        }
        public static void Plot2D(IEnumerable<IEnumerable<StateEpoch>> trajectories, Plot? plot = null, int? sampleInterval = null, string? title = null)
        {
            Plot2D(trajectories.Select(traj => traj.Select(se => se.State)), plot, sampleInterval, title);
        }
        public static void Plot2D(IEnumerable<State> trajectory, Plot? plot = null, int? sampleInterval = null, string? title = null)
        {
            Plot2D([trajectory], plot, sampleInterval, title);
        }
        public static void Plot2D(IEnumerable<StateEpoch> trajectory, Plot? plot = null, int? sampleInterval = null, string? title = null)
        {
            Plot2D(trajectory.Select(se => se.State), plot, sampleInterval, title);
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
            form.Show();
        }

        public static void Plot2DTimeSeries(IEnumerable<IEnumerable<double>> series, double dt, string? title = null, string? ylabel = null, string[]? seriesNames = null)
        {
            var plot = new Plot();
            plot.Add.Palette = new ScottPlot.Palettes.Nord();
            plot.Title(title);
            plot.XLabel("Time (s)");
            var xData = Enumerable.Range(0, series.First().Count()).Select(i => i * dt).ToArray();

            foreach (var s in series)
            {
                var scatter = plot.Add.Scatter(xData, s.ToArray());
                if (seriesNames is not null)
                    scatter.LegendText = seriesNames[plot.GetPlottables().Count() - 1];
                scatter.MarkerStyle = MarkerStyle.None;
                scatter.LineWidth = 2;
            }
            plot.YLabel(ylabel ?? "");

            ShowPlot(plot);
        }
    }
}
