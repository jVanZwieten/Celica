using ScottPlot;
using ScottPlot.WinForms;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Celica
{
    public static class PlotUtilities
    {
        public static void Plot2D(IEnumerable<StateEpoch> trajectory)
        {
            Plot2D(trajectory.Select(se => se.State));
        }

        public static void Plot2D(IEnumerable<State> trajectory)
        {
            var xData = trajectory.Select(state => state.X).ToArray();
            var yData = trajectory.Select(state => state.Y).ToArray();

            Plot plot = new();
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
