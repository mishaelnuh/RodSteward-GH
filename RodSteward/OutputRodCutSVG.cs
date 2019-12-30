using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using System.IO;
using Svg;
using Grasshopper;
using Grasshopper.Kernel.Data;

namespace RodSteward
{
    public class OutputRodCutSVG : GH_Component
    {
        private const double DOC_PADDING = 50;
        private const double ROD_PADDING = 10;

        private string outputMessage = "";

        public OutputRodCutSVG()
          : base("OutputRodCutSVG", "RSOutputRodCutSVG",
              "Outputs rod lengths into rod cutting plan as SVG in [mm]",
              "RodSteward", "Output")
        {
        }

        public override Guid ComponentGuid => new Guid("038721fc-cf6f-4db6-8900-027978c7ed66");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Model", "M", "Model object", GH_ParamAccess.item);
            pManager.AddNumberParameter("Stock Length", "SL", "Length of stock rods", GH_ParamAccess.item);
            pManager.AddTextParameter("Directory", "D", "Output directory", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Write Trigger", "T", "Triggers output", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Output", "O", "Output message", GH_ParamAccess.item);
            pManager.AddNumberParameter("Cutting plan", "CP", "Cutting plan", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Model model = new Model();
            double stockLength = 0;
            string dir = null;
            bool trigger = false;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref stockLength)) { return; }
            if (!DA.GetData(2, ref dir)) { return; }
            if (!DA.GetData(3, ref trigger)) { return; }

            if (model == null) { return; }

            var binLengths = new List<double>();
            var binsTree = new DataTree<double>();

            var lengths = model.RodCentrelines.Values
                .Select(c => c.GetLength())
                .OrderByDescending(c => c)
                .ToList();

            if (lengths.First() > stockLength)
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Stock length less than longest member length.");
                return;
            }

            foreach(var len in lengths)
            {
                bool added = false;
                for(int i = 0; i < binLengths.Count(); i++)
                {
                    if (binLengths[i] + len < stockLength)
                    {
                        var path = new GH_Path(i);
                        binLengths[i] += len;
                        binsTree.Add(len, path);
                        added = true;
                        break;
                    }
                }
                if (!added)
                {
                    var path = new GH_Path(binLengths.Count());
                    binLengths.Add(len);
                    binsTree.Add(len, path);
                }
            }

            DA.SetDataTree(1, binsTree);

            if (!trigger)
            {
                DA.SetData(0, outputMessage);
                return;
            }

            if (!Directory.Exists(dir))
            {
                outputMessage = "[" + DateTime.Now.ToString() + "]: Output directory does not exists.";
                DA.SetData(0, outputMessage);
                return;
            }

            var dirFiles = Directory.GetFiles(dir);
            foreach (var file in dirFiles.Where(f => f.Contains("RS_Laser_Cut_Plan")))
                File.Delete(file);

            // Get unit scaling
            var unit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            var scale = 1.0f;
            switch (unit)
            {
                case Rhino.UnitSystem.Millimeters:
                    scale = 1.0f;
                    break;
                case Rhino.UnitSystem.Centimeters:
                    scale = 10;
                    break;
                case Rhino.UnitSystem.Meters:
                    scale = 1000;
                    break;
                case Rhino.UnitSystem.Inches:
                    scale = 25.4f;
                    break;
                case Rhino.UnitSystem.Feet:
                    scale = 304.8f;
                    break;
            }

            var svgFile = new SvgDocument()
            {
                Width = new SvgUnit(SvgUnitType.None, (float)(stockLength * scale + DOC_PADDING * 2)),
                Height = new SvgUnit(SvgUnitType.None, (float)(model.Radius * 2 * binLengths.Count() * scale + ROD_PADDING * (binLengths.Count() - 1) + DOC_PADDING * 2)),
            };
            svgFile.ViewBox = new SvgViewBox(0, 0, svgFile.Width, svgFile.Height);

            double offset_y = DOC_PADDING;
            foreach (var b in binsTree.Branches)
            {
                double offset_x = DOC_PADDING;
                svgFile.Children.Add(new SvgLine()
                {
                    StartX = (float)(offset_x),
                    EndX = (float)(offset_x),
                    StartY = (float)(offset_y),
                    EndY = (float)(offset_y + model.Radius * 2 * scale),
                    Stroke = new SvgColourServer(System.Drawing.Color.Red),
                    StrokeWidth = 2,
                });
                foreach (var l in b)
                {
                    offset_x += (float)l * scale;
                    svgFile.Children.Add(new SvgLine()
                    {
                        StartX = (float)(offset_x),
                        EndX = (float)(offset_x),
                        StartY = (float)(offset_y),
                        EndY = (float)(offset_y + model.Radius * 2 * scale),
                        Stroke = new SvgColourServer(System.Drawing.Color.Red),
                        StrokeWidth = 2,
                    });
                }
                offset_y += model.Radius * 2 * scale + ROD_PADDING;
            }

            using (FileStream fs = File.Create(Path.Combine(dir, "RS_Laser_Cut_Plan.svg")))
            {
                svgFile.Write(fs);
            }

            outputMessage = "[" + DateTime.Now.ToString() + "]: Successfully wrote SVG laser cutting plan.";
            DA.SetData(0, outputMessage);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.svg;
            }
        }
    }
}
