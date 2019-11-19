using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino.Geometry;
using System.IO;
using IxMilia.Stl;
using Grasshopper;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace RodSteward
{
    public class OutputRodBinPacked : GH_Component
    {
        private string outputMessage = "";
        public OutputRodBinPacked()
          : base("OutputRodBinPacked", "RSOutputRodBinPacked",
              "Outputs rod lengths into rod cutting plan",
              "RodSteward", "Output")
        {
        }

        public override Guid ComponentGuid => new Guid("038721fc-cf6f-4db6-8900-027978c7ed66");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Rod Curves", "RC", "Joint meshes from Generator component", GH_ParamAccess.list);
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
            List<Curve> data = new List<Curve>();
            double stockLength = 0;
            string dir = null;
            bool trigger = false;

            if (!DA.GetDataList(0, data)) { return; }
            if (!DA.GetData(1, ref stockLength)) { return; }
            if (!DA.GetData(2, ref dir)) { return; }
            if (!DA.GetData(3, ref trigger)) { return; }

            if (data == null) { return; }
            if (data.Count == 0) { return; }

            var binLengths = new List<double>();
            var binsTree = new DataTree<double>();

            data = data.OrderByDescending(c => c.GetLength()).ToList();

            foreach(var c in data)
            {
                var len = c.GetLength();
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

            outputMessage = "[" + DateTime.Now.ToString() + "]: Successfully wrote DXF laser cutting plan.";
            DA.SetData(0, outputMessage);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.stl;
            }
        }
    }
}
