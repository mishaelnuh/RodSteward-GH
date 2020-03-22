using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using MathNet.Numerics.LinearAlgebra;
using System.Linq;

namespace RodSteward
{
    public class StructuralAnalysis : GH_Component
    {
        const double BIG_NUM = 10e9;

        private Model model = null;
        private List<double> utilization = null;

        public StructuralAnalysis()
          : base("StructuralAnalysis", "RSStructuralAnalysis",
              "Performs matrix frame analysis on the structure",
              "RodSteward", "Analysis")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Model", "M", "Model object", GH_ParamAccess.item);
            pManager.AddNumberParameter("Fu", "Fu", "Max allowable stress [MPa]", GH_ParamAccess.item);
            pManager.AddNumberParameter("E", "E", "Young's modulus of material [MPa]", GH_ParamAccess.item);
            pManager.AddNumberParameter("A", "A", "Cross-sectional area [mm^2]", GH_ParamAccess.item);
            pManager.AddNumberParameter("Iy", "Iy", "Strong axis second moment of area [mm^4]", GH_ParamAccess.item);
            pManager.AddNumberParameter("Iz", "Iz", "Weak axis second moment of area [mm^4]", GH_ParamAccess.item);
            pManager.AddNumberParameter("Sy", "Sy", "Section modulus in y axis [mm^3]", GH_ParamAccess.item);
            pManager.AddNumberParameter("Sz", "Sz", "Sectoin modulus in z axis [mm^3]", GH_ParamAccess.item);
            pManager.AddNumberParameter("Restraints", "R", "List of joints to be restrained", GH_ParamAccess.list);
            pManager.AddVectorParameter("Loads", "L", "Vector list of loads to be applied [N]", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Fmin", "Fmin", "Minimum stress [mPa]", GH_ParamAccess.list);
            pManager.AddNumberParameter("Fmax", "Fmax", "Maximum stress [mPa]", GH_ParamAccess.list);
            pManager.AddNumberParameter("Util", "U", "Utilization ratio", GH_ParamAccess.list);
        }
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (model == null)
                return;

            for(int i = 0; i < utilization.Count; i++)
            {
                var errorMaterial = new Rhino.Display.DisplayMaterial(
                    System.Drawing.Color.FromArgb(0,
                        255,
                        (int)Math.Max(255 * (1 - utilization[i]), 0),
                        (int)Math.Max(255 * (1 - utilization[i]), 0)),
                    0.3);
                var mesh = model.RodMeshes[model.Edges[i]];
                args.Display.DrawMeshShaded(mesh, errorMaterial);
            }
        }
        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (model == null)
                return;

            for (int i = 0; i < utilization.Count; i++)
            {
                var k = model.Edges[i];
                var l = model.RodCentrelines[k];
                var midPoint = l.PointAtNormalizedLength(0.5);
                var utilString = String.Format("{0:F2}", utilization[i]);

                args.Display.Draw2dText(utilString,
                    System.Drawing.Color.FromArgb(0,
                        255,
                        (int)Math.Max(255 * (1 - utilization[i]), 0),
                        (int)Math.Max(255 * (1 - utilization[i]), 0)),
                    midPoint, false, 20, "Lucida Console");
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Model model = new Model();
            double Fu = 0;
            double E = 0;
            double A = 0;
            double Iy = 0;
            double Iz = 0;
            double Sy = 0;
            double Sz = 0;
            List<double> restraints = new List<double>();
            List<Vector3d> loads = new List<Vector3d>();

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref Fu)) { return; }
            if (!DA.GetData(2, ref E)) { return; }
            if (!DA.GetData(3, ref A)) { return; }
            if (!DA.GetData(4, ref Iy)) { return; }
            if (!DA.GetData(5, ref Iz)) { return; }
            if (!DA.GetData(6, ref Sy)) { return; }
            if (!DA.GetData(7, ref Sz)) { return; }
            if (!DA.GetDataList(8, restraints)) { return; }
            if (!DA.GetDataList(9, loads)) { return; }

            var K = model.StiffnessMatrix(E, A, Iy, Iz);
            var K_PI = Matrix<double>.Build.DenseOfMatrix(K);

            var F = Matrix<double>.Build.Dense(model.Vertices.Count * 5, 1);

            for (int i = 0; i < model.Vertices.Count; i++)
            {
                if (restraints.Contains(i))
                {
                    F[i * 5, 0] = 0;
                    F[i * 5 + 1, 0] = 0;
                    F[i * 5 + 2, 0] = 0;

                    K_PI[i * 5, i * 5] = K[i * 5, i * 5] * BIG_NUM;
                    K_PI[i * 5 + 1, i * 5 + 1] = K[i * 5, i * 5] * BIG_NUM;
                    K_PI[i * 5 + 2, i * 5 + 2] = K[i * 5, i * 5] * BIG_NUM;
                }
                else
                {
                    F[i * 5, 0] = loads[i].X;
                    F[i * 5 + 1, 0] = loads[i].Y;
                    F[i * 5 + 2, 0] = loads[i].Z;
                }
            }

            var U = K_PI.Solve(F);

            for (int i = 0; i < model.Vertices.Count; i++)
            {
                if (restraints.Contains(i))
                {
                    U[i * 5, 0] = 0;
                    U[i * 5 + 1, 0] = 0;
                    U[i * 5 + 2, 0] = 0;
                }
            }

            var memberStresses = model.MaxMemberStress(U, E, A, Iy, Iz, Sy, Sz);

            this.model = model;

            utilization = memberStresses.Select((s, i) => {
                var smax = Math.Max(Math.Abs(s.Item1), Math.Abs(s.Item2));
                var bucklingStrength = Math.PI * Math.PI * E * Math.Min(Iy, Iz) / (Math.Pow(model.MemberLengthInMM(i), 2) * A);
                if (s.Item1 < 0)
                    return smax / Math.Min(Fu, bucklingStrength);
                else
                    return smax / Fu;
            }).ToList();

            DA.SetDataList(0, memberStresses.Select(s => s.Item1).ToList());
            DA.SetDataList(1, memberStresses.Select(s => s.Item2).ToList());
            DA.SetDataList(2, utilization);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.structanalysis;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("f7705f70-a00e-4b46-bedf-e05ad0a3e2ab"); }
        }   
    }
}