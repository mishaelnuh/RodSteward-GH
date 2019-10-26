using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace RodSteward
{
    public class Generator : GH_Component
    {
        public Generator()
          : base("Generator", "RSGenerator",
              "Generate rod lengths and 3D printed joint meshes",
              "RodSteward", "RodSteward")
        {
        }

        public override Guid ComponentGuid => new Guid("0ab517a3-7f77-4c94-b004-2a8ab75cc685");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Base mesh for structure", GH_ParamAccess.item);
            pManager.AddNumberParameter("Sides", "S", "Number of faces for rods", GH_ParamAccess.item);
            pManager.AddNumberParameter("Radius", "R", "Radius of rods", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Joints", "J", "Joint meshes for structure", GH_ParamAccess.list);
            pManager.AddMeshParameter("Rod", "R", "Rod lengths for structure", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh data = null;
            double sides = 0;
            double radius = 0;
            
            if (!DA.GetData(0, ref data)) { return; }
            if (!DA.GetData(1, ref sides)) { return; }
            if (!DA.GetData(2, ref radius)) { return; }

            if (data == null) { return; }
            if (radius <= 0) { return; }

            int sidesFloored = (int)Math.Floor(sides);

            data.Vertices.CombineIdentical(true, true);

            var edges = GetMeshEdges(data);
            var vertices = data.Vertices;

            var rods = GetRodMeshes(edges, vertices, radius, sidesFloored);

            DA.SetDataList(1, rods);
        }

        private List<Tuple<uint, uint>> GetMeshEdges(Mesh input)
        {
            var ngons = input.GetNgonAndFacesEnumerable();

            var edges = new List<Tuple<uint, uint>>();

            foreach (MeshNgon n in ngons)
            {
                var vIdx = n.BoundaryVertexIndexList();

                if (vIdx.Length < 3) { continue; }

                for (int i = 0; i < vIdx.Length; i++)
                {
                    var nextId = i == vIdx.Length - 1 ? 0 : i + 1;

                    if (vIdx[i] < vIdx[nextId])
                        edges.Add(new Tuple<uint, uint>(vIdx[i], vIdx[nextId]));
                    else if (vIdx[i] > vIdx[nextId])
                        edges.Add(new Tuple<uint, uint>(vIdx[nextId], vIdx[i]));
                    else
                        continue;
                }
            }
            
            return edges.Distinct().ToList();
        }

        private List<Mesh> GetRodMeshes(List<Tuple<uint, uint>> edges, Rhino.Geometry.Collections.MeshVertexList vertices, double radius = 1, int sides = 0)
        {
            var rods = new List<Mesh>();

            foreach (Tuple<uint, uint> e in edges)
            {
                var c = new LineCurve(vertices[(int)e.Item1], vertices[(int)e.Item2]);
                rods.Add(Mesh.CreateFromCurvePipe(c, radius, sides == 0 ? 1 : sides, 1, MeshPipeCapStyle.Box, sides > 0, null));
            }

            return rods;
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }
    }
}
