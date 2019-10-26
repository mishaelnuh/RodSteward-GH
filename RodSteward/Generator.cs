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
            pManager.AddNumberParameter("Joint Thickness", "JT", "Thickness of Joint", GH_ParamAccess.item);
            pManager.AddNumberParameter("Joint Length", "JL", "Thickness of Joint", GH_ParamAccess.item);
            pManager.AddNumberParameter("Tolerance", "e", "Tolerance", GH_ParamAccess.item);
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
            double jointThickness = 0;
            double jointLength = 0;
            double tolerance = 0;
            
            if (!DA.GetData(0, ref data)) { return; }
            if (!DA.GetData(1, ref sides)) { return; }
            if (!DA.GetData(2, ref radius)) { return; }
            if (!DA.GetData(3, ref jointThickness)) { return; }
            if (!DA.GetData(4, ref jointLength)) { return; }
            if (!DA.GetData(5, ref tolerance)) { return; }

            if (data == null) { return; }
            if (radius <= 0 || sides < 0 || jointThickness < 0 || jointLength < 0 || tolerance < 0) { return; }

            data.Vertices.CombineIdentical(true, true);

            var edges = GetMeshEdges(data);
            var vertices = data.Vertices;

            var offsets = GetRodOffsets(data, radius, tolerance);
            var rods = GetRodMeshes(edges, vertices, radius, (int)Math.Floor(sides));

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

        private Dictionary<string, double> GetRodOffsets(Mesh input, double radius, double tolerance)
        {
            var offsets = new Dictionary<string, double>();

            var topo = input.TopologyVertices;
            var vertices = input.Vertices;

            for (int vStart = 0; vStart < vertices.Count; vStart++)
            {
                var connectedVertices = topo.ConnectedTopologyVertices(vStart);
                for (int vEnd = 0; vEnd < connectedVertices.Length; vEnd++)
                {
                    for (int vCompare = vEnd + 1; vCompare < connectedVertices.Length; vCompare++)
                    {
                        var key1 = vStart.ToString() + " " + connectedVertices[vEnd].ToString();
                        var key2 = vStart.ToString() + " " + connectedVertices[vCompare].ToString();

                        var vec1 = vertices[connectedVertices[vEnd]] - vertices[vStart];
                        var vec2 = vertices[connectedVertices[vCompare]] - vertices[vStart];
                        vec1.Unitize();
                        vec2.Unitize();

                        var angle = Math.Acos(vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z);

                        double offset = 0;
                        try
                        {
                            offset = (radius + tolerance) / Math.Tan(angle / 2);
                        }
                        catch
                        {
                            offset = tolerance * 2;
                        }

                        if (offsets.ContainsKey(key1))
                            offsets[key1] = Math.Max(offset, offsets[key1]);
                        else
                            offsets[key1] = offset;

                        if (offsets.ContainsKey(key2))
                            offsets[key2] = Math.Max(offset, offsets[key2]);
                        else
                            offsets[key2] = offset;
                    }
                }
            }

            return offsets;
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
