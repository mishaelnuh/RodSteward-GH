using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace RodSteward
{
    public class Lines2Graph : GH_Component
    {
        public Lines2Graph()
          : base("Lines2Graph", "RSLines2Graph",
              "Converts lines to list of edges and vertices",
              "RodSteward", "Convert")
        {
        }

        public override Guid ComponentGuid => new Guid("9fe58242-b9b0-4a7b-b97f-8013ebea0ab6");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Lines", "L", "Lines for structure", GH_ParamAccess.list);
            pManager.AddNumberParameter("Tolerance", "e", "Tolerance for merging points", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Edges", "E", "Connectivity", GH_ParamAccess.list);
            pManager.AddPointParameter("Vertices", "V", "Rod meshes for structure", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> data = new List<Line>();
            double tolerance = 0;

            if (!DA.GetDataList(0, data)) { return; }
            if (!DA.GetData(1, ref tolerance)) { return; }

            if (data == null) { return; }

            List<Point3d> vertices = new List<Point3d>();
            List<Tuple<int, int>> edges = new List<Tuple<int, int>>();

            foreach (var l in data)
            {
                var p1 = l.PointAt(0);
                var p2 = l.PointAt(1);

                int match1 = vertices.FindIndex(p => p.DistanceTo(p1) < tolerance);
                int match2 = vertices.FindIndex(p => p.DistanceTo(p2) < tolerance);

                if (match1 < 0)
                {
                    vertices.Add(p1);
                    match1 = vertices.Count() - 1;
                }
                if (match2 < 0)
                {
                    vertices.Add(p2);
                    match2 = vertices.Count() - 1;
                }

                edges.Add(Tuple.Create(match1, match2));
            }

            // Prune
            List<List<Tuple<int, int>>> filteredEdges = new List<List<Tuple<int, int>>>();

            var visitedEdges = new List<Tuple<int, int>>();
            var startingIndex = 0;

            while (visitedEdges.Count < edges.Count)
            {
                filteredEdges.Add(ConnectedEdges(edges.Distinct().ToList(), vertices, startingIndex));
                visitedEdges.AddRange(filteredEdges[filteredEdges.Count - 1]);
                try
                {
                    startingIndex = edges.First(e => !visitedEdges.Contains(e)).Item1;
                }
                catch { break; }
            }

            DA.SetDataList(0, filteredEdges.OrderByDescending(i => i.Count).First().Distinct().ToList());
            DA.SetDataList(1, vertices);
        }

        private List<Tuple<int, int>> ConnectedEdges(List<Tuple<int,int>> edges, List<Point3d> vertices, int vIndex, List<Tuple<int,int>> searchedEdges = null)
        {
            if (searchedEdges == null)
                searchedEdges = new List<Tuple<int, int>>();

            var connectedEdges = edges.Where(v => v.Item1 == vIndex || v.Item2 == vIndex).ToList();

            foreach(var c in connectedEdges)
            {
                if (!searchedEdges.Contains(c))
                {
                    searchedEdges.Add(c);
                    ConnectedEdges(edges, vertices, c.Item1 == vIndex ? c.Item2 : c.Item1, searchedEdges);
                }
            }

            return searchedEdges;
        }

        private List<Tuple<int, int>> GetMeshEdges(Mesh input)
        {
            var ngons = input.GetNgonAndFacesEnumerable();

            var edges = new List<Tuple<int, int>>();

            foreach (MeshNgon n in ngons)
            {
                var vIdx = n.BoundaryVertexIndexList();

                if (vIdx.Length < 3) { continue; }

                for (int i = 0; i < vIdx.Length; i++)
                {
                    var nextId = i == vIdx.Length - 1 ? 0 : i + 1;

                    if (vIdx[i] < vIdx[nextId])
                        edges.Add(Tuple.Create((int)vIdx[i], (int)vIdx[nextId]));
                    else if (vIdx[i] > vIdx[nextId])
                        edges.Add(Tuple.Create((int)vIdx[nextId], (int)vIdx[i]));
                    else
                        continue;
                }
            }
            
            return edges.Distinct().ToList();
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.lines;
            }
        }
    }
}
