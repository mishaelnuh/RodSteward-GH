﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino.Geometry;
using MIConvexHull;

namespace RodSteward
{
    public class Mesh2Graph : GH_Component
    {
        const int CIRCLE_SIDES = 100;
        public bool FastRender { get; set; } = true;

        private Dictionary<Tuple<int, int>, Brep> rodBreps = new Dictionary<Tuple<int, int>, Brep>();
        private Dictionary<Tuple<int, int>, Curve> rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();
        private Dictionary<int, List<Brep>> jointBreps = new Dictionary<int, List<Brep>>();
        private List<Tuple<int, int>> clashedRods = new List<Tuple<int, int>>();
        private List<int> clashedJoints = new List<int>();

        public Mesh2Graph()
          : base("Mesh2Graph", "RSMesh2Graph",
              "Converts mesh to list of edges and vertices",
              "RodSteward", "RodSteward")
        {
        }

        public override Guid ComponentGuid => new Guid("63ef7f07-25a6-4422-b161-df67ad5f72b8");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Base mesh for structure", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Edges", "E", "Connectivity", GH_ParamAccess.list);
            pManager.AddPointParameter("Vertices", "V", "Rod meshes for structure", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh data = null;
            
            if (!DA.GetData(0, ref data)) { return; }

            if (data == null) { return; }

            data.Vertices.CombineIdentical(true, true);

            var edges = GetMeshEdges(data);
            var vertices = data.Vertices;

            DA.SetDataList(0, edges);
            DA.SetDataList(1, vertices);
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
                return Properties.Resources.generator;
            }
        }
    }
}
