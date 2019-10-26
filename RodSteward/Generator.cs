﻿using System;
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
        const int CIRCLE_SIDES = 100;

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
            pManager.AddMeshParameter("Rods", "R", "Rod meshes for structure", GH_ParamAccess.list);
            pManager.AddCurveParameter("Rod Curves", "RC", "Rod centreline curves", GH_ParamAccess.list);
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
            if (radius <= 0 || sides < 0 || jointThickness < 0 || jointLength < 0 || tolerance < 0) { throw new Exception("Invalid input."); }
            if (sides == 1 || sides == 2) { throw new Exception("Invalid number of sides."); }

            data.Vertices.CombineIdentical(true, true);

            var edges = GetMeshEdges(data);
            var vertices = data.Vertices;

            var offsets = GetRodOffsets(data, radius, tolerance);
            var rods = GetRodMeshes(edges, vertices, offsets, radius, (int)Math.Floor(sides));
            var joints = GetJointMeshes(rods.Item1, edges, vertices, offsets, radius, (int)Math.Floor(sides), jointLength, jointThickness, tolerance);

            ColourMeshCollision(rods.Item1);

            DA.SetDataList(0, joints.Values.ToList());
            DA.SetDataList(1, rods.Item1.Values.ToList());
            DA.SetDataList(2, rods.Item2.Values.ToList());
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

        private Dictionary<Tuple<int, int>, double> GetRodOffsets(Mesh input, double radius, double tolerance)
        {
            var offsets = new Dictionary<Tuple<int, int>, double>();

            var topo = input.TopologyVertices;
            var vertices = input.Vertices;

            for (int vStart = 0; vStart < vertices.Count; vStart++)
            {
                var connectedVertices = topo.ConnectedTopologyVertices(vStart);
                for (int vEnd = 0; vEnd < connectedVertices.Length; vEnd++)
                {
                    for (int vCompare = vEnd + 1; vCompare < connectedVertices.Length; vCompare++)
                    {
                        var key1 = Tuple.Create(vStart, connectedVertices[vEnd]);
                        var key2 = Tuple.Create(vStart, connectedVertices[vCompare]);

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

        private Tuple<Dictionary<Tuple<int, int>, Mesh>, Dictionary<Tuple<int, int>, Curve>> GetRodMeshes(List<Tuple<int, int>> edges,
            Rhino.Geometry.Collections.MeshVertexList vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides)
        {
            var rodMeshes = new Dictionary<Tuple<int, int>, Mesh>();
            var rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();

            foreach (Tuple<int, int> e in edges)
            {
                Curve c = new LineCurve(vertices[e.Item1], vertices[e.Item2]);

                try
                {
                    c = c.Trim(CurveEnd.Start, offsets[e])
                        .Trim(CurveEnd.End, offsets[Tuple.Create(e.Item2, e.Item1)]);
                }
                catch
                {
                    throw new Exception("Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                }

                if (c == null)
                    throw new Exception("Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                
                rodCentrelines[e] = c;
                rodMeshes[e] = Mesh.CreateFromCurvePipe(c, radius, sides == 0 ? CIRCLE_SIDES : sides, 1, MeshPipeCapStyle.Flat, true, null);
            }

            return Tuple.Create(rodMeshes, rodCentrelines);
        }

        private Dictionary<int, Mesh> GetJointMeshes(Dictionary<Tuple<int, int>, Mesh> rodMeshes, List<Tuple<int, int>> edges,
            Rhino.Geometry.Collections.MeshVertexList vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides,
            double jointLength, double jointThickness, double tolerance)
        {
            var jointMeshes = new Dictionary<int, Mesh>();
            var separatedJointMeshes = new Dictionary<int, List<Mesh>>();

            double jointRadius = radius + jointThickness + tolerance;

            // Add base sphere
            for (int v = 0; v < vertices.Count; v++)
            {
                separatedJointMeshes[v] = new List<Mesh>()
                {
                    //Mesh.CreateQuadSphere(new Sphere(vertices[v], jointRadius), 2)
                };
            }

            foreach(Tuple<int, int> e in edges)
            {
                Curve c = new LineCurve(vertices[e.Item1], vertices[e.Item2]);
                double len = c.GetLength();

                Curve startCurve = c.Trim(CurveEnd.End, len - (offsets[e] + jointLength));
                Curve endCurve = c.Trim(CurveEnd.Start, len - (offsets[Tuple.Create(e.Item2, e.Item1)] + jointLength));

                //if (offsets[e] + offsets[Tuple.Create(e.Item2, e.Item1)] + 2 * jointLength > len)
                //{
                //    throw new Exception("Joint lengths overlapping. Try reducing joint lengths or radius.");
                //}

                Mesh startMesh = Mesh.CreateFromCurvePipe(startCurve, jointRadius, sides == 0 ? CIRCLE_SIDES : sides, 1, MeshPipeCapStyle.Flat, true, null);
                Mesh endMesh = Mesh.CreateFromCurvePipe(endCurve, jointRadius, sides == 0 ? CIRCLE_SIDES : sides, 1, MeshPipeCapStyle.Flat, true, null);
                separatedJointMeshes[e.Item1].Add(startMesh);
                separatedJointMeshes[e.Item2].Add(endMesh);

                //separatedJointMeshes[e.Item1].AddRange(Mesh.CreateBooleanDifference(new Mesh[] { startMesh }, new Mesh[] { rodMeshes[e] }));
                //separatedJointMeshes[e.Item2].AddRange(Mesh.CreateBooleanDifference(new Mesh[] { endMesh }, new Mesh[] { rodMeshes[e] }));
            }

            foreach(KeyValuePair<int, List<Mesh>> kvp in separatedJointMeshes)
            {
                var meshes = Mesh.CreateBooleanUnion(kvp.Value);
                //meshes = Mesh.CreateBooleanDifference(meshes,
                //    rodMeshes.Where(m => m.Key.Item1 == kvp.Key || m.Key.Item2 == kvp.Key).Select(m => m.Value));
                jointMeshes[kvp.Key] = meshes.First();
            }

            return jointMeshes;
        }

        private void ColourMeshCollision(Dictionary<Tuple<int, int>, Mesh> rodMeshes)
        {
            List<Tuple<int, int>> clashed = new List<Tuple<int, int>>();

            var keys = rodMeshes.Keys.ToList();
            var vals = rodMeshes.Values.ToList();
            
            for (int i = 0; i < rodMeshes.Values.Count; i++)
            {
                for (int j = i + 1; j < rodMeshes.Values.Count; j++)
                {
                    if (Rhino.Geometry.Intersect.Intersection.MeshMeshFast(vals[i], vals[j]).Length > 0)
                    {
                        clashed.Add(keys[i]);
                        clashed.Add(keys[j]);
                    }
                }
            }

            foreach (var kvp in rodMeshes)
            {
                if (clashed.Contains(kvp.Key))
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(100, 255, 0, 0), kvp.Value.Vertices.Count).ToArray());
                else
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(100, 255, 255, 255), kvp.Value.Vertices.Count).ToArray());
            }
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
