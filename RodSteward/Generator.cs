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
            pManager.AddBrepParameter("Joints", "J", "Joint meshes for structure", GH_ParamAccess.list);
            pManager.AddBrepParameter("Rods", "R", "Rod meshes for structure", GH_ParamAccess.list);
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

            var rods = GetRodBreps(edges, vertices, offsets, radius, (int)Math.Floor(sides));
            var joints = GetJointBreps(rods.Item1, edges, vertices, offsets, radius, (int)Math.Floor(sides), jointLength, jointThickness, tolerance);

            //ColourMeshCollision(rods.Item1, joints);

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

        private Curve GetProfile(double radius, int sides, Vector3d normal)
        {
            var profilePlane = new Plane(new Point3d(0, 0, 0), normal);
            var circle = new Circle(profilePlane, radius);
            return Polyline.CreateInscribedPolygon(circle, sides).ToPolylineCurve();
        }
        private Tuple<Dictionary<Tuple<int, int>, Brep>, Dictionary<Tuple<int, int>, Curve>> GetRodBreps(List<Tuple<int, int>> edges,
            Rhino.Geometry.Collections.MeshVertexList vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides)
        {
            var rodBreps = new Dictionary<Tuple<int, int>, Brep>();
            var rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();

            foreach (Tuple<int, int> e in edges)
            {
                var vector = Point3d.Subtract(vertices[e.Item2], vertices[e.Item1]);
                Curve c = new LineCurve(new Point3d(0, 0, 0), Point3d.Add(new Point3d(0, 0, 0), vector));
                vector.Unitize();

                Curve centreline = new LineCurve(vertices[e.Item1], vertices[e.Item2]);
                try
                {
                    centreline = centreline.Trim(CurveEnd.Start, offsets[e])
                        .Trim(CurveEnd.End, offsets[Tuple.Create(e.Item2, e.Item1)]);
                    c = c.Trim(CurveEnd.Start, offsets[e])
                        .Trim(CurveEnd.End, offsets[Tuple.Create(e.Item2, e.Item1)]);
                }
                catch
                {
                    throw new Exception("Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                }

                if (c == null)
                    throw new Exception("Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                
                rodCentrelines[e] = centreline;

                var profile = GetProfile(radius, sides, vector);

                rodBreps[e] = Brep.CreateFromSweep(c, profile, true, 0.1).First();
                rodBreps[e] = rodBreps[e].CapPlanarHoles(0.1);
                rodBreps[e].Translate(vertices[e.Item1].X + vector.X * offsets[e], vertices[e.Item1].Y + vector.Y * offsets[e], vertices[e.Item1].Z + vector.Z * offsets[e]);
            }

            return Tuple.Create(rodBreps, rodCentrelines);
        }

        private Dictionary<int, Brep> GetJointBreps(Dictionary<Tuple<int, int>, Brep> rodBreps, List<Tuple<int, int>> edges,
            Rhino.Geometry.Collections.MeshVertexList vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides,
            double jointLength, double jointThickness, double tolerance)
        {
            var jointBreps = new Dictionary<int, Brep>();
            var separateJointBreps = new Dictionary<int, List<Brep>>();
            
            double jointRadius = radius + jointThickness + tolerance;

            for (int v = 0; v < vertices.Count; v++)
            {
                separateJointBreps[v] = new List<Brep>();
            }

            foreach(Tuple<int, int> e in edges)
            {
                var vector = Point3d.Subtract(vertices[e.Item2], vertices[e.Item1]);
                Curve c = new LineCurve(new Point3d(0, 0, 0), Point3d.Add(new Point3d(0, 0, 0), vector));
                vector.Unitize();

                double len = c.GetLength();

                Curve startCurve = c.Trim(CurveEnd.End, len - (offsets[e] + jointLength + tolerance));
                Curve endCurve = c.Trim(CurveEnd.Start, len - (offsets[Tuple.Create(e.Item2, e.Item1)] + jointLength + tolerance));

                if (startCurve == null || endCurve == null)
                {
                    throw new Exception("Joint lengths greater than rod length. Try reducing joint lengths or radius.");
                }

                var profile = GetProfile(jointRadius, sides, vector);

                Brep startBrep = Brep.CreateFromSweep(startCurve, profile, true, 0.1).First();
                startBrep = startBrep.CapPlanarHoles(0.1);
                startBrep.Translate(vertices[e.Item1].X, vertices[e.Item1].Y, vertices[e.Item1].Z);

                Brep endBrep = Brep.CreateFromSweep(endCurve, profile, true, 0.1).First();
                var endLength = endCurve.GetLength();
                endBrep = endBrep.CapPlanarHoles(0.1);
                endBrep.Translate(vertices[e.Item2].X - vector.X * endLength, vertices[e.Item2].Y - vector.Y * endLength, vertices[e.Item2].Z - vector.Z * endLength);

                separateJointBreps[e.Item1].AddRange(Brep.CreateBooleanDifference(startBrep, rodBreps[e], 0.1));
                separateJointBreps[e.Item2].AddRange(Brep.CreateBooleanDifference(endBrep, rodBreps[e], 0.1));
            }

            foreach (KeyValuePair<int, List<Brep>> kvp in separateJointBreps)
            {
                try
                {
                    jointBreps[kvp.Key] = Brep.CreateBooleanUnion(kvp.Value, DocumentTolerance()).First();
                }
                catch { }
            }
            return jointBreps;
        }

        private void ColourMeshCollision(Dictionary<Tuple<int, int>, Mesh> rodMeshes, Dictionary<int, Mesh> jointMeshes)
        {
            List<Tuple<int, int>> clashedRods = new List<Tuple<int, int>>();
            List<int> clashedJoints = new List<int>();

            var rodKeys = rodMeshes.Keys.ToList();
            var rodVals = rodMeshes.Values.ToList();
            var jointKeys = jointMeshes.Keys.ToList();
            var jointVals = jointMeshes.Values.ToList();

            for (int i = 0; i < rodKeys.Count; i++)
            {
                for (int j = i + 1; j < rodKeys.Count; j++)
                {
                    if (Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rodVals[i], rodVals[j]).Length > 0)
                    {
                        clashedRods.Add(rodKeys[i]);
                        clashedRods.Add(rodKeys[j]);
                    }
                }

                for (int j = 0; j < jointKeys.Count; j++)
                {
                    if (rodKeys[i].Item1 == jointKeys[j] || rodKeys[i].Item2 == jointKeys[j])
                        continue;

                    if (Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rodVals[i], jointVals[j]).Length > 0)
                    {
                        clashedRods.Add(rodKeys[i]);
                        clashedJoints.Add(jointKeys[j]);
                    }
                }
            }

            for (int i = 0; i < jointKeys.Count; i++)
            {
                for (int j = i + 1; j < jointKeys.Count; j++)
                {
                    if (Rhino.Geometry.Intersect.Intersection.MeshMeshFast(jointVals[i], jointVals[j]).Length > 0)
                    {
                        clashedJoints.Add(jointKeys[i]);
                        clashedJoints.Add(jointKeys[j]);
                    }
                }
            }

            foreach (var kvp in rodMeshes)
            {
                if (clashedRods.Contains(kvp.Key))
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(150, 255, 0, 0), kvp.Value.Vertices.Count).ToArray());
                else
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(100, 255, 255, 255), kvp.Value.Vertices.Count).ToArray());
            }

            foreach (var kvp in jointMeshes)
            {
                if (clashedJoints.Contains(kvp.Key))
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(150, 255, 0, 0), kvp.Value.Vertices.Count).ToArray());
                else
                    kvp.Value.VertexColors.SetColors(Enumerable.Repeat(System.Drawing.Color.FromArgb(200, 40, 40, 40), kvp.Value.Vertices.Count).ToArray());
            }
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
