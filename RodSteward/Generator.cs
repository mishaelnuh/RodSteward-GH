using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using MIConvexHull;

namespace RodSteward
{
    public class Generator : GH_Component
    {
        const int CIRCLE_SIDES = 100;
        private Dictionary<Tuple<int, int>, Brep> rodBreps = new Dictionary<Tuple<int, int>, Brep>();
        private Dictionary<Tuple<int, int>, Curve> rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();
        private Dictionary<int, Brep> jointBreps = new Dictionary<int, Brep>();
        private List<Tuple<int, int>> clashedRods = new List<Tuple<int, int>>();
        private List<int> clashedJoints = new List<int>();

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

            ((IGH_PreviewObject)pManager[0]).Hidden = true;
            ((IGH_PreviewObject)pManager[1]).Hidden = true;
            ((IGH_PreviewObject)pManager[2]).Hidden = true;
        }
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            var errorMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Red, 0.3);
            var rodMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.White, 0.3);
            var jointMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.DarkGray, 0.3);

            foreach (var kvp in rodBreps)
            {
                args.Display.DrawBrepShaded(kvp.Value, clashedRods.Contains(kvp.Key) ? errorMaterial : rodMaterial);
            }

            foreach (var kvp in jointBreps)
            {
                args.Display.DrawBrepShaded(kvp.Value, clashedJoints.Contains(kvp.Key) ? errorMaterial : jointMaterial);
            }
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
            rodBreps = rods.Item1;
            rodCentrelines = rods.Item2;

            jointBreps = GetJointBreps(rodBreps, edges, vertices, offsets, radius, (int)Math.Floor(sides), jointLength, jointThickness, tolerance);


            var collisions = FindBrepCollision(rodBreps, jointBreps);
            clashedRods = collisions.Item1;
            clashedJoints = collisions.Item2;

            DA.SetDataList(0, jointBreps.Values.ToList());
            DA.SetDataList(1, rodBreps.Values.ToList());
            DA.SetDataList(2, rodCentrelines.Values.ToList());
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
                    if (c == null || centreline == null)
                        throw new Exception();
                }
                catch
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Rod not created. Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                    continue;
                }

                rodCentrelines[e] = centreline;

                var profile = GetProfile(radius, sides, vector);

                rodBreps[e] = Brep.CreateFromSweep(c, profile, true, DocumentTolerance()).First();
                rodBreps[e] = rodBreps[e].CapPlanarHoles(DocumentTolerance());
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

            var jointCorePoints = new Dictionary<int, List<double[]>>();
            
            double jointRadius = radius + jointThickness + tolerance;

            for (int v = 0; v < vertices.Count; v++)
            {
                separateJointBreps[v] = new List<Brep>();
                jointCorePoints[v] = new List<double[]>();
            }

            foreach(Tuple<int, int> e in edges)
            {
                if (!rodBreps.ContainsKey(e))
                    continue;
                
                var vector = Point3d.Subtract(vertices[e.Item2], vertices[e.Item1]);
                Curve c = new LineCurve(new Point3d(0, 0, 0), Point3d.Add(new Point3d(0, 0, 0), vector));
                vector.Unitize();

                double len = c.GetLength();

                if (len == 0)
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Joint not created. Joint lengths greater than rod length. Try reducing joint lengths or radius.");
                    continue;
                }

                Curve startCurve;
                Curve endCurve;

                try
                {
                    startCurve = c.Trim(CurveEnd.End, len - (offsets[e] + jointLength + tolerance));
                    endCurve = c.Trim(CurveEnd.Start, len - (offsets[Tuple.Create(e.Item2, e.Item1)] + jointLength + tolerance));

                    if (startCurve == null || endCurve == null)
                        throw new Exception();
                }
                catch
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Joint not created. Joint lengths greater than rod length. Try reducing joint lengths or radius.");
                    continue;
                }

                var profile = GetProfile(jointRadius, sides, vector);

                Brep startBrep = Brep.CreateFromSweep(startCurve, profile, true, DocumentTolerance()).First();
                startBrep = startBrep.CapPlanarHoles(DocumentTolerance());
                startBrep.Translate(vertices[e.Item1].X, vertices[e.Item1].Y, vertices[e.Item1].Z);


                Brep endBrep = Brep.CreateFromSweep(endCurve, profile, true, DocumentTolerance()).First();
                var endLength = endCurve.GetLength();
                endBrep = endBrep.CapPlanarHoles(DocumentTolerance());
                endBrep.Translate(vertices[e.Item2].X - vector.X * endLength, vertices[e.Item2].Y - vector.Y * endLength, vertices[e.Item2].Z - vector.Z * endLength);

                separateJointBreps[e.Item1].AddRange(Brep.CreateBooleanDifference(startBrep, rodBreps[e], DocumentTolerance()));
                separateJointBreps[e.Item2].AddRange(Brep.CreateBooleanDifference(endBrep, rodBreps[e], DocumentTolerance()));

                Polyline corePoly;
                profile.TryGetPolyline(out corePoly);
                foreach (var l in corePoly)
                {
                    jointCorePoints[e.Item1].Add(new double[] { l.X + vertices[e.Item1].X, l.Y + vertices[e.Item1].Y, l.Z + vertices[e.Item1].Z });
                    jointCorePoints[e.Item2].Add(new double[] { l.X + vertices[e.Item2].X, l.Y + vertices[e.Item2].Y, l.Z + vertices[e.Item2].Z });
                }
            }

            foreach (KeyValuePair<int, List<double[]>> kvp in jointCorePoints)
            {
                var convHullRes = ConvexHull.Create(kvp.Value, DocumentTolerance());
                var hullPoints = convHullRes.Result.Points.ToList();
                var hullFaces = convHullRes.Result.Faces.ToList();

                var newMesh = new Mesh();
                newMesh.Vertices.AddVertices(hullPoints.Select(p => new Point3d(p.Position[0], p.Position[1], p.Position[2])));
                newMesh.Faces.AddFaces(hullFaces.Select(f => new MeshFace(hullPoints.IndexOf(f.Vertices[0]), hullPoints.IndexOf(f.Vertices[1]), hullPoints.IndexOf(f.Vertices[2]))));
                newMesh.Normals.ComputeNormals();
                newMesh.Compact();
                separateJointBreps[kvp.Key].Add(Brep.CreateFromMesh(newMesh, true));
            }

            foreach (KeyValuePair<int, List<Brep>> kvp in separateJointBreps)
            {
                try
                {
                    jointBreps[kvp.Key] = Brep.CreateBooleanUnion(kvp.Value, DocumentTolerance()).First();
                }
                catch
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to boolean joint brep. Try changing some parameters.");
                }
            }
            return jointBreps;
        }

        private Tuple<List<Tuple<int,int>>, List<int>> FindBrepCollision(Dictionary<Tuple<int, int>, Brep> rodBreps, Dictionary<int, Brep> jointBreps)
        {
            var clashedRods = new List<Tuple<int, int>>();
            var clashedJoints = new List<int>();

            var rodKeys = rodBreps.Keys.ToList();
            var rodVals = rodBreps.Values.ToList();
            var jointKeys = jointBreps.Keys.ToList();
            var jointVals = jointBreps.Values.ToList();

            for (int i = 0; i < rodKeys.Count; i++)
            {
                for (int j = i + 1; j < rodKeys.Count; j++)
                {
                    Curve[] intCurve;
                    Point3d[] intPoints;
                    Rhino.Geometry.Intersect.Intersection.BrepBrep(rodVals[i], rodVals[j], DocumentTolerance(), out intCurve, out intPoints);
                    if (intCurve.Length > 0 || intPoints.Length > 0)
                    {
                        clashedRods.Add(rodKeys[i]);
                        clashedRods.Add(rodKeys[j]);
                    }
                }

                for (int j = 0; j < jointKeys.Count; j++)
                {
                    if (rodKeys[i].Item1 == jointKeys[j] || rodKeys[i].Item2 == jointKeys[j])
                        continue;

                    Curve[] intCurve;
                    Point3d[] intPoints;
                    Rhino.Geometry.Intersect.Intersection.BrepBrep(rodVals[i], jointVals[j], DocumentTolerance(), out intCurve, out intPoints);
                    if (intCurve.Length > 0 || intPoints.Length > 0)
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
                    Curve[] intCurve;
                    Point3d[] intPoints;
                    Rhino.Geometry.Intersect.Intersection.BrepBrep(jointVals[i], jointVals[j], DocumentTolerance(), out intCurve, out intPoints);
                    if (intCurve.Length > 0 || intPoints.Length > 0)
                    {
                        clashedJoints.Add(jointKeys[i]);
                        clashedJoints.Add(jointKeys[j]);
                    }
                }
            }

            return Tuple.Create(clashedRods, clashedJoints);
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
