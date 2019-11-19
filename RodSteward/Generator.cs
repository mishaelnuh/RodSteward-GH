using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino.Geometry;
using MIConvexHull;
using CarveRC;
using System.Runtime.ExceptionServices;
using System.Security;

namespace RodSteward
{
    public class Generator : GH_Component
    {
        const int CIRCLE_SIDES = 100;
        public bool FastRender { get; set; } = true;

        private Dictionary<Tuple<int, int>, Mesh> rodMeshes = new Dictionary<Tuple<int, int>, Mesh>();
        private Dictionary<Tuple<int, int>, Curve> rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();
        private Dictionary<int, List<Mesh>> jointMeshes = new Dictionary<int, List<Mesh>>();
        private List<Tuple<int, int>> clashedRods = new List<Tuple<int, int>>();
        private List<int> clashedJoints = new List<int>();

        public Generator()
          : base("Generator", "RSGenerator",
              "Generate rod lengths and 3D printed joint meshes",
              "RodSteward", "Generator")
        {
        }
        public override void CreateAttributes()
        {
            m_attributes = new GeneratorAttribute(this);
        }

        public override Guid ComponentGuid => new Guid("0ab517a3-7f77-4c94-b004-2a8ab75cc685");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Edges", "E", "Connectivity", GH_ParamAccess.list);
            pManager.AddPointParameter("Vertices", "V", "Rod meshes for structure", GH_ParamAccess.list);
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

            ((IGH_PreviewObject)pManager[0]).Hidden = true;
            ((IGH_PreviewObject)pManager[1]).Hidden = true;
            ((IGH_PreviewObject)pManager[2]).Hidden = true;
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            var errorMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Red, 0.3);
            var rodMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.BurlyWood, 0.3);
            var jointMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Black, 0.3);

            foreach (var kvp in rodMeshes)
            {
                args.Display.DrawMeshShaded(kvp.Value, clashedRods.Contains(kvp.Key) ? errorMaterial : rodMaterial);
            }

            foreach (var kvp in jointMeshes)
            {
                foreach (var b in kvp.Value)
                    args.Display.DrawMeshShaded(b, clashedJoints.Contains(kvp.Key) ? errorMaterial : jointMaterial);
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Tuple<int, int>> edges = new List<Tuple<int, int>>();
            List<Point3d> vertices = new List<Point3d>();
            double sides = 0;
            double radius = 0;
            double jointThickness = 0;
            double jointLength = 0;
            double tolerance = 0;
            
            if (!DA.GetDataList(0, edges)) { return; }
            if (!DA.GetDataList(1, vertices)) { return; }
            if (!DA.GetData(2, ref sides)) { return; }
            if (!DA.GetData(3, ref radius)) { return; }
            if (!DA.GetData(4, ref jointThickness)) { return; }
            if (!DA.GetData(5, ref jointLength)) { return; }
            if (!DA.GetData(6, ref tolerance)) { return; }

            if (edges == null || vertices == null) { return; }
            if (radius <= 0 || sides < 0 || jointThickness < 0 || jointLength < 0 || tolerance < 0) { throw new Exception("Invalid input."); }
            if (sides == 1 || sides == 2) { throw new Exception("Invalid number of sides."); }

            var offsets = GetRodOffsets(edges, vertices, jointThickness, radius, tolerance);

            var rods = GetRodMeshes(edges, vertices, offsets, radius, (int)Math.Floor(sides));
            rodMeshes = rods.Item1;
            rodCentrelines = rods.Item2;

            jointMeshes = GetJointMeshes(rodMeshes, edges, vertices, offsets, radius, (int)Math.Floor(sides), jointLength, jointThickness, tolerance);

            var collisions = FindMeshCollision(rodMeshes, jointMeshes);
            clashedRods = collisions.Item1;
            clashedJoints = collisions.Item2;

            DA.SetDataList(0, jointMeshes.Values.SelectMany(j => j).ToList());
            DA.SetDataList(1, rodMeshes.Values.ToList());
            DA.SetDataList(2, rodCentrelines.Values.ToList());
        }

        private Dictionary<Tuple<int, int>, double> GetRodOffsets(List<Tuple<int, int>> edges, List<Point3d> vertices, double jointThickness, double radius, double tolerance)
        {
            var offsets = new Dictionary<Tuple<int, int>, double>();

            var jointRadius = radius + jointThickness + tolerance; // add extra tolerance
            var innerRadius = radius + tolerance;

            for (int vStart = 0; vStart < vertices.Count; vStart++)
            {
                var connectedVertices = edges.Where(e => e.Item1 == vStart).Select(e => e.Item2).ToList();
                connectedVertices.AddRange(edges.Where(e => e.Item2 == vStart).Select(e => e.Item1).ToList());

                if (connectedVertices.Count == 1)
                {
                    var key = Tuple.Create(vStart, connectedVertices[0]);

                    if (offsets.ContainsKey(key))
                        offsets[key] = Math.Max(radius/2, offsets[key]);
                    else
                        offsets[key] = radius/2;
                }
                else
                {
                    for (int vEnd = 0; vEnd < connectedVertices.Count; vEnd++)
                    {
                        for (int vCompare = vEnd + 1; vCompare < connectedVertices.Count; vCompare++)
                        {
                            var key1 = Tuple.Create(vStart, connectedVertices[vEnd]);
                            var key2 = Tuple.Create(vStart, connectedVertices[vCompare]);

                            var vec1 = vertices[connectedVertices[vEnd]] - vertices[vStart];
                            var vec2 = vertices[connectedVertices[vCompare]] - vertices[vStart];
                            vec1.Unitize();
                            vec2.Unitize();

                            var angle = Math.Acos(vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z);
                            double offset = innerRadius / 2;
                            try
                            {
                                offset = Math.Max((jointRadius) / Math.Tan(angle / 2), innerRadius / 2);
                                if (Double.IsNaN(offset))
                                    offset = innerRadius / 2;
                            }
                            catch
                            {
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
            }

            return offsets;
        }

        private Curve GetProfile(double radius, int sides, Vector3d normal)
        {
            var profilePlane = new Plane(new Point3d(0, 0, 0), normal);
            var circle = new Circle(profilePlane, radius);
            return Polyline.CreateInscribedPolygon(circle, sides).ToPolylineCurve();
        }

        private Tuple<Dictionary<Tuple<int, int>, Mesh>, Dictionary<Tuple<int, int>, Curve>> GetRodMeshes(List<Tuple<int, int>> edges,
            List<Point3d> vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides)
        {
            var rodMeshes = new Dictionary<Tuple<int, int>, Mesh>();
            var rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();

            foreach (Tuple<int, int> e in edges)
            {
                Curve centreline = new LineCurve(vertices[e.Item1], vertices[e.Item2]);
                try
                {
                    var o1 = offsets[e];
                    var o2 = offsets[Tuple.Create(e.Item2, e.Item1)];
                    if (o1 > 0)
                    { 
                        centreline = centreline.Trim(CurveEnd.Start, o1);
                    }
                    if (o2 > 0)
                    {
                        centreline = centreline.Trim(CurveEnd.End, o2);
                    }

                    if (centreline == null)
                        throw new Exception();
                }
                catch
                {
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Rod not created. Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                    continue;
                }

                rodCentrelines[e] = centreline;

                rodMeshes[e] = Mesh.CreateFromCurvePipe(centreline, radius, sides, 0, MeshPipeCapStyle.Flat, false);
            }

            return Tuple.Create(rodMeshes, rodCentrelines);
        }

        [HandleProcessCorruptedStateExceptions]
        [SecurityCritical]
        private Dictionary<int, List<Mesh>> GetJointMeshes(Dictionary<Tuple<int, int>, Mesh> rodMeshes, List<Tuple<int, int>> edges,
            List<Point3d> vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides,
            double jointLength, double jointThickness, double tolerance)
        {
            var jointMeshes = new Dictionary<int, List<Mesh>>();
            var separateJointMeshes = new Dictionary<int, List<Mesh>>();

            var jointCorePoints = new Dictionary<int, List<double[]>>();
            
            double jointRadius = radius + jointThickness + tolerance;

            for (int v = 0; v < vertices.Count; v++)
            {
                separateJointMeshes[v] = new List<Mesh>();
                jointCorePoints[v] = new List<double[]>();
            }

            foreach(Tuple<int, int> e in edges)
            {
                if (!rodMeshes.ContainsKey(e))
                    continue;

                Curve c = new LineCurve(vertices[e.Item1], vertices[e.Item2]);

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

                var vector = Point3d.Subtract(vertices[e.Item2], vertices[e.Item1]);
                vector.Unitize();

                Polyline outerPoly;
                Polyline innerPoly;
                var outerProfile = GetProfile(jointRadius, sides, vector);
                var innerProfile = GetProfile(radius, sides, vector);
                outerProfile.TryGetPolyline(out outerPoly);
                innerProfile.TryGetPolyline(out innerPoly);

                // Convex hull points
                foreach (var l in outerPoly.Skip(1))
                {
                    jointCorePoints[e.Item1].Add(new double[] { l.X + vertices[e.Item1].X, l.Y + vertices[e.Item1].Y, l.Z + vertices[e.Item1].Z });
                    jointCorePoints[e.Item2].Add(new double[] { l.X + vertices[e.Item2].X, l.Y + vertices[e.Item2].Y, l.Z + vertices[e.Item2].Z });
                }

                // Start joint
                var startMesh = new Mesh();
                Vector3d startLength = vector * startCurve.GetLength();
                Vector3d startOffsetLength = vector * offsets[e];

                // V
                startMesh.Vertices.Add(vertices[e.Item1]);
                startMesh.Vertices.Add(Point3d.Add(vertices[e.Item1], startOffsetLength));
                foreach (var p in outerPoly.Skip(1))
                    startMesh.Vertices.Add(Point3d.Add(p, vertices[e.Item1]));
                foreach (var p in outerPoly.Skip(1))
                    startMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item1]), startLength));
                foreach (var p in innerPoly.Skip(1))
                    startMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item1]), startOffsetLength));
                foreach (var p in innerPoly.Skip(1))
                    startMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item1]), startLength));

                // F
                for (int i = 2; i < sides + 1; i++)
                    startMesh.Faces.AddFace(0, i, i + 1);
                startMesh.Faces.AddFace(0, sides + 1, 2);

                for (int i = 2 * sides + 2; i < 3 * sides + 1; i++)
                    startMesh.Faces.AddFace(1, i, i + 1);
                startMesh.Faces.AddFace(1, 3 * sides + 1, 2 * sides + 2);

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        startMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2 + i, sides + 3 + i);
                        startMesh.Faces.AddFace(3 * sides + 2 + i, sides + 3 + i, 3 * sides + 3 + i);
                    }
                    else
                    {
                        startMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2 + i, sides + 2);
                        startMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2, 3 * sides + 2);
                    }
                }

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        startMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2 + i, 3 * sides + 3 + i);
                        startMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 3 + i, 2 * sides + 3 + i);
                    }
                    else
                    {
                        startMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2 + i, 3 * sides + 2);
                        startMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2, 2 * sides + 2);
                    }
                }

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        startMesh.Faces.AddFace(2 + i, sides + 2 + i, sides + 3 + i);
                        startMesh.Faces.AddFace(2 + i, sides + 3 + i, 3 + i);
                    }
                    else
                    {
                        startMesh.Faces.AddFace(2 + i, sides + 2 + i, sides + 2);
                        startMesh.Faces.AddFace(2 + i, sides + 2, 2);
                    }
                }
                startMesh.Normals.ComputeNormals();
                startMesh.UnifyNormals();
                startMesh.Normals.ComputeNormals();
                separateJointMeshes[e.Item1].Add(startMesh);

                // End joint
                var endMesh = new Mesh();
                Vector3d endLength = vector * -endCurve.GetLength();
                Vector3d endOffsetLength = vector * -offsets[Tuple.Create(e.Item2, e.Item1)];

                // V
                endMesh.Vertices.Add(vertices[e.Item2]);
                endMesh.Vertices.Add(Point3d.Add(vertices[e.Item2], endOffsetLength));
                foreach (var p in outerPoly.Skip(1))
                    endMesh.Vertices.Add(Point3d.Add(p, vertices[e.Item2]));
                foreach (var p in outerPoly.Skip(1))
                    endMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item2]), endLength));
                foreach (var p in innerPoly.Skip(1))
                    endMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item2]), endOffsetLength));
                foreach (var p in innerPoly.Skip(1))
                    endMesh.Vertices.Add(Point3d.Add(Point3d.Add(p, vertices[e.Item2]), endLength));

                // F
                for (int i = 2; i < sides + 1; i++)
                    endMesh.Faces.AddFace(0, i, i + 1);
                endMesh.Faces.AddFace(0, sides + 1, 2);

                for (int i = 2 * sides + 2; i < 3 * sides + 1; i++)
                    endMesh.Faces.AddFace(1, i, i + 1);
                endMesh.Faces.AddFace(1, 3 * sides + 1, 2 * sides + 2);

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        endMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2 + i, sides + 3 + i);
                        endMesh.Faces.AddFace(3 * sides + 2 + i, sides + 3 + i, 3 * sides + 3 + i);
                    }
                    else
                    {
                        endMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2 + i, sides + 2);
                        endMesh.Faces.AddFace(3 * sides + 2 + i, sides + 2, 3 * sides + 2);
                    }
                }

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        endMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2 + i, 3 * sides + 3 + i);
                        endMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 3 + i, 2 * sides + 3 + i);
                    }
                    else
                    {
                        endMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2 + i, 3 * sides + 2);
                        endMesh.Faces.AddFace(2 * sides + 2 + i, 3 * sides + 2, 2 * sides + 2);
                    }
                }

                for (int i = 0; i < sides; i++)
                {
                    if (i < sides - 1)
                    {
                        endMesh.Faces.AddFace(2 + i, sides + 2 + i, sides + 3 + i);
                        endMesh.Faces.AddFace(2 + i, sides + 3 + i, 3 + i);
                    }
                    else
                    {
                        endMesh.Faces.AddFace(2 + i, sides + 2 + i, sides + 2);
                        endMesh.Faces.AddFace(2 + i, sides + 2, 2);
                    }
                }
                endMesh.Normals.ComputeNormals();
                endMesh.UnifyNormals();
                endMesh.Normals.ComputeNormals();
                separateJointMeshes[e.Item2].Add(endMesh);
            }

            foreach (KeyValuePair<int, List<double[]>> kvp in jointCorePoints)
            {
                try
                {
                    var convHullRes = ConvexHull.Create(kvp.Value, DocumentTolerance());
                    var hullPoints = convHullRes.Result.Points.ToList();
                    var hullFaces = convHullRes.Result.Faces.ToList();

                    var newMesh = new Mesh();
                    newMesh.Vertices.AddVertices(hullPoints.Select(p => new Point3d(p.Position[0], p.Position[1], p.Position[2])));
                    newMesh.Faces.AddFaces(hullFaces.Select(f => new MeshFace(hullPoints.IndexOf(f.Vertices[0]), hullPoints.IndexOf(f.Vertices[1]), hullPoints.IndexOf(f.Vertices[2]))));
                    newMesh.Normals.ComputeNormals();
                    newMesh.UnifyNormals();
                    newMesh.Normals.ComputeNormals();
                    newMesh.Compact();
                    separateJointMeshes[kvp.Key].Insert(0,newMesh); // Add conv hull to beginning
                }
                catch { }
            }

            if (FastRender)
            {
                jointMeshes = separateJointMeshes;
            }
            else
            {
                foreach (KeyValuePair<int, List<Mesh>> kvp in separateJointMeshes)
                {
                    try
                    {
                        var resMesh = kvp.Value.First();
                        foreach (var m in kvp.Value.Skip(1))
                        {
                            resMesh = CarveOps.PerformCSG(resMesh, m, Operations.Union);
                            if (resMesh == null)
                                throw new Exception();
                        }

                        if (!resMesh.IsValid)
                            throw new Exception();

                        jointMeshes[kvp.Key] = new List<Mesh>() { resMesh };
                    }
                    catch
                    {
                        try
                        {
                            // Backup logic. Skip convex hull and use Rhino boolean
                            var rhMesh = Mesh.CreateBooleanUnion(kvp.Value.Skip(1));

                            if (rhMesh.Count() == 1)
                                jointMeshes[kvp.Key] = rhMesh.ToList();
                            else
                            {
                                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to boolean joint brep. Try changing some parameters.");
                            }
                        }
                        catch
                        {
                            this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to boolean joint brep. Try changing some parameters.");
                        }
                    }
                }
            }
            return jointMeshes;
        }

        private Tuple<List<Tuple<int,int>>, List<int>> FindMeshCollision(Dictionary<Tuple<int, int>, Mesh> rodMeshes, Dictionary<int, List<Mesh>> jointMeshes)
        {
            var clashedRods = new List<Tuple<int, int>>();
            var clashedJoints = new List<int>();

            var rodKeys = rodMeshes.Keys.ToList();
            var rodVals = rodMeshes.Values.ToList();
            var jointKeys = jointMeshes.Keys.ToList();
            var jointVals = jointMeshes.Values.ToList();

            for (int i = 0; i < rodKeys.Count; i++)
            {
                for (int j = i + 1; j < rodKeys.Count; j++)
                {
                    var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshAccurate(rodVals[i], rodVals[j], DocumentTolerance());
                    if (intersect != null && intersect.Length > 0)
                    {
                        clashedRods.Add(rodKeys[i]);
                        clashedRods.Add(rodKeys[j]);
                    }
                }

                for (int j = 0; j < jointKeys.Count; j++)
                {
                    if (rodKeys[i].Item1 == jointKeys[j] || rodKeys[i].Item2 == jointKeys[j])
                        continue;

                    foreach (var jv in jointVals[j])
                    {
                        var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshAccurate(rodVals[i], jv, DocumentTolerance());
                        if (intersect != null && intersect.Length > 0)
                        {
                            clashedRods.Add(rodKeys[i]);
                            clashedJoints.Add(jointKeys[j]);
                            break;
                        }
                    }
                }
            }

            var hit = false;
            for (int i = 0; i < jointKeys.Count; i++)
            {
                for (int j = i + 1; j < jointKeys.Count; j++)
                {
                    foreach (var iv in jointVals[i])
                    {
                        foreach (var jv in jointVals[j])
                        {
                            var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshAccurate(iv, jv, DocumentTolerance());
                            if (intersect != null && intersect.Length > 0)
                            {
                                clashedJoints.Add(jointKeys[i]);
                                clashedJoints.Add(jointKeys[j]);
                                hit = true;
                                break;
                            }
                        }
                        if (hit)
                        {
                            hit = false;
                            break;
                        }
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

    public class GeneratorAttribute : Grasshopper.Kernel.Attributes.GH_ComponentAttributes
    {
        public GeneratorAttribute(GH_Component owner) : base(owner) { }

        protected override void Layout()
        {
            base.Layout();

            System.Drawing.Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
            rec0.Height += 22 * 2;

            System.Drawing.Rectangle rec1 = rec0;
            rec1.Y = rec1.Bottom - 22 * 2;
            rec1.Height = 22;
            rec1.Inflate(-2, -2);

            System.Drawing.Rectangle rec2 = rec0;
            rec2.Y = rec2.Bottom - 22;
            rec2.Height = 22;
            rec2.Inflate(-2, -2);

            Bounds = rec0;
            ButtonBounds1 = rec1;
            ButtonBounds2 = rec2;
        }
        private System.Drawing.Rectangle ButtonBounds1 { get; set; }
        private System.Drawing.Rectangle ButtonBounds2 { get; set; }

        protected override void Render(GH_Canvas canvas, System.Drawing.Graphics graphics, GH_CanvasChannel channel)
        {
            base.Render(canvas, graphics, channel);

            if (channel == GH_CanvasChannel.Objects)
            {
                var component = Owner as Generator;
                GH_Capsule button1 = GH_Capsule.CreateTextCapsule(ButtonBounds1, ButtonBounds1, component.FastRender ? GH_Palette.Black : GH_Palette.Grey, "Fast Render", 2, 0);
                GH_Capsule button2 = GH_Capsule.CreateTextCapsule(ButtonBounds2, ButtonBounds2, !component.FastRender ? GH_Palette.Black : GH_Palette.Grey, "Full Render", 2, 0);
                button1.Render(graphics, Selected, Owner.Locked, false);
                button1.Dispose();
                button2.Render(graphics, Selected, Owner.Locked, false);
                button2.Dispose();
            }
        }
        public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Left)
            {
                System.Drawing.RectangleF rec1 = ButtonBounds1;
                System.Drawing.RectangleF rec2 = ButtonBounds2;
                if (rec1.Contains(e.CanvasLocation))
                {
                    var component = Owner as Generator;
                    if (component.FastRender != true)
                    {
                        component.FastRender = true;
                        component.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
                else if (rec2.Contains(e.CanvasLocation))
                {
                    var component = Owner as Generator;
                    if (component.FastRender != false)
                    {
                        component.FastRender = false;
                        component.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
            }
            return base.RespondToMouseDown(sender, e);
        }
    }
}
