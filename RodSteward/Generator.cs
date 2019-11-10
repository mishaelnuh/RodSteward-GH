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

namespace RodSteward
{
    public class Generator : GH_Component
    {
        const int CIRCLE_SIDES = 100;
        public bool FastRender { get; set; } = true;

        private Dictionary<Tuple<int, int>, Brep> rodBreps = new Dictionary<Tuple<int, int>, Brep>();
        private Dictionary<Tuple<int, int>, Curve> rodCentrelines = new Dictionary<Tuple<int, int>, Curve>();
        private Dictionary<int, List<Brep>> jointBreps = new Dictionary<int, List<Brep>>();
        private List<Tuple<int, int>> clashedRods = new List<Tuple<int, int>>();
        private List<int> clashedJoints = new List<int>();

        public Generator()
          : base("Generator", "RSGenerator",
              "Generate rod lengths and 3D printed joint meshes",
              "RodSteward", "RodSteward")
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
            var rodMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.BurlyWood, 0.1);
            var jointMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Black, 0.1);

            foreach (var kvp in rodBreps)
            {
                args.Display.DrawBrepShaded(kvp.Value, clashedRods.Contains(kvp.Key) ? errorMaterial : rodMaterial);
            }

            foreach (var kvp in jointBreps)
            {
                foreach(var b in kvp.Value)
                    args.Display.DrawBrepShaded(b, clashedJoints.Contains(kvp.Key) ? errorMaterial : jointMaterial);
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

            var offsets = GetRodOffsets(edges, vertices, radius, tolerance);

            var rods = GetRodBreps(edges, vertices, offsets, radius, (int)Math.Floor(sides));
            rodBreps = rods.Item1;
            rodCentrelines = rods.Item2;

            jointBreps = GetJointBreps(rodBreps, edges, vertices, offsets, radius, (int)Math.Floor(sides), jointLength, jointThickness, tolerance);

            var collisions = FindBrepCollision(rodBreps, jointBreps);
            clashedRods = collisions.Item1;
            clashedJoints = collisions.Item2;

            DA.SetDataList(0, jointBreps.Values.SelectMany(j => j).ToList());
            DA.SetDataList(1, rodBreps.Values.ToList());
            DA.SetDataList(2, rodCentrelines.Values.ToList());
        }

        private Dictionary<Tuple<int, int>, double> GetRodOffsets(List<Tuple<int, int>> edges, List<Point3d> vertices, double radius, double tolerance)
        {
            var offsets = new Dictionary<Tuple<int, int>, double>();
            
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

                            double offset = radius / 2;
                            try
                            {
                                offset = Math.Max((radius + tolerance) / Math.Tan(angle / 2), radius/2);
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
        private Tuple<Dictionary<Tuple<int, int>, Brep>, Dictionary<Tuple<int, int>, Curve>> GetRodBreps(List<Tuple<int, int>> edges,
            List<Point3d> vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides)
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
                    var o1 = offsets[e];
                    var o2 = offsets[Tuple.Create(e.Item2, e.Item1)];
                    if (o1 > 0)
                    { 
                        centreline = centreline.Trim(CurveEnd.Start, o1);
                        c = c.Trim(CurveEnd.Start, o1);
                    }
                    if (o2 > 0)
                    {
                        centreline = centreline.Trim(CurveEnd.End, o2);
                        c = c.Trim(CurveEnd.End, o2);
                    }

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

        private Dictionary<int, List<Brep>> GetJointBreps(Dictionary<Tuple<int, int>, Brep> rodBreps, List<Tuple<int, int>> edges,
            List<Point3d> vertices, Dictionary<Tuple<int, int>, double> offsets, double radius, int sides,
            double jointLength, double jointThickness, double tolerance)
        {
            var jointBreps = new Dictionary<int, List<Brep>>();
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

                separateJointBreps[e.Item1].Add(startBrep);
                separateJointBreps[e.Item2].Add(endBrep);

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
                try
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
                catch { }
            }

            if (FastRender)
            {
                jointBreps = separateJointBreps;
            }
            else
            {
                foreach (KeyValuePair<int, List<Brep>> kvp in separateJointBreps)
                {
                    try
                    {
                        jointBreps[kvp.Key] = Brep.CreateBooleanUnion(kvp.Value, DocumentTolerance()).ToList();
                        jointBreps[kvp.Key] = Brep.CreateBooleanDifference(jointBreps[kvp.Key], rodBreps.Where(r => r.Key.Item1 == kvp.Key || r.Key.Item2 == kvp.Key).Select(r => r.Value), DocumentTolerance()).ToList();
                    }
                    catch
                    {
                        this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to boolean joint brep. Try changing some parameters.");
                    }
                }
            }
            return jointBreps;
        }

        private Tuple<List<Tuple<int,int>>, List<int>> FindBrepCollision(Dictionary<Tuple<int, int>, Brep> rodBreps, Dictionary<int, List<Brep>> jointBreps)
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

                    foreach (var jv in jointVals[j])
                    {
                        Curve[] intCurve;
                        Point3d[] intPoints;
                        Rhino.Geometry.Intersect.Intersection.BrepBrep(rodVals[i], jv, DocumentTolerance(), out intCurve, out intPoints);
                        if (intCurve.Length > 0 || intPoints.Length > 0)
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
                            Curve[] intCurve;
                            Point3d[] intPoints;
                            Rhino.Geometry.Intersect.Intersection.BrepBrep(iv, jv, DocumentTolerance(), out intCurve, out intPoints);
                            if (intCurve.Length > 0 || intPoints.Length > 0)
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
