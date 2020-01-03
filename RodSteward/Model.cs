using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper;
using MIConvexHull;

namespace RodSteward
{
    public class Model
    {
        public int Sides { get; set; }
        public double Radius { get; set; }
        public double JointThickness { get; set; }
        public double JointLength { get; set; }
        public double Tolerance { get; set; }

        public double InnerWallRadius { get { return Radius + Tolerance; } }
        public double OuterWallRadius { get { return Radius + JointThickness + Tolerance; } }
        public DataTree<Mesh> JointMeshTree {
            get
            {
                var jointMeshTree = new DataTree<Mesh>();

                foreach (var kvp in JointMeshes)
                {
                    var path = new Grasshopper.Kernel.Data.GH_Path(kvp.Key);
                    jointMeshTree.AddRange(kvp.Value, path);
                }

                return jointMeshTree;
            }
        }

        public List<Tuple<int, int>> Edges { get; set; }
        public List<Point3d> Vertices { get; set; }

        public Dictionary<Tuple<int, int>, double> Offsets { get; set; }

        public Dictionary<Tuple<int, int>, Mesh> RodMeshes { get; set; }
        public Dictionary<Tuple<int, int>, Curve> RodCentrelines { get; set; }
        public Dictionary<int, List<Mesh>> JointMeshes { get; set; }

        public List<Tuple<int, int>> ClashedRods { get; set; }
        public List<int> ClashedJoints { get; set; }

        public Dictionary<string, Point3d> JointArmLabel { get; set; }

        public Model()
        {
            Edges = new List<Tuple<int, int>>();
            Vertices = new List<Point3d>();
            Offsets = new Dictionary<Tuple<int, int>, double>();
            RodMeshes = new Dictionary<Tuple<int, int>, Mesh>();
            RodCentrelines = new Dictionary<Tuple<int, int>, Curve>();
            JointMeshes = new Dictionary<int, List<Mesh>>();
            ClashedRods = new List<Tuple<int, int>>();
            ClashedJoints = new List<int>();
            JointArmLabel = new Dictionary<string, Point3d>();
        }

        public void Generate(bool label = true)
        {
            ClashedJoints.Clear();
            ClashedRods.Clear();

            CalculateRodOffsets();
            GenerateRodMeshes();
            GenerateJointMeshes(label);
        }

        public Dictionary<Tuple<int, int>, double> CalculateRodOffsets()
        {
            Offsets.Clear();

            for (int vStart = 0; vStart < Vertices.Count; vStart++)
            {
                var connectedVertices = Edges.Where(e => e.Item1 == vStart).Select(e => e.Item2).ToList();
                connectedVertices.AddRange(Edges.Where(e => e.Item2 == vStart).Select(e => e.Item1).ToList());

                if (connectedVertices.Count == 1)
                {
                    var key = Tuple.Create(vStart, connectedVertices[0]);

                    if (Offsets.ContainsKey(key))
                        Offsets[key] = Math.Max(Radius / 2, Offsets[key]);
                    else
                        Offsets[key] = Radius / 2;
                }
                else
                {
                    for (int vEnd = 0; vEnd < connectedVertices.Count; vEnd++)
                    {
                        for (int vCompare = vEnd + 1; vCompare < connectedVertices.Count; vCompare++)
                        {
                            var key1 = Tuple.Create(vStart, connectedVertices[vEnd]);
                            var key2 = Tuple.Create(vStart, connectedVertices[vCompare]);

                            // Calculate angle
                            // Use procedure from: https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
                            // to minimize error for small angles

                            var vec1 = Vertices[connectedVertices[vEnd]] - Vertices[vStart];
                            var vec2 = Vertices[connectedVertices[vCompare]] - Vertices[vStart];
                            var vec3 = Vector3d.Subtract(vec1, vec2);
                            var mag1 = vec1.Length;
                            var mag2 = vec2.Length;
                            var mag3 = vec3.Length;

                            if (mag2 > mag1)
                            {
                                var t = mag2;
                                mag2 = mag1;
                                mag1 = t;
                            }

                            double mu = 0;
                            if (mag2 >= mag3)
                                mu = mag3 - (mag1 - mag2);
                            else
                                mu = mag2 - (mag1 - mag3);

                            var t1 = ((mag1 - mag2) + mag3) * mu;
                            var t2 = (mag1 + (mag2 + mag3)) * ((mag1 - mag3) + mag2);

                            double divParam = 0;
                            double offset = 0;
                            if (Math.Abs(t2) <= 1e-6)
                                offset = OuterWallRadius;
                            else
                            {
                                divParam = Math.Sqrt(t1 / t2);
                                offset = Math.Max(OuterWallRadius / divParam, OuterWallRadius);
                            }

                            if (Offsets.ContainsKey(key1))
                                Offsets[key1] = Math.Max(offset, Offsets[key1]);
                            else
                                Offsets[key1] = offset;

                            if (Offsets.ContainsKey(key2))
                                Offsets[key2] = Math.Max(offset, Offsets[key2]);
                            else
                                Offsets[key2] = offset;
                        }
                    }
                }
            }

            return Offsets;
        }

        public Tuple<List<Tuple<int, int>>, List<int>>  CalculateClashes()
        {
            ClashedRods.Clear();
            ClashedJoints.Clear();

            var rodKeys = RodMeshes.Keys.ToList();
            var rodVals = RodMeshes.Values.ToList();
            var jointKeys = JointMeshes.Keys.ToList();
            var jointVals = JointMeshes.Values.ToList();

            for (int i = 0; i < rodKeys.Count; i++)
            {
                for (int j = i + 1; j < rodKeys.Count; j++)
                {
                    if (ClashedRods.Contains(rodKeys[i]) && ClashedRods.Contains(rodKeys[j]))
                        continue;

                    var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rodVals[i], rodVals[j]);
                    if (intersect != null && intersect.Length > 0)
                    {
                        ClashedRods.Add(rodKeys[i]);
                        ClashedRods.Add(rodKeys[j]);
                    }
                }

                for (int j = 0; j < jointKeys.Count; j++)
                {
                    if (rodKeys[i].Item1 == jointKeys[j] || rodKeys[i].Item2 == jointKeys[j])
                        continue;

                    if (ClashedRods.Contains(rodKeys[i]) && ClashedJoints.Contains(jointKeys[j]))
                        continue;

                    foreach (var jv in jointVals[j])
                    {
                        var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(rodVals[i], jv);
                        if (intersect != null && intersect.Length > 0)
                        {
                            ClashedRods.Add(rodKeys[i]);
                            ClashedJoints.Add(jointKeys[j]);
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
                    if (ClashedJoints.Contains(jointKeys[i]) && ClashedJoints.Contains(jointKeys[j]))
                        continue;

                    foreach (var iv in jointVals[i])
                    {
                        foreach (var jv in jointVals[j])
                        {
                            var intersect = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(iv, jv);
                            if (intersect != null && intersect.Length > 0)
                            {
                                ClashedJoints.Add(jointKeys[i]);
                                ClashedJoints.Add(jointKeys[j]);
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

            return Tuple.Create(ClashedRods, ClashedJoints);
        }

        public Dictionary<Tuple<int, int>, Mesh> GenerateRodMeshes()
        {
            RodMeshes.Clear();
            RodCentrelines.Clear();

            foreach (Tuple<int, int> e in Edges)
            {
                Curve centreline = new LineCurve(Vertices[e.Item1], Vertices[e.Item2]);
                try
                {
                    var o1 = Offsets[e];
                    var o2 = Offsets[Tuple.Create(e.Item2, e.Item1)];
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
                    throw new Exception("Rod not created. Radius and tolerance too large for edge lengths. Try reducing either or increase the edge lengths.");
                }

                RodCentrelines[e] = centreline;

                RodMeshes[e] = Mesh.CreateFromCurvePipe(centreline, Radius, Sides, 0, MeshPipeCapStyle.Flat, false);
            }

            return RodMeshes;
        }

        public Dictionary<int, List<Mesh>> GenerateJointMeshes(bool label = true)
        {
            JointMeshes.Clear();
            JointArmLabel.Clear();

            var separateJointMeshes = new Dictionary<int, List<Mesh>>();

            var jointArmCounter = new Dictionary<int, int>();

            var jointCorePoints = new Dictionary<int, List<double[]>>();

            for (int v = 0; v < Vertices.Count; v++)
            {
                separateJointMeshes[v] = new List<Mesh>();
                jointCorePoints[v] = new List<double[]>();
                jointArmCounter[v] = 1;
            }

            foreach (Tuple<int, int> e in Edges)
            {
                if (!RodMeshes.ContainsKey(e))
                    continue;

                Curve c = new LineCurve(Vertices[e.Item1], Vertices[e.Item2]);

                double len = c.GetLength();

                if (len == 0)
                {
                    throw new Exception("Joint not created. Joint lengths greater than rod length. Try reducing joint lengths or radius.");
                }

                Curve startCurve;
                Curve endCurve;

                try
                {
                    startCurve = c.Trim(CurveEnd.End, len - (Offsets[e] + JointLength));
                    endCurve = c.Trim(CurveEnd.Start, len - (Offsets[Tuple.Create(e.Item2, e.Item1)] + JointLength));

                    if (startCurve == null || endCurve == null)
                        throw new Exception();
                }
                catch
                {
                    throw new Exception("Joint not created. Joint lengths greater than rod length. Try reducing joint lengths or radius.");
                }

                // Convex hull points
                var vector = Point3d.Subtract(Vertices[e.Item2], Vertices[e.Item1]);
                vector.Unitize();

                Polyline outerPoly;
                var outerProfile = GetProfile(OuterWallRadius, vector);
                outerProfile.TryGetPolyline(out outerPoly);

                foreach (var l in outerPoly.Skip(1))
                {
                    jointCorePoints[e.Item1].Add(new double[] { l.X + Vertices[e.Item1].X, l.Y + Vertices[e.Item1].Y, l.Z + Vertices[e.Item1].Z });
                    jointCorePoints[e.Item2].Add(new double[] { l.X + Vertices[e.Item2].X, l.Y + Vertices[e.Item2].Y, l.Z + Vertices[e.Item2].Z });
                }

                // Create hollow joint arms
                var startMesh = GenerateJointArm(Vertices[e.Item1], vector, startCurve.GetLength(), Offsets[e]);
                var endMesh = GenerateJointArm(Vertices[e.Item2], vector, -endCurve.GetLength(), -Offsets[Tuple.Create(e.Item2, e.Item1)]);

                separateJointMeshes[e.Item1].Add(startMesh);
                separateJointMeshes[e.Item2].Add(endMesh);

                if (label)
                {
                    // Create joint label
                    var startLabel = GenerateJointArmLabel(Vertices[e.Item1], vector, e.Item1.ToString() + ((char)(jointArmCounter[e.Item1] + 64)).ToString(), startCurve.GetLength());
                    var endLabel = GenerateJointArmLabel(Vertices[e.Item2], vector, e.Item2.ToString() + ((char)(jointArmCounter[e.Item2] + 64)).ToString(), -endCurve.GetLength());

                    jointArmCounter[e.Item1]++;
                    jointArmCounter[e.Item2]++;

                    if (startLabel != null)
                        separateJointMeshes[e.Item1].Add(startLabel);
                    if (endLabel != null)
                        separateJointMeshes[e.Item2].Add(endLabel);
                }
            }

            foreach (KeyValuePair<int, List<double[]>> kvp in jointCorePoints)
            {
                try
                {
                    var scaling = Math.Floor(1000 / kvp.Value.SelectMany(p => p).Max());
                    var convHullRes = ConvexHull.Create(kvp.Value.Select(p => p.Select(pi => pi * scaling).ToArray()).ToList());
                    var hullPoints = convHullRes.Result.Points.ToList();
                    var hullFaces = convHullRes.Result.Faces.ToList();

                    var newMesh = new Mesh();
                    newMesh.Vertices.AddVertices(hullPoints.Select(p => new Point3d(p.Position[0] / scaling, p.Position[1] / scaling, p.Position[2] / scaling)));
                    newMesh.Faces.AddFaces(hullFaces.Select(f => new MeshFace(hullPoints.IndexOf(f.Vertices[0]), hullPoints.IndexOf(f.Vertices[1]), hullPoints.IndexOf(f.Vertices[2]))));
                    newMesh.Normals.ComputeNormals();
                    newMesh.UnifyNormals();
                    newMesh.Normals.ComputeNormals();
                    newMesh.Compact();
                    separateJointMeshes[kvp.Key].Insert(0, newMesh); // Add conv hull to beginning
                }
                catch { }
            }

            foreach (KeyValuePair<int, List<Mesh>> kvp in separateJointMeshes)
            {
                if (kvp.Value.Count > 0)
                {
                    var mesh = kvp.Value.First();
                    foreach (var m in kvp.Value.Skip(1))
                    {
                        mesh.Append(m);
                    }
                    mesh.Weld(Tolerance);
                    JointMeshes[kvp.Key] = new List<Mesh>() { mesh };
                }
            }

            return JointMeshes;
        }

        private Curve GetProfile(double radius, Vector3d normal)
        {
            var profilePlane = new Plane(new Point3d(0, 0, 0), normal);
            var circle = new Circle(profilePlane, radius);
            return Polyline.CreateInscribedPolygon(circle, Sides).ToPolylineCurve();
        }

        private Mesh GenerateJointArm(Point3d origin, Vector3d direction, double length, double offset)
        {
            var mesh = new Mesh();

            Polyline outerPoly;
            Polyline innerPoly;
            var outerProfile = GetProfile(OuterWallRadius, direction);
            var innerProfile = GetProfile(InnerWallRadius, direction);
            outerProfile.TryGetPolyline(out outerPoly);
            innerProfile.TryGetPolyline(out innerPoly);

            Vector3d startLength = direction * length;
            Vector3d startOffsetLength = direction * offset;

            // V
            mesh.Vertices.Add(origin);
            mesh.Vertices.Add(Point3d.Add(origin, startOffsetLength));
            foreach (var p in outerPoly.Skip(1))
                mesh.Vertices.Add(Point3d.Add(p, origin));
            foreach (var p in outerPoly.Skip(1))
                mesh.Vertices.Add(Point3d.Add(Point3d.Add(p, origin), startLength));
            foreach (var p in innerPoly.Skip(1))
                mesh.Vertices.Add(Point3d.Add(Point3d.Add(p, origin), startOffsetLength));
            foreach (var p in innerPoly.Skip(1))
                mesh.Vertices.Add(Point3d.Add(Point3d.Add(p, origin), startLength));

            // F
            for (int i = 2; i < Sides + 1; i++)
                mesh.Faces.AddFace(0, i, i + 1);
            mesh.Faces.AddFace(0, Sides + 1, 2);

            for (int i = 2 * Sides + 2; i < 3 * Sides + 1; i++)
                mesh.Faces.AddFace(1, i, i + 1);
            mesh.Faces.AddFace(1, 3 * Sides + 1, 2 * Sides + 2);

            for (int i = 0; i < Sides; i++)
            {
                if (i < Sides - 1)
                {
                    mesh.Faces.AddFace(3 * Sides + 2 + i, Sides + 2 + i, Sides + 3 + i);
                    mesh.Faces.AddFace(3 * Sides + 2 + i, Sides + 3 + i, 3 * Sides + 3 + i);
                }
                else
                {
                    mesh.Faces.AddFace(3 * Sides + 2 + i, Sides + 2 + i, Sides + 2);
                    mesh.Faces.AddFace(3 * Sides + 2 + i, Sides + 2, 3 * Sides + 2);
                }
            }

            for (int i = 0; i < Sides; i++)
            {
                if (i < Sides - 1)
                {
                    mesh.Faces.AddFace(2 * Sides + 2 + i, 3 * Sides + 2 + i, 3 * Sides + 3 + i);
                    mesh.Faces.AddFace(2 * Sides + 2 + i, 3 * Sides + 3 + i, 2 * Sides + 3 + i);
                }
                else
                {
                    mesh.Faces.AddFace(2 * Sides + 2 + i, 3 * Sides + 2 + i, 3 * Sides + 2);
                    mesh.Faces.AddFace(2 * Sides + 2 + i, 3 * Sides + 2, 2 * Sides + 2);
                }
            }

            for (int i = 0; i < Sides; i++)
            {
                if (i < Sides - 1)
                {
                    mesh.Faces.AddFace(2 + i, Sides + 2 + i, Sides + 3 + i);
                    mesh.Faces.AddFace(2 + i, Sides + 3 + i, 3 + i);
                }
                else
                {
                    mesh.Faces.AddFace(2 + i, Sides + 2 + i, Sides + 2);
                    mesh.Faces.AddFace(2 + i, Sides + 2, 2);
                }
            }
            mesh.Normals.ComputeNormals();
            mesh.UnifyNormals();
            mesh.Normals.ComputeNormals();

            return mesh;
        }

        private Mesh GenerateJointArmLabel(Point3d origin, Vector3d direction, string label, double length)
        {
            Polyline innerPoly;
            var innerProfile = GetProfile(InnerWallRadius, direction);
            innerProfile.TryGetPolyline(out innerPoly);

            var pSideMid = (innerPoly[1] + innerPoly[0]) / 2 + origin;

            Vector3d dir1 = length < 0 ? direction : -direction;
            Vector3d dir3 = pSideMid - origin;
            Vector3d dir2 = Vector3d.CrossProduct(dir3, dir1);
            dir1.Unitize();
            dir2.Unitize();
            dir3.Unitize();

            var textHeight = Math.Sqrt(OuterWallRadius * OuterWallRadius - InnerWallRadius * InnerWallRadius) / 1.5;
            var embossHeight = (OuterWallRadius - InnerWallRadius) * 1.2;

            Vector3d startTextOffset = (length < 0 ? -direction : direction) * (Math.Abs(length) - (OuterWallRadius - InnerWallRadius));

            Point3d planeOrigin = Point3d.Add(origin, startTextOffset);
            planeOrigin = Point3d.Add(planeOrigin, dir3 * (pSideMid - origin).Length);

            JointArmLabel[label] = planeOrigin;

            var plane = new Plane(planeOrigin, dir1, dir2);
            plane.UpdateEquation();

            var style = new Rhino.DocObjects.DimensionStyle();
            style.TextHorizontalAlignment = Rhino.DocObjects.TextHorizontalAlignment.Left;
            style.TextVerticalAlignment = Rhino.DocObjects.TextVerticalAlignment.Middle;
            style.TextHeight = textHeight;
            var prefFont = Rhino.DocObjects.Font.InstalledFonts("Lucida Console");
            if (prefFont.Length > 0)
                style.Font = prefFont.First();

            var writingPlane = new Plane(planeOrigin, new Vector3d(0, 0, 1)); // Must write to flat plane for some reason

            TextEntity textEnt = TextEntity.Create(label, writingPlane, style, false, Math.Abs(length), 0);
            textEnt.SetBold(true);

            var meshes = textEnt.CreateExtrusions(style, embossHeight)
                .Where(b => b != null)
                .SelectMany(b => Mesh.CreateFromBrep(b.ToBrep(), MeshingParameters.FastRenderMesh));

            var trans = Transform.PlaneToPlane(writingPlane, plane);

            if (meshes.Count() > 0)
            {
                var mesh = meshes.First();
                foreach (var m in meshes.Skip(1))
                {
                    mesh.Append(m);
                }
                mesh.Weld(Tolerance);
                mesh.Normals.ComputeNormals();
                mesh.UnifyNormals();
                mesh.Normals.ComputeNormals();
                mesh.Transform(trans);
                return mesh;
            }

            return null;
        }
    }
}
