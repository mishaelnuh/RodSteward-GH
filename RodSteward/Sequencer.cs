using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using GH_IO.Serialization;

namespace RodSteward
{
    public class Sequencer : GH_Component
    {
        private Model model;

        private List<Tuple<char, int>> orderedParts = new List<Tuple<char, int>>();
        private List<int> traversedEdges = new List<int>();
        private List<int> traversedVertices = new List<int>();
        private Queue<int> vertexQueue = new Queue<int>();

        public bool AnnotateRods = true;
        public bool AnnotateJoints = true;
        public bool AnnotateJointArms = true;

        public Sequencer()
          : base("Sequencer", "RSSequencer",
              "Sequences parts for construction",
              "RodSteward", "Sequencer")
        {
        }
        public override void CreateAttributes()
        {
            m_attributes = new SequencerAttribute(this);
        }

        public override Guid ComponentGuid => new Guid("1ba1532c-b1c9-4e77-9d3b-55b400a1f2eb");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Model", "M", "Model object", GH_ParamAccess.item);
            pManager.AddNumberParameter("Starting Joint", "SJ", "First joint to add", GH_ParamAccess.item);
            pManager.AddNumberParameter("Part Number", "PN", "Part number to add", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Output", "O", "Output message", GH_ParamAccess.item);
        }
        
        public override bool Write(GH_IWriter writer)
        {
            try
            {
                writer.SetBoolean("annotaterods", AnnotateRods);
                writer.SetBoolean("annotatejoints", AnnotateJoints);
                writer.SetBoolean("annotatejointarms", AnnotateJointArms);
            }
            catch (Exception err)
            {
                throw err;
            }
            return base.Write(writer);
        }

        public override bool Read(GH_IReader reader)
        {
            try
            {
                reader.TryGetBoolean("annotaterods", ref AnnotateRods);
                reader.TryGetBoolean("annotatejoints", ref AnnotateJoints);
                reader.TryGetBoolean("annotatejointarms", ref AnnotateJointArms);
            }
            catch (Exception err)
            {
                throw err;
            }
            return base.Read(reader);
        }
        
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (model == null)
                return;

            var errorMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Red, 0.3);
            var rodMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.BurlyWood, 0.3);
            var jointMaterial = new Rhino.Display.DisplayMaterial(System.Drawing.Color.Black, 0.3);

            foreach (var i in traversedEdges)
            {
                var conn = model.Edges[i];
                args.Display.DrawMeshShaded(model.RodMeshes[conn], model.ClashedRods.Contains(conn) ? errorMaterial : rodMaterial);
            }

            foreach (var i in traversedVertices)
            {
                var jointMeshes = model.JointMeshes[i];
                foreach (var b in jointMeshes)
                    args.Display.DrawMeshShaded(b, model.ClashedJoints.Contains(i) ? errorMaterial : jointMaterial);
            }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            if (model == null)
                return;

            if (orderedParts.Count <= 0)
                return;

            var lastPart = orderedParts.Last();

            if(lastPart.Item1 == 'E' && AnnotateRods)
            {
                var r = model.RodCentrelines[model.Edges[lastPart.Item2]];

                var midPoint = r.PointAtNormalizedLength(0.5);
                var lengthString = String.Format("{0:F2}", r.GetLength());

                args.Display.Draw2dText(lengthString, System.Drawing.Color.Blue, midPoint, false, 20, "Lucida Console");
            }
            else
            {
                var v = model.Vertices[lastPart.Item2];

                if (AnnotateJoints)
                    args.Display.Draw2dText(lastPart.Item2.ToString(), System.Drawing.Color.Orange, v, false, 20, "Lucida Console");

                if (AnnotateJointArms)
                {
                    var armLabels = model.JointArmLabel.Where(j => j.Key.StartsWith(lastPart.Item2.ToString()));

                    foreach (var kvp in armLabels)
                    {
                        args.Display.Draw2dText(kvp.Key, System.Drawing.Color.Green, kvp.Value, false, 20, "Lucida Console");
                    }
                }
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double start = 0;
            double num = 0;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref start)) { return; }
            if (!DA.GetData(2, ref num)) { return; }

            if (model == null) { return; }

            int startFloored = (int)Math.Floor(start);
            int numFloored = (int)Math.Floor(num);

            orderedParts.Clear();
            traversedEdges.Clear();
            traversedVertices.Clear();
            vertexQueue.Clear();

            if (startFloored < 0 || startFloored >= model.Vertices.Count()) { return; }
            if (numFloored < 0) { return; }

            if (numFloored >= 0)
            {
                orderedParts.Add(Tuple.Create('V', startFloored));
                traversedVertices.Add(startFloored);
                SearchFromVertex(startFloored, numFloored);
            }

            if (orderedParts.Count() > 0)
            {
                if (orderedParts.Last().Item1 == 'E')
                {
                    DA.SetData(0, "Rod: " + String.Format("{0:F2}", model.RodCentrelines[model.Edges[orderedParts.Last().Item2]].GetLength()));
                }
                else
                {
                    DA.SetData(0, "Joint: " + orderedParts.Last().Item2.ToString());
                }
            }
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.sequencer;
            }
        }

        private void SearchFromVertex(int vertex, int target)
        {
            if (orderedParts.Count() > target)
                return;

            var nextEdgeIndices = model.Edges
                .Where(e => e.Item1 == vertex || e.Item2 == vertex)
                .Select(e => model.Edges.IndexOf(e))
                .Where(i => !traversedEdges.Contains(i))
                .ToList();

            foreach(var e in nextEdgeIndices)
            {
                orderedParts.Add(Tuple.Create('E', e));
                traversedEdges.Add(e);

                if (orderedParts.Count() > target)
                    return;

                var nextVertex = model.Edges[e].Item1 == vertex ? model.Edges[e].Item2 : model.Edges[e].Item1;

                if (!traversedVertices.Contains(nextVertex))
                {
                    orderedParts.Add(Tuple.Create('V', nextVertex));
                    traversedVertices.Add(nextVertex);

                    if (orderedParts.Count() > target)
                        return;

                    vertexQueue.Enqueue(nextVertex);
                }
            }

            if (vertexQueue.Count() > 0)
                SearchFromVertex(vertexQueue.Dequeue(), target);
        }
    }

    public class SequencerAttribute : Grasshopper.Kernel.Attributes.GH_ComponentAttributes
    {
        public SequencerAttribute(GH_Component owner) : base(owner) { }

        private List<System.Drawing.Rectangle> CapsuleBounds { get; set; }

        protected override void Layout()
        {
            base.Layout();

            int numCapsules = 4;
            int capsuleHeight = 22;

            System.Drawing.Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
            rec0.Height += 22 * numCapsules;

            CapsuleBounds = new List<System.Drawing.Rectangle>();

            for(int i = 0; i < numCapsules; i++)
            {
                System.Drawing.Rectangle rec = rec0;
                rec.Y = rec.Bottom - 22 * (numCapsules - i);
                rec.Height = capsuleHeight;
                rec.Inflate(0, -2);
                CapsuleBounds.Add(rec);
            }

            Bounds = rec0;
        }

        protected override void Render(GH_Canvas canvas, System.Drawing.Graphics graphics, GH_CanvasChannel channel)
        {
            base.Render(canvas, graphics, channel);

            if (channel == GH_CanvasChannel.Objects)
            {
                var component = Owner as Sequencer;

                // Annotation
                var annotationTitle = GH_Capsule.CreateTextCapsule(CapsuleBounds[0], CapsuleBounds[0], GH_Palette.Grey, "Annotation", 0, 1);
                graphics.DrawString((component.AnnotateRods ? "☒" : "☐") + " Rods", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[1].Location);
                graphics.DrawString((component.AnnotateJoints ? "☒" : "☐") + " Joints", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[2].Location);
                graphics.DrawString((component.AnnotateJointArms ? "☒" : "☐") + " Joint arms", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[3].Location);

                annotationTitle.Render(graphics, Selected, Owner.Locked, false);
                annotationTitle.Dispose();
            }
        }
        public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Left)
            {
                var component = Owner as Sequencer;

                if (((System.Drawing.RectangleF)CapsuleBounds[1]).Contains(e.CanvasLocation))
                {
                    component.AnnotateRods = !component.AnnotateRods;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[2]).Contains(e.CanvasLocation))
                {
                    component.AnnotateJoints = !component.AnnotateJoints;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[3]).Contains(e.CanvasLocation))
                {
                    component.AnnotateJointArms = !component.AnnotateJointArms;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
            }

            return base.RespondToMouseDown(sender, e);
        }
    }
}
