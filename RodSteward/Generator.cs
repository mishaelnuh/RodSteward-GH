using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino.Geometry;
using GH_IO.Serialization;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;

namespace RodSteward
{
    public class Generator : GH_Component
    {
        private Model model;

        public bool PrintLabel = false;
        public bool Collision = false;
        public bool AnnotateRods = false;
        public bool AnnotateJoints = false;
        public bool AnnotateJointArms = false;

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

            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
            pManager[6].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Model", "M", "Model object", GH_ParamAccess.item);
            pManager.AddMeshParameter("Joints", "J", "Joint meshes for structure", GH_ParamAccess.tree);
            pManager.AddMeshParameter("Rods", "R", "Rod meshes for structure", GH_ParamAccess.list);
            pManager.AddCurveParameter("Rod Curves", "RC", "Rod centreline curves", GH_ParamAccess.list);

            ((IGH_PreviewObject)pManager[0]).Hidden = true;
            ((IGH_PreviewObject)pManager[1]).Hidden = true;
            ((IGH_PreviewObject)pManager[2]).Hidden = true;
            ((IGH_PreviewObject)pManager[3]).Hidden = true;
        }
        public override bool Write(GH_IWriter writer)
        {
            try
            {
                writer.SetBoolean("printlabel", PrintLabel);
                writer.SetBoolean("collision", Collision);
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
                reader.TryGetBoolean("printlabel", ref PrintLabel);
                reader.TryGetBoolean("collision", ref Collision);
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

            foreach (var kvp in model.RodMeshes)
            {
                args.Display.DrawMeshShaded(kvp.Value, model.ClashedRods.Contains(kvp.Key) ? errorMaterial : rodMaterial);
            }

            foreach (var kvp in model.JointMeshes)
            {
                foreach (var b in kvp.Value)
                    args.Display.DrawMeshShaded(b, model.ClashedJoints.Contains(kvp.Key) ? errorMaterial : jointMaterial);
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

            model = new Model();

            if (!DA.GetDataList(0, edges)) { return; }
            if (!DA.GetDataList(1, vertices)) { return; }
            if (!DA.GetData(2, ref sides)) { sides = model.Sides; }
            if (!DA.GetData(3, ref radius)) { radius = model.Radius; }
            if (!DA.GetData(4, ref jointThickness)) { jointThickness = model.JointThickness; }
            if (!DA.GetData(5, ref jointLength)) { jointLength = model.JointLength; }
            if (!DA.GetData(6, ref tolerance)) { tolerance = model.Tolerance; }

            if (edges == null || vertices == null) { return; }
            if (radius <= 0 || sides <= 2 || jointThickness < 0 || jointLength < 0 || tolerance < 0) { throw new Exception("Invalid input."); }

            model.Edges = edges;
            model.Vertices = vertices;
            model.Sides = (int)Math.Floor(sides);
            model.Radius = radius;
            model.JointThickness = jointThickness;
            model.JointLength = jointLength;
            model.Tolerance = tolerance;

            model.Generate();
            model.CalculateClashes();

            DA.SetData(0, model);
            DA.SetDataTree(1, model.JointMeshTree);
            DA.SetDataList(2, model.RodMeshes.Values.ToList());
            DA.SetDataList(3, model.RodCentrelines.Values.ToList());
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

        private List<System.Drawing.Rectangle> CapsuleBounds { get; set; }

        protected override void Layout()
        {
            base.Layout();

            int numCapsules = 7;
            int capsuleHeight = 22;

            System.Drawing.Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
            rec0.Height += 22 * numCapsules;

            CapsuleBounds = new List<System.Drawing.Rectangle>();

            for(int i = 0; i < numCapsules; i++)
            {
                System.Drawing.Rectangle rec = rec0;
                rec.Y = rec.Bottom - 22 * (numCapsules - i);
                rec.Height = capsuleHeight;
                rec.Inflate(-2, -2);
                CapsuleBounds.Add(rec);
            }

            Bounds = rec0;
        }

        protected override void Render(GH_Canvas canvas, System.Drawing.Graphics graphics, GH_CanvasChannel channel)
        {
            base.Render(canvas, graphics, channel);

            if (channel == GH_CanvasChannel.Objects)
            {
                var component = Owner as Generator;

                // Options
                var optionTitle = GH_Capsule.CreateTextCapsule(CapsuleBounds[0], CapsuleBounds[0], GH_Palette.Normal, "Options", 0, 1);
                graphics.DrawString((component.PrintLabel ? "☒" : "☐") + " Print label", optionTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[1].Location);
                graphics.DrawString((component.Collision ? "☒" : "☐") + " Collision", optionTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[2].Location);

                optionTitle.Render(graphics, Selected, Owner.Locked, false);
                optionTitle.Dispose();

                // Annotation
                var annotationTitle = GH_Capsule.CreateTextCapsule(CapsuleBounds[3], CapsuleBounds[3], GH_Palette.Normal, "Annotation", 0, 1);
                graphics.DrawString((component.AnnotateRods ? "☒" : "☐") + " Rods", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[4].Location);
                graphics.DrawString((component.AnnotateJoints ? "☒" : "☐") + " Joints", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[5].Location);
                graphics.DrawString((component.AnnotateJointArms ? "☒" : "☐") + " Joint arms", annotationTitle.Font, System.Drawing.Brushes.Black, CapsuleBounds[6].Location);

                annotationTitle.Render(graphics, Selected, Owner.Locked, false);
                annotationTitle.Dispose();
            }
        }
        public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
        {
            if (e.Button == System.Windows.Forms.MouseButtons.Left)
            {
                var component = Owner as Generator;

                if (((System.Drawing.RectangleF)CapsuleBounds[1]).Contains(e.CanvasLocation))
                {
                    component.PrintLabel = !component.PrintLabel;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[2]).Contains(e.CanvasLocation))
                {
                    component.Collision = !component.Collision;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[4]).Contains(e.CanvasLocation))
                {
                    component.AnnotateRods = !component.AnnotateRods;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[5]).Contains(e.CanvasLocation))
                {
                    component.AnnotateJoints = !component.AnnotateJoints;
                    component.ExpireSolution(true);
                    return GH_ObjectResponse.Handled;
                }
                else if (((System.Drawing.RectangleF)CapsuleBounds[6]).Contains(e.CanvasLocation))
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
