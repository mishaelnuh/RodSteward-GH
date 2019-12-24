﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino.Geometry;
using System.IO;
using IxMilia.Stl;
using Grasshopper;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace RodSteward
{
    public class OutputJointSTL : GH_Component
    {
        private string outputMessage = "";
        public OutputJointSTL()
          : base("OutputJointSTL", "RSOutputJointSTL",
              "Outputs joint mesh STLs to the target directory",
              "RodSteward", "Output")
        {
        }

        public override Guid ComponentGuid => new Guid("a63df5b2-aeea-4a84-9a3c-022793d433b0");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Model", "M", "Model object", GH_ParamAccess.item);
            pManager.AddTextParameter("Directory", "D", "Output directory", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Write Trigger", "T", "Triggers output", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Output", "O", "Output message", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Model model = new Model();
            string dir = null;
            bool trigger = false;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref dir)) { return; }
            if (!DA.GetData(2, ref trigger)) { return; }

            if (!trigger) {
                DA.SetData(0, outputMessage);
                return;
            }

            if (model == null) { return; }

            if (!Directory.Exists(dir))
            {
                outputMessage = "[" + DateTime.Now.ToString() + "]: Output directory does not exists.";
                DA.SetData(0, outputMessage);
                return;
            }

            var dirFiles = Directory.GetFiles(dir);
            foreach (var file in dirFiles.Where(f => f.Contains("RS_Joint_Mesh_")))
                File.Delete(file);

            int fileCounter = 0;

            foreach(var kvp in model.JointMeshes)
            {
                var stlFile = new StlFile();
                stlFile.SolidName = "RS_Joint_Mesh_" + kvp.Key.ToString();

                foreach(var m in kvp.Value)
                {
                    var vertices = m.Vertices.Select(v => new StlVertex(v.X, v.Y, v.Z)).ToList();
                    var faces = m.Faces;
                    var normals = m.FaceNormals.Select(n => new StlNormal(n.X, n.Y, n.Z)).ToList();

                    for(int f = 0; f < faces.Count(); f++)
                    {
                        stlFile.Triangles.Add(new StlTriangle(
                            normals[f],
                            vertices[faces[f].A],
                            vertices[faces[f].B],
                            vertices[faces[f].C]));
                        if (faces[f].IsQuad)
                        {
                            stlFile.Triangles.Add(new StlTriangle(
                                normals[f],
                                vertices[faces[f].A],
                                vertices[faces[f].C],
                                vertices[faces[f].D]));
                        }
                    }
                }

                if (stlFile.Triangles.Count > 0)
                {
                    using (FileStream fs = File.Create(Path.Combine(dir, stlFile.SolidName + ".stl")))
                    {
                        stlFile.Save(fs);
                        fileCounter++;
                    }
                }
            }

            outputMessage = "[" + DateTime.Now.ToString() + "]: Successfully wrote " + fileCounter.ToString() + " STL files.";
            DA.SetData(0, outputMessage);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.stl;
            }
        }
    }
}
