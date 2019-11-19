using System;
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
using GH_IO.Types;
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
            pManager.AddMeshParameter("Joint", "J", "Joint meshes from Generator component", GH_ParamAccess.tree);
            pManager.AddTextParameter("Directory", "D", "Output directory", GH_ParamAccess.item);
            pManager.AddBooleanParameter("WriteTrigger", "T", "Triggers output", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Output", "O", "Output message", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Structure<GH_Mesh> data = new GH_Structure<GH_Mesh>();
            string dir = null;
            bool trigger = false;

            if (!DA.GetDataTree(0, out data)) { return; }
            if (!DA.GetData(1, ref dir)) { return; }
            if (!DA.GetData(2, ref trigger)) { return; }

            if (!trigger) {
                DA.SetData(0, outputMessage);
            }

            if (data == null) { return; }
            if (data.PathCount == 0) { return; }

            if (!Directory.Exists(dir))
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Output directory does not exists.");
                return;
            }

            var dirFiles = Directory.GetFiles(dir);
            foreach (var file in dirFiles.Where(f => f.Contains("RS_Joint_Mesh_")))
                File.Delete(file);

            int fileCounter = 0;
            for(int i = 0; i < data.Branches.Count(); i++)
            {
                var stlFile = new StlFile();
                stlFile.SolidName = "RS_Joint_Mesh_" + i.ToString();
                foreach(var m in data.Branches[i])
                {
                    var vertices = m.Value.Vertices.Select(v => new StlVertex(v.X, v.Y, v.Z)).ToList();
                    var faces = m.Value.Faces;
                    var normals = m.Value.FaceNormals.Select(n => new StlNormal(n.X, n.Y, n.Z)).ToList();

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

            outputMessage = "Successfully wrote " + fileCounter.ToString() + "STL files.";
            DA.SetData(0, outputMessage);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.mesh;
            }
        }
    }
}
