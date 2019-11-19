using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace RodSteward
{
    public class RodStewardInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "RodSteward";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                return Properties.Resources.rs;
            }
        }
        public override string Description
        {
            get
            {
                return "Generates 3D printed joints and standard rod structure.";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("11a503d9-aed4-4c6a-9ff8-020c8a1df3f5");
            }
        }

        public override string AuthorName
        {
            get
            {
                return "Mishael Nuh";
            }
        }
        public override string AuthorContact
        {
            get
            {
                return "mishael.ebel.nuh@gmail.com";
            }
        }
    }
}
