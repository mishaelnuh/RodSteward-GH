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
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                return "Generates 3D printed joints and standard rod structure from mesh.";
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
                return "mishael.nuh@mail.utoronto.ca";
            }
        }
    }
}
