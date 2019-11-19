# RodSteward-GH

RodSteward-GH is a Grasshopper plugin which enables users to create structures from 3D printed joints and cut dowels. It is based on the RodSteward software package developed by [alecjacobson](https://github.com/alecjacobson/). The RodSteward-GH algorithm deviates from the original software as it leverages Rhino and Grasshopper functionality. More information on the RodSteward paper can be found [here](http://www.dgp.toronto.edu/projects/rodsteward/).


## Components
1. `Mesh2Graph` ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/mesh.png): Converts meshes to a list of edges and vertices for use in the `Generator` component.
2. `Line2Graph` ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/lines.png): Converts a list lines to a list of edges and vertices for use in the `Generator` component. Vertices are automatically merged if within the set tolerance [`e`]. If multiple graphs are found, the graph with the largest number of members is outputted.
3. `Generator` ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/generator.png): Generates joint meshes and dowel lengths for manufacturing. Inputs are as follows:
    - `Edges`: Generated from other components
    - `Vertices`: Generated from other components
    - `Sides`: Number of facets of the dowel
    - `Radius`: Radius of the dowel. If faceted, the dowel is inscribed within the radius.
    - `Joint Thickness`: Thickness of joint walls.
    - `Joint Length`: Length of dowel slotted into joint.
    - `Tolerance`: Machine tolerance for manufacturing.
4. `OutputJointSTL` ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/stl.png): Exports joint meshes as STL files to target directory.