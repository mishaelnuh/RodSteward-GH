# RodSteward-GH
![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/examples/readme_example.png)

RodSteward-GH is a Grasshopper plugin which enables users to create structures from 3D printed joints and cut dowels. It is based on the RodSteward software package developed by [alecjacobson](https://github.com/alecjacobson/). The RodSteward-GH code deviates from the original software as it leverages Rhino and Grasshopper functionality. More information on the RodSteward paper can be found [here](http://www.dgp.toronto.edu/projects/rodsteward/).

## Usage
RodSteward-GH is built for Rhinoceros 6 for Windows. The plugin has not been tested for use on the Mac version.

An example Grasshopper definition can be found [here](https://github.com/mishaelnuh/RodSteward-GH/blob/master/examples/Demo.gh).

## Components
1. ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/mesh.png) `Mesh2Graph`: Converts meshes to a list of edges and vertices for use in the `Generator` component.
2. ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/lines.png) `Line2Graph`: Converts a list lines to a list of edges and vertices for use in the `Generator` component. Vertices are automatically merged if within the set tolerance [`e`]. If multiple graphs are found, the graph with the largest number of members is outputted.
3. ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/generator.png) `Generator`: Generates joint meshes and dowel lengths for manufacturing. Inputs are as follows:
    - `Edges`: Generated from other components
    - `Vertices`: Generated from other components
    - `Sides`: Number of facets of the dowel
    - `Radius`: Radius of the dowel. If faceted, the dowel is inscribed within the radius.
    - `Joint Thickness`: Thickness of joint walls.
    - `Joint Length`: Length of dowel slotted into joint.
    - `Tolerance`: Machine tolerance for manufacturing.
4. ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/stl.png) `OutputJointSTL`: Exports joint meshes as STL files to target directory.
5. ![](https://raw.githubusercontent.com/mishaelnuh/RodSteward-GH/master/icons/svg.png) `OutputRodCutSVG`: Performs bin packing and exports laser cutting plan as SVG file to target directory.
