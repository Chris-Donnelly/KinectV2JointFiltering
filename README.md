# KinectV2JointFiltering
## Kinect V2 ("Kinect for Windows") joint/skeleton classes

Classes to filter Kinect V2 *(Kinect for Windows)* skeletal data

**Requirements:**

- Kinect For Windows / Kinect V2 SDK (try [here](https://www.microsoft.com/en-gb/download/details.aspx?id=44561))
- GLM libraries (either download and include manually or use Nuget in Visual Studio)

**About the work**

This code was used in my MSc dissertation to filter skeletal data for a Virtual Reality gesture-based painting program; it's quite custom and frankly both unoptimised and may be very difficult to follow, but I'll try explain the context of how I used it.

The *skeleton.h* file contains a map of booleans which tracks which skeleton joints we wish to use (simply set a joint to false, such as the head joint or for instance left elbow, and the joint will not be processed). Individual joints (from Kinect sensor/SDK) are added as Vec3 (glm type, an XYZ vector in millimetres) to individual filtered joint classes (*passthrough*, *averaged* and *double exponential*) - each class performing filtering as needed.

Skeletal grounding was also applied to 'ground' the skeleton to a floor plane to alleviate the need to measure the detected floor plane (which due to the nature of light diffusion and viewing limits can fluctuate). This simply translates the data to a given XZ plane at height = Y. From here, the data in each filtered joint class will be extracted (and filtered, grounded as specified) as a Vec3 type (again, an XYZ vector in millimetre space).

Multiple types of filtered joints can be combined to perform different filters for different body joints (such as averaging for arms, passthrough for legs, etc)

*Please note that a Z-reversal effect is applied to the skeleton to avoid mirroring for my specific application.*

*Initially written in Visual Studio 2015 - Using the Kinect V2 SDK and GLM*
