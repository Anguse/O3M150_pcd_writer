/* This file provides the definition of the struct AlgoIFOutputNoDI in the interface version DIA2_1.4.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_ALGOIFOUTPUTNODI_DIA2_1_4_CONVERTER_H_INCLUDED
#define IFM_O3M_ALGOIFOUTPUTNODI_DIA2_1_4_CONVERTER_H_INCLUDED

#include "ifm_types.h"

/* The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    /*
      Magic number of project, can be used to identify the output type.
    */
    ifm_o3m_sint8_t magic_no[4];
    /*
      Magic number of struct, can be used to identify the output type.
    */
    ifm_o3m_sint8_t struct_id[4];
    /*
      Version number of struct.
    */
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
    struct
    {
        /*
          number of valid output values.
          
          Unit: [N/A]
          Range: [1,64]
        */
        ifm_o3m_sint32_t numberOfGroups;
        struct
        {
            /*
              The group ID each ROI belongs to (a ROI can only be in one single group). A value of 0 indicates an
              undefined ROI. Valid group IDs are in the range of 1..64.
              
              Unit: [N/A]
              Range: [0,64]
            */
            ifm_o3m_uint8_t group[64];
            /*
              Coordinate of the upper left corner of the rectangle (x-part).
              
              Unit: [pixel]
              Range: [0,63]
            */
            ifm_o3m_uint8_t upperLeftX[64];
            /*
              Coordinate of the upper left corner of the rectangle (y-part).
              
              Unit: [pixel]
              Range: [0,15]
            */
            ifm_o3m_uint8_t upperLeftY[64];
            /*
              Coordinate of the lower right corner of the rectangle (x-part).         
                      
              This value must be >= the upper left x value.
              
              Unit: [pixel]
              Range: [0,63]
            */
            ifm_o3m_uint8_t lowerRightX[64];
            /*
              Coordinate of the lower right corner of the rectangle (y-part).         
                      
              This value must be >= the upper left y value.
              
              Unit: [pixel]
              Range: [0,15]
            */
            ifm_o3m_uint8_t lowerRightY[64];
            /*
              Number of valid entries in above arrays.
              
              Unit: [N/A]
              Range: [1,64]
            */
            ifm_o3m_sint32_t numberOfRois;
            /*
              Minimum x-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t minCartX[64];
            /*
              Maximum x-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t maxCartX[64];
            /*
              Minimum y-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t minCartY[64];
            /*
              Maximum y-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t maxCartY[64];
            /*
              Minimum z-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t minCartZ[64];
            /*
              Maximum z-value of corresponding VOI in cartesian coordinates.
              
              Unit: [cm]
              Range: [-10000,10000]
            */
            ifm_o3m_sint16_t maxCartZ[64];
        } regionsOfInterest;
        /*
          The output mode, that defines which data is sent out over CAN.         
                  
          0: x only 1: y only 2: z only 3: amplitude only 4: x/y/z 5: x/y/z/ampl
          
          Unit: [N/A]
          Range: [0,5]
        */
        ifm_o3m_sint32_t outputMode;
        /*
          the output value type per ROI group         
                  
          0: min 1: max 2: mean 3: percentile
          
          Unit: [N/A]
          Range: [0,3]
        */
        ifm_o3m_sint8_t groupValueType[64];
        /*
          Reference value for min/max/percentile output of group.         
                  
          0: independent 1: the output pixel is selected in the cartesian x data 2: the output pixel is
          selected in the cartesian y data 3: the output pixel is selected in the cartesian z data 4: the
          output pixel is selected in the cartesian amplitude data
          
          Unit: [N/A]
          Range: [0,4]
        */
        ifm_o3m_uint8_t groupReferenceDomain[64];
        /*
          the resulting x array for all groups.         
                  
          A value of NaN indicates an invalid group result.
        */
        ifm_o3m_float32_t groupX[64];
        /*
          the resulting y array for all groups.         
                  
          A value of NaN indicates an invalid group result.
        */
        ifm_o3m_float32_t groupY[64];
        /*
          the resulting z array for all groups.         
                  
          A value of NaN indicates an invalid group result.
        */
        ifm_o3m_float32_t groupZ[64];
        /*
          the resulting amplitude array for all groups.         
                  
          A value of NaN indicates an invalid group result. The amplitude magnitude is 1/10th of the
          normalized amplitude value of the distance image result.
        */
        ifm_o3m_float32_t groupAmpl[64];
    } basicFunctionsResult;
    /*
      Timestamp of the resulting data in masterclock domain.
      
      Unit: [1e-6 s]
      Range: [0,MAX_UINT32]
    */
    ifm_o3m_uint32_t masterclockTimestamp;
    /*
      Rolling frame counter.
      
      Unit: [N/A]
      Range: [0,MAX_UINT32]
    */
    ifm_o3m_uint32_t frameCounter;
    /*
      availability bitmask         
              
      
      available == 0 : available
      (available & 1 ) != 0: interference detected
      (available & 2 ) != 0: spray detected
      (available & 4 ) != 0: tracking error (e.g. necessary vehicle data is not available)
      (available & 8 ) != 0: invalid camera orientation
      (available & 16 ) != 0: lvds cable length not detected
      (available & 32 ) != 0: internal algo error
      (available & 64 ) != 0: blockage detected
      (available & 128) != 0: calibration in progress
      
      Unit: [enum]
      Range: [0,255]
    */
    ifm_o3m_uint32_t available;
    struct
    {
        /*
          X component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
          camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
          the camera is in front of the rear axle.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transX;
        /*
          Y component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
          longitudinal center plane to the camera's reference point. Positive values indicate that the camera
          is located at the left side of the vehicle.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transY;
        /*
          Z component of the camera's translation vector expressed in world coordinates.         
                  
          If the standard vehicle coordinate system is used, this value is the distance from the ground plane
          to the camera's reference point. Positive values indicate that the camera is above the ground.
          
          Unit: [m]
          Range: [-30,30]
        */
        ifm_o3m_float32_t transZ;
        /*
          Camera rotation around the X axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotX;
        /*
          Camera rotation around the Y axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotY;
        /*
          Camera rotation around the Z axis.         
                  
          The camera orientation is expressed in so-called Euler angles.
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t rotZ;
    } cameraCalibration;
    struct
    {
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the upper left corner.
        */
        ifm_o3m_float32_t upperLeft[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the upper right corner.
        */
        ifm_o3m_float32_t upperRight[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the lower left corner.
        */
        ifm_o3m_float32_t lowerLeft[3];
        /*
          Field of view, given as unit vectors in the world coordinate system.         
                  
          This is the vector (x,y,z) corresponding to the lower right corner.
        */
        ifm_o3m_float32_t lowerRight[3];
    } fieldOfView;
    struct
    {
        /*
          Focal length x direction.         
                  
          Negative in case of mirroring enabled for x direction. Zero in case of no 2D sensor present.
          
          Unit: [px]
          Range: [-MAX_FLOAT32,+MAX_FLOAT32]
        */
        ifm_o3m_float32_t intrCalib_2D_fx;
        /*
          Focal length y direction.         
                  
          Negative in case of mirroring enabled for y direction. Zero in case of no 2D sensor present.
          
          Unit: [px]
          Range: [-MAX_FLOAT32,+MAX_FLOAT32]
        */
        ifm_o3m_float32_t intrCalib_2D_fy;
        /*
          Main point in x.
          
          Unit: [px]
          Range: [-320,+320]
        */
        ifm_o3m_float32_t intrCalib_2D_mx;
        /*
          Main point in y.
          
          Unit: [px]
          Range: [-240,+240]
        */
        ifm_o3m_float32_t intrCalib_2D_my;
        /*
          skew parameter
          
          Unit: [N/A]
          Range: [-1,+1]
        */
        ifm_o3m_float32_t intrCalib_alpha;
        /*
          First radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k1;
        /*
          Second radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k2;
        /*
          Third radial distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k5;
        /*
          First tangential distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k3;
        /*
          Second tangential distortion parameter.
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t intrCalib_k4;
        /*
          x coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_tx;
        /*
          y coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_ty;
        /*
          z coordinate of center point for extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_center_tz;
        /*
          x coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_tx;
        /*
          y coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_ty;
        /*
          z coordinate of offset to ceneter point of extrinsic calibration
          
          Unit: [N/A]
          Range: [-10,10]
        */
        ifm_o3m_float32_t extrCalib_delta_tz;
        /*
          x rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_x;
        /*
          y rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_y;
        /*
          z rotation
          
          Unit: [rad]
          Range: [-pi,pi]
        */
        ifm_o3m_float32_t extrCalib_rot_z;
    } intrExtrCalib_2d;
    /*
      Absolute position of illumination in world coordinates [X,Y,Z].
      
      Unit: [m]
      Range: [-100,100]
    */
    ifm_o3m_float32_t illuPosition[3];
    /*
      Blockage ratio (for transmission over CAN)
      
      Unit: [N/A]
      Range: [0,1]
    */
    ifm_o3m_float32_t blockageRatio;
    /*
      Flag indicating if the blockage is available for this sample.
      
      Unit: [bool]
      Range: [0,1]
    */
    ifm_o3m_uint8_t blockageAvailable;
    ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
    struct
    {
        struct
        {
            /*
              valid flag for calibration
            */
            ifm_o3m_sint32_t calibValid;
            /*
              Stable calibration counter, can be used as an acceptance criterium.
            */
            ifm_o3m_sint32_t calibrationStableCounter;
            struct
            {
                /*
                  X component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from rear axle to the
                  camera's reference point parallel to the vehicle's driving direction. Positive values indicate that
                  the camera is in front of the rear axle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transX;
                /*
                  Y component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the vehicle's
                  longitudinal center plane to the camera's reference point. Positive values indicate that the camera
                  is located at the left side of the vehicle.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transY;
                /*
                  Z component of the camera's translation vector expressed in world coordinates.         
                          
                  If the standard vehicle coordinate system is used, this value is the distance from the ground plane
                  to the camera's reference point. Positive values indicate that the camera is above the ground.
                  
                  Unit: [m]
                  Range: [-30,30]
                */
                ifm_o3m_float32_t transZ;
                /*
                  Camera rotation around the X axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotX;
                /*
                  Camera rotation around the Y axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotY;
                /*
                  Camera rotation around the Z axis.         
                          
                  The camera orientation is expressed in so-called Euler angles.
                  
                  Unit: [rad]
                  Range: [-pi,pi]
                */
                ifm_o3m_float32_t rotZ;
            } calibResult;
        } commonCalibrationResult;
        struct
        {
            /*
              Number of triangles detected.
              
              Unit: [N/A]
              Range: [0,8]
            */
            ifm_o3m_uint8_t numTrianglesDetected;
            ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
            ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
            struct
            {
                /*
                  score of triangle detection (-1: inverse match, 1: perfect match)
                  
                  Unit: [N/A]
                  Range: [-1,1]
                */
                ifm_o3m_float32_t score;
                /*
                  Cartesian 3D position of triangle center (x, y, z)
                */
                ifm_o3m_float32_t pos3D[3];
                /*
                  Corner 2D positions of triangle corners (for image overlay).         
                          
                  The first index represents the corner point index in the triangle. The second index represents the
                  x (=0) or y (=1) coordinate on the 2D image plane.
                */
                ifm_o3m_float32_t corners[3][2];
            } triangleDetections[8];
            /*
              Boolean indicating a valid Pattern Autocalibration frame.
            */
            ifm_o3m_sint32_t frameValid;
            /*
              Mean calibration reprojection error per frame.
              
              Unit: [m]
              Range: [0, 0.25]
            */
            ifm_o3m_float32_t frameReprojectError;
        } pacCalibrationResult;
        struct
        {
            /*
              Set to 1 if street estimation is considered valid and may be used for further processing.
            */
            ifm_o3m_sint32_t planeValid;
            struct
            {
                /*
                  Pitch angle of the camera in [rad].
                */
                ifm_o3m_float32_t pitchAngle;
                /*
                  Roll angle of the camera in [rad].
                */
                ifm_o3m_float32_t rollAngle;
                /*
                  height of the camera in [m].
                */
                ifm_o3m_float32_t camHeight;
                /*
                  normal vector of the currently estimated road plane (x component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalx;
                /*
                  normal vector of the currently estimated road plane (y component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normaly;
                /*
                  normal vector of the currently estimated road plane (z component)         
                          
                  This is derived directly from pitchAngle and rollAngle and is provided for convenience.
                */
                ifm_o3m_float32_t normalz;
            } planeEstimation;
            /*
              Plausibility [0,1] for the estimated plane parameters.
            */
            ifm_o3m_float32_t plausibility;
            /*
              Standard deviation of distance to plane of all pixels, which are assigned as street pixels.
            */
            ifm_o3m_float32_t distanceDeviation;
        } streetCalibrationResult;
    } calibrationResult;
    struct
    {
        /*
          the digitial outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.digitalOutput[]". Using this pattern, it is possible to treat
          the digital outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_uint8_t digitalOutput[100];
        /*
          the analog outputs of the logic graph         
                  
          This output should be assigned with PVO nodes addressing ChannelID 2, expression
          "AlgoState.algoLogicState.ifOutput.analogOuptut[]". Using this pattern, it is possible to treat the
          analog outputs as a kind of external memory for the logic graph.
        */
        ifm_o3m_float32_t analogOutput[20];
    } logicOutput;
} ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4;

        
/* Casts the buffer to ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* ifm_o3m_ConvertBufferToLittleEndian_DIA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* ifm_o3m_ConvertBufferToBigEndian_DIA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
