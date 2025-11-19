## Usage Limitations of GMSL Cameras



GMSL cameras interface with various deserializer chips such as MAX9296 and MAX92716. Orbbec GMSL cameras support multiple streams including depth, color, IR, and IMU data, but certain usage limitations apply:

- GMSL only supports V4L2 and YUYV format; MJPG format is not supported. RGB output is derived from YUYV format conversion.
- Metadata for Gemini-335Lg is provided via a separate node, while metadata for other models is embedded within video frames, which remains transparent to users.
- When using the Max96712 as a deserializer chip, due to the characteristics of the Max96712 chip, a multi - machine synchronous trigger signal must be provided in the secondary_synced mode. Otherwise, data flow interruption will occur when switching the data stream
- Two cameras connected on the same MAX9296, MAX96712 LinkA/B, or MAX96712 LinkC/D have the following limitations:
  - Before driver version v1.2.02, there was a restriction that the RGB of one camera and the right IR of another camera could not stream simultaneously. After driver version v1.2.02, the restriction was modified to that the RGB of one camera and the left IR of another camera cannot stream simultaneously.
  - Before driver version v1.2.02, there was a restriction that the DEPTH of one camera and the left IR of another camera could not stream simultaneously. After driver version v1.2.02, the restriction was modified to that the DEPTH of one camera and the right IR of another camera cannot stream simultaneously.
  - The combined maximum number of active streams from both cameras is limited to four (satisfying the above two conditions ensures compliance).

For further known limitations, please refer to [Usage Limitations of Orbbec GMSL Cameras](https://github.com/orbbec/MIPI_Camera_Platform_Driver/blob/main/doc/Instructions%20for%20Using%20GMSL%20Camera.md)
