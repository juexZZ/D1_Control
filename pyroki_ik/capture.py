import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Enable depth stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# Enable color stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Adding alignment objects (after pipeline startup)
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Wait for a coherent frame set
        frames = pipeline.wait_for_frames()

        # Align depth frames to RGB coordinate system
        aligned_frames = align.process(frames)
        
        # Get both depth and color frames after alignment
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
            
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Apply colormap on depth image for better visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        # Stack both images horizontally for display
        images = np.hstack((color_image, depth_colormap))
        
        # Show images
        cv2.imshow('RealSense Depth and Color Streams', images)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite(f'rgb.png', color_image)
            cv2.imwrite(f'depth.png', depth_image)
            cv2.imwrite(f'depth_colormap.png', depth_colormap)
            print('Image saved successfully!')
            break
            
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()