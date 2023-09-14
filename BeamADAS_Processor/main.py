import usb.core
import struct
import cv2
import numpy as np

# Find your USB device by VID and PID
dev = usb.core.find(idVendor=your_vendor_id, idProduct=your_product_id)

if dev is None:
    raise ValueError("Device not found")

while True:
    # receive data
    # Receive data from the USB device
    received_data = dev.read(endpoint, size_to_receive, timeout=1000)

    # Parse the header to get image size and format
    header_size = struct.calcsize('<II')
    image_size, image_format = struct.unpack('<II', received_data[:header_size])

    # Extract the image data
    image_data = received_data[header_size:]

    # Convert image data to a suitable format for OpenCV
    np_array = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(np_array, image_format)

    # Now 'image' contains the image in a format suitable for OpenCV
    # do work
    # send results
    break

# Close the USB device
usb.util.dispose_resources(dev)