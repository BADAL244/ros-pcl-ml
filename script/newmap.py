import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from gray2color import gray2color
import random

def generate_color():
    r , g , b = (int(random.randint(0, 255)) for c in range(3))
    return r,g ,b

UPDATE_MIN, UPDATE_MAX = False, False
new_indices = 0
def decode_seg_image(image , class_length):
    # print(image)
    class_label_map = np.array([[0, 0, 0],
      [120, 120, 120],
      [180, 120, 120],
      [6, 230, 230],
      [80, 50, 50],
      [4, 200, 3],
      [120, 120, 80],
      [140, 140, 140],
      [204, 5, 255],
      [184, 255, 0],
      [4, 250, 7],
      [224, 5, 255],
      [235, 255, 7],
      [150, 5, 61],
      [120, 120, 70],
      [8, 255, 51],
      [255, 6, 82],
      [143, 255, 140],
      [204, 255, 4],
      [255, 51, 7],
      [204, 70, 3],
      [0, 102, 200],
      [61, 230, 250],
      [255, 6, 51],
      [11, 102, 255],
      [255, 7, 71],
      [255, 9, 224],
      [9, 7, 230],
      [220, 220, 220],
      [255, 9, 92],
      [112, 9, 255],
      [8, 255, 214],
      [7, 255, 224],
      [255, 184, 6],
      [10, 255, 71],
      [255, 41, 10],
      [7, 255, 255],
      [224, 255, 8],
      [102, 8, 255],
      [255, 61, 6],
      [255, 194, 7],
      [255, 122, 8],
      [0, 255, 20],
      [255, 8, 41],
      [255, 5, 153],
      [6, 51, 255],
      [235, 12, 255],
      [160, 150, 20],
      [0, 163, 255],
      [140, 140, 140],
      [250, 10, 15],
      [20, 255, 0],
      [31, 255, 0],
      [255, 31, 0],
      [255, 224, 0],
      [153, 255, 0],
      [0, 0, 255],
      [255, 71, 0],
      [0, 235, 255],
      [0, 173, 255],
      [31, 0, 255],
      [11, 200, 200],
      [255, 82, 0],
      [0, 255, 245],
      [0, 61, 255],
      [0, 255, 112],
      [0, 255, 133],
      [255, 0, 0],
      [255, 163, 0],
      [255, 102, 0],
      [194, 255, 0],
      [0, 143, 255],
      [51, 255, 0],
      [0, 82, 255],
      [0, 255, 41],
      [0, 255, 173],
      [10, 0, 255],
      [173, 255, 0],
      [0, 255, 153],
      [255, 92, 0],
      [255, 0, 255],
      [255, 0, 245],
      [255, 0, 102],
      [255, 173, 0],
      [255, 0, 20],
      [255, 184, 184],
      [0, 31, 255],
      [0, 255, 61],
      [0, 71, 255],
      [255, 0, 204],
      [0, 255, 194],
      [0, 255, 82],
      [0, 10, 255],
      [0, 112, 255],
      [51, 0, 255],
      [0, 194, 255],
      [0, 122, 255],
      [0, 255, 163],
      [255, 153, 0],
      [0, 255, 10],
      [255, 112, 0],
      [143, 255, 0],
      [82, 0, 255],
      [163, 255, 0],
      [255, 235, 0],
      [8, 184, 170],
      [133, 0, 255],
      [0, 255, 92],
      [184, 0, 255],
      [255, 0, 31],
      [0, 184, 255],
      [0, 214, 255],
      [255, 0, 112],
      [92, 255, 0],
      [0, 224, 255],
      [112, 224, 255],
      [70, 184, 160],
      [163, 0, 255],
      [153, 0, 255],
      [71, 255, 0],
      [255, 0, 163],
      [255, 204, 0],
      [255, 0, 143],
      [0, 255, 235],
      [133, 255, 0],
      [255, 0, 235],
      [245, 0, 255],
      [255, 0, 122],
      [255, 245, 0],
      [10, 190, 212],
      [214, 255, 0],
      [0, 204, 255],
      [20, 0, 255],
      [255, 255, 0],
      [0, 153, 255],
      [0, 41, 255],
      [0, 255, 204],
      [41, 0, 255],
      [41, 255, 0],
      [173, 0, 255],
      [0, 245, 255],
      [71, 0, 255],
      [122, 0, 255],
      [0, 255, 184],
      [0, 92, 255],
      [230, 230, 230],
      [0, 133, 255],
      [255, 214, 0],
      [25, 194, 194],
      [102, 255, 0],
      [92, 0, 255]])
    print(class_label_map.shape)
    r = np.zeros_like(image).astype(np.uint8)
    g = np.zeros_like(image).astype(np.uint8)
    b = np.zeros_like(image).astype(np.uint8)
    # indices = np.unique(image)
    # new_indices = indices.shape[0]
    for l in range(0 , 151):
        idx = image ==  l
        #print(idx.shape)
        # print(class_label_map[l,0])
        r[idx] = class_label_map[l,0]
        g[idx] = class_label_map[l,1]
        b[idx] = class_label_map[l,2]
        #print(r)
        rgb = np.stack([r,g,b] , axis =-1)
    return rgb

def republish_image(image):
    global min_value, max_value
    array = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, image.width)

    min_value = min(np.min(array).item(), min_value) if UPDATE_MIN else min_value
    max_value = max(np.max(array).item(), max_value) if UPDATE_MAX else max_value
    array = np.clip((array - min_value) / (max_value - min_value + sys.float_info.min) * 255, 0, 255 , dtype = "uint8" , casting='unsafe')
    # array.reshape((array.shape[0] , array.shape[1]))
    image_output = np.argmax(array , axis = 0)
    indices = np.unique(array).tolist()
    # indices = int(indices)
    print(indices)
    # print(type(indices.shape[0]))
    # new_indices = indices.shape[0]
    # if indices.shape[0]> new_indices:
    #     new_indices = indices.len
    # print(new_indices)
    image_new = decode_seg_image(array , len(indices))
    cv2.imshow("data" , image_new)
    cv2.waitKey(1)
    # im_gray =array.convert('L')
    # image = gray2color(im_gray, use_pallet='cityscape', custom_pallet=None).tobytes()
    print(image_new.shape)
    image.data = image_new.astype(np.uint8).tobytes()
    image.encoding = "rgb8"
    pub.publish(image)


if __name__ == "__main__":

    rospy.init_node("convert_32SC1_to_8UC1")

    # get configuration from ROS parameters
    input_topic = rospy.get_param("~input_topic", "/semantic")
    output_topic = rospy.get_param("~output_topic", "/semantic_republished")
    min_value = rospy.get_param("~min_value", None)
    max_value = rospy.get_param("~max_value", None)

    UPDATE_MIN = min_value is None
    UPDATE_MAX = max_value is None
    min_value = np.Inf if min_value is None else min_value
    max_value = -np.Inf if max_value is None else max_value

    # topics
    sub = rospy.Subscriber(input_topic, Image, republish_image)
    pub = rospy.Publisher(output_topic, Image, queue_size=10)

    # spin the node
    rospy.spin()
