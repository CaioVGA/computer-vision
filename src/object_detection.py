#! /usr/bin/env python

import rospy
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2
from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge


rospy.init_node('object_detection')
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--confiabilidade", type=float, default=0.2,
	help="probabilidade minima para filtrar falsos positivos")
args = vars(ap.parse_args())

CLASSES = ["background", "aviao", "bicicleta", "passaro", "barco",
	"garrafa", "onibus", "carro", "gato", "cadeira", "vaca", "mesa",
	"cachorro", "cavalo", "motocicleta", "pessoa", "vaso", "ovelha",
	"sofa", "trem", "TV"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

print("[INFO] Carregando Modelo...")

# leitura do diretorio dos arquivos de leitura
path_prototxt = r'/home/caio/catkin_ws/src/image_processing/arquivos/MobileNetSSD_deploy.caffemodel'
path_model = r'/home/caio/catkin_ws/src/image_processing/arquivos/MobileNetSSD_deploy.prototxt'
net = cv2.dnn.readNetFromCaffe(path_prototxt, path_model)

print("[INFO] Inicializando a stream de video...")
video = VideoStream(src=0).start()

time.sleep(2.0)
fps = FPS().start()

if __name__ == "__main__":
    
    rospy.init_node('publish_frames')
    pub = rospy.Publisher("/dados_frames", String, queue_size=10)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        # loop de frames do stream
        while True:
            camera = video.read()
            camera = cv2.flip(camera, 1)
            camera = imutils.resize(camera, width=600)
            (h, w) = camera.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(camera, (300, 300)), 0.007843, (300, 300), 127.5)
            net.setInput(blob)
            detections = net.forward()

            for i in np.arange(0, detections.shape[2]):
                conf = detections[0, 0, i, 2]

                if conf > args["confiabilidade"]:
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    label = "{}: {:.2f}%".format(CLASSES[idx],
                        conf * 100)
                    cv2.rectangle(camera, (startX, startY), (endX, endY),
                        COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(camera, label, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            cv2.imshow("Camera", camera)
            
            frame_update = fps.update()
            pub.publish(frame_update)
            rate.sleep()
        pub.spin()

    fps.stop()

    print("[INFO] tempo de exibicao: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    video.stop()
    video.release()
    cv2.destroyAllWindows()