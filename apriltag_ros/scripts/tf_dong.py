# #! /usr/bin/env python
# # import apriltag
# # # import pupil_apriltags as apriltag
# # import cv2
# # import matplotlib.pyplot as plt
# #
# # procfile = "/home/nav/Desktop/tag/0001.png"
# # img = cv2.imread(procfile)
# # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# #
# # plt.clf()
# # plt.figure(figsize=(12,12))
# # plt.axis("off")
# # plt.imshow(gray, cmap=plt.cm.gray)
# # plt.show()
# # atd = apriltag.Detector(families='tag36h11',
# #                         nthreads=1,
# #                        quad_decimate=1.0,
# #                        quad_sigma=0.0,
# #                        refine_edges=1,
# #                        decode_sharpening=0.25,
# #                        debug=0)
# # tags = atd.detect(gray)
# # print("%d apriltags have been detected."%len(tags))
# # for tag in tags:
# #     cv2.circle(img, tuple(tag.corners[0].astype(int)), 4,(255,0,0), 2) # left-top
# #     cv2.circle(img, tuple(tag.corners[1].astype(int)), 4,(255,0,0), 2) # right-top
# #     cv2.circle(img, tuple(tag.corners[2].astype(int)), 4,(255,0,0), 2) # right-bottom
# #     cv2.circle(img, tuple(tag.corners[3].astype(int)), 4,(255,0,0), 2) # left-bottom
# #
# # cv2.imshow("apriltag_test",img)
# # cv2.waitKey()
# import cv2
# import numpy as np
# from apriltag import apriltag
#
# imagepath = '/home/nav/Desktop/tag/0001.png'
# image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
# detector = apriltag("tag36h11")
#
# detections = detector.detect(image)
# print(detections)