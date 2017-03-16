import cv2
import cv2.cv as cv
import sys

Notebook_webcam = 0
USB_webcam = 2 # 1

DISPLAY_VIDEO = True
DETECT_HUMANS = False

def detect_faces(img, cascade):
	rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
	if len(rects) == 0:
		return None
	rects[:,2:] += rects[:,:2]
	return rects

def detect_humans(img, hog):
	(rects, weights) = hog.detectMultiScale(img, winStride=(4, 4), padding=(8, 8), scale=1.05)
	if len(rects) == 0:
		return None
	return rects

def get_first_coordinates(rects):
	if rects is not None:
		## Extract face coordinates			
		x1 = rects[0][1]
		y1 = rects[0][0]
		x2 = rects[0][3]
		y2 = rects[0][2]
		
		centre_x = (x1 + x2)/2
		centre_y = (y1 + y2)/2
		return x1, y1, x2, y2, centre_x, centre_y
	else:
		return None, None, None, None, 0, 0

def draw_rects(img, rects, color):
	for x1, y1, x2, y2 in rects:
		cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)


if __name__ == '__main__':

	if DISPLAY_VIDEO:
		cv2.namedWindow("preview")
	
	vc = cv2.VideoCapture(2)
	cascade = cv2.CascadeClassifier("haarcascade_frontalface_alt.xml")
#	cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
	if DETECT_HUMANS:
		hog     = cv2.HOGDescriptor()
		hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

	if vc.isOpened(): # try to get the first frame
		rval, frame = vc.read()
	else:
		rval = False

	while rval:
		rval, frame = vc.read()
		gray = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
		gray = cv2.equalizeHist(gray)
#		gray_small = cv2.resize(gray, size(gray), 0.5, 0.5)
		face_rects  = detect_faces(gray, cascade)
		if DETECT_HUMANS:
			human_rects = detect_humans(gray, hog)

		x1, y1, x2, y2, face_centre_x, face_centre_y = get_first_coordinates(face_rects)
		if DETECT_HUMANS:
			hx1, hy1, hx2, hy2, human_centre_x, human_centre_y = get_first_coordinates(human_rects)

		if DISPLAY_VIDEO:
			## Extract face ROI
			if x1:
				#faceROI = gray[x1:x2, y1:y2]
				draw_rects(frame, face_rects,  (0, 255, 0))
			if DETECT_HUMANS:
				if hx1:
					draw_rects(frame, human_rects, (0, 0, 255))
			#cv2.imshow("preview", faceROI)		  ## Show image in the window
			cv2.imshow("preview", frame)		  ## Show image in the window

		##### publish face_centre_*, human_centre_*

		key = cv2.waitKey(2)    # time in milliseconds
		if key == 27: # exit on ESC
			break
	cv2.destroyAllWindows()
