import cv2
import cv2.cv as cv
import sys

Notebook_webcam = 0
USB_webcam = 1

def detect(img, cascade):
	rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
	if len(rects) == 0:
		return None
	rects[:,2:] += rects[:,:2]
	return rects

def draw_rects(img, rects, color):
	for x1, y1, x2, y2 in rects:
		cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)


if __name__ == '__main__':

	cv2.namedWindow("preview")
	vc = cv2.VideoCapture(USB_webcam)
	cascade = cv2.CascadeClassifier("haarcascade_frontalface_alt.xml")
#	cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

	if vc.isOpened(): # try to get the first frame
		rval, frame = vc.read()
	else:
		rval = False

	while rval:
		rval, frame = vc.read()
		gray = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
		gray = cv2.equalizeHist(gray)
#		gray_small = cv2.resize(gray, size(gray), 0.5, 0.5)
		rects = detect(gray, cascade)
		if rects is not None:
			## Extract face coordinates			
			x1 = rects[0][1]
			y1 = rects[0][0]
			x2 = rects[0][3]
			y2 = rects[0][2]
			
			## Extract face ROI
			faceROI = gray[x1:x2, y1:y2]

			draw_rects(frame, rects, (0, 255, 0))

#		cv2.imshow("preview", faceROI)		  ## Show image in the window
		cv2.imshow("preview", frame)		  ## Show image in the window

		key = cv2.waitKey(15)    # time in milliseconds
		if key == 27: # exit on ESC
			break
	cv2.destroyAllWindows()
