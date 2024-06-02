import cv2
import numpy as np
import os

class ImageProcessor:
    def __init__(self):
        self.min_area = 100
        self.debug_mode = True
        self.remove_orange = True
        self.load_params()
        
    def threshold_hsv(self, image, lower_bound, upper_bound):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        thresholded_image = cv2.inRange(hsv_image, lower_bound, upper_bound)
        return thresholded_image

    def nuke_cone(self, image, lower_cone, upper_cone):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        thresholded_image = cv2.inRange(hsv_image, lower_cone, upper_cone)
        result = image.copy()
        debug_img = image.copy()

        for j in range(result.shape[1]):
            detected = False
            for i in range(result.shape[0] - 1, -1, -1):
                #print(thresholded_image[i,j])
                if thresholded_image[i, j] >= 125:
                    detected = True
                    debug_img[i, j] = (0, 0, 0)
                    #print('yes')
                if detected:
                    result[i, j] = (0, 0, 0)
                    
        return result, debug_img

    def load_params(self):
        calibration_file = 'calibrate_hsv.txt'
        if os.path.exists(calibration_file):
            with open(calibration_file, 'r') as file:
                lines = file.readlines()
                self.low_h, self.low_s, self.low_v = map(int, lines[0:3])
                self.high_h, self.high_s, self.high_v = map(int, lines[3:6])
                self.cone_low_h, self.cone_low_s, self.cone_low_v = map(int, lines[7:10])
                self.cone_high_h, self.cone_high_s, self.cone_high_v = map(int, lines[10:13])
        else:
            self.low_h = self.low_s = self.low_v = 0
            self.high_h = self.high_s = self.high_v = 255
            self.cone_low_h = self.cone_low_s = self.cone_low_v = 0
            self.cone_high_h = self.cone_high_s = self.cone_high_v = 255
            self.writefile(calibration_file)
    
    def writefile(self, filename):
        with open(filename, 'w') as file:
            file.write(f"{self.low_h}\n{self.low_s}\n{self.low_v}\n")
            file.write(f"{self.high_h}\n{self.high_s}\n{self.high_v}\n\n")
            file.write(f"{self.cone_low_h}\n{self.cone_low_s}\n{self.cone_low_v}\n")
            file.write(f"{self.cone_high_h}\n{self.cone_high_s}\n{self.cone_high_v}\n")

    def process_image(self, img):
        self.load_params()
        lower_bound = np.array([self.low_h, self.low_s, self.low_v])
        upper_bound = np.array([self.high_h, self.high_s, self.high_v])
        lower_cone = np.array([self.cone_low_h, self.cone_low_s, self.cone_low_v])
        upper_cone = np.array([self.cone_high_h, self.cone_high_s, self.cone_high_v])
        #print(lower_bound)
        nuked_img = img.copy()
        debug_img = img.copy()

        if self.remove_orange:
            nuked_img, debug_img = self.nuke_cone(img, lower_cone, upper_cone)

        if self.debug_mode:
            cv2.imshow('Debug Image', debug_img)

        img_thr = self.threshold_hsv(nuked_img, lower_bound, upper_bound)

        contours, _ = cv2.findContours(img_thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros(img_thr.shape, dtype=np.uint8)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= self.min_area:
                cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

        return mask

def main():
    cap = cv2.VideoCapture(0)
    processor = ImageProcessor()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        processed_frame = processor.process_image(frame)
        
        cv2.imshow('Original', frame)
        cv2.imshow('Processed', processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
