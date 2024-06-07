import cv2
import torch

class YOLOv5Detector:
    def __init__(self):
        self.model = self.load_yolo_model()
    
    def load_yolo_model(self):
        model = torch.hub.load('ultralytics/yolov5', 'custom', '/home/jongchanjang/pinkbot/Art_yolov5s.pt')
        return model

    def start_detection(self):

        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.resize(frame, (320, 240))

            # YOLOv5를 이용한 객체 감지
            results = self.model(frame)

            # 감지된 객체에 대해 사각형 그리기
            for detection in results.pred[0]:
                if detection[4] > 0.40:
                    bbox = detection[:4].int().cpu().numpy()

                    cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)

                    label = f"{results.names[int(detection[5])]}: {detection[4]:.2f}"
                
                    cv2.putText(frame, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    print(label)

            # 결과 표시
            cv2.imshow('YOLOv5 Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = YOLOv5Detector()
    detector.start_detection()
