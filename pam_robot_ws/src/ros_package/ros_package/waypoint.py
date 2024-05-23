#로봇이 웨이포인트 도착시 Arrive를 보내면 서비스를 시작.
self.human_detect_service = self.create_service(RobotCommand, '/command', self.human_detect_callback)   

#사람 감지하는 함수
 def human_detect_callback(self, request, response):
        self.get_logger().info('human detecting start')
        if request.command == "Arrive":

        # 이미지를 YOLO로 처리
            if detect_people(self.model, self.cv_img) and self.command_sent == False:
                self.get_logger().info('Human detected! Sending command : HD.')
                response.success = True
                response.message = "HD" #HD를 결과로 보내면 로봇에서 사람 탐지 이후의 동작 실행 할 예정
            else:
                self.get_logger().info('No person detected.')
        
    #관리자 서버에서 로봇에서 발행하는 카메라 토픽 구독
    def camera_callback(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.get_logger().info('Received /camera')
        