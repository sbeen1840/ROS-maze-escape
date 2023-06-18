#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Int8
import threading
import math
import time


class MazeEscape:

# ! #############################################        
# ! 클래스 생성자 함수

    def __init__(self):
        rospy.init_node('maze_escape')

# < 사용자 변수 설정 >
        self.scan = []

# < 발행 설정 >
        # /cmd_vel 발행
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# < 구독 설정 >
        # /scan 구독
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

# < 메인 쓰레드 설정>
        self.main_thread = threading.Thread(target=self.main)
        self.main_thread.start()

#! #############################################        
# ! 콜백 함수

    def scan_callback(self, scan):
        
        self.scan = scan.ranges

        self.front = self.scan[0:5]+self.scan[355:360] 
        self.front_avg = sum(self.front)/10
    
        self.side_left = self.scan[5:35]
        self.side_left_avg = sum(self.side_left)/10
    
        self.side_right = self.scan[325:355]
        self.side_right_avg = sum(self.side_right)/10
    
        self.left = self.scan[80:100]
        self.left_avg = sum(self.left)/20
    
        self.right = self.scan[260:280] 
        self.right_avg = sum(self.right)/20

# ! ############################################# 
# ! 통과 방향 확인 함수

    def front_ok(self): 
        self.front = self.scan[0:5]+self.scan[355:360] 
        self.front_avg = sum(self.front)/10
        return self.front_avg > 0.3
    
    def side_left_ok(self):
        self.side_left = self.scan[5:35] 
        self.side_left_avg = sum(self.side_left)/10
        return self.side_left_avg > 1.0 
    
    def side_right_ok(self):
        self.side_right = self.scan[325:355]
        self.side_right_avg = sum(self.side_right)/10
        return self.side_right_avg > 1.0 
    
    def left_ok(self): 
        self.left = self.scan[80:100]
        self.left_avg = sum(self.left)/20
        return self.left_avg > 0.5
    
    def right_ok(self): 
        self.right = self.scan[260:280]
        self.right_avg = sum(self.right)/20
        return self.right_avg > 0.5

# ! #############################################
# ! 모터 자율 주행 함수     
        
    def move_auto(self): 
        move = Twist()

        if self.front_ok() :
            if self.side_right_avg< 1.4:
                move.linear.x = 0.03 
                move.angular.z = 0.4 
                
                print("\n\t -- move auto -- 앞에 장애물 없으나 앞의 오른쪽이 벽과 가까움, 부드럽게 좌회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t side_left : {:.2f}".format(self.side_left_avg))
                print ("\t side_right : {:.2f}".format(self.side_right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

            if self.side_left_avg < 1.4 : 
                move.linear.x = 0.03 
                move.angular.z = -0.4
                
                print("\n\t -- move auto -- 앞에 장애물 없으나 앞의 왼쪽이 벽과 가까움, 부드럽게 우회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t side_left : {:.2f}".format(self.side_left_avg))
                print ("\t side_right : {:.2f}".format(self.side_right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

            if self.left_avg < 0.24:
                move.linear.x = 0.03
                move.angular.z = -0.15
                
                print("\n\t -- move auto -- 앞에 장애물 없으나 왼쪽벽 위험, 살짝 우회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t left : {:.2f}".format(self.left_avg))
                print ("\t right : {:.2f}".format(self.right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

            if self.right_avg < 0.24:
                move.linear.x = 0.03 
                move.angular.z = 0.15
                
                print("\n\t -- move auto -- 앞에 장애물 없으나 오른벽 위험, 살짝 좌회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t left : {:.2f}".format(self.left_avg))
                print ("\t right : {:.2f}".format(self.right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)
            
            move.linear.x = 0.03 
            move.angular.z = 0.0
            
            print("\n\t -- move auto -- 앞에 장애물 없음, 직진")
            print ("\t front : {:.2f}".format(self.front_avg))
            print ("\t side_left : {:.2f}".format(self.side_left_avg))
            print ("\t side_right : {:.2f}".format(self.side_right_avg))
            print ("\t left : {:.2f}".format(self.left_avg))
            print ("\t right : {:.2f}".format(self.right_avg))
                            
        else:
            if self.right_ok() and self.left_ok() and self.side_right_avg > self.side_left_avg:
                move.linear.x = 0.0 
                move.angular.z = -0.5
                
                rospy.sleep(1.0)
                print("\n\t -- move auto -- 앞에 장애물있고 오른쪽 완전 여유있음, 완전 우회전")
                print ("\t right : {:.2f}".format(self.right_avg))
                
            elif self.right_ok() and self.left_ok() and self.side_right_avg < self.side_left_avg:
                move.linear.x = 0.0
                move.angular.z = 0.5
                
                print("\n\t -- move auto -- 앞에 장애물있고 왼쪽 완전 여유있음, 완전 좌회전")
                print ("\t left : {:.2f}".format(self.left_avg))
                
            else:
                if self.left_avg < self.right_avg: 
                    move.linear.x = -0.02
                    move.angular.z = -0.2 
                    self.cmd_pub.publish(move)
                    rospy.sleep(1.0)

                    move.linear.x = -0.02
                    move.angular.z = 0.0
                    print("\n\t -- move auto -- 앞과 양쪽 막히고 왼쪽벽 너무 가까움, 왼쪽벽에서 멀어지는 후진")
                    print ("\t right : {:.2f}".format(self.right_avg))
                    print ("\t left : {:.2f}".format(self.left_avg))

                else:                     
                    move.linear.x = -0.02
                    move.angular.z = 0.2 
                    self.cmd_pub.publish(move)
                    rospy.sleep(1.0)

                    move.linear.x = -0.02
                    move.angular.z = 0.0
                    print("\n\t -- move auto -- 앞과 양쪽 막히고 오른벽 너무 가까움, 오른벽에서 멀어지는 후진")
                    print ("\t right : {:.2f}".format(self.right_avg))
                    print ("\t left : {:.2f}".format(self.left_avg))

        self.cmd_pub.publish(move)

# ! #############################################        
# ! 메인 루프 함수

    def main(self):

        while not rospy.is_shutdown():
            
            if (len(self.scan)==360):

                print('\n')
                print('|------------------ front: {:.2f} -------------------|'.format(self.front_avg))
                print('|side_left : {:.2f} -------------------- side_right : {:.2f}|'.format(self.side_left_avg, self.side_right_avg))
                print('|left: {:.2f} --------------------------------- right : {:.2f}|'.format(self.left_avg, self.right_avg))
                print('\n')

                print('\n')
                print('|------------------ front_ok: {} -------------------|'.format(self.front_ok()))
                print('|side_left_ok : {} -------------------- side_right_ok : {:.2f}|'.format(self.side_left_ok(), self.side_right_ok()))
                print('|left_ok: {} --------------------------------- right_ok : {:.2f}|'.format(self.left_ok(), self.right_ok()))
                print('\n')
                            
                self.move_auto()
                rospy.sleep(0.5)

            else :
                print("\n\t ----- 라이다 데이터 쌓는 중 -----")

if __name__ == "__main__":
    maze_escape = MazeEscape()
    try:
        rospy.spin()
    except Exception as e:
        print("Exception occurred:", str(e))