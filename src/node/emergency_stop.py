#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Start : press   [space] key
Stop  : release [space] key by 1s or more

CTRL-C to quit
"""
spaceBtn = ' '

class PublishThread(threading.Thread):
    def __init__(self, sub_num):
        super(PublishThread, self).__init__()

        self.start_flag = False
        self.update_time = -1
        self.pub_emergency = False

        self.publishers = []
        self.sub_num = sub_num
        for i in range(self.sub_num):
            self.publishers.append(rospy.Publisher('/anafi' + str(i) + '/land',Empty,queue_size = 1))

        self.condition = threading.Condition()
        self.done = False

        self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown():
            ok = True
            for n in range(self.sub_num):
                if self.publishers[n].get_num_connections() == 0:
                    ok = False

            if ok:
                break

            if i == 4:
                for n in range(self.sub_num):
                    if self.publishers[n].get_num_connections() == 0:
                        print("Waiting for subscriber to connect to {}".format(self.publishers[n].name))

            rospy.sleep(0.5)
            i += 1
            i = i % 5
        
        for n in range(self.sub_num):
            print("ok {}".format(self.publishers[n].name))
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self,update_time):
        self.condition.acquire()

        ret_str = ""
        if (self.start_flag is False) and (rospy.Time.now() - update_time).to_sec() < 1.0:
            self.start_flag = True
            ret_str = "Start Emergency mode"

        if (self.start_flag) is True and (rospy.Time.now() - update_time).to_sec() > 1.0:
            self.pub_emergency = True
            ret_str = "Emergency!! Publish lang msg"

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

        return ret_str

    def stop(self):
        self.done = True
        self.update(rospy.Time(0))
        self.join()

    def run(self):
        empty_msg = Empty()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            self.condition.release()
            # Publish.
            if self.pub_emergency:
                for n in range(self.sub_num):
                    self.publishers[n].publish(empty_msg)

        # Publish stop message when thread exits.
        for n in range(self.sub_num):
            self.publishers[n].publish(empty_msg)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('emergency_stop')

    sub_num = rospy.get_param("~sub_num",4)
    key_timeout = rospy.get_param("~key_timeout", 1.0)

    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(sub_num)

    update_time = rospy.Time(0)

    status = 0

    try:
        pub_thread.wait_for_subscribers()
        ret_str = pub_thread.update(update_time)

        if len(ret_str) != 0:
            print(ret_str)

        print(msg)

        emergency_cnt = 0;
        while(1):
            key = getKey(key_timeout)
            if key == spaceBtn:
                update_time = rospy.Time.now()
            else:
                if key == '\x03':
                    break

            ret_str = pub_thread.update(update_time)
            if len(ret_str) != 0:
                print(ret_str)
                if ret_str == "Emergency!! Publish lang msg":
                    emergency_cnt = emergency_cnt + 1

                if emergency_cnt >= 5:
                    break

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
