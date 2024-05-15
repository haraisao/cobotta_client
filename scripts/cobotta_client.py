#
# -*- coding: utf-8 -*-
# Control a RC8 using b-CAP
# Copyright(C) 2021, Isao Hara, RT Corporation, All rights reserved
#   This software licensed with  'RT Corp. Software License version.1' 
#
from ctypes import RTLD_LOCAL
import bcapclient
import time,copy
import traceback

#######################
PACKING_POS=[89.98, -30.0, 120, -170, -94, 0.01]
ZERO_POS=[0, 0, 80, 0, 0, 0]

########################################
def maxmin(x, mx=20, mn=-20):
  return max(min(x, mx), mn)

########################################
#
class Rc8Client(object):
  def __init__(self, host="192.168.0.1", port=5007, timeout=200):
    self.host = host
    self.port = port
    self.timeout = timeout
    self.m_client = None
    self.hCtrl = None
    self.hRobot = None
    self.take_armed=False
    self.tool_type=''
    
    self.tool_id=0
    self.approach_z=48
    self.grasp_pos=15
    self.approach_height=200
    self.task_names=[]
    self.hTask = None
    self.hFile = None

  #
  #  Connect to RC8
  def connect(self, flag=False, speed=10, timeout=0):
    if self.hCtrl is not None: return True
    if timeout: self.timeout=timeout*100
    try:
      self.m_client = bcapclient.BCAPClient(self.host, self.port, self.timeout)
      ### start b_cap Service
      self.m_client.service_start("")
      print("Send SERVICE_START packet")

      ### Connect to RC8 (RC8(VRC)provider)
      self.hCtrl = self.m_client.controller_connect("", "CaoProv.DENSO.VRC",("localhost"), (""))
      print("Connect RC8")
      ### get Robot Object Handl
      self.hRobot = self.m_client.controller_getrobot(self.hCtrl,"Arm","")

      if flag :
        self.motion_preparation()
        self.set_speed(speed)
        self.take_arm()
    except:
      #traceback.print_exc()
      return False
    return True

    #
    #
  def is_connected(self):
    if self.hCtrl is not None: return True
    return False

  #
  #  b-Cap low level command
  def robot_execute(self, cmd, param=None):
    if self.hRobot:
      return self.m_client.robot_execute(self.hRobot,cmd,param)
    else:
      print("Error in robot_execute")
    return None

  #
  #
  def controller_execute(self, cmd, param=None):
    if self.hCtrl :
      return self.m_client.controller_execute(self.hCtrl, cmd, param)
    else:
      print("Error in controller_execute")
    return None

  #
  #  Get Control
  def take_arm(self):
    try:
      ### TakeArm
      self.robot_execute("TakeArm",[0,0])
      self.take_armed=True
    except:
      print("Fail to takeArm")
    return
  #
  # Release Control
  def give_arm(self):
    if self.take_armed:
      ###Give Arm
      self.robot_execute("GiveArm")
      print("GiveArm")
      self.take_armed=False
    return
  #
  #  Motor On/Off
  def motor(self, flag=True):
    if flag :
      Param = [1,0]
    else:
      Param = [0, 0]
    self.robot_execute("Motor",Param)
    return

  #
  # set Speed ratio (par = 0.1-100)
  def set_speed(self, par=30):
    Speed = par
    Accel = par
    Decel = par
    Param = [Speed,Accel,Decel]
    self.robot_execute("ExtSpeed",Param)
    return

  #
  # Disconnect communication with COBOTTA
  def disconnect(self):
    ###Give Arm
    self.give_arm()
    if self.m_client is None: return
    #Disconnect
    if self.hRobot :
      self.m_client.robot_release(self.hRobot)
      self.hRobot = None

    if self.hCtrl :
      self.m_client.controller_disconnect(self.hCtrl)
      self.hCtrl = None

    self.m_client.service_stop()
    print("B-CAP service Stop")
    return

  #
  # Motion preparation
  def motion_preparation(self):
    try:
      state = self.robot_execute("GetMotionPreparationState")
      if not state :
        self.robot_execute("ManualResetPreparation")
        self.robot_execute("MotionPreparation")
        state = self.robot_execute("GetMotionPreparationState")
    except:
      state=None
    return state
  #
  #  send reset command
  def reset(self):
    self.robot_execute("ManualResetPreparation")
    self.robot_execute("MotionPreparation")
    self.controller_execute("ClearError")
    return

  # Check out_range or not
  def out_range(self, pos):
    return self.robot_execute("OutRange", "P{}".format(tuple(pos)))

  #
  #  primitive level movement
  def move_pose(self, pose, mode="P", opt=""):
    if not self.hRobot:
      print("Error in move")
      return None
    Comp = 1
    position_Value = pose
    Pose = [position_Value,mode,"@E"]
    try:
      self.m_client.robot_move(self.hRobot, Comp, Pose, opt)
    except:
      traceback.print_exc()
      return False
    return True

  #
  # Relative movement
  def move(self, x, y, z, roll=0, pitch=0, yaw=0):
    pos = self.get_current_pos()
    pos[0] += x
    pos[1] += y
    pos[2] += z
    pos[3] += roll
    pos[4] += pitch
    pos[5] += yaw

    return self.move_pose(pos, "L")

  #
  # Absolute movement
  def move_to(self, x, y, z, roll=180, pitch=0, yaw=180, hand = -1):
    if not self.take_armed : return
    pos = self.get_current_pos()
    pos[0:6] = [x, y, z, roll, pitch, yaw]
    
    res = self.move_pose(pos)
    if not res : return False

    if hand == 1: self.grasp()
    elif hand == 0: self.release()
    return True

  #
  # Go to named position
  def go(self, name, dpos=[0,0,0], rot=None):
    pos = self.get_named_pos(name)
    if pos is None:
      print("No such position:", name)
      return False

    pos[0] += dpos[0]
    pos[1] += dpos[1]
    pos[2] += dpos[2]
    if rot :
      pos[3] += rot[0]
      pos[4] += rot[1]
      pos[5] += rot[2]
       
    res = self.move_to(*pos)
    if not res:
      print("Fail to go to target position... Reset Cobotta.")
      self.reset()
    return res

  #
  # move with waypoints
  def move_way(self, way):
    res = True
    for pos in way:
      if len(pos) == 6:
        res = self.move_to(*pos)
      elif len(pos) > 6:
        pos1 = pos[0:6]
        res = self.move_to(*pos1, pos[6])
      if not res: return False
    return True

  #
  # move with named waypoints
  def move_waypoint(self, *name):
    way_pos = []
    for p in name:
      if type(p) is str:
        pos = self.get_named_pos(p)
        if pos : way_pos.append(pos)
      elif type(p) is list and len(p) == 6:
        pos = p
        way_pos.append(p)
      else:
        pass
    if way_pos:
      return self.move_way(way_pos)
    return False

  #
  # State of COBOTTA
  def get_current_pos(self):
    try:
      return self.robot_execute("CurPos")
    except:
      print("Fail to get current pose")
    return None

  #
  #
  def get_current_joints(self):
    try:
      return self.robot_execute("CurJnt")
    except:
      print("Fail to get current pose")
    return None

  #
  #
  def get_current_fig(self):
    try:
      return self.robot_execute("CurFig")
    except:
      print("Fail to get current pose")
    return None

  #
  # get variable in VirtualTP
  def get_variable(self,name, opt=""):
    if not self.m_client: return
    hVal = self.m_client.controller_getvariable(self.hCtrl, name, opt)
    val=self.m_client.variable_getvalue(hVal)
    self.m_client.variable_release(hVal)
    return val

  #
  # set variable in VirtualTP
  def set_variable(self,name, new_val, opt=""):
    if not self.m_client:
      print("===", name, new_val)
      return
    hVal = self.m_client.controller_getvariable(self.hCtrl, name, opt)
    self.m_client.variable_putvalue(hVal, new_val)
    self.m_client.variable_release(hVal)
    return

  # get variable in VirtualTP
  def get_robot_variable(self,name, opt=""):
    if not self.m_client: return
    hVal = self.m_client.robot_getvariable(self.hRobot, name, opt)
    val=self.m_client.variable_getvalue(hVal)
    self.m_client.variable_release(hVal)
    return val

  #
  # set variable in VirtualTP
  def set_robot_variable(self,name, new_val, opt=""):
    if not self.m_client: return
    hVal = self.m_client.robot_getvariable(self.hRobot, name, opt)
    self.m_client.variable_putvalue(hVal, new_val)
    self.m_client.variable_release(hVal)
    return

  #
  # set variable in VirtualTP
  def get_task_names(self, opt=""):
    self.task_names = self.m_client.controller_gettasknames(self.hCtrl, opt)
    return self.task_names

  #
  #
  def get_task(self, name, opt=""):
    if self.hTask is not None:
      print("get_task:;Call task_release")
      return False
    self.hTask = self.m_client.controller_gettask(self.hCtrl,name,opt)
    return True

  #
  #
  def task_release(self, opt=""):
    if self.hTask is None:
       print("No task")
       return
    self.m_client.task_release(self.hTask)
    self.hTask = None
    return

  #
  #
  def task_status(self, name, param=None):
    if not self.get_task(name):
      return 
    stat=self.m_client.task_execute(self.hTask, "GetStatus", param)
    print(stat)
    self.task_release()
    return

  #
  #
  def task_start(self, name, mode=1, param=None):
    if not self.get_task(name):
      return 
    stat=self.m_client.task_start(self.hTask, mode, param)
    self.task_release()
    return

  #
  #
  def task_stop(self, name, mode=4, param=None):
    if not self.get_task(name):
      return
    stat=self.m_client.task_stop(self.hTask, mode, param)
    self.task_release()
    return

  #### File
  def get_filenames(self, opt=""):
    res = self.m_client.controller_getfilenames(self.hCtrl,opt)
    return res

  def get_file(self, name, opt=""):
    if self.hFile is not None:
       print("Call file_release")
       return False
    self.hFile = self.m_client.controller_getfile(self.hCtrl,name,opt)
    return True

  def check_file_handle(self):
    if self.hFile is None:
       return False
    return True

  def file_release(self):
    if self.check_file_handle():
      self.m_client.file_release(self.hFile)
      self.hFile = None
    return True

  def get_file_content(self, name, opt=""):
    if not self.check_file_handle():
      self.get_file(name, opt)
      res=self.m_client.file_getvalue(self.hFile)
      self.file_release()
      return res
    else:
      print("Error in get_file_content")
      traceback.print_exc()
    return None

  def get_file_size(self, name, opt=""):
    if not self.check_file_handle():
      self.get_file(name, opt)
      res=self.m_client.file_getsize(self.hFile)
      self.file_release()
      return res
    else:
      print("Error in get_file_content")
      traceback.print_exc()
    return None

  def put_file_content(self, name, content, opt=""):
    if not self.check_file_handle():
      self.get_file(name, opt)
      res=self.m_client.file_putvalue(self.hFile, content)
      self.file_release()
      return res
    else:
      print("Error in put_file_content")
      traceback.print_exc()
    return None

  def write_content(self, name, data):
    with open(name, mode="w", encoding="shift-jis") as f:
      f.write(data)
    return

  def read_content(self, name):
    with open(name, encoding="shift-jis") as f:
      data = f.read()
    return data

  def upload_pcs_file(self, fname, opt=""):
    if os.path.isfile(fname):
      pcs_file_name=os.path.basename(fname)
      content = self.read_content(fname)
      self.put_file_content(pcs_file_name, content)
    else:
      print("Error: no such file:", fname)
    return

  def download_pcs_file(self, fname, dirname=""):
    content = self.get_file_content(name)
    if content :
      self.write_content(os.path.join(dirname, fname), content)
    return 

  #
  #
  def is_ready(self):
    mode=self.get_variable('@MODE')
    #print("is_ready", mode)
    if mode == 3: return True
    return False

  #
  #
  def mode(self):
    return self.get_variable('@MODE')

  #
  #
  def motor_state(self):
    return self.get_robot_variable('@SERVO_ON')

  #
  #
  def get_error_code(self):
    return self.get_variable('@ERROR_CODE')
    #return self.get_variable('@ERROR_CODE_HEX')

  #
  #
  def clear_error_code(self):
    return self.set_variable('@ERROR_CODE', 0)
    
  #
  #
  def get_error_description(self):
    return self.get_variable('@ERROR_DESCRIPTION')

  #
  #
  def get_e_stop(self):
    return self.get_variable('@EMERGENCY_STOP')

  def set_e_stop(self, v):
    return self.set_variable('@EMERGENCY_STOP', v)
  #
  #
  def get_error_log_count(self):
    return self.controller_execute('GetErrorLogCount')

  #
  #
  def get_error_log(self, n=1):
    err_count=self.controller_execute('GetErrorLogCount')
    res=[]
    for x in range(n):
      id_ = err_count - n + x
      desc_=self.controller_execute('GetErrorLog', id_)
      code_ = desc_[0]
      date_ = "%d-%02d-%02d %02d:%02d:%02d"  % (desc_[1], desc_[2], desc_[4], desc_[5], desc_[6], desc_[7])
      info_ = desc_[12]
      res.append( (code_, date_, info_) )
    return res

  #
  #
  def get_operation_log_count(self):
    return self.controller_execute('GetOprLogCount')

  #
  #
  def get_operation_log(self, n=1):
    err_count=self.controller_execute('GetOprLogCount')
    res=[]
    for x in range(n):
      id_ = err_count - n + x
      desc_=self.controller_execute('GetOprLog', id_)
      code_ = desc_[0]
      date_ = "%d-%02d-%02d %02d:%02d:%02d"  % (desc_[1], desc_[2], desc_[4], desc_[5], desc_[6], desc_[7])
      info_ = desc_[11]
      res.append( (code_, date_, info_) )
    return res

  def move_draw(self, x, y=0, z=0):
    x=maxmin(x)
    y=maxmin(y)
    z=maxmin(z, 5, -5)
    self.set_variable("V0", [x, y, z])
    self.task_start("ExecuteDraw")
    return

  def move_rotate(self, x):
    x=maxmin(x, 10, -10)
    self.set_variable("I14", x)
    self.task_start("ExecuteRotate")
    return

  def save_current_pos(self, x):
    pos=self.get_current_pos()
    self.set_variable("P%d" % x, pos)
    return

  def get_ext_speed(self):
    return self.get_robot_variable("@EXTSPEED")

  def set_ext_speed(self, v):
    return self.set_robot_variable("@EXTSPEED", v)

  #
  # Cobotta Hand
  def hand_move_A(self, v, sp=100):
    val = maxmin(v, 30, 0)
    self.controller_execute("HandMoveA", [val, sp])
    return

  def send_error(self, err=8429148):
      self.set_variable("@ERROR", err)
      return

