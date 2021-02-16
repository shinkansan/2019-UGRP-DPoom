# Auto Drive integration
import threading
import numpy
import rospy
rospy.init_node('robot_mvs', anonymous = False)
import os
import autodrive as ad
import ground_seg as morp
from easygo import easyGo as eg
import os
def _cls():
  os.system("cls" if os.name == "nt" else "clear")
  pass

slam_launch_command = '''roslaunch realsense2_camera opensource_tracking_tk_localization.launch'''
init_dpoom_command = '''sh init_dpoom.sh'''
handle_mode = 'none'

def handle_master(mode):
    if mode == 'autodrive':
        handle_mode = 'autodrive'
        DRIFT_handling_authority = True
        ad.DRIFT_handling_authority = DRIFT_handling_authority
        morp.handle_easy = False
        rospy.set_param('/dap_handle', True)
        print('dap_handle  :: ', rospy.get_param('/dap_handle'))
        print('***MORP Handle Status ***' + str(morp.handle_easy))
    else:
        handle_mode = 'morp'
        rospy.set_param('/dap_handle', False)
        morp.handle_easy = True
        print('***MORP Handle Status ***' + str(morp.handle_easy))
    print('Current handle mode ', handle_mode)

def slam_launcher():
    os.system(slam_launch_command)

def init_dpoom():
    os.system(init_dpoom_command)

def auto_drive_launcher():
    ad.d_main()
    print('Desination Reached!!!')
    raise('ERROR')

def morp_launcher():
    morp.main()
    pass

def morp_dap_transfer():
    while True:
        _cls()
        print('MORP Report : ' + morp.currentStatus)
        print('Localization : ', ad.flagLocal)
        if morp.currentStatus == "YES OBSTACLE" and ad.flagLocal:
            handle_master('morp')
        else:
            handle_master('autodrive')

def GoEasy(direc):
	if direc == 0:
		easyGo.mvCurve(-SPEED, 0)
	elif direc == 1:
		easyGo.mvCurve(SPEED, 0)
	elif direc == 2:
		easyGo.mvRotate(ROTATE_SPEED, -1, False)
	elif direc == 3:
		easyGo.mvRotate(ROTATE_SPEED, -1, True)

def load_drive_component():

    #Start driving procedure
    print('Start driving componet...')
    # load slam component
    print('Slam loading')
    slam_thread = threading.Thread(target=slam_launcher)
    slam_thread.start()

    #auto drive

    print('Autodirve loading')
    autodrive_thread = threading.Thread(target=auto_drive_launcher)
    autodrive_thread.start()
    print('[INFO] Autodirve loaded')

    #morp ->
    print('[INFO] morp loading')
    morp_thread = threading.Thread(target=morp_launcher)
    morp_thread.start()
    print('[INFO] morp loaded')
    # [upper] localization and global moving

    print('== ALL Component loaded ==')
    handle_master('autodrive')

    handle_tranfer = threading.Thread(target=morp_dap_transfer)
    handle_tranfer.start()



if __name__ == "__main__":



    load_drive_component()
    pass

else:
    print("ERROR!, run standalone")
    raise("incompatible running type")
