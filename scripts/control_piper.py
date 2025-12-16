import numpy as np
import threading

import genesis as gs

import rospy
from sensor_msgs.msg import JointState

########################## 初始化 ##########################
gs.init(backend=gs.cuda)

########################## 创建场景 ##########################
scene = gs.Scene(
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (0, -3.5, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        res           = (960, 640),
        max_FPS       = 60,
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = True,
)

########################## 实体 ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(
        file  = '/home/khalillee/genesis_workspace/piper_rl/xml/agilex_piper/piper.xml',
    ),
)
########################## 构建 ##########################
scene.build()

jnt_names = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

############ 可选：设置控制增益 ############
# 设置位置增益
franka.set_dofs_kp(
    kp             = np.array([4500, 4500, 3500, 4500, 2000, 2000]),
    dofs_idx_local = dofs_idx,
)
# 设置速度增益
franka.set_dofs_kv(
    kv             = np.array([450, 450, 350, 450, 200, 200]),
    dofs_idx_local = dofs_idx,
)
# 设置安全的力范围
franka.set_dofs_force_range(
    lower          = np.array([-87, -87, -87, -87, -12, -12]),
    upper          = np.array([ 87,  87,  87,  87,  12,  12]),
    dofs_idx_local = dofs_idx,
)
# 硬重置
franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0]), dofs_idx)
scene.step()

########################## ROS 初始化 ##########################
rospy.init_node('genesis_piper_control', anonymous=True)

# 存储当前接收到的关节状态
current_joint_state_ = None
joint_state_lock = threading.Lock()

# 关节名称映射
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

def joint_states_callback(msg):
    """
    订阅 /joint_states 话题的回调函数
    将接收到的关节数据下发给genesis控制机械臂
    """
    global current_joint_state_
    
    # 创建字典以便快速查找关节位置
    joint_dict = {}
    for i, name in enumerate(msg.name):
        if len(msg.position) > i:
            joint_dict[name] = msg.position[i]
    
    # 提取j1-j6对应关节1-6，j7是夹爪
    joint_positions = []
    for i in range(6):
        joint_name = f"joint{i+1}"
        if joint_name in joint_dict:
            joint_positions.append(joint_dict[joint_name])
        else:
            # 如果找不到对应的关节，使用0.0
            joint_positions.append(0.0)
    
    # 提取j7（夹爪）的位置
    j7_position = 0.0
    if "joint7" in joint_dict:
        j7_position = joint_dict["joint7"]
    
    # 更新当前关节状态（保存完整的关节信息，包括j7）
    with joint_state_lock:
        current_joint_state_ = {
            'positions': joint_positions + [j7_position],  # j1-j6 + j7
            'names': msg.name,
            'timestamp': msg.header.stamp,
            'j7_position': j7_position
        }
    
    # 将关节位置下发给genesis控制机械臂（j1-j6）
    if len(joint_positions) == 6:
        franka.control_dofs_position(
            np.array(joint_positions),
            dofs_idx,
        )

# 订阅 /joint_states 话题
joint_states_sub = rospy.Subscriber('/joint_states', JointState, joint_states_callback)

# 发布 /joint_states_feedback 话题
joint_states_feedback_pub = rospy.Publisher('/joint_states_feedback', JointState, queue_size=10)

# 主循环
rate = rospy.Rate(100)  # 100Hz，对应dt=0.01

while not rospy.is_shutdown():
    # 获取genesis当前反馈的关节状态
    current_positions = franka.get_dofs_position(dofs_idx)
    
    # 创建反馈消息
    feedback_msg = JointState()
    feedback_msg.header.stamp = rospy.Time.now()
    feedback_msg.name = joint_names  # j1-j7
    feedback_msg.position = current_positions.tolist()
    
    # 添加j7（夹爪）的位置
    # 从当前关节状态中获取j7的位置
    with joint_state_lock:
        if current_joint_state_ is not None:
            j7_position = current_joint_state_.get('j7_position', 0.0)
        else:
            j7_position = 0.0  # 默认夹爪位置为0
    
    feedback_msg.position.append(j7_position)
    
    # 发布反馈
    joint_states_feedback_pub.publish(feedback_msg)
    
    # 执行仿真步进
    scene.step()
    
    rate.sleep()