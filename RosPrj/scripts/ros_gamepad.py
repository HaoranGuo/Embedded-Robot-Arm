import rospy
import embedded_robot_arm.msg as arm_msg
import math
import pygame

    
# ros要写在main函数里面
if __name__ == '__main__':
    rospy.init_node('ros_test', anonymous=True)
    pub = rospy.Publisher('/robot_cmd', arm_msg.cmd, queue_size=10) # 定义发布器，内参不要改
    rate = rospy.Rate(5) # 2hz，即每秒发送2次
    cmd_msg = arm_msg.cmd()
    cmd_msg.cmd = 'moveJ' # 两个控制方式，moveJ是关节空间的运动，moveL是笛卡尔空间的运动
    cmd_msg.mode = 0 # 1是角度控制，0是位置控制——x,y,z,rx,ry,rz
    # 机械臂的初始位置（角度）
     # data是一个list，分别对应x,y,z,rx,ry,rz或者6个关节角度
    
    # 初始化pygame
    pygame.init()
    # 初始化手柄
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # 初始化左右两个摇杆的值
    left_stick_x = 0
    left_stick_y = 0
    right_stick_x = 0
    right_stick_y = 0

    #设定机械臂末端初位姿
    x = 90      #对应左摇杆左右拨动 
    y = 50     #对应左摇杆上下拨动
    z = 150      #对应右摇杆上下拨动
################################################需要根据机械臂初始化时的位姿定义
    alpha = math.pi   #对应X、B按键
    beta = 0    #对应Y、A按键
    gamma = 0   #对应LB、RB按键
####################################################
    #映射倍数
    mapping = 5         #速度
    angular_vel = 0.05   #角速度

    #设定X、Y、A按键状态
    X_pressed = False   
    Y_pressed = False
    A_pressed = False
    B_pressed = False
    LB_pressed = False
    RB_pressed = False
    
    cmd_msg.data = [x, y, z, alpha, beta, gamma]
    
    pub.publish(cmd_msg)
    
    rospy.Duration(0.5)

    print("等待手柄事件，按下手柄菜单键以退出...")

    while not rospy.is_shutdown():
        
        # left_stick_x = 0
        # left_stick_y = 0
        # right_stick_y = 0
        
        # 获取到数据后直接填充在cmd_msg.data里面
        #cmd_msg.data[2] -= 0.1
        #改变姿态
        if X_pressed:
            alpha += angular_vel
        if Y_pressed:
            beta += angular_vel
        if A_pressed:
            beta -= angular_vel 
        if B_pressed:
            alpha -= angular_vel
        if LB_pressed:
            gamma += angular_vel 
        if RB_pressed:
            gamma -= angular_vel
        
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"按键 {event.button} 被按下")
                if event.button == 0:  # A 按钮
                    A_pressed = True # 设置 A 按钮被按下的标志
                elif event.button == 1 :  # B 按钮
                    B_pressed = True # 设置 B 按钮被按下的标志
                elif event.button == 2 :  # X 按钮
                    X_pressed = True # 设置 X 按钮被按下的标志
                elif event.button == 3:  # Y 按钮
                    Y_pressed = True # 设置 Y 按钮被按下的标志
                elif event.button == 4 :  # LB 按钮
                    LB_pressed = True # 设置 LB 按钮被按下的标志
                elif event.button == 5:  # RB 按钮
                    RB_pressed = True # 设置 RB 按钮被按下的标志                     
                
                elif event.button == 6:  # 地图 按钮
                    cmd_msg.mode = 0 # 切换为位置控制
                elif event.button == 7:  # 菜单按钮
                    break  # 退出循环
                
            
            elif event.type == pygame.JOYBUTTONUP:
                print(f"按键 {event.button} 被松开")
                if event.button == 0: # A 按钮
                    A_pressed = False # 设置  按钮被松开的标志                
                elif event.button == 1: # B 按钮
                    B_pressed = False # 设置 B 按钮被松开的标志
                elif event.button == 2: # X 按钮
                    X_pressed = False # 设置 X 按钮被松开的标志                
                elif event.button == 3: # Y 按钮
                    Y_pressed = False # 设置 Y 按钮被松开的标志
                elif event.button == 4: # LB 按钮
                    LB_pressed = False # 设置 LB 按钮被松开的标志
                elif event.button == 5: # RB 按钮
                    RB_pressed = False # 设置 RB 按钮被松开的标志
                

            elif event.type == pygame.JOYAXISMOTION:
                # 更新左右两个摇杆的值
                if event.axis == 0:  # X 轴
                    left_stick_x = mapping*event.value
                elif event.axis == 1:  # Y 轴
                    left_stick_y = mapping*event.value
                elif event.axis == 2:  # X 轴（右摇杆）
                    right_stick_x = mapping*event.value
                elif event.axis == 3:  # Y 轴（右摇杆）
                    right_stick_y = mapping*event.value
                        
        
        # 判断并赋值
        left_stick_x = 0 if abs(left_stick_x) < 0.3 else left_stick_x
        left_stick_y = 0 if abs(left_stick_y) < 0.3 else left_stick_y
        right_stick_x = 0 if abs(right_stick_x) < 0.3 else right_stick_x
        right_stick_y = 0 if abs(right_stick_y) < 0.3  else right_stick_y
        
        #变更机械臂末端坐标
        x -= left_stick_x
        y -= left_stick_y
        z -= right_stick_y                                                     
        
        # 使用 f-string 格式化输出坐标
        formatted_coordinates = f"位姿: ({x:.2f}, {y:.2f}, {z:.2f});  ({alpha:.2f}°, {beta:.2f}°, {gamma:.2f}°)"
        print(formatted_coordinates)                     

        # pygame.time.delay(100)  # 添加50毫秒的延迟
        
        cmd_msg.data[0] = x
        cmd_msg.data[1] = y
        cmd_msg.data[2] = z
        cmd_msg.data[3] = alpha
        cmd_msg.data[4] = beta
        cmd_msg.data[5] = gamma
        
        pub.publish(cmd_msg)
        rate.sleep()
    # 释放手柄资源
    joystick.quit()