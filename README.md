km_dolly - ROS package for KM-1 Dolly Kit
=========================================

Introduction
--

Sample to realize differential two-wheeler using two KeiganMotor.

Requirements
--

- ROS Kinetic
- pykeigan_motor v2.1.0 or above

https://github.com/keigan-motor/pykeigan_motor/

How to use
--

Install

        cd ~/catkin_ws/src/
        git clone https://github.com/keigan-motor/ros_km_dolly_simple
        cd ~/catkin_ws/
        catkin_make


Set the motor address in the launch file.

Connect the motor with USB and check the motor address with the following command.

    $ls /dev/serial/by-id/

   Change the following in km_control_usb.launch(~/catkin_ws/src/km_dolly/src/launch)

    <arg name="right_w_addr" value="/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00KHAH-if00-port0" />
    <arg name="left_w_addr" value="/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00LSSA-if00-port0" />


 Run

    $ roslaunch km_dolly km_control_usb.launch


Operation(teleop_twist_keyboard)
--


    -------------------------------------------
        u(Forward left)    i(Forward)    o(Forward right)
    
        j(Rotate left)　　k(Stop)    l(Rotate right)
    
        m(Left backward)    ,(Backward)    .(Right backward)
    
    -------------------------------------------
    
        q/z：Maximum speed (increase/decrease　±10%)
        w/x：Linear velocity (±10%)
        e/c：Angular velocity (±10%)
    
        Ctrl + C:Exit

See http://wiki.ros.org/teleop_twist_keyboard for detailed package description