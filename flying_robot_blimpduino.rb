# flying_robot_blimpduino
#
# Implementation of flying_robot for the blimpduino
#
# Based on flying_robot library (http://flyingrobo.com)
# Uses Ruby Arduino Development to create Unmanned Aerial Vehicles
# Written by Ron Evans (http://deadprogrammersociety.com)
#
# This sketch that uses the flying_robot parser, you just need to implement the methods that make up its interface
# so that it will respond to the standard command set.
# 
# The following commands are supported:
# (h)ail - See if the UAV can still respond. Should send "Roger" back.
# (s)tatus - Grab a snapshot of all instrument readings plus any other status info that the UAV can support
# (e)levators - Set the elevators. The command supports two parameters:
#   direction - enter 'u' for up, 'c' for centered, or 'd' for down
#   deflection - enter an angle between 0 and 90 degrees
# (r)udder - Set the rudder. This command supports two parameters:
#   direction - enter 'l' for left, 'c' for centered, or 'r' for right
#   deflection - enter an angle between 0 and 90 degrees
# (t)hrottle - Set the throttle. This command supports two parameters:
#   direction - enter 'f' for forward, or 'r' for reverse
#   speed - enter a percentage from 0 to 100
# (i)nstruments - Read the current data for one of the installed instruments on the UAV. This command supports one parameter:
#   id - enter an id for which instrment readings should be returned from. If there is not an instrument installed
#        for that id 'Invalid instrument' should be returned
# (a)utopilot - Turns on/off the autopilot. This command supports one parameter:
#   id - enter an id for which autopilot routine should be turned on. Entering '0' is reserved to turn Off the autopilot.
#
class FlyingRobotBlimpduino < ArduinoSketch
  serial_begin :rate => 19200
  
  # read battery voltage, to protect our expensive LiPo from going below minimum power.
  input_pin 0, :as => :battery
  
  # L293D motor controller
  output_pin 2, :as => :right_motor
  output_pin 3, :as => :right_speed
  
  output_pin 4, :as => :left_motor
  output_pin 5, :as => :left_speed
  
  # vectoring servo
  output_pin 10, :as => :vectoring_servo, :device => :servo
  
  # IR sensors
  input_pin 8, :as => :ir_front
  input_pin 6, :as => :ir_right
  input_pin 7, :as => :ir_rear
  input_pin 9, :as => :ir_left
  
  # ultrasonic sensor
  input_pin 2, :as => :range_finder
  output_pin 15, :as => :range_finder_reset
  @dist = "0, long"
  
  define "MAX_SPEED 127"
  @forward = "1, byte"
  @reverse = "0, byte"
  @direction = "1, byte"
  @left_direction = "1, byte"
  @right_direction = "1, byte"
  @left_motor_speed = "0, long"
  @right_motor_speed = "0, long"
  @deflection = "0, byte"
  @deflection_percent = "0, long"
  @deflection_val = "0, long"
  @autopilot_update_frequency = "500, unsigned long"
  @last_autopilot_update = "0, unsigned long"
  
  # main command loop, required for any arduino program
  def loop
    be_flying_robot
    battery_test
    range_finder.update_maxsonar(range_finder_reset)
    update_ir_receiver(ir_front, ir_right, ir_rear, ir_left)
    
    handle_autopilot_update
    
    process_command
    servo_refresh
  end

  # flying robot interface
  def hail
    serial_println "Roger"
  end
  
  def status
    serial_println "Status: operational"
    check_battery_voltage
    check_ir
    check_altitude
  end
  
  def elevators
    print_current_command("Elevators", current_elevator_deflection)
    if current_elevator_direction == 'c'
      @deflection = 22
    end
    if current_elevator_direction == 'u'
      @deflection = 22 - (current_elevator_deflection / 4)
    end
    if current_elevator_direction == 'd'
      @deflection = 22 + (current_elevator_deflection / 4)
    end

    if @deflection < 0
      @deflection = 0
    end
    if @deflection > 45
      @deflection = 45
    end
    
    vectoring_servo.position @deflection
    servo_refresh
  end
    
  def rudder
    print_current_command("Rudder", current_rudder_deflection)
    set_thrusters
  end
  
  def throttle
    print_current_command("Throttle", current_throttle_speed)
    set_thrusters
  end
    
  def instruments
    if current_command_instrument == 'b'
      check_battery_voltage
    end
    
    if current_command_instrument == 'i'
      check_ir
    end

    if current_command_instrument == 'a'
      check_altitude
    end
  end
  
  def autopilot
    if current_command_autopilot == '0'
      # autopilot cancel, so we should shutoff motors
      throttle_speed = 0
      set_thrusters
      serial_println "Autopilot Is Off"
    end
    
    if current_command_autopilot == '1'
      # follow IR beacon
      autopilot_on
      serial_println "Autopilot Is On"
    end
  end
  
  # motor control
  def set_thrusters
    if current_throttle_direction == 'f'
      @left_direction = @forward
      @right_direction = @forward
    else
      @left_direction = @reverse
      @right_direction = @reverse
    end
    
    calculate_motor_speeds
    activate_thrusters
  end
  
  def activate_thrusters
    left_motor.L293_send_command(left_speed, @left_direction, @left_motor_speed)
    right_motor.L293_send_command(right_speed, @right_direction, @right_motor_speed)
  end
  
  def calculate_motor_speeds
    if current_rudder_direction == 'c'
      @left_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
      @right_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
    end
    if current_rudder_direction == 'l'
      if current_rudder_deflection >= 45
        if (current_throttle_direction == 'f')
          @left_direction = @reverse
        else
          @left_direction = @forward
        end
                
        @left_motor_speed = hard_turn_throttle_speed / 10000
        @right_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
      else
        @left_motor_speed = adjusted_throttle_speed / 10000
        @right_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
      end
    end
    if current_rudder_direction == 'r'
      if current_rudder_deflection >= 45
        if (current_throttle_direction == 'f')
          @right_direction = @reverse
        else
          @right_direction = @forward
        end
                
        @left_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
        @right_motor_speed = hard_turn_throttle_speed / 10000
      else
        @left_motor_speed = current_throttle_speed / 100.0 * MAX_SPEED
        @right_motor_speed = adjusted_throttle_speed / 10000
      end
    end
  end
  
  def adjusted_throttle_speed
    @deflection_percent = (current_rudder_deflection * 100 / 90)
    @deflection_val = 100 - @deflection_percent
    return @deflection_val * current_throttle_speed * MAX_SPEED
  end
  
  def hard_turn_throttle_speed
    @deflection_percent = (current_rudder_deflection * 100 / 90)
    return @deflection_percent * current_throttle_speed * MAX_SPEED
  end
  
  # instruments
  def check_battery_voltage
    serial_print "Battery: "
    serial_println int(battery.voltage)
  end
  
  def check_ir   
    serial_print "IR: "
    serial_println current_ir_beacon_direction
  end
  
  def check_altitude
    serial_print "Alt: "
    serial_println maxsonar_distance
  end
  
  # autopilot
  def handle_autopilot_update
    if is_autopilot_on && millis() - @last_autopilot_update > @autopilot_update_frequency
      if current_command_autopilot == '1'
        navigate_using_ir
      end

      # if current_command_autopilot == '2'
      #   navigate_using_range_finder
      # end
    
      @last_autopilot_update = millis()
    end
  end
  
  def navigate_using_ir
    if ir_beacon_not_detected
      @left_motor_speed = 0
      @right_motor_speed = 0
      @left_direction = @forward
      @right_direction = @forward
    end
    
    if ir_beacon_forward
      @left_motor_speed = 0
      @right_motor_speed = 0
      @left_direction = @forward
      @right_direction = @forward
    end
    
    if ir_beacon_right
      @left_motor_speed = MAX_SPEED / 10
      @left_direction = @forward	          
      @right_motor_speed = MAX_SPEED / 10
      @right_direction = @reverse
    end
    
    if ir_beacon_back
      @left_motor_speed = MAX_SPEED / 10
      @left_direction = @forward
      @right_motor_speed = MAX_SPEED / 10
      @right_direction = @reverse
    end
    
    if ir_beacon_left
      @left_motor_speed = MAX_SPEED / 10
      @left_direction = @reverse
      @right_motor_speed = MAX_SPEED / 10
      @right_direction = @forward
    end
    activate_thrusters
  end
  
end