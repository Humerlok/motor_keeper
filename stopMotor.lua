-- This script stops one or more motors in flight, to test motor failure handling
-- USE AT YOUR OWN RISK !

-- value to be used on a RC switch to kill one or more motors
local motorKillSwitchValue  = 300 

-- will read 'MOT_STOP_BITMASK' param
local stopMotorBitmask      = Parameter() 

-- add new param MOT_STOP_BITMASK
local PARAM_TABLE_KEY = 75
assert(param:add_table(PARAM_TABLE_KEY,             "MOT_", 1), "[stopMotor] Could not add param table 'MOT_'.")
assert(param:add_param(PARAM_TABLE_KEY, 1,  "STOP_BITMASK", 0), "[stopMotor] Could not add MOT_STOP_BITMASK param.")

-- check if MOT_STOP_BITMASK was created
assert(             stopMotorBitmask:init("MOT_STOP_BITMASK"),  "[stopMotor] Could not find MOT_STOP_BITMASK param.")

-- find rc switch with option 300
local switch = assert(rc:find_channel_for_option(motorKillSwitchValue),string.format("[stopMotor] Could not find switch with option %d",motorKillSwitchValue))

-- read spin min param, we set motors to this PWM to stop them
local minPWM
if quadplane then
  minPWM = assert(param:get("Q_M_PWM_MIN"),"[stopMotor] Could not read Q_M_PWM_MIN")
else
  minPWM = assert(param:get("MOT_PWM_MIN"),"[stopMotor] Could not read MOT_PWM_MIN")
end

local stopMotorChannel, lastMotorBitmask

-- find any motors enabled, populate channels numbers to stop
local function updateStopMotors(newBitmask)
  if lastMotorBitmask == newBitmask then
    return
  end
  stopMotorChannel = {}
  for i = 1, 12 do
    if ((1 << (i-1)) & newBitmask) ~= 0 then
      -- convert motor number to output function number
      local outputFunction
      if i <= 8 then
        outputFunction = i+32
      else
        outputFunction = i+81-8
      end

      -- get channel number for output function
      local tempChannel = SRV_Channels:find_channel(outputFunction)
      if tempChannel then
        table.insert(stopMotorChannel, tempChannel)
      end
    end
  end
  lastMotorBitmask = newBitmask
end

function update()

  updateStopMotors(stopMotorBitmask:get())

  if switch:get_aux_switch_pos() == 2 then
    for i = 1, #stopMotorChannel do
      -- override for 15ms, called every 10ms
      -- using timeout means if the script dies the timeout will expire and all motors will come back
      -- we cant leave the vehicle in a un-flyable state
      SRV_Channels:set_output_pwm_chan_timeout(stopMotorChannel[i],minPWM,15)
    end
  end

  return update, 10 -- reschedule at 100hz
end
gcs:send_text(6, "[stopMotor] Started stopMotor.lua !")
return update() -- run immediately before starting to reschedule