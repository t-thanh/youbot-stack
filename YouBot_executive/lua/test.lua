require "rttlib"
 
local tc=rtt.getTC();
local events_out,state_in
local events_test_table={"e_start","jnt01234pos.e_reached","jnt01234vel.e_lim_exceded","jnt01234vel.e_reached","jnt01234vel.e_lim_exided","jnt01234vel.e_reached","timer.e_lim_exceded","jnt01234pos.e_reached"}


local cnt=1
  state_in =rtt.InputPort("string")
  events_out = rtt.OutputPort("string")
  tc:addPort(events_out,"events_out","")
  tc:addPort(state_in,"state_in","")

function configureHook()
    return true
end
 
function updateHook() 
   print("\n sending event" .. events_test_table[cnt] .. "\n")
   events_out:write(events_test_table[cnt])
   cnt = cnt + 1
   local fs, data =state_in:read()
   print("before the trigger FSM was in"..tostring(data)..", flowstatus " .. fs .. "\n\n")
end
 
function cleanupHook()
   -- cleanup the created ports.
   tc:removePort(events_out:info().name)
   tc:removePort(state_in:info().name)
   events_out:delete()
   state_in:delete()
end