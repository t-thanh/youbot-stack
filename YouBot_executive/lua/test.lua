require "rttlib"
 
local tc=rtt.getTC();
local events_out,state_in
local events_test_table={"jntpos.e_reached","jntvel.e_lim_exceded",
                        "jntvel.e_reached","jntvel.e_lim_exided","jntvel.e_reached","timer.e_lim_exceded"}
local cnt=1
function configureHook()
   state_in =rtt.InputPort("string")
   events_out = rtt.OutputPort("string")
   tc:addPort(events_out,"events_out","")
   tc:addPort(state_in,"state_in","")
   return true
end
 
function updateHook() 
   print("\n sending event" .. events_test_table[cnt] .. "\n")
   events_out:write(events_test_table[cnt])
   cnt = cnt + 1
   local fs, data =state_in:read()
   print("before the trigger FSM was in"..tostring(data)..", flowstatus WFT?¡" .. fs .. "\n\n")
end
 
function cleanupHook()
   -- cleanup the created ports.
   tc:removePort(events_out:info().name)
   tc:removePort(state_in:info().name)
   events_out:delete()
   state_in:delete()
end
