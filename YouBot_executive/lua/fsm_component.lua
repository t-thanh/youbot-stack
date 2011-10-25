require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rttros"
 
local tc=rtt.getTC();
local fsm
local fqn_out, events_in
arm_pose=rtt.Variable("float64[]")
arm_pose:resize(8)
gripper_pose=rtt.Variable("float64[]")
gripper_pose:resize(6)
   -- load state machine
   fsm = rfsm.init(rfsm.load(rttros.find_rospack("YouBot_executive").."/lua/fsm.lua"))
 
   -- the following creates a string input port, adds it as a event
   -- driven port to the Taskcontext. The third line generates a
   -- getevents function, which returns all data on the current port as
   -- events. This function is called by the rFSM core to check for
   -- new events.
   events_in = rtt.InputPort("string")
   tc:addEventPort(events_in, "events", "rFSM event input port")
   fsm.getevents = rfsm_rtt.gen_read_events(events_in)
 
   -- optional: create a string port to which the currently active
   -- state of the FSM will be written. gen_write_fqn generates a
   -- function suitable to be added to the rFSM step hook to do this.
   fqn_out = rtt.OutputPort("string")
   tc:addPort(fqn_out,"state","Current state of the FSM")
   fsm.step_hook=rfsm_rtt.gen_write_fqn(fqn_out)
function configureHook()

   -- NOTE: executive is a global variable !!!!
   executive=tc:getPeer("executive")
   return true
end
 
function updateHook() 
   rfsm.run(fsm) 
end
 
function cleanupHook()
   -- cleanup the created ports.
   tc:removePort(fqn_out:info().name)
   tc:removePort(events_in:info().name)
   fqn_out:delete()
   events_in:delete()
end
