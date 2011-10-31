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
   snake_pose=rtt.Variable("float64[]")
   snake_pose:resize(8)
   snake_pose[0]=0
   snake_pose[1]=0
   snake_pose[2]=0
   snake_pose[3]=0
   snake_pose[4]=-0.6
   snake_pose[5]=0.7
   snake_pose[6]=0.7
   snake_pose[7]=0.0
function getOODLangles(fullAngles)
   local temp=rtt.Variable("float64[]")
   temp:resize(5)
   temp[0]=fullAngles[3]+2.9496
   temp[1]=fullAngles[4]+1.1345
   temp[2]=fullAngles[5]-2.5482
   temp[3]=fullAngles[6]+1.7890
   temp[4]=fullAngles[7]+2.9234

   return temp
end
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
   monitor=tc:getPeer("monitor")

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
