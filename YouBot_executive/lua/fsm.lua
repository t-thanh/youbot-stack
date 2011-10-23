return rfsm.csta {
   homing = rfsm.sista {
      entry=
         function() 
         local op=executive:getOperation("unfoldArm")
         local handle=op:send()
         end,
   },
    wait_for_user = rfsm.sista {
       entry=
          function()
          local op=executive:getOperation("gravityMode")
          local handle=op:send()
          print("homing done")
          print("gravity compensation mode")
          print("wait for the velocity non zero")
          end,
    },
    learning_position = rfsm.sista{
       entry=
          function() 
          print("arm is moving")
          print("timout is not gone")
          print("gravity compensation mode") 
          end,
    },
    proving_position = rfsm.sista{
       entry=
          function() 
          local op1=executive:getOperation("getArmPose")
          local handle1=op1:send()
          local op2=executive:getOperation("getGripperPose")
          local handle2=op2:send()
          print("timeout is gone")
          print("position is recorded")
          end,
    },
    positioning = rfsm.sista {
       entry=
          function() 
          local op=executive:getOperation("unfoldArm")
          local handle=op:send()
          print("start going to defined pose") 
          end,
    },

    rfsm.trans {src="initial", tgt="homing" },
    rfsm.trans {src="homing", tgt="wait_for_user",               events={"jntpos.e_reached"    }},
    rfsm.trans {src="wait_for_user", tgt="learning_position",    events={"jntvel.e_lim_exceded"}},
    rfsm.trans {src="learning_position", tgt="proving_position", events={"jntvel.e_reached"    }},--loop
    rfsm.trans {src="proving_position", tgt="learning_position", events={"jntvel.e_lim_exided" }},--loop
    rfsm.trans {src="proving_position", tgt="positioning",       events={"timer.e_lim_exceded" }},--exit
}
-- getArmPose getGripperPose gravityMode positionArm positionGripper sunfoldArm 
