return rfsm.csta {
   homing = rfsm.sista {
     entry=
         function() 
         local op=executive:getOperation("unfoldArm")
         local handle=op:send()
         local m_op=monitor:getOperation("activate_monitor")
         local m_handle=m_op:send("jnt_velocity_zero")
         print("homing state")
         end,
    exit  =
         function() 
         local m_op=monitor:getOperation("deactivate_monitor")
         local m_handle=m_op:send("jnt_velocity_zero")
         print("homing state")
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
          local handle1=op1:send(arm_pose)
          local op2=executive:getOperation("getGripperPose")
          local handle2=op2:send(gripper_pose)
          print("timeout is gone")
          print("position is recorded")
          end,
    },
    positioning_1 = rfsm.sista {
       entry=
          function() 
          local op=executive:getOperation("unfoldArm")
          local handle=op:send()
          print("start going to defined pose") 
          end,
    },
    positioning_2 = rfsm.sista {
       entry=
          function() 
          local op=executive:getOperation("positionArm")
          local handle=op:send(arm_pose)
          print("start going to leaned pose") 
          end,
    },
    move_down = rfsm.sista{
      entry=
         function()
            local op=executive:getOperation("positionGripper")
            gripper_pose[2]=gripper_pose[2]-0.1 --10cm

            local handle=op:send(gripper_pose)
         end,   
    },

    rfsm.trans {src="initial", tgt="homing" },
    rfsm.trans {src="homing", tgt="wait_for_user",               events={"jnt01234pos.e_POS_REACHED_true" }},
    rfsm.trans {src="wait_for_user", tgt="learning_position",    events={"jnt01234vel.e_VEL_ZERO_false"}},
    rfsm.trans {src="learning_position", tgt="proving_position", events={"jnt01234vel.e_VEL_ZERO_true" }},--loop
    rfsm.trans {src="proving_position", tgt="learning_position", events={"jnt01234vel.e_VEL_ZERO_false" }},--loop
    rfsm.trans {src="proving_position", tgt="positioning_1",       events={"timer.e_lim_exceded" }},
    rfsm.trans {src="positioning_1", tgt="positioning_2",       events={ "jnt01234pos.e_reached"}},
    rfsm.trans {src="positioning_2", tgt="move_down",       events={ "jnt01234pos.e_reached"}},
}
-- getArmPose getGripperPose gravityMode positionArm positionGripper sunfoldArm 
