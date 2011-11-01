return rfsm.csta {
   homing = rfsm.sista {
     entry=
         function() 
            print("Entry homing state")
            local unfoldArm=executive:getOperation("unfoldArm")
            local unfoldArm_result=unfoldArm:send()
            local activate_monitor=monitor:getOperation("activate_monitor")
            local activate_monitor_result=activate_monitor:send("jnt_pos_reached_up")
         end,
    exit  =
         function() 
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("jnt_pos_reached_up")
            print("Exit homing state")
         end,
   },
    wait_for_user = rfsm.sista {
       entry=
          function()
             print("Entry wait for user")
             local gravityMode=executive:getOperation("gravityMode")
             local handle1=gravityMode:send()
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor:send("jnt_velocity_zero")
          end,
    },
    learning_position = rfsm.sista{
       entry=
          function() 
             print("Entry learning_position") 
          end,
    },
    proving_position = rfsm.sista{
       entry=
          function()
             print("Entry proving_position")
             --arm_pose & gripper_pose are global variables
             local getArmPose=executive:getOperation("getArmPose")
             local handle1=getArmPose:send(arm_pose)
             local getGripperPose=executive:getOperation("getGripperPose")
             local handle2=getGripperPose:send(gripper_pose)
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor:send("timer")

          end,
          exit=
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("timer")
            local deactivate_monitor_1=monitor:getOperation("deactivate_monitor")
            local handle1_1=deactivate_monitor_1:send("timer")

            print("Exit proving position")
          end,
    },
    positioning_snake = rfsm.sista {
       entry=
          function() 
            print("Entry positioning_snake")
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local deactivate_jnt_velocity_zero_result=deactivate_monitor:send("jnt_velocity_zero")
          
            local copy_monitor=monitor:getOperation("copy_monitor")
            local copy_jnt_pos_reached_result=copy_monitor:send("jnt_pos_reached_up","jnt_pos_reached_snake")
            local setup_monitor=monitor:getOperation("assign_values")
            local setup_monitor_result=setup_monitor:send("jnt_pos_reached_snake",getOODLangles(snake_pose))
            local activate_monitor=monitor:getOperation("activate_monitor")
            local activate_jnt_pos_reached_snake_result=activate_monitor:send("jnt_pos_reached_snake")
   
            local positionArm=executive:getOperation("positionArm")
            local positionArm_result=positionArm:send(snake_pose)
          end,
       exit= 
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("jnt_pos_reached_snake")
            print("Exit positioning_snake")
          end,

    },
    wait_for_bric=rfsm.sista{
      entry=
         function()
            print("Entry wait_for_bric")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local activate_jnt_torque_lim_exceeded_result=activate_monitor:send("jnt_torque_lim_exceeded")
            local openGripper=executive:getOperation("openGripper")
            local openGripper_result=openGripper:send()
         end,
      exit=
         function()
            local deactivate_monitor=monitor:getOperation("activate_monitor")
            local deactivate_jnt_torque_lim_exceeded_result=deactivate_monitor:send("jnt_torque_lim_exceeded")
            local closeGripper= executive:getOperation("closeGripper")
            local closeGripper_result=closeGripper:send()
            print("Exit wait_for_bric")
         end,
    },
    positioning_2 = rfsm.sista {
       entry=
          function() 
             print("Entry positioning_2")
             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
             local deactivate_jnt_velocity_zero_result=deactivate_monitor:send("jnt_velocity_zero")
          
             local copy_monitor=monitor:getOperation("copy_monitor")
             local copy_jnt_pos_reached_result=copy_monitor:send("jnt_pos_reached_up","jnt_pos_reached_learned")
             local setup_monitor=monitor:getOperation("assign_values")
             local setup_monitor_result=setup_monitor:send("jnt_pos_reached_learned",getOODLangles(arm_pose))
             local activate_monitor=monitor:getOperation("activate_monitor")
             local activate_jnt_pos_reached_snake_result=activate_monitor:send("jnt_pos_reached_learned")

             local positionArm=executive:getOperation("positionArm")
             local positionArm_result=positionArm:send(arm_pose)
          end,
       exit= 
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("jnt_pos_reached_learned")
            print("Exit positioning_2")
          end,
    },
    move_down = rfsm.sista{
      entry=
         function()
            print("Entry move_down")
            local op=executive:getOperation("positionGripper")
            gripper_pose[2]=gripper_pose[2]-0.3 --30cm
            local handle=op:send(gripper_pose)
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor:send("jnt_velocity_zero")
         end,  
         exit=
         function()
            local setCartesianStiffness=executive:getOperation("setCartesianStiffness")
            local setPoint=rtt.Variable("float64[]")
            setPoint:resize(2);
            setPoint[0]=20;
            setPoint[1]=100;
            local setCartesianStiffness_result=setCartesianStiffness:send(setPoint)
            local deactivate_monitor=monitor:getOperation("activate_monitor")
            local handle1=deactivate_monitor:send("jnt_velocity_zero")

         end
    },

    align_brics= rfsm.sista{
         entry=
         function()
            print("Entry align_brics")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor:send("timer")
            local guardMove=executive:getOperation("guardMove")
            local setPoint=rtt.Variable("float64[]")
            setPoint:resize(6);
            setPoint[0]=-0.01;
            setPoint[1]=-0.05;
            setPoint[2]=-0.05;
            setPoint[3]=0.0;
            setPoint[4]=0.0;
            setPoint[5]=0.0;
            local guardMove_result=guardMove:send(setPoint)
            end,   
         exit=
         function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("timer")
            print("Exit align_brics ")
         end

         },
      lose_bric  = rfsm.sista{

         entry=
         function()
            print("Entry lose_bric")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor:send("timer")
            local gravityMode=executive:getOperation("gravityMode")
            local handle1=gravityMode:send()

            local openGripper=executive:getOperation("openGripper")
            local openGripper_result=openGripper:send()
            end,   
         exit=
         function()
             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("timer")
            print("Exit lose_bric ")

         end

         },

      retract_gripper = rfsm.sista{

         entry=
         function()
            print("Entry align_brics")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor:send("timer")
            local retractGripper=executive:getOperation("retractGripper")
            local retractGripper_result=retractGripper:send()
            end,   
         exit=
         function()
             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor:send("timer")
            print("Exit lose_bric ")

         end

         },
     go_up = rfsm.sista{
         entry=
         function()
             print("Entry move_down")
             local getGripperPose=executive:getOperation("getGripperPose")
             local handle2=getGripperPose:send(gripper_pose)

            local op=executive:getOperation("positionGripper")
            gripper_pose[2]=gripper_pose[2]+0.6 --60cm
            local handle=op:send(gripper_pose)
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor:send("jnt_velocity_zero")
            end,   
         exit=
         function()
            
         end

         },

     

    rfsm.trans {src="initial", tgt="homing" },
    rfsm.trans {src="homing", tgt="wait_for_user",               events={"jnt01234pos.e_POS_REACHED_true" }},
    rfsm.trans {src="wait_for_user", tgt="learning_position",    events={"jnt01234vel.e_VEL_ZERO_false"}},
    rfsm.trans {src="learning_position", tgt="proving_position", events={"jnt01234vel.e_VEL_ZERO_true" }},--loop
    rfsm.trans {src="proving_position", tgt="learning_position", events={"jnt01234vel.e_VEL_ZERO_false" }},--loop
    rfsm.trans {src="proving_position", tgt="positioning_snake",       events={"timer.e_TIMEOUT" }},
    rfsm.trans {src="positioning_snake", tgt="wait_for_bric",       events={"jnt01234pos.e_POS_REACHED_true" }},
    rfsm.trans {src="wait_for_bric", tgt="positioning_2",       events={ "jnt1eff.e_LIM_EXCEEDED_true"}},
    rfsm.trans {src="positioning_2", tgt="move_down",       events={ "jnt01234pos.e_POS_REACHED_true"}},
    rfsm.trans {src="move_down", tgt="align_brics",       events={ "jnt01234vel.e_VEL_ZERO_true"}},
    rfsm.trans {src="align_brics", tgt="lose_bric",       events={ "timer.e_TIMEOUT"}},
    rfsm.trans {src="lose_bric", tgt="retract_gripper",       events={ "timer.e_TIMEOUT"}},
    rfsm.trans {src="retract_gripper", tgt="positioning_snake",       events={ "timer.e_TIMEOUT"}},




}
-- getArmPose getGripperPose gravityMode positionArm positionGripper unfoldArm 
