return rfsm.csta {
   homing = rfsm.sista {
     entry=
         function() 
            print("Entry homing state")
            local unfoldArm=executive:getOperation("unfoldArm")
            local result_unfoldArm=unfoldArm()
            local activate_monitor=monitor:getOperation("activate_monitor")
            local result_activate_monitor=activate_monitor("jnt_pos_reached_up")
         end,
    exit  =
         function() 
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local result_deactivate_monitor=deactivate_monitor("jnt_pos_reached_up")
            print("Exit homing state")
         end,
   },
    wait_for_user = rfsm.sista {
       entry=
          function()
             print("Entry wait for user")
             local gravityMode=executive:getOperation("gravityMode")
             local handle1=gravityMode()
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor("jnt_velocity_zero")
          end,
	exit=
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local deactivate_jnt_velocity_zero_result=deactivate_monitor("jnt_velocity_zero")
            print("Exit wait for user")
          end,
    },
    learning_position = rfsm.sista{
       entry=
          function() 
             print("Entry learning_position") 
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor("jnt_velocity_zero")
          end,
       exit=
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local deactivate_jnt_velocity_zero_result=deactivate_monitor("jnt_velocity_zero")
            print("Exit learning_position")
          end,
    },
    proving_position = rfsm.sista{
       entry=
          function()
             print("Entry proving_position")
             --arm_pose & gripper_pose are global variables
             local getArmPose=executive:getOperation("getArmPose")
             local handle1=getArmPose(arm_pose)
             local getGripperPose=executive:getOperation("getGripperPose")
             local handle2=getGripperPose(gripper_pose)
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor("timer")
             local activate_monitor=monitor:getOperation("activate_monitor")
             local handle1=activate_monitor("jnt_velocity_zero")
          end,
          exit=
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("timer")
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local deactivate_jnt_velocity_zero_result=deactivate_monitor("jnt_velocity_zero")
            print("Exit proving position")
          end,
    },
    positioning_snake = rfsm.sista {
       entry=
          function() 
            print("Entry positioning_snake")
          
            local copy_monitor=monitor:getOperation("copy_monitor")
            local copy_jnt_pos_reached_result=copy_monitor("jnt_pos_reached_up","jnt_pos_reached_snake")
            local setup_monitor=monitor:getOperation("assign_values")
            local setup_monitor_result=setup_monitor("jnt_pos_reached_snake",getOODLangles(snake_pose))
            local activate_monitor=monitor:getOperation("activate_monitor")
            local activate_jnt_pos_reached_snake_result=activate_monitor("jnt_pos_reached_snake")
   
            local positionArm=executive:getOperation("positionArm")
            local positionArm_result=positionArm(snake_pose)
          end,
       exit= 
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("jnt_pos_reached_snake")
            print("Exit positioning_snake")
          end,

    },
    wait_for_bric=rfsm.sista{
      entry=
         function()
            print("Entry wait_for_bric")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local activate_jnt_torque_lim_exceeded_result=activate_monitor("jnt_torque_lim_exceeded")
            local openGripper=executive:getOperation("openGripper")
            local openGripper_result=openGripper()
         end,
      exit=
         function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local deactivate_jnt_torque_lim_exceeded_result=deactivate_monitor("jnt_torque_lim_exceeded")
            local closeGripper= executive:getOperation("closeGripper")
            local closeGripper_result=closeGripper()
            print("Exit wait_for_bric")
         end,
    },
    positioning_2 = rfsm.sista {
       entry=
          function() 
             print("Entry positioning_2")
--             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
--             local deactivate_jnt_velocity_zero_result=deactivate_monitor("jnt_velocity_zero")
          
             local copy_monitor=monitor:getOperation("copy_monitor")
             local copy_jnt_pos_reached_result=copy_monitor("jnt_pos_reached_up","jnt_pos_reached_learned")
             local setup_monitor=monitor:getOperation("assign_values")
             local setup_monitor_result=setup_monitor("jnt_pos_reached_learned",getOODLangles(arm_pose))
             local activate_monitor=monitor:getOperation("activate_monitor")
             local activate_jnt_pos_reached_snake_result=activate_monitor("jnt_pos_reached_learned")

             local positionArm=executive:getOperation("positionArm")
             local positionArm_result=positionArm(arm_pose)
          end,
       exit= 
          function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("jnt_pos_reached_learned")
            print("Exit positioning_2")
          end,
    },
    move_down = rfsm.sista{
      entry=
         function()
            print("Entry move_down")
            local op=executive:getOperation("positionGripper")
            gripper_pose[2]=gripper_pose[2]-0.3 --30cm
            local handle=op(gripper_pose)
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor("jnt_velocity_zero")
         end,  
         exit=
         function()
            local setCartesianStiffness=executive:getOperation("setCartesianStiffness")
            local setPoint=rtt.Variable("float64[]")
            setPoint:resize(2);
            setPoint[0]=20;
            setPoint[1]=100;
            local setCartesianStiffness_result=setCartesianStiffness(setPoint)
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("jnt_velocity_zero")

         end
    },

    align_brics= rfsm.sista{
         entry=
         function()
            print("Entry align_brics")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor("timer")
            local guardMove=executive:getOperation("guardMove")
            local setPoint=rtt.Variable("float64[]")
            setPoint:resize(3);
            setPoint[0]=0.2;
            setPoint[1]=-0.2;
            setPoint[2]=-0.01;
         --   setPoint[3]=0.0;
          --  setPoint[4]=0.0;
          --  setPoint[5]=0.0;
            local guardMove_result=guardMove(setPoint)
            end,   
         exit=
         function()
            local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("timer")
            print("Exit align_brics ")
         end

         },
      lose_bric  = rfsm.sista{

         entry=
         function()
            print("Entry lose_bric")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor("timer")
            local gravityMode=executive:getOperation("gravityMode")
            local handle1=gravityMode()

            local openGripper=executive:getOperation("openGripper")
            local openGripper_result=openGripper()
            end,   
         exit=
         function()
             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("timer")
            print("Exit lose_bric ")

         end

         },

      retract_gripper = rfsm.sista{

         entry=
         function()
            print("Entry align_brics")
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor("timer")
            local retractGripper=executive:getOperation("retractGripper")
            local retractGripper_result=retractGripper()
            end,   
         exit=
         function()
             local deactivate_monitor=monitor:getOperation("deactivate_monitor")
            local handle1=deactivate_monitor("timer")
            print("Exit lose_bric ")

         end

         },
     go_up = rfsm.sista{
         entry=
         function()
             print("Entry move_down")
             local getGripperPose=executive:getOperation("getGripperPose")
             local handle2=getGripperPose(gripper_pose)

            local op=executive:getOperation("positionGripper")
            gripper_pose[2]=gripper_pose[2]+0.6 --60cm
            local handle=op(gripper_pose)
            local activate_monitor=monitor:getOperation("activate_monitor")
            local handle1=activate_monitor("jnt_velocity_zero")
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
