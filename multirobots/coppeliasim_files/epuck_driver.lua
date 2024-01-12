function sysCall_init()
    objectHandle=sim.getObject('.')
    leftMotor=sim.getObject('./leftJoint')
    rightMotor=sim.getObject('./rightJoint')    
    proxSens={}
    proxSensDist={}
    for i=1,8,1 do
        proxSens[i]=sim.getObject('./proxSensor',{index=i-1})
    end
    
    alias=sim.getObjectAlias(objectHandle,0)   
    
    simTimePub=simROS2.createPublisher('/sim/clock','rosgraph_msgs/msg/Clock')
    odometryPub=simROS2.createPublisher('/model/'..alias..'/odometry','nav_msgs/msg/Odometry')
    laserScanPub=simROS2.createPublisher('/model/'..alias..'/laser_scan','std_msgs/msg/Float64MultiArray')
    -- velSub= simROS2.createSubscription('/model/ep1c/cmd_vel', 'geometry_msgs/msg/Twist', 'setVelocity_cb')
    
    velSub= simROS2.createSubscription('/model/'..alias..'/cmd_vel', 'geometry_msgs/msg/Twist', 'setVelocity_cb')
    
    left_vel = 0.0
    right_vel = 0.0
end

function sysCall_nonSimulation()
    -- is executed when simulation is not running
end

function sysCall_beforeSimulation()
    -- is executed before a simulation starts
    
end

function sysCall_afterSimulation()
    -- is executed before a simulation 
end

function sysCall_actuation()
    publish_time()
    publish_odometry()
    publish_laser_scan()
    sim.setJointTargetVelocity(leftMotor, left_vel)
    sim.setJointTargetVelocity(rightMotor, right_vel)
end

function sysCall_cleanup()
    simROS2.shutdownPublisher(simTimePub)
    simROS2.shutdownPublisher(odometryPub)
    simROS2.shutdownPublisher(laserScanPub)
    simROS2.shutdownSubscription(velSub)
end


function publish_time()
    t=sim.getSimulationTime()
    -- t=sim.getSimulationTimeStep()
    simROS2.publish(simTimePub, {clock={
        sec=math.floor(t),
        nanosec=(t-math.floor(t))*10^3}}
    )
end

function publish_odometry()
    position=sim.getObjectPosition(objectHandle,sim.handle_world)
    -- orientation=sim.getObjectQuaternion(objectHandle,sim.handle_world)
    orientation=sim.getObjectOrientation(objectHandle,sim.handle_world)
    
    simROS2.publish(odometryPub,{   
        header={
                stamp=simROS2.getSimulationTime(),
                frame_id='ep1c/odom'
            },
        child_frame_id='ep1c',    
        pose={                 
        pose={
            
            position = {
                x=position[1],
                y=position[2],
                z=position[3]    
            },
            orientation={
                x=orientation[1],
                y=orientation[2],
                z=orientation[3],
                w=orientation[1]            
            }
        }}
    })
end

function publish_laser_scan()
    large_distance = 0.05
    for i=1,8,1 do
        proxSensDist[i]=large_distance
        res,dist=sim.readProximitySensor(proxSens[i])
        if (res>0) and (dist<large_distance) then
            proxSensDist[i]=dist
        end
    end
    
    simROS2.publish(laserScanPub,{
        layout={
            dim={{label="s", size=8, stride=1}},
            data_offset=0        
        },
        data={
            proxSensDist[1],
            proxSensDist[2],
            proxSensDist[3],
            proxSensDist[4],
            proxSensDist[5],
            proxSensDist[6],
            proxSensDist[7],
            proxSensDist[8]
        }
    })
    

end


function setVelocity_cb(msg)
  -- The message provides linear velocity in m/s
  -- but coppelia receives it in rad/s, we need to convert it
  linear = msg.linear.x
  angular = msg.angular.z

  -- https://www.inf.ufrgs.br/~prestes/Courses/Robotics/manual_pioneer.pdf
  wheel_radius = 0.02 -- 2 cm
  robot_width = 0.052 -- 5.2 cm
  vl = linear - (angular * robot_width) / 2
  vr = linear + (angular * robot_width) / 2

  left_vel = vl / wheel_radius
  right_vel = vr / wheel_radius
end



-- See the user manual or the available code snippets for additional callback functions and details

