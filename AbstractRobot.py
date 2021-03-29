import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
import RobotInfoConverter
import traceback, os, copy, time, threading, warnings
import general_robotics_toolbox as Rox
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil

class AbstractRobot(object):
	def __init__(self,robot_info,default_joint_count):
		self._robot_info = robot_info
		if robot_info.joint_info:
			self._joint_names=[]
			for j_info in robot_info.joint_info:
				self.j_names.append(j_info.joint_identifier.name)
		else:
			if (default_joint_count <= 0):

				raise Exception("Joints must be specified in RobotInfo structure")

			self._joint_names = ['joint_' + str(j) for j in range(default_joint_count)]
		

		self._joint_count = len(self._joint_names)

		self._robot_uuid = robot_info.device_info.device.uuid

		self._robot_caps = robot_info.robot_capabilities
		###lib RobotCapabilities?
		if (self._robot_caps & RobotCapabilities.homing_command) != 0:
			self._uses_homing = True

		if (self._robot_caps & RobotCapabilities.position_command) != 0:
		
			self._has_position_command = True
		

		if (self._robot_caps & RobotCapabilities.velocity_command) != 0:
		
			self._has_velocity_command = True
		
		try:
		
			self._rox_robots = []
			for i in range(robot_info.chains.Count):
				self._rox_robots[i].append(RobotInfoConverter.ToToolboxRobot(robot_info))
			
		
		except:   
			raise Exception("invalid robot_info, could not populate GeneralRoboticsToolbox.Robot")
			traceback.print_exc()

		self._current_tool = []
		self._current_payload =[]

		for i in range(robot_info.chains.Count):
		
			if (robot_info.chains[i].current_tool):
			
				self._current_tool.append(robot_info.chains[i].current_tool)


			if (robot_info.chains[i].current_payload):
			
				self._current_payload[i].append(robot_info.chains[i].current_payload)
			

		for i in range(_joint_count):
		
			limits = robot_info.joint_info[i].joint_limits
			if (limits.velocity <= 0):
			
				raise Exception("Invalid joint velocity for joint i")
			
			if (limits.reduced_velocity <= 0):
			
				limits.reduced_velocity = limits.velocity
			
			if (limits.acceleration <= 0):
			
			   raise Exception("Invalid joint acceleration for joint i")
			
			if (limits.reduced_acceleration <= 0):
			
				limits.reduced_acceleration = limits.acceleration

		#modification param
		self._keep_going = False
		self._lock = threading.Lock()
		self._update_period = 10
		self._state_seqno = 0
		self._wire_position_command_sent
		self._wire_velocity_command_sent

		self._wire_position_command_last_seqno = 0
		self._wire_velocity_command_last_seqno = 0

		self._wire_position_command_last_ep = 0
		self._wire_velocity_command_last_ep = 0

		self._trajectory_valid = False
		self._trajectory_current_time=0
		self._trajectory_max_time=0
		self._trajectory_waypoint=0

		self._jog_start_time=0
		self._jog_trajectory_generator=0
		self._jog_completion_source=0

		self._jog_joint_limit = Math.PI * (10000.0 / 180.0)
		self._trajectory_error_tol = Math.PI * (5.0 / 180.0)
				
		self._command_mode = RobotCommandMode.halt
		self._operational_mode = RobotOperationalMode.manual_reduced_speed
		self._controller_state = RobotControllerState.undefined

		self._position_command = None
		self._velocity_command = None


		self._homed = False
		self._ready = False
		self._enabled = False
		self._stopped = False
		self._error = False
		self._estop_source = 0

		self._communication_failure = True
		self._communication_timeout = 250 # milliseconds


		self._speed_ratio = 1

		self._uses_homing = False

		self._has_position_command = False

		self._has_velocity_command = False

		self._has_jog_command = True

		self._current_tool = None

		self._current_payload = None


	def _start_robot(self):
		
		# self._stopwatch = Stopwatch.StartNew()
		self._stopwatch_epoch = DateTimeUtil.TimeSpec2Now(RRN)
					
		self._keep_going = True
		t=threading.Thread(target=self._loop_thread_func)
		t.start()
	

	def _stop_robot(self):
	
		self._keep_going = False
		# self._loop_thread.Join()
	

	
	def _loop_thread_func(self):

		next_wait = time.time()

		now = next_wait

		while (self._keep_going):
		
			self._run_timestep(now)

			now = time.time()
			
			while(True):
			
				next_wait += self._update_period
			
				if next_wait <= now:
					break

			while (time.time() < next_wait):
			
				time.sleep(0.01)
			
		
	
	#no value assigned?
	# _last_robot_state
	# _last_joint_state
	# _last_endpoint_state

	
	def Dispose(self):
	
		self._keep_going = False
		

	def _run_timestep(self,now):
	
		joint_pos_cmd = None
		joint_vel_cmd = None
		###to be imported
		rr_robot_state=RobotState()
		rr_advanced_robot_state=AdvancedRobotState()
		rr_state_sensor_data=RobotStateSensorData()
		downsampler_step=BroadcastDownsamplerStep()
		with self._lock:
		
			downsampler_step = None
			if (self._broadcast_downsampler):
			
				downsampler_step = BroadcastDownsamplerStep(self._broadcast_downsampler)
			

			self._state_seqno+=1
			res = self._verify_communication(now)
			res = res and self._verify_robot_state(now)
			res = res and self._fill_robot_command(now, joint_pos_cmd, joint_vel_cmd)

			self._fill_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data)

		

		if res:
		
			self._send_robot_command(now, joint_pos_cmd, joint_vel_cmd)
		
		self._send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data)                
		
	

	def _fill_state_flags(now):
	
		f = 0

		if (self._communication_failure):
		
			f = f | RobotStateFlags.communication_failure
			return f
		

		if (self._error):
		
			f = f | RobotStateFlags.error
		

		if (self._stopped):
		
			f = f | RobotStateFlags.estop

			if self._estop_source==1:
				f = f | RobotStateFlags.estop_button1
			elif self._estop_source== 2:
				f = f | RobotStateFlags.estop_other
			elif self._estop_source== 3:
				f = f | RobotStateFlags.estop_fault
			elif self._estop_source== 4:
				f = f | RobotStateFlags.estop_internal                    

			
		

		if (self._enabled):
		
			f = f | RobotStateFlags.enabled
		

		if (self._ready):
		
			f = f | RobotStateFlags.ready
		

		if (self._uses_homing):
		
			if (self._homed):
			
				f = f | RobotStateFlags.homed
			
			else:
			
				f = f | RobotStateFlags.homing_required
			
		

		if (self._wire_position_command_sent):
		
			f = f | RobotStateFlags.valid_position_command
		

		if (self._wire_velocity_command_sent):
		
			f = f | RobotStateFlags.valid_velocity_command
		

		if (self._trajectory_valid):
		
			f = f | RobotStateFlags.trajectory_running
		

		return f

	
	def _calc_endpoint_pose(self,chain):
	
		# CALL LOCKED!
		if (self._current_tool[chain]):
		
			return self._endpoint_pose[chain]
		###lib GeometryConverter?
		endpoint_transform = GeometryConverter.ToTransform(self._endpoint_pose[chain])
		tool_transform = GeometryConverter.ToTransform(self._current_tool[chain].tcp)
		res = endpoint_transform * tool_transform
		return GeometryConverter.ToPose(res)            
	

	def _calc_endpoint_poses(self):
	
		if (self._endpoint_pose):
			###lib/array Pose?
			return 
		
		o=[]
		for i in range(self._endpoint_pose.Length):
		
			o.append(self._calc_endpoint_pose(i))
		
		return o
	

	def _calc_endpoint_vel(self, chain):
	
		# CALL LOCKED!
		if (self._current_tool[chain]):
		
			return self._endpoint_vel[chain]
		
		endpoint_vel_lin = GeometryConverter.ToVector(self._endpoint_vel[chain].linear)
		endpoint_vel_ang = GeometryConverter.ToVector(self._endpoint_vel[chain].angular)
		current_tool_p = GeometryConverter.ToVector(self._current_tool[chain].tcp.translation)            
		
		endpoint_transform = GeometryConverter.ToTransform(self._endpoint_pose[chain])
		
		o = SpatialVelocity()
		o.linear = GeometryConverter.ToVector3(endpoint_vel_lin + Rox.Functions.Cross(endpoint_vel_ang, endpoint_transform.R * current_tool_p))
		o.angular = self._endpoint_vel[chain].angular

		return o
	

	def _calc_endpoint_vels(self):
	
		if (_endpoint_vel):
			return 
		

		o = []
		for i in range(self._endpoint_pose.Length):
		
			o.append(self._calc_endpoint_vel(i))
		
		return o
	


	def _fill_states(self, now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data):
	
		ts = DateTimeUtil.TimeSpec3Now(RRN)

		rob_state = RobotState()
		rob_state.ts = ts
		rob_state.seqno = self._state_seqno
		rob_state.command_mode = self._command_mode
		rob_state.operational_mode = self._operational_mode
		rob_state.controller_state = self._controller_state

		flags = self._fill_state_flags(now)

		rob_state.robot_state_flags = flags

		rob_state.joint_position = copy.deepcopy(self._joint_position)
		rob_state.joint_velocity = copy.deepcopy(self._joint_velocity)
		rob_state.joint_effort = copy.deepcopy(self._joint_effort)

		rob_state.joint_position_command = self._position_command
		rob_state.joint_velocity_command = self._velocity_command
		rob_state.kin_chain_tcp = self._calc_endpoint_poses()
		rob_state.kin_chain_tcp_vel = self._calc_endpoint_vels()
		rob_state.trajectory_running = self._trajectory_valid

		a_rob_state = AdvancedRobotState()
		a_rob_state.ts = ts
		a_rob_state.seqno = rob_state.seqno
		a_rob_state.command_mode = rob_state.command_mode
		a_rob_state.operational_mode = rob_state.operational_mode
		a_rob_state.controller_state = rob_state.controller_state
		a_rob_state.joint_position = rob_state.joint_position
		a_rob_state.joint_velocity = rob_state.joint_velocity
		a_rob_state.joint_effort = rob_state.joint_effort
		a_rob_state.joint_position_command = rob_state.joint_position_command
		a_rob_state.joint_velocity_command = rob_state.joint_velocity_command
		a_rob_state.kin_chain_tcp = rob_state.kin_chain_tcp
		a_rob_state.trajectory_running = rob_state.trajectory_running
		###=byte cast
		a_rob_state.joint_position_units = [JointPositionUnits.radian]*7
		a_rob_state.joint_effort_units = [JointEffortUnits.newton_meter]*7
		a_rob_state.trajectory_running = self._trajectory_valid
		a_rob_state.trajectory_time = self._trajectory_current_time
		a_rob_state.trajectory_max_time = self._trajectory_max_time
		a_rob_state.trajectory_current_waypoint = self._trajectory_waypoint
		### operator
		sensor_data_header = SensorDataUtil.FillSensorDataHeader(RRN,self._robot_info.device_info, self._state_seqno)
		
		sensor_data = RobotStateSensorData()
		sensor_data.data_header = sensor_data_header
		sensor_data.robot_state = a_rob_state

		rr_robot_state = rob_state
		rr_advanced_robot_state = a_rob_state
		rr_state_sensor_data = sensor_data
	

	def _send_states(self,now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data):
		### where rrvar_* defined?
		if (rrvar_robot_state):
		
			rrvar_robot_state.OutValue = rr_robot_state
		

		if (rradvanced_robot_state):
		
			rradvanced_robot_state.OutValue = rr_advanced_robot_state
		
		### not sure  
		# rrvar_robot_state_sensor_data?.AsyncSendPacket(rr_state_sensor_data).ContinueWith(t =>  ignore = t.Exception , System.Threading.Tasks.TaskContinuationOptions.OnlyOnFaulted)                        
		rrvar_robot_state_sensor_data.AsyncSendPacket(rr_state_sensor_data)

		if (rrdevice_clock_now):
		
			rrdevice_clock_now.OutValue = DateTimeUtil.FillDeviceTime(RRN, robot_info.device_info, self._state_seqno)
		
	

	# abstract Task _send_disable()
	
	# public async Task async_disable(int time= -1)
	
	#     await _send_disable()
	

	# public async Task async_enable(int time= -1)
	
	#     await _send_enable()
	

	# abstract Task _send_enable()

	# public async Task async_reset_errors(int time= -1)
	
	#     await _send_reset_errors()
	

	# abstract Task _send_reset_errors()


	# public Task async_halt(int time= -1)
	
	#     if (_command_mode == RobotCommandMode.invalid_state)
		
	#         return Task.FromResult(0)
		
	#     _command_mode = RobotCommandMode.halt

	#     return Task.FromResult(0)
	
	
	def _verify_communication(self,now):
	   
		if ((now - self._last_joint_state > self._communication_timeout)
			or (now - self._last_robot_state > self._communication_timeout)
			or (now - self._last_endpoint_state > self._communication_timeout)):               
		
			self._communication_failure = True

			self._command_mode = RobotCommandMode.invalid_state
			self._operational_mode = RobotOperationalMode.undefined
			self._controller_state = RobotControllerState.undefined

			# _endpoint_pose = None
			# _endpoint_vel = None

			#_send_disable()

			return False
		

		self._operational_mode = RobotOperationalMode.cobot
					
		self._communication_failure = False
		return True
	

	def _verify_robot_state(self, now):
	
		if (self._command_mode == RobotCommandMode.homing):
		
			if (self._enabled and not self._error and not self._communication_failure):
			
				self._controller_state = RobotControllerState.motor_off
				return True
			
		

		if (not self._ready or self._communication_failure):
		
			if (self._stopped):
			
				self._controller_state = RobotControllerState.emergency_stop
			
			elif (self._error):
			
				self._controller_state = RobotControllerState.guard_stop
			
			else:
			
				self._controller_state = RobotControllerState.motor_off
			

			self._command_mode = RobotCommandMode.invalid_state                
			return False
		
		
		if (not self._enabled):
		
			self._controller_state = RobotControllerState.motor_off
			self._command_mode = RobotCommandMode.invalid_state
			return False
		


		if (self._command_mode == RobotCommandMode.invalid_state):
		
			self._command_mode = RobotCommandMode.halt
		

		self._controller_state = RobotControllerState.motor_on

		return True
	

	

	def _fill_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):

		self._wire_position_command_sent = False
		self._wire_velocity_command_sent = False

		self._trajectory_valid = False
		self._trajectory_current_time = 0
		self._trajectory_max_time = 0
		self._trajectory_waypoint = 0

		if (self._command_mode != RobotCommandMode.trajectory):
		
			if (self._active_trajectory):
			
				self._active_trajectory.invalid_mode()
				self._active_trajectory = None
			
			
			if (_queued_trajectories.Count > 0):
			
				for t in self._queued_trajectories:
				
					t.invalid_mode()
				

				self._queued_trajectories.Clear()
			
		

		# if (self._command_mode != RobotCommandMode.velocity_command):
		
			#self._velocity_command = None
		

		if self._command_mode==RobotCommandMode.jog:
				
					
			if (self._jog_trajectory_generator):
			
				
				jog_time = (now - self._jog_start_time)/1000.0

				if (jog_time > self._jog_trajectory_generator.T_Final):
				
					if (self._jog_completion_source):
					
						self._jog_completion_source.TrySetResult(0)
						self._jog_completion_source = None
					
					self._jog_trajectory_generator = None
					return False
				

				if (not self._jog_trajectory_generator.GetCommand(jog_time, jog_command)):
												
					return False
				
				joint_pos_cmd = jog_command.command_position
				return True
			
			else:
			
				if (self._jog_completion_source):
				
					self._jog_completion_source.TrySetResult(0)
					self._jog_completion_source = None
				
			
									
			return True
				
		elif  self._command_mode==RobotCommandMode.position_command:
			### default?
			# RobotJointCommand pos_cmd = default
			ts=TimeSpec()
			# ep = default
			if not(rrposition_command.TryGetInValue(pos_cmd, ts, ep)):
										
				return True
			

			if (self._wire_position_command_last_ep != ep):
			
				self._wire_position_command_last_ep = ep
				self._wire_position_command_last_seqno = 0
			

			if (pos_cmd 
				or (pos_cmd.seqno < self._wire_position_command_last_seqno)
				or (np.abs(pos_cmd.state_seqno - self._state_seqno) > 10)
				or (pos_cmd.command.Length != self._joint_count)
				or (pos_cmd.units.Length != 0 and pos_cmd.units.Length != _joint_count)):
			
				
				return True
			

			if (pos_cmd.units.Length == 0):
			
				pos_cmd_j = pos_cmd.command
			
			else:
				pos_cmd_j = []
				for i in range(_joint_count):
				
					if pos_cmd.units[i]==JointPositionUnits.radian:
						pos_cmd_j.append(pos_cmd.command[i])
						
					elif pos_cmd.units[i]==JointPositionUnits.degree:
						pos_cmd_j.append(pos_cmd.command[i] * (np.pi / 180.0))
						
					elif pos_cmd.units[i]==JointPositionUnits.ticks_rot:
						pos_cmd_j.append((pos_cmd.command[i] / np.float64((2 ^ 20)) * (np.pi * 2.0)))
						
					elif pos_cmd.units[i]==JointPositionUnits.nanoticks_rot:
						pos_cmd_j.append((pos_cmd.command[i] / np.float64((2 ^ 20) * 1e9) * (np.pi * 2.0)))
						
					else:
						
						# Invalid units!
						
						return True
							
					
				
									

			self._wire_position_command_last_seqno = pos_cmd.seqno
			
			joint_pos_cmd = pos_cmd_j
			
			self._wire_position_command_sent = True

			return True
				
		elif self._command_mode==RobotCommandMode.velocity_command:
				
			# RobotJointCommand vel_cmd = default
			ts = TimeSpec()
			#default?
			# ep = default
			if not(rrvelocity_command.TryGetInValue(vel_cmd, ts, ep)):
										
				return True
			

			if (self._wire_velocity_command_last_ep != ep):
			
				self._wire_velocity_command_last_ep = ep
				self._wire_velocity_command_last_seqno = 0
			

			if (vel_cmd
				or (vel_cmd.seqno < _wire_velocity_command_last_seqno)
				or (np.abs(vel_cmd.state_seqno - self._state_seqno) > 50)
				or (vel_cmd.command.Length != self._joint_count)
				or (vel_cmd.units.Length != 0 and vel_cmd.units.Length != self._joint_count)):
										
				return True
			

			if (vel_cmd.units.Length == 0):
			
				vel_cmd_j = vel_cmd.command
			
			else:
			
				vel_cmd_j = []
				for i in range(_joint_count):
				
					if vel_cmd.units[i]==JointVelocityUnits.radian_second:
						vel_cmd_j.append(vel_cmd.command[i])
						
					elif vel_cmd.units[i]== JointVelocityUnits.degree_second:
						vel_cmd_j.append(vel_cmd.command[i] * (np.pi / 180.0))
						
					elif vel_cmd.units[i]== JointVelocityUnits.ticks_rot_second:
						vel_cmd_j.append((vel_cmd.command[i] / np.float64((2 ^ 20)) * (np.pi * 2.0)))
						
					elif vel_cmd.units[i]== JointVelocityUnits.nanoticks_rot_second:
						vel_cmd_j.append((vel_cmd.command[i] / np.float64((2 ^ 20) * 1e9) * (np.pi * 2.0)))
						
					else:
						
						# Invalid units!
						return True
							
					
				
			

			self._wire_velocity_command_last_seqno = vel_cmd.seqno

			if (self._speed_ratio != 1.0):
			
				for i in range(vel_cmd_j.Length):
				
					vel_cmd_j[i] = vel_cmd_j[i] * _speed_ratio
				
			
									
			joint_vel_cmd = vel_cmd_j

			self._wire_velocity_command_sent = True

			return True
				
		elif self._command_mode==RobotCommandMode.trajectory:
				
			if (self._active_trajectory):
			
				interp_res = self._active_trajectory.get_setpoint(now, self._joint_position, traj_pos, traj_vel, traj_t, traj_max_time, traj_waypoint)
				if interp_res==TrajectoryTaskRes.ready:
					self._trajectory_valid = True
					send_traj_cmd = False
					
				# elif interp_res== TrajectoryTaskRes.first_valid_setpoint:
				elif interp_res== TrajectoryTaskRes.valid_setpoint:
					self._trajectory_valid = True
					send_traj_cmd = True
					
				elif interp_res== TrajectoryTaskRes.trajectory_complete:
					self._trajectory_valid = True
					send_traj_cmd = True
					self._active_trajectory = None
					if (self._queued_trajectories.Count >0):
					
						self._active_trajectory = self._queued_trajectories[0]
						self._queued_trajectories.RemoveAt(0)
					
					
				else:
					self._trajectory_valid = False
					send_traj_cmd = False
					self._active_trajectory = None
					for w in self._queued_trajectories:
					
						w._cancelled_in_queue()
					
					self._queued_trajectories.Clear()
					
				

				if (self._trajectory_valid):
				
					self._trajectory_current_time = traj_t
					self._trajectory_max_time = traj_max_time
					self._trajectory_waypoint = traj_waypoint
				

				if (send_traj_cmd):
				
					joint_pos_cmd = traj_pos
				
				else:
				
					joint_pos_cmd = None
				
			
			else:
			
				joint_pos_cmd = None
			
			return True
				
		else:
				
			joint_pos_cmd = None
			return True
				
		
	
	#definition?
	# def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
   
	###time unused?
	def async_get_command_mode(self, time= -1):
	
		with self._lock:
			return Task.FromResult(self._command_mode)

	def async_set_command_mode(self, value, time= -1):
	
		with self._lock:
		
			if (self._command_mode == RobotCommandMode.invalid_state and value == RobotCommandMode.homing):
			
				if (not self._enabled or self._communication_failure):
				
					raise Exception("Cannot set homing command mode in current state")
				

				self._command_mode = RobotCommandMode.homing
				return Task.FromResult(0)
			

			if (not self._ready or self._communication_failure):
			
				raise Exception("Cannot set robot command mode in current state")
			

			if (self._command_mode != RobotCommandMode.halt and value != RobotCommandMode.halt):
			
				raise Exception("Must switch to \"halt\" before selecting new mode")
			

			if value==RobotCommandMode.jog:
					
				if (not self._has_jog_command):
				
					raise Exception("Robot does not support jog command mode")
				
				self._jog_trajectory_generator = None
				self._command_mode = RobotCommandMode.jog
				
				
			elif value== RobotCommandMode.halt:
				
					self._command_mode = value
					
				
			elif value== RobotCommandMode.homing:
				
				if (not self._uses_homing):
				
					raise Exception("Robot does not support homing mode")
				
				self._command_mode = value
				
				
			elif value== RobotCommandMode.position_command:
				
				if (not self._has_position_command):
				
					raise Exception("Robot does not support position command mode")
				
				self._command_mode = value
					
				
			elif value== RobotCommandMode.velocity_command:
				
				if (not self._has_velocity_command):
				
					raise Exception("Robot does not support velocity command mode")
				
				self._command_mode = value
					
				
			elif value== RobotCommandMode.trajectory:
				self._command_mode = value
				
			else:
				raise Exception("Invalid command mode specified")
			
		

		return Task.FromResult(0)
	


	


	# def async_jog_freespace(self, joint_position, max_velocity, wait, time= -1)
	
	#     with self._lock:
		
	#         if (self._command_mode != RobotCommandMode.jog):
			
	#             raise Exception("Robot not in jog mode")
			

	#         if (not self._ready):
			
	#             raise Exception("Robot not ready")
			
			
	#         if (joint_position.Length != self._joint_count):
			
	#             raise Exception("joint_position array must have _joint_count elements")
			

	#         if (max_velocity.Length != self._joint_count):
			
	#             raise Exception("max_velocity array must have _joint_count elements")
			

	#         for i in range(self._joint_count):
			
	#             if (np.abs(self._joint_position[i] - joint_position[i]) > self._jog_joint_limit)
				
	#                 raise Exception("Position command must be within 15 degrees from current")
				

	#             if (max_velocity[i] < 0):
				
	#                 raise Exception("max_vel must be greater than zero")
				
			


	#         if (self._jog_completion_source):
			
	#             self._jog_completion_source.TrySetException(OperationAbortedException("Operation interrupted by new jog command"))
	#             self._jog_completion_source = None
			

	#         now = self._stopwatch.ElapsedMilliseconds

	#         if (self._jog_trajectory_generator):
			
	#             limits = JointTrajectoryLimits()
	#             if self._operational_mode== RobotOperationalMode.manual_reduced_speed:
	#                 limits.a_max = [x.joint_limits.reduced_acceleration for x in self._robot_info.joint_info]
	#                 limits.v_max = [x.joint_limits.reduced_velocity for x in self._robot_info.joint_info]
					
	#             elif self._operational_mode== RobotOperationalMode.manual_full_speed:
	#             elif self._operational_mode== RobotOperationalMode.cobot:
	#                 limits.a_max = [x.joint_limits.acceleration for x in self._robot_info.joint_info]
	#                 limits.v_max = [x.joint_limits.velocity for x in self._robot_info.joint_info]
					
	#             else:
	#                 raise Exception("Invalid operation mode for robot jog")
				
			
	#             limits.x_min = [x.joint_limits.lower for x in self._robot_info.joint_info]
	#             limits.x_max = [x.joint_limits.upper for x in self._robot_info.joint_info]

	#             for i in range(self._joint_count):
				
	#                 if (np.abs(max_velocity[i]) > limits.v_max[i]):
					
	#                     raise Exception("max_velocity[i] is greater than joint limits (limits.v_max[i])")
					
				

	#             self._jog_trajectory_generator = TrapezoidalJointTrajectoryGenerator(self._joint_count, limits)
	#             # ??
	#             new_req = JointTrajectoryPositionRequest()
	#             new_req.current_position = self._position_command ?? (double[])_joint_position.Clone()
	#             new_req.current_velocity = self._velocity_command ?? new double[_joint_count]
	#             new_req.desired_position = joint_position
	#             new_req.desired_velocity = new double[_joint_count]
	#             new_req.max_velocity = max_velocity
	#             new_req.speed_ratio = self._speed_ratio

	#             self._jog_trajectory_generator.UpdateDesiredPosition(new_req)
	#             self._jog_start_time = now
			
	#         else
			
	#             jog_trajectory_t = (now - self._jog_start_time)/1000.0
	#             if (not self._jog_trajectory_generator.GetCommand(jog_trajectory_t, cmd)):
				
	#                 raise Exception("Cannot update jog command")
				

	#             new_req = new JointTrajectoryPositionRequest()
	#             new_req.current_position = cmd.command_position
	#             new_req.current_velocity = cmd.command_velocity
	#             new_req.desired_position = joint_position
	#             new_req.desired_velocity = [0. for i in range(self._joint_count)]
	#             new_req.max_velocity = max_velocity
	#             new_req.speed_ratio = self._speed_ratio

	#             self._jog_trajectory_generator.UpdateDesiredPosition(new_req)
	#             self._jog_start_time = now
			

	#         if (not wait):
			
	#             self._jog_completion_source = None
	#             return Task.FromResult(0)
			
	#         else:
			
	#             self._jog_completion_source = TaskCompletionSource()
	#             return self._jog_completion_source.Task
			
		
	

	# def async_jog_joint(self, joint_velocity, timeout, wait, rr_time= -1)
	
	#     with self._lock:
		
	#         if (self._command_mode != RobotCommandMode.jog):
			
	#             raise Exception("Robot not in jog mode")
			

	#         if (not self._ready):
			
	#             raise Exception("Robot not ready")
			

	#         if (joint_velocity.Length != self._joint_count):
			
	#             raise Exception("joint_velocity array must have _joint_count elements")
			

	#         if (time<= 0):
			
	#             raise Exception("Invalid jog timespecified")
			

	#         for i in range( self._joint_count):
			
	#             if (np.abs(joint_velocity[i]) > self._robot_info.joint_info[i].joint_limits.reduced_velocity):
				
	#                 raise Exception("Joint velocity exceeds joint limits")
				
			

	#         if (self._jog_completion_source):
			
	#             self._jog_completion_source.TrySetException(OperationAbortedException("Operation interrupted by new jog command"))
	#             self._jog_completion_source = None
			

	#         now = self._stopwatch.ElapsedMilliseconds

	#         if (self._jog_trajectory_generator):
			
	#             limits = JointTrajectoryLimits()
	#             if self._operational_mode== RobotOperationalMode.manual_reduced_speed:
	#                 limits.a_max = [x.joint_limits.reduced_acceleration for x in self._robot_info.joint_info]
	#                 limits.v_max = [x.joint_limits.reduced_velocity for x in self._robot_info.joint_info]
					
	#             # elif self._operational_mode== RobotOperationalMode.manual_full_speed:
	#             elif self._operational_mode== RobotOperationalMode.cobot:
	#                 limits.a_max = [x.joint_limits.acceleration for x in self._robot_info.joint_info]
	#                 limits.v_max = [x.joint_limits.velocity for x in self._robot_info.joint_info]
					
	#             else:
	#                 raise Exception("Invalid operation mode for robot jog")
				
	#             limits.x_min = [x.joint_limits.lower for x in self._robot_info.joint_info]
	#             limits.x_max = [x.joint_limits.upper for x in self._robot_info.joint_info]

	#             self._jog_trajectory_generator = TrapezoidalJointTrajectoryGenerator(self._joint_count, limits)
	#             ###??
	#             new_req = JointTrajectoryVelocityRequest()
	#             new_req.current_position = _position_command ?? _joint_position
	#             new_req.current_velocity = _velocity_command ?? new double[_joint_count]
	#             new_req.desired_velocity = joint_velocity
	#             new_req.speed_ratio = _speed_ratio
	#             new_req.time= timeout

	#             self._jog_trajectory_generator.UpdateDesiredVelocity(new_req)
	#             self._jog_start_time = now
			
	#         else:
			
	#             jog_trajectory_t = (now - self._jog_start_time) / 1000.0
	#             if (not self._jog_trajectory_generator.GetCommand(jog_trajectory_t, cmd)):
				
	#                 raise Exception("Cannot update jog command")
				

	#             new_req = new JointTrajectoryVelocityRequest()
	#             new_req.current_position = cmd.command_position
	#             new_req.current_velocity = cmd.command_velocity
	#             new_req.desired_velocity = joint_velocity
	#             new_req.time= timeout
	#             new_req.speed_ratio = self._speed_ratio

	#             self._jog_trajectory_generator.UpdateDesiredVelocity(new_req)
	#             self._jog_start_time = now
			

	#         if (not wait)
			
	#             self._jog_completion_source = None
	#             return Task.FromResult(0)
			
	#         else
			
	#             self._jog_completion_source = TaskCompletionSource()
	#             return self._jog_completion_source.Task
			
		
	

	def async_get_robot_info(time= -1):
	
		with self._lock:
		
			for i in range(_robot_info.chains.Count):
			
				self._robot_info.chains[i].current_tool = self._current_tool[i]
				self._robot_info.chains[i].current_payload = self._current_payload[i]
			
			return Task.FromResult(self._robot_info)
		
	
	
	# TrajectoryTask _active_trajectory
	# List<TrajectoryTask> _queued_trajectories = new List<TrajectoryTask>()
	# def execute_trajectory(self, trajectory):
	
		
	#     owner_ep = ServerEndpoint.CurrentEndpoint


	#     double[] current_joint_pos
	#     double speed_ratio
	#     with self._lock:
		
	#         if (_command_mode != RobotCommandMode.trajectory)
			
	#             raise Exception("Robot must be in trajectory mode to execute trajectory")
			

	#         speed_ratio = _speed_ratio
	#         current_joint_pos = _joint_position
		
		
		
	#     interp = new JointTrajectoryInterpolator(_robot_info)
	#     interp.LoadTrajectory(trajectory, speed_ratio)
				   
	#     interp.Intepolate(0, joint_pos1, current_waypoint1)

	#     for i in range(current_joint_pos.Length):
		
	#         if (np.abs(current_joint_pos[i] - joint_pos1[i]) > _trajectory_error_tol)
			
	#             raise Exception("Starting waypoint too far from current joint positions")
			
		

	#     with self._lock:
		
	#         if (_command_mode != RobotCommandMode.trajectory)
			
	#             raise Exception("Robot must be in trajectory mode to execute trajectory")
			

	#         TrajectoryTask traj_task

	#         if (_active_trajectory)
			
	#             traj_task = new TrajectoryTask(this, interp, False, owner_ep)
	#             _active_trajectory = traj_task
			
	#         else
			
	#             traj_task = new TrajectoryTask(this, interp, True, owner_ep)
	#             _queued_trajectories.Add(traj_task)
			

	#         return traj_task
		

	#     raise NotImplementedException("Trajectory passed checks")

	

	# def _cancel_trajectory(TrajectoryTask trajectory)
	
	#     with self._lock:
		
	#         if (trajectory.Equals(_active_trajectory))
			
	#             _active_trajectory = None
	#             for t in _queued_trajectories)
				
	#                 t._cancelled_in_queue()
				
	#             _queued_trajectories.Clear()
			
	#         else
			
	#             int t_index = -1
	#             for (int i = 0 i<_queued_trajectories.Count):
				
	#                 if (trajectory.Equals(_queued_trajectories[i]))
					
	#                     t_index = i
					
				

	#             for (int i = _queued_trajectories.Count - 1 i > t_index i--)
				
	#                 _queued_trajectories[i]._cancelled_in_queue()
	#                 _queued_trajectories.RemoveAt(i)
				

	#             _queued_trajectories.RemoveAt(t_index)
			
		
	

	# def _abort_trajectory(TrajectoryTask trajectory)
	
	#     _command_mode = RobotCommandMode.halt
	
	
	# def async_get_speed_ratio(self, time= -1):
	
	#     return Task.FromResult(self._speed_ratio)
	

	
	# def async_set_speed_ratio(self, value, time= -1):
	
	#     if (value < 0.1 or value > 10):
		
	#         raise Exception("Invalid speed_ratio")
		

	#     self._speed_ratio = value
	#     return Task.FromResult(0)
	

	# def async_get_operational_mode(self, rr_time= -1):
	
	#     with self._lock:
		
	#         return Task.FromResult(self._operational_mode)
		
	

	# def async_get_controller_state(self, rr_time= -1):
	
	#     with self._lock:
		
	#         return Task.FromResult(self._controller_state)
		
	

	# def async_get_current_errors(self, rr_time= -1):
	#     ### eventlogmessage?
	#     return Task.FromResult(new List<EventLogMessage>())
	

	# def async_jog_cartesian(self, velocity, double timeout, wait, rr_time= -1):
	
	#     raise NotImplementedException()
	

	# def async_execute_trajectory(self, trajectory, self, rr_time= -1):
	
	#     # NOTE: Not called because this is a generator function, execute_trajector is called instead
	#     raise Exception("Call execute_trajectory()")
	

	# def async_home(self, rr_time= -1):
	
	#     raise NotImplementedException()
	

	# def async_getf_signal(self, signal_name, rr_time= -1):
	
	#     raise NotImplementedException()
	

	# def async_setf_signal(self, signal_name, value_, rr_time= -1):
	
	#     raise NotImplementedException()
	

	# def async_tool_attached(self, chain, tool, rr_time= -1):
	
	#     if (tool):
		
	#         raise NonerenceException("Tool cannot be None")
		
	#     if (chain < 0 or not(chain < _current_tool.Length)):
		
	#         raise Exception("Invalid kinematic chain chain for tool")
		
	#     with self._lock:
		
	#         if (_current_tool[chain]):
			
	#             raise Exception("Tool alread attached to kinematic chain chain")
			

	#         self._current_tool[chain] = tool

	#         rrfire_tool_changed(chain, tool?.device_info?.device?.name ?? "")
	#         self._state_seqno+=1                
		

	#     return Task.FromResult(0)
	

	# def async_tool_detached(self, chain, tool_name, rr_time= -1):
	
	#     if (chain < 0 or not (chain < _current_tool.Length)):
		
	#         raise Exception("Invalid kinematic chain chain for tool")
		

	#     with self._lock:
		
	#         if (_current_tool[chain]):
			
	#             raise Exception("Tool not attached to kinematic chain chain")
			

	#         if (_current_payload[chain]):
			
	#             raise Exception("Cannot remove tool while payload attached")
			

	#         if (not string.IsNoneOrEmpty(tool_name)):
			
	#             if (self._current_tool[chain]?.device_info?.device?.name != tool_name)
				
	#                 raise Exception("Invalid tool name to detach from kinematic chain chain")
				
			

	#         self._current_tool[chain] = None

	#         rrfire_tool_changed(chain, "")
	#         self._state_seqno+=1
		

	#     return Task.FromResult(0)

	

	# def async_payload_attached(self, chain, payload, pose, rr_time= -1):
	
	#     if (payload):
		
	#         raise NonerenceException("Tool cannot be None")
		
	#     if (chain < 0 or not(chain < self._current_tool.Length)):
		
	#         raise Exception("Invalid kinematic chain chain for payload")
		
	#     with self._lock:
		
	#         if (self._current_tool[chain]):
			
	#             raise Exception("No tool attached to kinematic chain chain, cannot attach payload")
			

	#         if (self._current_payload[chain]):
			
	#             raise Exception("Payload alread attached to kinematic chain chain")
			

	#         self._current_payload[chain] = payload
	#         ###??
	#         rrfire_payload_changed(chain, payload?.device_info?.device?.name ?? "")
	#         self._state_seqno+=1
		

	#     return Task.FromResult(0)
	

	# def async_payload_detached(self, chain, payload_name, rr_time= -1):
	
	#     if (chain < 0 or not(chain < _current_tool.Length)):
		
	#         raise Exception("Invalid kinematic chain chain for tool")
		

	#     with self._lock:
		
	#         if (self._current_payload[chain]):
			
	#             raise Exception("Payload not attached to kinematic chain chain")
			

	#         if (not string.IsNoneOrEmpty(payload_name)):
			
	#             if (self._current_payload[chain]?.device_info?.device?.name != payload_name):
				
	#                 raise Exception("Invalid payload name to detach from kinematic chain chain")
				
			

	#         self._current_payload[chain] = None

	#         rrfire_payload_changed(chain, "")
	#         self._state_seqno+=1
		

	#     return Task.FromResult(0)
	

	# def async_getf_param(self, param_name, rr_time= -1)
	
	#     raise Exception("Invalid parameter")
	

	# def async_setf_param(self, param_name, value_, rr_time= -1):
	
	#     raise Exception("Invalid parameter")
	

	# def async_get_device_info(self, rr_time= -1):
	
	#     return Task.FromResult(self._robot_info.device_info)
	

	# def async_get_isoch_info(self, rr_time= -1):
	

	#     isoch_info = new IsochInfo()
	#     isoch_info.update_rate = 1.0 / self._update_period
	#     isoch_info.max_downsample = 1000
		
	#     with self._lock:
		
	#         isoch_info.isoch_epoch = self._stopwatch_epoch
		
	#     return Task.FromResult(isoch_info)            
	

	# def async_get_isoch_downsample(self, rr_time= -1):
	
	#     with self._lock:
		
	#         return Task.FromResult(_broadcast_downsampler.GetClientDownsample(ServerEndpoint.CurrentEndpoint))
		
	

	# def async_set_isoch_downsample(self, value, rr_time= -1):
	
	#     with self._lock:
		
	#         self._broadcast_downsampler.SetClientDownsample(ServerEndpoint.CurrentEndpoint, value)
	#         return Task.FromResult(0)
		
	

	def RRServiceObjectInit(self, context, service_path):
	
		rrvar_robot_state_sensor_data.MaxBacklog = 3
		self._broadcast_downsampler = BroadcastDownsampler(context, 0)
		self._broadcast_downsampler.AddPipeBroadcaster(rrvar_robot_state_sensor_data)
		self._broadcast_downsampler.AddWireBroadcaster(rrvar_robot_state)
		self._broadcast_downsampler.AddWireBroadcaster(rradvanced_robot_state)
		self._broadcast_downsampler.AddWireBroadcaster(rrdevice_clock_now)
	


# enum TrajectoryTaskRes

#     unknown = 0,
#     ready,
#     first_valid_setpoint,
#     valid_setpoint,
#     trajectory_complete,
#     invalid_state,
#     joint_tol_error,
#     failed


# public class TrajectoryTask : Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus>


#     AbstractRobot parent
#     JointTrajectoryInterpolator path
#     next_called = False
#     started = False
#     start_time = 0
#     aborted = False
#     cancelled = False
#     joint_tol_error = False
#     finished = False
#     TaskCompletionSource<int> next_wait = new TaskCompletionSource<int>()
#     TaskCompletionSource<int> queue_wait = new TaskCompletionSource<int>()
#     queued

#     owner_ep

#     RobotRaconteurNode node

#     public TrajectoryTask(AbstractRobot parent, JointTrajectoryInterpolator path, queued, owner_ep)
	
#         this.parent = parent
#         this.path = path
#         this.queued = queued

#         this.owner_ep = owner_ep
#         #TODO: find which node is being used
#         this.node = RRN

#         connection_test().ContinueWith(t =>  )
	

#     public Abort()
	
#         raise NotImplementedException()
	

#     public Task AsyncAbort(int time= -1)
	
#         aborted = True
#         parent._abort_trajectory(this)
#         next_wait.TrySetException(new OperationAbortedException("Trajectory execution aborted"))
#         return Task.FromResult(0)
		
	

#     public Close()
	
#         raise NotImplementedException()
	

#     public Task AsyncClose(int time= -1)
	
#         cancelled = True
#         parent._cancel_trajectory(this)
#         next_wait.TrySetException(new OperationAbortedException("Trajectory execution cancelled"))
#         return Task.FromResult(0)
	

#     success_sent = False
#     public async Task<TrajectoryStatus> AsyncNext(int time= -1)
	
#         if (success_sent)
		
#             raise StopIterationException("")
		

#         with self._lock:
		
#             first_call = False
#             if (not next_called)
			
#                 first_call = True
			
#             next_called = True

#             if (first_call and queued)
			
#                 #Report back that we are queued immediately
#                 ret = new TrajectoryStatus()
#                 ret.action_status = ActionStatusCode.queued
#                 ret.trajectory_time = 0
#                 ret.current_waypoint = 0
#                 ret.seqno = parent._state_seqno
#                 return ret
			


		

#         Task<Task> wait_task
#         if (queued)
		
#             wait_task = Task.WhenAny(Task.Delay(5000), next_wait.Task, queue_wait.Task)
		
#         else
		
#             wait_task = Task.WhenAny(Task.Delay(5000), next_wait.Task)
		

#         wait_task1 = await wait_task
#         await wait_task1

#         if (not this.started)
		
#             # Still queued...
#             ret = new TrajectoryStatus()
#             ret.action_status = ActionStatusCode.queued
#             ret.trajectory_time = 0
#             ret.current_waypoint = 0
#             ret.seqno = parent._state_seqno
#             return ret
		

#         if (finished)
		
#             success_sent = True
#             ret = new TrajectoryStatus()
#             ret.action_status = ActionStatusCode.complete
#             ret.trajectory_time = traj_t
#             ret.current_waypoint = (traj_waypoint
#             ret.seqno = parent._state_seqno
#             return ret
		
#         else
		
#             ret = new TrajectoryStatus()
#             ret.action_status = ActionStatusCode.running
#             ret.trajectory_time = traj_t
#             ret.current_waypoint = (traj_waypoint
#             ret.seqno = parent._state_seqno
#             return ret
		
	

#     public Task<TrajectoryStatus[]> NextAll(CancellationToken cancel = default)
	
#         raise NotImplementedException()
	

#     _cancelled_in_queue()
	
#         cancelled = True
#         next_wait.TrySetException(new OperationAbortedException("Trajectory cancelled by controller before start"))
	

#     invalid_mode()
	
#         aborted = True            
#         next_wait.TrySetException(new OperationAbortedException("Invalid mode for trajectory execution"))     
	

#     double traj_t = 0
#     int traj_waypoint = 0
#     TrajectoryTaskRes get_setpoint(now, double[] current_joint_pos, double[] joint_pos, double[] joint_vel, double trajectory_time, double trajectory_max_time, int current_waypoint)
	
#         if (cancelled or aborted)
		
#             joint_pos = None
	#         joint_vel = None
	#         trajectory_time = 0
	#         current_waypoint = 0
	#         trajectory_max_time = 0
	#         return TrajectoryTaskRes.failed
		

	#     first_call = False

	#     double t = 0

	#     if (next_called)
		
	#         if (not started)
			
	#             start_time = now
	#             started = True
	#             first_call = True
			

	#         t_= now - start_time
	#         t = ((double)t_ / 1000.0
		

	#     this.path.Intepolate(t, joint_pos1, current_waypoint1)

	#     for i in range( current_joint_pos.Length):
		
	#         if (np.abs(current_joint_pos[i] - joint_pos1[i]) > parent._trajectory_error_tol)
			
	#             joint_tol_error = True
	#             joint_pos = None
	#             joint_vel = None
	#             trajectory_time = 0
	#             current_waypoint = 0
	#             trajectory_max_time = 0
	#             next_wait.TrySetException(new OperationFailedException("Trajectory tolerance failure"))
	#             return TrajectoryTaskRes.joint_tol_error
			
		

	#     if (not next_called)
		
	#         joint_pos = None
	#         joint_vel = None
	#         trajectory_time = 0
	#         current_waypoint = 0
	#         trajectory_max_time = path.MaxTime
	#         return TrajectoryTaskRes.ready
		

	#     joint_pos = joint_pos1
	#     joint_vel = None
	#     trajectory_time = t
	#     traj_t = t
	#     traj_waypoint = current_waypoint1
	#     current_waypoint = current_waypoint1
	#     trajectory_max_time = path.MaxTime

	#     if (t > path.MaxTime)
		
	#         trajectory_time = path.MaxTime
	#         traj_t = path.MaxTime
	#         finished = True                
	#         next_wait.TrySetResult(0)
	#         return TrajectoryTaskRes.trajectory_complete
		

	#     if (first_call)
		
	#         if (queued)
			
	#             queued = False
	#             queue_wait.TrySetResult(0)
			
	#         return TrajectoryTaskRes.first_valid_setpoint
		
	#     else
		
	#         return TrajectoryTaskRes.valid_setpoint
		
	

	# async Task connection_test()
	
	#     while (not finished)
		
	#         try
			
	#             node.CheckConnection(owner_ep)
			
	#         catch (Exception)
			
	#             parent._cancel_trajectory(this)
	#             next_wait.TrySetException(new ConnectionException("Connection lost"))
			
	#         await Task.Delay(50)
		
	

	# public TrajectoryStatus Next()
	
	#     raise NotImplementedException()
	

	# public TrajectoryStatus[] NextAll()
	
	#     raise NotImplementedException()
	# 