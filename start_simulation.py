import subprocess
import sys

def start_simulation(folder_name, line_number):
	copy_wp=None
	copy_rp=None
	copy_np=None
	copy_wg=None
	print('compiling world plugin')
	world_plugin = subprocess.run("cd sources/w_swarm1/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if world_plugin.returncode:
		print('************world plugin compilation failed')
		print(world_plugin.stderr)
	else:
		print('world plugin compilation successful')
		copy_wp = subprocess.run("cp sources/w_swarm1/build/libwp_swarm1.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling robot plugin')
	robot_plugin = subprocess.run("cd sources/mp_swarm1/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if robot_plugin.returncode:
		print('************robot plugin compilation failed')
		print(robot_plugin.stderr)
	else:
		print('robot plugin compilation successful')
		copy_rp = subprocess.run("cp sources/mp_swarm1/build/libmp_swarm1.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling nest plugin')
	nest_plugin = subprocess.run("cd sources/m_nest/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if nest_plugin.returncode:
		print('************nest plugin compilation failed')
		print(nest_plugin.stderr)
	else:
		print('nest plugin compilation successful')
		copy_np = subprocess.run("cp sources/m_nest/build/libnest_plugin.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling world governor')
	world_governor = subprocess.run("cd sources/world_governor/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if world_governor.returncode:
		print('*************world governor compilation failed')
		print(world_governor.stdout)
	else:
		print('world governor compilation successful')
		subprocess.run("rm world_governor",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
		copy_wg = subprocess.run("cp sources/world_governor/build/world_governor .",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
		#print(copy_wg.stderr)
		

	all_set = sum([world_plugin.returncode,copy_wp.returncode,\
				robot_plugin.returncode,copy_rp.returncode,\
				nest_plugin.returncode,copy_np.returncode,\
				world_governor.returncode,copy_wg.returncode])

	#start up gazebo if all processes are successful
	if(all_set == 0):
		load_world = subprocess.Popen("./start_simulation.sh",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)#w_swarm1.world
		
		if load_world.returncode==None:
			load_governor = subprocess.Popen("./world_governor {} {}".format(folder_name,line_number),stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
			if load_governor.returncode==None:
				print('''
				\n\n
				***********************************************
				All set: Simulation started and Governor loaded
				***********************************************
				\n\n''')

		while True:
			load_governor.poll()
			load_world.poll()
			#print(load_governor.returncode ,load_world.returncode)
			if load_governor.returncode != None or load_world.returncode != None:
				load_governor.kill()
				load_world.kill()
				print('''
				********************************
					Simulation Terminated
					World Governor = {}
					World Status   = {}
				Process killed because of non "None"
				value.
				********************************
				'''.format(load_governor.returncode,load_world.returncode))
				#print('world governor status: ',load_governor.returncode,'world status: ',load_world.returncode)
				break
	else:
		print(all_set)

		
		
if __name__=='__main__':
	start_simulation(sys.argv[1], sys.argv[2])