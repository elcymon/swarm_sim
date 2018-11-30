import subprocess
import sys
import os.path

def start_simulation(folder_name, line_number):
	copy_wp=None
	copy_rp=None
	copy_np=None
	copy_wg=None
	print('Checking control plugins')

	if os.path.isfile('compiled_plugins/libwp_swarm1.so'):
		copy_wp = 0
	else:
		copy_wp = 1

	if os.path.isfile('compiled_plugins/libmp_swarm1.so'):
		copy_rp = 0
	else:
		copy_rp = 1

	if os.path.isfile('compiled_plugins/libnest_plugin.so'):
		copy_np = 0
	else:
		copy_np = 1

	if os.path.isfile('./world_governor'):
		copy_wg = 0
	else:
		copy_wg = 1
	all_set = sum([copy_wp,copy_rp,copy_wg])

	#start up gazebo if all processes are successful
	if(all_set == 0):
		load_world = subprocess.Popen("./start_simulation.sh",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)#w_swarm1.world
		
		if load_world.returncode==None:
			load_logger = subprocess.Popen("./world_governor {} {}".format(folder_name, line_number),stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
			if load_logger.returncode==None:
				print('''
				\n\n
				***********************************************
				All set: Simulation started and Logger loaded
				***********************************************
				\n\n''')

		while True:
			load_logger.poll()
			load_world.poll()
			#print(load_logger.returncode ,load_world.returncode)
			if load_logger.returncode != None and load_world.returncode != None:
				load_logger.kill()
				load_world.kill()
				print('''
				********************************
					Simulation Terminated
					World logger = {}
					World Status   = {}
				Process killed because of non "None"
				value.
				********************************
				'''.format(load_logger.returncode,load_world.returncode))
				#print('world logger status: ',load_logger.returncode,'world status: ',load_world.returncode)
				break
	else:
		print(all_set)

if __name__=='__main__':
	start_simulation(sys.argv[1], sys.argv[2])