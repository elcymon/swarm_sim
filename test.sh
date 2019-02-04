#gzserver w_sound_modelling.world & #-s & #$1
 #./data_logger &

#singularity instance.start ../gazebo-libgazebo7-xenial.simg sim$1
if $1; then
	echo "hello2"
else
	echo "hi"
fi
