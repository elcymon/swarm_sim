#gzserver w_sound_modelling.world & #-s & #$1
 #./data_logger &

singularity instance.start ../gazebo-libgazebo7-xenial.simg sim$1
