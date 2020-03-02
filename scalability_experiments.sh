experiment=scalability
param_file=params/params_mSSD220_s2s_u2s.csv
start=1
stop=1
hpc=$1
world=$2

port_shift=$3
swarmsize=( 9 16 25 36 49 64 81 100 )
for rb in "${swarmsize[@]}"
do
    # echo $rb $port_shift
    ./mult_div_submission_loop.sh $hpc $start $stop $world $experiment $param_file $port_shift $rb
    port_shift=$(( $port_shift + $stop ))
done
