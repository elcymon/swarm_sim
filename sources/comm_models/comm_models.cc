#include "comm_models.hh"

CommModels::CommModels(int qSize, std::string filterType)
:queue_size(qSize), filter_type(filterType) {
    //do stuff if needed
    std::cout<<"queue_size: "<<queue_size<<std::endl
                <<"filter type "<<filter_type<<std::endl;   
}

double CommModels::get_value(std::string desired_value) {
    //use if statements to select the right parameter to return

}

void CommModels::update_comm_signals(double call_comm, double repel_comm, double t) {

}

//functions to handle different communication filters
void CommModels::average_filter(std::deque<double> *signal){
    /*
    uses non-overlapping chunks of instantaneous values to compute
    the signal value by using an average of a chunk to represent
    the magnitude of the signal. A chunk is equivalent to the 
    queue_size.
    */
}