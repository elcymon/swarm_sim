#ifndef H_COMM_MODELS
#define H_COMM_MODELS
    #include "comm_models.hh"
#endif 

CommModels::CommModels() {
    this->queue_size = 1;
    this->filter_type = "average_filter";

    //initialize signal values to 0
    this->prev_call_signal = 0;
    this->curr_call_signal = 0;
    
    this->prev_repel_signal = 0;
    this->curr_repel_signal = 0;

    this->delta_call_signal = 0;
    this->delta_repel_signal = 0;
}
CommModels::CommModels(int qSize, std::string filterType) {
//:queue_size(qSize), filter_type(filterType) {
    //do stuff if needed
    this->queue_size = qSize;
    this->filter_type = filterType;

    //initialize signal values to 0
    this->prev_call_signal = 0;
    this->curr_call_signal = 0;
    
    this->prev_repel_signal = 0;
    this->curr_repel_signal = 0;

    this->delta_call_signal = 0;
    this->delta_repel_signal = 0;

    // std::cout<<"queue_size: "<<queue_size<<std::endl
    //             <<"filter type "<<filter_type<<std::endl;   
}

double CommModels::get_value(std::string desired_value) {
    //use if statements to select the right parameter to return
    if(desired_value.compare("prev_call_signal") == 0) {
        return this->prev_call_signal;
    }
    else if (desired_value.compare("curr_call_signal") == 0) {
        return this->curr_call_signal;
    }
    else if (desired_value.compare("prev_repel_signal") == 0) {
        return this->prev_repel_signal;
    }
    else if (desired_value.compare("curr_repel_signal") == 0) {
        return this->curr_repel_signal;
    }
    else if (desired_value.compare("queue_size") == 0) {
        return this->queue_size;
    }
    else if (desired_value.compare("delta_call_signal") == 0) {
        return this->delta_call_signal;
    }
    else if (desired_value.compare("delta_repel_signal") == 0) {
        return this->delta_repel_signal;
    }
    else {
        return 9999.9999;// used to represent unknown value requested
    }

}

void CommModels::update_comm_signals(double call_comm, double repel_comm, double t) {
    /*
    Adds elements at the end of the queue. This means that the last element added 
    is the most recent element (i.e. it.end() is most current) and 
    first element (i.e. it.begin()) is the oldest communicated signal
    */

    this->call_queue.push_back(call_comm);
    this->repel_queue.push_back(repel_comm);
    this->time_stamp.push_back(t);

    if( (int) this->time_stamp.size() >= this->queue_size ) {
        //compute signal intensitites
        if ((this->filter_type).compare("average_filter") == 0){
            this->average_filter(&(this->call_queue), &(this->prev_call_signal), &(this->curr_call_signal));
            this->average_filter(&(this->repel_queue), &(this->prev_repel_signal), &(this->curr_repel_signal));
            this->time_stamp.clear();//clear time (done for average filter only)
        }
        else if ((this->filter_type).compare("sliding_window_filter") == 0) {
            this->sliding_window_filter(&(this->call_queue), &(this->prev_call_signal), &(this->curr_call_signal));
            this->sliding_window_filter(&(this->repel_queue), &(this->prev_repel_signal), &(this->curr_repel_signal));
            this->time_stamp.pop_front();
        }
        //compute gradients
        this->delta_call_signal = this->curr_call_signal - this->prev_call_signal;
        this->delta_repel_signal = this->curr_repel_signal - this->prev_repel_signal;


    }
}

//functions to handle different communication filters
void CommModels::average_filter(std::deque<double> *signal, double *prev, double *curr){
    /*
    uses non-overlapping chunks of instantaneous values to compute
    the signal value by using an average of a chunk to represent
    the magnitude of the signal. A chunk is equivalent to the 
    queue_size.
    */

    //update prev signal
    *prev = *curr;

    //compute average
    double total_magnitude = std::accumulate(signal->begin(), signal->end(), 0);//computes sum
    
    //update curr signal
    *curr = total_magnitude / std::distance(signal->begin(), signal->end());
    
    //clear contents
    signal->clear();
    


}

void CommModels::sliding_window_filter(std::deque<double> *signal, double *prev, double *curr){
    std::deque<double>::iterator middle = std::next(signal->begin(),signal->size() / 2);

    *prev = std::accumulate(signal->begin(), middle - 1, 0) / std::distance(signal->begin(), middle - 1);
    *curr = std::accumulate(middle, signal->end(), 0) / std::distance(middle, signal->end());
    
    signal->pop_front();//remove first measurement in list/queue
}