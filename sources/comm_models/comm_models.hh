#include <iostream>
#include <string>
#include <deque>


class CommModels {
    private:
        double prev_call_signal;
        double curr_call_signal;
        std::deque<double> call_queue; //twice size of queue_size to hold values to compute prev & curr communicated value
        
        double prev_repel_signal;
        double curr_repel_signal;
        std::deque<double> repel_queue;

        double queue_size; //size of array to compute single value (e.g. prev_call_signal)

        std::deque<double> time_stamp; //time stamp on the communicated signals

        // measure of change in intensity between prev and curr communication signals
        double delta_call_signal;
        double delta_repel_signal;

        /*
        Declare methods to handing different filtering approaches as private
        they should be used to update desired values based on user's choice
        */
        std::string filter_type;
        void average_filter(std::deque<double> *signal);

    public:
        CommModels(int qSize, std::string filterType);

        double get_value(std::string desired_value);//used to access signals and gradients
        
        //accept communicated information and time stamp
        void update_comm_signals(double call_comm, double repel_comm, double t);


};